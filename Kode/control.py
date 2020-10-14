from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
from threading import Thread
import math
from aconfigs import *
import sensor
from math import radians, cos, sin


# A wrapper for threading function
def threaded(fn):
    """
    A threading wrapper
    Function : to run a function in another thread

    example usage:
    in function declaration
    -----------------------
    @threaded
    def printCount():
        for i in range(10):
            print(i)

    In call declaration
    -----------------------
    handle = printCount()
    -- Do another stuffs --
    handle.join()
    """

    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.setDaemon(True)
        thread.start()
        return thread

    return wrapper



class Drone:
    def __init__(self):
        """
        Drone initialize
        """
        print('Vehicle : Connecting to vehicle on: %s' % MINIPIX_PORT)
        self.vehicle = connect(MINIPIX_PORT, baud=MINIPIX_BAUDRATE, wait_ready=VEHICLE_WAIT_READY)

        # Time
        self.time_now = None
        self.time_past = None
        self.time_out = None

    def sendMavlink(self, vx, vy, vz):
        self.set_velocity_body(vx, vy, vz)

    def set_velocity_body(self, vx, vy, vz):
        """ Remember: vz is positive downward!!!
        http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

        Bitmask to indicate which dimensions should be ignored by the vehicle
        (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
        none of the setpoint dimensions should be ignored). Mapping:
        bit 1: x,  bit 2: y,  bit 3: z,
        bit 4: vx, bit 5: vy, bit 6: vz,
        bit 7: ax, bit 8: ay, bit 9:
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            DEFAULT_MASK,  # -- BITMASK -> Consider only the velocities
            0, 0, 0,  # -- POSITION
            vx, vy, vz,  # -- VELOCITY
            0, 0, 0,  # -- ACCELERATIONS
            0, 0)
        self.vehicle.send_mavlink(msg)

    def setMode(self, mode):
        """
        :param mode: SET VEHICLE MODE
        :return:
        """
        self.vehicle.mode = VehicleMode(mode)

        timeout = time.time() + VEHICLE_SWITCH_MODE_TIMEOUT
        while self.vehicle.mode.name is not mode:
            if time.time() >= timeout:
                return False
        return True

    def checkMode(self, compare=None):
        """
        :return: current vehicle mode
        """
        if compare is None:
            return self.vehicle.mode.name
        else:
            return self.vehicle.mode == VehicleMode(compare)

    def disArm(self):
        self.vehicle.mode = VehicleMode("LAND")
        self.vehicle.armed = False
        self.vehicle.close()


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0, speedCutoff=MAX_SPEED, current_time=None):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.setPoint = 0
        self.speedCutoff = speedCutoff

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def clearPID(self):
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Integral protector
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, value, current_time=None):
        error = self.setPoint - value
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if self.ITerm < - self.windup_guard:
                self.ITerm = - self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

            if self.speedCutoff is not None:
                if abs(self.output) > self.speedCutoff:
                    self.output = (self.output / abs(self.output)) / self.speedCutoff
                    return self.output
                else:
                    return self.output

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """
        Maximum Integrator
        :param windup:
        :return:
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

