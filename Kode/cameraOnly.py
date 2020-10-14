import time
import sys
from control import PID
from sensor2 import Camera, SRFThread, SRFNoThread, objectDetection, calculateFPS, checkQuit
from aconfigs import *
import argparse
from timeit import default_timer as timer
import dataRecord
import cv2 as cv


def parse():
    # Parse the arguments
    parser = argparse.ArgumentParser(prog='Dynamic Landing',
                                     description='Tugas akhir Dionisius')
    parser.add_argument('--cam', default=0, help='0 - PiCamera, 1 - Logitech')

    args = parser.parse_args()

    # Get variables from parsing
    camNum = int(args.cam)

    return camNum


def setup():
    """
    Setting the drone
    :return:
    """
    camNumber = parse()

    altitude = SRFThread(ARDU_PORT)
    camera = Camera(camNumber=camNumber)

    return altitude, camera


def landingControl(camera, altitude):
    pidX = PID(P=KP_X, I=KI_X, D=KD_X)
    pidX.setPoint = SET_X
    pidY = PID(P=KP_Y, I=KI_Y, D=KD_Y)
    pidY.setPoint = SET_Y

    while True:
        ret, frame = camera.read()
        start = timer()

        if ret:
            curDist = altitude.readSRF()

            if curDist <= SHORT:
                if curDist <= LAND:
                    option = 4
                    xCenter, yCenter, frame, masking = objectDetection(frame, option)
                else:
                    option = 6
                    xCenter, yCenter, frame, masking = objectDetection(frame, option)
                    if xCenter < 0 and yCenter < 0:
                        option = 1
                        xCenter, yCenter, frame, masking = objectDetection(frame, option)
            elif SHORT < curDist <= MIDDLE:
                option = 2
                xCenter, yCenter, frame, masking = objectDetection(frame, option)
                if xCenter < 0 and yCenter < 0:
                    option = 5
                    xCenter, yCenter, frame, masking = objectDetection(frame, option)
            elif MIDDLE < curDist <= LONG:
                option = 3
                xCenter, yCenter, frame, masking = objectDetection(frame, option)
                if xCenter < 0 and yCenter < 0:
                    option = 7
                    xCenter, yCenter, frame, masking = objectDetection(frame, option)
            else:
                print(f"Drone max altitude offset reached : {curDist} m")
                break

            if xCenter < 0 and yCenter < 0:
                vx, vy, vz = 0, 0, - 0.02
            else:
                pidX.update(xCenter + 1)
                pidY.update(yCenter + 1)
                vx = pidY.output
                vy = pidX.output * (-1)
                vz = 0.2

            fps = f"{str(round(1 / (timer() - start), 2))}"
            error = f"{vx, vy, vz}"

            calculateFPS(fps, curDist, option, frame, (xCenter, yCenter), error)
            cv.imshow("MASKING", masking)

            if checkQuit():
                print("Drone now landing...")
                break

    camera.release()


def main():
    """
    Main program
    :return:
    """
    try:
        # Setup vehicle
        altitude, camera = setup()
        groundPoint = altitude.readSRF()
        print(f"Drone will land at {groundPoint}")
        # Start vehicle

        # checkCondition(drone)

        landingControl(camera, altitude)

        print("Program ending")
        sys.exit()
    except:
        sys.exc_info()


if __name__ == "__main__":
    main()

