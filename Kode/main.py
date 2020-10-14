import time
import sys
from control import Drone, PID
from sensor import Camera, objectDetection, calculateFPS, checkQuit, SRFThread, showFrame
from aconfigs import *
import argparse
from timeit import default_timer as timer
import dataRecord
import os


def parse():
    # Parse the arguments
    parser = argparse.ArgumentParser(prog='Dynamic Landing',
                                     description='Tugas akhir Dionisius')
    parser.add_argument('--cam', default=0, help='0 - PiCamera, 1 - Logitech')

    args = parser.parse_args()

    # Get variables from parsing
    camNum = int(args.cam)

    return camNum


def case():
    pass


def setup():
    """
    Setting the drone
    :return:
    """
    camNumber = parse()

    drone = Drone()
    camera = Camera(camNumber=camNumber)
    altitude = SRFThread(ARDU_PORT)

    return drone, camera, altitude


def landingControl(camera, drone, altitude):
    rec = dataRecord.Record()
    option = 0
    groundPoint = altitude.readSRF()

    xCenter = -1
    yCenter = -1

    pidX = PID(P=KP_X, I=KI_X, D=KD_X)
    pidX.setPoint = SET_X
    pidY = PID(P=KP_Y, I=KI_Y, D=KD_Y)
    pidY.setPoint = SET_Y

    xList = []
    yList = []
    optList = []
    altitudeList = []
    pitchList = []
    yawList = []
    rollList = []

    drone.setMode("STABILIZE")
    prevMode = "STABILIZE"

    while True:
        ret, frame = camera.read()
        start = timer()

        if drone.checkMode() is not prevMode:
            prevMode = drone.checkMode()
            print(drone.checkMode())

        if drone.checkMode() is "GUIDED" or drone.checkMode() is "GUIDED_NOGPS" or drone.checkMode() is "LAND":
            curDist = altitude.readSRF()
            if curDist <= SHORT:
                if curDist <= LAND:
                    option = 4
                    xCenter, yCenter, frame = objectDetection(frame, option)
                    drone.setMode("LAND")
                    print(f"Drone landing : {curDist}")
                    if curDist <= groundPoint + 0.01:
                        break
                elif LAND < curDist <= SHORT:
                    option = 6
                    xCenter, yCenter, frame = objectDetection(frame, option)
                    if xCenter < 0 and yCenter < 0:
                        option = 1
                        xCenter, yCenter, frame = objectDetection(frame, option)
            elif SHORT < curDist <= MIDDLE:
                option = 2
                xCenter, yCenter, frame = objectDetection(frame, option)
                if xCenter < 0 and yCenter < 0:
                    option = 5
                    xCenter, yCenter, frame = objectDetection(frame, option)
            elif MIDDLE < curDist <= LONG:
                option = 3
                xCenter, yCenter, frame = objectDetection(frame, option)
                if xCenter < 0 and yCenter < 0:
                    option = 7
                    xCenter, yCenter, frame = objectDetection(frame, option)
            else:
                drone.setMode("LAND")
                print(f"Drone max altitude offset reached : {curDist} m")
                break

            # CALCULATE PID
            xCenter = xCenter + 1
            yCenter = yCenter + 1

            if xCenter < 0 and yCenter < 0:
                vx, vy, vz = 0, 0, -0.2
                # if move:
                #    drone.setMode("LAND")
                #    print(f"Drone landing : {curDist}")
                #    break
            else:
                pidX.update(xCenter)
                pidY.update(yCenter)
                vx = pidY.output
                vy = pidX.output * (-1)
                #if 120 < xCenter < 200 and 100 < yCenter < 140:
                #    vz = 0.02  # 0.2
                #else:
                #    vz = 0
                vz = 0.2  # 0.2

                drone.sendMavlink(vx, vy, vz)
                
                if vx > 0:
                    pitch = "forward"
                elif vx < 0:
                    pitch = "backward"
                else:
                    pitch = "pas"

                if vy > 0:
                    roll = "right"
                elif vy < 0:
                    roll = "left"
                else:
                    roll = "pas"
		
                #print(vx, vy)
                fps = f"{str(round(1 / (timer() - start), 2))}"

                calculateFPS(fps, curDist, option, frame, (xCenter, yCenter))

                xList.append(xCenter)
                yList.append(yCenter)
                optList.append(option)
                altitudeList.append(curDist)

                pitchList.append(drone.vehicle._pitchspeed)
                yawList.append(drone.vehicle._yawspeed)
                rollList.append(drone.vehicle._rollspeed)
        else:
            showFrame(frame)
            pidY.clearPID()
            pidX.clearPID()

        if checkQuit():
            if drone.checkMode() is "GUIDED" or drone.checkMode() is "GUIDED_NOGPS":
                drone.setMode("LAND")
            else:
                pass
            print("Drone now landing...")
            break

    camera.release()
    drone.disArm()

    if drone.checkMode() is "GUIDED" or drone.checkMode() is "GUIDED_NOGPS" or drone.checkMode() is "LAND":
        drone.disArm()
        # rec.stopRecord()
        rec.recordData(xList, yList, altitudeList, optList, pitchList, yawList, rollList)
        csvSize = os.path.getsize(rec.csvName)
        if csvSize < 30:
            rec.deleteData(rec.csvName)
        else:
            print(f"{rec.csvName} is written !!!")


def main():
    """
    Main program
    :return:
    """
    try:
        # Setup vehicle
        startTimer = timer()
        drone, camera, altitude = setup()
        print(f"Drone will land at {altitude.readSRF()}")
        landingControl(camera, drone, altitude)

        print(f"Program ending at {timer() - startTimer}")
        sys.exit()
    except:
        sys.exc_info()


if __name__ == "__main__":
    main()

