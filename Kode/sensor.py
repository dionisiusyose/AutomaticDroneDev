from cv2 import VideoCapture, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT
import cv2 as cv
from queue import Queue
from threading import Thread
from time import sleep
from serial import Serial
from aconfigs import *
from datetime import datetime


class Camera:
    def __init__(self, camNumber):
        self.camNumber = camNumber

        # Connect to Camera
        self.cap = VideoCapture(camNumber)
        self.cap.set(CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH)
        self.cap.set(CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT)

        # Camera Threading
        self.qCamera = Queue()
        self.tCamera = Thread(target=self._reader)
        self.tCamera.daemon = True
        self.tCamera.start()

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            # frame = cv.rotate(frame, cv.ROTATE_180)
            if not ret:
                break
            if not self.qCamera.empty():
                try:
                    self.qCamera.get_nowait()
                except:
                    pass
            self.qCamera.put((ret, frame))

    def read(self):
        return self.qCamera.get()

    def release(self):
        self.cap.release()
        cv.destroyAllWindows()


class SRFThread:
    def __init__(self, port):
        # Serial connection to Arduino
        self.port = port
        print("Connecting to arduino...")
        self.arduinoSerial = Serial(port=self.port, baudrate=ARDU_BAUD, timeout=1)
        sleep(3)
        print("Arduino connected")
        self.prevDistance = 0.01

        # SRF Threading
        self.qSRF = Queue()
        tSRF = Thread(target=self._distance)
        tSRF.daemon = True
        tSRF.start()

    def _distance(self):
        while True:
            self.arduinoSerial.flush()
            self.arduinoSerial.write(b'a')
            srf_distance = self.arduinoSerial.readline().decode('ascii').strip()

            try:
                distance = int(srf_distance) / 100.0
                if not self.qSRF.empty():
                    try:
                        self.qSRF.get_nowait()
                    except:
                        pass
                self.qSRF.put(distance)
            except:
                pass

    def readSRF(self):
        distance = self.qSRF.get()
        if distance is None or distance is 0:
            return self.prevDistance
        else:
            self.prevDistance = distance
            return distance


class SRFNoThread:
    def __init__(self, port):
        # Serial connection to Arduino
        self.port = port
        print("Connecting to arduino...")
        self.arduinoSerial = Serial(port=self.port, baudrate=ARDU_BAUD, timeout=1)
        sleep(3)
        print("Arduino connected")

    def distance(self):
        while True:
            self.arduinoSerial.flush()
            self.arduinoSerial.write(b'a')
            srf_distance = self.arduinoSerial.readline().decode('ascii').strip()

            try:
                distance = int(srf_distance) / 100.0
                return distance
            except:
                continue


def objectDetection(frame, option):
    # frame = cv.bilateralFilter(frame, 10, 40, 40)
    frame = cv.GaussianBlur(frame, BLUR_GAUSSIAN, sigmaX=0)
    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lowerHue, lowerSat, lowerVal, upperHue, upperSat, upperVal = openFile(option)

    # HSV value of landing pad
    lowerRange = np.array([lowerHue, lowerSat, lowerVal])
    upperRange = np.array([upperHue, upperSat, upperVal])
    maskHSV = cv.inRange(frameHSV, lowerRange, upperRange)
    dilationMask = cv.erode(maskHSV, MORPH_KERNEL, iterations=1)

    # Find contour
    contours, _ = cv.findContours(dilationMask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Declaration of center
    cogX = int(0)
    cogY = int(0)
    area = int(1)
    landingFound = False

    # Find contour using center of gravity
    for contour in contours:
        x, y, w, h = cv.boundingRect(contour)

        areaLuas = w * h
        if option == 1:
            minim = MAX_1
        elif option == 2:
            minim = MAX_2
        elif option == 3:
            minim = MAX_3
        elif option == 4:
            minim = 2000
        elif option == 5:
            minim = TRANS_MID
        else:
            minim = 500

        if w * h > minim:
            landingFound = True

            x1 = int(((x + x + w) / 2))
            y1 = int(((y + y + h) / 2))
            area += int(w * h)

            cogX += int(x1 * w * h)
            cogY += int(y1 * w * h)
            cv.rectangle(frame, (x, y), (x + w, y + h), RED, thickness=3)

    if landingFound:
        cogXFinal = int(cogX / area)
        cogYFinal = int(cogY / area)
        return cogXFinal, cogYFinal, frame
    else:
        cogXFinal = -1
        cogYFinal = -1
        return cogXFinal, cogYFinal, frame


def calculateFPS(fps, dist, action, frame, centerPos):
    cv.putText(frame, f"{centerPos}", centerPos, cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0))
    cv.putText(frame, f"FPS: {fps}", (1, 10), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0))
    cv.putText(frame, f"Altitude: {dist}, Action: {action}", (1, 20), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0))
    cv.imshow("Result", frame)


def showFrame(frame):
    cv.imshow("Result", frame)


def checkQuit():
    key = cv.waitKey(60) & 0xFF
    if key == 27:
        return True


def main():
    dist = SRFThread(ARDU_PORT)

    while True:
        print(dist.readSRF())
        # print(dist.distance())

# if __name__ == '__main__':
#    main()
