from aconfigs import *
import cv2 as cv
import sys


def trackbarCalibration(calibrationChoice):  # Open Calibration File
    lowerHue, lowerSat, lowerVal, upperHue, upperSat, upperVal = openFile(calibrationChoice)

    cv.namedWindow("HSV Trackbars")
    cv.createTrackbar("L - H", "HSV Trackbars", int(lowerHue), DEFAULT_MAX_VALUE_HUE, nothing)
    cv.createTrackbar("L - S", "HSV Trackbars", int(lowerSat), DEFAULT_MAX_VALUE, nothing)
    cv.createTrackbar("L - V", "HSV Trackbars", int(lowerVal), DEFAULT_MAX_VALUE, nothing)
    cv.createTrackbar("U - H", "HSV Trackbars", int(upperHue), DEFAULT_MAX_VALUE_HUE, nothing)
    cv.createTrackbar("U - S", "HSV Trackbars", int(upperSat), DEFAULT_MAX_VALUE, nothing)
    cv.createTrackbar("U - V", "HSV Trackbars", int(upperVal), DEFAULT_MAX_VALUE, nothing)


def trackbarsValue():  # Get Position of Trackbar
    # Trackbars Position
    lowerHue = cv.getTrackbarPos("L - H", "HSV Trackbars")
    upperHue = cv.getTrackbarPos("U - H", "HSV Trackbars")
    lowerSat = cv.getTrackbarPos("L - S", "HSV Trackbars")
    upperSat = cv.getTrackbarPos("U - S", "HSV Trackbars")
    lowerVal = cv.getTrackbarPos("L - V", "HSV Trackbars")
    upperVal = cv.getTrackbarPos("U - V", "HSV Trackbars")
    return lowerHue, upperHue, lowerSat, upperSat, lowerVal, upperVal


def colorCalibration(calibrationChoice, originalFrame):  # Start Calibration
    #originalFrame = cv.bilateralFilter(originalFrame, 10, 50, 50)
    originalFrame = cv.GaussianBlur(originalFrame, BLUR_GAUSSIAN, 0)
    frameHSV = cv.cvtColor(originalFrame, cv.COLOR_BGR2HSV)
    while True:
        lowerHue, upperHue, lowerSat, upperSat, lowerVal, upperVal = trackbarsValue()  # Get trackbar value

        lowerRange = np.array([lowerHue, lowerSat, lowerVal])
        upperRange = np.array([upperHue, upperSat, upperVal])
        hsvMasking = cv.inRange(frameHSV, lowerRange, upperRange)
        dilateMask = cv.erode(hsvMasking, MORPH_KERNEL, iterations=2)
        #dilateMask = cv.dilate(hsvMasking, MORPH_KERNEL, iterations=1)
        resultFrame = cv.bitwise_and(originalFrame, originalFrame, mask=dilateMask)
        cv.imshow("Calibration", resultFrame)
        cv.imshow("dilate Mask", dilateMask)

        key = cv.waitKey(120) & 0XFF
        if key == 27:
            saveFile(calibrationChoice, lowerHue, lowerSat, lowerVal, upperHue, upperSat, upperVal)
            print(lowerRange, upperRange)
            break
    cv.destroyAllWindows()


def main():
    import argparse

    parser = argparse.ArgumentParser(prog='Calibratiion',
                                     description='Set Calibration')
    parser.add_argument('--file', default="10.png", help='Open file')
    parser.add_argument('--cal', default=1, help='File')

    args = parser.parse_args()

    # Get variables from parsing
    file = args.file
    calibrationChoice = int(args.cal)

    originalFrame = cv.imread(file)

    if 1 <= calibrationChoice <= 7:
        trackbarCalibration(calibrationChoice)
        colorCalibration(calibrationChoice, originalFrame)
    else:
        sys.exit()


if __name__ == "__main__":
    main()



