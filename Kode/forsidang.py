import cv2 as cv
from aconfigs import *



cv.namedWindow("HSV Trackbars")
cv.createTrackbar("L - H", "HSV Trackbars", 0, DEFAULT_MAX_VALUE_HUE, nothing)
cv.createTrackbar("L - S", "HSV Trackbars", 0, DEFAULT_MAX_VALUE, nothing)
cv.createTrackbar("L - V", "HSV Trackbars", 0, DEFAULT_MAX_VALUE, nothing)
cv.createTrackbar("U - H", "HSV Trackbars", 179, DEFAULT_MAX_VALUE_HUE, nothing)
cv.createTrackbar("U - S", "HSV Trackbars", 255, 255, nothing)
cv.createTrackbar("U - V", "HSV Trackbars", 255, 255, nothing)


def trackbarsValue():  # Get Position of Trackbar
    # Trackbars Position
    lowerHue = cv.getTrackbarPos("L - H", "HSV Trackbars")
    upperHue = cv.getTrackbarPos("U - H", "HSV Trackbars")
    lowerSat = cv.getTrackbarPos("L - S", "HSV Trackbars")
    upperSat = cv.getTrackbarPos("U - S", "HSV Trackbars")
    lowerVal = cv.getTrackbarPos("L - V", "HSV Trackbars")
    upperVal = cv.getTrackbarPos("U - V", "HSV Trackbars")
    return lowerHue, upperHue, lowerSat, upperSat, lowerVal, upperVal


img = cv.imread("plus0.png")
# cv.imshow("Original", img)
#
# frame = cv.GaussianBlur(img, BLUR_GAUSSIAN, sigmaX=0)
# cv.imshow("Blur", frame)
#
frame = cv.cvtColor(img, cv.COLOR_BGR2HSV)
# cv.imshow("HSV", frame)

while 1:
    lowerHue, upperHue, lowerSat, upperSat, lowerVal, upperVal = trackbarsValue()  # Get trackbar value

    lowerRange = np.array([lowerHue, lowerSat, lowerVal])
    upperRange = np.array([upperHue, upperSat, upperVal])
    hsvMasking = cv.inRange(frame, lowerRange, upperRange)

    cv.imshow("Thresholding", hsvMasking)

    cv.waitKey(0)

cv.destroyAllWindows()
