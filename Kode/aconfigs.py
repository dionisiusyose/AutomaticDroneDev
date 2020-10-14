"""
CONFIGURATION
"""

import numpy as np

# CONTROL
MINIPIX_BAUDRATE = 115200
MINIPIX_PORT = "/dev/ttyACM0"
VEHICLE_WAIT_READY = False
VEHICLE_SWITCH_MODE_TIMEOUT = 4

DEFAULT_TAKEOFF = 0.58
SMOOTH_TAKEOFF = 0.53

MAX_1 = 20000  # 20000
MAX_2 = 2000  # 6000
MAX_3 = 5000  # 16500
TRANS_MID = 20000
TRANS_LOW = 10000
TRANS_UP = 5000
RANGE_LAND = 15000


LONG = 2
MIDDLE = 0.8
SHORT = 0.5
LAND = 0.1

ALTITUDE_DEFAULT = 0.5

# PID
SET_X = 160
SET_Y = 120
KP_X = 0.0025
KI_X = 0.0000
KD_X = 0.0005
KP_Y = 0.0025
KI_Y = 0.0000
KD_Y = 0.0002

MAX_SPEED = 0.5

# Type Mask Mavlink
DEFAULT_MASK = 0b0000111111000111  # Velocity only
GAIN_HEIGHT = 0b0000111111011111  # For gaining height

# VISION
CAMERA_DEFAULT = 0
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240
DEFAULT_MAX_VALUE = 255
DEFAULT_MAX_VALUE_HUE = 179
DEFAULT_CAMERA = 0
BLUR_GAUSSIAN = (3, 3)
MORPH_KERNEL = np.ones(BLUR_GAUSSIAN, np.uint8)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# ARDUINO
ARDU_PORT = "/dev/ttyUSB0"
ARDU_BAUD = 9600

# Check
CHECK_CAMERA = True


def nothing(x):
    pass


def openFile(detectionType):
    if detectionType == 1:
        file = 'inner.txt'
    elif detectionType == 2:
        file = 'middle.txt'
    elif detectionType == 3:
        file = 'outer.txt'
    elif detectionType == 4:
        file = 'landrange.txt'
    elif detectionType == 5:
        file = 'transmid.txt'
    elif detectionType == 6:
        file = 'translow.txt'
    elif detectionType == 7:
        file = 'transup.txt'
    else:
        file = None

    if file is not None:
        with open(file, 'r') as read:
            rangeOfHSV = read.readline().split(',')
            lowerHue, lowerSat, lowerVal, upperHue, upperSat, upperVal = rangeOfHSV
            return int(lowerHue), int(lowerSat), int(lowerVal), int(upperHue), int(upperSat), int(upperVal)
    else:
        print("Open file error")
        exit()


def saveFile(detectionType, lowerHue, lowerSat, lowerVal, upperHue, upperSat, upperVal):
    if detectionType == 1:
        file = 'inner.txt'
    elif detectionType == 2:
        file = 'middle.txt'
    elif detectionType == 3:
        file = 'outer.txt'
    elif detectionType == 4:
        file = 'landrange.txt'
    elif detectionType == 5:
        file = 'transmid.txt'
    elif detectionType == 6:
        file = 'translow.txt'
    elif detectionType == 7:
        file = 'transup.txt'
    else:
        file = None

    if file is not None:
        with open(file, 'w') as write:
            write.write(f"{lowerHue},{lowerSat},{lowerVal},{upperHue},{upperSat},{upperVal}")
    else:
        print("File save error")
        exit()

