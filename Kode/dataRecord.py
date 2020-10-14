import cv2 as cv
import csv
from datetime import datetime
import os
from queue import Queue
from threading import Thread
import numpy as np


class Record:
    def __init__(self):
        now = datetime.now()
        self.dt = now.strftime("%d_%m_%Y %H-%M-%S")

        self.csvName = f"{self.dt}.csv"

    def recordData(self, xCenter, yCenter, zCenter, option, pitch, yaw, roll):
        with open(self.csvName, 'w', newline='') as write:
            theWriter = csv.writer(write)

            theWriter.writerow(['x', 'y', 'z', 'option', 'pitch', 'yaw', 'roll'])

            counter = len(xCenter)

            for i in range(counter):
                theWriter.writerow([xCenter[i], yCenter[i], zCenter[i], option[i], pitch[i], yaw[i], roll[i]])

    @staticmethod
    def deleteData(name):
        os.remove(name)

