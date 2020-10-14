from aconfigs import *
import sensor
import dataRecord
import cv2 as cv


def setup():
    camera = sensor.Camera(0)
    record = dataRecord.Record()
    srf = sensor.SRFNoThread(ARDU_PORT)
    return camera, record, srf


def main():
    camera, record, srf = setup()

    while True:
        ret, frame = camera.read()

        if ret:
            distance = srf.distance()

            record.recordVideo(frame)
            cv.putText(frame, f"{distance}", (1, 30), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255))
            cv.imshow("Frame", frame)

            key = cv.waitKey(60) & 0xFF
            if key == 27:
                break

    camera.release()
    record.stopRecord()


if __name__ == "__main__":
    main()


