from aconfigs import *
import sensor
import cv2 as cv
from datetime import datetime
import time

now = datetime.now()
dt = now.strftime("%d_%m_%Y %H-%M-%S")

cam = sensor.Camera(0)
srf = sensor.SRFNoThread(ARDU_PORT)

imgCounterQ1 = 0
imgCounterQ2 = 0
imgCounterQ3 = 0
imgTransition = 0
counter = 0

landWriter = False
shortDist1 = False

# out = cv.VideoWriter(f"{dt}.avi", cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (320, 240))

while True:
    ret, frame = cam.read()

    if ret:
        if LAND - 0.02 < srf.distance() < LAND:
            if not landWriter:
                img_name = f"land.png"
                cv.imwrite(img_name, frame)
                print(f"{img_name} is written")
                landWriter = True
            else:
                pass
        elif LAND + 0.1 < srf.distance() < LAND + 0.2:
            # cv.putText(frame, f"Alt{srf.distance()}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            if imgTransition < 2:
                img_name = f"T{imgTransition}.png"
                cv.imwrite(img_name, frame)
                print(f"{img_name} is written")
                imgTransition += 1
            else:
                pass
        elif SHORT - 0.1 < srf.distance() <= SHORT:
            # cv.putText(frame, f"Alt{srf.distance()}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            if imgCounterQ1 < 2:
                img_name = f"1{imgCounterQ1}.png"
                cv.imwrite(img_name, frame)
                print(f"{img_name} is written")
                imgCounterQ1 += 1
            else:
                pass
        elif MIDDLE - 0.1 < srf.distance() <= MIDDLE:
            # cv.putText(frame, f"Alt{srf.distance()}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            if imgCounterQ2 < 3:
                img_name = f"2{imgCounterQ2}.png"
                cv.imwrite(img_name, frame)
                print(f"{img_name} is written")
                imgCounterQ2 += 1
            else:
                pass
        elif 1.2 < srf.distance() <= LONG:
            # cv.putText(frame, f"Alt{srf.distance()}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            if imgCounterQ3 < 3:
                img_name = f"3{imgCounterQ3}.png"
                cv.imwrite(img_name, frame)
                print(f"{img_name} is written")
                imgCounterQ3 += 1
            else:
                pass

        cv.putText(frame, f"Alt{srf.distance()}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        cv.imshow("Image", frame)

        if landWriter:
            if imgTransition > 1 and imgCounterQ1 > 1 and imgCounterQ2 > 2 and imgCounterQ3 > 2:
                break

        k = cv.waitKey(1)

        if k == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k == ord('q'):
            img_name = f"plus{counter}.png"
            cv.imwrite(img_name, frame)
            print(f"{img_name} is written")
            counter += 1

        cv.putText(frame, f"Alt{srf.distance()}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        cv.imshow("Image", frame)

cam.release()
# out.release()
cv.destroyAllWindows()

while True:
    pic1 = cv.imread("10.png")
    cv.imshow("Pic1", pic1)
    pic2 = cv.imread("20.png")
    cv.imshow("Pic2", pic2)
    pic3 = cv.imread("30.png")
    cv.imshow("Pic3", pic3)
    picLand = cv.imread("land.png")
    cv.imshow("PicLand", picLand)
    transPic = cv.imread("T0.png")
    cv.imshow("Trans", transPic)

    try:
        picPlus = cv.imread("plus1.png")
        cv.imshow("Plus", picPlus)
    except:
        pass

    k = cv.waitKey(0)
    if k == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

cv.destroyAllWindows()
