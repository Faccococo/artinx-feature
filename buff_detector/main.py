import cv2
from detect import *

cap = cv2.VideoCapture("lbb_fan.mp4")
while True:
    ret, frame = cap.read()
    binary = run(frame)
    cv2.imshow("video", binary)
    key = cv2.waitKey(50)
    if key == ord('q'):  # 判断是哪一个键按下
        break
