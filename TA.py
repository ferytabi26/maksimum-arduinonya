from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import serial
import time

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
args = vars(ap.parse_args())

orangeLower = (0, 196, 0)
orangeUpper = (69, 255, 255)
counter = 0
counterA = 0
counterB = 0
counterC = 0
data = 0

if not args.get("video", False):
    camera = cv2.VideoCapture(0)
else:
    camera = cv2.VideoCapture(args["video"])
try:
    res = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    res.flush()

    while True:
        (grabbed, frame) = camera.read()

        if args.get("video") and not grabbed:
            break

        frame = imutils.resize(frame, width=400)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, orangeLower, orangeUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) == 0:
            counter = counter + 1
            if counter == 1:
                if data == 1:
                    res.write("x".encode('utf-8'))    
                if data == 2:
                    res.write("y".encode('utf-8'))
                if data == 3:
                    res.write("z".encode('utf-8'))
            if counter > 10 :
                counter = 0
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.putText(frame, ('x='+str(int(x))), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,100), 2, cv2.LINE_AA)
                cv2.putText(frame, ('y='+str(int(y))), (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,100), 2, cv2.LINE_AA)
                cv2.putText(frame, ('z='+str(int(radius))), (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,100), 2, cv2.LINE_AA)
                counter = 0
                
            if x>0 and x<40:
                res.write("a".encode('utf-8'))
            if x>41 and x<80:
                res.write("b".encode('utf-8'))
            if x>81 and x<120:
                res.write("c".encode('utf-8'))
            if x>121 and x<160:
                res.write("d".encode('utf-8'))
            if x>161 and x<200:
                res.write("e".encode('utf-8'))
            if x>201 and x<240:
                res.write("f".encode('utf-8'))
            if x>241 and x<280:
                res.write("g".encode('utf-8'))
            if x>281 and x<320:
                res.write("h".encode('utf-8'))
            if x>321 and x<360:
                res.write("i".encode('utf-8'))
            if x>361 and x<400:
                res.write("j".encode('utf-8'))
            if y>0 and y<20:
                res.write("k".encode('utf-8'))
            if y>21 and y<40:
                res.write("l".encode('utf-8'))
            if y>41 and y<60:
                res.write("m".encode('utf-8'))
            if y>61 and y<80:
                res.write("n".encode('utf-8'))
            if y>81 and y<100:
                res.write("o".encode('utf-8'))
            if y>101 and y<120:
                res.write("p".encode('utf-8'))
            if y>121 and y<140:
                res.write("q".encode('utf-8'))
            if y>141 and y<160:
                res.write("r".encode('utf-8'))
            if y>161 and y<180:
                res.write("s".encode('utf-8'))
            if y>181 and y<200:
                res.write("t".encode('utf-8'))
            if y>201 and y<225:
                res.write("u".encode('utf-8'))        
            if radius>15 and radius<100:
                res.write("z".encode('utf-8'))
            if radius>100:
                res.write("x".encode('utf-8'))
                
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()
except KeyboardInterrupt:
    res.close()




