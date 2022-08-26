import cv2
import numpy as np
import imutils
from imutils.video import WebcamVideoStream
from _thread import *
import time
from pyzbar.pyzbar import decode

QR_CODE = [
    "TOWER_ROBOT_001"
]

if __name__ == "__main__":

    global cx,w,h,flag,minArea,maxArea,lock,fw,threadStop

    threadStop = False #To terminate the thread which drives the robot whenever the user quits

    lock = False #To lock the object

    flag = 0 #Its 1 whenever the object is detected

    #Range of the colors to detect an Object in this range
    lower_thresh = np.array([255, 103, 0])
    high_thresh = np.array([250, 214, 165])

    #To stream the video from the webcam
    device = WebcamVideoStream(src=0).start()

    first = True

    while True:
        frame = device.read()
        while True:
            success, camera = device.read()
            if not success:
                break
        for barcode in decode(camera):
            myData = barcode.data.decode("utf-8")

            if myData in myDataList:
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)

            if myData == "TOWER_ROBOT_001":
                process()
                time.sleep(1)
            else:
                print("Tower Robot Not Found")
                break

        def process():
            #Pre-processing of the frame to detect the object
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower_thresh, high_thresh)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            fh,fw,_ = frame.shape
            #cv2.rectangle(frame,(int(fw/5),0),(int(4*fw/5),fh),(0,0,255),3)
            cv2.rectangle(frame,(int(fw/4),0),(int(3*fw/4),fh),(0,255,255),3)

            #It draws rectangle around the object and finds its centre co-ordinates
            if len(cnts)>0:
                flag = 1
                c = max(cnts, key=cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #cv2.circle(frame,(cx,cy),3,(0,255,255),-1)

                if first:
                    #print("Area = ",w*h)
                    #exit(0)
                    maxArea = 3*w*h/2
                    minArea = w*h/2
                else:
                    flag = 0

        cv2.imshow("Frame",frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            threadStop = True
            break
elif k == ord('l') and flag==1:
    print("Locked")
    print("Fw = ",fw)
    print("Fh = ",fh)
    first = False
    lock = True

#device.release()
device.stop()
cv2.destroyAllWindows()
