import cv2
"""
Color detection in Python(using opencv)
"""
# Import useful libraries
import numpy as np
import cv2
import math
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from gpiozero import CamJamKitRobot, DistanceSensor
from coloer_detect import DetectBlue

#from Obstacle_Avoidance

print("Hey there")

camera = PiCamera ()
camera.resolution = (160, 120)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size =(160,120))
time.sleep(0.1)

robot=CamJamKitRobot()
hownear=15.0
reversetime=0.5
turntime=0.75

leftmotorspeed=0.5
rightmotorspeed=0.5

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorleft=(leftmotorspeed,0)
motorright=(0,rightmotorspeed)


####our color detect function
import numpy as np
import cv2
import math
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

camera = PiCamera ()
camera.resolution = (320, 240)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size =(320,240))
time.sleep(0.1)

def DetectBlue (): 
    camera = PiCamera ()
    camera.resolution = (160, 120)
    camera.framerate = 16
    rawCapture = PiRGBArray(camera, size =(160,120))
    time.sleep(0.1)


    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame.array
        blurred = cv2.medianBlur(frame, 5)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lower_red = np.array([100,100,100])
        upper_red = np.array([130,255,255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(frame,frame, mask = mask)

        contours = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
        for c in contours:
            cv2.drawContours(frame, [c], -1, (0,255,0),3)

            #display resulting frame
            cv2.imshow('Frame',frame)

            # find the biggest area and put a green square around it
            c_m = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c_m)
            cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)

            #report the centroid of the biggest contour
            M = cv2.moments(c_m)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print(cX)
            #print("cx=", cX, "cy=", cY)
            #this bit is neccesary for when object is detected
            else:
                cX, cY = 0, 0
                #print("No centroid found")
                

            M = 0

            
        

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        
        cv2.imshow("Frame", frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)

      
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("q"):
            break
    
    cv2.destroyAllWindows()

    camera.destroy()
###########################################################



while True:
    print("hi")
    DetectBlue()
    print(cX)
    if cX !=150:
        robot.value=motorforward
    else:
        robot.value=motorbackward
#try:
    #if isnearobstacle(hownear):
        #robot.stop()
       # avoidobstacle()
        #except 

#cv2.destroyAllWindows()

#camera.destroy()
