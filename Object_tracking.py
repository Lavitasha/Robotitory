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
from gpiozero import CamJamKitRobot


#  Motor settings #
robot=CamJamKitRobot()
hownear=15.0
reversetime=0.5
turntime=0.75

<<<<<<< HEAD
leftmotorspeed=0.30
rightmotorspeed=0.288

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorright=(leftmotorspeed,0)
motorleft=(0,rightmotorspeed)
=======
leftmotorspeed=0.35
rightmotorspeed=0.35

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorleft=(leftmotorspeed,0)
motorright=(0,rightmotorspeed)
>>>>>>> 0a651a12db615942c58878a061cfb4efcef30713

#  Camera settings #
camera = PiCamera ()
camera.resolution = (160, 120)
camera.framerate = 15
rawCapture = PiRGBArray(camera, size =(160,120))
<<<<<<< HEAD
camera.rotation=180 
=======
>>>>>>> 0a651a12db615942c58878a061cfb4efcef30713

#  Color to detect settings
lower_red = np.array([100,100,100])
upper_red = np.array([130,255,255])

<<<<<<< HEAD
def TurnRight():
    robot.value=motorright
    time.sleep(0.25)
    robot.stop()
    print("right")

def TurnLeft():
    robot.value=motorleft
    time.sleep(0.25)
    robot.stop()
    print("left")

def Charge():
    robot.value=motorforward
    print("Attack!!!")

=======
>>>>>>> 0a651a12db615942c58878a061cfb4efcef30713

"""  Action Time  """


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = frame.array

    # Apply different mask the threshold according to color settings
    blurred = cv2.medianBlur(frame, 5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame,frame, mask = mask)

    # Draw contours and report centroid for biggest one
    contours = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    for c in contours:
        cv2.drawContours(frame, [c], -1, (0,255,0),3)

        # Display resulting frame
        cv2.imshow('Frame',frame)

        # Find the biggest area and put a green square around it
        c_m = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c_m)
        cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Report the centroid of the biggest contour, if none set to zero
        M = cv2.moments(c_m)
        if M["m00"] != 0:
            print("Blue Ballon detected")
            cX = int(M["m10"] / M["m00"])
            #cY = int(M["m01"] / M["m00"])
            # we could potentially use this parameter to exclude blue detected high up
<<<<<<< HEAD
            while 45 < cX < 100:
                Charge()                
                break
                
            if cX <45:
                TurnLeft()
                               

            if cX >100:
                TurnRight()
                
                

        else:
            robot.stop()
            print("No Blue")
=======
            while 60 < cX < 80:
                robot.value=motorforward
                print("Attack!!!")
                break
                
            while cX <60:
                robot.value=motorleft
                time.sleep(2)
                robot.value=motorforward
                time.sleep(3)
                print("Turn left")
                break

            while cX >80:
                robot.value=motorright
                time.sleep(2)
                robot.value=motorforward
                time.sleep (3)
                
                print("Right")
                break
                

        else:
            robot.value=motorright
            time.sleep(3)
            robot.value=motorforward
            time.sleep (4)
            robot.value=motorleft
            time.sleep(3)
            robot.value=motorforward
            time.sleep(4)
            print("Must Explore!")
>>>>>>> 0a651a12db615942c58878a061cfb4efcef30713
            
            
        #M = 0          
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            robot.stop()

    cv2.imshow("Frame", frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

      
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord("q"):
        break
        robot.stop()

robot.stop()
    
cv2.destroyAllWindows()
