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


#  Motor settings #
robot=CamJamKitRobot()
hownear=15.0
reversetime=0.5
turntime=0.75

leftmotorspeed=0.30
rightmotorspeed=0.288

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorright=(leftmotorspeed,0)
motorleft=(0,rightmotorspeed)

#  Camera settings #
camera = PiCamera ()
camera.resolution = (160, 120)
camera.framerate = 15
rawCapture = PiRGBArray(camera, size =(160,120))
camera.rotation=180 

#  Color to detect settings
lower_red = np.array([100,100,100])
upper_red = np.array([130,255,255])

# Distance Variables
hownear=15.0
reversetime=0.5
turntime=0.75
pinTrigger = 17
pinEcho = 18
sensor = DistanceSensor(echo=pinEcho, trigger=pinTrigger)

#### Functions ####
def MoveGirl(direction, speed=1):
    #calculate relative motor speeds 
    # direction is a float from -1 to 1
    # speed is a overall modifier
    rightspeed = (1-direction)/2
    leftspeed = 1-rightspeed
    robot.value=(leftspeed * speed, rightspeed * speed)

def Stuck():
    robot.value=motorbackward
    time.sleep(1)
    robot.value=motorright
    time.sleep(1)
    robot.value=motorforward
    time.sleep(0.5)

def Avoidobstacle():
    print("Backwards")
    robot.value=motorbackward
    time.sleep(reversetime)
    robot.stop()
    print("Right")
    robot.value=motorright
    time.sleep(turntime)
    robot.stop()

def Nearobstacle(localhownear):
    distance=sensor.distance*100
    print("IsNearObstacle: " + str(distance))
    if distance < localhownear:
        print("Near obstacle!")
        return True
    else:
        print("Path Clear")
        return False 

## Initialize a variable for the explore function
last_seen_balloon = 0
time_turned = time.time()  
turning_left = True

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
        area = cv2.contourArea(c_m)
        M = cv2.moments(c_m)
        print("Area is",area)

        #if Nearobstacle(hownear) and area
        
        if (M["m00"] != 0) and (area > 100):
            print("Blue Ballon detected")
            cX = int(M["m10"] / M["m00"])
            # cY = int(M["m01"] / M["m00"])
            # we could potentially use this parameter to exclude blue detected high up
            # get the distance between the object and the center of the field of view
            offset = cX - 80
            # turn this into rotation speed
            turn_amount = offset / 160
            MoveGirl(turn_amount, speed = 1)
            last_seen_balloon = time.time()


            #while 45 < cX < 100 or area >= 5500:
            #    Charge()                
            #    break
                
            #if cX <45:
            #    TurnLeft()
                               

            #if cX >100:
            #    TurnRight()
                
               

        else:
            #Explore()
            # drive randomly
            if time.time() - time_turned > 2:
                turning_left = not turning_left
                time_turned = time.time()
                
            if turning_left:
                MoveGirl(-0.5, 0.5) ##actually turn left
                print("Exploring to the left")
            else:
                MoveGirl(0.5, 0.5)            
                print("Exploring to the right")

        ###When Cx and Cy haven't change in x seconds enable "wiggle"
            
            
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
