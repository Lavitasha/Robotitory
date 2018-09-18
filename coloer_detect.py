import cv2
"""
Color detection in Python(using opencv)
"""
# Import useful libraries
import numpy as np
#import matplotlib.pyplot as plt
import cv2
import math
import time
import PiCamera as camera

# Specify camera/device number (usually 0)
camera = 0

#some kind of compression, 2 by 2 binning

# Specifiy number of frames to acquire
num_frames = 50

# Open camera stream
cap = cv2.VideoCapture(camera)

# find color array for red
#red = np.uint8([[[0, 0, 255]]])
#hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
#print(hsv_red)


while(1):

    # Take each frame
    ret, frame = cap.read()

    #could potentially need a step of blurring the image (GaussianBlur)
    blurred = cv2.medianBlur(frame, 5)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([100,100,100])
    upper_red = np.array([130,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask = mask)

    #do I need a threshold here to make the image binary?
    #ret,thresh = cv2.threshold(mask, 40, 255, 0)

    #find contours in video
    #consider adding a "round" criteria for optimization
    contours = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    for c in contours:
        cv2.drawContours(frame, [c], -1, (0,255,0),3)

        #display resulting frame
        cv2.imshow('frame',frame)

        # find the biggest area and put a green square around it
        c_m = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c_m)
        cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #report the centroid of the biggest contour
        M = cv2.moments(c_m)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            print("cx=", cX, "cy=", cY)
        #this bit is neccesary for when object is detected
        else:
            cX, cY = 0, 0
            print("No centroid found")

        M = 0

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # show the images
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

cam.release()
