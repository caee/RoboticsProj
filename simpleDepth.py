# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 14:35:47 2023

@author: carle
"""
import cv2
from matplotlib import pyplot as plt
import numpy as np
from robotConnect import *
from cameraCalib import calibrateCamera

def detect_round_object(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help with contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use HoughCircles to detect circles in the image
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=20,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=50
    )

    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # Draw the circles on the frame
        for (x, y, r) in circles:
            if (frame[y][x][2] > 80 and frame[y][x][1] < 80 and frame[y][x][1] < 80):
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

    return frame, circles
# Finds the depth of smarties in a plane, 
# from the known height of the camera used, the focal length and centroid positions

#Camera Calibration
direc = "C:\\Users\\carle\\Documents\\RoboticsProj\\calib"
ret,mtx,dist,rvecs,tvecs=calibrateCamera(direc)


#Open camera an get 2 images of smarties
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Error: Could not open camera.")

print("video capture is open")

#Connect Robot
portHandler, packetHandler = robotConnect("COM5")
# zeroPos = [150, 150, 150, 150]

#Defining a theoretical camera position horizontal to the table
#table1Pos = [150, 150, 150-90, 150]
#Offset for real value (measured)
table1Pos = [172, 155, 150-83, 150]
#Distance to lens
h_lens=150 #mm
h_mm=7 #Height of m&m
robotMove(portHandler, packetHandler, table1Pos)
_,_ = cap.read() #flush camera frame
ret, frame = cap.read()
cv2.imshow("Frame",frame)
cv2.waitKey(5000)
h,w = frame.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

cv2.imwrite('calibresult_simple.png', dst)
print("undistortion done")


round_objects,mms = detect_round_object(dst)
cv2.imshow("Detected!",round_objects)
cv2.waitKey(5000)


x=(w/2-mms[:,0])/newcameramtx[0,0]*h_lens
y=(h/2-mms[:,1])/newcameramtx[0,0]*h_lens
z=h_lens-h_mm


#OFFSET from image center coordinates to robot coords
#In world x
x_off=135 #mm
z_off=-45 ##




robotTerminate(portHandler, packetHandler)
cv2.destroyAllWindows()