# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 14:35:47 2023

@author: carle
"""
import cv2
from matplotlib import pyplot as plt
from InverseKinematics import inverse_kin
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
table1Pos = np.array([150, 150, 150-90, 150])
#Offset for real value (measured)
tableOffsets=np.array([22,5,7,0])
table1Pos = table1Pos+tableOffsets

#[172, 155, 150-83, 150]

#Distance to lens
h_lens=150 #mm
h_mm=7 #Height of m&m

#Offsets for center of image to world(robot base) coordinates
x_off=135 #mm
z_off=-45 ##


robotMove(portHandler, packetHandler, table1Pos)
_,_ = cap.read() #flush camera frame
while True:  
    count=1
    ret, frame = cap.read()
    if ret==True:
        cap.release()
        break
    elif count==10:
        print("no image")
        cap.release()
        break

cv2.imshow("Frame",frame)
cv2.waitKey(1000)
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
cv2.waitKey(1000)

#Getting the scaling factor
#From world measurement
x_chess=16#mm
x_px_chess=75#px
# fx_sx=h_lens*x_px_chess/x_chess

#very simple
#scaling factor found at specific height h_lens=150mm, 
#really only valid for center points cx,cy, but assumed to be constant
scaling_factor=x_chess/x_px_chess#mm/px

cx=newcameramtx[0,2]
cy=newcameramtx[1,2]

d_camera=(mms[:,:2]-[cx,cy])*scaling_factor
d=np.zeros((len(d_camera),3))
for i in range(0,len(d_camera)):
    #OFFSET from image center coordinates to robot coords

    d[i,0]=x_off-d_camera[i,1]
    d[i,1]=-d_camera[i,0]
    d[i,2]=z_off+h_mm+25

#### TEST###
q=inverse_kin(d[0,:].T,-np.pi/2,theta_3=2)

#Convert to degrees
q_deg=np.array([q[0],q[1],q[2],q[3]])*180/np.pi
#Magic offsets found from experiments
qOffsets=[170,60,150,240]
q_deg=q_deg+qOffsets


robotMove(portHandler, packetHandler, q_deg)


for i in range(0,len(d)):
    q=inverse_kin(d[i,:].T,-np.pi/2,theta_3=2)
    #Convert to degrees
    q_deg=np.array([q[0],q[1],q[2],q[3]])*180/np.pi
    #Magic offsets found from experiments
    qOffsets=[170,60,150,240]
    q_deg=q_deg+qOffsets


    robotMove(portHandler, packetHandler, q_deg)
    robotMove(portHandler, packetHandler, table1Pos)
    

#Using the fact that 
#distance_in_word/height_in_world=distance_in_image/focal_length
#We can calculate height



cap.release()
robotTerminate(portHandler, packetHandler)
cv2.destroyAllWindows()