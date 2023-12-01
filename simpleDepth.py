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


def detect_round_object(frame,debug=False):
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
    #Converting image to HSV color space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    ###########
    # MASKING #
    ###########
    #Masking to get only the red ones. First, thresholding hue, then threshold
    #value based on manual debugging
    h,s,v = cv2.split(hsv_image)
    if debug:
        #If debugging to fine-tune parameters
        titles=['h','s','v']
        plt.figure(figsize = (16,4))
        plt.subplot(1,3,1), plt.imshow(h)
        plt.title(titles[0])
        plt.xticks([]),plt.yticks([])
        plt.subplot(1,3,2), plt.imshow(s)
        plt.title(titles[1])
        plt.xticks([]),plt.yticks([])
        plt.subplot(1,3,3), plt.imshow(v)
        plt.title(titles[2])
        plt.xticks([]),plt.yticks([])
    
    #First, thresholding hue
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    threshold = 15
    threshold_value = 255

    thresh1 = cv2.threshold(h, threshold, threshold_value, cv2.THRESH_BINARY_INV)[1]
    #Closing to eliminate noise 
    #and getting a mask for the hue based on finding the red smarties
    mask1=cv2.erode(thresh1,kernel,iterations=3)
    mask1=cv2.dilate(mask1,kernel,iterations=3)
    if debug:
        plt.figure(figsize = (16,4))
        plt.imshow(mask1, cmap='gray')
    
    #Then thresholding value
    threshold = 100
    threshold_value = 255
    thresh2 = cv2.threshold(v, threshold, threshold_value, cv2.THRESH_BINARY)[1]
    
    
    #Again, closing
    mask2=cv2.erode(thresh2,kernel,iterations=3)
    mask2=cv2.dilate(mask2,kernel,iterations=3)
    if debug:
        plt.figure(figsize = (16,4))
        plt.imshow(mask2, cmap='gray')
    
    #Combining the masks
    mask=mask2 & mask1
    if debug:
        plt.figure(figsize = (16,4))
        plt.imshow(mask, cmap='gray')
    #cv2.waitKey(10000)
    
    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        redCircles=[]
        # apply masks and check whether circle center coordinates are located 
        #in positive parts of mask
        for (x, y, r) in circles:
            if mask[y-1,x-1]==255:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                redCircles.append([x,y,r])
                

    return frame, np.array(redCircles)
# Finds the depth of smarties in a plane, 
# from the known height of the camera used, the focal length and centroid positions

#Apply Camera Calibration
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
#Get image only if it is a real image
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

#Show the image captured
cv2.imshow("Frame",frame)
cv2.waitKey(1000)
h,w = frame.shape[:2]

#Optimize camera matrix based on undistortion params from camera calibration
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

cv2.imwrite('calibresult_simple.png', dst)
print("undistortion done")

#Apply detect_round_object to find redd smarties
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

#Magic factors to compensate for weird motor slip
xfactor=0.3
yfactor=1.1
for i in range(0,len(d_camera)):
    #OFFSET from image center coordinates to robot coords

    d[i,0]=x_off-d_camera[i,1]
    d[i,1]=-d_camera[i,0]*yfactor
    d[i,2]=z_off+h_mm+25-d_camera[i,1]*xfactor #xfactor added to compensate for weird z drift



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