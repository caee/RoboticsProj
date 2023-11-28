# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 16:17:03 2023

@author: carle
"""
import cv2
from matplotlib import pyplot as plt
import numpy as np
from robotConnect import *
from cameraCalib import calibrateCamera
# import os
# os.chdir('[C:\\Users\\carle\\Documents\\RoboticsProj]')


def detect_round_object(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help with contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    #blurred=gray
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

    return frame,circles

#For calibration purposes
direc = "C:\\Users\\carle\\Documents\\RoboticsProj\\calib"
ret,mtx,dist,rvecs,tvecs=calibrateCamera(direc)

#Open camera an get 2 images of smarties
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Error: Could not open camera.")

print("video capture is open")
portHandler, packetHandler = robotConnect("COM5")
# zeroPos = [150, 150, 150, 150]
# robotMove(portHandler, packetHandler, zeroPos)
# table1Pos = [140, 120, 98, 150]
# table2Pos = [160, 120, 98, 150]
table1Pos = [170, 135, 108, 140]
table2Pos = [185, 135, 108, 140]

#First, get 2 frames

robotMove(portHandler, packetHandler, table1Pos)
_,_ = cap.read() #flush camera frame
ret, frame1 = cap.read()

# Check if the frame was successfully captured
if not ret:
    print("Error: Could not read frame.")
print("waiting...")    
# cv2.waitKey(2000)    
robotMove(portHandler, packetHandler, table2Pos)

print("what")
_,_ = cap.read() #flush camera frame
ret, frame2 = cap.read()
# Check if the frame was successfully captured
if not ret:
    print("Error: Could not read frame.")
cap.release()

# cv2.imshow("Frame1",frame1)
# cv2.imshow("Frame2",frame2)
# cv2.waitKey(5000)    
#Then, finding camera matrixes from images
h,w = frame1.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))


    
#Undistort images from intrinsics found before

dst1 = cv2.undistort(frame1, mtx, dist, None, newcameramtx)
dst2 = cv2.undistort(frame2, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst1 = dst1[y:y+h, x:x+w]
dst2 = dst2[y:y+h, x:x+w]
cv2.imwrite('calibresult1.png', dst1)
cv2.imwrite('calibresult2.png', dst2)
print("calibration done")
#cv2.imshow("F1", frame1)
#cv2.imshow("undistorted F1", dst1)
#cv2.imshow("F2", frame2)
#cv2.imshow("Undistorted F2", dst2)
#cv2.waitKey(0)

cv2.destroyAllWindows()
gray1=cv2.cvtColor(dst1,cv2.COLOR_BGR2GRAY)
gray2=cv2.cvtColor(dst2,cv2.COLOR_BGR2GRAY)


#IMAGE RECTIFICATION
# Initiate SIFT detector
sift = cv2.SIFT_create()
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(gray1, None)
kp2, des2 = sift.detectAndCompute(gray2, None)

#match keypoints
bf = cv2.BFMatcher()
matches = bf.match(des1, des2)
#sort matches
matches = sorted(matches, key = lambda x:x.distance)

#plot them
kp_img = cv2.drawKeypoints(gray1, kp1, frame1, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
plt.figure(figsize = (10,10))
plt.imshow(kp_img)
img3 = cv2.drawMatches(gray1,kp1,gray2,kp2,matches[:30],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

plt.figure(figsize = (20,20))
plt.imshow(img3)



#finding best matches and a fundamental matrix
nb_matches = 100

good = []
pts1 = []
pts2 = []

for m in matches[:nb_matches]:
    good.append(m)
    pts1.append(kp1[m.queryIdx].pt)
    pts2.append(kp2[m.trainIdx].pt)

pts1 = np.int32(pts1)
pts2 = np.int32(pts2)
    
"""
Implement findFundamentalMat here:
"""
F, mask = cv2.findFundamentalMat(pts1,pts2, method = cv2.FM_RANSAC, ransacReprojThreshold = 3.,confidence = 0.99)

# We select only inlier points
pts1 = pts1[mask.ravel() == 1]
pts2 = pts2[mask.ravel() == 1]

#Rectifying
imageSize=gray1.shape[::-1][1:3]
_, H1, H2 = cv2.stereoRectifyUncalibrated(
    np.float32(pts1), np.float32(pts2), F, imgSize=(w, h)
)

img1_rectified = cv2.warpPerspective(frame1, H1, (w, h))
img2_rectified = cv2.warpPerspective(frame2, H2, (w, h))
cv2.imwrite("rectified_1.png", img1_rectified)
cv2.imwrite("rectified_2.png", img2_rectified)

#Stereo Block Matching
print("doing disparity calculations")
min_disp = 7
num_disp = 3 * 16
block_size = 15
stereo = cv2.StereoBM_create(numDisparities = num_disp, blockSize = block_size)
stereo.setMinDisparity(min_disp)
stereo.setDisp12MaxDiff(100)
stereo.setUniquenessRatio(1)
stereo.setSpeckleRange(3)
stereo.setSpeckleWindowSize(3)

disp = stereo.compute(gray1,gray2).astype(np.float32) / 16.0
print("disparity done, plotting")
plt.figure(figsize=(18,18))
plt.imshow(disp,'gray')



robotTerminate(portHandler, packetHandler)
    
