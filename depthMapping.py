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

def drawlines(img1src, img2src, lines, pts1src, pts2src):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r, c = img1src.shape
    img1color = cv2.cvtColor(img1src, cv2.COLOR_GRAY2BGR)
    img2color = cv2.cvtColor(img2src, cv2.COLOR_GRAY2BGR)
    # Edit: use the same random seed so that two images are comparable!
    np.random.seed(0)
    for r, pt1, pt2 in zip(lines, pts1src, pts2src):
        color = tuple(np.random.randint(0, 255, 3).tolist())
        x0, y0 = map(int, [0, -r[2]/r[1]])
        x1, y1 = map(int, [c, -(r[2]+r[0]*c)/r[1]])
        img1color = cv2.line(img1color, (x0, y0), (x1, y1), color, 1)
        img1color = cv2.circle(img1color, tuple(pt1), 5, color, -1)
        img2color = cv2.circle(img2color, tuple(pt2), 5, color, -1)
    return img1color, img2color

#For calibration purposes
direc = "C:\\Users\\carle\\Documents\\RoboticsProj\\calib"
ret,mtx,dist,rvecs,tvecs=calibrateCamera(direc)


#### MOVE ROBOT and take photos for 3D depth detection####
# #Open camera an get 2 images of smarties
# cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# if not cap.isOpened():
#     print("Error: Could not open camera.")

# print("video capture is open")
# portHandler, packetHandler = robotConnect("COM5")
# # zeroPos = [150, 150, 150, 150]
# # robotMove(portHandler, packetHandler, zeroPos)
# # table1Pos = [140, 120, 98, 150]
# # table2Pos = [160, 120, 98, 150]
# table1Pos = [170, 135, 108, 140]
# table2Pos = [185, 135, 108, 140]

# #First, get 2 frames

# robotMove(portHandler, packetHandler, table1Pos)
# _,_ = cap.read() #flush camera frame
# ret, frame1 = cap.read()

# # Check if the frame was successfully captured
# if not ret:
#     print("Error: Could not read frame.")
# print("waiting...")    
# # cv2.waitKey(2000)    
# robotMove(portHandler, packetHandler, table2Pos)

# print("what")
# _,_ = cap.read() #flush camera frame
# ret, frame2 = cap.read()
# # Check if the frame was successfully captured
# if not ret:
#     print("Error: Could not read frame.")
# cap.release()

# # cv2.imshow("Frame1",frame1)
# # cv2.imshow("Frame2",frame2)
# # cv2.waitKey(5000)    
# #Then, finding camera matrixes from images
# h,w = frame1.shape[:2]
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))


    
# #Undistort images from intrinsics found before

# dst1 = cv2.undistort(frame1, mtx, dist, None, newcameramtx)
# dst2 = cv2.undistort(frame2, mtx, dist, None, newcameramtx)
# # crop the image
# x, y, w, h = roi
# dst1 = dst1[y:y+h, x:x+w]
# dst2 = dst2[y:y+h, x:x+w]
# cv2.imwrite('calibresult1.png', dst1)
# cv2.imwrite('calibresult2.png', dst2)
# print("calibration done")
# #cv2.imshow("F1", frame1)
# #cv2.imshow("undistorted F1", dst1)
# #cv2.imshow("F2", frame2)
# #cv2.imshow("Undistorted F2", dst2)
# #cv2.waitKey(0)

dst1=cv2.imread('calibresult1.png')
dst2=cv2.imread('calibresult2.png')
# cv2.destroyAllWindows()
gray1=cv2.cvtColor(dst1,cv2.COLOR_BGR2GRAY)
gray2=cv2.cvtColor(dst2,cv2.COLOR_BGR2GRAY)


#IMAGE RECTIFICATION
#Based on https://www.andreasjakl.com/understand-and-apply-stereo-rectification-for-depth-maps-part-2/
# Initiate SIFT detector
sift = cv2.SIFT_create()
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(gray1, None)
kp2, des2 = sift.detectAndCompute(gray2, None)

#match keypoints
#BF MATCHING
bf = cv2.BFMatcher()
matches = bf.match(des1, des2)
#sort matches
matches = sorted(matches, key = lambda x:x.distance)

#plot them
print("plotting matches")
kp_img = cv2.drawKeypoints(gray1, kp1, dst1, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
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

#E = cv2.findEssentialMat(pt1,pt2,newcameramtx)
#F_from_E= newcameramtx.inv().t() * E * camera_matrix.inv()


# We select only inlier points
pts1 = pts1[mask.ravel() == 1]
pts2 = pts2[mask.ravel() == 1]

## FINDING EPILINES


plt.figure(figsize = (20,20))
# Find epilines corresponding to points in right image (second image) and
# drawing its lines on left image
lines1 = cv2.computeCorrespondEpilines(
    pts2.reshape(-1, 1, 2), 2, F)
lines1 = lines1.reshape(-1, 3)
img5, img6 = drawlines(gray1, gray2, lines1, pts1, pts2)

# Find epilines corresponding to points in left image (first image) and
# drawing its lines on right image
lines2 = cv2.computeCorrespondEpilines(
    pts1.reshape(-1, 1, 2), 1, F)
lines2 = lines2.reshape(-1, 3)
img3, img4 = drawlines(gray2, gray1, lines2, pts2, pts1)
print("printing epipolar lines")
plt.subplot(121), plt.imshow(img5)
plt.subplot(122), plt.imshow(img3)
plt.suptitle("Epilines in both images")
plt.show()






#Rectifying
imageSize=gray1.shape[::-1][1:3]
print("rectifying images...")
_, H1, H2 = cv2.stereoRectifyUncalibrated(
    np.float32(pts1), np.float32(pts2), F, imgSize=(w, h)
)

img1_rectified = cv2.warpPerspective(frame1, H1, (w, h))
img2_rectified = cv2.warpPerspective(frame2, H2, (w, h))
cv2.imwrite("rectified_1.png", img1_rectified)
cv2.imwrite("rectified_2.png", img2_rectified)
cv2.imshow("REctified 1",img1_rectified)
cv2.waitKey(1)
grayrect1 = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
grayrect2 = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)


#Stereo Block Matching
# print("doing disparity calculations")
# min_disp = 7
# num_disp = 3 * 16
# block_size = 15
# stereo = cv2.StereoBM_create(numDisparities = num_disp, blockSize = block_size)
# stereo.setMinDisparity(min_disp)
# stereo.setDisp12MaxDiff(100)
# stereo.setUniquenessRatio(1)
# stereo.setSpeckleRange(3)
# stereo.setSpeckleWindowSize(3)


# disp = stereo.compute(grayrect1,grayrect2)
# print("disparity done, plotting")
# plt.figure(figsize=(18,18))
# plt.imshow(disp,'gray')



#ANOTHER DISPARITY CALC
# ------------------------------------------------------------
# CALCULATE DISPARITY (DEPTH MAP)
# Adapted from: https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py
# and: https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html

# StereoSGBM Parameter explanations:
# https://docs.opencv.org/4.5.0/d2/d85/classcv_1_1StereoSGBM.html

# Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
block_size = 5
min_disp = 1*16
max_disp = 4*16
# Maximum disparity minus minimum disparity. The value is always greater than zero.
# In the current implementation, this parameter must be divisible by 16.
num_disp = max_disp - min_disp
# Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
# Normally, a value within the 5-15 range is good enough
uniquenessRatio = 5
# Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
# Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
speckleWindowSize = 200
# Maximum disparity variation within each connected component.
# If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
# Normally, 1 or 2 is good enough.
speckleRange = 2
disp12MaxDiff = 0

stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    uniquenessRatio=uniquenessRatio,
    speckleWindowSize=speckleWindowSize,
    speckleRange=speckleRange,
    disp12MaxDiff=disp12MaxDiff,
    P1=8 * 1 * block_size * block_size,
    P2=32 * 1 * block_size * block_size,
)
disparity_SGBM = stereo.compute(grayrect1, grayrect2)

# Normalize the values to a range from 0..255 for a grayscale image
disparity_SGBM = cv2.normalize(disparity_SGBM, disparity_SGBM, alpha=255,
                              beta=0, norm_type=cv2.NORM_MINMAX)
disparity_SGBM = np.uint8(disparity_SGBM)
cv2.imshow("Disparity", disparity_SGBM)
cv2.imwrite("disparity_SGBM_norm.png", disparity_SGBM)


robotTerminate(portHandler, packetHandler)
cv2.destroyAllWindows()
    


