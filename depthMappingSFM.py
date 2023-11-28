# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 13:41:15 2023

@author: carle
"""
import cv2
from matplotlib import pyplot as plt
import numpy as np
from triangulation import linear_LS_triangulation
#from robotConnect import *
#from cameraCalib import calibrateCamera
import os
direc = "C:\\Users\\carle\\Documents\\RoboticsProj\\calib"
os.chdir(direc)

#Inspired by https://github.com/hsuanhauliu/structure-from-motion-with-OpenCV/blob/master/main.py

###### FOR TESTING PURPOSES #######
dst1=cv2.imread('calibresult1.png')
dst2=cv2.imread('calibresult2.png')
h,w=np.shape(dst1)[:2]
newcameramtx=np.genfromtxt("testnewcameramtx.csv", delimiter=",")

#### END TESTING LOAD#####

# cv2.destroyAllWindows()
gray1=cv2.cvtColor(dst1,cv2.COLOR_BGR2GRAY)
gray2=cv2.cvtColor(dst2,cv2.COLOR_BGR2GRAY)


#Keypoint descriptors from both
#detect sift features for both images
sift = cv2.SIFT_create()
orb = cv2.ORB_create()
# find the keypoints and descriptors with SIFT
# kp1, des1 = sift.detectAndCompute(gray1, None)
# kp2, des2 = sift.detectAndCompute(gray2, None)

kp1, des1 = orb.detectAndCompute(gray1, None)
kp2, des2 = orb.detectAndCompute(gray2, None)

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


#Finding best matches:
nb_matches = 200

good = []
pts1 = []
pts2 = []

for m in matches[:nb_matches]:
    good.append(m)
    pts1.append(kp1[m.queryIdx].pt)
    pts2.append(kp2[m.trainIdx].pt)

pts1 = np.int32(pts1)
pts2 = np.int32(pts2)


E, mask = cv2.findEssentialMat(pts1, pts2, newcameramtx, cv2.RANSAC, 0.999, 1.0);

#Inliers
pts1 = pts1[mask.ravel() == 1]
pts2 = pts2[mask.ravel() == 1]



#Recovering camera pose
points, R, t, mask = cv2.recoverPose(E, pts1, pts2)
print("Rotation:")
print(R)
print("Translation:")
print(t)
# p1_tmp = np.expand_dims(np.squeeze(p1), 0)
p1_tmp = np.ones([3, pts1.shape[0]])
p1_tmp[:2,:] = np.squeeze(pts1).T
p2_tmp = np.ones([3, pts2.shape[0]])
p2_tmp[:2,:] = np.squeeze(pts2).T
#print((np.dot(R, p2_tmp) + t) - p1_tmp)



###
#Triangulation
###
#calculate projection matrix for both camera
M_r = np.hstack((R, t))
M_l = np.hstack((np.eye(3, 3), np.zeros((3, 1))))

P_l = np.dot(newcameramtx,  M_l)
P_r = np.dot(newcameramtx,  M_r)

# undistort points
#pts1 = pts1[np.asarray(matchesMask)==1,:,:]
#pts2 = pts2[np.asarray(matchesMask)==1,:,:]
#p1_un=pts1.reshape(-1,1,2)
#p2_un=pts2.reshape(-1,1,2)
#p1_un = cv2.undistortPoints(pts1,newcameramtx,None)
#p2_un = cv2.undistortPoints(pts2,newcameramtx,None)
#p1_un = np.squeeze(p1_un)
#p2_un = np.squeeze(p2_un)
p1_un=pts1.T
p2_un=pts2.T


#triangulate points this requires points in normalized coordinate
points_3d=linear_LS_triangulation(p1_un.T,P_l,p2_un.T,P_r)
points_3d=points_3d[0]
#point_4d_hom = cv2.triangulatePoints(P_l, P_r, p1_un.T, p2_un.T)
#print(point_4d_hom)
# point_3d = point_4d_hom / np.tile(point_4d_hom[-1, :], (4, 1))
# point_3d = point_3d[:3, :].T

#############################
#5----output 3D pointcloud--#
#############################
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')

# for x, y, z in point_3d:
#     ax.scatter(x, y, z, c="r", marker="o")

# plt.show()
# counter=1
# fig.savefig('3-D_' + str(counter) + '.jpg')


