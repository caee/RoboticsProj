# -*- coding: utf-8 -*-
"""
Created on Tue Nov 21 16:25:32 2023

@author: carle
"""
import numpy as np
import cv2 
import glob
import os

direc = "C:\\Users\\carle\\Documents\\RoboticsProj\\calib"
os.chdir(direc) 

def captureChess(j):
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Error: Could not open camera.")

    print("video capture is open")
    i=0
    while i<j:
        i+=1
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Error: Could not read frame.")
            break
        cv2.imshow("captured frame",frame)
        # write to 
        filename="CI{}.jpg".format(i)
        cv2.imwrite(filename,frame)
        
        # Break the loop if 'q' key is pressed
        pressedKey = cv2.waitKey(1000) & 0xFF
        if pressedKey == ord('q'):
            break
        

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

def calibrateCamera(directory=direc,nb_vert=9,nb_horiz=6,showcalib=False):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((nb_vert*nb_horiz,3), np.float32)
    objp[:,:2] = np.mgrid[0:nb_vert,0:nb_horiz].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    os.chdir(directory)
    images = glob.glob('*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nb_vert,nb_horiz), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            if showcalib:
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (nb_vert,nb_horiz), corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(500)
    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return ret,mtx,dist,rvecs,tvecs
#captureChess(15)
#ret,mtx,dist,rvecs,tvecs=calibrateCamera(showcalib=True)