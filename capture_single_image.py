# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 15:40:18 2023

@author: carle
"""
import cv2
from matplotlib import pyplot as plt
import numpy as np
from robotConnect import *
from cameraCalib import calibrateCamera

direc = "...\\RoboticsProj\\calib"
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
cv2.imwrite('testimage.png', dst)

robotTerminate(portHandler, packetHandler)
cv2.destroyAllWindows()