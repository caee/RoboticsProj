# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 16:17:03 2023

@author: carle
"""
import cv2
import numpy as np


import os
os.chdir('[C:\\Users\\carle\\Documents\\RoboticsProj]')


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

img=cv2.imread("Plain-M&Ms-Pile.jpg")
aspect=500
r = aspect / img.shape[0]
dim = (int(img.shape[1] * r), aspect)


img=cv2.resize(img,dim)
roundImg,circles=detect_round_object(img)


cv2.imshow("Round Object Detection", roundImg)
cv2.waitKey(0)
cv2.destroyAllWindows()
    
