import cv2
from matplotlib import pyplot as plt
import numpy as np
from robotConnect import *
import InverseKinematics as IK
'''
cam = cv2.VideoCapture(0)
result, img = cam.read()
cv2.waitKey(0)

# cv2.imshow('image', img)
# cv2.waitKey(0)
'''
'''
imgheight = img.shape[0]
imglenght = img.shape[1]
for i in range(imgheight-1):
    for j in range(imglenght-1):
        if (70 > img[i][j][0] > 30 and 70 > img[i][j][1] > 30 and 80 < img[i][j][2] < 140):
            img[i][j][0] = 0
            img[i][j][1] = 0
            img[i][j][2] = 0


cv2.imshow('image', img)
cv2.waitKey(0)
'''
'''
cam = cv2.VideoCapture(0)
result, img = cam.read()
image = cv2.circle(img, (320, 240), 20, (255, 0, 0), 2)

cv2.imshow('image', img)
print(img.shape)
print(img[240][320])
cv2.waitKey(0)
'''


# def detect_round_object(frame):
#     # Convert the frame to grayscale
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Apply GaussianBlur to reduce noise and help with contour detection
#     blurred = cv2.GaussianBlur(gray, (5, 5), 0)

#     # Use HoughCircles to detect circles in the image
#     circles = cv2.HoughCircles(
#         blurred,
#         cv2.HOUGH_GRADIENT,
#         dp=1,
#         minDist=20,
#         param1=50,
#         param2=30,
#         minRadius=10,
#         maxRadius=50
#     )

#     if circles is not None:
#         # Convert the (x, y) coordinates and radius of the circles to integers
#         circles = np.round(circles[0, :]).astype("int")

#         # Draw the circles on the frame
#         for (x, y, r) in circles:
#             if (frame[y][x][2] > 80 and frame[y][x][1] < 80 and frame[y][x][1] < 80):
#                 cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

#     return frame

# def detect_round_object(frame):
#     # Convert the frame to grayscale
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Apply GaussianBlur to reduce noise and help with contour detection
#     blurred = cv2.GaussianBlur(gray, (5, 5), 0)

#     # Use HoughCircles to detect circles in the image
#     circles = cv2.HoughCircles(
#         blurred,
#         cv2.HOUGH_GRADIENT,
#         dp=1,
#         minDist=20,
#         param1=50,
#         param2=30,
#         minRadius=10,
#         maxRadius=50
#     )

#     if circles is not None:
#         # Convert the (x, y) coordinates and radius of the circles to integers
#         circles = np.round(circles[0, :]).astype("int")

#         # Draw the circles on the frame
#         for (x, y, r) in circles:
#             if (frame[y][x][2] > 80 and frame[y][x][1] < 80 and frame[y][x][1] < 80):
#                 cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

#     return frame

def detect_round_object(frame, debug=False):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help with contour detection
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    blurred = gray
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
            if (frame[y-1][x-1][2] > 110 and frame[y-1][x-1][1] < 70 and frame[y-1][x-1][0] < 30):
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            elif (y > 450 and x > 610 and frame[y+10][x+10][2] > 110 and frame[y+10][x+10][1] < 70 and frame[y+10][x+10][0] < 30):
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
    if debug == True:
        frame[230:250][:][2] = 0
        frame[230:250][:][1] = 0
        frame[230:250][:][0] = 0
        print(frame[240][320][2], frame[240][320][1], frame[240][320][0])
    return frame

# def circleCentroid(circle):

#     return


def main():
    # Open the default camera (camera index 0)
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Error: Could not open camera.")

    print("video capture is open")
    portHandler, packetHandler = robotConnect("COM6")
    zeroPos = [175, 150, 150, 150]
    testPos = [170, 170, 170, 170]
    tablePos = [175, 120, 98, 150]
    Postition = [100, 32, 170]
    thetaValues = IK.inverse_kin(Postition, 0)
    tablePos = [180/np.pi*thetaValues[0]+170, 180/np.pi*thetaValues[1]+150-90,
                180/np.pi*thetaValues[2]+150, 180/np.pi*thetaValues[3]+150]

    pos = zeroPos
    robotMove(portHandler, packetHandler, pos)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame was successfully captured
        if not ret:
            print("Error: Could not read frame.")
            break

        # Detect round objects in the frame
        result_frame = detect_round_object(frame)

        # Display the resulting frame
        cv2.imshow("Round Object Detection", result_frame)

        # Break the loop if 'q' key is pressed
        pressedKey = cv2.waitKey(1) & 0xFF
        if pressedKey == ord('q'):
            break
        elif pressedKey == ord('z'):
            robotMove(portHandler, packetHandler, zeroPos)
        elif pressedKey == ord('x'):
            robotMove(portHandler, packetHandler, tablePos)

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    robotTerminate(portHandler, packetHandler)


if __name__ == "__main__":
    main()
