import cv2
from matplotlib import pyplot as plt
import numpy as np
from robotConnect import *

# Open the default camera (camera index 0)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the frame was read successfully
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Display the frame
    cv2.imshow("Webcam Feed", frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
if not cap.isOpened():
    print("Error: Could not open camera.")

print("video capture is open")
portHandler, packetHandler = robotConnect("COM4")
zeroPos = [150, 150, 150, 150]
testPos = [170, 170, 170, 170]
tablePos = [150, 120, 98, 150]
pos = zeroPos
robotMove(portHandler, packetHandler, pos)
if not cap.isOpened():
    print("Error: Could not open camera.")


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
