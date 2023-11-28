import cv2
from matplotlib import pyplot as plt
import numpy as np
from robotConnect import *
import InverseKinematics as IK


def detect_round_object(frame):
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
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    print(hsv_image[240][360])
    cv2.circle(frame, (360, 240), 10, (0, 0, 255), 4)
    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # Draw the circles on the frame
        for (x, y, r) in circles:
            if (False):
                cv2.circle(frame, (x, y), r, (0, 0, 255), 4)

    return frame


def main():
    # Open the default camera (camera index 0)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

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
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
