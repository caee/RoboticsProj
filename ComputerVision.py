import cv2
from matplotlib import pyplot as plt
import numpy as np
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


def detect_round_object(frame):
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

    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # Draw the circles on the frame
        for (x, y, r) in circles:
            if (frame[y][x][2] > 80 and frame[y][x][1] < 80 and frame[y][x][1] < 80):
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

    return frame


def main():
    # Open the default camera (camera index 0)
    cap = cv2.VideoCapture(0)

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
        if cv2.waitKey(1) &     0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
