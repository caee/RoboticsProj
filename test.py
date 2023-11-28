import cv2
import dlib

face_detector = dlib.get_frontal_face_detector()

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_detector(gray)

    for face in faces:
        x, y, w, h = face.left(), face.top(), face.width(), face.height()

        roi_left_eye = gray[y + int(0.1 * h):y + int(0.4 * h),
                            x + int(0.1 * w):x + int(0.5 * w)]
        roi_right_eye = gray[y + int(0.1 * h):y + int(0.4 * h),
                             x + int(0.5 * w): x + int(0.9 * w)]

        _, thresh_left_eye = cv2.threshold(
            roi_left_eye, 30, 255, cv2.THRESH_BINARY)
        _, thresh_right_eye = cv2.threshold(
            roi_right_eye, 30, 255, cv2.THRESH_BINARY)

        contours_left, _ = cv2.findContours(
            thresh_left_eye, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours_right, _ = cv2.findContours(
            thresh_right_eye, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours_left:
            x_c, y_c, w_c, h_c = cv2.boundingRect(contour)
            center_x = x + int(0.1 * w) + x_c + w_c // 2
            center_y = y + int(0.1 * h) + y_c + h_c // 2
            radius = max(w_c, h_c) // 3
            cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)

        for contour in contours_right:
            x_c, y_c, w_c, h_c = cv2.boundingRect(contour)
            center_x = x + int(0.5 * w) + x_c + w_c // 2
            center_y = y + int(0.1 * h) + y_c + h_c // 2
            radius = max(w_c, h_c) // 3
            cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)

    cv2.imshow('Eye Tracking', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
