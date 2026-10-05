import cv2

# Change the camera indexes here in one place.
# For a setup using USB camera 1 and USB camera 2, leave this as [1, 2].
CAMERA_INDEXES = [1, 2]
current_index = 0

while True:
    current = CAMERA_INDEXES[current_index]

    cap = cv2.VideoCapture(current)
    if not cap.isOpened():
        print(f"Camera {current} not found")
        current_index = (current_index + 1) % len(CAMERA_INDEXES)
        continue

    print(f"Opened Camera {current}")

    while True:
        ret, frame = cap.read()

        if not ret:
            print(f"Camera {current} failed")
            cap.release()
            current_index = (current_index + 1) % len(CAMERA_INDEXES)
            break

        cv2.putText(
            frame,
            f"Camera Index: {current}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        cv2.imshow("Camera Test", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == 32:
            cap.release()
            current_index = (current_index + 1) % len(CAMERA_INDEXES)
            break

        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            exit()

cv2.destroyAllWindows()