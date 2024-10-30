import cv2

pipeline = "v4l2src device=/dev/video4 ! videoconvert ! appsink"
cap = cv2.VideoCapture(0)


while not cap.isOpened():
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Can not open cam!")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't read frame!")
            break

        cv2.imshow("Low Latency Camera Stream", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
