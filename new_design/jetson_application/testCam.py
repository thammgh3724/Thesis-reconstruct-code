import cv2

pipeline = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw, width=640, height=480, framerate=30/1 ! "
    "videoconvert ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

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
