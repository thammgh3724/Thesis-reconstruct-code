import cv2
import newCamVar

pipeline = (
    "v4l2src device=/dev/video0 ! "
    "videoconvert ! "
    "appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while not cap.isOpened():
    print("cap in normal")
    cap = cv2.VideoCapture(cv2.CAP_V4L2)

if not cap.isOpened():
    print("Can not open cam!")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't read frame!")
            break
        frame = cv2.undistort(frame, newCamVar.K_array, newCamVar.Dis_array, None, newCamVar.New_array)
        cv2.imshow("Low Latency Camera Stream", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
