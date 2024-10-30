import cv2
import time


pipeline = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw, width=640, height=480, framerate=30/1 ! "
    "videoconvert ! "
    "appsink"
)

def capture_and_show():

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Không thể nhận khung hình từ camera.")
            break

        cv2.imshow("Camera", frame)
        time.sleep(0.5)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Giải phóng tài nguyên
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_show()
