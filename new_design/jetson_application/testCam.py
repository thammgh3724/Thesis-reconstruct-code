import cv2
import time

def capture_and_show():
    cap = cv2.VideoCapture(cv2.CAP_V4L2)

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
