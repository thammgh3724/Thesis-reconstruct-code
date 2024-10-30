import cv2

# Pipeline GStreamer cho OpenCV
# Điều chỉnh độ phân giải và tốc độ khung hình để tối ưu hóa hiệu suất
pipeline = (
    "v4l2src device=/dev/video0 ! "  # Đường dẫn thiết bị, thay đổi nếu cần
    "video/x-raw, width=640, height=480, framerate=30/1 ! "  # Cấu hình độ phân giải và FPS
    "videoconvert ! "  # Chuyển đổi định dạng cho OpenCV
    "queue max-size-buffers=1 leaky=downstream ! "  # Hạn chế độ trễ
    "appsink"
)

# Mở capture với pipeline GStreamer
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# Kiểm tra xem camera có mở được không
if not cap.isOpened():
    print("Không thể mở camera")
else:
    while True:
        # Đọc khung hình
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình")
            break

        # Hiển thị khung hình
        cv2.imshow("Low Latency Camera Stream", frame)

        # Nhấn 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Giải phóng bộ nhớ
cap.release()
cv2.destroyAllWindows()
