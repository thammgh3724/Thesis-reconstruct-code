from ultralytics import YOLO
import cv2

model = YOLO("best.pt")
# model.predict(source = 0, save = False, show = True, conf = 0.5)
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Predict from opencv frame
    results = model(frame)

    for result in results:
        for box in result.boxes.xyxy: 
            x_min, y_min, x_max, y_max = box
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2

            cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
            cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)
            print(f"x_center: {x_center}, y_center: {y_center}")

    cv2.imshow("YOLOv8 Real-Time", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()