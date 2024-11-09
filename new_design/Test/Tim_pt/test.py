from ultralytics import YOLO
import cv2
import numpy as np     
import camera_var
import time

model = YOLO("best5_finetune.pt")
# model.predict(source = 0, save = False, show = True, conf = 0.5)


def cam_proc():
    cap = cv2.VideoCapture(0)

    object_positions = []
    accumulate_count = 0

    while True:
        time.sleep(0.01)
        ret, frame = cap.read()

        if not ret:
            break

        # Predict from opencv frame
        frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
        results = model(frame,conf=0.5, verbose=False)
        
        current_positions = []
        for result in results:
            for box in result.boxes.xyxy: 
                x_min, y_min, x_max, y_max = box
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                current_positions.append((x_center, y_center))
                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
                cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)


        if len(current_positions) == 1:
            if len(object_positions) == 0:
                object_positions = current_positions
                accumulate_count = 1
            else:
                stable = True
                for old_pos, new_pos in zip(object_positions, current_positions):
                    if abs(old_pos[0] - new_pos[0]) > 5 or abs(old_pos[1] - new_pos[1]) > 5:
                        stable = False
                        break
                if stable:
                    accumulate_count += 1
                else:
                    object_positions = current_positions 
                    accumulate_count = 1
            if accumulate_count >= 6:
                # print(f"Object positions stabilized at: {object_positions}")
                return object_positions
        cv2.imshow("YOLOv8 Real-Time", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    while True:
        try: 
            hand_pos = cam_proc()
            X_axis = round(hand_pos[0][0].item(), 5)
            Y_axis = round(hand_pos[0][1].item(), 5)
            message = f"!{X_axis}:{Y_axis}H#"
            # serialObject.write(bytes(message), encoding="utf-8")
            print(f"Send Serial: {message}")
            time.sleep(1)
        except:
            # serialObject.write(bytes("Invalid"), encoding="utf-8")
            print("Send Serial: Invalid")
            break
    
    
