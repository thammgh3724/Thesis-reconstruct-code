import threading
import time
import cv2
import numpy as np
import camera_var
from ultralytics import YOLO

class HandDetectHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.model = YOLO("best4.pt")
        self.isRunning = False
        self.hand_position = None  # Placeholder to store hand position

    def stop(self):
        self.isRunning = False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            # Call the camera processing function to get hand position
            hand_pos = self.cam_proc()
            if hand_pos:
                self.hand_position = hand_pos  # Update the hand position
            time.sleep(1)  # Adjust delay as needed

    def cam_proc(self):
        """Camera processing to detect hand with position stabilization"""
        cap = cv2.VideoCapture(0)
        object_positions = []
        accumulate_count = 0

        while self.isRunning:
            time.sleep(0.3)
            ret, frame = cap.read()

            if not ret:
                break

            # Undistort the frame using calibration values
            frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
            results = self.model(frame, verbose=False)

            current_positions = []
            for result in results:
                for box in result.boxes.xyxy:
                    x_min, y_min, x_max, y_max = box
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2
                    current_positions.append((x_center, y_center))
                    cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)

            # Stabilize the detected position by accumulating over multiple frames
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

                # If the hand position has been stable for 3 consecutive frames, return the position
                if accumulate_count >= 3:
                    cap.release()
                    cv2.destroyAllWindows()
                    return object_positions  # Return stabilized hand position

            # Display the frame with bounding boxes
            cv2.imshow("YOLOv8 Real-Time", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        return None  # Return None if the loop ends
