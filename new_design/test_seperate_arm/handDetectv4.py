import threading
import time
import cv2
import numpy as np
import camera_var
import torch 
import sys
import os
import requests 
from PIL import Image, ImageOps
from ultralytics import YOLO

class HandDetectHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.model = YOLO("best5_finetune.engine")
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.hand_position = None
        self.cap = None
        self.threshold = 40  # Threshold for position change in pixels

    def stop(self):
        self.stop_event.set()
        self.pause_event.set()

    def pause(self):
        self.pause_event.clear()
        if self.cap and self.cap.isOpened():
            self.cap.release()  # Release the camera
        cv2.destroyAllWindows()  # Close the camera window

    def resume(self):
        self.pause_event.set()
        if not self.cap or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cv2.CAP_V4L2)  # Reopen the camera if it's closed

    def run(self):
        # Check if CUDA is available
        if torch.cuda.is_available():
            print("CONFIRM CUDA AVAILABLE")

        self.stop_event.clear()
        self.pause_event.set()
        self.cap = cv2.VideoCapture(cv2.CAP_V4L2)

        while not self.stop_event.is_set():
            if self.pause_event.is_set():
                hand_pos = self.cam_proc()
                if hand_pos:
                    # Update hand position only if the change exceeds the threshold
                    if not self.hand_position or self.is_position_changed(hand_pos[0]):
                        self.hand_position = hand_pos
                        time.sleep(0.3)  # Avoid overwhelming the system with updates
                        print(hand_pos)

        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def cam_proc(self):
        object_positions = []
        accumulate_count = 0

        while not self.stop_event.is_set() and self.pause_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                break

            frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
            results = self.model.track(frame, imgsz=640, conf=0.2, device=0, save=False, verbose=False)

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
                if not object_positions:
                    object_positions = current_positions
                    accumulate_count = 1
                else:
                    stable = all(abs(old_pos[0] - new_pos[0]) <= 5 and abs(old_pos[1] - new_pos[1]) <= 5
                                for old_pos, new_pos in zip(object_positions, current_positions))
                    if stable:
                        accumulate_count += 1
                    else:
                        object_positions = current_positions
                        accumulate_count = 1

                if accumulate_count >= 3:
                    if self.hand_position:
                        if self.is_position_changed(current_positions[0]):
                            self.hand_position = current_positions
                            return object_positions  # Only return if position changed significantly
                    else:
                        self.hand_position = current_positions
                        return object_positions

            cv2.imshow("YOLOv8 Real-Time", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        return None  # Return None if no valid hand position detected


    def is_position_changed(self, new_position):
        """
        Check if the position change exceeds the threshold.
        """
        if not self.hand_position:
            return True
        old_x, old_y = self.hand_position[0]
        new_x, new_y = new_position
        return abs(new_x - old_x) > self.threshold or abs(new_y - old_y) > self.threshold


if __name__ == "__main__":

    if torch.cuda.is_available():
      print("Confirm CUDA recognized")

    from ultralytics import YOLO
    print("here")
    
    handdt = HandDetectHandler()
    handdt.start()
