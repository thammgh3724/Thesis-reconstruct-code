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
        self.stop_event.clear()
        self.pause_event.set()
        self.cap = cv2.VideoCapture(cv2.CAP_V4L2)

        while not self.stop_event.is_set():
            if self.pause_event.is_set():
                hand_pos = self.cam_proc()
                if hand_pos:
                    self.hand_position = hand_pos
                    print(hand_pos)

        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def cam_proc(self):
        object_positions = []  # Stores stable positions of detected objects
        accumulate_count = 0   # Counter to accumulate stable positions

        while not self.stop_event.is_set() and self.pause_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                break

            # Undistort and preprocess the frame
            frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
            
            # Perform hand detection using YOLO
            results = self.model.track(frame, imgsz=640, conf=0.5, device=0, save=False, verbose=False)
            current_positions = []

            # Extract detected bounding boxes and calculate their centers
            for result in results:
                for box in result.boxes.xyxy:
                    x_min, y_min, x_max, y_max = box
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2
                    current_positions.append((x_center, y_center))

                    # Draw bounding boxes and center points for visualization
                    cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)

            if len(current_positions) == 1:
                # Single object detected: Get current hand position
                x_current, y_current = current_positions[0]

                if len(object_positions) == 0:
                    # First detection: Initialize the position
                    object_positions = current_positions
                    accumulate_count = 1
                else:
                    # Calculate stability based on pixel threshold
                    stable = all(abs(old_pos[0] - new_pos[0]) <= 5 and abs(old_pos[1] - new_pos[1]) <= 5
                                for old_pos, new_pos in zip(object_positions, current_positions))
                    
                    # Check significant movement (> 20 pixels) from last sent position
                    THRESHOLD = 20  # Define movement threshold
                    significant_movement = (abs(x_current - object_positions[0][0]) > THRESHOLD or
                                            abs(y_current - object_positions[0][1]) > THRESHOLD)
                    
                    if stable and significant_movement:
                        accumulate_count += 1  # Increment stable count if both conditions met
                    else:
                        # Reset stability and accumulate count if movement is insignificant
                        object_positions = current_positions
                        accumulate_count = 1

                # If position remains stable for 3 frames and exceeds movement threshold, return it
                if accumulate_count >= 3:
                    accumulate_count = 0
                    object_positions = current_positions
                    return object_positions

            # Display the processed frame
            cv2.imshow("YOLOv8 Real-Time", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Quit on 'q' key press
                break

        return None  # Return None if no valid hand position detected


if __name__ == "__main__":

    if torch.cuda.is_available():
      print("Confirm CUDA recognized")

    from ultralytics import YOLO
    print("here")
    
    handdt = HandDetectHandler()
    handdt.start()
