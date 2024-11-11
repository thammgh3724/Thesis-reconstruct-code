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
        self.model = YOLO("best4.engine")
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
        # check if CUDA available
        if torch.cuda.is_available():
            print("CONFIRM CUDA AVAILABLE")

        self.stop_event.clear()
        self.pause_event.set()
        self.cap = cv2.VideoCapture(cv2.CAP_V4L2)

        while not self.stop_event.is_set():
            if self.pause_event.is_set():
                hand_pos = self.cam_proc()
                if hand_pos:
                    self.hand_position = hand_pos
                    time.sleep(0.3)
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
            results = self.model.track(frame, imgsz=640, conf=0.2, device = 0, save = False, verbose = False)

            current_positions = []
            for result in results:
                for box in result.boxes.xyxy:
                    x_min, y_min, x_max, y_max = box
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2
                    if (x_center - 320 > 10) or (y_center - 240 > 10):
                        current_positions.append((x_center, y_center))
                    cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)

            if len(current_positions) == 1:
                if len(object_positions) == 0:
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
                    accumulate_count = 0
                    return object_positions

            cv2.imshow("YOLOv8 Real-Time", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        return None


if __name__ == "__main__":
    handdt = HandDetectHandler()
    handdt.start()

    time.sleep(3)
    handdt.pause() 
    time.sleep(5)
    handdt.resume()  

    time.sleep(5)
    handdt.stop()
    handdt.join()  
