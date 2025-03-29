import threading
import camera_var
import time
import cv2
import numpy as np
import torch 
import sys
import os
import yaml 
import requests 
from PIL import Image, ImageOps
from ultralytics import YOLO
from serialObjectSingleton import SerialSingleton

class AutoModeHandler(threading.Thread):
    def __init__(self, serialObj):
        threading.Thread.__init__(self)
        ## load model for classify fruit
        self.model = YOLO("beta_fruit1.engine")
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.fruit_position = None
        self.isSending = False
        self.cap = None
        self.threshold = 60  # Threshold for position change in pixels

        # Thêm biến đếm và thời gian
        self.detection_count = 0
        self.start_time = None

        # Add serial object for instant slider control signal
        self.serialObj = serialObj

        # Add verification slider signal variabl
        self.isSendSliderSignal = False

        # Class labels
        self.class_names = ['Guava', 'Mango', 'fresh orange']
        
        self.class_labels = []

    # Function: Send instant slider control signal
    def isSendSlider(self):
        return self.isSendSliderSignal
    
    def sendInstantSliderSignal(self):
        print("SLIDER: MOVING CLOSER TO THE TARGET")
        self.serialObj.write(bytes(str("!1:0S#"), encoding='utf-8'))
        self.isSendSliderSignal = True
        time.sleep(1)
        self.serialObj.write(bytes(str("!sstop#"), encoding="utf-8"))
        print("SLIDER: MOVE DONE, CLOSER TO THE TARGET")

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
                    self.fruit_position = hand_pos
                    self.isSending = True
                    time.sleep(10)

                    
                    # self.detection_count += 1

                    # if self.detection_count == 5:
                    #     elapsed_time = time.time() - self.start_time
                    #     print(f"Detected 5 fruit positions in {elapsed_time:.2f} seconds. Fruit is {self.class_labels[0]}")

                    #     # Reset count and timer
                    #     self.detection_count = 0
                    #     self.start_time = None

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
                
                boxes = result.boxes.xyxy.cpu().numpy()  # [x_min, y_min, x_max, y_max]
                class_ids = result.boxes.cls.cpu().numpy()  # Danh sách class ID
                
                # Sắp xếp theo tọa độ x_min (vị trí từ trái sang phải)
                sorted_indices = np.argsort(boxes[:, 0])  # Sắp xếp theo x_min
                self.class_labels = [self.class_names[int(class_ids[i])] for i in sorted_indices]

            if len(current_positions) >= 1:
                min_x = float("inf")
                min_pos = None # Tuple
                obj = 0
                for pos in current_positions:
                    x_center, y_center = pos
                    obj = obj + 1
                    # print(obj)
                    # print("Object  has position: ", pos)
                    if x_center < min_x:
                        min_x = x_center
                        min_pos = pos
                        
                if (abs(min_pos[0]) > 440 or abs(min_pos[0]) < 160):
                    print(f"object need to be closer :{min_pos} ")
                    #TODO: Add signal to automate slider movement immediately
                    self.sendInstantSliderSignal()
                    continue

                if len(object_positions) == 0:
                    object_positions.append(min_pos)
                    accumulate_count = 1
                else:
                    stable = all(abs(old_pos[0] - min_pos[0]) <= 5 and abs(old_pos[1] - min_pos[1]) <= 5
                                 for old_pos in object_positions)
                    if stable:
                        accumulate_count += 1
                        print("Accumulated count ", accumulate_count)
                    else:
                        object_positions = []
                        object_positions.append(min_pos)
                        accumulate_count = 1
                time.sleep(0.5)

                if accumulate_count >= 5:
                    accumulate_count = 0
                    return object_positions

            cv2.imshow("YOLOv8 Real-Time", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        return None  # Return None if no fruits is detected

    def is_position_changed(self, new_position):
        """
        Check if the position change exceeds the threshold.
        """
        old_x, old_y = self.fruit_position[0]
        new_x, new_y = new_position
        return abs(new_x - old_x) > self.threshold or abs(new_y - old_y) > self.threshold


if __name__ == "__main__":

    if torch.cuda.is_available():
      print("Confirm CUDA recognized")

    from ultralytics import YOLO
    print("here")

    serial_port = '/dev/ttyACM0'
    baud_rate = 115200
    serial_obj = SerialSingleton(serial_port, baud_rate, 0.01)
    
    fruitAutoDetection = AutoModeHandler(serial_obj)
    fruitAutoDetection.start()
