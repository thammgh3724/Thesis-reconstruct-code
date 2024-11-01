import threading
import time
import cv2
import numpy as np
import camera_var
from ultralytics import YOLO

class HandDetectHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.model = YOLO("best.engine", task='detect')  # Load YOLO model for hand detection
        self.stop_event = threading.Event()  # Event to control stopping/halting the thread
        self.pause_event = threading.Event()  # Event to control pausing hand detection
        self.hand_position = None  # Placeholder to store hand position
        self.cap = None  # VideoCapture object to be opened when the thread runs

    def stop(self):
        # Set the event to fully stop the thread
        self.stop_event.set()
        self.pause_event.set()  # Resume if paused to allow full exit from the loop

    def pause(self):
        # Clear event to pause hand detection
        self.pause_event.clear()

    def resume(self):
        # Set event to continue hand detection
        self.pause_event.set()

    def run(self):
        self.stop_event.clear()
        self.pause_event.set()  # Begin with running state
        self.cap = cv2.VideoCapture(cv2.CAP_V4L2)  # Open camera when the thread runs

        while not self.stop_event.is_set():
            if self.pause_event.is_set():  # Only run if not paused
                hand_pos = self.cam_proc()
                if hand_pos:
                    self.hand_position = hand_pos  # Update hand position
                    print(hand_pos)
                time.sleep(0.3)  # Adjust delay as needed
            else:
                time.sleep(0.1)  # Wait and check pause state again

        if self.cap.isOpened():
            self.cap.release()  # Release camera when thread exits
        cv2.destroyAllWindows()

    def cam_proc(self):
        """Process camera frames for hand detection and stabilize position"""
        object_positions = []
        accumulate_count = 0

        while not self.stop_event.is_set() and self.pause_event.is_set():
            time.sleep(0.2)
            ret, frame = self.cap.read()

            if not ret:
                break

            # Perform undistortion on frame using calibration values
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

            # Stabilize detection by accumulating across multiple frames
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

                # Return hand position if stable for 3 consecutive frames
                if accumulate_count >= 3:
                    accumulate_count = 0
                    return object_positions  # Return stabilized hand position

            # Display frame with bounding boxes
            cv2.imshow("YOLOv8 Real-Time", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        return None  # Return None if exiting the loop


if __name__ == "__main__":
    handdt = HandDetectHandler()
    handdt.start()

    # # Example: Pause and resume hand detection
    # time.sleep(3)
    # handdt.pause()  # Pause
    # time.sleep(5)
    # handdt.resume()  # Resume

    # # Fully stop hand detection
    # time.sleep(5)
    # handdt.stop()
    # handdt.join()  # Wait for thread to fully exit
