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
        self.current_frame = None  # To store the current frame for display
        self.cap = cv2.VideoCapture(cv2.CAP_V4L2)

    def stop(self):
        self.isRunning = False
        self.cap.release()

    def run(self):
        self.isRunning = True
        object_positions = []
        accumulate_count = 0

        while self.isRunning:
            time.sleep(0.2)  # Adjust delay as needed
            ret, frame = self.cap.read()

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

                # If the hand position has been stable for 3 consecutive frames, update position
                if accumulate_count >= 3:
                    self.hand_position = object_positions  # Update the stabilized hand position
                    print(self.hand_position)

            # Update the current frame for display in a separate thread
            self.current_frame = frame

    def get_frame(self):
        """Retrieve the current frame for display."""
        return self.current_frame

def display_video(handler):
    """Display video in a separate thread."""
    while handler.isRunning:
        frame = handler.get_frame()
        if frame is not None:
            cv2.imshow("YOLOv8 Real-Time", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            handler.stop()  # Stop the handler thread if 'q' is pressed
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    handdt = HandDetectHandler()
    handdt.start()

    # Start the display thread
    display_thread = threading.Thread(target=display_video, args=(handdt,))
    display_thread.start()

    # Wait for the threads to finish
    display_thread.join()
    handdt.join()
