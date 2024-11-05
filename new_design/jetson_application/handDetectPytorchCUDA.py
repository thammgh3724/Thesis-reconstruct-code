import time
import cv2
import numpy as np
import newCamVar
import threading
import torch


class HandDetectHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model("best4.pt")
        self.isRunning = False
        self.hand_position = None  # Placeholder to store hand position

    def load_model(self, model_path):
        # Load the PyTorch model and move it to the GPU if available
        model = torch.load(model_path, map_location=self.device)
        model.eval()
        return model

    def stop(self):
        self.isRunning = False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            # Call the camera processing function to get hand position
            hand_pos = self.cam_proc()
            if hand_pos:
                self.hand_position = hand_pos  # Update the hand position
                print(hand_pos)
                time.sleep(0.3)  # Adjust delay as needed

    def preprocess_frame(self, frame):
        """Preprocess frame for PyTorch model input."""
        img = cv2.resize(frame, (640, 640))
        img = img.transpose((2, 0, 1))  # Change to CHW format
        img = np.expand_dims(img, axis=0).astype(np.float32) / 255.0  # Normalize to [0, 1]
        return torch.from_numpy(img).to(self.device)

    def cam_proc(self):
        """Camera processing to detect hand with position stabilization"""
        pipeline = (
            "v4l2src device=/dev/video1 ! "
            "video/x-raw, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "appsink"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        while not cap.isOpened():
            print("cap in normal")
            cap = cv2.VideoCapture(0)
            
        object_positions = []
        accumulate_count = 0

        while self.isRunning:
            time.sleep(0.2)
            ret, frame = cap.read()

            if not ret:
                break

            # Undistort the frame using calibration values
            frame = cv2.undistort(frame, newCamVar.K_array, newCamVar.Dis_array, None, newCamVar.New_array)
            
            # Preprocess the frame and perform inference
            input_tensor = self.preprocess_frame(frame)
            with torch.no_grad():
                outputs = self.model(input_tensor)[0]  # Assuming YOLO model returns detections in first output

            # Process model outputs
            current_positions = []
            for box in outputs:
                x_min, y_min, x_max, y_max, confidence = box[:5]  # Adjust based on output structure
                if confidence > 0.5:  # Confidence threshold
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
                    accumulate_count = 0
                    time.sleep(0.3)
                    return object_positions  # Return stabilized hand position

            # Display the frame with bounding boxes
            cv2.imshow("YOLOv5 CUDA Real-Time", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        return None  # Return None if the loop ends


if __name__ == "__main__":
    handdt = HandDetectHandler()
    handdt.start()
