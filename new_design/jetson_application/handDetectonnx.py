import threading
import time
import cv2
import numpy as np
import camera_var
import onnxruntime as ort

class HandDetectHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.model = self.load_model("best.onnx")  # Load the ONNX model
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.hand_position = None
        self.cap = None

    def load_model(self, model_path):
        return ort.InferenceSession(model_path)

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

    def preprocess_image(self, frame):
        img = cv2.resize(frame, (640, 640))  # Resize to match model input
        img = img.transpose((2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0).astype(np.float32)  # Add batch dimension
        img /= 255.0  # Normalize to [0, 1]
        return img

    def cam_proc(self):
        object_positions = []
        accumulate_count = 0

        while not self.stop_event.is_set() and self.pause_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                break

            # Undistort the frame using calibration values
            frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)

            # Preprocess the image
            input_image = self.preprocess_image(frame)

            # Run inference
            outputs = self.model.run(None, {self.model.get_inputs()[0].name: input_image})

            # Assuming the output format is as expected for YOLO (modify as necessary)
            current_positions = []
            for output in outputs :
                    print(output)
                    print("================================================")
                    # x_center, y_center, width, height, confidence, class_id = box
                    # if confidence > 0.5:  # Only consider detections with a certain confidence
                    #     x_min = int(x_center - (width / 2))
                    #     y_min = int(y_center - (height / 2))
                    #     x_max = int(x_center + (width / 2))
                    #     y_max = int(y_center + (height / 2))
                    #     current_positions.append((x_min, y_min, x_max, y_max, confidence, class_id))
                    #     cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                    #     cv2.putText(frame, f'Class {int(class_id)}: {confidence:.2f}', 
                    #     (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


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

            cv2.imshow("YOLOv5 ONNX Real-Time", frame)
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
