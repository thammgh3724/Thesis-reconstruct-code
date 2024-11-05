import cv2
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit  # Automatically initialize CUDA context
from ctypes import c_void_p
import tensorrt as trt

# Load the TensorRT engine
def load_engine(engine_file_path):
    with open(engine_file_path, "rb") as f:
        return trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(f.read())

# Preprocess the image
def preprocess_image(frame):
    h, w = frame.shape[:2]
    # Resize to model input size (assuming YOLOv5 input size of 640x640)
    img = cv2.resize(frame, (640, 640))
    img = img.transpose((2, 0, 1))  # HWC to CHW
    img = np.expand_dims(img, axis=0).astype(np.float32)  # Add batch dimension
    img /= 255.0  # Normalize to [0, 1]
    return img

# Perform inference
def infer(engine, input_image):
    with engine.create_execution_context() as context:
        # Allocate device memory
        input_size = trt.volume(context.get_binding_shape(0)) * engine.max_batch_size
        output_size = trt.volume(context.get_binding_shape(1)) * engine.max_batch_size
        d_input = cuda.mem_alloc(input_image.nbytes)
        d_output = cuda.mem_alloc(output_size * 4)  # Assuming float output

        # Transfer data to the GPU
        cuda.memcpy_htod(d_input, input_image)

        # Run inference
        context.execute(batch_size=1, bindings=[int(d_input), int(d_output)])

        # Transfer predictions back to host
        output = np.empty(output_size, dtype=np.float32)
        cuda.memcpy_dtoh(output, d_output)

    return output

# Main function
if __name__ == "__main__":
    engine = load_engine("bestW.engine")
    cap = cv2.VideoCapture(0)  # Open the first camera

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        input_image = preprocess_image(frame)
        output = infer(engine, input_image)

        # Post-process the output (add your bounding box drawing logic here)
        # Example: loop through output and draw boxes on frame

        cv2.imshow("YOLOv5 Inference", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
