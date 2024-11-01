#!/usr/bin/env python3
import time
import serial
import threading
from ultralytics import YOLO
import cv2
import numpy as np     
import camera_var

serialPort = '/dev/ttyACM0'
serialBaudrate = 115200
model = YOLO("best.pt")

serialObject = serial.Serial(
    port=serialPort, 
    baudrate=serialBaudrate,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, 
    timeout=0.01
) 

def cam_proc():
    cap = cv2.VideoCapture(0)

    object_positions = []
    accumulate_count = 0

    while True:
        time.sleep(0.03)
        ret, frame = cap.read()

        if not ret:
            break

        # Predict from opencv frame
        frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
        results = model(frame)
        
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
            if accumulate_count >= 6:
                # print(f"Object positions stabilized at: {object_positions}")
                return object_positions
        cv2.imshow("YOLOv8 Real-Time", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def read_from_serial():
    while True:
        if serialObject.in_waiting > 0:
            line = serialObject.readline().decode('utf-8').strip()
            if line:
                if line.startswith('!'):
                    print(f"\nRead from Serial: {line}")

def write_to_serial():
    while True:
        user_input = input("Input (or 'exit' to exit): ")
        if user_input.lower() == 'exit':
            break
        serialObject.write(bytes(str(user_input), encoding='utf-8'))
        print(f"Send Serial: {user_input}")

def ai_write_to_serial():
    time.sleep(1)
    init = "!init#"
    serialObject.write(bytes(str(init), encoding='utf-8'))
    print(f"Send Serial {init}")
    # while True: 
    #     try: 
    #         hand_pos = cam_proc()                                   # Camera processing to return (x_center, y_center)
    #         X_center = round(hand_pos[0][0].item(), 5)              # X_center
    #         Y_center = round(hand_pos[0][1].item(), 5)              # Y_center
    #         message = f"!{X_center}:{Y_center}H#"                   # Message to send to arduino
    #         serialObject.write(bytes(str(message), encoding="utf-8"))    
    #         print(f"Send Serial: {message}")
    #         time.sleep(2)
    #     except:                                                     # If the camera stop working, the loop will end
    #         serialObject.write(bytes(str("Invalid"), encoding="utf-8"))
    #         print("Send Serial: Invalid")
    #         break
# !1:1:1:1:1:1A#
# !init# , !gohome#
read_thread = threading.Thread(target=read_from_serial, daemon=True)
write_thread = threading.Thread(target=ai_write_to_serial, daemon=True)

read_thread.start()
write_thread.start()

write_thread.join()

serialObject.close()
print("Done.")

