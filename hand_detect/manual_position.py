#!/usr/bin/env python3
import time
import serial
import threading

serialPort = '/dev/ttyACM0'
serialBaudrate = 115200

serialObject = serial.Serial(
    port=serialPort, 
    baudrate=serialBaudrate,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, 
    timeout=0.01
) 

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
# !1:1:1:1:1:1A#
# !init# , !gohome#
read_thread = threading.Thread(target=read_from_serial, daemon=True)
write_thread = threading.Thread(target=write_to_serial, daemon=True)

read_thread.start()
write_thread.start()

write_thread.join()

serialObject.close()
print("Done.")

