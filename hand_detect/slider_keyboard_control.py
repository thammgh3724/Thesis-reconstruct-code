from pynput.keyboard import Key, Listener
import serial
import threading

serialPort = '/dev/ttyACM0'
# serialPort = 'COM4'
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

def show(key): 
    print('\nYou Entered {0}'.format(key))
    if (key == Key.left):
        serialObject.write(bytes(str("!LEFTS#"), encoding='utf-8'))
    if (key == Key.right):
        serialObject.write(bytes(str("!RIGHTS#"), encoding='utf-8'))
    if (key == Key.up):
        serialObject.write(bytes(str("!AUTOS#"), encoding='utf-8'))
    if (key == Key.down):
        serialObject.write(bytes(str("!STOPS#"), encoding='utf-8'))
    if key == Key.delete:
        # Stp listener
        return False
 
# Collect all event until released
with Listener(on_press = show) as listener:   
    read_from_serial()
    listener.join()