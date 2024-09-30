#!/usr/bin/env python3
import pygame
import time
import serial
import threading
# Initialize pygame
pygame.init()

serialPort = '/dev/ttyACM0'
#serialPort = 'COM4'
serialBaudrate = 115200
ack = True
debounce = 0
stopFlag = True
idleFlag = False
# Initialize the joystick
joystick_count = pygame.joystick.get_count()

serialObject = serial.Serial(
    port=serialPort, 
    baudrate=serialBaudrate,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,  
    stopbits=serial.STOPBITS_ONE, 
    timeout=0.01
) 

time.sleep(10)
initalstring = "!init#"
serialObject.write(bytes(str(initalstring), encoding='utf-8'))

# INitialize input values
buffer = [0, 0, 0, 0, 0, 0, 0, 0]
newValue = False

def handle_button_release(button, buffer):
    if button == 0:
        print("Button A released")
        buffer[5] = 0
    elif button == 1:
        print("Button B released")
        buffer[4] = 0
    elif button == 3:
        print("Button X released")
        buffer[4] = 0
    elif button == 4:
        print("Button Y released")
        buffer[5] = 0
    elif button == 6:
        print("Left bumper released")
    elif button == 7:
        print("Right bumper released")
    elif button == 10:
        print("Back button released")
    elif button == 11:
        print("Start button released")
    elif button == 13:
        print("Left stick button released")
    elif button == 14:
        print("Right stick button released")
    elif button == 8:
        print("Left trigger (L2) released")
    elif button == 9:
        print("Right trigger (R2) released")
    else: 
        print("Helloooooo")
    
def handle_axis_motion(axis, value, buffer):
    if axis == 0:  # X-axis of the left stick
        print(f"Left stick X-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[0] = 1
        elif value > 0.9 and value <=1.1:
            buffer[0] = 2
        elif value >= -0.2 and value <= 0.2: buffer[0] = 0
    elif axis == 1:  # Y-axis of the left stick
        print(f"Left stick Y-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[1] = 1
        elif value > 0.9 and value <=1.1:
            buffer[1] = 2
        elif value >= -0.2 and value <= 0.2: buffer[1] = 0
    elif axis == 2:  # X-axis of the right stick
        print(f"Right stick X-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[3] = 1
        elif value > 0.9 and value <=1.1:
            buffer[3] = 2
        elif value >= -0.2 and value <= 0.2: buffer[3] = 0
    elif axis == 3:  # Y-axis of the right stick
        print(f"Right stick Y-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[2] = 1
        elif value > 0.9 and value <=1.1:
            buffer[2] = 2
        elif value >= -0.2 and value <= 0.2: buffer[2] = 0
    elif axis == 5:  # Left trigger (L2)
        print(f"Left trigger (L2) value: {value}")
    elif axis == 4:  # Right trigger (R2)
        print(f"Right trigger (R2) value: {value}")
    elif axis == 6:  # D-pad X-axis
        handle_dpad_x(value)
    elif axis == 7:  # D-pad Y-axis
        handle_dpad_y(value)

def handle_button_press(button, buffer):
    global debounce
    if button == 0:
        print("Button A pressed")
        buffer[5] = 1
    elif button == 1:
        print("Button B pressed")
        buffer[4] = 2
    elif button == 3:
        print("Button X pressed")
        buffer[4] = 1
    elif button == 4:
        print("Button Y pressed")
        buffer[5] = 2
    elif button == 6:
        print("Left bumper pressed")
    elif button == 7:
        print("Right bumper pressed")
        #buffer[5] = 1
    elif button == 10:
        print("Back button pressed")
    elif button == 11:
        print("Start button pressed")
    elif button == 13:
        print("Left stick button pressed")
    elif button == 14:
        print("Right stick button pressed")
    elif button == 8:
        print("Left trigger (L2) pressed")
        debounce = 1
    elif button == 9:
        print("Right trigger (R2) pressed")
        debounce = 2
    else: 
        print("Helloooooo")
    print("Buffer: ", buffer)
    
def handle_dpad_x(value):
    global debounce
    if value == 1.0:
        buffer[6] = 1
        print("D-pad right pressed")
    elif value == -1.0:
        buffer[6] = 2
        print("D-pad left pressed")
    else:
        buffer[6] = 0
        print("D-pad X-axis released")
        # debounce = 0

def handle_dpad_y(value):
    if value == 1.0:
        buffer[7] = 1
        print("D-pad down pressed")
    elif value == -1.0:
        buffer[7] = 2
        print("D-pad up pressed")
    else:
        buffer[7] = 0
        print("D-pad Y-axis released")

def read_from_serial():
    global ack
    while True:
        if serialObject.in_waiting > 0:
            line = serialObject.readline().decode('utf-8').strip()
            if line:
                if line.startswith('!'):
                    print(f"\nRead from Serial: {line}")
            if (line == "!GO MANUAL DONE" or line == "!GO SLIDE DONE"):  # Maybe bug
                print("acknowledage")
                ack = True
            elif line == "KCA\r\n": 
                print("stopped")
                ack = True
                


def write_to_serial():
    global ack
    if joystick_count > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"Joystick found: {joystick.get_name()}")
        debounce = 0
        while True:
            # pygame.event.pump()
            # choose 1 button to control slide then config "slide_signal" in the handle button funtion
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    handle_button_press(event.button, buffer)
                if event.type == pygame.JOYBUTTONUP:
                    handle_button_release(event.button, buffer)
                if event.type == pygame.JOYAXISMOTION:
                    handle_axis_motion(event.axis, event.value, buffer)
                if event.type == pygame.JOYHATMOTION:
                    x_value, y_value = event.value
                    handle_dpad_x(x_value)
                    handle_dpad_y(y_value)
            newValue = False
            if idleFlag == False:
                for b in buffer:
                    if (b != 0):
                        newValue = True
                        stopFlag = False
                        break
            if newValue == True and ack == True:  
                string = ':'.join(str(x) for x in buffer)
                string = '!' + string + 'M#'
                print('send: ', string)
                serialObject.write(bytes(str(string), encoding='utf-8'))
                ack = False
        pygame.quit()

    else:
        print("No joystick found.")

read_thread = threading.Thread(target=read_from_serial, daemon=True)
write_thread = threading.Thread(target=write_to_serial, daemon=True)

read_thread.start()
write_thread.start()

write_thread.join()

serialObject.close()
print("Done.")