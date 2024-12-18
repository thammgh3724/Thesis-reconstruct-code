from serialObjectSingleton import SerialSingleton
from gamepad import GamepadHandler
from handDetectv2 import HandDetectHandler
from writeSerial import WriteSerialObject, Message
from readSerial import ReadSerialObject
import serial
import time
import queue
import threading
import serial.tools.list_ports

# Function to find and return Arduino's port name
def getPort():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in p.description:
            return p.name
    raise IOError("Arduino port not found!!!")

# Function to format the message from the gamepad buffer
def format_gamepad_message(buffer, mode_char):
    return f"!{':'.join(map(str, buffer))}{mode_char}#"

def main():
    serial_port = '/dev/ttyACM0'
    baud_rate = 115200
    serial_obj = SerialSingleton(serial_port, baud_rate, 0.01)
    
    # Wait for the creation of the serial object to stabilize the connection
    time.sleep(10)
    
    # Use ack_event to share ACK status between read and write threads
    ack_event = threading.Event()
    
    # Initialize all objects
    write_serial = WriteSerialObject(serial_obj, ack_event)
    read_serial = ReadSerialObject(serial_obj, ack_event)
    gamepad_handler = GamepadHandler()
    slider_serial = WriteSerialObject(serial_obj, ack_event)
    hand_detect_handler = HandDetectHandler()

    # Start all threads
    gamepad_handler.start()
    read_serial.start()
    write_serial.start()

    # Timer for STOP signal debounce
    last_stop_time = 0
    slider_last_time = 0
    STOP_INTERVAL = 1  # Time interval to send STOP signal (in seconds)
    
    # Check if hand detection thread started
    hand_detect_started = False
    try:
        # Send !init# to initialize the robotic arm
        init_message = Message("!init#")
        write_serial.addMessage(init_message)
        print("Sent initialization message: !init#")

        # Wait for ACK I!# from Arduino
        while True:
            if ack_event.is_set():
                print("Initialization ACK received: I!#")
                ack_event.clear()  # Reset ACK event for next message
                break
            time.sleep(0.1)  # Short delay to avoid busy-waiting

        print("Start controlling")

        # Main control loop
        while True:
            mode = gamepad_handler.getMode()

            # SYSTEM MODE: GAMEPAD
            if mode == "gamepad":
                hand_detect_started = False
                # Process gamepad inputs
                if gamepad_handler.newValue: 
                    # Retrieve buffer and sliders signals
                    slider_signal = gamepad_handler.getSlidersSignal()
                    buffer = gamepad_handler.getBuffer()

                    # Format the message based on mode
                    message_content = format_gamepad_message(buffer, "M")

                    # If the buffer contains all zeros, send a STOP signal
                    if buffer == [0] * len(buffer):
                        current_time = time.time()
                        if current_time - last_stop_time > STOP_INTERVAL:
                            write_serial.addMessage(Message("!0:0:0:0:0:0M#"))
                            last_stop_time = current_time
                    else:
                        message = Message(message_content)
                        write_serial.addMessage(message)

                    # Need to consider creating a separate thread for sliders.
                    if slider_signal == [0] * len(slider_signal):
                        current_time = time.time()
                        if current_time - slider_last_time > STOP_INTERVAL:
                            slider_serial.addMessage(Message("!sstop#"))
                            slider_last_time = current_time
                    else: 
                        slideMsg = Message(format_gamepad_message(slider_signal, "S"))
                        slider_serial.addMessage(slideMsg)
                else:
                    # If no new value, check if we need to send STOP
                    current_time = time.time()
                    if current_time - last_stop_time > STOP_INTERVAL:
                        write_serial.addMessage(Message("!0:0:0:0:0:0M#"))
                        slider_serial.addMessage(Message("!sstop#"))
                        last_stop_time = current_time
            
            # SYSTEM MODE: HAND DETECTION
            elif mode == "hand_detect":

                if not hand_detect_started:
                    print("Switching to hand_detect mode")
                    hand_detect_handler.start()  # Start the hand detect thread
                    time.sleep(10)
                    hand_detect_started = True

                # Check if hand_detect_handler has detected a hand position
                if hand_detect_handler.hand_position:
                    # Get the hand position
                    x_center = round(hand_detect_handler.hand_position[0][0].item(), 5)
                    y_center = round(hand_detect_handler.hand_position[0][1].item(), 5)
                    
                    message_content = f"!{round(x_center, 5)}:{round(y_center, 5)}H#\0"  # Format message
                    message = Message(message_content)
                    
                    # Add the message to the write_serial queue
                    write_serial.addMessage(message)
                    print(f"Send to write_serial queue: {message_content}")
                hand_detect_handler.hand_position = None
            elif mode == "auto":
                # TODO: Add auto mode logic here
                # Placeholder for auto mode command
                hand_detect_started = False
                hand_detect_handler.stop()
                print("Auto mode")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping threads...")
        write_serial.stop()
        read_serial.stop()
        write_serial.join()
        read_serial.join()
        gamepad_handler.join()
        print("All threads stopped.")

    finally:
        serial_obj.close()

if __name__ == "__main__":
    main()
