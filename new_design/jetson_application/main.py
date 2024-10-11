from serialObjectSingleton import SerialSingleton
from gamepad import GamepadHandler
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
    # Wait for the creation of serial object
    time.sleep(10)
    # Use ack_event to share ACK status between read and write threads
    ack_event = threading.Event()
    
    # Initialize all objects
    write_serial = WriteSerialObject(serial_obj, ack_event)
    read_serial = ReadSerialObject(serial_obj, ack_event)
    gamepad_handler = GamepadHandler()

    # Start all threads
    gamepad_handler.start()
    read_serial.start()
    write_serial.start()

    write_serial.ack_event.set()
    read_serial.ack_event.set()

    # Timer for STOP signal debounce
    last_stop_time = 0
    STOP_INTERVAL = 1  # Time interval to send STOP signal (in seconds)

    try:
        # Send !init# to initialize the arm
        init_message = Message("!init#")
        write_serial.addMessage(init_message)
        print("Sent initialization message: !init#")
        time.sleep(10)

        # Wait for ACK I!# from Arduino
        while True:
            if ack_event.is_set():
                print("Initialization ACK received: I!#")
                break
            time.sleep(0.1)

        print("Start controlling")

        # Main control loop
        while True:
            mode = gamepad_handler.getMode()

            if mode == "gamepad":
                # Process gamepad inputs
                if gamepad_handler.newValue: 
                    # Retrieve buffer and sliders signals
                    slider_signal = gamepad_handler.getSlidersSignal()
                    buffer = gamepad_handler.getBuffer()

                    # Format the message based on mode
                    message_content = format_gamepad_message(buffer, "M")

                    # If buffer contains all zeros, we send a STOP signal
                    if buffer == [0] * len(buffer):
                        current_time = time.time()
                        if current_time - last_stop_time > STOP_INTERVAL:
                            write_serial.addMessage(Message("!astop#"))
                            last_stop_time = current_time
                    else:
                        message = Message(message_content)
                        write_serial.addMessage(message)

                    # Need time considering creating a thread for slider.
                    if slider_signal != [0] * len(slider_signal):
                        slideMsg = Message(format_gamepad_message(slider_signal, "S"))
                        write_serial.addMessage(slideMsg)
                else:
                    # If no new value, check if we need to send STOP
                    current_time = time.time()
                    if current_time - last_stop_time > STOP_INTERVAL:
                        write_serial.addMessage(Message("!astop#"))
                        last_stop_time = current_time

            elif mode == "auto":
                # TODO: Add auto mode logic here
                # Placeholder for auto mode command
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
