from serialObjectSingleton import SerialSingleton
from gamepad import GamepadHandler
from writeSerial import WriteSerialObject, Message
from readSerial import ReadSerialObject
import serial
import time
import queue

'''
Function getPort() use for finding and get arduino port's name
'''
def getPort():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in p.description:
            return p.name
    raise IOError("Arduino port not found!!!")

def format_gamepad_message(buffer, mode_char):
    # Format the buffer into the required message format !x:x:x:x:x:x#
    return f"!{':'.join(map(str, buffer))}{mode_char}#"

def main():
    serial_port = getPort()
    baud_rate = 115200
    serial_obj = SerialSingleton(serial_port, baud_rate, 0.01)
    
    ack_queue = queue.Queue(maxsize=1)
    
    # Initialize all objects
    write_serial = WriteSerialObject(serial_obj, ack_queue)
    read_serial = ReadSerialObject(serial_obj, ack_queue)
    gamepad_handler = GamepadHandler()

    # Start all threads
    write_serial.start()
    read_serial.start()
    gamepad_handler.start()

    try:
        #Send !init# to initialize the arm
        init_message = Message("!init#")
        write_serial.addMessage(init_message)
        print("Sent initialization message: !init#")

        # Wait for ACK I!# from Arduino
        while True:
            if not ack_queue.empty():
                ack = ack_queue.get()
                if ack:
                    print("Initialization ACK received: I!#")
                    break
            time.sleep(0.1)

        # Enter gamepad control loop
        while True:
            # Check for new input from gamepad
            if gamepad_handler.newValue:
                buffer = gamepad_handler.getBuffer()
                mode = gamepad_handler.getMode()

                # Format the message based on mode
                if mode == "gamepad":
                    # Convert buffer to message format
                    message_content = format_gamepad_message(buffer, "A")
                else:
                    message_content = "AUTO MODE COMMAND"  # Placeholder for auto mode command

                # Add STOP message if buffer is all zeros
                if buffer == [0] * len(buffer):
                    message_content = "!STOP#"

                message = Message(message_content)
                
                # Add message to the write queue
                write_serial.addMessage(message)
                
                gamepad_handler.newValue = False
            
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
