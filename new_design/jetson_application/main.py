from serialObjectSingleton import SerialSingleton
from gamepad import GamepadHandler
from writeSerial import WriteSerialObject, Message
from readSerial import ReadSerialObject
import serial
import time
import queue
import threading
import serial.tools.list_ports

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
    serial_port = '/dev/ttyACM0'
    baud_rate = 115200
    # serial_obj = SerialSingleton(serial_port, baud_rate, 0.01)
    # Waiting for the creation of serial object
    time.sleep(10)
    # Use ack_event to share ACK status between read and write threads
    ack_event = threading.Event()
    # Initialize all objects
    write_serial = WriteSerialObject(None, ack_event)
    # read_serial = ReadSerialObject(None, ack_event)
    gamepad_handler = GamepadHandler()

    # Start all threads
    gamepad_handler.start()
    # read_serial.start()
    write_serial.start()
    write_serial.ack_event.set()
    # read_serial.ack_event.set()
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
                # ack_event.clear()  # Clear event after receiving ACK
                break
            time.sleep(0.1)

        # Send !astop# to stop the arm
        stop_msg = Message("!astop#")
        write_serial.addMessage(stop_msg)
        print("Send Stop command")
        time.sleep(5)

        print("Start controlling")
        # Enter gamepad control loop
        while True:
            # Check for new input from gamepad
            print(f"Size of write queu: {write_serial.messageQueue.qsize()}")
            if gamepad_handler.newValue: 
                print("NEW VALUE")
                slider_signal = gamepad_handler.getSlidersSignal()
                buffer = gamepad_handler.getBuffer()
                mode = gamepad_handler.getMode()

                # Format the message based on mode
                if mode == "gamepad":
                    # Convert buffer to message format
                    message_content = format_gamepad_message(buffer, "M")
                else:
                    #TODO: ADD AUTO MODE AND OTHER MODE LATER
                    message_content = "AUTO MODE COMMAND"  # Placeholder for auto mode command

                # Add STOP message if buffer is all zeros
                if buffer == [0] * len(buffer):
                    message_content = "!astop#"

                print(f"Message is: {message_content}")
                message = Message(message_content)
                
                # Add message to the write queue
                write_serial.addMessage(message)
                
                gamepad_handler.newValue = False
            else: 
                print("NO NEW VALUE")
                write_serial.addMessage(Message("!astop#"))
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping threads...")
        write_serial.stop()
        # read_serial.stop()
        write_serial.join()
        # read_serial.join()
        # gamepad_handler.join()
        print("All threads stopped.")

    finally:
        serial_obj.close()

if __name__ == "__main__":
    main()
    # print(getPort())  
    # serial_port = getPort()
    # baud_rate = 115200
    # serial_obj = SerialSingleton(serial_port, baud_rate, 0.01)

    # time.sleep(10)

    # serial_obj.write(bytes(str("!init#"), encoding='utf-8'))
    # time.sleep(20)
    # ack_event = threading.Event()
    # write_serial = WriteSerialObject(None, ack_event)
    # write_serial.start()

    # write_serial.addMessage(Message("!astop#"))
    # try:
    #     while True:
    #         print(f"The size of queue: {write_serial.getQueueSize()}")
    #         print("Sending stop message")
    #         write_serial.addMessage(Message("!astop#"))
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     print("Stopping threads...")
    #     write_serial.stop()
    #     # read_serial.stop()
    #     write_serial.join()
    #     # read_serial.join()
    #     # gamepad_handler.join()
    #     print("All threads stopped.") 