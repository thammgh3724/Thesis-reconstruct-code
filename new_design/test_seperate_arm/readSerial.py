import threading
import time

class ReadSerialObject(threading.Thread):
    def __init__(self, serialObj, ack_event, ack_data, ack_lock, robot_status_event, robot_status, robot_status_lock, messageHandler=None):
        threading.Thread.__init__(self)
        self.serialObj = serialObj
        self.isReading = False
        self.messageHandler = messageHandler
        self.ack_event = ack_event
        self.ack_data = ack_data
        self.ack_lock = ack_lock
        self.robot_status_event = robot_status_event
        self.robot_status = robot_status
        self.robot_status_lock = robot_status_lock

    def stop(self):
        self.isReading = False

    def run(self):
        """ Continuously read data from the serial port """
        self.isReading = True
        while self.isReading:
            if self.serialObj.in_waiting > 0:
                incomingData = self.serialObj.readline().decode('utf-8').strip()
                print(f"Received: {incomingData}")
                self.processIncomingData(incomingData)
            time.sleep(0.1)

    def processIncomingData(self, data):
        # Define the list of valid ACK messages
        # ack_messages = ["!I#", "!AH#", "!AS#", "!SS#", "!M#", "!A#", "!HA#", "!S#", "!X#"]
        
        if data.startswith("!"):
            self.ack_event.set()
            print(f"Data received: {data}")
        elif data.startswith("@"):
            with self.ack_lock:
                if self.ack_data[0] == "":
                    # Signal ACK received by setting the event
                    self.ack_event.set()
                    self.ack_data[0] = data
                    print(f"ACK: {self.ack_data[0]}")
                else:
                    # already have an ack signal need to be consumed
                    print(f"passby ack received: {data}")
        elif data.startswith("$"):
            with self.robot_status_lock:
                if self.robot_status[0] == "":
                    # Signal ACK received by setting the event
                    self.robot_status_event.set()
                    self.robot_status[0] = data
                    print(f"ROBOT STATUS received: {self.robot_status[0]}")
                else:
                    # already have an status signal need to be consumed
                    print(f"passby ROBOT STATUS received: {data}")
        else:
            print(f"Unknown message received: {data}")

if __name__ == "__main__":
    pass
