import threading
import time

class ReadSerialObject(threading.Thread):
    def __init__(self, serialObj, ack_event, messageHandler=None):
        threading.Thread.__init__(self)
        self.serialObj = serialObj
        self.isReading = False
        self.messageHandler = messageHandler
        self.ack_event = ack_event

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
        #ack_messages = ["!I#", "!AH#", "!AS#", "!SS#", "!M#", "!A#", "!HA#", "!S#", "!X#"]
        
        if data.startswith("!"):
            print(f"ACK received: {data}")
            # Signal ACK received by setting the event
            self.ack_event.set()
        else:
            print(f"Unknown message received: {data}")

if __name__ == "__main__":
    pass
