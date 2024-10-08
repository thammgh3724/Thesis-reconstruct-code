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
        self.ack_event.set()
        self.isReading = True
        while self.isReading:
            if self.serialObj.in_waiting > 0:
                incomingData = self.serialObj.readline().decode('utf-8').strip()
                print(f"Received: {incomingData}")
                self.processIncomingData(incomingData)
            time.sleep(0.1)

    def processIncomingData(self, data):
        ack_messages = ["I!#", "IM#", "IA#", "IHA#", "ISL#", "IAS#"]
        
        if data in ack_messages:
            print(f"ACK received: {data}")
            # Signal ACK received
        else:
            print(f"Unknown message received: {data}")


if __name__ == "__main__":
    pass
