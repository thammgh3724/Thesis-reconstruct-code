import threading
import queue
import time
import copy

QUEUE_MAX_SIZE = 10
MAX_RETRY = 3
TIMEOUT_MS = 1000

class Message:
    def __init__(self, message, retryCount=0) -> None:
        self.message = message
        self.retryCount = retryCount

    def getMessage(self):
        return self.message

    def encodeMessage(self):
        return f"{self.message}".encode('utf-8')

    def increaseCall(self):
        self.retryCount += 1

    def compareMessage(self, messageObj):
        return self.getMessage() == messageObj.getMessage()

    def setCallCounter(self, retryCount):
        self.retryCount = retryCount

    def excessCall(self, limitCall):
        return self.retryCount > limitCall
    

class WriteSerialObject(threading.Thread):
    def __init__(self, serialObj, ack_event) -> None:
        threading.Thread.__init__(self)
        self.serialObj = serialObj
        self.messageQueue = queue.Queue(maxsize=QUEUE_MAX_SIZE)
        self.lastSentMessage = Message("!#")  # Placeholder for last sent message
        self.retryCount = 0
        self.isRunning = False
        self.ack_event = ack_event

    def getQueueSize(self):
        return self.messageQueue.qsize()

    def addMessage(self, message: 'Message'):
        # Only add if queue is not full and message is different from last sent
        if not self.messageQueue.full():
            if not self.lastSentMessage.compareMessage(message):
                self.messageQueue.put(message)
        else:
            print("Queue is full, unable to add message")

    def stop(self):
        self.isRunning = False

    def run(self):
        self.ack_event.set()  # Set ACK initially to prevent stalling
        self.isRunning = True
        while self.isRunning:
            if not self.messageQueue.empty():
                currentMessage = self.messageQueue.queue[0]  # Get the top message
                
                if not self.lastSentMessage.compareMessage(currentMessage):
                    self.serialObj.write(currentMessage.encodeMessage())
                    self.lastSentMessage = copy.deepcopy(currentMessage)

                # Remove the message from the queue after sending or skipping
                self.messageQueue.get()
                
                # Placeholder for ACK and retry logic
                # You can implement ACK wait and retry logic here if needed
            else:
                time.sleep(0.1)  # Sleep if queue is empty to avoid busy-waiting

if __name__ == "__main__":
    pass
