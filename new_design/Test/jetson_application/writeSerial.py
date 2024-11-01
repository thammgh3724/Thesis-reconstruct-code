import threading
import queue
import time

QUEUE_MAX_SIZE = 10
MAX_RETRY = 3
TIMEOUT_MS = 100

class Message:
    def __init__(self, message, retryCount=0) -> None:
        self.message = message
        self.retryCount = retryCount

    def getMessage(self):
        return self.message

    def encodeMessage(self):
        return f"!{self.message}#\n".encode('utf-8')

    def increaseCall(self):
        self.retryCount += 1

    def setCallCounter(self, retryCount):
        self.retryCount = retryCount

    def excessCall(self, limitCall):
        return self.retryCount > limitCall
    

class WriteSerialObject(threading.Thread):
    def __init__(self, serialObj, ack_queue) -> None:
        threading.Thread.__init__(self)
        self.serialObj = serialObj
        self.messageQueue = queue.Queue(maxsize=QUEUE_MAX_SIZE)
        self.lastSentMessage = None
        self.retryCount = 0
        self.isRunning = False
        self.ack_queue = ack_queue

    def addMessage(self, message: 'Message'):
        if not self.messageQueue.full():
            self.messageQueue.put(message)
        else:
            print("Queue is full, unable to add message")

    def stop(self):
        self.isRunning = False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            if not self.messageQueue.empty():
                currentMessage = self.messageQueue.queue[0]
                
                # Send only if message is new or requires retry
                if currentMessage != self.lastSentMessage:
                    self.serialObj.write(currentMessage.encodeMessage())
                    self.lastSentMessage = currentMessage
                    print(f"Sent message: {currentMessage.getMessage()}")
                
                # Wait for ACK or retry
                ack = self.waitForAck()
                if not ack:
                    time.sleep(TIMEOUT_MS / 1000.0)
                    self.retryCount += 1
                    if self.retryCount >= MAX_RETRY:
                        print(f"Failed to send message after {MAX_RETRY} retries, dropping message")
                        self.messageQueue.get()
                        self.retryCount = 0
                    else:
                        print(f"Retrying to send message: {currentMessage.getMessage()}")
                else:
                    self.messageQueue.get()
                    self.retryCount = 0
                    print("ACK received, moving to next message")
            else:
                time.sleep(0.1)

    def waitForAck(self):
        try:
            ack = self.ack_queue.get(timeout=TIMEOUT_MS / 1000.0)
            return ack
        except queue.Empty:
            return False
        
if __name__ == "__main__":
    pass