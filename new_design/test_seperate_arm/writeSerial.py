import threading
import queue
import time
import copy
import re

QUEUE_MAX_SIZE = 10
MAX_RETRY = 3  # Maximum number of retries if no ACK is received
TIMEOUT_MS = 1000  # Timeout for waiting for ACK (1000ms = 1 second)

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
    def __init__(self, serialObj, ack_event, ack_data, ack_lock) -> None:
        threading.Thread.__init__(self)
        self.serialObj = serialObj
        self.messageQueue = queue.Queue(maxsize=QUEUE_MAX_SIZE)
        self.lastSentMessage = Message("!#")  # Placeholder for last sent message
        self.lastSentHandPos = [None, None]   # To compare with latest returned hand pos
        self.isRunning = False
        self.ack_event = ack_event
        self.ack_data = ack_data
        self.ack_lock = ack_lock
        self.stop_signals = {
            "!astop#": 0,
            "!0:0:0:0:0:0M#": 0,
            "!sstop#": 0,
            "!gstop#": 0 
        }
        self.message_ack_map = {
            "!init#": "@I#",
            "!agohome#": "@AH#",
            "!sgohome#": "@SH#",
            "M#": "@M#",
            "A#": "@A#",
            "H#": "@HA#",
            "S#": "@SL#",
            "!astop#": "@AS#",
            "!sstop#": "@SS#",
            "!gstop#": "@GS#",
            "!gopen#": "@GO#",
            "!gclose#": "@GCLS#",
        }

        self.regex_ack_map = [
            (re.compile(r"^!\d+:\d+:\d+:\d+:\d+:\d+M#$"), "@M#"),
            (re.compile(r"^!\d+:\d+S#$"), "@S#")
        ]

    def isStopSignalLocked(self, message):
        return self.stop_signals[message] == 0

    def resetStopCounter(self, message): 
        self.stop_signals[message] = 0

    def lockStopSignal(self, message): 
        self.stop_signals[message] = 1

    def getQueueSize(self):
        return self.messageQueue.qsize()

    def addMessage(self, message: 'Message'):
        # Only add message to the queue if it's not full and is different from the last sent message
        if not self.messageQueue.full():
            if not self.lastSentMessage.compareMessage(message):
                if (message.getMessage() in self.stop_signals):
                    if self.stop_signals[message.getMessage()] == 0:
                        self.stop_signals[message.getMessage()] = 1
                        self.messageQueue.put(message)
                else: 
                    self.messageQueue.put(message)
        else:
            self.clearQueue()
            print("Queue is full, unable to add message")

    def stop(self):
        self.isRunning = False

    def clearQueue(self):
        if (self.getQueueSize() == 0):
            return
        print("CLEARING QUEUE IN PROGRESS...")
        while (self.getQueueSize() != 0):
            self.messageQueue.get()
        print("CLEARING QUEUE DONE!")

    def checkACK(self):
        with self.ack_lock:
            for last_char in self.message_ack_map:
                if (self.lastSentMessage.message.endswith(last_char) and self.ack_data[0] == self.message_ack_map[last_char] ):
                    self.ack_data[0] = ""
                    return True
                
            for regex, ack in self.regex_ack_map:
                if regex.match(self.lastSentMessage.message) and self.ack_data[0] == ack:
                    self.ack_data[0] = ""
                    return True
                
            self.ack_data[0] = ""
            print("wrong ack")
            return False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            if not self.messageQueue.empty():
                currentMessage = self.messageQueue.queue[0]  # Get the top message from the queue
                
                if not self.lastSentMessage.compareMessage(currentMessage):
                    # Send the message
                    self.serialObj.write(currentMessage.encodeMessage())
                    self.lastSentMessage = copy.deepcopy(currentMessage)
                    
                    # Wait for ACK within TIMEOUT_MS
                    ack_received = self.ack_event.wait(timeout=TIMEOUT_MS / 1000.0)

                    if not ack_received:
                        # If no ACK received, increase retry count
                        print("CASE 1: NO ACK RECEIVED")
                        currentMessage.increaseCall()
                        if currentMessage.excessCall(MAX_RETRY):
                            print(f"Failed to send message after {MAX_RETRY} retries, dropping message")
                            self.messageQueue.get()  # Remove the message after too many retries
                        else:
                            print(f"Retrying to send message: {currentMessage.getMessage()}")
                    else:
                        # If ACK is received, 
                        # + If ACK is valid, remove the message from the queue and reset retry count
                        # + else retry sending message
                        if self.checkACK():
                            self.messageQueue.get()
                            self.ack_event.clear()  # Clear ACK status for the next message
                        else: 
                            print("CASE 2: WRONG ACK RECEIVED")
                            currentMessage.increaseCall()
                            if currentMessage.excessCall(MAX_RETRY):
                                print(f"Failed to send message after {MAX_RETRY} retries, dropping message")
                                self.messageQueue.get()  # Remove the message after too many retries
                            else:
                                print(f"Retrying to send message: {currentMessage.getMessage()}")

                else:
                    # If the message is a duplicate, just remove it from the queue
                    self.messageQueue.get()
            else:
                time.sleep(0.1)  # Wait if the queue is empty to avoid busy-waiting

if __name__ == "__main__":
    pass
