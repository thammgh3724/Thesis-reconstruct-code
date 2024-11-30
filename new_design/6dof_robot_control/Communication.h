#ifndef __COMMUNICATION__H
#define __COMMUNICATION__H

#include "Arduino.h"
#include "global.h"

class Listener {
    private:
        String command;

    public:
        Listener();
        ~Listener();
        void readData();
        int parseCommandToAction();
        void consumeCommand(int action, double* output);

    private:
        bool isdataTransferring = false;
        bool isCommandReady = false;

    private:
        void getDoubleArrayData(double* output, int numDataToGet, int commandLength, char charToPassBy);
};

// class MessageQueue {
//     private:
//         const int QUEUE_SIZE = 10;
//         String queue[QUEUE_SIZE];
//         int front = 0;
//         int rear = 0;
//     public:
//         MessageQueue();
//         ~MessageQueue();
//         bool addMessage();
//         bool deleteFirstMessage();
//         String getFirstMessage();
// }

class Sender {
    public:
        Sender();
        ~Sender();
        void sendData(String data);
        void sendACK(String ack);

        void sendSystemStatus(String status);
};

#endif
