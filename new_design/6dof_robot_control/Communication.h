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

class Sender {
    public:
        Sender();
        ~Sender();
        void sendData(String data);
        void sendACK(String ack);
};

#endif
