#ifndef __TIMER__H
#define __TIMER__H

#include "Arduino.h"
#include "global.h"

class Timer {
    private:
        unsigned long long previousTime;
        unsigned long long currentTime;
        unsigned long timeout;

    public:
        Timer();
        ~Timer();
        void setLoopAction(unsigned long timeout, unsigned long long currentTime);
        bool checkTimeoutAction();
};

#endif