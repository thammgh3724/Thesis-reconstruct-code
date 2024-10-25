#ifndef __TIMER__H
#define __TIMER__H

#include "Arduino.h"
#include "global.h"

class Timer {
    private:
        unsigned long long previousTime;
        unsigned long long currentTime;

    public:
        Timer();
        ~Timer();
        void setLoopAction(unsigned long timeout, unsigned long long currentTime);
        bool checkTimeoutAction();
        unsigned long timeout;
};

#endif