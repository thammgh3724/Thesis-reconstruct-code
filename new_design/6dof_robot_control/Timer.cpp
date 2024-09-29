#include "Timer.h"

Timer::Timer(){
    this->previousTime = 0;
    this->currentTime = 0;
    this->timeout = 0;
}

Timer::~Timer(){};

void Timer::setLoopAction(unsigned long timeout, unsigned long long currentTime){
    this->timeout = timeout;
    this->previousTime = currentTime;
    this->currentTime = currentTime;
}

bool Timer::checkTimeoutAction(){
    this->currentTime = micros();
    if(this->currentTime - this->previousTime >= this->timeout){
        this->previousTime = this->currentTime;
        return 1;
    }
    return 0;
}