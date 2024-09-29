#ifndef __SLIDER__H
#define __SLIDER__H

#include "Arduino.h"
#include "global.h"

class Slider {
    private:
        int state;
        double position;
        const double MAX_POSITION = 180.0;
        const double MIN_POSITION = 0.0;
    
    public:
        Slider();
        ~Slider();
        void onStart();
        int getCurrentState();
        void setState(int state);
        double getCurrentPosition();
        void setPosition(double position);
        void doAction();

};


#endif