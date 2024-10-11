#ifndef __SLIDER__H
#define __SLIDER__H

#include "Arduino.h"
#include "global.h"

class Slider {
    private:
        int state;
        double position;
        const double MAX_POSITION = 28246.0;
        const double MIN_POSITION = 0.0;
        bool PULstat = 0;
        int PUL_PINS = SLIDER_PUL;
        int DIR_PINS = SLIDER_DIR; 
    
    public:
        Slider();
        ~Slider();
        int onStart();
        int getCurrentState();
        void setState(int state);
        double getCurrentPosition();
        void setPosition(double position);
        void doAction();
        void manualMove(double input);
        int inductiveSrDetect();
        int validatePosition(double input);
        void generalAutoMove(uint8_t DIR, int totSteps, int delValue = 400, int incValue = 15, int accRate = 20);
};


#endif