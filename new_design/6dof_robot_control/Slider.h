#ifndef __SLIDER__H
#define __SLIDER__H

#include "Arduino.h"
#include "global.h"
#include "Communication.h"

class Slider {
    private:
        int state;
        double position;
        double nextPosition;
        double numberStepToGo;
        double numberStepDone;

        const double MAX_POSITION = 28246.0;
        const double MIN_POSITION = 0.0;
        double dl = 0.1;

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
        double getNumberStepToGo();
        void setPosition(double position);
        void manualMove(double input);
        int inductiveSrDetect();
        int validatePosition(double input);
        void calculateTotalSteps();
        void setNextPosition(int newPosition);
        void updatePosition();
        void initStepDone();
        double double_abs(double value);
        bool isAutoMoveDone();
        void generalAutoMove(unsigned long &delValue, int incValue = 15, int accRate = 20);
};


#endif