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
        double previousPosition;
        double numberStepToGo;
        double numberStepDone;

        bool PULstat = 0;
        int PUL_PINS = SLIDER_PUL;
        int DIR_PINS = SLIDER_DIR;

        Sender* sender;

        public:
        
        const double MAX_POSITION = 5000.0;
        const double MIN_POSITION = 0.0;
        double dl = 0.1;
        // variables for specific use case move

        // classify auto moving
        bool isHomeMove = true;
        bool isLeftMove = true;
        bool isRightMove = true;
    
    public:
        Slider();
        ~Slider();
        int onStart();
        int getCurrentState();
        void setState(int state);
        double getCurrentPosition();
        double getNumberStepToGo();
        double getNumberStepDone();
        void setPosition(double position);
        void manualMove(double input);
        int inductiveSrDetect();
        int validatePosition(double input);
        void calculateTotalSteps();
        void setNextPosition(double newPosition);
        double getNextPosition();
        double getPreviousPosition();
        void setPreviousPosition(double prevPosition);
        void updatePosition();
        void initStepDone();
        double double_abs(double value);
        bool isAutoMoveDone();
        void generalAutoMove(unsigned long &delValue, int incValue = 15, int accRate = 20);
};


#endif