#ifndef __SYSTEM__H
#define __SYSTEM__H

#include "Arduino.h"
#include "global.h"
#include "Slider.h"
#include "Arm.h"
#include "Gripper.h"
#include "Communication.h"
#include "Timer.h"

class System {
    private:
        int state;
        int nextAction;
        int nextArmAction;
        int nextSliderAction;
        int nextGripperAction; 
        Slider* slider1;
        Arm* arm;
        Gripper* gripper; 
        Listener* listener;
        Sender* sender;
        Timer* timer_arm[6]; // 6 timer for arm to take loop action for 6 joint
                             // can creat more
        Timer* timer_slider;
        Timer* timer_gripper; 

    public:
        System();
        ~System();
        void communicate();
        void arm_fsm();
        void slider_fsm();
        void gripper_fsm(); 
        void gripper_setup(); 

    private: 
        // attributes as static variables using for store array
        double gamepad_data[6];
        double nextPosition_data[6];
        double model_data[2];
        double output_slider_manual;
        double output_slider_auto;

        // Gripper data 
        double output_gripper; 

    private:
        void distributeAction();
};


#endif