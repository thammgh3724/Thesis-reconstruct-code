#ifndef __SYSTEM__H
#define __SYSTEM__H

#include "Arduino.h"
#include "global.h"
#include "Slider.h"
#include "Arm.h"
#include "Communication.h"
#include "Timer.h"

class System {
    private:
        int state;
        int nextAction;
        int nextArmAction;
        int nextSliderAction;
        Slider* slider1;
        Arm* arm;
        Listener* listener;
        Sender* sender;
        Timer* timer_arm[6]; // 6 timer for arm to take loop action for 6 joint
                             // can creat more
        Timer* timer_slider;

    public:
        System();
        ~System();
        void communicate();
        void arm_fsm();
        void slider_fsm();

    private: 
        // attributes as static variables using for store array
        double output_arm6_mannual_1[6];
        double output_arm6_mannual_2[6];
        double output_arm6_auto_1[6];
        double output_arm6_auto_2[6];
        double output_arm6_auto_3[6];
        double output_slider_manual;
        double output_slider_auto;
        double output_arm2[2];

    private:
        void distributeAction();
};


#endif