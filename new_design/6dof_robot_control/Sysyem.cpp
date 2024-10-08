#include "System.h"

int count = 1;
System::System(){
    this->state = INIT;
    this->nextAction = NO_ACTION;
    this->nextArmAction = NO_ACTION;
    this->nextSliderAction = NO_ACTION;
    this->slider1 = new Slider();
    this->arm = new Arm();
    this->listener = new Listener();
    this->sender = new Sender();
    for (int i = 0; i < 6; ++i){
        this->timer_arm[i] = new Timer();  
    }
}

System::~System(){
    delete(this->slider1);
    delete(this->arm);
    delete(this->listener);
    delete(this->sender);
    for (int i = 0; i < 6; ++i) {
        delete this->timer_arm[i];  
    }
}

void System::communicate(){
    this->listener->readData(); // read 1 char per loop, command ready if have "#"
    this->nextAction = this->listener->parseCommandToAction(); // if command ready -> read cmd and return action need to do 
    this->distributeAction(); // consume command (get needed data and reset cmd to "") and distribute action to the right object 
}

void System::arm_fsm(){
    switch (this->arm->getCurrentState())
    {
    case INIT:
        // waiting init action
        switch (this->nextArmAction)
        {
        case ARM_INIT_ACTION:
            this->arm->onStart(); // cannot interrupt
            this->arm->setState(HOME);
            #ifdef DEBUG
            this->sender->sendData("!GO STATE HOME");
            #endif
            break;
        default:
            break;
        }
        break;
    case HOME:
        // waiting new action
        #ifdef DEBUG
        if (count == 1){
            String nxt = "! next arm action :" + String(this->nextArmAction);
            count = 0;
            this->sender->sendData(nxt);
        }
        #endif
        if (this->nextArmAction == ARM_AUTO_MOVE_POSITION_ACTION){
            this->arm->calculateTotalSteps(this->output_arm6_auto_2, this->output_arm6_auto_1, this->output_arm6_auto_3);//rename fuck
            #ifdef DEBUG
            String data_print = "!Total Step: ";
            #endif
            if(this->arm->validateJoint(this->output_arm6_auto_3) == 0){
                //can move
                this->arm->setState(GENERAL_AUTO_MOVING);
                for(int i = 0; i < 6; i++){
                    this->output_arm6_auto_3[i] = 0.0;
                    this->timer_arm[i]->setLoopAction(3000, micros()); //int delValue = 4000
                    #ifdef DEBUG
                    data_print += String(this->output_arm6_auto_2[i]);
                    data_print += ":";
                    #endif
                }
                #ifdef DEBUG
                this->sender->sendData(data_print);
                this->sender->sendData("!GO AUTO POSITION");
                #endif
            }
        }
        else if (this->nextArmAction == ARM_AUTO_MOVE_DETECT_HAND_ACTION){
            this->arm->setState(DETECT_HAND_AUTO_MOVING);
        }
        else if (this->nextArmAction ==  ARM_MANUAL_MOVE_DISTANCE_ACTION){
            this->arm->setState(MANUAL_MOVING);
            this->timer_arm[0]->setLoopAction(2000, micros()); //int delValue = 4000
            #ifdef DEBUG
            this->sender->sendData("!GO MANUAL");
            #endif
        }
        else if (this->nextArmAction == ARM_STOP_ACTION){
            this->arm->setState(STOP);
            #ifdef DEBUG
            this->sender->sendData("!STOP");
            #endif
        }
        else {
        }
        #ifdef DEBUG
        if (count == 1){
            String nxt = "! next arm action :" + String(this->nextArmAction);
            count = 0;
            this->sender->sendData(nxt);
        }
        #endif
        break;
    case MANUAL_MOVING:
        if(this->nextArmAction == ARM_MANUAL_MOVE_DISTANCE_ACTION){
            this->arm->setState(MANUAL_MOVING);
            if(this->timer_arm[0]->checkTimeoutAction()) {
                this->arm->manualMove(this->output_arm6_mannual_1);
            }
        }
        else if(this->nextArmAction == ARM_STOP_ACTION){
            this->arm->setState(STOP);
        }
        else {
        }
        break;
    case GENERAL_AUTO_MOVING:
        // #ifdef DEBUG
        //     this->sender->sendData("!in auto moving");
        // #endif

        if (this->nextArmAction == ARM_AUTO_MOVE_POSITION_ACTION)
        {
            this->arm->setState(GENERAL_AUTO_MOVING);
            for(int i = 0; i < 6; i++){
                if(this->timer_arm[i]->checkTimeoutAction()) {
                    this->arm->generalAutoMove(i, this->output_arm6_auto_2, this->output_arm6_auto_3, this->timer_arm[i]->timeout);
                }
            }
        }
        else if (this->nextArmAction == ARM_STOP_ACTION){
            this->arm->setState(STOP);
            #ifdef DEBUG
            this->sender->sendData("!STOP");
            #endif
        }
        else{
        }
        break;
    case STOP:
        // waiting new action
        if (this->nextArmAction == ARM_AUTO_MOVE_POSITION_ACTION){
            this->arm->setState(GENERAL_AUTO_MOVING);
        }
        else if (this->nextArmAction == ARM_AUTO_MOVE_DETECT_HAND_ACTION){
            this->arm->setState(DETECT_HAND_AUTO_MOVING);
        }
        else if (this->nextArmAction == ARM_MANUAL_MOVE_DISTANCE_ACTION){
            this->arm->setState(MANUAL_MOVING);
            this->timer_arm[0]->setLoopAction(2000, micros()); //int delValue = 4000
            #ifdef DEBUG
            this->sender->sendData("!GO MANUAL");
            #endif
        }
        else{
        }
        break;
    default:
        break;
    }
    #ifdef DEBUG
      String curState = "State: ";
      curState += String(this->arm->getCurrentState());
      this->sender->sendData(curState);
    #endif
}

// void System::slider_fsm(){
//     switch (this->slider1->getCurrentState())
//     {
//     case INIT:
//         // waiting init action
//         switch (this->nextSliderAction)
//         {
//         case SLIDER_INIT_ACTION:
//             this->slider1->onStart(); // cannot interrupt
//             this->slider1->setState(HOME);
//             break;
//         default:
//             break;
//         }
//         break;
//     case HOME:
//         // waiting new action
//         switch (this->nextSliderAction)
//         {
//         case SLIDER_AUTO_MOVE_FREE_ACTION:
//             if(this->slider1->validatePosition(this->output_slider_auto) == 0){
//                 //can move
//                 this->slider1->setState(GENERAL_AUTO_MOVING);
//                 this->output_slider_auto = 0.0;
//                 this->timer_slider->setLoopAction(4000, micros()); 
//             }
//             break;
//         case SLIDER_AUTO_MOVE_DETECT_HAND_ACTION:
//             this->slider1->setState(DETECT_HAND_AUTO_MOVING);
//             break;
//         case SLIDER_MANUAL_MOVE_DISTANCE_ACTION:
//             this->slider1->setState(MANUAL_MOVING);
//             this->timer_slider->setLoopAction(4000, micros()); //int delValue = 4000
//             break;
//         case SLIDER_STOP_ACTION:
//             this->slider1->setState(STOP);
//             break;
//         default:
//             break;
//         }
//         break;
//     case MANUAL_MOVING:
//         switch (this->nextSliderAction)
//         {
//         case SLIDER_MANUAL_MOVE_DISTANCE_ACTION:
//             this->slider1->setState(MANUAL_MOVING);
//             if(this->timer_slider->checkTimeoutAction()) {
//                 this->slider1->manualMove(this->output_slider_manual);
//             }
//             break;
//         case SLIDER_STOP_ACTION:
//             this->slider1->setState(STOP);
//             break;
//         default:
//             break;
//         }
//         break;
//     case GENERAL_AUTO_MOVING:
//         switch (this->nextSliderAction)
//         {
//         case SLIDER_AUTO_MOVE_FREE_ACTION:
//             this->slider1->setState(GENERAL_AUTO_MOVING);
//             if(this->timer_slider->checkTimeoutAction()) {
//                 this->slider1->generalAutoMove(i, this->output_arm6_auto_2, this->output_arm6_auto_3, this->timer_arm[i]->timeout);
//             }
//             break;
//         case SLIDER_STOP_ACTION:
//             this->slider1->setState(STOP);
//             break;
//         default:
//             break;
//         }
//         break;
//     case STOP:
//         // waiting new action
//         switch (this->nextSliderAction)
//         {
//         case SLIDER_AUTO_MOVE_FREE_ACTION:
//             this->slider1->setState(GENERAL_AUTO_MOVING);
//             break;
//         case SLIDER_AUTO_MOVE_DETECT_HAND_ACTION:
//             this->slider1->setState(DETECT_HAND_AUTO_MOVING);
//             break;
//         case SLIDER_MANUAL_MOVE_DISTANCE_ACTION:
//             this->slider1->setState(MANUAL_MOVING);
//             this->timer_slider->setLoopAction(4000, micros()); //int delValue = 4000
//             break;
//         default:
//             break;
//         }
//         break;
//     default:
//         break;
//     }
//}

void System::distributeAction(){
    switch (this->nextAction)
    {
    case INIT_ACTION:
        this->nextArmAction = ARM_INIT_ACTION;
        this->nextSliderAction = SLIDER_INIT_ACTION;
        this->listener->consumeCommand(ARM_INIT_ACTION, nullptr);
        this->listener->consumeCommand(SLIDER_INIT_ACTION,nullptr);
        this->nextAction = NO_ACTION;
        break;
    case ARM_GOHOME_ACTION:
        this->nextArmAction = ARM_GOHOME_ACTION;
        this->listener->consumeCommand(ARM_GOHOME_ACTION, nullptr);
        this->nextAction = NO_ACTION;
        break;
    case ARM_STOP_ACTION:
        this->nextArmAction = ARM_STOP_ACTION;
        this->listener->consumeCommand(ARM_STOP_ACTION, nullptr);
        this->nextAction = NO_ACTION;
        break;
    case ARM_MANUAL_MOVE_DISTANCE_ACTION:
        this->nextArmAction = ARM_MANUAL_MOVE_DISTANCE_ACTION;
        this->listener->consumeCommand(ARM_MANUAL_MOVE_DISTANCE_ACTION, this->output_arm6_mannual_1);
        this->nextAction = NO_ACTION;
        break;
    case ARM_AUTO_MOVE_POSITION_ACTION:
        this->nextArmAction = ARM_AUTO_MOVE_POSITION_ACTION;
        this->listener->consumeCommand(ARM_AUTO_MOVE_POSITION_ACTION, this->output_arm6_auto_1);
        this->nextAction = NO_ACTION;
        break;
    case ARM_AUTO_MOVE_DETECT_HAND_ACTION:
        this->nextArmAction = ARM_AUTO_MOVE_DETECT_HAND_ACTION;
        this->listener->consumeCommand(ARM_AUTO_MOVE_DETECT_HAND_ACTION, this->output_arm2);
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_MANUAL_MOVE_DISTANCE_ACTION:
        this->nextSliderAction = SLIDER_MANUAL_MOVE_DISTANCE_ACTION;
        this->listener->consumeCommand(SLIDER_MANUAL_MOVE_DISTANCE_ACTION,&this->output_slider_manual);
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_AUTO_MOVE_FREE_ACTION:
        this->nextSliderAction = SLIDER_AUTO_MOVE_FREE_ACTION;
        this->listener->consumeCommand(SLIDER_AUTO_MOVE_FREE_ACTION,&this->output_slider_auto);
        this->nextAction = NO_ACTION;
        break;
    default:
        break;
    }
}