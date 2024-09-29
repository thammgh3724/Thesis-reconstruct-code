#include "System.h"

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
        this->arm->onStart();
        this->arm->setState(HOME);
        break;
    case HOME:
        // waiting new action
        switch (this->nextArmAction)
        {
        case ARM_AUTO_MOVE_POSITION_ACTION:
            this->arm->setState(GENERAL_AUTO_MOVING);
            this->arm->calculateTotalSteps(this->output_arm6_auto_2, this->output_arm6_auto_1, this->output_arm6_auto_3);
            if(this->arm->validateJoint(this->output_arm6_auto_3) == 0){
                //can move
                this->arm->setState(GENERAL_AUTO_MOVING);
                for(int i = 0; i < 6; i++){
                    this->output_arm6_auto_3[i] = 0.0;
                    this->timer_arm[i]->setLoopAction(4000, micros()); //int delValue = 4000
                }
            }
            break;
        case ARM_AUTO_MOVE_DETECT_HAND_ACTION:
            this->arm->setState(DETECT_HAND_AUTO_MOVING);
            break;
        case ARM_MANUAL_MOVE_DISTANCE_ACTION:
            this->arm->setState(MANUAL_MOVING);
            this->timer_arm[0]->setLoopAction(4000, micros()); //int delValue = 4000
            break;
        default:
            break;
        }
        break;
    case MANUAL_MOVING:
        switch (this->nextArmAction)
        {
        case ARM_MANUAL_MOVE_DISTANCE_ACTION:
            this->arm->setState(MANUAL_MOVING);
            if(this->timer_arm[0]->checkTimeoutAction()) {
                this->arm->manualMove(this->output_arm6_mannual_1);
            }
            break;
        case ARM_STOP_ACTION:
            this->arm->setState(STOP);
            break;
        default:
            break;
        }
        break;
    case GENERAL_AUTO_MOVING:
        switch (this->nextArmAction)
        {
        case ARM_AUTO_MOVE_POSITION_ACTION:
            this->arm->setState(GENERAL_AUTO_MOVING);
            for(int i = 0; i < 6; i++){
                if(this->timer_arm[i]->checkTimeoutAction()) {
                    this->arm->generalAutoMove(i, this->output_arm6_auto_2, this->output_arm6_auto_3, this->timer_arm[i]->timeout);
                }
            }
            break;
        case ARM_STOP_ACTION:
            this->arm->setState(STOP);
            break;
        default:
            break;
        }
        break;
    case STOP:
        // waiting new action
        switch (this->nextArmAction)
        {
        case ARM_AUTO_MOVE_POSITION_ACTION:
            this->arm->setState(GENERAL_AUTO_MOVING);
            break;
        case ARM_AUTO_MOVE_DETECT_HAND_ACTION:
            this->arm->setState(DETECT_HAND_AUTO_MOVING);
            break;
        case ARM_MANUAL_MOVE_DISTANCE_ACTION:
            this->arm->setState(MANUAL_MOVING);
            this->timer_arm[0]->setLoopAction(4000, micros()); //int delValue = 4000
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void System::distributeAction(){
    switch (this->nextAction)
    {
    case ARM_INIT_ACTION:
        this->listener->consumeCommand(ARM_INIT_ACTION, nullptr);
        break;
    case ARM_GOHOME_ACTION:
        this->listener->consumeCommand(ARM_GOHOME_ACTION, nullptr);
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
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_AUTO_MOVE_FREE_ACTION:
        this->listener->consumeCommand(SLIDER_AUTO_MOVE_FREE_ACTION, nullptr);
        break;
    case ARM_STOP_ACTION:
        this->nextArmAction = ARM_STOP_ACTION;
        this->listener->consumeCommand(ARM_STOP_ACTION, nullptr);
        this->nextAction = NO_ACTION;
        break;
    default:
        break;
    }
}