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
    this->timer_slider = new Timer();
}

System::~System(){
    delete(this->slider1);
    delete(this->arm);
    delete(this->listener);
    delete(this->sender);
    for (int i = 0; i < 6; ++i) {
        delete this->timer_arm[i];  
    }
    delete this->timer_slider;
}

void System::gripper_setup() {
    this-gripper->setupGripper(); 
}

void System::communicate(){
    this->listener->readData(); // read 1 char per loop, command ready if have "#"
    this->nextAction = this->listener->parseCommandToAction(); // if command ready -> read cmd and return action need to do 
    this->distributeAction(); // consume command (get needed data and reset cmd to "") and distribute action to the right object 
    // and send ACK here
}

void System::arm_fsm(){
    switch (this->arm->getCurrentState())
    {
    case INIT:
        // waiting init action
        if (this->nextArmAction == ARM_INIT_ACTION){
            this->arm->onStart(); // cannot interrupt
            this->nextArmAction == ARM_STOP_ACTION;
            this->arm->updateCurrentPosition();
            this->arm->setState(STOP);
            #ifdef DEBUG
            this->sender->sendData("!GO STATE STOP");
            #endif
        }
        else{
        }
        break;
    case MANUAL_MOVING:
        if(this->nextArmAction == ARM_MANUAL_MOVE_DISTANCE_ACTION){
            if(this->arm->isManualMoveDone()) {
                this->arm->updateCurrentPosition();
                this->nextArmAction = ARM_STOP_ACTION;
                this->arm->setState(STOP);
                #ifdef DEBUG
                this->sender->sendData("!GO STATE STOP");
                #endif
            }
            else {
                this->arm->setState(MANUAL_MOVING);
                for(int i = 0; i < 6; i++){
                    if(this->timer_arm[i]->checkTimeoutAction()) {
                        this->arm->manualMove(i, this->gamepad_data, this->timer_arm[i]->timeout);
                    }
                }
            }
        }
        else if(this->nextArmAction == ARM_STOP_ACTION){
            this->arm->updateCurrentPosition();
            this->arm->setState(STOP);
            #ifdef DEBUG
            this->sender->sendData("!GO STATE STOP");
            #endif
        }
        else {
        }
        break;
    case GENERAL_AUTO_MOVING:
        if (this->nextArmAction == ARM_AUTO_MOVE_POSITION_ACTION || this->nextArmAction == ARM_GOHOME_ACTION)
        {
            if(this->arm->isAutoMoveDone()) {
                this->arm->updateCurrentPosition();
                this->nextArmAction = ARM_STOP_ACTION;
                this->arm->setState(STOP);
                #ifdef DEBUG
                this->sender->sendData("!GO STATE STOP");
                #endif
            }
            else {
                this->arm->setState(GENERAL_AUTO_MOVING);
                for(int i = 0; i < 6; i++){
                    if(this->timer_arm[i]->checkTimeoutAction()) {
                        this->arm->generalAutoMove(i, this->timer_arm[i]->timeout);
                    }
                }
            }
        }
        else if (this->nextArmAction == ARM_STOP_ACTION){
            this->arm->updateCurrentPosition();
            this->arm->setState(STOP);
            #ifdef DEBUG
            this->sender->sendData("!STOP");
            #endif
        }
        else{
        }
        break;
    case DETECT_HAND_AUTO_MOVING:
        if (this->nextArmAction == ARM_AUTO_MOVE_DETECT_HAND_ACTION)
        {
            if(this->arm->isAutoMoveDone()) {
                this->arm->updateCurrentPosition();
                this->nextArmAction = ARM_AUTO_MOVE_DETECT_HAND_ACTION;
                this->arm->setState(STOP);
                #ifdef DEBUG
                this->sender->sendData("!GO STATE STOP");
                #endif
            }
            else {
                this->arm->setState(DETECT_HAND_AUTO_MOVING);
                for(int i = 0; i < 6; i++){
                    if(this->timer_arm[i]->checkTimeoutAction()) {
                        this->arm->generalAutoMove(i, this->timer_arm[i]->timeout);
                    }
                }
            }
        }
        else if (this->nextArmAction == ARM_STOP_ACTION){
            this->arm->updateCurrentPosition();
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
        if (this->nextArmAction == ARM_GOHOME_ACTION){
            this->arm->setNextPosition(this->arm->home_position);
            this->arm->setNextJoint(this->arm->home_joint);
            this->arm->calculateTotalSteps();
            double initNumberStepsDone[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            this->arm->setNumberStepDone(initNumberStepsDone);
            this->arm->initjointAutoMoveDone();
            this->arm->setState(GENERAL_AUTO_MOVING);
            for(int i = 0; i < 6; i++){
                this->timer_arm[i]->setLoopAction(3000, micros()); //int delValue = 3000
            }
            #ifdef DEBUG
            this->sender->sendData("!GO HOME");
            #endif
        }
        else if (this->nextArmAction == ARM_AUTO_MOVE_POSITION_ACTION){
            this->arm->setNextPosition(this->nextPosition_data);
            this->arm->calculateNextJoint();
            if(this->arm->validateNextJoint() == 0){
                //can move
                this->arm->calculateTotalSteps();
                double initNumberStepsDone[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                this->arm->setNumberStepDone(initNumberStepsDone);
                this->arm->initjointAutoMoveDone();
                this->arm->setState(GENERAL_AUTO_MOVING);
                for(int i = 0; i < 6; i++){
                    this->timer_arm[i]->setLoopAction(3000, micros()); //int delValue = 3000
                }
                #ifdef DEBUG
                this->sender->sendData("!GO AUTO POSITION");
                #endif
            }
        }
        else if (this->nextArmAction == ARM_AUTO_MOVE_DETECT_HAND_ACTION){
            if(this->arm->isHorizontalMove) {
                // set up horizontal move
                this->arm->calculateHorizontalNextPosition_detectHand(this->model_data);
                this->arm->calculateHorizontalNextJoint_detectHand();
                if(this->arm->validateNextJoint() == 0){
                    #ifdef DEBUG
                    this->sender->sendData("!GO HORIZONTAL");
                    #endif
                    //can move
                    this->arm->calculateTotalSteps();
                    double initNumberStepsDone[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    this->arm->setNumberStepDone(initNumberStepsDone);
                    this->arm->initjointAutoMoveDone();
                    this->arm->setState(DETECT_HAND_AUTO_MOVING);
                    for(int i = 0; i < 6; i++){
                        this->timer_arm[i]->setLoopAction(3000, micros()); //int delValue = 3000
                    }
                }
                this->arm->isHorizontalMove = false;
            } else if(this->arm->isLengthwiseMove) {
                // set up lengthwise move
                this->arm->calculateLengthwiseNextPosition_detectHand(this->model_data);
                this->arm->calculateLengthwiseNextJoint_detectHand();
                if(this->arm->validateNextJoint() == 0){
                    #ifdef DEBUG
                    this->sender->sendData("!GO LENGTHWISE");
                    #endif
                    //can move
                    this->arm->calculateTotalSteps();
                    double initNumberStepsDone[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    this->arm->setNumberStepDone(initNumberStepsDone);
                    this->arm->initjointAutoMoveDone();
                    this->arm->setState(DETECT_HAND_AUTO_MOVING);
                    for(int i = 0; i < 6; i++){
                        this->timer_arm[i]->setLoopAction(3000, micros()); //int delValue = 3000
                    }
                }
                this->arm->isLengthwiseMove = false;
            }
            else {
                this->arm->isHorizontalMove = true;
                this->arm->isLengthwiseMove = true;
                this->arm->updateCurrentPosition();
                this->nextArmAction = ARM_STOP_ACTION;
                this->arm->setState(STOP);
                #ifdef DEBUG
                this->sender->sendData("!GO STATE STOP");
                #endif
            }
        }
        else if (this->nextArmAction == ARM_MANUAL_MOVE_DISTANCE_ACTION){
            this->arm->setState(MANUAL_MOVING);
            this->arm->initManualMove();
            for(int i = 0; i < 6; i++){
                this->timer_arm[i]->setLoopAction(3000, micros()); //int delValue = 3000
            }
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
}

void System::slider_fsm(){
    switch (this->slider1->getCurrentState())
    {
    case INIT:
        // waiting init action
        if (this->nextSliderAction == SLIDER_INIT_ACTION)
        {
            #ifdef DEBUG
            this->sender->sendData("!SLIDER INIT");
            #endif
            int tmp = this->slider1->onStart(); // cannot interrupt
            #ifdef DEBUG
            String tmp_str = "!return count :" + String(tmp); // may crash
            this->sender->sendData(tmp_str);
            #endif
            this->slider1->setState(HOME);
        }
        else{
        }
        break;
    case HOME:
        // waiting new action
        if (this->nextSliderAction == SLIDER_AUTO_MOVE_FREE_ACTION)
        {
            if(this->slider1->validatePosition(this->output_slider_auto) == 0){
                //can move
                this->slider1->setNextPosition(this->output_slider_auto);
                this->slider1->calculateTotalSteps();
                this->slider1->initStepDone();
                this->slider1->setState(GENERAL_AUTO_MOVING);
                this->output_slider_auto = 0.0;
                this->timer_slider->setLoopAction(400, micros());
                #ifdef DEBUG
                this->sender->sendData("!GO AUTO SLIDER");
                #endif
            }
        }
        else if (this->nextSliderAction ==  SLIDER_AUTO_MOVE_DETECT_HAND_ACTION){
            this->slider1->setState(DETECT_HAND_AUTO_MOVING);
        }
        else if (this->nextSliderAction ==  SLIDER_MANUAL_MOVE_DISTANCE_ACTION){
            this->slider1->setState(MANUAL_MOVING);
            this->timer_slider->setLoopAction(100, micros()); //int delValue = 400
        }
        else if (this->nextSliderAction == SLIDER_STOP_ACTION){
            this->slider1->setState(STOP);
        }
        else{
        }
        break;
    case MANUAL_MOVING:
        if(this->nextSliderAction == SLIDER_MANUAL_MOVE_DISTANCE_ACTION)
        {
            this->slider1->setState(MANUAL_MOVING);
            if(this->timer_slider->checkTimeoutAction()) {
                this->slider1->manualMove(this->output_slider_manual);
            }
        }
        else if(this->nextSliderAction == SLIDER_STOP_ACTION){
            this->slider1->setState(STOP);
        }
        else{
        }
        break;
    case GENERAL_AUTO_MOVING:
        if (this->nextSliderAction = SLIDER_AUTO_MOVE_FREE_ACTION){
            if (this->slider1->isAutoMoveDone()){
                this->slider1->updatePosition();
                this->nextSliderAction = SLIDER_STOP_ACTION;
                this->slider1->setState(STOP);
                #ifdef DEBUG
                this->sender->sendData("!SLIDER GO STATE STOP");
                #endif
            }
            else{
                this->slider1->setState(GENERAL_AUTO_MOVING);
                if(this->timer_slider->checkTimeoutAction()) {
                    #ifdef DEBUG
                    String tmpstr = "!SLIDER GO step to move" + String(this->slider1->getNumberStepToGo());
                    this->sender->sendData(tmpstr);
                    #endif
                    this->slider1->generalAutoMove(this->timer_slider->timeout);
                    #ifdef DEBUG
                    this->sender->sendData("!SLIDER GO AUTO");
                    #endif
                }
            }
        }
        else if (this->nextSliderAction == SLIDER_STOP_ACTION){
            this->slider1->updatePosition();
            this->slider1->setState(STOP);
            #ifdef DEBUG
            this->sender->sendData("!SLIDER GO STATE STOP");
            #endif
        }
        else {
        }
        break;
    case STOP:
        // waiting new action
        if (this->nextSliderAction == SLIDER_AUTO_MOVE_FREE_ACTION)
        {
            this->slider1->setState(GENERAL_AUTO_MOVING);
        }
        else if  (this->nextSliderAction == SLIDER_AUTO_MOVE_DETECT_HAND_ACTION){
            this->slider1->setState(DETECT_HAND_AUTO_MOVING);
        }
        else if (this->nextSliderAction ==  SLIDER_MANUAL_MOVE_DISTANCE_ACTION){
            this->slider1->setState(MANUAL_MOVING);
            this->timer_slider->setLoopAction(100, micros()); //int delValue = 4000
        }
        else{
        }
        break;
    default:
        break;
    }
}

/** GRIPPER FSM */
void System::gripper_fsm() {
    switch(this->gripper->getCurrentState()) 
    {
    case INIT: 
        if (this->nextGripperAction == GRIPPER_INIT_ACTION) {
            #ifdef DEBUG
            this->sender->sendData("!GRIPPER INIT");
            #endif
            int tmp = this->gripper->initGripper(); // cannot interrupt
            this->gripper->setCurrentState(HOME);
        }
        break;
    case HOME:
        if (this->nextGripperAction == GRIPPER_STOP_ACTION) {
            this->gripper->setCurrentState(STOP); 
        } 
        else if (this->nextGripperAction == GRIPPER_MANUAL_MOVE_ACTION) {
            this->gripper->setCurrentState(MANUAL_MOVING); 
            this->timer_gripper->setLoopAction(100, micros()); 
        }
        break; 
    //TODO: manual move
    case MANUAL_MOVING: 
        break;
    case STOP: 
        break; 
    default: 
        break; 
    }
}

void System::distributeAction(){ // send ACK here
    switch (this->nextAction)
    {
    case INIT_ACTION:
        this->nextArmAction = ARM_INIT_ACTION;
        this->nextSliderAction = SLIDER_INIT_ACTION;
        this->nextGripperAction = GRIPPER_INIT_ACTION; 
        this->listener->consumeCommand(ARM_INIT_ACTION, nullptr);
        this->listener->consumeCommand(SLIDER_INIT_ACTION,nullptr);
        this->listener->consumeCommand(GRIPPER_GOHOME_ACTION, nullptr); 
        this->sender->sendACK("!I#");
        this->nextAction = NO_ACTION;
        break;
    case ARM_GOHOME_ACTION:
        this->nextArmAction = ARM_GOHOME_ACTION;
        this->listener->consumeCommand(ARM_GOHOME_ACTION, nullptr);
        this->sender->sendACK("!AH#");
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_GOHOME_ACTION:
        this->nextSliderAction = SLIDER_GOHOME_ACTION;
        this->listener->consumeCommand(SLIDER_GOHOME_ACTION, nullptr);
        this->nextAction = NO_ACTION;
        break;
    case GRIPPER_GOHOME_ACTION:
        this->nextGripperAction = GRIPPER_GOHOME_ACTION; 
        this->listener->consumeCommand(GRIPPER_GOHOME_ACTION, nullptr); 
        this->nextAction = NO_ACTION; 
        break; 
    case ARM_STOP_ACTION:
        this->nextArmAction = ARM_STOP_ACTION;
        this->listener->consumeCommand(ARM_STOP_ACTION, nullptr);
        this->sender->sendACK("!AS#");
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_STOP_ACTION:
        this->nextSliderAction = SLIDER_STOP_ACTION;
        this->listener->consumeCommand(SLIDER_STOP_ACTION,nullptr);
        this->sender->sendACK("!SS#");
        this->nextAction = NO_ACTION;
    case GRIPPER_STOP_ACTION:
        this->nextGripperAction = GRIPPER_STOP_ACTION;
        this->listener->consumeCommand(GRIPPER_STOP_ACTION,nullptr);
        this->sender->sendACK("!GS#");
        this->nextAction = NO_ACTION;
    case ARM_MANUAL_MOVE_DISTANCE_ACTION:
        this->nextArmAction = ARM_MANUAL_MOVE_DISTANCE_ACTION;
        this->listener->consumeCommand(ARM_MANUAL_MOVE_DISTANCE_ACTION, this->gamepad_data);
        this->sender->sendACK("!M#");
        this->nextAction = NO_ACTION;
        break;
    case ARM_AUTO_MOVE_POSITION_ACTION:
        this->nextArmAction = ARM_AUTO_MOVE_POSITION_ACTION;
        this->listener->consumeCommand(ARM_AUTO_MOVE_POSITION_ACTION, this->nextPosition_data);
        this->sender->sendACK("!A#");
        this->nextAction = NO_ACTION;
        break;
    case ARM_AUTO_MOVE_DETECT_HAND_ACTION:
        this->nextArmAction = ARM_AUTO_MOVE_DETECT_HAND_ACTION;
        this->listener->consumeCommand(ARM_AUTO_MOVE_DETECT_HAND_ACTION, this->model_data);
        this->arm->isHorizontalMove = true;
        this->arm->isLengthwiseMove = true;
        this->sender->sendACK("!HA#");
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_MANUAL_MOVE_DISTANCE_ACTION:
        this->nextSliderAction = SLIDER_MANUAL_MOVE_DISTANCE_ACTION;
        this->listener->consumeCommand(SLIDER_MANUAL_MOVE_DISTANCE_ACTION,&this->output_slider_manual);
        this->sender->sendACK("!S#");
        this->nextAction = NO_ACTION;
        break;
    case SLIDER_AUTO_MOVE_FREE_ACTION:
        this->nextSliderAction = SLIDER_AUTO_MOVE_FREE_ACTION;
        this->listener->consumeCommand(SLIDER_AUTO_MOVE_FREE_ACTION,&this->output_slider_auto);
        this->sender->sendACK("!X#");
        this->nextAction = NO_ACTION;
        break;
    /** GRIPPER CONTROLLER */
    case GRIPPER_MANUAL_MOVE_ACTION: 
        this->nextGripperAction = GRIPPER_MANUAL_MOVE_ACTION; 
        this->listener->consumeCommand(GRIPPER_MANUAL_MOVE_ACTION, &this->output_gripper); 
        this->sender->sendACK("!GM#"); 
        this->nextAction = NO_ACTION; 
        break; 
    case 
    default:
        break;
    }
}