  #ifndef __ARM__H
#define __ARM__H

#include "Arduino.h"
#include "global.h"
#include "Communication.h"

class Arm {
    private:
        int state;
        double joint[6]; // current joints angle
        double position[6]; // current {x, y, z, ZYZ Euler angles}
        double nextJoint[6]; // next joints angle
        double nextPosition[6]; // next {x, y, z, ZYZ Euler angles}
        double numberStepDoneAccelerate_manual[6];
        double numberStepDoneDecelerate_manual[6];
        double numberStepToGo[6];
        double numberStepDone[6];
        bool jointManualMoveDone[6];
        bool jointAutoMoveDone[6];
        Sender* sender;

    public:
        const double home_position[6] = {192.5, 0.0, 269.0, 0.0, 90.0, 0.0};
        const double home_joint[6] = {0.0, 0.0, 0.0, 180.0, 0.0, 0.0};
    
    public:
        Arm();
        ~Arm();
        void onStart();
        int getCurrentState();
        void setState(int state);
        void getCurrentJoint(double* output);
        void setJoint(double* joint);
        void getCurrentPosition(double* output);
        void setPosition(double* position);
        void getNextJoint(double* output);
        void setNextJoint(double* joint);
        void getNextPosition(double* output);
        void setNextPosition(double* position);
        void getNumberStepToGo(double* output);
        void setNumberStepToGo(double* steps);
        void getNumberStepDone(double* output);
        void setNumberStepDone(double* steps);
        void printCurrentJoint();
        void printCurrentPos();

        int validateNextJoint();
        void initManualMove();
        bool isManualMoveDone();
        void manualMove(int i, double* input, unsigned long &timeout, double incValue = 300);
        void calculateNextJoint();
        void calculateTotalSteps();
        void initjointAutoMoveDone();
        bool isAutoMoveDone();
        void updateCurrentPosition();
        void generalAutoMove(int i, unsigned long &timeout, double incValue = 3.5, int accRate = 530);

    public:
        // funtions for specific use case move Lengthwise
        void calculateHorizontalNextPosition_detectHand(double* model_data);
        void calculateHorizontalNextJoint_detectHand();
        void calculateLengthwiseNextPosition_detectHand(double* model_data);
        void calculateLengthwiseNextJoint_detectHand();
        // void autoMove_detectHand(double* Xnext, double vel0, double acc0, double velini, double velfin);
        // void calculateNewPosition_detectHand(double* output, double* input);

    public:
        // variables for specific use case move
        bool isHorizontalMove = true;
        bool isLengthwiseMove = true;

    private:
        // funtions for general move
        void singleJointMove_onStart(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530); // can not be interrupt
        double double_abs(double num);

    private:
        //robot geometry
        const double MAX_JOINT_ANGLE[6] = {90.0, 87.0, 80.0, 180.0, 120.0, 180.0}; // refind max min joint
        const double MIN_JOINT_ANGLE[6] = {-90.0, -60.0, -80.0, -180.0, -90.0, -180.0};

        const double r1 = 47.0;
        const double r2 = 110.0;
        const double r3 = 26.0; 
        const double d1 = 133.0;
        const double d3 = 0.0;
        const double d4 = 117.50;
        const double d6 = 28.0;

        //Angle per step move = microstep / gear_ratio
        const double dl1 = 360.0/200.0/32.0/4.8;
        const double dl2 = 360.0/200.0/32.0/4.0;
        const double dl3 = 360.0/200.0/32.0/5.0;
        const double dl4 = 360.0/200.0/16.0/2.9;
        const double dl5 = 360.0/200.0/16.0/2.1;
        const double dl6 = 360.0/200.0/16.0/1.0;

        const double DL[6] = {dl1, dl2, dl3, dl4, dl5, dl6};

        bool PULstat[6] = {0, 0, 0, 0, 0, 0};
        int PUL_PINS[6] = {PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN, PUL5_PIN, PUL6_PIN};
        int DIR_PINS[6] = {DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN, DIR5_PIN, DIR6_PIN}; 

        const double velG = 0.25e-4;
        double start_vel = 1 * velG;
        double end_vel = 1 * velG; 

    private:
        // kinematic funtions
        void InverseK(double* Xik, double* Jik);
        void ForwardK(double* Jfk, double* Xfk);
        void invtran(double* Titi, double* Titf);
        void tran2pos(double* Ttp, double* Xtp);
        void pos2tran(double* Xpt, double* Tpt);
        void DH1line(double thetadh, double alfadh, double rdh, double ddh, double* Tdh);
        void MatrixPrint(double* A, int m, int n, String label);
        void MatrixCopy(double* A, int n, int m, double* B);
        void MatrixMultiply(double* A, double* B, int m, int p, int n, double* C);
        void MatrixAdd(double* A, double* B, int m, int n, double* C);
        void MatrixSubtract(double* A, double* B, int m, int n, double* C);
        void MatrixTranspose(double* A, int m, int n, double* C);
        void MatrixScale(double* A, int m, int n, double k);

};


#endif