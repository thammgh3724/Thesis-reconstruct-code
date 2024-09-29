#ifndef __ARM__H
#define __ARM__H

#include "Arduino.h"
#include "global.h"
#include "communication.h"

class Arm {
    private:
        int state;
        double joint[6]; // current joints angle
        double position[6]; // current {x, y, z, ZYZ Euler angles}
        double nextJoint[6]; // next joints angle
        double nextPosition[6]; // next {x, y, z, ZYZ Euler angles}
        Sender* sender;
    
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
        void printCurrentJoint();
        void printCurrentPos();

        void manualMove(double* input);

    private:
        //robot geometry
        const double MAX_JOINT_ANGLE[6] = {90, 87, 80, 180, 120, 180};
        const double MIN_JOINT_ANGLE[6] = {-90, -60, -80, -180, -90, -180};

        const double r1 = 47.0;
        const double r2 = 110.0;
        const double r3 = 26.0; 
        const double d1 = 133.0;
        const double d3 = 0.0;
        const double d4 = 117.50;
        const double d6 = 28.0;

        bool PULstat[6] = {false, false, false, false, false, false};
        int PUL_PINS[6] = {PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN, PUL5_PIN, PUL6_PIN};
        int DIR_PINS[6] = {DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN, DIR5_PIN, DIR6_PIN}; 

        const double velG = 0.25e-4;
        double start_vel = 1 * velG;
        double end_vel = 1 * velG; 

    private:
        // funtions for general move
        int validateJoint(double* input);
        void singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530);
        void generalAutoMove(double* Xnext, double vel0, double acc0, double velini, double velfin);
        // void calculateNewJoint_manual(double* output, double* input);

    private:
        // variables for specific use case move
        bool isFirstmove = false; 
        bool isHorizontalMove = false;
        bool isLengthwiseMove = false;

    private:
        // funtions for specific use case move
        void autoMove_detectHand(double* Xnext, double vel0, double acc0, double velini, double velfin);
        void calculateNewPosition_detectHand(double* output, double* input);

    private:
        // kinematic funtions
        void goStrightLine(double* xfi, double* xff, double vel0, double acc0, double velini, double velfin);
        void goTrajectory(double* Jf);
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