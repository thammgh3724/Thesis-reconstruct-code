//Kinematics library
#ifndef __KINEMATICS__H
#define __KINEMATICS__H

#include "Arduino.h"

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
void setcurJoint(double pos1, double pos2, double pos3, double pos4, double pos5, double pos6);
double getcurJoint(uint8_t i);
void printcurJoint();

#endif