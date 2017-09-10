#ifndef PARROT_H
#define PARROT_H

#ifdef WIN32
#include "CHeliWin.h"
#else
#include "CHeli.h"
#include "CGui.h"
#include "CRecognition.h"
#endif
class Parrot
{
protected:
    int nr;
    unsigned char buf[10];
    bool stop;
    CGui *gui;
    CRawImage *image;
    CRecognition *recognition;
    SPixelPosition pos;
    bool move;
    CHeli *heli;
    float pitch,roll,yaw,height;
    bool calibracion;
    int ajusteAltura, ajusteYaw, ajustePitch, ajusteRoll;
    unsigned char canales[4];

public:
    Parrot();
    void connect();
    void disconnect();
    void takeOff();
    void land();
    void setAngles(float roll, float pitch, float yaw, float verticalSpeed, int hover);
    void getHeliData(double &phi, double &psi, double &theta, double &altitude, double &battery, double &vx, double &vy, double &vz);

};

#endif // PARROT_H
