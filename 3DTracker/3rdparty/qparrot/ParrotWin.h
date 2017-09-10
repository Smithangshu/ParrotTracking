#ifndef PARROTWIN_H
#define PARROTWIN_H

#include "CHeliWin.h"

class Parrot
{
protected:
    int nr;
    unsigned char buf[10];
    bool stop;
    CGui *gui;
    CRawImage *image;
    SDL_Event event;
    CRecognition *recognition;
    SPixelPosition pos;
    bool move;
    Uint8 lastKeys[10000];
    int keyNumber;
    Uint8 *keys;
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
    void setAngles(float roll, float pitch, float yaw, float verticalSpeed);
};

#endif // PARROTWIN_H
