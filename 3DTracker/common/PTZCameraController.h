#ifndef PTZCAMERACONTROLLER_H
#define PTZCAMERACONTROLLER_H

#include "qextserialport.h"
#include <sstream>
#include <iomanip>
#include <vector>
#include "opencv2/core/core.hpp"
#include <QThread>
#include <time.h>

class PTZCameraController : public QThread
{
private:
    QextSerialPort* port;
    int deltaPan;
    int deltaTilt;
    int deltaXYThreshold;
    int windowWidth;
    int windowHeight;
    bool connected;
    bool panTiltMoving;
    clock_t elapsedTime;
    bool busy;

    enum ViscaInquiry {PAN_TILT_POSITION, PAN_TILT_STATUS} lastViscaInquiry;

public:
    int pan;
    int tilt;
    int panSpeed;
    int tiltSpeed;

    struct CameraMovement
    {
        bool cameraMoved;
        int previousPan;
        int previousTilt;
        int newPan;
        int newTilt;
    };

    PTZCameraController();
    bool isConnected();
    void setSerialPort(std::string port);
    void connect();
    void disconnect();
    void run();
    bool isMoving();
    bool isReady();
    void setZoom(int zoom);
    void setPanTiltSpeed(int panSpeed, int tiltSpeed);
    void setPan(int pan);
    void setTilt(int tilt);
    void setPanTilt(int pan, int tilt);
    void setDeltaPanTilt(int deltaPan, int deltaTilt);
    void setDeltaXYThreshold(int deltaXYThreshold);
    void setFocus(int focus);
    void setFocusAutoOn(bool focusAutoOn);
    void setBrightnessAutoOn();
    void setBrightnessAutoOff();
    void setBrightnessUp();
    void setBrightnessDown();
    CameraMovement centerCamera(std::vector<cv::Point2f> centerPoints);
    void setWindowDimensions(int width, int height);
};

#endif // PTZCAMERACONTROLLER_H
