#ifndef PARROTCONTROLLER_H
#define PARROTCONTROLLER_H

#include <QThread>
#include <QElapsedTimer>
#include "3rdparty/qparrot/Parrot.h"
#include "3rdparty/qparrot/app.h"
#include "opencv2/core/core.hpp"
#include "common/KalmanFilter.h"

class ParrotController : public QThread
{
    Q_OBJECT
private:
    // Store of joypad values
    int joypadRoll;
    int joypadPitch;
    int joypadYaw;
    int joypadVerticalSpeed;
    // Store for haptic input
    int hapticInputX;
    int hapticInputY;

    float roll;
    float pitch;
    float yaw;
    float positionX;
    float positionY;
    float positionZ;
    //Datos del IMU
    double phi;
    double psi;
    double theta;
    double battery;
    double altitude;
    double vx;
    double vy;
    double vz;
    cv::Mat pose;
    float verticalSpeed;
    bool joypadAvailable;
    bool connected;
    bool takeOffFlag;
    bool landFlag;
    bool enableAutomaticControl;
    bool receivingParrotValues;
    int landingThreshold;
    // Variable for control
    float spRoll;
    float spPitch;
    float spYaw;
    float spPositionX;
    float spPositionY;
    float spPositionZ;
    float preErrorRoll;
    float preErrorPitch;
    float preErrorYaw;
    float preErrorVerticalSpeed;
    float preErrorRollSpeed;
    float preErrorPitchSpeed;
    float prePositionX;
    float prePositionZ;
    float integralErrorRoll;
    float integralErrorPitch;
    float integralErrorYaw;
    float integralErrorVerticalSpeed;
    float integralErrorRollSpeed;
    float integralErrorPitchSpeed;
    // Proportional term
    float KpRollPitch;
    float KpRollPitchSpeed;
    float KpYaw;
    float KpVerticalSpeed;
    // Derivative term
    float KdRollPitch;
    float KdRollPitchSpeed;
    float KdYaw;
    float KdVerticalSpeed;
    // Integral term
    float KiRollPitch;
    float KiRollPitchSpeed;
    float KiYaw;
    float KiVerticalSpeed;
    // For speed control
    float preSpeedX;
    float preSpeedY;
    float speedX;
    float speedY;
    float maxRollPitchSpeed;
    float targetRollSpeed;
    float targetPitchSpeed;
    // Trajectory
    cv::Mat trajectory;
    int currentTrajectoryIndex;

    // Filters for results
    Filter filter;
    Filter filterOutput;
    Filter filterSpeeds;

protected:
    Parrot parrot;
public:
    ParrotController();
    void setJoypadValues(float roll, float pitch, float yaw, float verticalSpeed);
    void setParrotAngles(float roll, float pitch, float yaw, float verticalSpeed, int hover);
    void setAnglesValues(float roll, float pitch, float yaw, float positionX, float positionY, float positionZ);
    void setPose(cv::Mat &pose);
    void setTrajectory(cv::Mat trajectory);
    void resetFilter(cv::Mat &pose);
    void connect();
    void disconnect();
    void takeOff();
    void land();
    void run();
    bool isConnected();
    void setAutomaticControl(bool enableAutomaticControl);
    void estimateControlVars(float &targetRoll, float &targetPitch, float &targetYaw, float &targetVerticalSpeed);
    float normalize(float input);
    void rawToMat(cv::Mat &destImage, CRawImage* sourceImage);
signals:
    void setBatteryChargeValueSignal(int battery);
    void writeParrotPositionToHapticSignal(std::string);
public slots:
    void connectSlot();
    void disconnectSlot();
    void takeOffSlot();
    void landSlot();
    void setJoypadValuesSlot(float roll, float pitch, float yaw, float verticalSpeed);
    void setHapticInputSlot(int x, int y);
    void setAnglesValuesSlot(float roll, float pitch, float yaw, float positionX, float positionY, float positionZ);
    void setPoseSlot(cv::Mat pose);
    void setTrajectorySlot(cv::Mat trajectory);
    void resetFilterSlot(cv::Mat pose);
    void setControlVariablesSlot(float KpRollPitch, float KpRollPitchSpeed, float KpYaw, float KpVerticalSpeed, float KdRollPitch, float KdRollPitchSpeed, float KdYaw, float KdVerticalSpeed, float KiRollPitch, float KiRollPitchSpeed, float KiYaw, float KiVerticalSpeed, float inputFilterScale, float outputFilterScale, float maxRollPitchSpeed);
    void getControlVariablesSlot(float &KpRollPitch, float &KpRollPitchSpeed, float &KpYaw, float &KpVerticalSpeed, float &KdRollPitch, float &KdRollPitchSpeed, float &KdYaw, float &KdVerticalSpeed, float &KiRollPitch, float &KiRollPitchSpeed, float &KiYaw, float &KiVerticalSpeed);
};

#endif // PARROTCONTROLLER_H
