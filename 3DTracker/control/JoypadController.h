#ifndef JOYPADCONTROLLER_H
#define JOYPADCONTROLLER_H

#include <QThread>
#include <QString>
#include <QList>

#include "SDL/SDL.h"

class JoypadController : public QThread
{
    Q_OBJECT
signals:
    void setJoypadValuesSignal(float roll, float pitch, float yaw, float verticalSpeed);
private:
    SDL_Joystick* m_joystick;
public:
    JoypadController();
    ~JoypadController();
    int availableJoysticks();
    int currentJoystick();
    QString joystickName(int id);
    int joystickNumAxes(int id);
    int joystickNumButtons(int id);
    QList<int> axis;
    QList<bool> buttons;
    void run();
    void setJoystick(int jsNumber);
};

#endif // JOYPADCONTROLLER_H
