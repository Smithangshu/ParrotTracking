#include "control/JoypadController.h"
#include "common/TrackingCommon.h"
#include <QDebug>

JoypadController::JoypadController()
{
    // Sure, we're only using the Joystick, but SDL doesn't work if video isn't initialised
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
}

int JoypadController::currentJoystick()
{
    return SDL_JoystickIndex(m_joystick);
}

QString JoypadController::joystickName(int js)
{
    Q_ASSERT(js < availableJoysticks());
    Q_ASSERT(js >= 0);
    return QString(SDL_JoystickName(js));
}

int JoypadController::joystickNumAxes(int js)
{
    Q_ASSERT(js < availableJoysticks());
    Q_ASSERT(js >= 0);
    return (SDL_JoystickNumAxes(m_joystick));
}

int JoypadController::joystickNumButtons(int js)
{
    Q_ASSERT(js < availableJoysticks());
    Q_ASSERT(js >= 0);
    return (SDL_JoystickNumButtons(m_joystick));
}

void JoypadController::setJoystick(int js)
{
    Q_ASSERT(js < availableJoysticks());
    Q_ASSERT(js >= 0);

    SDL_JoystickClose(m_joystick);
    m_joystick = SDL_JoystickOpen(js);
}

JoypadController::~JoypadController()
{
    axis.clear();
    buttons.clear();
    SDL_JoystickClose(m_joystick);
    SDL_QuitSubSystem(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    this->deleteLater();
}

int JoypadController::availableJoysticks()
{
     return SDL_NumJoysticks();
}

void JoypadController::run()
{
    while (true)
    {
        axis.clear();
        buttons.clear();

        SDL_Event event;

        SDL_PollEvent(&event);

        float joypadRoll = -100.0 * (SDL_JoystickGetAxis(m_joystick, 0) / 24000.0);
        float joypadPitch = 100.0 * (SDL_JoystickGetAxis(m_joystick, 1) / 24000.0);
        float joypadVerticalSpeed = 100.0 * (SDL_JoystickGetAxis(m_joystick, 2) / 24000.0);
        float joypadYaw = -100.0 * (SDL_JoystickGetAxis(m_joystick, 4) / 24000.0);

        emit setJoypadValuesSignal(joypadRoll, joypadPitch, joypadYaw, joypadVerticalSpeed);

//    qDebug() << "joypad: " << TrackingCommon::joypadRoll << ", " << TrackingCommon::joypadPitch << ", " << TrackingCommon::joypadYaw << ", " << TrackingCommon::joypadVerticalSpeed;
//    for (int i=0; i<SDL_JoystickNumAxes(m_joystick); i++)
//    {
//        axis.append(SDL_JoystickGetAxis(m_joystick,i));
//    }

//    for(int i=0;i<SDL_JoystickNumButtons(m_joystick);i++)
//    {
//        buttons.append(SDL_JoystickGetButton(m_joystick,i));
//    }
        usleep(40000);
    }
}

