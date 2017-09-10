#include "PTZCameraController.h"
#include <QDebug>

PTZCameraController::PTZCameraController()
{
    busy = false;
    connected = false;
    panSpeed = 20;
    tiltSpeed = 20;
    pan = 0;
    tilt = 0;
    deltaPan = 100;
    deltaTilt = 100;
    windowWidth = 720;
    windowHeight = 480;
    deltaXYThreshold = 10;
    panTiltMoving = false;

    // Reset elapsed time
    elapsedTime = clock();
}

bool PTZCameraController::isConnected()
{
    return connected;
}

void PTZCameraController::setSerialPort(std::string port)
{
    this->port = new QextSerialPort(QLatin1String(port.c_str()), QextSerialPort::Polling);
    this->port->setBaudRate(BAUD9600);
    this->port->setFlowControl(FLOW_OFF);
    this->port->setParity(PAR_NONE);
    this->port->setDataBits(DATA_8);
    this->port->setStopBits(STOP_1);
    //set timeouts to 500 ms
    this->port->setTimeout(500);
}

void PTZCameraController::connect()
{
    if (!connected)
    {
        port->open(QIODevice::ReadWrite | QIODevice::Unbuffered);
        //port->open(QIODevice::ReadWrite);
        connected = port->isOpen();
        qDebug("Port Status: %d", connected);
    }
    else
    {
        qDebug() << "Port already connected";
    }
}

void PTZCameraController::disconnect()
{
    if (connected)
    {
        port->close();
        connected = port->isOpen();
        qDebug("Port Status: %d", connected);
    }
    else
    {
        qDebug() << "Port already disconnected";
    }
}

void PTZCameraController::setPanTiltSpeed(int panSpeed, int tiltSpeed)
{
    this->panSpeed = panSpeed;
    this->tiltSpeed = tiltSpeed;
}

void PTZCameraController::setPan(int pan)
{
    this->pan = pan;
    PTZCameraController::setPanTilt(pan, tilt);
}

void PTZCameraController::setTilt(int tilt)
{
    this->tilt = tilt;
    PTZCameraController::setPanTilt(pan, tilt);
}

void PTZCameraController::setPanTilt(int pan, int tilt)
{
    this->pan = pan;
    this->tilt = tilt;
    if (connected)
    {
        // Reset elapsed time
        elapsedTime = clock();

        std::stringstream commandBuilder;
        commandBuilder << "81010602";
        // Add pan tilt speed
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << panSpeed;
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << tiltSpeed;
        // Add absolute pan
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((pan >> 12) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((pan >> 8) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((pan >> 4) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((pan >> 0) & 0x0f);
        // Add absolute tilt
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((tilt >> 12) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((tilt >> 8) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((tilt >> 4) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((tilt >> 0) & 0x0f);
        commandBuilder << "FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}

void PTZCameraController::run()
{
    // 0: Start, 1: Socket, 2: (Optional) Information, 3: Terminator
    // ACK: 90 41 FF
    // COMPLETION: 90 51 FF
    int nextStatus = 0;

    char buffer[12];
    int bufferIndex = 0;

    unsigned char header = 0x90;
    unsigned char end = 0xff;

    while (true)
    {
        if (port->bytesAvailable())
        {
            char currentChar;
            port->getChar(&currentChar);
            unsigned char currentCharValue = currentChar;

            if (currentCharValue == 0x90 && nextStatus == 0)
            {
                // Camera reply header (Address)
                nextStatus = 1;
                buffer[bufferIndex++] = currentChar;
            }
            else if (currentCharValue == 0xff)
            {
                // Its a terminator, everything else does not matter
                nextStatus = 0;
                buffer[bufferIndex++] = currentChar;
                std::stringstream currentPacket;
                for (int i = 0; i < bufferIndex; ++i)
                {
                    currentPacket <<  std::hex << std::setw(2) << std::setfill('0') << static_cast<int>((unsigned char)buffer[i]) << " ";
                }
                //qDebug() << "Packet: " << currentPacket.str().c_str();

                if (bufferIndex == 3)
                {
                    if (static_cast<int>((unsigned char)buffer[0]) == 0x90 && static_cast<int>((unsigned char)buffer[2]) == 0xff)
                    {
                        if (static_cast<int>((unsigned char)buffer[1]) == 0x41)
                        {
                            //qDebug() << "Setting to busy";
                            busy = true;
                        }
                        else if (static_cast<int>((unsigned char)buffer[1]) == 0x51)
                        {
                            //qDebug() << "Setting to ready";
                            busy = false;
                        }
                    }
                }

                bufferIndex = 0;
            }
            else if (nextStatus == 1)
            {
                buffer[bufferIndex++] = currentChar;
                if (currentCharValue == 0x50)
                {
                    // Information reply header
                    nextStatus = 2;
                }
            }
            else if (nextStatus == 2)
            {
                buffer[bufferIndex++] = currentChar;
                // Process inquiry
                switch (lastViscaInquiry)
                {
                case PAN_TILT_STATUS:
                    if (bufferIndex == 3)
                    {
                        //qDebug() << "ZZ " << ((unsigned char) buffer[0]) << " , ZZ " << ((unsigned char)buffer[1]);
                        if (buffer[0] & 0x0c)
                            panTiltMoving = false;
                        else
                            panTiltMoving = true;
                    }
                    break;
                }
            }
            else
            {
                qDebug() << "Oh no, bug was found!";
            }

        }
        else
        {
            //  No bytes received, then sleep
            usleep(2000);
        }
    }
}

bool PTZCameraController::isMoving()
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "81090610FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
        lastViscaInquiry = PAN_TILT_STATUS;
        return panTiltMoving;
    }
    return true;
}

bool PTZCameraController::isReady()
{
    if (((float)(clock() - elapsedTime)) / CLOCKS_PER_SEC > 500)
    {
        // Reset
        qDebug() << "Oh no, serial port Time Out";
        busy = false;
        elapsedTime = clock();
    }
    return !busy;
    // Before we used a simple timeout, but it won't work ok.
    //return ((float)(clock() - elapsedTime)) / CLOCKS_PER_SEC > 0.1;
}

void PTZCameraController::setZoom(int zoom)
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "81010447";
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((zoom >> 12) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((zoom >> 8) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((zoom >> 4) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((zoom >> 0) & 0x0f);
        commandBuilder << "FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}

void PTZCameraController::setDeltaPanTilt(int deltaPan, int deltaTilt)
{
    this->deltaPan = deltaPan;
    this->deltaTilt = deltaTilt;
}

void PTZCameraController::setDeltaXYThreshold(int deltaXYThreshold)
{
    this->deltaXYThreshold = deltaXYThreshold;
}

void PTZCameraController::setFocus(int focus)
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "81010448";
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((focus >> 12) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((focus >> 8) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((focus >> 4) & 0x0f);
        commandBuilder << std::setw(2) << std::setfill('0') << std::hex << ((focus >> 0) & 0x0f);
        commandBuilder << "FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
    else
    {
        qDebug() << "Not in manual focus mode";
    }
}

void PTZCameraController::setFocusAutoOn(bool focusAutoOn)
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "8101043810FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}

void PTZCameraController::setBrightnessAutoOn()
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "8101043900FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}

void PTZCameraController::setBrightnessAutoOff()
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "810104390DFF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}

void PTZCameraController::setBrightnessUp()
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "8101040D02FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}

void PTZCameraController::setBrightnessDown()
{
    if (connected)
    {
        std::stringstream commandBuilder;
        commandBuilder << "8101040D03FF";
        QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
        port->write(text);
    }
}


PTZCameraController::CameraMovement PTZCameraController::centerCamera(std::vector<cv::Point2f> centerPoints)
{
    bool moveCamera = false;
    cv::Point2f center = cv::Point2f(0, 0);
    for (int i = 0; i < centerPoints.size(); ++i)
    {
        cv::Point2f currentPoint = centerPoints[i];
        center.x += currentPoint.x;
        center.y += currentPoint.y;
    }
    center.x /= centerPoints.size();
    center.y /= centerPoints.size();

    int deltaX = windowWidth / 2 - center.x;
    int deltaY = windowHeight / 2 - center.y;

    CameraMovement cameraMovement;
    cameraMovement.previousPan = pan;
    cameraMovement.previousTilt = tilt;

    if (deltaX  > deltaXYThreshold && pan >= -15000)
    {
        pan = pan - deltaPan;
        moveCamera = true;
    }
    else if (deltaX  < -deltaXYThreshold && pan <= 15000)
    {
        pan = pan + deltaPan;
        moveCamera = true;
    }

    if (deltaY > deltaXYThreshold && tilt <= 7000)
    {
        tilt = tilt + deltaTilt;
        moveCamera = true;
    }
    else if (deltaY < -deltaXYThreshold && tilt >= -7000)
    {
        tilt = tilt - deltaTilt;
        moveCamera = true;
    }

    cameraMovement.cameraMoved = moveCamera;
    cameraMovement.newPan = pan;
    cameraMovement.newTilt = tilt;

    if (moveCamera)
    {
        setPanTilt(pan, tilt);
    }

    return cameraMovement;
}

void PTZCameraController::setWindowDimensions(int width, int height)
{
    windowWidth = width;
    windowHeight = height;
}
