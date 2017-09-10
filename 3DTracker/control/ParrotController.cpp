#include "control/ParrotController.h"
#include "common/TrackingCommon.h"
#include "common/MainController.h"

ParrotController::ParrotController()
{
    joypadAvailable = false;
    connected = false;
    takeOffFlag = false;
    landFlag = false;
    enableAutomaticControl = false;
    receivingParrotValues = false;

    hapticInputX = 0;
    hapticInputY = 0;

    KpRollPitch = 0.065;
    KpRollPitchSpeed = 0.5;
    KpYaw = 0.4;
    KpVerticalSpeed = 0.5;

    KdRollPitch = 7;
    KdRollPitchSpeed = 3;
    KdYaw = 0.0;
    KdVerticalSpeed = 0;

    KiRollPitch = 0.005;
    KiRollPitchSpeed = 0.0001;
    KiYaw = 0.0;
    KiVerticalSpeed = 0.0;

    preErrorRoll = 0;
    preErrorPitch = 0;
    preErrorYaw = 0;
    preErrorVerticalSpeed = 0;
    preErrorRollSpeed = 0;
    preErrorPitchSpeed = 0;

    integralErrorRoll = 0;
    integralErrorPitch = 0;
    integralErrorYaw = 0;
    integralErrorVerticalSpeed = 0;
    integralErrorRollSpeed = 0;
    integralErrorPitchSpeed = 0;

    spRoll = 0;
    spPitch = 0;
    spYaw = 0;
    spPositionX = 0;
    spPositionY = 0;
    spPositionZ = 0;

    speedX = 0;
    speedY = 0;
    preSpeedX = 0;
    preSpeedY = 0;

    //Kalman filter
    filter.initialize(4, 30);
    filterOutput.initialize(4, 10);
    filterSpeeds.initialize(2, 10);
}

void ParrotController::run()
{
    while (true)
    {
        TrackingCommon::mainController->displayParrotControlValues(joypadRoll, joypadPitch, joypadYaw, joypadVerticalSpeed);
        if (connected)
        {
            /////////////////////////////////////////////////
            // Shows Parrot's Sensors
            ////////////////////////////////////////////////

            parrot.getHeliData(phi,psi, theta, altitude, battery, vx, vy, vz);
            //qDebug() << "phi: " << phi << "psi: " << psi << "theta: " << theta << "altitude: " << altitude << "battery: " << battery << "vx: " << vx << "vy: " << vy << "vz: " << vz;
            emit setBatteryChargeValueSignal(battery);            

            if (enableAutomaticControl)// && receivingParrotValues)
            {

                //////////////////////////////////////////////////////////
                // This first control routine assumes target is 0, 0, 0
                ///////////////////////////////////////////////////////////

                // Parrot control if user moves control or if divergence occurs
                if (joypadRoll != 0 || joypadPitch != 0 || joypadYaw != 0 || joypadVerticalSpeed != 0 ) {
                    //qDebug("Landing...");
                    //connected = false;
                    enableAutomaticControl = false;
                    //land();
                    TrackingCommon::mainController->setParrotAutomaticControl(false);
                }
                else
                {
                    if (TrackingCommon::enableHapticControl) {
                        // Range: ([0-800],[0-400])
                        // Map HapticX positive to z negative
                        spPositionZ = -hapticInputX;
                        // Map HapticY positive to z negative
                        spPositionX = -hapticInputY;
                        qDebug() << "Auto mode, with sp " << spPositionZ << ", " << spPositionX;
                    }

                    // Values will be maximum half its maximum value
                    float targetRoll, targetPitch, targetYaw, targetVerticalSpeed;
                    estimateControlVars(targetRoll, targetPitch, targetYaw, targetVerticalSpeed);

                    cv::Mat poseInfo = (cv::Mat_<float>(1, 12) << roll, pitch, yaw, positionX, positionY, positionZ, targetRoll, targetPitch, targetYaw, targetVerticalSpeed, targetRollSpeed, targetPitchSpeed);

                    TrackingCommon::mainController->displayPose(std::string("Poses"), std::string("Parrot"), poseInfo);
                    setParrotAngles(targetRoll, targetPitch, targetYaw, targetVerticalSpeed, 1);
                }
            }
            else
            {
                if (joypadVerticalSpeed < -80.0)
                {
                    land();
                } else if (joypadVerticalSpeed > 15.0 && takeOffFlag == false)
                {
                    takeOff();
                }
                else
                {
                    spPositionX = 0;
                    spPositionY = 0;

                    ((cv::Mat) (TrackingCommon::objectZero * TrackingCommon::getCurrentCameraRotationMatrix()));
                    //bool doHover = joypadRoll == joypadPitch == joypadYaw == joypadVerticalSpeed == 0;
                    setParrotAngles(joypadRoll, joypadPitch, joypadYaw, joypadVerticalSpeed, 1);
                }
            }
        }
        usleep(25000);
    }
}

void ParrotController::estimateControlVars(float &targetRoll, float &targetPitch, float &targetYaw, float &targetVerticalSpeed)
{
    cv::Mat currentPose = (cv::Mat_<float>(4, 1) << positionX, positionZ, pitch, positionY);
    filter.filter(currentPose);
    float fPositionX = filter.FPose.at<float>(0, 0);
    float fPositionZ = filter.FPose.at<float>(1, 0);
    float fPitch = filter.FPose.at<float>(2, 0);
    float fPositionY = filter.FPose.at<float>(3, 0);

    // Building haptic output
    //std::stringstream hapticOutput;
    //hapticOutput << "A" << ((int)fPositionX) << "B" << ((int)fPositionZ);
    //qDebug() << "hapticOutput " << hapticOutput.str().c_str();
    //emit writeParrotPositionToHapticSignal(hapticOutput.str());

    //////////////////////////////////////////////////////////////////////////////
    //   Position Control                                                       //
    //////////////////////////////////////////////////////////////////////////////

    float errorPositionX = fPositionX - spPositionX;
    float errorPositionY = fPositionZ - spPositionZ;

    float theta = fPitch * (CV_PI / 180.0);
    float errorRoll = errorPositionX * std::cos(theta) - errorPositionY * std::sin(theta);
    float errorPitch = errorPositionX * std::sin(theta) + errorPositionY * std::cos(theta);

    targetRollSpeed = KpRollPitch * errorRoll + KdRollPitch * (errorRoll - preErrorRoll) + KiRollPitch * integralErrorRoll;
    targetPitchSpeed = KpRollPitch * errorPitch + KdRollPitch * (errorPitch - preErrorPitch) + KiRollPitch * integralErrorPitch;

    integralErrorRoll += errorRoll;
    integralErrorPitch += errorPitch;

    if (std::abs(targetRollSpeed) > maxRollPitchSpeed)
    {
        targetRollSpeed = (targetRollSpeed > 0 ? 1 : -1) * maxRollPitchSpeed;
    }
    if (std::abs(targetPitchSpeed) > maxRollPitchSpeed)
    {
        targetPitchSpeed = (targetPitchSpeed > 0 ? 1 : -1) * maxRollPitchSpeed;
    }

    float maxVerticalSpeed = 25;
    if (std::abs(verticalSpeed) > maxVerticalSpeed)
    {
        verticalSpeed = (verticalSpeed > 0 ? 1 : -1) * maxVerticalSpeed;
    }

    float errorYaw = spPitch - fPitch;
    if (std::cos(spPitch) == -1 && std::cos(pitch*CV_PI/180) < -0.99)
    {
        errorYaw = 0;
    }
    errorYaw = errorYaw * CV_PI / 180;
    errorYaw = std::atan2(std::sin(errorYaw), std::cos(errorYaw));
    errorYaw = errorYaw * 180 / CV_PI;
    integralErrorYaw += errorYaw;
    targetYaw = normalize(KpYaw * errorYaw + KdYaw * (errorYaw - preErrorYaw) + KiYaw * integralErrorYaw);
    if (std::abs(targetYaw) > 10)
    {
        targetYaw = targetYaw > 0 ? 10 : -10;
    }
    preErrorYaw = errorYaw;

    float errorVerticalSpeed = fPositionY - spPositionY;
    integralErrorVerticalSpeed += errorVerticalSpeed;
    targetVerticalSpeed = normalize(KpVerticalSpeed * errorVerticalSpeed + KdVerticalSpeed * (errorVerticalSpeed - preErrorVerticalSpeed) + KiVerticalSpeed * integralErrorVerticalSpeed);
    preErrorVerticalSpeed = errorVerticalSpeed;

    //////////////////////////////////////////////////////////////////////////////
    //   Speed Control                                                          //
    //////////////////////////////////////////////////////////////////////////////

    // Estimate current speed on each axis
    //speedX = preErrorRoll - errorRoll;
    //speedY = preErrorPitch - errorPitch;

    float deltaX = -(fPositionX - prePositionX);
    float deltaY = -(fPositionZ - prePositionZ);

    // To avoid problems with deltaX = 0, better we use the measured angle
    float rollSpeed = deltaX * std::cos(theta) - deltaY * std::sin(theta);
    float pitchSpeed = deltaX * std::sin(theta) + deltaY * std::cos(theta);

    cv::Mat currentSpeeds = (cv::Mat_<float>(2, 1) << rollSpeed, pitchSpeed);
    filterSpeeds.filter(currentSpeeds);
    rollSpeed = filterSpeeds.FPose.at<float>(0, 0);
    pitchSpeed = filterSpeeds.FPose.at<float>(1, 0);

    float errorRollSpeed = targetRollSpeed - rollSpeed;
    float errorPitchSpeed = targetPitchSpeed - pitchSpeed;

    targetRoll = normalize(KpRollPitchSpeed * errorRollSpeed + KdRollPitchSpeed * (errorRollSpeed - preErrorRollSpeed) + KiRollPitchSpeed * integralErrorRollSpeed);
    targetPitch = normalize(KpRollPitchSpeed * errorPitchSpeed + KdRollPitchSpeed * (errorPitchSpeed - preErrorPitchSpeed) + KiRollPitchSpeed * integralErrorPitchSpeed);
    cv::Mat rollDebug  = (cv::Mat_<float>(1, 5) << rollSpeed, targetRollSpeed, errorRollSpeed, targetRoll, KdRollPitchSpeed);
    TrackingCommon::mainController->displayData(std::string("RollDebug"), rollDebug, DataDisplayer::FULL);

    float maxRollPitchManipulation = 25;
    if (std::abs(targetRoll) > maxRollPitchManipulation)
    {
        targetRoll = (targetRoll > 0 ? 1 : -1) * maxRollPitchManipulation;
    }
    if (std::abs(targetPitch) > maxRollPitchManipulation)
    {
        targetPitch = (targetPitch > 0 ? 1 : -1) * maxRollPitchManipulation;
    }

    preErrorRollSpeed = errorRollSpeed;
    preErrorPitchSpeed = errorPitchSpeed;

    integralErrorRollSpeed += errorRollSpeed;
    integralErrorPitchSpeed += errorPitchSpeed;

    preSpeedX = speedX;
    preSpeedY = speedY;

    if (KpRollPitchSpeed + KdRollPitchSpeed + KiRollPitchSpeed == 0.0)
    {
        targetRoll = rollSpeed;
        targetPitch = pitchSpeed;
    }

    preErrorRoll = errorRoll;
    preErrorPitch = errorPitch;

    prePositionX = fPositionX;
    prePositionZ = fPositionZ;

    //////////////////////////////////////////////////////////////////////////////
    //   Output Filter                                                          //
    //////////////////////////////////////////////////////////////////////////////

    cv::Mat currentOutput = (cv::Mat_<float>(4, 1) << targetRoll, targetPitch, targetYaw, targetVerticalSpeed);
    filterOutput.filter(currentOutput);
    targetRoll = filterOutput.FPose.at<float>(0, 0);
    targetPitch = filterOutput.FPose.at<float>(1, 0);
    targetYaw = filterOutput.FPose.at<float>(2, 0);
    targetVerticalSpeed = filterOutput.FPose.at<float>(3, 0);

    //////////////////////////////////////////////////////////////////////////////
    //   Trajectory Control                                                     //
    //////////////////////////////////////////////////////////////////////////////

    // Check Trajectory Points
    float rollPitchEpsilon = 50.0;
    float yawEpsilon = 15.0;
    float verticalSpeedEpsilon = 40.0;
    if (std::abs(errorPositionX) < rollPitchEpsilon && std::abs(errorPositionY) < rollPitchEpsilon && std::abs(errorYaw) < yawEpsilon && std::abs(errorVerticalSpeed) < verticalSpeedEpsilon)
    {
        // Now we have reached the point
        if (!trajectory.empty() && currentTrajectoryIndex < trajectory.rows)
        {
            TrackingCommon::mainController->updateParrotTrajectoryStatus(currentTrajectoryIndex, true);
            if (currentTrajectoryIndex + 1 < trajectory.rows)
            {
                ++currentTrajectoryIndex;
                spPitch = trajectory.at<float>(currentTrajectoryIndex, 3);
                spPositionY = trajectory.at<float>(currentTrajectoryIndex, 1);
                spPositionX = trajectory.at<float>(currentTrajectoryIndex, 0);
                spPositionZ = trajectory.at<float>(currentTrajectoryIndex, 2);
            }
        }
    }

    cv::Mat controlDebug  = (cv::Mat_<float>(1, 37) <<
                             positionX, positionZ, pitch, positionY, 111, // Read from camera
                             fPositionX, fPositionZ, fPitch, fPositionY, 222, // Read from camera
                             spPositionX, spPositionZ, spPitch, spPositionY, 333,  // Read from camera
                             errorPositionX, errorPositionY, 444,  // For both Roll and Pitch
                             errorRoll, rollSpeed, speedX, errorRollSpeed, targetRoll, 555, // Roll Control
                             errorPitch, pitchSpeed, speedY, errorPitchSpeed, targetPitch, 666, // Pitch Control
                             targetRollSpeed, targetPitchSpeed, 777,
                             errorYaw, targetYaw, // Yaw Control
                             errorVerticalSpeed, targetVerticalSpeed // Vertical Speed Control
                             );
    TrackingCommon::mainController->displayData(std::string("ControlDebug"), controlDebug, DataDisplayer::APPEND);
}

void ParrotController::setJoypadValues(float roll, float pitch, float yaw, float verticalSpeed)
{
    joypadRoll = roll;
    joypadPitch = pitch;
    joypadYaw = yaw;
    joypadVerticalSpeed = verticalSpeed;
    joypadAvailable = true;
}

void ParrotController::setParrotAngles(float roll, float pitch, float yaw, float verticalSpeed, int hover)
{
    parrot.setAngles(roll, pitch, yaw, verticalSpeed, hover);
}

void ParrotController::setAnglesValues(float roll, float pitch, float yaw, float positionX, float positionY, float positionZ)
{
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
    this->positionX = positionX;
    this->positionY = positionY;
    this->positionZ = positionZ;
    receivingParrotValues = true;
}
void ParrotController::setPose(cv::Mat &pose)
{
    // Calculate angle
    pose.copyTo(this->pose);
    TrackingCommon::getAnglesFromTransform(pose, roll, pitch, yaw);
    positionX = pose.at<double>(0, 3);
    positionY = pose.at<double>(1, 3);
    positionZ = pose.at<double>(2, 3);
    receivingParrotValues = true;
}

void ParrotController::setTrajectory(cv::Mat trajectory)
{
    this->trajectory = trajectory.clone();

    qDebug() << "Trajectory is received";
    spPositionX = trajectory.at<float>(0, 0);
    spPositionY = trajectory.at<float>(0, 1);
    spPositionZ = trajectory.at<float>(0, 2);
    spPitch = trajectory.at<float>(0, 3);
    currentTrajectoryIndex = 0;

}

void ParrotController::resetFilter(cv::Mat &pose)
{
    qDebug() << "Resetting Filter";
    filter.setState(pose);
}

void ParrotController::connect()
{
    parrot.connect();
    takeOffFlag = false;
    landFlag = false;
}

void ParrotController::disconnect()
{
    connected = false;
    //parrot.disconnect();
}

void ParrotController::takeOff()
{
    takeOffFlag = true;
    parrot.takeOff();
}

void ParrotController::land()
{
    landFlag = true;
    takeOffFlag = false;
    parrot.land();
}

void ParrotController::connectSlot()
{
    qDebug() << "ConnectSlot";
    connect();
    connected = true;
}

void ParrotController::disconnectSlot()
{
    qDebug() << "DisconnectSlot";
    disconnect();
}

void ParrotController::takeOffSlot()
{
    qDebug() << "takeOffSlot";
    takeOff();
}

void ParrotController::landSlot()
{
    qDebug() << "landSlot";
    land();
}

bool ParrotController::isConnected()
{
    return connected;
}

void ParrotController::setAutomaticControl(bool enableAutomaticControl)
{
    this->enableAutomaticControl = enableAutomaticControl;
}

float ParrotController::normalize(float input)
{
    return input > 100.0 ? 100.0 : (input < -100.0 ? -100.0 : input);
}

void ParrotController::setJoypadValuesSlot(float roll, float pitch, float yaw, float verticalSpeed)
{
    setJoypadValues(normalize(roll), normalize(pitch), normalize(yaw), normalize(verticalSpeed));
}

void ParrotController::setHapticInputSlot(int x, int y)
{
    hapticInputX = x;
    hapticInputY = y;
}

void ParrotController::setAnglesValuesSlot(float roll, float pitch, float yaw, float positionX, float positionY, float positionZ)
{
    setAnglesValues(roll, pitch, yaw, positionX, positionY, positionZ);
}

void ParrotController::setPoseSlot(cv::Mat pose)
{
    setPose(pose);
}

void ParrotController::setTrajectorySlot(cv::Mat trajectory)
{
    setTrajectory(trajectory);
}

void ParrotController::resetFilterSlot(cv::Mat pose)
{
    resetFilter(pose);
    integralErrorPitch = 0;
    integralErrorRoll = 0;
    integralErrorPitchSpeed = 0;
    integralErrorRollSpeed = 0;
}

void ParrotController::rawToMat( Mat &destImage, CRawImage* sourceImage)
{
    uchar *pointerImage = destImage.ptr(0);

    for (int i = 0; i < 240*320; i++)
    {
        pointerImage[3*i] = sourceImage->data[3*i+2];
        pointerImage[3*i+1] = sourceImage->data[3*i+1];
        pointerImage[3*i+2] = sourceImage->data[3*i];
    }
}

void ParrotController::setControlVariablesSlot(float KpRollPitch, float KpRollPitchSpeed, float KpYaw, float KpVerticalSpeed, float KdRollPitch, float KdRollPitchSpeed, float KdYaw, float KdVerticalSpeed, float KiRollPitch, float KiRollPitchSpeed, float KiYaw, float KiVerticalSpeed, float inputFilterScale, float outputFilterScale, float maxRollPitchSpeed)
{
    this->KpRollPitch = KpRollPitch;
    this->KpRollPitchSpeed = KpRollPitchSpeed;
    this->KpYaw = KpYaw;
    this->KpVerticalSpeed = KpVerticalSpeed;

    this->KdRollPitch = KdRollPitch;
    this->KdRollPitchSpeed = KdRollPitchSpeed;
    this->KdYaw = KdYaw;
    this->KdVerticalSpeed = KdVerticalSpeed;

    this->KiRollPitch = KiRollPitch;
    this->KiRollPitchSpeed = KiRollPitchSpeed;
    this->KiYaw = KiYaw;
    this->KiVerticalSpeed = KiVerticalSpeed;

    this->maxRollPitchSpeed = maxRollPitchSpeed;

    filter.setScale(inputFilterScale);
    filterOutput.setScale(outputFilterScale);
    filterSpeeds.setScale(outputFilterScale);

    qDebug() << "New control values: " <<
                this->KpRollPitch << "," << this->KpRollPitchSpeed << "," << this->KpYaw << "," << this->KpVerticalSpeed << "," <<
                this->KdRollPitch << "," << this->KdRollPitchSpeed << "," << this->KdYaw << "," << this->KdVerticalSpeed << "," <<
                this->KiRollPitch << "," << this->KiRollPitchSpeed << "," << this->KiYaw << "," << this->KiVerticalSpeed << "," <<
                inputFilterScale << "," << outputFilterScale << "," << maxRollPitchSpeed;
}

void ParrotController::getControlVariablesSlot(float &KpRollPitch, float &KpRollPitchSpeed, float &KpYaw, float &KpVerticalSpeed, float &KdRollPitch, float &KdRollPitchSpeed, float &KdYaw, float &KdVerticalSpeed, float &KiRollPitch, float &KiRollPitchSpeed, float &KiYaw, float &KiVerticalSpeed)
{
    KpRollPitch = this->KpRollPitch ;
    KpRollPitchSpeed = this->KpRollPitchSpeed ;
    KpYaw = this->KpYaw;
    KpVerticalSpeed = this->KpVerticalSpeed;

    KdRollPitch = this->KdRollPitch;
    KdRollPitchSpeed = this->KdRollPitchSpeed;
    KdYaw = this->KdYaw;
    KdVerticalSpeed = this->KdVerticalSpeed;

    KiRollPitch = this->KiRollPitch;
    KiRollPitchSpeed = this->KiRollPitchSpeed;
    KiYaw = this->KiYaw;
    KiVerticalSpeed = this->KiVerticalSpeed;
}
