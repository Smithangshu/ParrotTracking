#include "common/MainController.h"
#include "common/TrackingCommon.h"
#include "gui/WindowCallbacks.h"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <QMessageBox>
#include <QDebug>
#include <QImage>

MainController::MainController()
{
    QThread();
    programStatus = VIDEO_STARTED;
    mainCVWindowName = std::string("TrackerWindow");
    videoMode = VIDEO_CAMERA;
    testMode = false;
    skipPauseOnFirstFrame = false;
}

void MainController::prepareImageWindow(std::string windowTitle)
{
    imageWindowsInfoIterator = imageWindowsInfo.find(windowTitle);
    if (imageWindowsInfoIterator == imageWindowsInfo.end())
    {
        imageWindows[windowTitle] = new ImageWindow;
        imageWindows[windowTitle]->setWindowTitle(QString(windowTitle.c_str()));
        imageWindows[windowTitle]->show();
        imageWindowsInfo[windowTitle].windowId = imageWindowsInfo.size();
        imageWindowsInfo[windowTitle].windowTitle = imageWindowsInfo.size();
    }
}

void MainController::destroyImageWindows()
{
    for (imageWindowsInfoIterator = imageWindowsInfo.begin(); imageWindowsInfoIterator != imageWindowsInfo.end(); imageWindowsInfoIterator++)
    {
        std::string key = imageWindowsInfoIterator->first;
        imageWindows[key]->close();
    }
    imageWindowsInfo.clear();
    imageWindows.clear();

}

void MainController::displayImageWindow(std::string windowTitle, cv::Mat image)
{
    QImage imageP;
    cv::Mat targetImage;
    if (!image.isContinuous())
    {
        targetImage.create(image.size(), CV_8UC3);
        image.copyTo(targetImage);
    }
    else
    {
        targetImage = image;
    }

    if (image.channels() == 1)
    {
        imageP = QImage((const unsigned char*)targetImage.data, targetImage.cols, targetImage.rows, QImage::Format_Indexed8);
    }
    else
    {
        imageP = QImage((const unsigned char*)targetImage.data, targetImage.cols, targetImage.rows, QImage::Format_RGB888);
    }

    emit imageWindowReadySignal(windowTitle, imageP);
}

void MainController::displayImageWindowCV(std::string windowTitle, cv::Mat image)
{
    emit displayCVImageSignal(windowTitle, image);
}


void MainController::setImageWindowMouseCallBack(std::string windowTitle, WindowCallBack* windowCallBack)
{
    imageWindows[windowTitle]->setWindowCallback(windowCallBack);
}

void MainController::displayData(std::string tabName, cv::Mat mat, DataDisplayer::DataDisplayModes dataDisplayMode)
{
    emit dataDisplayerSignal(tabName, mat, dataDisplayMode);
}

void MainController::displayCoordinates(std::string tabName, std::string displayName, cv::Mat &displayData)
{
    emit displayCoordinatesSignal(tabName, displayName, displayData);
}

void MainController::displayPose(std::string tabName, std::string displayName, cv::Mat &displayData)
{
    emit displayPoseSignal(tabName, displayName, displayData);
}

void MainController::displayParrotControlValues(int roll, int pitch, int yaw, int verticalSpeed)
{
    emit displayParrotControlValuesSignal(roll, pitch, yaw, verticalSpeed);
}

void MainController::setParrotAutomaticControl(bool automaticControl)
{
    emit setParrotAutomaticControlSignal(automaticControl);
}

void MainController::setParrotConnectionStatus(bool connected)
{
    emit setParrotConnectionStatusSignal(connected);
}

void MainController::setTrajectory(cv::Mat trajectory)
{
    emit setTrajectorySignal(trajectory);
}


void MainController::updateParrotTrajectoryStatus(int point, bool completed)
{
    emit updateParrotTrajectoryStatusSignal(point, completed);
}

void MainController::setParrotControlVariables(float KpRollPitch, float KpRollPitchSpeed, float KpYaw, float KpVerticalSpeed, float KdRollPitch, float KdRollPitchSpeed, float KdYaw, float KdVerticalSpeed, float KiRollPitch, float KiRollPitchSpeed, float KiYaw, float KiVerticalSpeed, float inputFilterScale, float outputFilterScale, float maxRollPitchSpeed)
{
    emit setParrotControlVariablesSignal(KpRollPitch, KpRollPitchSpeed, KpYaw, KpVerticalSpeed, KdRollPitch, KdRollPitchSpeed, KdYaw, KdVerticalSpeed, KiRollPitch, KiRollPitchSpeed, KiYaw, KiVerticalSpeed, inputFilterScale, outputFilterScale, maxRollPitchSpeed);
}

void MainController::getParrotControlVariables(float &KpRollPitch, float &KpRollPitchSpeed, float &KpYaw, float &KpVerticalSpeed, float &KdRollPitch, float &KdRollPitchSpeed, float &KdYaw, float &KdVerticalSpeed, float &KiRollPitch, float &KiRollPitchSpeed, float &KiYaw, float &KiVerticalSpeed)
{

}

void MainController::setCameraIndex(int index)
{
    TrackingCommon::cameraIndex = index;
}

void MainController::setMediaFileName(std::string fileName)
{
    TrackingCommon::videoFileName = fileName;
}

void MainController::setVideoMode(enum VideoModes videoMode)
{
    this->videoMode = videoMode;
}

void MainController::startVideo()
{
    if (videoMode == VIDEO_CAMERA && programStatus != VIDEO_PLAYING)
    {
        TrackingCommon::capture = cv::VideoCapture(TrackingCommon::cameraIndex);
        TrackingCommon::totalWindows = 0;
        programStatus = VIDEO_PLAYING;
    }
    else if (videoMode == VIDEO_FILE)
    {
        if (TrackingCommon::videoFileName != "")
        {
            TrackingCommon::capture = cv::VideoCapture(TrackingCommon::videoFileName.c_str());
            TrackingCommon::totalWindows = 0;
            programStatus = VIDEO_PLAYING;
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Please provide a media file");
            msgBox.exec();
            programStatus = VIDEO_STOPPED;
        }
    }
}

void MainController::pauseVideo()
{
    programStatus = VIDEO_PAUSED;
}


void MainController::stopVideo()
{
    programStatus = VIDEO_STOPPED;
}

void MainController::run()
{

    qDebug() << "Main tracking loop started";
    bool firstFrame = true;
    clock_t startTime = clock();

    TrackingCommon::capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    TrackingCommon::capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    TrackingCommon::capture.set(CV_CAP_PROP_BRIGHTNESS, 0.4);
    TrackingCommon::capture.set(CV_CAP_PROP_HUE, 0.5);
    TrackingCommon::capture.set(CV_CAP_PROP_GAIN, 0.4);
    TrackingCommon::capture.set(CV_CAP_PROP_GAMMA, 0.4);
    TrackingCommon::capture.set(CV_CAP_PROP_FPS, 30);
    //TrackingCommon::capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    //TrackingCommon::capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);


    Filter test;
    double inputs[20] = {0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45, 0.39, 0.50, 0.48, 0.29, 2.25, 2.32, 0.34, 0.48, 0.41, 0.45};
    test.initialize(1, 0.909);
    for (int i = 0; i < 20; ++i) {
        cv::Mat testVal = cv::Mat(1, 1, CV_32F);
        testVal.setTo(inputs[i]);
        test.filter(testVal);
        qDebug() << " Values: " << test.K.at<float>(0, 0) << ", " << inputs[i] << ", " << test.FPose.at<float>(0, 0);
    }

    while (programStatus == VIDEO_PLAYING || programStatus == VIDEO_PAUSED)
    {
        if (programStatus != VIDEO_PAUSED)
        {
            TrackingCommon::capture >> TrackingCommon::frame;
            //cv::GaussianBlur(TrackingCommon::frame, TrackingCommon::frame, cv::Size(9, 9), 0);


            if (!TrackingCommon::frame.empty()) {
                if (TrackingCommon::enableDeinterlace)
                    TrackingCommon::deinterlace(TrackingCommon::frame);

                if (TrackingCommon::enableUndistort && TrackingCommon::calibrated)
                {
                    cv::undistort(TrackingCommon::frame, TrackingCommon::currentFrame, TrackingCommon::cameraMatrix, TrackingCommon::distCoeffs);
                    TrackingCommon::currentFrame.copyTo(TrackingCommon::frame);
                }
                else
                {
                    TrackingCommon::frame.copyTo(TrackingCommon::currentFrame);
                }
            }
        }
        else
        {
            TrackingCommon::frame.copyTo(TrackingCommon::currentFrame);
        }

        if (!TrackingCommon::frame.empty())
        {
            //TrackingCommon::fileOutput.open("Out.avi", -1, 30, TrackingCommon::frame.size(), true);

            if (firstFrame)
            {
                if (testMode)
                {
                    TrackingCommon::testsController.launchTest(testIndex);
                }
                else
                {
                    displayImageWindow(mainCVWindowName, TrackingCommon::frame);
                    TrackingCallback* trackingCallbackObject = new TrackingCallback();
                    emit imageWindowMouseCallBackSignal(mainCVWindowName, trackingCallbackObject);
                }

                firstFrame = false;

                if (!skipPauseOnFirstFrame)
                {
                    if (TrackingCommon::pauseOnFirstFrame)
                    {
                        programStatus = VIDEO_PAUSED;
                        skipPauseOnFirstFrame = true;
                    }
                }
            }

            // If current mode is homography discovery, draw corresponding circles
            if (TrackingCommon::programMode == TrackingCommon::HOMOGRAPHY_DISCOVERY)
            {
                for (int i = 0; i < TrackingCommon::positionEstimatorController.positionEstimator.totalHomographyTraining; ++i)
                {
                    cv::circle(TrackingCommon::currentFrame, TrackingCommon::positionEstimatorController.positionEstimator.homographyTrainingImage[i], 5, cv::Scalar(0, 255, 255), 2);
                    std::stringstream windowIndex; windowIndex << (i + 1);
                    cv::Point windowIndexPosition = cv::Point(TrackingCommon::positionEstimatorController.positionEstimator.homographyTrainingImage[i].x + 10, TrackingCommon::positionEstimatorController.positionEstimator.homographyTrainingImage[i].y + 10);
                    cv::putText(TrackingCommon::currentFrame, windowIndex.str(), windowIndexPosition, 0, 0.5, cv::Scalar(0, 255, 255), 1);
                }
            }

            // This handles rectangular selection window and zoom
            if (TrackingCommon::drawingBox)
            {
                TrackingCommon::drawBox(TrackingCommon::currentFrame, TrackingCommon::selectionBox);
            }
            else if (TrackingCommon::zoomActive)
            {
                TrackingCommon::zoomImage(TrackingCommon::mouseX, TrackingCommon::mouseY);
            }

            // Handle creation of tracking windows
            if (TrackingCommon::createTrackingWindow)
            {
                if (TrackingCommon::trackingMode == TrackingCommon::TRACKING_MS)
                {
                    MeanShiftWithScaling* tracker = new MeanShiftWithScaling();
                    tracker->setTargetObject(TrackingCommon::frame, TrackingCommon::selectionBox);
                    TrackingCommon::trackingWindows[TrackingCommon::totalWindows] = tracker;
                    ++TrackingCommon::totalWindows;
                    // 2D Localization special case: Let two versions of the tracker to run on same object
                    if (TrackingCommon::enableScalingWithHomography && TrackingCommon::displayUnscaled)
                    {
                        qDebug() << "Creating ghost window";
                        MeanShiftTracker* trackerGhost = new MeanShiftTracker();
                        trackerGhost->highlightColor = cv::Scalar(0, 255, 0);
                        trackerGhost->setTargetObject(TrackingCommon::frame, TrackingCommon::selectionBox);
                        TrackingCommon::trackingWindows[TrackingCommon::totalWindows] = trackerGhost;
                        ++TrackingCommon::totalWindows;
                    }
                }
                else if (TrackingCommon::trackingMode == TrackingCommon::TRACKING_ESM)
                {
                    ESMTrackerPyramidal* tracker = new ESMTrackerPyramidal();
                    //ESMTrackerPyramidalF* tracker = new ESMTrackerPyramidal();
                    tracker->setTopPyramid(2);
                    tracker->enableColor(true);
                    tracker->setTargetObject(TrackingCommon::frame, TrackingCommon::selectionBox);
                    TrackingCommon::trackingWindows[TrackingCommon::totalWindows] = tracker;
                    ++TrackingCommon::totalWindows;

                    if (TrackingCommon::show3DReconstruction)
                    {
                        qDebug() << "Setting up 3DReconstruction";
                        PlaneDrawing* planeDrawing = new PlaneDrawing();
                        TrackingCommon::drawings3D[TrackingCommon::totalDrawing3D] = planeDrawing;
                        planeDrawing->setTracker(tracker);
                        cv::Mat selectedTarget = tracker->targetObject.clone();
                        planeDrawing->setGLParameters(Drawing3D::MULTI_VIEW, "3D Reconstruction", selectedTarget);

                        ++TrackingCommon::totalDrawing3D;
                    }
                }
                else if (TrackingCommon::trackingMode == TrackingCommon::TRACKING_CUBOID)
                {
                    qDebug() << "Setting up selected Cuboid";
                    CuboidDrawing* cuboid = new CuboidDrawing();
                    TrackingCommon::drawings3D[TrackingCommon::totalDrawing3D] = cuboid;
                    cuboid->setDefaultCamera();

                    CuboidTracker* cuboidTracker = cuboid->getCuboidTracker();
                    TrackingCommon::trackingWindows[TrackingCommon::totalWindows] = cuboidTracker;
                    ++TrackingCommon::totalWindows;
                    ++TrackingCommon::totalDrawing3D;

                    //cuboidTracker->globalTracker.setTargetObject(TrackingCommon::currentFrame, test.globalTrackerSelectionRectangle);

                    cuboidTracker->cuboidTrackingMode = CuboidTracker::MULTI_VIEW;
                    cuboidTracker->cuboidTrackingOptimization = CuboidTracker::MOST_ALIGNED_NORMAL;

                    TestsController::GenericTest test = TrackingCommon::testsController.models[TrackingCommon::cuboidSelectedModel];
                    test.selectedFace = TrackingCommon::cuboidSelectedFace;

                    cuboid->prepareTextures(test.textures[0], test.textures[1], test.textures[2], test.textures[3], test.textures[4], test.textures[5]);
                    cuboidTracker->setFacesDimensions(cv::Size2i(test.textures[0].cols, test.textures[0].rows), cv::Size2i(test.textures[1].cols, test.textures[1].rows), cv::Size2i(test.textures[4].cols, test.textures[4].rows));
                    cuboidTracker->setModel(test.selectedFace, test.textures[0], test.textures[1], test.textures[2], test.textures[3], test.textures[4], test.textures[5]);

                    // Set the initial homography
                    qDebug() << "test.selectedFace " << test.selectedFace;
                    cuboidTracker->setBaseTracker(test.selectedFace);
                    cuboidTracker->trackers[test.selectedFace].setHomography(TrackingCommon::discoveredHomography);

                    // Calculate initial cuboid state
                    cuboidTracker->enableFaceLock(TrackingCommon::enableLockFace);
                    cuboidTracker->calculateTransformationFromHomography();
                    cuboidTracker->calculateAllTransformationsFromSingleTransformation();
                    cuboidTracker->calculateHomographiesFromTransformation();

                    cuboid->setGLParameters(Drawing3D::MULTI_VIEW, "Cuboid 3D Reconstruction");

                }
                TrackingCommon::createTrackingWindow = false;
            }

            // Center the camera and homography interpolation
            if (TrackingCommon::centerCamera && TrackingCommon::pTZCameraController.isConnected() && TrackingCommon::totalWindows > 0)
            {
                PTZCameraController::CameraMovement cameraMovement;
                cameraMovement.cameraMoved = false;
                if (TrackingCommon::pTZCameraController.isReady())
                {
                    std::vector<cv::Point2f> windowsCenterPoints;
                    for (int i = 0; i < TrackingCommon::totalWindows; ++i)
                    {
                        if (TrackingCommon::trackingWindows[i]->type != Tracker::TYPE_CUBOID && TrackingCommon::trackingWindows[i]->type != Tracker::TYPE_CUBOID_SINGLE_FACE)
                        {
                           windowsCenterPoints.push_back(TrackingCommon::trackingWindows[i]->getObjectCenter());
                        }
                    }

                    cameraMovement = TrackingCommon::pTZCameraController.centerCamera(windowsCenterPoints);
                }

                if (TrackingCommon::positionEstimatorController.isReady() && TrackingCommon::calibrated)
                {
                    if (TrackingCommon::enableHomographyInterpolation)
                    {
                        if (cameraMovement.cameraMoved)
                        {
                            TrackingCommon::positionEstimatorController.setCameraMovement(cameraMovement);
                            TrackingCommon::positionEstimatorController.startHomographyInterpolation();
                        }
                        else
                        {
                            TrackingCommon::positionEstimatorController.updateHomographyInterpolation();
                        }
                    }
                    else if (cameraMovement.cameraMoved)
                    {
                        TrackingCommon::positionEstimatorController.updateTargetHomography(cameraMovement.newPan, cameraMovement.newTilt, true, true);
                    }
                }
            }

            // Generic Tracker (Will take latest Descendant)
            for (int i = 0; i < TrackingCommon::totalWindows; ++i)
            {
                Tracker* tracker = TrackingCommon::trackingWindows[i];
                tracker->trackObject(TrackingCommon::frame);
                if (TrackingCommon::showMeanShiftModel && programMode == TrackingCommon::TRACKING_MS)
                {
                    displayImageWindow(tr("MeanShiftModel %1").arg(i).toStdString(), tracker->targetObject);
                }
                if (programMode == TrackingCommon::TRACKING_MS && TrackingCommon::positionEstimatorController.isReady())
                {
                    if (TrackingCommon::enableScalingWithHomography && TrackingCommon::displayUnscaled && i % 2 == 0 || !TrackingCommon::displayUnscaled)
                    {
                        std::stringstream windowIndex; windowIndex << "Object" << (i + 1);
                        cv::Mat data = TrackingCommon::calculate2DPosition(tracker->getObjectCenter());
                        if (TrackingCommon::displayUnscaled)
                        {
                            // If we are displaying both versions, then extend matrix
                            cv::Mat allDisplayData = cv::Mat(1, 28, CV_32F);
                            cv::Mat dataGhost = TrackingCommon::calculate2DPosition(TrackingCommon::trackingWindows[i + 1]->getObjectCenter());
                            for (int x = 0; x < 14; ++x)
                            {
                                allDisplayData.at<float>(0, x) = data.at<float>(0, x);
                                allDisplayData.at<float>(0, x + 14) = dataGhost.at<float>(0, x);
                            }
                            //displayCoordinates("Coordinates", windowIndex.str(), allDisplayData);
                            data = allDisplayData;
                        }
                        displayCoordinates("Coordinates", windowIndex.str(), data);

                        if (TrackingCommon::enableSendingPositionData)
                        {
                            //usleep(50000);
//                            1 byte  INICIO AA
//                            4 byte ID        00000810
//                            1 byte rft        00
//                            1 byte lengh   08
//                            8 byte  datos (sólo x y y, no se como se envía el dato)
//                            1 byte  final    BB

                            std::stringstream commandBuilder;
                            commandBuilder << "AA";
                            //commandBuilder << "00000810";
                            commandBuilder << "00000833";
                            commandBuilder << "00";
                            commandBuilder << "08";
                            // Add pan tilt speed
                            commandBuilder << std::setw(8) << std::setfill('0') << 1234;
                            commandBuilder << std::setw(8) << std::setfill('0') << 5678;
                            //commandBuilder << std::setw(8) << std::setfill('0') << std::setprecision(3) << std::fixed << 123.45678;
                            //commandBuilder << std::setw(8) << std::setfill('0') << std::setprecision(3) << std::fixed << 987.65432;
                            commandBuilder << "BB";
                            qDebug() << "Char: " << commandBuilder.str().c_str();
                            //QByteArray text = QByteArray::fromHex(commandBuilder.str().c_str());
                            QByteArray text = QByteArray(commandBuilder.str().c_str());
                            TrackingCommon::gpSerialPort->write(text);
                        }

                        if (TrackingCommon::recordPositionEstimation)
                        {
                            displayData(windowIndex.str(), data, DataDisplayer::APPEND);
                        }
                    }
                }
            }

            // Assume there is only one object (if more, this loop would overwrite the pose, since
            // it would take the last one
            for (int i = 0; i < TrackingCommon::totalDrawing3D && !TrackingCommon::searchPlaneLines; ++i)
            {
                if (TrackingCommon::markObjectZero)
                {
                    TrackingCommon::setZero(((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransform);
                    //TrackingCommon::setZero(((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.transform);
                    emit setZeroSignal();
                    cv::Mat zeroPose = cv::Mat(4, 1, CV_32F);
                    zeroPose.setTo(0);
                    emit resetParrotFilterSignal(zeroPose);
                }

                // Parrot box correction
                //cv::Mat boxCorrection = TrackingCommon::buildRotationMatrix(0, -2.5 * (CV_PI / 180.0));

                // Automatic parrot control
                //cv::Mat pose = TrackingCommon::objectZero * TrackingCommon::getCurrentCameraRotationMatrix() * ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransform;
                //cv::Mat pose = TrackingCommon::objectZero * ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.transform;
                cv::Mat pose = TrackingCommon::objectZero * ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransform;
                cv::Mat poseAlternate;
                if (TrackingCommon::enableMultipleFaceTracking && !((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransformAlternate.empty())
                {
                    poseAlternate = TrackingCommon::objectZero * ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransformAlternate;
                }
                //cv::Mat pose = (((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.transform) * TrackingCommon::objectZero;
                //cv::Mat pose = TrackingCommon::objectZero * (((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.transform);

                //displayData("Corr", boxCorrection, DataDisplayer::FULL);
                //displayData("Pose", pose, DataDisplayer::FULL);
                float roll, pitch, yaw, tx, ty, tz;

                displayData("Pose", pose, DataDisplayer::APPEND);
                for (int m = 0; m < 6 && false; ++m)
                {
                    cv::Mat faceTransform = ((CuboidTrackerMultipleFace*) ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker)->cuboidReconstructions[m];
                    if (!faceTransform.empty())
                    {
                        faceTransform = TrackingCommon::objectZero * faceTransform;
                        TrackingCommon::getAnglesFromTransform(faceTransform, roll, pitch, yaw);
                        tx = pose.at<double>(0, 3);
                        ty = pose.at<double>(1, 3);
                        tz = pose.at<double>(2, 3);
                        cv::Mat poseByDim =(cv::Mat_<float>(1, 6) << roll, pitch, yaw, tx, ty, tz);
                        std::string tabName = "Cub"; tabName += '0' + m; TrackingCommon::mainController->displayData(tabName, poseByDim, DataDisplayer::APPEND);
                    }
                }
                //displayData("H", (((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.homography), DataDisplayer::FULL);
                //displayData("t", ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransform, DataDisplayer::FULL);

                //TrackingCommon::getAnglesFromTransform(((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransform, roll, pitch, yaw);
                //TrackingCommon::getAnglesFromTransform(((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.transform, roll, pitch, yaw);
                TrackingCommon::getAnglesFromTransform(pose, roll, pitch, yaw);
                float realWorldFactor = 13.07 / 116.0;
                //realWorldFactor = 1.0;
                tx = pose.at<double>(0, 3) * realWorldFactor;
                ty = pose.at<double>(1, 3) * realWorldFactor;
                tz = pose.at<double>(2, 3) * realWorldFactor;
                std::stringstream anotations;
                anotations << "Pose: "
                           << std::setw(4) << std::setprecision(2) << std::fixed << roll << ", "
                           << std::setw(4) << std::setprecision(2) << std::fixed << pitch << ", "
                           << std::setw(4) << std::setprecision(2) << std::fixed << yaw << ", "
                           << std::setw(4) << std::setprecision(2) << std::fixed << tx << ", "
                           << std::setw(4) << std::setprecision(2) << std::fixed << ty << ", "
                           << std::setw(4) << std::setprecision(2) << std::fixed << tz << ", ";

                cv::Mat poseByDim =(cv::Mat_<float>(1, 6) << roll, pitch, yaw, tx, ty, tz);
                displayData("PoseByDim", poseByDim, DataDisplayer::APPEND);

                if (TrackingCommon::enableMultipleFaceTracking && !poseAlternate.empty())
                {
                    TrackingCommon::getAnglesFromTransform(poseAlternate, roll, pitch, yaw);
                    tx = pose.at<double>(0, 3);
                    ty = pose.at<double>(1, 3);
                    tz = pose.at<double>(2, 3);
                    cv::Mat poseAltByDim =(cv::Mat_<float>(1, 6) << roll, pitch, yaw, tx, ty, tz);
                    displayData("PoseAltByDim", poseAltByDim, DataDisplayer::APPEND);
                }

                //cv::rectangle(TrackingCommon::currentFrame, cv::Rect(0, TrackingCommon::currentFrame.rows - 28, TrackingCommon::currentFrame.cols, 28), cv::Scalar(255, 255, 255), -1);
                //cv::putText(TrackingCommon::currentFrame, anotations.str(), cv::Point(20, TrackingCommon::currentFrame.rows - 8), FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 2);

                // Print on screen the path
                if (TrackingCommon::enablePathDrawing)
                    TrackingCommon::drawPath(TrackingCommon::currentFrame, ((cv::Mat) TrackingCommon::objectZero.inv()));

                // At the moment, assume there will be only CuboidParrot instance
                emit setParrotPoseSignal(pose);
                //emit setParrotValuesSignal(angleX, angleY, angleZ, pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
            }

            if (TrackingCommon::findModels)
            {
                TrackingCommon::planeDetector.addImage(TrackingCommon::frame);
                TrackingCommon::planeDetector.findPlanesFlowHomography(TrackingCommon::frame);
            }

            if (TrackingCommon::searchPlaneLines)
            {
                TrackingCommon::planeDetector.setMethod(PlaneDetector::BINARY_SEARCH_DISTINCT);
                TrackingCommon::planeDetector.addImage(TrackingCommon::frame);
                if (TrackingCommon::planeDetector.findPlanes(TrackingCommon::currentFrame))
                {
                    if (TrackingCommon::calibrated)
                    {
                        if (TrackingCommon::searchPlaneLinesFirstTime)
                        {
                            std::stringstream fullTexturePath;
                            fullTexturePath << TrackingCommon::testsController.basePath << "Graphics/ct.jpg";
                            cv::Mat templateImage = cv::imread(fullTexturePath.str());
                            PlaneDrawing* planeDrawing = new PlaneDrawing();
                            planeDrawing->enableTracker(false);
                            TrackingCommon::drawings3D[TrackingCommon::totalDrawing3D] = planeDrawing;
                            planeDrawing->setGLParameters(Drawing3D::MULTI_VIEW, "JaguarBands", templateImage);

                            ++TrackingCommon::totalDrawing3D;
                            if (TrackingCommon::show3DReconstruction)
                            {
                                TrackingCommon::mainController->enable3DShowReconstruction(true);
                                TrackingCommon::show3DReconstruction = true;
                            }
                            TrackingCommon::searchPlaneLinesFirstTime = false;
                        }
                        if (TrackingCommon::totalDrawing3D > 0)
                        {
                            TrackingCommon::planeDetector.foundTransform.copyTo(((PlaneDrawing*) TrackingCommon::drawings3D[TrackingCommon::totalDrawing3D - 1])->W);
                        }
                    }
                }
            }

            for (int i = 0; i < TrackingCommon::totalWindows; ++i)
            {
                Tracker* tracker = TrackingCommon::trackingWindows[i];
                // Just drawing the box
                tracker->highlightObject(TrackingCommon::currentFrame);
            }

            if (TrackingCommon::show3DReconstruction)
            {
                //emit create3DReconstructionWindowSignal();
                //TrackingCommon::show3DReconstruction = false;
                if (!glThread.isRunning())
                {
                    glThread.start(QThread::LowPriority);
                }
            }

            // Temp: Display Segmentation
//            qDebug() << "Starting Segmentation";
//            cv::Mat frameReduced = cv::Mat(TrackingCommon::frame.rows/2, TrackingCommon::frame.cols/2, TrackingCommon::frame.type());
//            cv::pyrDown(TrackingCommon::frame, frameReduced);

//            cv::Mat frame4C;
//            std::vector<cv::Mat> image;
//            cv::split(frameReduced, image);
//            cv::Mat empty4C = ((cv::Mat)image[0]).clone();
//            image.push_back(empty4C);
//            cv::merge(image, frame4C);

//            cv::Mat currentFrame4C = frame4C.clone();
//            qDebug() << "frame4C type: " << frame4C.type() << " vs " << CV_8UC4 << " or " << TrackingCommon::frame.type();
//            cv::gpu::GpuMat gFrame(frame4C);

//            cv::gpu::meanShiftSegmentation(gFrame, currentFrame4C, 7, 7, 20);

//            std::vector<cv::Mat> imageRes;
//            cv::split(currentFrame4C, imageRes);
//            imageRes.pop_back();
//            cv::merge(imageRes, TrackingCommon::currentFrame);

//            qDebug() << "Ended Segmentation";

            displayImageWindow(mainCVWindowName, TrackingCommon::currentFrame);

            //displayImageWindow(std::string("Original"), TrackingCommon::frame);
            //TrackingCommon::fileOutput << TrackingCommon::frame;
            msleep(10);
        }
        else
        {
            programStatus = VIDEO_STOPPED;
            qDebug() << "Total time: " << (clock() - startTime) / CLOCKS_PER_SEC;
            break;
        }
    }

    if (programStatus == VIDEO_STOPPED)
    {
        TrackingCommon::capture.release();
        glThread.glThreadStatus = GLThread::STOPPED;
        while (glThread.isRunning())
        {
            usleep(2000);
        }
        TrackingCommon::totalWindows = 0;
        TrackingCommon::totalDrawing3D = 0;
        testMode = false;
        skipPauseOnFirstFrame = false;
        emit imageWindowsDestroySignal();
    }

    qDebug() << "Exiting Main Tracking Thread";
    return;
}

bool MainController::loadHomography(std::string fileName)
{
    cv::FileStorage fileStorage = cv::FileStorage(fileName, CV_STORAGE_READ);
    if (fileStorage.isOpened())
    {
        fileStorage["H"] >> TrackingCommon::positionEstimatorController.positionEstimator.H;
        fileStorage["H_orig"] >> TrackingCommon::positionEstimatorController.positionEstimator.H_orig;
        fileStorage["H"] >> TrackingCommon::positionEstimatorController.positionEstimator.H_interpolated;
        fileStorage["H_inv"] >> TrackingCommon::positionEstimatorController.positionEstimator.H_inv;
        fileStorage["H_inv_orig"] >> TrackingCommon::positionEstimatorController.positionEstimator.H_inv_orig;
        fileStorage["H_inv"] >> TrackingCommon::positionEstimatorController.positionEstimator.H_inv_interpolated;
        fileStorage.release();
        TrackingCommon::positionEstimatorController.positionEstimator.setReady(true);
        TrackingCommon::lastUsedHomography = fileName;

        // Apply Home Correction (Remove offset)
        if (TrackingCommon::calibrated)
        {
            TrackingCommon::positionEstimatorController.updateTargetHomography(-TrackingCommon::homePan, -TrackingCommon::homeTilt, true, true);
            TrackingCommon::positionEstimatorController.positionEstimator.H.copyTo(TrackingCommon::positionEstimatorController.positionEstimator.H_orig);
            TrackingCommon::positionEstimatorController.positionEstimator.H_inv.copyTo(TrackingCommon::positionEstimatorController.positionEstimator.H_inv_orig);
            TrackingCommon::positionEstimatorController.updateTargetHomography(TrackingCommon::homePan, TrackingCommon::homeTilt, true, true);
        }
        return true;
    }
    return false;
}

void MainController::saveHomography(std::string fileName)
{
    cv::FileStorage fileStorage = cv::FileStorage(fileName, CV_STORAGE_WRITE);
    fileStorage << "H" << TrackingCommon::positionEstimatorController.positionEstimator.H;
    fileStorage << "H_orig" << TrackingCommon::positionEstimatorController.positionEstimator.H_orig;
    fileStorage << "H_inv" << TrackingCommon::positionEstimatorController.positionEstimator.H_inv;
    fileStorage << "H_inv_orig" << TrackingCommon::positionEstimatorController.positionEstimator.H_inv_orig;
    fileStorage.release();
    TrackingCommon::lastUsedHomography = fileName;
}

void MainController::setMultiFaceTracking(bool enable)
{
    emit enableMultiFaceTracking(enable);
    TrackingCommon::enableMultipleFaceTracking = enable;
}

void MainController::enableHomographySave(bool enable)
{
    emit enableSaveHomographySignal(enable);
}

void MainController::enable3DShowReconstruction(bool enable)
{
    emit enable3DReconstructionSignal(enable);
}

void MainController::displayCurrentHomography()
{
    if (TrackingCommon::positionEstimatorController.positionEstimator.isReady)
    {
        std::string currentHomographyLabel = "CurrentH";
        TrackingCommon::dataDisplayer.displayData(currentHomographyLabel, TrackingCommon::positionEstimatorController.positionEstimator.H, DataDisplayer::FULL);
    }
}

void MainController::clearCurrentHomography()
{
    TrackingCommon::positionEstimatorController.positionEstimator.clear();
    TrackingCommon::lastUsedHomography = "";
}

void MainController::saveCalibration(std::string fileName)
{
    cv::FileStorage fileStorage = cv::FileStorage(fileName, CV_STORAGE_WRITE);
    fileStorage << "camera_matrix" << TrackingCommon::cameraMatrix;
    fileStorage << "distortion_coefficients" << TrackingCommon::distCoeffs;
    fileStorage.release();
    TrackingCommon::calibrated = true;
    TrackingCommon::lastUsedCalibration = fileName;
}

bool MainController::loadCalibration(std::string fileName)
{
    cv::FileStorage fileStorage = cv::FileStorage(fileName, CV_STORAGE_READ);
    if (fileStorage.isOpened())
    {
        fileStorage["camera_matrix"] >> TrackingCommon::cameraMatrix;
        fileStorage["distortion_coefficients"] >> TrackingCommon::distCoeffs;
        fileStorage["focal_length_x_step"] >> TrackingCommon::focalLengthStepX;
        fileStorage["focal_length_y_step"] >> TrackingCommon::focalLengthStepY;
        fileStorage["focal_length_offset_x"] >> TrackingCommon::focalLengthOffsetX;
        fileStorage["focal_length_offset_y"] >> TrackingCommon::focalLengthOffsetY;
        fileStorage.release();
        TrackingCommon::calibrated = true;
        TrackingCommon::lastUsedCalibration = fileName;
        TrackingCommon::positionEstimatorController.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);
        return true;
    }
    return false;
}

void MainController::displayCurrentCalibration()
{
    TrackingCommon::dataDisplayer.displayData(std::string("CameraMatrix"), TrackingCommon::cameraMatrix, DataDisplayer::FULL);
}

void MainController::clearCurrentCalibration()
{
    TrackingCommon::calibrated = false;
    TrackingCommon::lastUsedCalibration = "";
}


bool MainController::connectGPSerialPort()
{
    if (!TrackingCommon::gpSerialPort->isOpen())
    {
        TrackingCommon::gpSerialPort->setPortName(QLatin1String(TrackingCommon::gpSerialPortName.c_str()));
        TrackingCommon::gpSerialPort->setBaudRate(BAUD57600);
        TrackingCommon::gpSerialPort->setFlowControl(FLOW_OFF);
        TrackingCommon::gpSerialPort->setParity(PAR_NONE);
        TrackingCommon::gpSerialPort->setDataBits(DATA_8);
        TrackingCommon::gpSerialPort->setStopBits(STOP_1);
        TrackingCommon::gpSerialPort->open(QIODevice::WriteOnly | QIODevice::Unbuffered);
        if (TrackingCommon::gpSerialPort->isOpen())
            return true;
    }
    else
    {
        qDebug() << "GP Serial Port is Open";
    }
    return false;
}

bool MainController::disconnectGPSerialPort()
{
    if (TrackingCommon::gpSerialPort->isOpen())
    {
        qDebug("GP Serial Port Open...  trying to disconnect");
        TrackingCommon::gpSerialPort->close();
        return TrackingCommon::gpSerialPort->disconnect();
    }
    return false;
}

void MainController::connectJoypad(int index)
{
    if (TrackingCommon::joypadController.availableJoysticks())
    {
        qDebug() << "Connecting Joypad";
        TrackingCommon::joypadController.setJoystick(0);
        TrackingCommon::joypadController.start();
        TrackingCommon::parrotController.start();
    }
}

void MainController::disconnectJoypad(int index)
{
    TrackingCommon::joypadController.exit();
}

void MainController::connectParrot()
{
    qDebug() << "Emitting connectParrotSignal";
    emit connectParrotSignal();
}

void MainController::disconnectParrot()
{
    qDebug() << "Emitting disconnectParrotSignal";
    emit disconnectParrotSignal();
}

void MainController::takeOffParrot()
{
    qDebug() << "Emitting TakeOff";
    emit takeOffParrotSignal();
}

void MainController::landParrot()
{
    qDebug() << "Emitting Land";
    emit landParrotSignal();
}

void MainController::addPlaneModel(std::string fileName)
{
    cv::Mat image = cv::imread(fileName);
    FeatureBasedModel* model = new FeatureBasedModel();
    model->templates.push_back(image);
    model->calculateModel();
    TrackingCommon::objectModels.push_back(model);
}

void MainController::setFaceLock(bool lock)
{
    for (int i = 0; i < TrackingCommon::totalDrawing3D; ++i)
    {
        ((CuboidTracker*)TrackingCommon::drawings3D[i])->enableFaceLock(lock);
    }
}

void MainController::saveCameraReference(std::string fileName)
{
    cv::FileStorage fileStorage = cv::FileStorage(fileName, CV_STORAGE_WRITE);
    fileStorage << "camera_reference" << TrackingCommon::objectZero;
    fileStorage.release();
}

void MainController::loadCameraReference(std::string fileName)
{
    cv::FileStorage fileStorage = cv::FileStorage(fileName, CV_STORAGE_READ);
    if (fileStorage.isOpened())
    {
        fileStorage["camera_reference"] >> TrackingCommon::objectZero;
        fileStorage.release();
    }
}

void MainController::updateCameraCalibrationInterpolation(int value)
{
    // TODO: Update camera calibration with new zoom value
    // TrackingCommon::camera <- los nuevos Fx, Fy
    // TrackingCommon::camera = [ Fx  0  Cx ]
    //                          [ 0  Fy  Cy ]
    //                          [ 0   0   1 ]
    // Fx = focal_length_x_step * value + focal_length_offset_x
    // Fy = focal_length_y_step * value + focal_length_offset_y
    if (TrackingCommon::focalLengthStepX != 0.0 && TrackingCommon::focalLengthStepY != 0.0)
    {
        TrackingCommon::cameraMatrix.at<double>(0, 0) = TrackingCommon::focalLengthStepX * (value / 1000.0) + TrackingCommon::focalLengthOffsetX;
        TrackingCommon::cameraMatrix.at<double>(1, 1) = TrackingCommon::focalLengthStepY * (value / 1000.0) + TrackingCommon::focalLengthOffsetY;
    }
    else
    {
        qDebug() << "This calibration file does not have values for interpolation";
    }
}
