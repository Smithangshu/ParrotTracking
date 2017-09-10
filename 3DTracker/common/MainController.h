#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "opencv2/core/core.hpp"
#include <string>
#include <ctime>
#include <QThread>
#include <QImage>
#include <map>
#include "gui/ImageWindow.h"
#include "gui/WindowCallbacks.h"
#include "gui/DataDisplayer.h"
#include "drawing/GLDrawingController.h"

using namespace cv;

class MainController : public QThread
{
    Q_OBJECT
    private:
        std::string mainCVWindowName;

    signals:
        void imageWindowReadySignal(std::string windowTitle, const QImage &image);
        void displayCVImageSignal(std::string windowTitle, cv::Mat image);
        void imageWindowsDestroySignal();
        void imageWindowMouseCallBackSignal(std::string windowTitle, WindowCallBack* windowCallBack);
        void dataDisplayerSignal(std::string tabName, cv::Mat mat, DataDisplayer::DataDisplayModes dataDisplayMode);
        void displayCoordinatesSignal(std::string tabName, std::string displayName, cv::Mat displayData);
        void displayPoseSignal(std::string tabName, std::string displayName, cv::Mat displayData);
        void enable3DReconstructionSignal(bool enable);
        void enableSaveHomographySignal(bool enable);
        void enableMultiFaceTracking(bool enable);
        void displayParrotControlValuesSignal(int roll, int pitch, int yaw, int verticalSpeed);
        void connectParrotSignal();
        void disconnectParrotSignal();
        void takeOffParrotSignal();
        void landParrotSignal();
        void setParrotValuesSignal(float angleX, float angleY, float angleZ, float positionX, float positionY, float positionZ);
        void setParrotPoseSignal(cv::Mat pose);
        void setZeroSignal();
        void setParrotAutomaticControlSignal(bool automaticControl);
        void setParrotConnectionStatusSignal(bool connected);
        void setTrajectorySignal(cv::Mat trajectory);
        void resetParrotFilterSignal(cv::Mat pose);
        void updateParrotTrajectoryStatusSignal(int point, bool completed);
        void setParrotControlVariablesSignal(float KpRollPitch, float KpRollPitchSpeed, float KpYaw, float KpVerticalSpeed, float KdRollPitch, float KdRollPitchSpeed, float KdYaw, float KdVerticalSpeed, float KiRollPitch, float KiRollPitchSpeed, float KiYaw, float KiVerticalSpeed, float inputFilterScale, float outputFilterScale, float maxRollPitchSpeed);
        void getParrotControlVariablesSignal(float &KpRollPitch, float &KpRollPitchSpeed, float &KpYaw, float &KpVerticalSpeed, float &KdRollPitch, float &KdRollPitchSpeed, float &KdYaw, float &KdVerticalSpeed, float &KiRollPitch, float &KiRollPitchSpeed, float &KiYaw, float &KiVerticalSpeed);
        void create3DReconstructionWindowSignal();
        void displayParrotBatteryChargeSignal(int batteryCharge);

    public:
        enum ProgramStatus {VIDEO_STARTED, VIDEO_PLAYING, VIDEO_PAUSED, VIDEO_STOPPED} programStatus;
        static enum ProgramModes {IMAGE_TRACKING, PLANE_TRACKING} programModes;
        enum ProgramModes programMode;
        static enum VideoModes {VIDEO_CAMERA, VIDEO_FILE, STATIC_IMAGE} videoModes;
        enum VideoModes videoMode;
        bool testMode;
        int testIndex;
        bool skipPauseOnFirstFrame;
        GLThread glThread;

        struct ImageWindowInfo {
            std::string windowTitle;
            int windowId;
        };
        std::map<std::string, ImageWindowInfo> imageWindowsInfo;
        std::map<std::string, ImageWindowInfo>::iterator imageWindowsInfoIterator;
        std::map<std::string, ImageWindow*> imageWindows;

        MainController();

        void displayImageWindow(std::string windowTitle, cv::Mat image);
        void displayImageWindowCV(std::string windowTitle, cv::Mat image);
        void prepareImageWindow(std::string windowTitle);
        void destroyImageWindows();
        void setImageWindowMouseCallBack(std::string windowTitle, WindowCallBack* windowCallBack);
        void displayData(std::string tabName, cv::Mat mat, DataDisplayer::DataDisplayModes dataDisplayMode);
        void displayCoordinates(std::string tabName, std::string displayName, cv::Mat &displayData);
        void displayPose(std::string tabName, std::string displayName, cv::Mat &displayData);
        void displayParrotControlValues(int roll, int pitch, int yaw, int verticalSpeed);
        void setParrotAutomaticControl(bool automaticControl);
        void setParrotConnectionStatus(bool connected);
        void setTrajectory(cv::Mat trajectory);
        void updateParrotTrajectoryStatus(int point, bool completed);
        void setParrotControlVariables(float KpRollPitch, float KpRollPitchSpeed, float KpYaw, float KpVerticalSpeed, float KdRollPitch, float KdRollPitchSpeed, float KdYaw, float KdVerticalSpeed, float KiRollPitch, float KiRollPitchSpeed, float KiYaw, float KiVerticalSpeed, float inputFilterScale, float outputFilterScale, float maxRollPitchSpeed);
        void getParrotControlVariables(float &KpRollPitch, float &KpRollPitchSpeed, float &KpYaw, float &KpVerticalSpeed, float &KdRollPitch, float &KdRollPitchSpeed, float &KdYaw, float &KdVerticalSpeed, float &KiRollPitch, float &KiRollPitchSpeed, float &KiYaw, float &KiVerticalSpeed);

        void setCameraIndex(int index);
        void setMediaFileName(std::string fileName);
        void setVideoMode(enum VideoModes videoMode);
        void startVideo();
        void pauseVideo();
        void stopVideo();
        void run();
        bool loadHomography(std::string fileName);
        void saveHomography(std::string fileName);
        void setMultiFaceTracking(bool enable);
        void displayCurrentHomography();
        void clearCurrentHomography();
        bool loadCalibration(std::string fileName);
        void saveCalibration(std::string fileName);
        void displayCurrentCalibration();
        void clearCurrentCalibration();
        bool connectGPSerialPort();
        bool disconnectGPSerialPort();
        void saveCameraReference(std::string fileName);
        void loadCameraReference(std::string fileName);
        void updateCameraCalibrationInterpolation(int value);

        // Parrot control
        void connectJoypad(int index);
        void disconnectJoypad(int index);
        void connectParrot();
        void disconnectParrot();
        void takeOffParrot();
        void landParrot();

        // Model related
        void addPlaneModel(std::string fileName);
        void setFaceLock(bool lock);

        //GUI related
        void enableHomographySave(bool enable);
        void enable3DShowReconstruction(bool enable);
};

#endif
