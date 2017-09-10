#ifndef TRACKING_COMMON_H
#define TRACKING_COMMON_H

#include <sstream>
#include "opencv2/core/core.hpp"
#include "common/MainController.h"
#include "reconstruction/PositionEstimator.h"
#include "reconstruction/PositionEstimatorController.h"
#include "drawing/PlaneDrawing.h"
#include "drawing/CuboidDrawing.h"
#include "reconstruction/HomographyExtender.h"
#include "gui/DataDisplayer.h"
#include "common/TestsController.h"
#include "common/PTZCameraController.h"
#include "gui/HomographyCaptureWindow.h"
#include "gui/DataDisplayerWindow.h"
#include "tracking/MeanShiftWithScaling.h"
#include "reconstruction/PlaneDetector.h"
#include "control/JoypadController.h"
#include "control/ParrotController.h"
#include "common/SocketsController.h"

namespace TrackingCommon
{
	// Images
    extern cv::Mat frame;
    extern cv::Mat currentFrame;
    extern cv::Mat currentPyrs[2];
    extern cv::VideoCapture capture;

    // Playback settings
    extern int cameraIndex;
    extern std::string videoFileName;

    // Mouse status
    extern int mouseX;
    extern int mouseY;
    extern int mouseEvent;

    // Selection related
    extern cv::Rect selectionBox;
    extern int drawingBox;
    extern int drawingSource;
	extern char mode;
    extern int runSimilarity;
    extern int drawingHomographyDiscovery;
    extern int homographyDiscoveryStep;
    extern cv::Point2f homographyDiscoveryPoints[4];
    extern cv::Mat discoveredHomography;

    // Homography discovery and zoom feature
    extern int currentMouseX;
    extern int currentMouseY;
    extern cv::Rect currentSelectionWindow;
    extern bool zoomActive;
    extern HomographyCaptureWindow* homographyCaptureWindow;

    // Data displayer
    extern DataDisplayerWindow* dataDisplayerWindow;

    // Sockets
    extern SocketsController mainSocket;
    extern bool enableHapticControl;

    // Tracking windows
	const int max_windows = 15;
    extern cv::Rect windows[max_windows];
	extern Tracker *trackingWindows[max_windows];
	extern int totalWindows;

    // Program settings and control
    extern MainController* mainController;
    extern int pauseOnFirstFrame;
    extern bool pauseOnEachFrame;
    extern bool enableDeinterlace;
    extern bool enableUndistort;
    enum PlaybackStatus {PLAYING, PAUSED};
	extern enum PlaybackStatus playbackStatus;
    enum TrackingModes {TRACKING_MS, TRACKING_LK, TRACKING_ESM, TRACKING_CUBOID};
    extern enum TrackingModes trackingMode;
    enum ProgramModes {TRACKING, HOMOGRAPHY_DISCOVERY};
    extern enum ProgramModes programMode;
    extern bool show3DReconstruction;
    extern bool showDiscoveredHomography;
    extern enum ScalingModes {SCALE_WITH_HOMOGRAPHY, SCALE_WITH_MODEL} scalingMode;
    extern bool showLKParameters;
    extern bool showLKIntermediumImages;
    extern std::string lastUsedHomography;
    extern std::string lastUsedCalibration;
    extern int glDelay;
    extern cv::Mat reconstructionBaseReferential;
    extern bool showMeanShiftModel;
    extern bool enableMeanShiftModelUpdate;
    extern bool enableMeanShiftScaleAdaptation;
    extern bool createTrackingWindow;
    extern bool delayTracker1sInEachIteration;
    extern bool selfRecover;
    extern bool findModels;
    extern int cuboidSelectedFace;
    extern int cuboidSelectedModel;
    extern bool enableLockFace;
    extern bool enableMultipleFaceTracking;
    extern int maxTrackedFaces;

    // Filtering
    extern bool enableFilteringMeanShift;
    extern bool enableFilteringMeanShiftToggled;
    extern bool enableFiltering3DPose;
    extern bool enableFiltering3DPoseToggled;

    // 2D Localization
    extern bool enableHeightCorrection;
    extern float objectHeight;
    extern bool enableHomographyInterpolation;
    extern bool displayUnscaled;
    extern bool enableScalingWithHomography;
    extern bool recordPositionEstimation;
    extern bool enableSendingPositionData;

    // Calibration vars
    extern Mat cameraMatrix;
    extern Mat distCoeffs;
    extern float focalLengthStepX;
    extern float focalLengthOffsetX;
    extern float focalLengthStepY;
    extern float focalLengthOffsetY;
    extern int calibrated;
	extern int enableCalibration;
	extern int homography;
	extern int homography_max_points;
	extern cv::Point2f homography_training[];
	extern cv::Point2f homography_training_real[];
	extern cv::Point2f current_point;
	extern int total_homography_training;
	extern PositionEstimatorController positionEstimatorController;
	extern int current_homography;
	extern double _h[];
	extern double _h_orig[];
	extern CvMat H;
	extern CvMat H_orig;
	extern double _h_inv[];
	extern double _h_inv_orig[9];
	extern CvMat H_inv;
	extern CvMat H_inv_orig;
	extern int camera_has_moved;
	
	// Camera control
    extern PTZCameraController pTZCameraController;
	extern long int pan;
	extern long int tilt;
	extern int panSpeed;
	extern int tiltSpeed;
	extern int enablePanTilt;
    extern int homePan;
    extern int homeTilt;
    extern int homeSpeed;
    extern int homeZoom;
    extern int homeFocus;
    extern int homeBrightness;
    extern bool focusAutoOn;
    extern int deltaPanTilt;
    extern int deltaXYThreshold;
    extern bool centerCamera;

    extern bool markObjectZero;
    extern bool markObjectRotZero;
    extern cv::Mat objectZero;
    extern float linesWidth;
    extern float linesHeight;
    extern float lineWidth;
    extern int areaThreshold;
    extern int areaMaxThreshold;
    extern int colorBrightnessThreshold;
    extern int colorHSVThreshold;

    // Control module
    extern JoypadController joypadController;
    extern ParrotController parrotController;

    // 3D OpenGL Drawings
    extern int totalDrawing3D;
    extern Drawing3D* drawings3D[];

    // Plane detection
    extern PlaneDetector planeDetector;
    extern bool enableMotionFilter;
    extern bool searchPlaneLines;
    extern bool searchPlaneLinesFirstTime;
    extern bool searchPlaneLinesRecord;

    // Extended homography mode
	extern HomographyExtender homographyExtender;

    // Parrot related
    extern bool enablePathDrawing;
    extern cv::Mat parrotTrajectory;

    // General purpouse serial port
    extern QextSerialPort* gpSerialPort;
    extern std::string gpSerialPortName;

    // Data Displayer
    extern DataDisplayer dataDisplayer;

	// Tests and Benchmarks
	extern TestsController testsController;
	
    // Object Models
    extern std::vector<ObjectModel*> objectModels;

    extern cv::VideoWriter fileOutput;

    void loadCameraParameters(const char *fileName);
	void loadHomography(const char *fileName);
    void drawBox(cv::Mat &image, cv::Rect selectionBox);
    void zoomImage(int x, int y);
    cv::Mat calculate2DPosition(cv::Point2f imageCoordinates);
    void setupCameraBasePose();
    void highlightKeypoints(Mat &image, std::vector<cv::KeyPoint> keyPoints, cv::Scalar color = cv::Scalar(0, 255, 128));
    void setZero(cv::Mat objectCenter);
    cv::Mat buildRotationMatrix(int axis, double angle);
    cv::Mat getCurrentCameraRotationMatrix();
    void getAnglesFromTransform(cv::Mat &transform, float &angleX, float &angleY, float &angleZ);
    cv::Mat getTransformFromAngles(float angleX, float angleY, float angleZ);
    void getEulerAnglesFromTransform(cv::Mat &transform, float &angleX, float &angleY, float &angleZ);
    cv::Mat getQuaternionFromTransform(cv::Mat &transform);
    cv::Mat averageQuaternions(std::vector<cv::Mat> quaternions);
    cv::Mat getTransformFromQuaternion(cv::Mat &quaternion);
    double lengthOfVector(cv::Mat vector);
    void deinterlace(cv::Mat &image);
    void drawPath(cv::Mat &image, cv::Mat referential);
    float inRangeAndCoerce(float maxAbsoluteValue, float input);
}

#endif
