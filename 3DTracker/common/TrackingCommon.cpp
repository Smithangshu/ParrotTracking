#include "TrackingCommon.h"

namespace TrackingCommon
{
    // Images
    cv::Mat frame;
    cv::Mat currentFrame;
    cv::Mat currentPyrs[2];
    cv::VideoCapture capture;

    // Playback settings
    int cameraIndex = 0;
    std::string videoFileName;

    // Mouse status
    int mouseX = 0;
    int mouseY = 0;
    int mouseEvent = 0;

    // Selection related
    cv::Rect selectionBox;
    int drawingBox = false;
    int drawingSource = 0;
    char mode = '0';
    int runSimilarity = true;
    int drawingHomographyDiscovery = false;
    int homographyDiscoveryStep = 0;
    cv::Point2f homographyDiscoveryPoints[4];
    cv::Mat discoveredHomography = cv::Mat(3, 3, CV_64F);

    // Homography discovery and zoom feature
    int currentMouseX = 0;
    int currentMouseY = 0;
    cv::Rect currentSelectionWindow;
    bool zoomActive = false;
    HomographyCaptureWindow* homographyCaptureWindow;

    // Data displayer
    DataDisplayerWindow* dataDisplayerWindow;

    // Sockets
    SocketsController mainSocket;
    bool enableHapticControl = false;

   // Tracking windows
    cv::Rect windows[max_windows];
    Tracker *trackingWindows[max_windows];
    int totalWindows;

    // Program settings and control
    MainController* mainController;
    int pauseOnFirstFrame = false;
    bool pauseOnEachFrame = false;
    bool enableDeinterlace = false;
    bool enableUndistort = false;
    enum PlaybackStatus playbackStatus = PLAYING;
    enum TrackingModes trackingMode = TRACKING_MS;
    enum ProgramModes programMode = TRACKING;
    bool show3DReconstruction = false;
    bool showDiscoveredHomography = false;
    enum ScalingModes scalingMode = SCALE_WITH_HOMOGRAPHY;
    bool showLKParameters = false;
    bool showLKIntermediumImages = false;
    std::string lastUsedHomography;
    std::string lastUsedCalibration;
    int glDelay = 105;
    cv::Mat reconstructionBaseReferential = cv::Mat(4, 4, CV_64F);

    // For displaying mean shift current model
    bool showMeanShiftModel = false;
    bool enableMeanShiftModelUpdate = false;
    bool enableMeanShiftScaleAdaptation = false;
    bool createTrackingWindow = false;
    bool delayTracker1sInEachIteration = false;
    bool selfRecover = false;
    bool findModels = false;
    int cuboidSelectedFace = 0;
    int cuboidSelectedModel = 0;
    bool enableLockFace = false;
    bool enableMultipleFaceTracking = false;
    int maxTrackedFaces = 1;

    // Filtering
    bool enableFilteringMeanShift = false;
    bool enableFilteringMeanShiftToggled = false;
    bool enableFiltering3DPose = false;
    bool enableFiltering3DPoseToggled = false;

    // 2D Localization
    bool enableHeightCorrection = false;
    float objectHeight = 0.0;
    bool enableHomographyInterpolation = false;
    bool displayUnscaled = false;
    bool enableScalingWithHomography = false;
    bool recordPositionEstimation = false;
    bool enableSendingPositionData = false;

	// Calibration vars
	Mat cameraMatrix;
	Mat distCoeffs;
    float focalLengthStepX = 0.0;
    float focalLengthOffsetX = 0.0;
    float focalLengthStepY = 0.0;
    float focalLengthOffsetY = 0.0;
    int calibrated = false;
	int enableCalibration = false;
	int homography = false;
	int homography_max_points = 10;
	cv::Point2f homography_training[20];
	cv::Point2f homography_training_real[20];
	cv::Point2f current_point = cvPoint2D32f( 0.0, 0.0);
	int total_homography_training = 0;
    PositionEstimatorController positionEstimatorController = PositionEstimatorController();
	int current_homography;
	double _h[9];
	double _h_orig[9];
	CvMat H = cvMat(3, 3, CV_64F, _h);
	CvMat H_orig = cvMat(3, 3, CV_64F, _h_orig);
	double _h_inv[9];
	double _h_inv_orig[9];
	CvMat H_inv = cvMat(3, 3, CV_64F, _h_inv);
	CvMat H_inv_orig = cvMat(3, 3, CV_64F, _h_inv_orig);
	int camera_has_moved = false;
	
	// Camera control
    PTZCameraController pTZCameraController;
    long int pan = 0;
	long int tilt = 0;
    int homePan = 0;
    int homeTilt = 0;
    int homeSpeed = 0;
    int homeZoom = 0;
    int homeFocus = 0;
    int homeBrightness = 0;
    bool focusAutoOn = false;
    int deltaPanTilt = 100;
    int deltaXYThreshold = 10;
    bool centerCamera = false;

    bool markObjectZero = false;
    bool markObjectRotZero = false;
    cv::Mat objectZero = cv::Mat(4, 4, CV_64F);
    float linesWidth = 0.0;
    float linesHeight = 0.0;
    float lineWidth = 0.0;

    // Control module
    JoypadController joypadController;
    ParrotController parrotController;

    // For plane prawing (mode = MODE_DECOMPOSITION)
    int totalDrawing3D = 0;
    Drawing3D* drawings3D[10];

    // Plane detection
    PlaneDetector planeDetector;
    bool enableMotionFilter = false;
    bool searchPlaneLines = false;
    bool searchPlaneLinesFirstTime = true;
    bool searchPlaneLinesRecord = false;
    int areaThreshold = 0;
    int areaMaxThreshold = 0;
    int colorBrightnessThreshold = 0;
    int colorHSVThreshold = 0;

    // General purpouse serial port
    QextSerialPort* gpSerialPort = new QextSerialPort(QextSerialPort::Polling);
    std::string gpSerialPortName;

    // Extended homography mode
	HomographyExtender homographyExtender = HomographyExtender();

    // Parrot related
    bool enablePathDrawing = false;
    cv::Mat parrotTrajectory;

    // Data Displayer
    DataDisplayer dataDisplayer;

    // Object Models
    std::vector<ObjectModel*> objectModels;

    // Tests and Benchmarks
    //TestsController testsController = TestsController("/home/v170/Dropbox/3DTracker/Experiments/");
    TestsController testsController = TestsController("/mnt/eRobots/Users/Manlio/Experiments/");
    //TestsController testsController = TestsController("/media/lito/Lito/Experiments/");

    cv::VideoWriter fileOutput;
}

void TrackingCommon::loadCameraParameters(const char *fileName)
{
	CvFileStorage* fs = cvOpenFileStorage(fileName, 0, CV_STORAGE_READ);
	cameraMatrix = (CvMat *) cvReadByName(fs, 0, "camera_matrix");
	distCoeffs = (CvMat *) cvReadByName(fs, 0, "distortion_coefficients");
	calibrated = true;
	cvReleaseFileStorage(&fs);
}

void TrackingCommon::loadHomography(const char *fileName)
{
	CvFileStorage* fs = cvOpenFileStorage(fileName, 0, CV_STORAGE_READ);

	CvMat H_temp = *((CvMat *) cvReadByName(fs, 0, "H_Object_to_Image"));
	CvMat H_orig = *((CvMat *) cvReadByName(fs, 0, "H_Object_to_Image"));
	CvMat H_inv = *((CvMat *) cvReadByName(fs, 0, "H_Image_to_Object"));
	CvMat H_inv_orig = *((CvMat *) cvReadByName(fs, 0, "H_Image_to_Object"));
	
    positionEstimatorController.positionEstimator.H = cv::cvarrToMat(&H_temp);
    positionEstimatorController.positionEstimator.H_orig = cv::cvarrToMat(&H_orig);
    positionEstimatorController.positionEstimator.H_inv = cv::cvarrToMat(&H_inv);
    positionEstimatorController.positionEstimator.H_inv_orig = cv::cvarrToMat(&H_inv_orig);
	current_homography = 0;

	cvReleaseFileStorage( &fs );

    positionEstimatorController.positionEstimator.setReady(true);
	homography = true;
}

void TrackingCommon::drawBox(Mat &image, Rect box)
{
    cv::rectangle(image, box, cv::Scalar(0, 0xd8, 0xff), 2);
}

void TrackingCommon::zoomImage(int x, int y)
{
    currentMouseX = x;
    currentMouseY = y;

    float xProportionCorrection = (1.0 * x) / currentFrame.cols;
    float yProportionCorrection = (1.0 * y) / currentFrame.rows;
    int zoomedSelectionWidth = currentFrame.cols / 4;
    int zoomedSelectionHeight = currentFrame.rows / 4;
    cv::Rect zoomedSelection = cv::Rect(x - xProportionCorrection * zoomedSelectionWidth, y - yProportionCorrection * zoomedSelectionHeight, zoomedSelectionWidth, zoomedSelectionHeight);
    if (zoomedSelection.x < 0)
    {
        zoomedSelection.x = 0;
    }
    else if (zoomedSelection.x > currentFrame.cols - zoomedSelectionWidth)
    {
        zoomedSelection.x = currentFrame.cols - zoomedSelectionWidth;
    }
    if (zoomedSelection.y < 0)
    {
        zoomedSelection.y = 0;
    }
    else if (zoomedSelection.y > currentFrame.rows - zoomedSelectionHeight)
    {
        zoomedSelection.y = currentFrame.rows - zoomedSelectionHeight;
    }
    currentSelectionWindow = zoomedSelection;

    cv::Mat zoomedWindow = frame(zoomedSelection);
    cv::Mat zoomed2x = cv::Mat(zoomedWindow.rows * 2, zoomedWindow.cols * 2, CV_8UC3);
    cv::Mat zoomed4x = cv::Mat(zoomedWindow.rows * 2, zoomedWindow.cols * 2, CV_8UC3);

    cv::pyrUp(zoomedWindow, zoomed2x);
    cv::pyrUp(zoomed2x, currentFrame);
}

cv::Mat TrackingCommon::calculate2DPosition(Point2f imageCoordinates)
{
    cv::Point3f realWorldCoordinates;
    cv::Point3f realWorldCoordinatesCorrected;
    cv::Point3f realWorldCoordinatesInterpolated;
    cv::Point3f realWorldCoordinatesCorrectedInteporlated;

    positionEstimatorController.positionEstimator.setObjectHeight(0.0);
    realWorldCoordinates = positionEstimatorController.positionEstimator.calculateTransformation(imageCoordinates, PositionEstimator::INVERSE);

    if (enableHomographyInterpolation)
    {
        realWorldCoordinatesInterpolated = positionEstimatorController.positionEstimator.calculateTransformation(imageCoordinates, PositionEstimator::INTERPOLATED_INVERSE);
    }
    if (enableHeightCorrection)
    {
        positionEstimatorController.positionEstimator.setObjectHeight(objectHeight);
        positionEstimatorController.positionEstimator.updateHeightCorrection();
        realWorldCoordinatesCorrected = positionEstimatorController.positionEstimator.calculateTransformation(imageCoordinates, PositionEstimator::CORRECTED_INVERSE);
    }
    if (enableHomographyInterpolation && enableHeightCorrection)
    {
        positionEstimatorController.positionEstimator.setObjectHeight(objectHeight);
        positionEstimatorController.positionEstimator.updateHeightCorrection(PositionEstimator::INTERPOLATED);
        realWorldCoordinatesCorrectedInteporlated = positionEstimatorController.positionEstimator.calculateTransformation(imageCoordinates, PositionEstimator::CORRECTED_INVERSE);
    }

    cv::Mat displayData = cv::Mat(1, 14, CV_32F);
    displayData.at<float>(0, 0) = imageCoordinates.x;
    displayData.at<float>(0, 1) = imageCoordinates.y;

    displayData.at<float>(0, 2) = realWorldCoordinates.x;
    displayData.at<float>(0, 3) = realWorldCoordinates.y;
    displayData.at<float>(0, 4) = realWorldCoordinates.z;

    displayData.at<float>(0, 5) = realWorldCoordinatesCorrected.x;
    displayData.at<float>(0, 6) = realWorldCoordinatesCorrected.y;
    displayData.at<float>(0, 7) = realWorldCoordinatesCorrected.z;

    displayData.at<float>(0, 8) = realWorldCoordinatesInterpolated.x;
    displayData.at<float>(0, 9) = realWorldCoordinatesInterpolated.y;
    displayData.at<float>(0, 10) = realWorldCoordinatesInterpolated.z;

    displayData.at<float>(0, 11) = realWorldCoordinatesCorrectedInteporlated.x;
    displayData.at<float>(0, 12) = realWorldCoordinatesCorrectedInteporlated.y;
    displayData.at<float>(0, 13) = realWorldCoordinatesCorrectedInteporlated.z;

    return displayData;
}

void TrackingCommon::highlightKeypoints(cv::Mat &image, std::vector<cv::KeyPoint> keyPoints, cv::Scalar color)
{
    for (int i = 0; i < keyPoints.size(); ++i)
    {
        cv::KeyPoint keypoint = (cv::KeyPoint) keyPoints.at(i);
        cv::circle(image, keypoint.pt, 3, color);
    }
}

void TrackingCommon::setZero(cv::Mat objectCenter)
{
    //////////////////////////////////////////////
    // Define a custom "ZERO"
    //////////////////////////////////////////////

    // Camera pan tilt
    //TrackingCommon::objectZero = objectCenter.inv() * (getCurrentCameraRotationMatrix()).inv();
    TrackingCommon::objectZero = objectCenter.inv();
    //TrackingCommon::objectZero = objectCenter;
    TrackingCommon::markObjectZero = false;
}

cv::Mat TrackingCommon::buildRotationMatrix(int axis, double angle)
{
    cv::Mat R;
    cv::Mat RFull = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat RFullRot = RFull(cv::Range(0, 3), cv::Range(0, 3));
    double _rx[9];
    double _ry[9];
    double _rz[9];

    switch (axis)
    {
    case 0:
        // Build X-rotation matrix
        _rx[0] = 1;		_rx[1] = 0;				_rx[2] = 0;
        _rx[3] = 0;		_rx[4] = cos(angle);	_rx[5] = sin(angle);
        _rx[6] = 0;		_rx[7] = -sin(angle);	_rx[8] = cos(angle);
        R = cv::Mat(3, 3, CV_64F, _rx);
        break;
    case 1:
        // Build Y-rotation matrix
        _ry[0] = cos(angle);	_ry[1] = 0;		_ry[2] = -sin(angle);
        _ry[3] = 0;				_ry[4] = 1;		_ry[5] = 0;
        _ry[6] = sin(angle);	_ry[7] = 0;		_ry[8] = cos(angle);
        R = cv::Mat(3, 3, CV_64F, _ry);
        break;
    default:
        // Build Z-rotation matrix
        _rz[0] = cos(angle);	_rz[1] = sin(angle);	_rz[2] = 0;
        _rz[3] = -sin(angle);	_rz[4] = cos(angle);	_rz[5] = 0;
        _rz[6] = 0;             _rz[7] = 0;             _rz[8] = 1;
        R = cv::Mat(3, 3, CV_64F, _rz);
        break;
    }
    R.copyTo(RFullRot);
    return RFull;
}

cv::Mat TrackingCommon::getCurrentCameraRotationMatrix()
{
    float xAngle = ( 3.1416 / 12 ) * TrackingCommon::pTZCameraController.tilt / (7789);
    float yAngle = ( 3.1416 / 6 ) * TrackingCommon::pTZCameraController.pan / (15578);
    cv::Mat Rx = TrackingCommon::buildRotationMatrix(0, -xAngle);
    cv::Mat Ry = TrackingCommon::buildRotationMatrix(1, -yAngle);
    return Rx * Ry;
}

void TrackingCommon::getAnglesFromTransform(cv::Mat &transform, float &angleX, float &angleY, float &angleZ)
{
    cv::Mat rotation = transform(Range(0, 3), Range(0, 3));
    cv::Mat angles = cv::Mat(3, 1, CV_64F);
    cv::Rodrigues(rotation, angles);
    double toDeg = 180.0 / CV_PI;
    angleX = angles.at<double>(0, 0) * toDeg;
    angleY = angles.at<double>(1, 0) * toDeg;
    angleZ = angles.at<double>(2, 0) * toDeg;
}

void TrackingCommon::getEulerAnglesFromTransform(cv::Mat &transform, float &angleX, float &angleY, float &angleZ)
{
    cv::Mat rotation = transform(Range(0, 3), Range(0, 3));
    double toDeg = 180.0 / CV_PI;
    angleX = std::atan2(rotation.at<double>(2, 0), rotation.at<double>(2, 1)) * toDeg;
    angleY = std::acos(rotation.at<double>(2, 2)) * toDeg;
    angleZ = -std::atan2(rotation.at<double>(0, 2), rotation.at<double>(1, 2)) * toDeg;
}

cv::Mat TrackingCommon::getQuaternionFromTransform(cv::Mat &transform)
{
    //    double q1, q2, q3, q4;
    //    qDebug() << "Quaternion sum: " << 1 + transform.at<double>(0, 0) + transform.at<double>(1, 1) + transform.at<double>(2, 2);
    //    q4 = 0.5 * std::sqrt(1 + transform.at<double>(0, 0) + transform.at<double>(1, 1) + transform.at<double>(2, 2));
    //    q1 = (1 / (4 * q4)) * (transform.at<double>(2, 1) - transform.at<double>(1, 2));
    //    q2 = (1 / (4 * q4)) * (transform.at<double>(0, 2) - transform.at<double>(2, 0));
    //    q3 = (1 / (4 * q4)) * (transform.at<double>(1, 0) - transform.at<double>(0, 1));

    float m00, m01, m02, m10, m11, m12, m20, m21, m22, qx, qy, qz, qw;
    m00 = transform.at<double>(0, 0);
    m01 = transform.at<double>(0, 1);
    m02 = transform.at<double>(0, 2);
    m10 = transform.at<double>(1, 0);
    m11 = transform.at<double>(1, 1);
    m12 = transform.at<double>(1, 2);
    m20 = transform.at<double>(2, 0);
    m21 = transform.at<double>(2, 1);
    m22 = transform.at<double>(2, 2);

    float tr = m00 + m11 + m22;

    if (tr > 0) {
        float S = sqrt(tr+1.0) * 2; // S=4*qw
        qw = 0.25 * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11)&(m00 > m22)) {
        float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
        qw = (m21 - m12) / S;
        qx = 0.25 * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25 * S;
        qz = (m12 + m21) / S;
    } else {
    float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25 * S;
    }

    cv::Mat quaternion  = (cv::Mat_<double>(4, 1) << qx, qy, qz, qw);

    return quaternion;
}

cv::Mat TrackingCommon::averageQuaternions(std::vector<cv::Mat> quaternions)
{
    cv::Mat quaternion = cv::Mat::zeros(4, 1, CV_64F);
    for (int i = 0; i < quaternions.size(); ++i)
    {
        if (quaternions[0].dot(quaternions[i]) > 0)
            quaternion += quaternions[i];
        else
            quaternion -= quaternions[i];
    }
    // Average
    quaternion /= quaternions.size();

    // Normalize
    quaternion /= lengthOfVector(quaternion);

    return quaternion;
}

cv::Mat TrackingCommon::getTransformFromQuaternion(cv::Mat &quaternion)
{
    //    rotation.at<double>(0, 0) = 1 - 2 * std::pow(q2, 2) - 2 * std::pow(q3, 2);
    //    rotation.at<double>(1, 0) = 2 * (q1 * q2 + q3 * q4);
    //    rotation.at<double>(2, 0) = 2 * (q1 * q3 - q2 * q4);

    //    rotation.at<double>(0, 1) = 2 * (q1 * q2 - q3 * q4);
    //    rotation.at<double>(1, 1) = 1 - 2 * std::pow(q1, 2) - 2 * std::pow(q3, 2);
    //    rotation.at<double>(2, 1) = 2 * (q1 * q4 + q2 * q3);

    //    rotation.at<double>(0, 2) = 2 * (q1 * q3 + q2 * q4);
    //    rotation.at<double>(1, 2) = 2 * (q2 * q3 - q1 * q4);
    //    rotation.at<double>(2, 2) = 1 - 2 * std::pow(q1, 2) - 2 * std::pow(q2, 2);

    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);

    float m00, m01, m02, m10, m11, m12, m20, m21, m22;
    float qx = quaternion.at<double>(0, 0);
    float qy = quaternion.at<double>(1, 0);
    float qz = quaternion.at<double>(2, 0);
    float qw = quaternion.at<double>(3, 0);

    double sqw = qw*qw;
    double sqx = qx*qx;
    double sqy = qy*qy;
    double sqz = qz*qz;

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    m11 = (-sqx + sqy - sqz + sqw)*invs ;
    m22 = (-sqx - sqy + sqz + sqw)*invs ;

    double tmp1 = qx*qy;
    double tmp2 = qz*qw;
    m10 = 2.0 * (tmp1 + tmp2)*invs ;
    m01 = 2.0 * (tmp1 - tmp2)*invs ;

    tmp1 = qx*qz;
    tmp2 = qy*qw;
    m20 = 2.0 * (tmp1 - tmp2)*invs ;
    m02 = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = qy*qz;
    tmp2 = qx*qw;
    m21 = 2.0 * (tmp1 + tmp2)*invs ;
    m12 = 2.0 * (tmp1 - tmp2)*invs ;

    transform.at<double>(0, 0) = m00;
    transform.at<double>(0, 1) = m01;
    transform.at<double>(0, 2) = m02;
    transform.at<double>(1, 0) = m10;
    transform.at<double>(1, 1) = m11;
    transform.at<double>(1, 2) = m12;
    transform.at<double>(2, 0) = m20;
    transform.at<double>(2, 1) = m21;
    transform.at<double>(2, 2) = m22;

    return transform;
}

cv::Mat TrackingCommon::getTransformFromAngles(float angleX, float angleY, float angleZ)
{
    double toRad = CV_PI / 180.0;

    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat rotation = transform(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat angles = cv::Mat(3, 1, CV_64F);
    angles.at<double>(0, 0) = angleX * toRad;
    angles.at<double>(1, 0) = angleY * toRad;
    angles.at<double>(2, 0) = angleZ * toRad;

    cv::Rodrigues(angles, rotation);
    return transform;
}

double TrackingCommon::lengthOfVector(cv::Mat vector)
{
    return std::sqrt(cv::sum(vector.mul(vector))[0]);
}

void TrackingCommon::deinterlace(cv::Mat &image)
{
    int height = image.rows;
    int width = image.cols;
    int channels = image.channels();
    //cv::Mat cvImage = cv::Mat(height, width, image.type());


    for (int y = 1; y < height-1; y += 2)
    {
        Vec3b* prevRow = image.ptr<Vec3b>(y-1);
        Vec3b* currRow = image.ptr<Vec3b>(y);
        Vec3b* nextRow = image.ptr<Vec3b>(y+1);

        for (int x = 0; x < width; x++)
        {
            for (int c = 0; c < channels; ++c)
                currRow[x][c] = (0.5f*(unsigned char)prevRow[x][c]) + (0.5f * (unsigned char)nextRow[x][c]);
        }
    }
    //cvImage.copyTo(image);

//    if (height > 1 && height % 2 == 0)
//    {
//        lineA = cvImage.imageData + ((height-2) * cvImage.widthStep);
//        lineB = cvImage.imageData + ((height-1) * cvImage.widthStep);
//        memcpy(lineB,lineA, width);
//    }
}

void TrackingCommon::drawPath(Mat &image, Mat referential)
{
    if (!TrackingCommon::parrotTrajectory.empty() && TrackingCommon::calibrated)
    {
        cv::Mat path = TrackingCommon::parrotTrajectory(cv::Range(0, TrackingCommon::parrotTrajectory.rows), cv::Range(0, 3) ).clone();
        cv::Mat status = TrackingCommon::parrotTrajectory(cv::Range(0, TrackingCommon::parrotTrajectory.rows), cv::Range(3, 4) ).clone();
        //cv::Mat path = TrackingCommon::parrotTrajectory;

        cv::Mat rotation = referential(cv::Range(0,3),cv::Range(0,3));
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
        cv::Rodrigues(rotation,rvec);
        cv::Mat tvec = referential(cv::Range(0,3),cv::Range(3,4));
        cv::Mat distortion = cv::Mat::zeros(4,1, CV_32F);
        cv::Mat projected;
        cv::projectPoints(path, rvec, tvec, TrackingCommon::cameraMatrix, distortion, projected);
//        cv::Mat cameraMatrixExtended = cv::Mat::zeros(3, 4, CV_32F);
//        TrackingCommon::cameraMatrix.copyTo(cameraMatrixExtended(cv::Range(0, 3), cv::Range(0, 3)));
//        TrackingCommon::mainController->displayData(std::string("MExt"), cameraMatrixExtended, DataDisplayer::FULL);
//        projected = TrackingCommon::cameraMatrix * cv::Mat::eye(4)
//        TrackingCommon::mainController->displayData(std::string("Path"), path, DataDisplayer::FULL);
//        TrackingCommon::mainController->displayData(std::string("rvec"), rvec, DataDisplayer::FULL);
//        TrackingCommon::mainController->displayData(std::string("tvec"), tvec, DataDisplayer::FULL);
//        TrackingCommon::mainController->displayData(std::string("Projected"), projected, DataDisplayer::FULL);
//        TrackingCommon::mainController->displayData(std::string("OZ"), TrackingCommon::objectZero, DataDisplayer::FULL);
//        qDebug() << "Dim " << projected.rows << ", " << projected.cols;
//        for (int m = 0; m < projected.rows; ++m)
//        {
//            qDebug() << ((cv::Point2f)projected.at<cv::Point2f>(m, 0)).x << ", " << ((cv::Point2f)projected.at<cv::Point2f>(m, 0)).y;
//        }
        if (!image.empty())
        {
            cv::Scalar green = CV_RGB(0,250,0);
            if (projected.rows <= 1)
            {
                cv::Mat projectZero;
                cv::Mat zeroParrot = cv::Mat::zeros(1,3, CV_32F);
                cv::projectPoints(zeroParrot, rvec, tvec, TrackingCommon::cameraMatrix, distortion, projectZero);
                cv::line(image, projectZero.at<cv::Point2f>(0,0),projected.at<cv::Point2f>(0,0),green);
            }
            else
            {
                for (int n = 0; n < projected.rows - 1; ++n)
                {
                    cv::line(image, projected.at<cv::Point2f>(n,0) ,projected.at<cv::Point2f>(n+1,0),green, 1, CV_AA);
                }
                for (int n = 0; n < projected.rows - 1; ++n)
                {
                    cv::circle(image, projected.at<cv::Point2f>(n+1,0), 3, cv::Scalar(0, 128, 255), -1, CV_AA);
                    if (status.at<float>(n, 0) > 0.0)
                        cv::circle(image, projected.at<cv::Point2f>(n+1,0), 7, cv::Scalar(0, 50, 255), 2, CV_AA);
                }
            }
        }
        else
        {
            qDebug() << "No hay video";
        }
    }
    else
    {
        qDebug() << "Agrega una trayectoria";
    }
}

float TrackingCommon::inRangeAndCoerce(float maxAbsoluteValue, float input)
{
    float coerced = input;
    if (std::abs(input) > maxAbsoluteValue)
    {
        coerced = (input > 0 ? 1 : -1) * maxAbsoluteValue;
    }
    return coerced;
}
