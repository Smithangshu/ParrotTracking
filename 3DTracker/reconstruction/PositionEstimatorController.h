#ifndef POSITION_ESTIMATOR_CONTROLLER_H
#define POSITION_ESTIMATOR_CONTROLLER_H

#include <sstream>
#include "opencv/cv.h"
#include "reconstruction/PositionEstimator.h"
#include "reconstruction/HomographyExtender.h"
#include "common/PTZCameraController.h"

class PositionEstimatorController
{
	public:
		int enableHomographyInterpolation;
		int enableHeightCorrection;
		int displayAllModes;

        PositionEstimator positionEstimator;
		HomographyExtender homographyExtender;

        PositionEstimatorController();
        void setPositionEstimator(PositionEstimator positionEstimator);
		void setMode(int enableHomographyInterpolation, int enableHeightCorrection, int displayAllModes);
		void setObjectHeight(double objectHeight);
		void setCameraIntrinsicParameters(cv::Mat cameraIntrinsicParameters);
		void setCameraSpeed(int panSpeed, int tiltSpeed);
		void setInterpolationRanges(int initialPan, int initialTilt, int endPan, int endTilt);
        void setCameraMovement(PTZCameraController::CameraMovement cameraMovement);
		void startHomographyInterpolation();
        bool interpolationIsFinished();
        void updateHomographyInterpolation();
        void updateTargetHomography(int pan, int tilt, bool writeToHomography = false, bool writeToInterpolated = false);
        bool isReady();
        bool isInterpolating();

		void estimateAndDisplayPosition(int x, int y, std::string windowName);
};

#endif
