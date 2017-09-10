#include "PositionEstimatorController.h"

PositionEstimatorController::PositionEstimatorController()
{
	enableHeightCorrection = false;
	enableHomographyInterpolation = false;
	displayAllModes = false;
    positionEstimator = PositionEstimator();
    homographyExtender.setPositionEstimator(&positionEstimator);
}

void PositionEstimatorController::setPositionEstimator(PositionEstimator positionEstimator)
{
    this->positionEstimator = positionEstimator;
	homographyExtender.setPositionEstimator(&this->positionEstimator);
}

void PositionEstimatorController::setMode(int enableHomographyInterpolation, int enableHeightCorrection, int displayAllModes)
{
	this->enableHomographyInterpolation = enableHomographyInterpolation;
	this->enableHeightCorrection = enableHeightCorrection;
	this->displayAllModes = displayAllModes;
}

void PositionEstimatorController::setObjectHeight(double objectHeight)
{
	positionEstimator.setObjectHeight(objectHeight);
}

void PositionEstimatorController::setInterpolationRanges(int initialPan, int initialTilt, int endPan, int endTilt)
{
	homographyExtender.setInterpolationRanges(initialPan, initialTilt, endPan, endTilt);
}

void PositionEstimatorController::setCameraMovement(PTZCameraController::CameraMovement cameraMovement)
{
    setInterpolationRanges(cameraMovement.previousPan, cameraMovement.previousTilt, cameraMovement.newPan, cameraMovement.newTilt);
}

void PositionEstimatorController::startHomographyInterpolation()
{
    homographyExtender.startInterpolation();
}

bool PositionEstimatorController::interpolationIsFinished()
{
    return homographyExtender.interpolationIsFinished;
}

void PositionEstimatorController::updateHomographyInterpolation()
{
    homographyExtender.updateHomography();
}

void PositionEstimatorController::updateTargetHomography(int pan, int tilt, bool writeToHomography, bool writeToInterpolated)
{
    homographyExtender.updateTargetHomography(pan, tilt, writeToHomography, writeToInterpolated);
}

void PositionEstimatorController::setCameraIntrinsicParameters(cv::Mat cameraIntrinsicParameters)
{
	positionEstimator.homographyDecomposer.setCameraIntrinsicParameters(cameraIntrinsicParameters);
	homographyExtender.setCameraIntrinsicParameters(cameraIntrinsicParameters);
}

void PositionEstimatorController::setCameraSpeed(int panSpeed, int tiltSpeed)
{
	homographyExtender.setCameraSpeed(panSpeed, tiltSpeed);
}

bool PositionEstimatorController::isReady()
{
    return positionEstimator.isReady;
}

bool PositionEstimatorController::isInterpolating()
{
    return homographyExtender.interpolating;
}

void PositionEstimatorController::estimateAndDisplayPosition(int x, int y, std::string windowName)
{
	//double angle_x = - ( PI / 12 ) * tilt / (7789);
	//double angle_y = - ( PI / 6 ) * pan / (15578);
	//positionEstimator.setObjectHeight(10.75);
	//positionEstimator.homographyDecomposer.setCameraIntrinsicParameters(cameraMatrix);
	//positionEstimator.updateHeightCorrection();
	//std::cout << "\nW1: ";
	//for (int i = 0; i < 9; ++i)
	//	std::cout << positionEstimator.H.at<double>(i / 3, i % 3) << ", ";
	//homographyExtender.universalHomography(cameraMatrix, positionEstimator.H_inv_orig, angle_x, angle_y, positionEstimator.H_inv);
	//cv::invert(positionEstimator.H_inv, positionEstimator.H);

	std::string windowNameInterpolated;
	std::string windowNameCorrected;
	std::string windowNameInterpolatedCorrected;
	windowNameInterpolated = windowName + "Interpolated";
	windowNameCorrected = windowName + "Corrected";
	windowNameInterpolatedCorrected = windowName + "InterpolatedCorrected";

	if (enableHomographyInterpolation)
	{
		if (homographyExtender.interpolating == true)
		{
			positionEstimator.setHomography(homographyExtender.updateHomographyInterpolation(), PositionEstimator::INTERPOLATED);
		}
		if (displayAllModes || enableHeightCorrection == false)
		{
			positionEstimator.transformationMode = PositionEstimator::INTERPOLATED_INVERSE;
			positionEstimator.showCoordinatesTransform(cv::Point(x, y), windowNameInterpolated.c_str());
		}
	}
	else
	{
		positionEstimator.setHomography(homographyExtender.targetHomography);
	}
	
	if (displayAllModes || enableHeightCorrection == false && enableHomographyInterpolation == false)
	{
		positionEstimator.transformationMode = PositionEstimator::INVERSE;
		positionEstimator.setHomography(homographyExtender.targetHomography);
		positionEstimator.showCoordinatesTransform(cv::Point(x, y), windowName.c_str());
	}

	if (enableHeightCorrection)
	{
		positionEstimator.transformationMode = PositionEstimator::CORRECTED_INVERSE;

		if (displayAllModes || enableHomographyInterpolation == false)
		{
			positionEstimator.updateHeightCorrection(PositionEstimator::DIRECT);
			positionEstimator.showCoordinatesTransform(cv::Point(x, y), windowNameCorrected.c_str());
		}

		if (enableHomographyInterpolation)
		{
			positionEstimator.updateHeightCorrection(PositionEstimator::INTERPOLATED);
			positionEstimator.showCoordinatesTransform(cv::Point(x, y), windowNameInterpolatedCorrected.c_str());
		}
	}
}
