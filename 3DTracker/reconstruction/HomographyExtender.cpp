#include "reconstruction/HomographyExtender.h"
#include <iostream>
#include <fstream>
#include "common/TrackingCommon.h"

HomographyExtender::HomographyExtender()
{
	isReady = false;
	currentTrackers = 0;
	imagePointsSet = false;
	
	// For homography interpolation
	interpolating = false;
    interpolationIsFinished = true;
	deltaPanParameters = cv::Mat(3, 3, CV_64F);
	deltaTiltParameters = cv::Mat(3, 3, CV_64F);
	HCurrent = cv::Mat(3, 3, CV_64F);
	targetHomographyPan = cv::Mat(3, 3, CV_64F);
	targetHomographyTilt = cv::Mat(3, 3, CV_64F);
	targetHomography = cv::Mat(3, 3, CV_64F);

    homographyExtensionMethod = INTERPOLATION;
}

void HomographyExtender::setPositionEstimator(PositionEstimator* positionEstimator)
{
	this->positionEstimator = positionEstimator;
}

void HomographyExtender::setExtensionMethod(enum HomographyExtensionMethods homographyExtensionMethod)
{
	this->homographyExtensionMethod = homographyExtensionMethod;
}
		
void HomographyExtender::addTrackingWindow(MeanShiftTracker* tracker)
{
	trackers[currentTrackers] = tracker;
	++currentTrackers;
}

void HomographyExtender::trackAllObjects(cv::Mat &image)
{
	for (int i = 0; i < currentTrackers; ++i)
	{
		trackers[i]->trackObject(image);
	}
}

void HomographyExtender::highlightAllObjects(cv::Mat &image)
{
	for (int i = 0; i < currentTrackers; ++i)
	{
		trackers[i]->highlightObject(image);
	}
}

int HomographyExtender::getTotalPoints()
{
	return currentTrackers;
}

void HomographyExtender::updateHomography()
{
	switch (homographyExtensionMethod)
	{
		case FEATURE_BASED: updateHomographyFeatureBased(); break;
        case INTERPOLATION: updateHomographyInterpolation(); break;
	}
}

void HomographyExtender::updateHomographyFeatureBased()
{
	if (currentTrackers >= 4)
	{
		if (!imagePointsSet) 
		{
			for (int i = 0; i < currentTrackers; ++i)
			{
				baseImagePoints[i] = trackers[i]->getWindowCenter();
			}
			imagePointsSet = true;
		}
		cv::Mat sourcePlanePoints = cv::Mat(currentTrackers, 2, CV_32F);
		cv::Mat destinationPlanePoints = cv::Mat(currentTrackers, 2, CV_32F);
		for (int i = 0; i < currentTrackers; ++i)
		{
			cv::Point X1 = baseImagePoints[i];
			cv::Point X2 = trackers[i]->getWindowCenter();
			sourcePlanePoints.at<float>(i, 0) = X1.x;
			sourcePlanePoints.at<float>(i, 1) = X1.y;
			destinationPlanePoints.at<float>(i, 0) = X2.x;
			destinationPlanePoints.at<float>(i, 1) = X2.y;
		}
		cv::Mat A = cv::findHomography(sourcePlanePoints, destinationPlanePoints);
		cv::Mat H1 = positionEstimator->getHomography(1);
		cv::Mat H2 = cv::Mat(3, 3, CV_64F);

		cv::gemm(A, H1, 1.0, cv::Mat(), 0.0, H2); 
		positionEstimator->setHomography(H2);
		isReady = true;
	}
}

void HomographyExtender::updateTargetHomography(int pan, int tilt, bool writeToHomography, bool writeToInterpolated)
{
    qDebug() << "updateTargetHomography with " << pan << ", " << tilt;
    double angle_x = - ( 3.1416 / 12 ) * tilt / (7789);
    double angle_y = - ( 3.1416 / 6 ) * pan / (15578);
	cv::Mat currentExtension = cv::Mat(3, 3, CV_64F);

    universalHomography(intrinsicParameters, positionEstimator->H_inv_orig, angle_x, angle_y, currentExtension);
	cv::invert(currentExtension, targetHomography);
    if (writeToHomography)
    {
        targetHomography.copyTo(positionEstimator->H);
        currentExtension.copyTo(positionEstimator->H_inv);
    }
    if (writeToInterpolated)
    {
        targetHomography.copyTo(positionEstimator->H_interpolated);
        currentExtension.copyTo(positionEstimator->H_inv_interpolated);
    }
}

void HomographyExtender::startInterpolation()
{
	this->startTime = std::clock();
	endTimePan = abs(endPan - initialPan) / panSpeed;
	endTimeTilt = abs(endTilt - initialTilt) / tiltSpeed;

	positionEstimator->H.copyTo(HCurrent);
	
    double initialXRotation = - ( 3.1416 / 12 ) * initialTilt / (7789);
    double initialYRotation = - ( 3.1416 / 6 ) * initialPan / (15578);
    double endXRotation = - ( 3.1416 / 12 ) * endTilt / (7789);
    double endYRotation = - ( 3.1416 / 6 ) * endPan / (15578);

	cv::Mat currentExtension = cv::Mat(3, 3, CV_64F);
	universalHomography(intrinsicParameters, positionEstimator->H_inv_orig, endXRotation, initialYRotation, currentExtension);
	cv::invert(currentExtension, targetHomographyTilt);
	universalHomography(intrinsicParameters, positionEstimator->H_inv_orig, initialXRotation, endYRotation, currentExtension);
	cv::invert(currentExtension, targetHomographyPan);
	
	universalHomography(intrinsicParameters, positionEstimator->H_inv_orig, endXRotation, endYRotation, currentExtension);
	cv::invert(currentExtension, targetHomography);

	if ((endPan - initialPan) != 0)
	{
		deltaPanParameters = targetHomographyPan - positionEstimator->H;
		deltaPanParameters /= (endPan - initialPan);
	}
	else
	{
		deltaPanParameters.setTo(cv::Scalar(0));
	}

	if ((endTilt - initialTilt) != 0)
	{
		deltaTiltParameters = targetHomographyTilt - positionEstimator->H;
		deltaTiltParameters /= (endTilt - initialTilt);
	}
	else
	{
		deltaTiltParameters.setTo(cv::Scalar(0));
	}

	interpolating = true;
    interpolationIsFinished = false;

    // When interpolation mode is enabled, we must update homography with no interpolation
    positionEstimator->setHomography(targetHomography, PositionEstimator::DIRECT);
}

void HomographyExtender::setCameraIntrinsicParameters(cv::Mat cameraIntrinsicParameters)
{
	this->intrinsicParameters = cameraIntrinsicParameters;
}

void HomographyExtender::setCameraSpeed(int panSpeed, int tiltSpeed)
{
	this->panSpeed = panSpeed;
	this->tiltSpeed = tiltSpeed;
}

void HomographyExtender::setCameraParameters(cv::Mat &intrinsicParameters, int panSpeed, int tiltSpeed)
{
	this->intrinsicParameters = intrinsicParameters;
	this->panSpeed = panSpeed;
	this->tiltSpeed = tiltSpeed;
}

void HomographyExtender::setInterpolationRanges(int initialPan, int initialTilt, int endPan, int endTilt)
{
	this->initialPan = initialPan;
	this->initialTilt = initialTilt;
	this->endPan = endPan;
	this->endTilt = endTilt;
}

cv::Mat HomographyExtender::updateHomographyInterpolation()
{
    cv::Mat currentParameters = cv::Mat(3, 3, CV_64F);
	if (interpolating)
	{
		cv::Mat currentPanParameters = cv::Mat(3, 3, CV_64F);
		cv::Mat currentTiltParameters = cv::Mat(3, 3, CV_64F);

		std::clock_t currentTime = clock();
		double t = (1000.0 * (currentTime - startTime)) / CLOCKS_PER_SEC;
		double currentDeltaPan = (panSpeed * (endPan - initialPan >= 0 ? 1 : -1)) * t;
		double currentDeltaTilt = (tiltSpeed * (endTilt - initialTilt >= 0 ? 1 : -1)) * t;

		if (t > endTimePan && t > endTimeTilt) 
		{
			interpolating = false;
            interpolationIsFinished = true;
            //targetHomography.copyTo(currentParameters);
            positionEstimator->setHomography(targetHomography, PositionEstimator::INTERPOLATED);
        }
		else
		{
			currentPanParameters = deltaPanParameters * (t > endTimePan ? endPan - initialPan : currentDeltaPan);
			currentTiltParameters = deltaTiltParameters * (t > endTimeTilt ? endTilt - initialTilt : currentDeltaTilt);

			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					currentParameters.at<double>(i, j) = currentPanParameters.at<double>(i, j) + currentTiltParameters.at<double>(i, j)  + HCurrent.at<double>(i, j);
				}
			}
            positionEstimator->setHomography(currentParameters, PositionEstimator::INTERPOLATED);
	
            ofstream myfile;
			myfile.open ("DebugInterpolation.txt", ios::app);
			myfile << "DPan," << (endPan - initialPan) << ",DTilt," << (endTilt - initialTilt) << ",";
			myfile << "SourceHomography,";
			int params = 4;
			for (int i = 0; i < params; ++i)
				myfile << HCurrent.at<double>(i / 3, i % 3) << ", ";
			myfile << "CurrentHomograghy,";
			for (int i = 0; i < params; ++i)
				myfile << currentParameters.at<double>(i / 3, i % 3) << ", ";
			myfile << "Target,";
			for (int i = 0; i < params; ++i)
				myfile << targetHomography.at<double>(i / 3, i % 3) << ", ";
			myfile << "currentPanParameters,";
			for (int i = 0; i < params; ++i)
				myfile << currentPanParameters.at<double>(i / 3, i % 3) << ", ";
			myfile << "TargetPan,";
			for (int i = 0; i < params; ++i)
				myfile << targetHomographyPan.at<double>(i / 3, i % 3) << ", ";
			myfile << "currentTiltParameters,";
			for (int i = 0; i < params; ++i)
				myfile << currentTiltParameters.at<double>(i / 3, i % 3) << ", ";
			myfile << "TargetTilt,";
			for (int i = 0; i < params; ++i)
				myfile << targetHomographyTilt.at<double>(i / 3, i % 3) << ", ";
			myfile << std::endl;
		}
    }
    else
	{
        //targetHomography.copyTo(currentParameters);
        positionEstimator->setHomography(targetHomography, PositionEstimator::INTERPOLATED);
    }
	return currentParameters;
}

void HomographyExtender::universalHomography(cv::Mat &cameraMatrix, cv::Mat &initialHomography, double x_angle, double y_angle, cv::Mat &newHomography)
{
	//Build X-rotation matrix
	double _rx[9];
	_rx[0] = 1;		_rx[1] = 0;				_rx[2] = 0;
	_rx[3] = 0;		_rx[4] = cos(x_angle);	_rx[5] = sin(x_angle);
	_rx[6] = 0;		_rx[7] = -sin(x_angle);	_rx[8] = cos(x_angle);
	cv::Mat Rx = cv::Mat(3, 3, CV_64F, _rx);
	
	//Build Y-rotation matrix	
	double _ry[9];
	_ry[0] = cos(y_angle);	_ry[1] = 0;		_ry[2] = -sin(y_angle);
	_ry[3] = 0;				_ry[4] = 1;		_ry[5] = 0;
	_ry[6] = sin(y_angle);	_ry[7] = 0;		_ry[8] = cos(y_angle);
	cv::Mat Ry = cv::Mat(3, 3, CV_64F, _ry);

	// Initial rotation
	cv::Mat cameraMatrixInverted = cv::Mat(3, 3, CV_64F);
	cv::invert(cameraMatrix, cameraMatrixInverted);
	
	// First multiplication: camera matrix inverted * initial homography
	cv::Mat cmiH = cvCreateMat(3, 3, CV_64F);

	cv::gemm(cameraMatrixInverted, initialHomography, 1, cv::Mat(), 0, cmiH);
	// Ri is the rotation matrix
	cv::Mat Ri = cv::Mat(3, 3, CV_64F);
	cv::gemm(cmiH, cameraMatrix, 1, cv::Mat(), 0, Ri);
	
	// Camera Rotation
	cv::Mat Rc = cv::Mat(3, 3, CV_64F);
	cv::gemm(Rx, Ry, 1, cv::Mat(), 0, Rc);
	cv::Mat R = cv::Mat(3, 3, CV_64F);
	cv::gemm(Ri, Rc, 1, cv::Mat(), 0, R);

	// Universal Homography
	cv::Mat K_R = cv::Mat(3, 3, CV_64F);
	cv::gemm(cameraMatrix, R, 1, cv::Mat(), 0, K_R);

	cv::gemm(K_R, cameraMatrixInverted, 1, cv::Mat(), 0, newHomography);
}

