#include "common/TrackingCommon.h"
#include "tracking/CuboidTrackerSingleFace.h"
#include <iostream>

CuboidTrackerSingleFace::CuboidTrackerSingleFace()
{
    type = TYPE_CUBOID_SINGLE_FACE;
    qDebug() << "Tracking Single Face";
}

int CuboidTrackerSingleFace::calculateFaceWithLeastError()
{
	int bestIndex = -1;
	double currentError = DBL_MAX;
	for (int i = 0; i < 6; ++i)
	{
		if (faceIsVisible[i])
		{
            cv::warpPerspective(trackers[i].I, trackers[i].I_w, trackers[i].W_3x3, trackers[i].I_w.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);
            double newError = trackers[i].computeErrorImage(trackers[i].T, trackers[i].I_w, trackers[i].error_image);
			//std::cout << newError << ",";
			if (newError < currentError)
			{
				currentError = newError;
				bestIndex = i;
			}
		}
	}
	//std::cout << "\n";
	return bestIndex;
}

int CuboidTrackerSingleFace::calculateFaceWithNormalMostAligned()
{
    if (lockFace)
        return face;

	cv::Mat centerTranslation = centerTransform.col(3);
	int bestIndex = -1;
	double currentValue = 0.0;

    cv::Mat normalsDebug = cv::Mat(6, 1, CV_32F);
    for (int i = 0; i < 6; ++i)
	{
		//if (faceIsVisible[i])
		if (true)
		{
			double newValue = centerTranslation.dot(normals[i]);
            normalsDebug.at<float>(i, 0) = newValue;
			if (currentValue < newValue)
			{
				currentValue = newValue;
				bestIndex = i;
			}
		}
	}
    TrackingCommon::mainController->displayData(std::string("Normals"), normalsDebug, DataDisplayer::APPEND);
	return bestIndex;
}

void CuboidTrackerSingleFace::trackObject(cv::Mat &newImage)
{
    // Sync camera calibration
    homographyDecomposer.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);
    if (this->cuboidTrackingMode == CuboidTracker::MULTI_VIEW)
	{
        //cuboidTracker->calculateTranslationFromGlobalTracker(newImage);
        trackers[face].trackObject(newImage);
        calculateTransformationFromHomography();
        calculateAllTransformationsFromSingleTransformation();
        calculateHomographiesFromTransformation();

        if (!lockFace)
        {
            printCenterMatrix();
            updateNormals();
            calculateVisibleFaces();

            if (cuboidTrackingOptimization == CuboidTracker::FACE_WITH_LEAST_ERROR)
            {
                for (int i = 0; i < 6; ++i)
                {
                    if (faceIsVisible[i])
                    {
                        trackers[i].I = trackers[face].I;
                    }
                }
                int leastErrorFace = calculateFaceWithLeastError();
                setBaseTracker(leastErrorFace);
            }
            else
            {
                int leastErrorFace = calculateFaceWithNormalMostAligned();
                setBaseTracker(leastErrorFace);
            }
        }
	}
	else
	{
        ESMTracker* tracker = getTracker(0);
		tracker->trackObject(newImage);
		calculateTransformationFromHomography();
		calculateAllTransformationsFromSingleTransformation();
	}
}

void CuboidTrackerSingleFace::highlightObject(cv::Mat &targetImage)
{
    for (int i = 0; i < 6; ++i)
    {
		if (i != face)
			trackers[i].highlightColor = cv::Scalar(255, 0, 0);		
        trackers[i].highlightObject(targetImage);
	}
	trackers[face].highlightColor = cv::Scalar(0, 0, 255);
    trackers[face].highlightObject(targetImage);
}
