#ifndef HOMOGRAPHY_EXTENSION_H
#define HOMOGRAPHY_EXTENSION_H

#include <ctime>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "tracking/MeanShiftTracker.h"
#include "reconstruction/PositionEstimator.h"

class HomographyExtender
{
	private:
		// User for feature based
		MeanShiftTracker* trackers[10];
		cv::Point baseImagePoints[10];
		int imagePointsSet;
		int currentTrackers;
		int isReady;

		// For homography interpolation
		std::clock_t startTime;
		double endTimePan;
		double endTimeTilt;
		int initialPan;
		int initialTilt;
		int endPan;
		int endTilt;
		cv::Mat intrinsicParameters;
		int panSpeed;
		int tiltSpeed;
		cv::Mat deltaPanParameters;
		cv::Mat deltaTiltParameters;
		cv::Mat HCurrent;
		cv::Mat targetHomographyPan;
		cv::Mat targetHomographyTilt;
	public:
		cv::Mat targetHomography;

        enum HomographyExtensionMethods {INTERPOLATION, FEATURE_BASED} homographyExtensionMethod;

		PositionEstimator* positionEstimator;
		bool interpolating;
        bool interpolationIsFinished;

		HomographyExtender();
		void setPositionEstimator(PositionEstimator* positionEstimator);
		void setExtensionMethod(enum HomographyExtensionMethods homographyExtensionMethod);
        void trackAllObjects(cv::Mat &image);
		void universalHomography(cv::Mat &cameraMatrix, cv::Mat &initialHomography, double x_angle, double y_angle, cv::Mat &newHomography);
		void updateHomography();

		// These are specific to feature based
		void updateHomographyFeatureBased();
		void addTrackingWindow(MeanShiftTracker* tracker);
        void highlightAllObjects(cv::Mat &image);
		int getTotalPoints();
		
		// Target Homography Estimation
        void updateTargetHomography(int pan, int tilt, bool writeToHomography = false, bool writeToInterpolated = false);

		// For homography interpolation
		void startInterpolation();
		void setCameraIntrinsicParameters(cv::Mat cameraIntrinsicParameters);
		void setCameraSpeed(int panSpeed, int tiltSpeed);
		void setCameraParameters(cv::Mat &intrinsicParameters, int panSpeed, int tiltSpeed);
		void setInterpolationRanges(int initialPan, int initialTilt, int endPan, int endTilt);
		cv::Mat updateHomographyInterpolation();
};

#endif
