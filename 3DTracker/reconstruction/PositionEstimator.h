#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "opencv2/core/core.hpp"
#include "opencv/highgui.h"
#include "reconstruction/HomographyDecomposer.h"
#include "tracking/CuboidTracker.h"

class PositionEstimator
{
	public:
		static const int maxTrainingPoints = 20;
        enum TransformationModes {INVERSE, INVERSE_ORIGINAL, DIRECT, DIRECT_ORIGINAL, CORRECTED, CORRECTED_INVERSE, INTERPOLATED, INTERPOLATED_INVERSE} transformationMode;

		double zReal;
		cv::Point2f homographyTrainingImage[maxTrainingPoints];
		cv::Point2f homographyTrainingReal[maxTrainingPoints];
		int totalHomographyTraining;

		cv::Mat H;
		cv::Mat H_orig;
		cv::Mat H_inv;
		cv::Mat H_inv_orig;

		bool isReady;

		CvFont font;
		CvFont font_large;

		double objectHeight;
		cv::Mat H_corrected;
		cv::Mat H_inv_corrected;
		HomographyDecomposer homographyDecomposer;

		cv::Mat H_interpolated;
		cv::Mat H_inv_interpolated;

		PositionEstimator();
		void addTrainingPoint(cv::Point2f trainingPointImage, cv::Point2f trainingPointReal);
        void clearTrainingPoints();
        void clear();
		int findHomography();
		int findHomography(const cv::Mat trainingPointsImage, const cv::Mat trainingPointsReal);
        void setReady(bool ready);
        cv::Mat getHomography(int originalHomography = 0);
		void setHomography(cv::Mat homography, enum TransformationModes transformationMode = DIRECT);
		void setRealHeight(double newHeight);
		void showCoordinatesTransform(cv::Point2f img_coord, const char *windowName);
		cv::Point3f calculateTransformation(cv::Point2f test_point, enum TransformationModes transformationModes = INVERSE);

		// Height Correction through Homography Decomposition
		void setObjectHeight(double objectHeight);
		void updateHeightCorrection(enum TransformationModes transformationMode = DIRECT);
};

#endif
