#ifndef HOMOGRAPHY_DECOMPOSER_H
#define HOMOGRAPHY_DECOMPOSER_H

#include "opencv2/core/core.hpp"
#include <math.h>
#include <stdio.h>
#include <errno.h>

using namespace cv;

class HomographyDecomposer
{
	private:
		cv::Point3f normalizedCrossProduct(Point3f X, Point3f Y);

	public:
        enum DecompMethods {ANALYTICAL, ITERATIVE, ROTATIONAL} decompMethod;

        cv::Mat homography;
		cv::Mat cameraIntrinsicParameters;
        cv::Mat transform;

		double rotationalNormal;
        double decompositionError;

		HomographyDecomposer();
		void decompose();
		void setMethod(enum DecompMethods decompMethod);
		void setHomography(cv::Mat &homography);
		void setCameraIntrinsicParameters(cv::Mat &cameraIntrinsicParameters);
		void decomposeAnalytical();
		void decomposeIterative();
		void decomposeRotational();

		static cv::Point3f getEulerAngles(cv::Mat transform);
		static cv::Point3f getAxisAngles(cv::Mat transform);
		static cv::Point3f getTranslation(cv::Mat transform);
};

#endif
