#include "reconstruction/PositionEstimator.h"
#include "common/TrackingCommon.h"
#include <fstream>
#include <ctime>

PositionEstimator::PositionEstimator()
{
	transformationMode = INVERSE;
	zReal = 0.0;
	totalHomographyTraining = 0;
    isReady = false;
	
	H = cv::Mat(3, 3, CV_64F);
	H_orig = cv::Mat(3, 3, CV_64F);
	H_inv = cv::Mat(3, 3, CV_64F);
	H_inv_orig = cv::Mat(3, 3, CV_64F);

	homographyDecomposer = HomographyDecomposer();
}

void PositionEstimator::addTrainingPoint(cv::Point2f trainingPointImage, cv::Point2f trainingPointReal)
{
	if (totalHomographyTraining < maxTrainingPoints)
	{
		homographyTrainingImage[totalHomographyTraining] = trainingPointImage;
		homographyTrainingReal[totalHomographyTraining] = trainingPointReal;
		++totalHomographyTraining;
	}
}

void PositionEstimator::clearTrainingPoints()
{
    totalHomographyTraining = 0;
}

void PositionEstimator::clear()
{
    H.setTo(0);
    H_orig.setTo(0);
    H_inv.setTo(0);
    H_inv_orig.setTo(0);
    clearTrainingPoints();
    isReady = false;
}

void PositionEstimator::setReady(bool ready)
{
    this->isReady = ready;
}

void PositionEstimator::setRealHeight(double newHeight)
{
	zReal = newHeight;
}

int PositionEstimator::findHomography()
{
	if (totalHomographyTraining >= 4)
	{
		cv::Mat _pt1, _pt2;
		_pt1 = cv::Mat(1, totalHomographyTraining, CV_32FC2, &homographyTrainingImage);
		_pt2 = cv::Mat(1, totalHomographyTraining, CV_32FC2, &homographyTrainingReal);
		this->findHomography(_pt1, _pt2);
	}
	return 0;
}

int PositionEstimator::findHomography(const cv::Mat trainingPointsImage, const cv::Mat trainingPointsReal)
{
	H = cv::findHomography(trainingPointsReal, trainingPointsImage);
    H.copyTo(H_orig);
    H.copyTo(H_interpolated);
    H_inv = cv::findHomography(trainingPointsImage, trainingPointsReal);
    H_inv.copyTo(H_inv_orig);
    H_inv.copyTo(H_inv_interpolated);
    isReady = true;
	return 1;
}

cv::Mat PositionEstimator::getHomography(int originalHomography)
{
	if (originalHomography)
		return H_orig;
	return H;
}

void PositionEstimator::setHomography(cv::Mat homography, enum TransformationModes transformationMode)
{
	switch (transformationMode)
	{
		case INTERPOLATED: 
			homography.copyTo(H_interpolated); 
			cv::invert(H_interpolated, H_inv_interpolated, DECOMP_LU); 
			break;
		default:
			homography.copyTo(H);
			cv::invert(H, H_inv, DECOMP_LU);
	}
	
	
}

void PositionEstimator::showCoordinatesTransform(cv::Point2f img_coord, const char *windowName)
{
	int x = img_coord.x;
	int y = img_coord.y;

    cv::Point image_point = cv::Point( x, y );

	cv::Point3f real_point = this->calculateTransformation(image_point, transformationMode); 
}

cv::Point3f PositionEstimator::calculateTransformation(cv::Point2f test_point, enum TransformationModes transformationModes)
{
	double x = test_point.x;
	double y = test_point.y;

	double Z;
	double X;
	double Y;

	double _h[9];

    cv::Mat transform;
	switch (transformationModes)
	{
        case INVERSE: transform = H_inv; break;
        case INVERSE_ORIGINAL: transform = H_inv_orig; break;
        case DIRECT: transform = H; break;
        case DIRECT_ORIGINAL: transform = H_orig; break;
        case CORRECTED: transform = H_corrected; break;
        case CORRECTED_INVERSE: transform = H_inv_corrected; break;
        case INTERPOLATED: transform = H_interpolated; break;
        case INTERPOLATED_INVERSE: transform = H_inv_interpolated; break;
	}

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
            _h[i * 3 + j] = transform.at<double>(i, j);
		}
	}
	Z = 1.0 / (_h[6] * x + _h[7] * y + _h[8]);
	X = (_h[0] * x + _h[1] * y + _h[2]) * Z;
	Y = (_h[3] * x + _h[4] * y + _h[5]) * Z;

    return cv::Point3f(X, Y, 0);
}

void PositionEstimator::setObjectHeight(double objectHeight)
{
	this->objectHeight = objectHeight;
}

void PositionEstimator::updateHeightCorrection(enum TransformationModes transformationMode)
{
	switch (transformationMode)
	{
		case INTERPOLATED: homographyDecomposer.setHomography(H_interpolated); break;
		default: homographyDecomposer.setHomography(H);
	}
	homographyDecomposer.decompose();

	cv::Mat transform = cv::Mat(4, 4, CV_64F);
	cv::Mat newHomography = cv::Mat(3, 3, CV_64F);
	
	// Prepare the translation matrix
	cv::Mat applyTranslation = cv::Mat(4, 4, CV_64F);
	cv::setIdentity(applyTranslation);
	applyTranslation.at<double>(0, 3) = 0.0;
	applyTranslation.at<double>(1, 3) = 0.0;
	applyTranslation.at<double>(2, 3) = objectHeight;

	// Back project to image
    cv::gemm(homographyDecomposer.transform, applyTranslation, 1, cv::Mat(), 0, transform);
	CuboidTracker::reduceTransformationMatrix(transform, newHomography);
	cv::gemm(homographyDecomposer.cameraIntrinsicParameters, newHomography, 1, cv::Mat(), 0, newHomography);
	//newHomography /= homographyDecomposer.rotationalNormal;
	//newHomography  /= newHomography.at<double>(2, 2);

	newHomography.copyTo(H_corrected);
	cv::invert(H_corrected, H_inv_corrected);
}
