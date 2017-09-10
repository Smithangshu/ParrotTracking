#include "MeanShiftWithScaling.h"
#include <iostream>

MeanShiftWithScaling::MeanShiftWithScaling()
{
	MeanShiftTracker::MeanShiftTracker();
	W = cv::Mat(3, 3, CV_64F);
	cv::setIdentity(W);
	A = cv::Mat(3, 3, CV_64F);
	cv::setIdentity(A);
	homography = cv::Mat(3, 3, CV_64F);
	cv::setIdentity(homography);

	Xr1 = cv::Mat(3, 1, CV_64F);
	cv::setIdentity(Xr1);
	Xr2 = cv::Mat(3, 1, CV_64F);
	cv::setIdentity(Xr2);

	imgWidthSource = cv::Mat(3, 1, CV_64F);
	cv::setIdentity(imgWidthSource);
	imgWidthDestination = cv::Mat(3, 1, CV_64F);
	cv::setIdentity(imgWidthDestination);

	Winv = cv::Mat(3, 3, CV_64F);
	Ginv = cv::Mat(3, 3, CV_64F);
}

void MeanShiftWithScaling::setPositionEstimator(PositionEstimator* positionEstimator)
{
	this->positionEstimator = positionEstimator;
}

void MeanShiftWithScaling::trackObject(IplImage *newImage)
{
	std::cout << "Updating scale\n";
	updateScale();
	MeanShiftTracker::trackObject(newImage);
}

void MeanShiftWithScaling::updateScale()
{
	cv::Point3f A = positionEstimator->calculateTransformation(cv::Point2f(originalWindow.x, originalWindow.y), PositionEstimator::INVERSE_ORIGINAL);
	cv::Point3f B = positionEstimator->calculateTransformation(cv::Point2f(originalWindow.x + originalWindow.width, originalWindow.y + originalWindow.height), PositionEstimator::INVERSE_ORIGINAL);

	cv::Rect unscaledWindow = cv::Rect(trackingWindowCenter.x - originalWindow.width / 2, trackingWindowCenter.y - originalWindow.height / 2, originalWindow.width, originalWindow.height);
	cv::Point3f Aa = positionEstimator->calculateTransformation(cv::Point(unscaledWindow.x, unscaledWindow.y));
	cv::Point3f Ba = positionEstimator->calculateTransformation(cv::Point(unscaledWindow.x + unscaledWindow.width, unscaledWindow.y + unscaledWindow.height));

	cv::Point2f C = cv::Point2f(0.5 * (B.x - A.x), 0.5 * (B.y - A.y));
	cv::Point2f Ca = cv::Point2f(0.5 * (Ba.x + Aa.x), 0.5 * (Ba.y + Aa.y));

	cv::Point2f Aaa = cv::Point2f(Ca.x - C.x, Ca.y - C.y);
	cv::Point2f Baa = cv::Point2f(Ca.x + C.x, Ca.y + C.y);

	cv::Point3f Xa = positionEstimator->calculateTransformation(Aaa, PositionEstimator::TransformationModes::DIRECT);
	cv::Point3f Xb = positionEstimator->calculateTransformation(Baa, PositionEstimator::TransformationModes::DIRECT);
	
	cv::Rect newTrackingWindow = cv::Rect(Xa.x, Xa.y, Xb.x - Xa.x, Xb.y - Xa.y);
	trackingWindow = newTrackingWindow;
}

//void MeanShiftWithScaling::updateScale()
//{
//	cv::Point3f A = positionEstimator->calculateTransformation(cv::Point(originalWindow.x, originalWindow.y));
//	cv::Point3f B = positionEstimator->calculateTransformation(cv::Point(originalWindow.x + originalWindow.width, originalWindow.y + originalWindow.height));
//	cv::Rect unscaledWindow = cv::Rect(trackingWindowCenter.x - originalWindow.width / 2, trackingWindowCenter.y - originalWindow.height / 2, originalWindow.width, originalWindow.height);
//	cv::Point3f A_ = positionEstimator->calculateTransformation(cv::Point(unscaledWindow.x, unscaledWindow.y));
//	cv::Point3f B_ = positionEstimator->calculateTransformation(cv::Point(unscaledWindow.x + unscaledWindow.width, unscaledWindow.y + unscaledWindow.height));
//	Xr1.at<double>(0, 0) = B.x - A.x;
//	Xr1.at<double>(1, 0) = B.y - A.y;
//	Xr2.at<double>(0, 0) = B_.x - A_.x;
//	Xr2.at<double>(1, 0) = B_.y - A_.y;
//	double alphaX = Xr1.at<double>(0, 0) / Xr2.at<double>(0, 0);
//	double alphaY = Xr1.at<double>(1, 0) / Xr2.at<double>(1, 0);
//	std::cout << "\nalphaX " << alphaX << " alphaY" << alphaY;
//	W.at<double>(0, 0) = alphaX;
//	W.at<double>(1, 1) = alphaY;
//	cv::invert(W, Winv, DECOMP_LU);
//	cv::invert(positionEstimator->getHomography(0), Ginv, DECOMP_LU);
//
//	imgWidthSource.at<double>(0, 0) = originalWindow.width;
//	imgWidthSource.at<double>(1, 0) = originalWindow.height;
//	cv::gemm(positionEstimator->getHomography(0), Winv, 1.0, cv::Mat(), 0, Winv);
//	cv::gemm(Winv, Ginv, 1.0, cv::Mat(), 0, this->A);
//	cv::gemm(this->A, imgWidthSource, 1.0, cv::Mat(), 0, imgWidthDestination);
//	std::cout << "\nwidth " << imgWidthDestination.at<double>(0, 0) << " height" << imgWidthDestination.at<double>(1, 0);
//}