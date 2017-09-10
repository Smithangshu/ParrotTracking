#include "tracking/MeanShiftWithScaling.h"
#include "common/TrackingCommon.h"

MeanShiftWithScaling::MeanShiftWithScaling()
{
    MeanShiftTracker();
}

void MeanShiftWithScaling::trackObject(cv::Mat &newImage)
{
    if (TrackingCommon::scalingMode == TrackingCommon::SCALE_WITH_HOMOGRAPHY)
    {
        if (TrackingCommon::positionEstimatorController.isReady() && TrackingCommon::enableScalingWithHomography)
            updateScale();
        MeanShiftTracker::trackObject(newImage);
    }
    else // TrackingCommon::SCALE_WITH_MODEL
    {
        double scales[3];
        double errors[3];
        scales[0] = currentScale;
        scales[1] = 1.1 * currentScale;
        scales[2] = 0.9 * currentScale;
        int leastErrorIndex = 0;
        cv::Point currentWindowCenter = trackingWindowCenter;
        cv::Point newWindowCenter[3];
        cv::Rect newTrackingWindow[3];
        cv::Rect testTrackingWindow[3];

        for (int i = 0; i < 3; ++i)
        {
            testTrackingWindow[i] = cv::Rect(0, 0, originalWindow.width * scales[i], originalWindow.height * scales[i]);
            // cambiar tamano
            trackingWindow = testTrackingWindow[i];
            trackingWindow = calculateTrackingWindow(currentWindowCenter);
            MeanShiftTracker::trackObject(newImage);
            newWindowCenter[i] = trackingWindowCenter;
            newTrackingWindow[i] = trackingWindow;
            errors[i] = lastError;
        }
        //qDebug() << "Errors: " << errors[0] << ", " << errors[1] << ", " << errors[2];

        leastErrorIndex = 0;
        for (int i = 1; i <= 2; i++){
            if (errors[i] > errors[leastErrorIndex]) {
            //if(i == 2) {
                leastErrorIndex = i;
            }
        }

     //   if(scales[leastErrorIndex] < 1.5 || scales[leastErrorIndex] > .5){
        currentScale = scales[leastErrorIndex];
        qDebug() << "currentScale " << currentScale;

        trackingWindowCenter = newWindowCenter[leastErrorIndex];
        trackingWindow = newTrackingWindow[leastErrorIndex];
       // }
    }
}

void MeanShiftWithScaling::updateScale()
{
    PositionEstimator* positionEstimator = &TrackingCommon::positionEstimatorController.positionEstimator;
    // a means aphostrophe
    cv::Point3f ca_r = positionEstimator->calculateTransformation(cv::Point2f(trackingWindowCenter.x, trackingWindowCenter.y));
    cv::Point3f p1_r = positionEstimator->calculateTransformation(cv::Point2f(originalWindow.x, originalWindow.y));
    cv::Point3f p2_r = positionEstimator->calculateTransformation(cv::Point2f(originalWindow.x + originalWindow.width, originalWindow.y + originalWindow.height));

    cv::Point2f deltaP =  cv::Point2f((p2_r.x - p1_r.x) / 2.0, (p2_r.y - p1_r.y) / 2.0);
    cv::Point2f p1a_r = cv::Point2f(ca_r.x - deltaP.x, ca_r.y - deltaP.y);
    cv::Point2f p2a_r = cv::Point2f(ca_r.x + deltaP.x, ca_r.y + deltaP.y);

    cv::Point3f p1a, p2a;
    if (TrackingCommon::enableHeightCorrection && !TrackingCommon::positionEstimatorController.positionEstimator.H_corrected.empty()) {
        p1a = positionEstimator->calculateTransformation(p1a_r, PositionEstimator::CORRECTED);
        p2a = positionEstimator->calculateTransformation(p2a_r, PositionEstimator::CORRECTED);
    } else {
        p1a = positionEstimator->calculateTransformation(p1a_r, PositionEstimator::DIRECT);
        p2a = positionEstimator->calculateTransformation(p2a_r, PositionEstimator::DIRECT);
    }

    cv::Rect newTrackingWindow = cv::Rect(p1a.x, p1a.y, p2a.x - p1a.x, p2a.y - p1a.y);
    trackingWindow = newTrackingWindow;
}

//void MeanShiftWithScaling::updateScale()
//{
//	cv::Point3f A = positionEstimator->calculateTransformation(cv::Point2f(originalWindow.x, originalWindow.y));
//	cv::Point3f B = positionEstimator->calculateTransformation(cv::Point2f(originalWindow.x + originalWindow.width, originalWindow.y + originalWindow.height));

//	cv::Rect unscaledWindow = cv::Rect(trackingWindowCenter.x - originalWindow.width / 2, trackingWindowCenter.y - originalWindow.height / 2, originalWindow.width, originalWindow.height);
//	cv::Point3f Aa = positionEstimator->calculateTransformation(cv::Point(unscaledWindow.x, unscaledWindow.y));
//	cv::Point3f Ba = positionEstimator->calculateTransformation(cv::Point(unscaledWindow.x + unscaledWindow.width, unscaledWindow.y + unscaledWindow.height));

//	cv::Point2f C = cv::Point2f(0.5 * (B.x - A.x), 0.5 * (B.y - A.y));
//	cv::Point2f Ca = cv::Point2f(0.5 * (Ba.x + Aa.x), 0.5 * (Ba.y + Aa.y));

//	cv::Point2f Aaa = cv::Point2f(Ca.x - C.x, Ca.y - C.y);
//	cv::Point2f Baa = cv::Point2f(Ca.x + C.x, Ca.y + C.y);

//    cv::Point3f Xa = positionEstimator->calculateTransformation(Aaa, PositionEstimator::DIRECT);
//    cv::Point3f Xb = positionEstimator->calculateTransformation(Baa, PositionEstimator::DIRECT);

//	cv::Rect newTrackingWindow = cv::Rect(Xa.x, Xa.y, Xb.x - Xa.x, Xb.y - Xa.y);
//	trackingWindow = newTrackingWindow;
//}

//void MeanShiftWithScaling::updateScale()
//{
//W = cv::Mat(3, 3, CV_64F);
//cv::setIdentity(W);
//A = cv::Mat(3, 3, CV_64F);
//cv::setIdentity(A);
//homography = cv::Mat(3, 3, CV_64F);
//cv::setIdentity(homography);

//Xr1 = cv::Mat(3, 1, CV_64F);
//cv::setIdentity(Xr1);
//Xr2 = cv::Mat(3, 1, CV_64F);
//cv::setIdentity(Xr2);

//imgWidthSource = cv::Mat(3, 1, CV_64F);
//cv::setIdentity(imgWidthSource);
//imgWidthDestination = cv::Mat(3, 1, CV_64F);
//cv::setIdentity(imgWidthDestination);

//Winv = cv::Mat(3, 3, CV_64F);
//Ginv = cv::Mat(3, 3, CV_64F);
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
