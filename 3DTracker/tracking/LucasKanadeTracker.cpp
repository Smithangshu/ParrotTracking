#include "tracking/LucasKanadeTracker.h"
#include "common/TrackingCommon.h"
#include <QDebug>
#include <QString>

LucasKanadeTracker::LucasKanadeTracker()
{
    epsilon = 0.02;
    maxIterations = 15;
//    epsilon = 0.001;
//    maxIterations = 30;
//    epsilon = 0.0002;
//    maxIterations = 50;
    topPyramid = 2;

    fromTransformToParameters = cv::Mat(3, 3, CV_32F);
    cv::setIdentity(fromTransformToParameters);
    fromParametersToTransform = cv::Mat(8, 1, CV_32F);
    fromParametersToTransform.setTo(0);
    fromParametersToTransform.at<float>(0, 0) = -1;
    fromParametersToTransform.at<float>(4, 0) = -1;

    // Default image formats
    useColor = false;
    imageFormat8UCX = CV_8UC1;
    imageFormat16SCX = CV_16SC1;
    imageFormat32FCX = CV_32FC1;
}

void LucasKanadeTracker::setTargetObject(cv::Mat &image, cv::Rect selection)
{
	Tracker::setTargetObject(image, selection);

    parameters = cv::Mat(8, 1, CV_32F);
	SET_VECTOR_8(parameters, 0, 0, 0, 0, 0, 0, selection.x, selection.y);

    T = cv::Mat(targetObject.size(), CV_8UC1);

    cv::cvtColor(targetObject, T, CV_RGB2GRAY);

    I = cv::Mat(image.size(), CV_8UC1);
    W_3x3 = cv::Mat(3, 3, CV_32FC1);
    I_w = cv::Mat(T.size(), CV_8UC1);

    I_w_gradient_x = cv::Mat(T.size(), CV_16SC1);
    I_w_gradient_y = cv::Mat(T.size(), CV_16SC1);

    I_gradient_x = cv::Mat(I.size(), CV_16SC1);
    I_gradient_y = cv::Mat(I.size(), CV_16SC1);
    warped_image = image.clone();
    error_image =cv::Mat(T.size(), CV_16SC1);
		
    H_ = cv::Mat(8, 8, CV_32F);
    H_inv = cv::Mat(8, 8, CV_32F);
    dp_right_operand = cv::Mat(8, 1, CV_32F);
    dp = cv::Mat(8, 1, CV_32F);
	for (int i = 0; i < 8; ++i)
        steepest_descend_images[i] = cv::Mat(T.size(), CV_32FC1);
	for (int i = 0; i < 8; ++i) 
	{
        dw_dp[0][i] = cv::Mat(T.size(), CV_32FC1);
        dw_dp[1][i] = cv::Mat(T.size(), CV_32FC1);
	}

    //highlightColor = cv::Scalar(255, 0, 0);
    highlightColor = cv::Scalar(0, 0, 0);
    highlightLineWidth = 2;

    setupPyramids();

    for (int i = 0; i < pyramids; ++i)
    {
        templateWeight[i] = cv::Mat(templates[i].size(), CV_32FC1);
        templateWeight[i].setTo(1.0);
        for (int y = 0; y < templateWeight[i].rows; ++y)
        {
            for (int x = 0; x < templateWeight[i].cols; ++x)
            {
                // This generates symetric kernel
                float distanceX = std::abs(templateWeight[i].cols / 2 - x) / ((float) templateWeight[i].cols);
                float distanceY = std::abs(templateWeight[i].rows / 2 - y) / ((float) templateWeight[i].rows);
                float length = std::sqrt((float) (distanceX * distanceX) + (float) (distanceY * distanceY));
                templateWeight[i].at<float>(y, x) = 1.0 * (1 - std::pow(length, 2));
                // This generates symetric kernel
                //templateWeight[i].at<float>(y, x) = 1.0 * (1 - length * length);
            }
        }

        std::stringstream displayMatName;
        displayMatName << "Mask" << i;
        //TrackingCommon::mainController->displayData(displayMatName.str(), templateWeight[i], DataDisplayer::FULL);
    }

    newTransform = cv::Mat(3, 3, CV_32F);

    scaleTransform = cv::Mat(3, 3, CV_32F);
    restoreTransform = cv::Mat(3, 3, CV_32F);

    cv::setIdentity(scaleTransform);
    scaleTransform.at<float>(0, 0) = 0.5;
    scaleTransform.at<float>(1, 1) = 0.5;

    cv::setIdentity(restoreTransform);
    restoreTransform.at<float>(0, 0) = 2.0;
    restoreTransform.at<float>(1, 1) = 2.0;
}

void LucasKanadeTracker::highlightObject(cv::Mat &targetImage)
{
    drawWarpedRect(targetImage);
}

void LucasKanadeTracker::trackObject(cv::Mat &newImage)
{
}

void LucasKanadeTracker::calculateJacobians(cv::Mat dw_dp[2][8], cv::Mat &W)
{
    float p1, p2, p3, p4, p5, p6, p7, p8;
	GET_VECTOR_8(W, p1, p2, p3, p4, p5, p6, p7, p8);

    for (int y = 0; y < dw_dp[0][0].rows; ++y)
	{
        for (int x = 0; x < dw_dp[0][0].cols; ++x)
		{
            float factor = 1 / (1 + p3 * x +  p6 * y);
            float factor_squared = factor * factor;
			
            dw_dp[0][0].at<float>(y, x) = x * factor;
            dw_dp[0][1].at<float>(y, x) = 0;
            dw_dp[0][2].at<float>(y, x) = (-x * ((1 + p1) * x + p4 * y + p7)) * factor_squared;
            dw_dp[0][3].at<float>(y, x) = y * factor;
            dw_dp[0][4].at<float>(y, x) = 0;
            dw_dp[0][5].at<float>(y, x) = (-y * ((1 + p1) * x + p4 * y + p7)) * factor_squared;
            dw_dp[0][6].at<float>(y, x) = 1 * factor;
            dw_dp[0][7].at<float>(y, x) = 0;

            dw_dp[1][0].at<float>(y, x) = 0;
            dw_dp[1][1].at<float>(y, x) = x * factor;
            dw_dp[1][2].at<float>(y, x) = (-x * (p2 * x + (1 + p5) * y + p8)) * factor_squared;
            dw_dp[1][3].at<float>(y, x) = 0;
            dw_dp[1][4].at<float>(y, x) = y * factor;
            dw_dp[1][5].at<float>(y, x) = (-y * (p2 * x + (1 + p5) * y + p8)) * factor_squared;
            dw_dp[1][6].at<float>(y, x) = 0;
            dw_dp[1][7].at<float>(y, x) = 1 * factor;
		}
	}
}

double LucasKanadeTracker::computeErrorImage(cv::Mat &image_1, cv::Mat &image_2, cv::Mat &error_image) {
    double total_error = 0.0;
    for (int y = 0; y < T.rows; ++y)
    {
        for (int x = 0; x < T.cols; ++x)
        {
            error_image.at<short>(y, x) = image_1.at<uchar>(y, x) - image_2.at<uchar>(y, x);
            total_error += pow((double) error_image.at<short>(y, x), 2);
        }
    }
    return total_error;
}

void LucasKanadeTracker::computeSteepestDescendImages(cv::Mat &gradient_x, cv::Mat &gradient_y, cv::Mat jacobians[2][8], cv::Mat sd_images[8]) {
    for (int i = 0; i < 8; ++i)
    {
        for (int y = 0; y < T.rows; ++y)
        {
            for (int x = 0; x < T.cols; ++x)
            {
                double Tx = gradient_x.at<short>(y, x) * jacobians[0][i].at<float>(y, x);
                double Ty = gradient_y.at<short>(y, x) * jacobians[1][i].at<float>(y, x);
                sd_images[i].at<float>(y, x) = Tx + Ty;
			}
		}
	}
}

void LucasKanadeTracker::computeHessian(cv::Mat sd_images[8], cv::Mat &H) {
    H.setTo(0);
    for (int i = 0; i < 8; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            for (int y = 0; y < T.rows; ++y)
            {
                for (int x = 0; x < T.cols; ++x)
                {
                    H.at<float>(j, i) += sd_images[i].at<float>(y, x) * sd_images[j].at<float>(y, x);
				}
			}
		}
	}
}

void LucasKanadeTracker::computeSteepestDescendUpdate(cv::Mat sd_images[8], cv::Mat &error_image, cv::Mat &dp_right_operand) {
    dp_right_operand.setTo(0);
    for (int i = 0; i < 8; ++i)
    {
        for (int y = 0; y < T.rows; ++y)
        {
            for (int x = 0; x < T.cols; ++x)
            {
                dp_right_operand.at<float>(i, 0) += sd_images[i].at<float>(y, x) * error_image.at<short>(y, x);
			}
		}
	}
}

void LucasKanadeTracker::drawWarpedRect(cv::Mat &pImage) {
    drawWarpedRect(pImage, W_3x3, this->trackingWindow.width, this->trackingWindow.height);
}

void LucasKanadeTracker::drawWarpedRect(cv::Mat &pImage, cv::Mat &homography, int width, int height) {
    cv::Rect rect = cv::Rect(0, 0, width, height);
    cv::Point sourcePointsInner[4];
    cv::Point sourcePoints[4];
    sourcePoints[0] = cv::Point(rect.x, rect.y); // tl
    sourcePoints[1] = cv::Point(rect.x + rect.width, rect.y); // tr
    sourcePoints[2] = cv::Point(rect.x + rect.width, rect.y + rect.height); // br
    sourcePoints[3] = cv::Point(rect.x, rect.y + rect.height); // bl

    int offsetInnerWidth = rect.width * 0.1;
    int offsetInnerHeight = rect.height * 0.1;
    sourcePointsInner[0] = cv::Point(rect.x + offsetInnerWidth, rect.y + offsetInnerHeight); // tl
    sourcePointsInner[1] = cv::Point(rect.x + rect.width - offsetInnerWidth, rect.y + offsetInnerHeight); // tr
    sourcePointsInner[2] = cv::Point(rect.x + rect.width - offsetInnerWidth, rect.y + rect.height - offsetInnerHeight); // br
    sourcePointsInner[3] = cv::Point(rect.x + offsetInnerWidth, rect.y + rect.height - offsetInnerHeight); // bl
    cv::Point targetPoints[4];
    cv::Point targetPointsOffset[4];

    cv::Mat targetPoint = cv::Mat(3, 1, CV_32FC1);
    cv::Mat sourcePoint = cv::Mat(3, 1, CV_32FC1);
    sourcePoint.setTo(1);
    targetPoint.setTo(1);
    for (int i = 0; i < 4; ++i)
    {
        // Outer lines
        sourcePoint.at<float>(0, 0) = sourcePoints[i].x;
        sourcePoint.at<float>(1, 0) = sourcePoints[i].y;
        targetPoint = homography * sourcePoint;
        targetPoint /= targetPoint.at<float>(2, 0);
        targetPoints[i].x = targetPoint.at<float>(0, 0);
        targetPoints[i].y = targetPoint.at<float>(1, 0);

        // Inner lines
        sourcePoint.at<float>(0, 0) = sourcePointsInner[i].x;
        sourcePoint.at<float>(1, 0) = sourcePointsInner[i].y;
        targetPoint = homography * sourcePoint;
        targetPoint /= targetPoint.at<float>(2, 0);
        targetPointsOffset[i].x = targetPoint.at<float>(0, 0);
        targetPointsOffset[i].y = targetPoint.at<float>(1, 0);
    }

    // draw rectangle
    for (int i = 0; i < 4; ++i)
    {
        cv::line(pImage, targetPoints[i], targetPoints[(i + 1) % 4], highlightColor, 2, CV_AA);
        //cv::line(pImage, targetPoints[i], targetPoints[(i + 1) % 4], highlightColor, highlightLineWidth, CV_AA);
        //cv::line(pImage, targetPointsOffset[i], targetPointsOffset[(i + 1) % 4], highlightColor, 1);
    }

}

cv::Point LucasKanadeTracker::pointTransformPerspective(cv::Point point, cv::Mat &W) {
	double p1, p2, p3, p4, p5, p6, p7, p8;
	GET_VECTOR_8(W, p1, p2, p3, p4, p5, p6, p7, p8);
    cv::Point new_point = cv::Point(0, 0);

    double factor = 1 + p3 * point.x + p6 * point.y;
    assert(factor != 0);
    new_point.x = (1 / factor) * ((1 + p1) * point.x + p4 * point.y + p7);
    new_point.y = (1 / factor) * (p2 * point.x + (1 + p5) * point.y + p8);

    return new_point;
}

double LucasKanadeTracker::computeSSDBetweenTemplateAndWarpedImage() {
	SET_TRANSFORM_MAT(W_3x3, parameters);
    cv::warpPerspective(I, I_w, W_3x3, I_w.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);
	return computeErrorImage(T, I_w, error_image);
}

void LucasKanadeTracker::updateWarpAdditive(cv::Mat &dp, cv::Mat &W_dest) {
	double p1, p2, p3, p4, p5, p6, p7, p8;
	GET_VECTOR_8(W_dest, p1, p2, p3, p4, p5, p6, p7, p8);

	double dp1, dp2, dp3, dp4, dp5, dp6, dp7, dp8;
	GET_VECTOR_8(dp, dp1, dp2, dp3, dp4, dp5, dp6, dp7, dp8);

	double new_p1, new_p2, new_p3, new_p4, new_p5, new_p6, new_p7, new_p8;

	new_p1 = p1 + dp1;
	new_p2 = p2 + dp2;
	new_p3 = p3 + dp3;
	new_p4 = p4 + dp4;
	new_p5 = p5 + dp5;
	new_p6 = p6 + dp6;
	new_p7 = p7 + dp7;
	new_p8 = p8 + dp8;

	SET_VECTOR_8(W_dest, new_p1, new_p2, new_p3, new_p4, new_p5, new_p6, new_p7, new_p8);
}

void LucasKanadeTracker::updateWarpInverseCompositional(cv::Mat &dp, cv::Mat &W_dest) {
    cv::Mat delta_M = cv::Mat(3, 3, CV_32F);
	SET_TRANSFORM_MAT(delta_M, dp);

    cv::Mat delta_M_inv = cv::Mat(3, 3, CV_32F);
    cv::invert(delta_M, delta_M_inv, CV_LU);

    cv::Mat W = cv::Mat(3, 3, CV_32F);
	SET_TRANSFORM_MAT(W, W_dest);

    cv::Mat comp_M = cv::Mat(3, 3, CV_32F);
    cv::gemm(W, delta_M_inv, 1, cv::Mat(), 0, comp_M);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
            comp_M.at<float>(i, j) = comp_M.at<float>(i, j) / comp_M.at<float>(2, 2);
		}
	}
	SET_TRANSFORM_VECTOR_8(W_dest, comp_M);
}

void LucasKanadeTracker::updateWarpCompositional(cv::Mat &dp, cv::Mat &W_dest) {
    cv::Mat delta_M = cv::Mat(3, 3, CV_32F);
	SET_TRANSFORM_MAT(delta_M, dp);

    cv::Mat W = cv::Mat(3, 3, CV_32F);
	SET_TRANSFORM_MAT(W, W_dest);

    cv::Mat comp_M = cv::Mat(3, 3, CV_32F);
    cv:gemm(W, delta_M, 1.0, 0, 0, comp_M);

    comp_M /= comp_M.at<float>(2, 2);

	SET_TRANSFORM_VECTOR_8(W_dest, comp_M);
}

void LucasKanadeTracker::setHomography(cv::Mat &homography, bool convertMatFormat)
{
    if (convertMatFormat)
    {
        homography.convertTo(W_3x3, CV_32F);
    }
    else
    {
        homography.copyTo(W_3x3);
    }

	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 2 || (j < 3 && i < 2); ++j)
            parameters.at<float>(3 * i + j, 0) = W_3x3.at<float>(j, i);
    parameters.at<float>(0, 0) -= 1;
    parameters.at<float>(4, 0) -= 1;
}

void LucasKanadeTracker::setupPyramids()
{
    templates[0] = T.clone();
    templates[1] = cv::Mat(T.rows / 2, T.cols / 2, T.type());
    cv::pyrDown(templates[0], templates[1]);
    templates[2] = cv::Mat(T.rows / 4, T.cols / 4, T.type());
    cv::pyrDown(templates[1], templates[2]);

    int floatMatFormat = T.channels() == 3 ? CV_16SC3 : CV_16SC1;

    for (int i = 0; i < pyramids; ++i)
    {
        imagePyramids[i] = cv::Mat(templates[i].size(), T.type());
        imagePyramidsDisplay[i] = cv::Mat(templates[i].size(), T.type());
        warpedImages[i] = cv::Mat(templates[i].size(), T.type());
        warpedImagesGradientX[i] = cv::Mat(templates[i].size(), floatMatFormat);
        warpedImagesGradientY[i] = cv::Mat(templates[i].size(), floatMatFormat);
    }

    for (int i = 0; i < pyramids; ++i)
    {
        qDebug() << "New Pyramid sizes: " << templates[i].cols  << ", " << templates[i].rows;
    }
}

cv::Mat LucasKanadeTracker::changeTransformScale(cv::Mat transform, cv::Rect templateSize, float factor)
{
//        TrackingCommon::mainController->displayData("W_3x3", W_3x3, DataDisplayer::FULL);
//        sleep(1);
    cv::Mat sourcePlanePoints = cv::Mat(4, 2, CV_32F);
    cv::Mat destinationPlanePoints = cv::Mat(4, 2, CV_32F);
    cv::Mat homography;
    cv::Rect rect = templateSize;

    cv::Point sourcePoints[4];
    cv::Point targetPoints[4];
    sourcePoints[0] = cv::Point(rect.x, rect.y); // tl
    sourcePoints[1] = cv::Point(rect.x + rect.width, rect.y); // tr
    sourcePoints[2] = cv::Point(rect.x + rect.width, rect.y + rect.height); // br
    sourcePoints[3] = cv::Point(rect.x, rect.y + rect.height); // bl

    cv::Mat targetPoint = cv::Mat(3, 1, CV_32FC1);
    cv::Mat sourcePoint = cv::Mat(3, 1, CV_32FC1);
    sourcePoint.setTo(1);
    targetPoint.setTo(1);
    for (int i = 0; i < 4; ++i)
    {
        sourcePoint.at<float>(0, 0) = sourcePoints[i].x * factor;
        sourcePoint.at<float>(1, 0) = sourcePoints[i].y * factor;
        targetPoint = transform * sourcePoint;
        targetPoint /= targetPoint.at<float>(2, 0);
        targetPoints[i].x = targetPoint.at<float>(0, 0);
        targetPoints[i].y = targetPoint.at<float>(1, 0);

        sourcePlanePoints.at<float>(i, 0) = sourcePoints[i].x;
        sourcePlanePoints.at<float>(i, 1) = sourcePoints[i].y;
        destinationPlanePoints.at<float>(i, 0) = targetPoints[i].x / factor;
        destinationPlanePoints.at<float>(i, 1) = targetPoints[i].y / factor;
    }
    homography = cv::findHomography(sourcePlanePoints, destinationPlanePoints);
    homography.convertTo(homography, CV_32FC1);
    return homography;
}

void LucasKanadeTracker::setTopPyramid(int topPyramid)
{
    this->topPyramid = topPyramid;
}

void LucasKanadeTracker::enableMaskedTemplate(bool enable)
{
    enableTemplateWeighting = enable;
}

void LucasKanadeTracker::enableColor(bool enable)
{
    useColor = enable;
    if (useColor)
    {
        imageFormat8UCX = CV_8UC3;
        imageFormat16SCX = CV_16SC3;
        imageFormat32FCX = CV_32FC3;
    }
    else
    {
        imageFormat8UCX = CV_8UC1;
        imageFormat16SCX = CV_16SC1;
        imageFormat32FCX = CV_32FC1;
    }
}

void LucasKanadeTracker::setTransformFromParameters(cv::Mat &W_3x3, cv::Mat &parameters)
{
    W_3x3.at<float>(0, 0) = (float)(1.0 + parameters.at<float>(0, 0));\
    W_3x3.at<float>(1, 0) = (float)(parameters.at<float>(1, 0));\
    W_3x3.at<float>(2, 0) = (float)(parameters.at<float>(2, 0));\
    W_3x3.at<float>(0, 1) = (float)(parameters.at<float>(3, 0));\
    W_3x3.at<float>(1, 1) = (float)(1.0 + parameters.at<float>(4, 0));\
    W_3x3.at<float>(2, 1) = (float)(parameters.at<float>(5, 0));\
    W_3x3.at<float>(0, 2) = (float)(parameters.at<float>(6, 0));\
    W_3x3.at<float>(1, 2) = (float)(parameters.at<float>(7, 0));\
    W_3x3.at<float>(2, 2) = 1.0;
}

void LucasKanadeTracker::setParametersFromTransform(cv::Mat &parameters, cv::Mat &W_3x3)
{
    parameters.at<float>(0, 0) = (float)(W_3x3.at<float>(0, 0) - 1.0);\
    parameters.at<float>(1, 0) = (float)(W_3x3.at<float>(1, 0));\
    parameters.at<float>(2, 0) = (float)(W_3x3.at<float>(2, 0));\
    parameters.at<float>(3, 0) = (float)(W_3x3.at<float>(0, 1));\
    parameters.at<float>(4, 0) = (float)(W_3x3.at<float>(1, 1) - 1.0);\
    parameters.at<float>(5, 0) = (float)(W_3x3.at<float>(2, 1));\
    parameters.at<float>(6, 0) = (float)(W_3x3.at<float>(0, 2));\
    parameters.at<float>(7, 0) = (float)(W_3x3.at<float>(1, 2));
}

double LucasKanadeTracker::getMagnitude(cv::Mat &columnMatrix)
{
    double magnitude = 0;
    for (int i = 0; i < columnMatrix.rows; ++i)
        magnitude += pow(columnMatrix.at<float>(i, 0), 2);
    return pow(magnitude, 0.5);
}
