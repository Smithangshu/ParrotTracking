#include "tracking/ESMTrackerConstrained.h"
#include "common/TrackingCommon.h"
#include <QDebug>
#include "opencv2/gpu/gpu.hpp"

ESMTrackerConstrained::ESMTrackerConstrained()
{
    qDebug() << "Constructor ESM Constrained";
    //this->type = MALIS_ESM;
}

void ESMTrackerConstrained::setTargetObject(cv::Mat &image, cv::Rect selection)
{
    qDebug() << "Object ESM Constrained set";

    /**** HERE STARTS LucasKanadeTracker::setTargetObject(image, selection); ****/
    Tracker::setTargetObject(image, selection);

    parameters = cv::Mat(totalParameters, 1, CV_32F);
    parameters.setTo(0);
    parameters.at<float>(4, 0) = selection.x;
    parameters.at<float>(5, 0) = selection.y;

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

    H_ = cv::Mat(totalParameters, totalParameters, CV_32F);
    H_inv = cv::Mat(totalParameters, totalParameters, CV_32F);
    dp_right_operand = cv::Mat(totalParameters, 1, CV_32F);
    dp = cv::Mat(totalParameters, 1, CV_32F);
    for (int i = 0; i < totalParameters; ++i)
        steepest_descend_images[i] = cv::Mat(T.size(), CV_32FC1);
    for (int i = 0; i < totalParameters; ++i)
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
            }
        }
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

    /**** HERE ENDS LucasKanadeTracker::setTargetObject(image, selection); ****/

    if (useColor)
    {
        T = targetObject.clone();
        setupPyramids();
    }

    cv::Mat parameters_0 = cv::Mat(totalParameters, 1, CV_32F);
    parameters_0.setTo(0);
    for (int i = 0; i < pyramids; ++i)
    {
        int totalPixels = templates[i].rows * templates[i].cols;

        // (3) Evaluate the gradient \nabla T of the template T(x)
        templateGradientX[i] = cv::Mat(templates[i].size(), imageFormat16SCX);
        templateGradientY[i] = cv::Mat(templates[i].size(), imageFormat16SCX);
        cv::Sobel(templates[i], templateGradientX[i], templateGradientX[i].depth(), 1, 0, 3, 0.125, 0, cv::BORDER_REFLECT_101);
        cv::Sobel(templates[i], templateGradientY[i], templateGradientY[i].depth(), 0, 1, 3, 0.125, 0, cv::BORDER_REFLECT_101);

        // (4) Evaluate the Jacobian dW_dp at (x; 0)
        for (int j = 0; j < totalParameters; ++j)
        {
            dWdP[i][0][j] = cv::Mat(templates[i].size(), CV_32FC1);
            dWdP[i][1][j] = cv::Mat(templates[i].size(), CV_32FC1);
        }
        calculateJacobians(dWdP[i], parameters_0);

        Je[i] = cv::Mat(totalPixels, totalParameters, imageFormat32FCX);
        jacobian_pk = cv::Mat(2, totalParameters, CV_32F);
        I_element = cv::Mat(1, 2, CV_32F);
        r_k = cv::Mat(1, totalParameters, CV_32F);

        // Calculation of J_e
        for (int y = 0; y < templates[i].rows; ++y)
        {
            for (int x = 0; x < templates[i].cols; ++x)
            {
                int q = y * templates[i].cols + x;
                getJacobianElement(dWdP[i], y, x, jacobian_pk);

                if (useColor)
                {
                    for (int channel = 0; channel < T.channels(); ++channel)
                    {
                        I_element.at<float>(0, 0) = templateGradientX[i].at<Vec3s>(y, x)[channel];
                        I_element.at<float>(0, 1) = templateGradientY[i].at<Vec3s>(y, x)[channel];
                        cv::gemm(I_element, jacobian_pk, 1, Mat(), 0, r_k);
                        for (int j = 0; j < totalParameters; ++j) {
                            Je[i].at<Vec3f>(q, j)[channel] = r_k.at<float>(0, j);
                        }
                    }
                }
                else
                {
                    I_element.at<float>(0, 0) = templateGradientX[i].at<short>(y, x);
                    I_element.at<float>(0, 1) = templateGradientY[i].at<short>(y, x);
                    cv::gemm(I_element, jacobian_pk, 1, Mat(), 0, r_k);
                    for (int j = 0; j < totalParameters; ++j) {
                        Je[i].at<float>(q, j) = r_k.at<float>(0, j);
                    }
                }
            }
        }
        Je[i] = Je[i].t();

        // Preallocation Mats:
        Jc[i] = cv::Mat(totalParameters, totalPixels, imageFormat32FCX);
        delta_s[i] = cv::Mat(1, totalPixels, imageFormat32FCX);
        delta_x[i] = cv::Mat(totalParameters, 1, CV_32F);
        J_sum[i] = cv::Mat(totalPixels, totalParameters, CV_32F);
        J_sum_inv[i] = cv::Mat(totalParameters, totalPixels, CV_32F);
        J_sum_transpose_times_J_sum[i] = cv::Mat(totalParameters, totalParameters, CV_32F);

        //Convert Jacobians to array format for faster multiplication
        for (int j = 0; j < totalParameters; ++j)
        {
            if (useColor)
            {
                std::vector<cv::Mat> JacobiansXVector;
                JacobiansXVector.push_back(dWdP[i][0][j]);
                JacobiansXVector.push_back(dWdP[i][0][j]);
                JacobiansXVector.push_back(dWdP[i][0][j]);
                cv::Mat JacobiansXMerged = cv::Mat(dWdP[i][0][j].size(), CV_32FC3);;
                cv::merge(JacobiansXVector, JacobiansXMerged);

                std::vector<cv::Mat> JacobiansYVector;
                JacobiansYVector.push_back(dWdP[i][1][j]);
                JacobiansYVector.push_back(dWdP[i][1][j]);
                JacobiansYVector.push_back(dWdP[i][1][j]);
                cv::Mat JacobiansYMerged = cv::Mat(dWdP[i][0][j].size(), CV_32FC3);
                cv::merge(JacobiansYVector, JacobiansYMerged);

                JacobiansX[i][j] = JacobiansXMerged.reshape(0, 1);
                JacobiansY[i][j] = JacobiansYMerged.reshape(0, 1);
            }
            else
            {
                JacobiansX[i][j] = dWdP[i][0][j].reshape(JacobiansX[i][j].channels(), 1);
                JacobiansY[i][j] = dWdP[i][1][j].reshape(JacobiansY[i][j].channels(), 1);
            }
        }

        // Reshape Weighting Mask
        templateWeight[i] = templateWeight[i].reshape(templateWeight[i].channels(), 1);
    }
}

void ESMTrackerConstrained::trackObject(cv::Mat &newImage)
{
    if (useColor)
        newImage.copyTo(I);
    else
        cv::cvtColor(newImage, I, CV_RGB2GRAY);

    // Fill image pyramids
    imagePyramids[0] = I;
    cv::pyrDown(imagePyramids[0], imagePyramids[1]);
    imagePyramids[1].copyTo(imagePyramidsDisplay[1]);
    cv::pyrDown(imagePyramids[1], imagePyramids[2]);
    imagePyramids[2].copyTo(imagePyramidsDisplay[2]);

    // Start by scaling parameters to fit higher pyramind
    if (topPyramid > 0)
    {
        setTransformFromParameters(W_3x3, parameters);
        W_3x3 = changeTransformScale(W_3x3, cv::Rect(0, 0, templates[topPyramid].cols, templates[topPyramid].rows), 1.0 * (1 << topPyramid));
        setParametersFromTransform(parameters, W_3x3);
    }
    TrackingCommon::mainController->displayData("Parameters", parameters, DataDisplayer::APPEND);
    currentSubIteration = 0;
    for (int i = topPyramid; i >= 0; --i)
    {
        // Current dp
        double dp_magnitude = 0.0;
        int iterations = 0;

        if (i != topPyramid)
        {
            setTransformFromParameters(W_3x3, parameters);
            W_3x3 = changeTransformScale(W_3x3, cv::Rect(0, 0, templates[i].cols, templates[i].rows), 0.5);
            setParametersFromTransform(parameters, W_3x3);
        }

        do
        {
            setTransformFromParameters(W_3x3, parameters);
            cv::warpPerspective(imagePyramids[i], warpedImages[i], W_3x3, warpedImages[i].size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);
            if (TrackingCommon::showLKIntermediumImages)
            {
                TrackingCommon::mainController->displayImageWindow(std::string("WarpedImage"), warpedImages[i]);
                cv::imshow(std::string("Template"), templates[i]);
                cv::imshow(std::string("WarpedImage"), warpedImages[i]);
            }

            cv::Sobel(warpedImages[i], warpedImagesGradientX[i], warpedImagesGradientX[i].depth(), 1, 0, 3, 0.125, 0, cv::BORDER_REFLECT_101);
            cv::Sobel(warpedImages[i], warpedImagesGradientY[i], warpedImagesGradientY[i].depth(), 0, 1, 3, 0.125, 0, cv::BORDER_REFLECT_101);
            warpedImagesGradientX[i] = warpedImagesGradientX[i].reshape(warpedImagesGradientX[i].channels(), 1);
            warpedImagesGradientY[i] = warpedImagesGradientY[i].reshape(warpedImagesGradientY[i].channels(), 1);

            for (int j = 0; j < totalParameters; ++j)
            {
                cv::multiply(warpedImagesGradientX[i], JacobiansX[i][j], J_xc_x[i][j], 1, imageFormat32FCX);
                cv::multiply(warpedImagesGradientY[i], JacobiansY[i][j], J_xc_y[i][j], 1, imageFormat32FCX);
                cv::add(J_xc_x[i][j], J_xc_y[i][j], Jc[i].row(j));
            }

            cv::subtract(warpedImages[i], templates[i], delta_s[i], cv::Mat(), imageFormat32FCX);

            if (useColor)
            {
                // Combine Channels for delta_s
                cv::cvtColor(delta_s[i], delta_s[i], CV_BGR2GRAY);
            }

            delta_s[i] = delta_s[i].reshape(delta_s[i].channels(), 1);

            if (enableTemplateWeighting)
                cv::multiply(delta_s[i], templateWeight[i], delta_s[i]);

            cv::add(Je[i], Jc[i], J_sum[i]);
            if (useColor)
            {
                // Combine Channels for Jacobian Sum
                cv::cvtColor(J_sum[i], J_sum[i], CV_BGR2GRAY);
            }
            cv::gemm(J_sum[i], J_sum[i], 1, Mat(), 0, J_sum_transpose_times_J_sum[i], GEMM_2_T); // Before: GEMM_1_T
            cv::invert(J_sum_transpose_times_J_sum[i], J_sum_transpose_times_J_sum[i], CV_LU);
            cv::gemm(J_sum_transpose_times_J_sum[i], J_sum[i], 1, Mat(), 0, J_sum_inv[i]); // Before: GEMM_2_T
            cv::gemm(J_sum_inv[i], delta_s[i], -2, Mat(), 0, delta_x[i], GEMM_2_T); // Before: None
            if (TrackingCommon::showLKParameters)
            {
                TrackingCommon::mainController->displayData("ESM_W", parameters, DataDisplayer::APPEND);
            }
            updateWarpCompositional(delta_x[i], parameters);
            dp_magnitude = getMagnitude(delta_x[i]);
            ++iterations;
            ++currentSubIteration;

            if (TrackingCommon::delayTracker1sInEachIteration)
            {
                #ifndef WIN32
                sleep(1);
                #endif
            }
            if (TrackingCommon::showLKParameters)
            {
                TrackingCommon::mainController->displayData("ESM_Delta", delta_x[i], DataDisplayer::APPEND);
            }

        }
        while (dp_magnitude > epsilon  && iterations < maxIterations);

        lastError = dp_magnitude;
        //lastTime = (timeEnd.tv_nsec - timeGlobalStart.tv_nsec) / 1000000.0;

    }

    setTransformFromParameters(W_3x3, parameters);

    ++currentIteration;
}

void ESMTrackerConstrained::getJacobianElement(cv::Mat X[2][totalParameters], int y, int x, cv::Mat &W)
{
    qDebug("Getting jacobian element");
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < totalParameters; ++j)
        {
            W.at<float>(i, j) = (float)(X[i][j].at<float>(y, x));
        }
    }
}

void ESMTrackerConstrained::calculateJacobians(cv::Mat dw_dp[2][totalParameters], cv::Mat &parameters)
{
    float p1, p2, p3, p4, p5, p6;
    p1 = parameters.at<float>(0, 0);
    p2 = parameters.at<float>(1, 0);
    p3 = parameters.at<float>(2, 0);
    p4 = parameters.at<float>(3, 0);
    p5 = parameters.at<float>(4, 0);
    p6 = parameters.at<float>(5, 0);

    for (int row = 0; row < dw_dp[0][0].rows; ++row)
    {
        for (int column = 0; column < dw_dp[0][0].cols; ++column)
        {
            float y = row;
            float x = column;
            float g = 1 + x * (1 - pow(1 + p1, 2) - pow(p2, 2)) + y * (1 - pow(p3, 2) - pow(1 + p4, 2));
            float g2 = g * g;
            float fx = (1 + p1) * x + p3 * y + p5;
            float fy = p2 * x + (1 + p4) * y + p6;

            dw_dp[0][0].at<float>(row, column) = (x * (g + 2 * fx * (1 + p1))) / g2;
            dw_dp[0][1].at<float>(row, column) = (2 * x * fx * p2) / g2;
            dw_dp[0][2].at<float>(row, column) = (y * (g + 2 * fx * p3)) / g2;
            dw_dp[0][3].at<float>(row, column) = (2 * y * fx * (1 + p4)) / g2;
            dw_dp[0][4].at<float>(row, column) = 1.0 / g;
            dw_dp[0][5].at<float>(row, column) = 0;

            dw_dp[1][0].at<float>(row, column) = (2 * x * fy * (1 + p1)) / g2;
            dw_dp[1][1].at<float>(row, column) = (x * (g + 2 * fy * p2)) / g2;
            dw_dp[1][2].at<float>(row, column) = (2 * y * fy * p3) / g2;
            dw_dp[1][3].at<float>(row, column) = (y * (g + 2 * fy * (1 + p4))) / g2;
            dw_dp[1][4].at<float>(row, column) = 0;
            dw_dp[1][5].at<float>(row, column) = 1.0 / g;
        }
    }
}

void ESMTrackerConstrained::setTransformFromParameters(cv::Mat &W_3x3, cv::Mat &parameters)
{
    float p1, p2, p3, p4, p5, p6;
    p1 = parameters.at<float>(0, 0);
    p2 = parameters.at<float>(1, 0);
    p3 = parameters.at<float>(2, 0);
    p4 = parameters.at<float>(3, 0);
    p5 = parameters.at<float>(4, 0);
    p6 = parameters.at<float>(5, 0);

    W_3x3.at<float>(0, 0) = 1 + p1;
    W_3x3.at<float>(1, 0) = p2;
    W_3x3.at<float>(2, 0) = 1 - pow(1 + p1, 2) - pow(p2, 2);
    W_3x3.at<float>(0, 1) = p3;
    W_3x3.at<float>(1, 1) = 1 + p4;
    W_3x3.at<float>(2, 1) = 1 - pow(p3, 2) - pow(1 + p4, 2);
    W_3x3.at<float>(0, 2) = p5;
    W_3x3.at<float>(1, 2) = p6;
    W_3x3.at<float>(2, 2) = 1.0;
}

void ESMTrackerConstrained::setParametersFromTransform(cv::Mat &parameters, cv::Mat &W_3x3)
{
    parameters.at<float>(0, 0) = (float)(W_3x3.at<float>(0, 0) - 1.0);
    parameters.at<float>(1, 0) = (float)(W_3x3.at<float>(1, 0));
    parameters.at<float>(2, 0) = (float)(W_3x3.at<float>(0, 1));
    parameters.at<float>(3, 0) = (float)(W_3x3.at<float>(1, 1) - 1.0);
    parameters.at<float>(4, 0) = (float)(W_3x3.at<float>(0, 2));
    parameters.at<float>(5, 0) = (float)(W_3x3.at<float>(1, 2));
}

void ESMTrackerConstrained::updateWarpCompositional(cv::Mat &dp, cv::Mat &W_dest) {
    cv::Mat delta_M = cv::Mat(3, 3, CV_32F);
    setTransformFromParameters(delta_M, dp);

    cv::Mat W = cv::Mat(3, 3, CV_32F);
    setTransformFromParameters(W, W_dest);

    cv::Mat comp_M = cv::Mat(3, 3, CV_32F);
    cv:gemm(W, delta_M, 1.0, 0, 0, comp_M);

    comp_M /= comp_M.at<float>(2, 2);

    setParametersFromTransform(W_dest, comp_M);
}
