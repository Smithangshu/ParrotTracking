#include "tracking/ESMTrackerPyramidal.h"
#include "common/TrackingCommon.h"
#include <QDebug>
//#include "sys/time.h"
#include "opencv2/gpu/gpu.hpp"

ESMTrackerPyramidal::ESMTrackerPyramidal()
{
    qDebug() << "Constructor ESM Pyr";
    //this->type = MALIS_ESM;
}

void ESMTrackerPyramidal::setTargetObject(cv::Mat &image, cv::Rect selection)
{
    qDebug() << "Object ESM Pyr set";
    ESMTracker::setTargetObject(image, selection);

    if (useColor)
    {
        T = targetObject.clone();
        setupPyramids();
    }

    cv::Mat parameters_0 = cv::Mat(8, 1, CV_32F);
    parameters_0.setTo(0);
    for (int i = 0; i < pyramids; ++i)
    {
        int totalPixels = templates[i].rows * templates[i].cols;
        cv::equalizeHist(templates[i], templates[i]);

        // (3) Evaluate the gradient \nabla T of the template T(x)
        templateGradientX[i] = cv::Mat(templates[i].size(), imageFormat16SCX);
        templateGradientY[i] = cv::Mat(templates[i].size(), imageFormat16SCX);
        cv::Sobel(templates[i], templateGradientX[i], templateGradientX[i].depth(), 1, 0, 3, 0.125, 0, cv::BORDER_REFLECT_101);
        cv::Sobel(templates[i], templateGradientY[i], templateGradientY[i].depth(), 0, 1, 3, 0.125, 0, cv::BORDER_REFLECT_101);

        // (4) Evaluate the Jacobian dW_dp at (x; 0)
        for (int j = 0; j < 8; ++j)
        {
            dWdP[i][0][j] = cv::Mat(templates[i].size(), CV_32FC1);
            dWdP[i][1][j] = cv::Mat(templates[i].size(), CV_32FC1);
        }
        calculateJacobians(dWdP[i], parameters_0);

        Je[i] = cv::Mat(totalPixels, 8, imageFormat32FCX);
        jacobian_pk = cv::Mat(2, 8, CV_32F);
        I_element = cv::Mat(1, 2, CV_32F);
        r_k = cv::Mat(1, 8, CV_32F);

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
                        for (int j = 0; j < 8; ++j) {
                            Je[i].at<Vec3f>(q, j)[channel] = r_k.at<float>(0, j);
                        }
                    }
                }
                else
                {
                    I_element.at<float>(0, 0) = templateGradientX[i].at<short>(y, x);
                    I_element.at<float>(0, 1) = templateGradientY[i].at<short>(y, x);
                    cv::gemm(I_element, jacobian_pk, 1, Mat(), 0, r_k);
                    for (int j = 0; j < 8; ++j) {
                        Je[i].at<float>(q, j) = r_k.at<float>(0, j);
                    }
                }
            }
        }
        Je[i] = Je[i].t();

        // Preallocation Mats:
        Jc[i] = cv::Mat(8, totalPixels, imageFormat32FCX);
        delta_s[i] = cv::Mat(1, totalPixels, imageFormat32FCX);
        delta_x[i] = cv::Mat(8, 1, CV_32F);
        J_sum[i] = cv::Mat(totalPixels, 8, CV_32F);
        J_sum_inv[i] = cv::Mat(8, totalPixels, CV_32F);
        J_sum_transpose_times_J_sum[i] = cv::Mat(8, 8, CV_32F);

        //Convert Jacobians to array format for faster multiplication
        for (int j = 0; j < 8; ++j)
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

    // Apply gaussian blur
    cv::GaussianBlur(templates[2], templates[2], cv::Size(9, 9), 0);
    cv::GaussianBlur(templates[1], templates[1], cv::Size(9, 9), 0);
    //cv::GaussianBlur(templates[2], templates[2], cv::Size(7, 7), 0);
    //cv::GaussianBlur(templates[1], templates[1], cv::Size(5, 5), 0);
}

void ESMTrackerPyramidal::trackObject(cv::Mat &newImage)
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

    // Apply gaussian blur
    cv::GaussianBlur(imagePyramids[2], imagePyramids[2], cv::Size(9, 9), 0);
    cv::GaussianBlur(imagePyramids[1], imagePyramids[1], cv::Size(9, 9), 0);
    //cv::GaussianBlur(imagePyramids[2], imagePyramids[2], cv::Size(7, 7), 0);
    //cv::GaussianBlur(imagePyramids[1], imagePyramids[1], cv::Size(5, 5), 0);

    //qDebug() << "Pyramids: " << topPyramid;
    // Adaptively choose top pyramid
    int registrationTopPyramid = topPyramid;
    //if (lastError < 0.025)
    //    registrationTopPyramid = 0;
    //else if (lastError < 0.05)
    //    registrationTopPyramid = 1;

    // Start by scaling parameters to fit higher pyramind
    if (registrationTopPyramid  > 0)
    {
        SET_TRANSFORM_MAT(W_3x3, parameters);
        W_3x3 = changeTransformScale(W_3x3, cv::Rect(0, 0, templates[registrationTopPyramid].cols, templates[registrationTopPyramid].rows), 1.0 * (1 << registrationTopPyramid));
        SET_TRANSFORM_VECTOR_8(parameters, W_3x3);
    }

    currentSubIteration = 0;
    for (int i = registrationTopPyramid; i >= 0; --i)
    {
        //qDebug() << "Pyramid " << i;
        // Current dp
        float scale = 1 << i;
        double dp_magnitude = 0.0;
        int iterations = 0;

        //timespec timeGlobalStart;
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeGlobalStart);

        cv::Mat algorithmTimes = cv::Mat(1, 11, CV_64F);
        algorithmTimes.setTo(0);
        //timespec timeStart;
        //timespec timeEnd;

        if (i != registrationTopPyramid)
        {
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            SET_TRANSFORM_MAT(W_3x3, parameters);
            W_3x3 = changeTransformScale(W_3x3, cv::Rect(0, 0, templates[i].cols, templates[i].rows), 0.5);
            SET_TRANSFORM_VECTOR_8(parameters, W_3x3);
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);

            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, 10) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;
        }

        // Counter of the consecutive diverging iterations: Stop if more than
        // 4 iterations in a row produce divergence
        int iterationsDiverging = 0;
        double preError = 0;

        do
        {
            // (1) Warp I with W(x; p) to compute I(W(x; p))
            int currentCell = 0;
            // Mark 1
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);

            SET_TRANSFORM_MAT(W_3x3, parameters);
            cv::warpPerspective(imagePyramids[i], warpedImages[i], W_3x3, warpedImages[i].size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);
            cv::equalizeHist(warpedImages[i], warpedImages[i]);
            if (TrackingCommon::showLKIntermediumImages)
            {
                TrackingCommon::mainController->displayImageWindowCV(std::string("WarpedImage"), warpedImages[i]);
                //TrackingCommon::mainController->displayImageWindowCV(std::string("imagePyramids"), imagePyramids[i]);
                TrackingCommon::mainController->displayImageWindowCV(std::string("Template"), templates[i]);
                //TrackingCommon::mainController->displayImageWindowCV(std::string("WarpedImage"), warpedImages[i]);
                //imagePyramids[i].copyTo(imagePyramidsDisplay[i]);
                //drawWarpedRect(imagePyramidsDisplay[i], W_3x3, trackingWindow.width / scale, trackingWindow.height / scale);
                //TrackingCommon::mainController->displayImageWindowCV(std::string("Pyr1"), imagePyramidsDisplay[i]);
            }

            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //   algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // (3a) Warp the gradient \nabla I with W (x; p)
            // Mark 2
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);

            cv::Sobel(warpedImages[i], warpedImagesGradientX[i], warpedImagesGradientX[i].depth(), 1, 0, 3, 0.125, 0, cv::BORDER_REFLECT_101);
            cv::Sobel(warpedImages[i], warpedImagesGradientY[i], warpedImagesGradientY[i].depth(), 0, 1, 3, 0.125, 0, cv::BORDER_REFLECT_101);
            warpedImagesGradientX[i] = warpedImagesGradientX[i].reshape(warpedImagesGradientX[i].channels(), 1);
            warpedImagesGradientY[i] = warpedImagesGradientY[i].reshape(warpedImagesGradientY[i].channels(), 1);

            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // Calculation of J_xc
            // Mark 3 ... SLOW
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            for (int j = 0; j < 8; ++j)
            {
                cv::multiply(warpedImagesGradientX[i], JacobiansX[i][j], J_xc_x[i][j], 1, imageFormat32FCX);
                cv::multiply(warpedImagesGradientY[i], JacobiansY[i][j], J_xc_y[i][j], 1, imageFormat32FCX);
                cv::add(J_xc_x[i][j], J_xc_y[i][j], Jc[i].row(j));
            }

            cv::subtract(warpedImages[i], templates[i], delta_s[i], cv::Mat(), imageFormat32FCX);
            double currentError = cv::sum(delta_s[i])[0];

            lastAverageError = std::abs(currentError / (templates[i].cols * templates[i].rows));
            //qDebug() << "AVG Error: " << lastAverageError;
            if (lastAverageError > 1.15 && currentIteration > 8) {
                iterations = maxIterations;
                //qDebug() << "Detected divergence";
                break;
            }

            if (useColor)
            {
                // Combine Channels for delta_s
                cv::cvtColor(delta_s[i], delta_s[i], CV_BGR2GRAY);
            }

            delta_s[i] = delta_s[i].reshape(delta_s[i].channels(), 1);

            if (enableTemplateWeighting)
                cv::multiply(delta_s[i], templateWeight[i], delta_s[i]);

            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // Mark 4
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            cv::add(Je[i], Jc[i], J_sum[i]);
            if (useColor)
            {
                // Combine Channels for Jacobian Sum
                cv::cvtColor(J_sum[i], J_sum[i], CV_BGR2GRAY);
            }
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // Mark 5 ... SLOW
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            cv::gemm(J_sum[i], J_sum[i], 1, Mat(), 0, J_sum_transpose_times_J_sum[i], GEMM_2_T); // Before: GEMM_1_T
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // Mark 6
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            cv::invert(J_sum_transpose_times_J_sum[i], J_sum_transpose_times_J_sum[i], CV_LU);
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // Mark 7 ... SLOW
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            cv::gemm(J_sum_transpose_times_J_sum[i], J_sum[i], 1, Mat(), 0, J_sum_inv[i]); // Before: GEMM_2_T
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // Mark 8
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            cv::gemm(J_sum_inv[i], delta_s[i], -2, Mat(), 0, delta_x[i], GEMM_2_T); // Before: None
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // (9) Update the warp
            // Mark 9
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
            updateWarpCompositional(delta_x[i], parameters);
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
            //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
            //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

            // (10) Validation

            GET_MAGNITUDE_8(delta_x[i], dp_magnitude);
            if (dp_magnitude > lastError && iterations > 0)
            {
                // Show deverging message
                //qDebug() << iterations << "," << currentSubIteration << " Diverging!!!!!!";
                //break;
            }
            lastError = dp_magnitude;

            ++iterations;
            ++currentSubIteration;

            if (TrackingCommon::showLKParameters)
            {
                TrackingCommon::mainController->displayData("Delta_x", delta_x[i], DataDisplayer::APPEND);
                TrackingCommon::mainController->displayData("Parameters", parameters, DataDisplayer::APPEND);
            }

        }
        while (dp_magnitude > epsilon && iterations < maxIterations);

        // Flag to indicate whether the algorithm converged before iterations were over
        converged = iterations < maxIterations && dp_magnitude <= epsilon;

        lastError = dp_magnitude;
        //lastTime = (timeEnd.tv_nsec - timeGlobalStart.tv_nsec) / 1000000.0;

        //TrackingCommon::mainController->displayData("ESM_Times", algorithmTimes, DataDisplayer::APPEND);
        if (TrackingCommon::delayTracker1sInEachIteration)
        {
            #ifndef WIN32
            sleep(1);
            #endif
        }
    }

    SET_TRANSFORM_MAT(W_3x3, parameters);
    if (TrackingCommon::showLKParameters)
    {
        TrackingCommon::mainController->displayData("W_3x3", W_3x3, DataDisplayer::FULL);
    }

    ++currentIteration;
}
