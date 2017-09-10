#include "tracking/ESMTracker.h"
#include "common/TrackingCommon.h"
#include <QDebug>
//#include "sys/time.h"
#include "opencv2/gpu/gpu.hpp"

ESMTracker::ESMTracker()
{
    //this->type = MALIS_ESM;
}

void ESMTracker::setTargetObject(cv::Mat &image, cv::Rect selection)
{
    qDebug() << "Object ESM set\n";
    LucasKanadeTracker::setTargetObject(image, selection);

    // (3) Evaluate the gradient \nabla T of the template T(x)
    T_gradient_x_0 = cv::Mat(T.size(), CV_16SC1);
    T_gradient_y_0 = cv::Mat(T.size(), CV_16SC1);
    cv::Sobel(T, T_gradient_x_0, T_gradient_x_0.depth(), 1, 0, 3, 0.125, 0, cv::BORDER_REFLECT_101);
    cv::Sobel(T, T_gradient_y_0, T_gradient_y_0.depth(), 0, 1, 3, 0.125, 0, cv::BORDER_REFLECT_101);

    // (4) Evaluate the Jacobian dW_dp at (x; 0)
    for (int i = 0; i < 8; ++i)
    {
        dw_dp_0[0][i] = cv::Mat(T.size(), CV_32FC1);
        dw_dp_0[1][i] = cv::Mat(T.size(), CV_32FC1);
    }
    cv::Mat parameters_0 = cv::Mat(8, 1, CV_32F);
    parameters_0.setTo(0);
    calculateJacobians(dw_dp_0, parameters_0);

    // For ESM
    J_e = cv::Mat(T.rows * T.cols, 8, CV_32F);
    jacobian_pk = cv::Mat(2, 8, CV_32F);
    I_element = cv::Mat(1, 2, CV_32F);
    r_k = cv::Mat(1, 8, CV_32F);

    // Calculation of J_e
    for (int y = 0; y < T.rows; ++y)
    {
        for (int x = 0; x < T.cols; ++x)
        {
            int q = y * T.cols + x;
            getJacobianElement(dw_dp_0, y, x, jacobian_pk);

            I_element.at<float>(0, 0) = T_gradient_x_0.at<short>(y, x);
            I_element.at<float>(0, 1) = T_gradient_y_0.at<short>(y, x);
            cv::gemm(I_element, jacobian_pk, 1, Mat(), 0, r_k);
            for (int i = 0; i < 8; ++i) {
                J_e.at<float>(q, i) = r_k.at<float>(0, i);
            }
        }
    }
    J_e = J_e.t();

    // Preallocation Mats:
    J_xc = cv::Mat(8, T.rows * T.cols, CV_32F);
    delta_s = cv::Mat(1, T.rows * T.cols, CV_32F);
    delta_x = cv::Mat(8, 1, CV_32F);
    J_sum = cv::Mat(T.rows * T.cols, 8, CV_32F);
    J_sum_inv = cv::Mat(8, T.rows * T.cols, CV_32F);
    J_sum_transpose_times_J_sum = cv::Mat(8, 8, CV_32F);

    //Convert Jacobians to array format for faster multiplication
    for (int i = 0; i < 8; ++i)
    {
        Jacobians_x[i] = dw_dp_0[0][i].reshape(Jacobians_x[i].channels(), 1);
        Jacobians_y[i] = dw_dp_0[1][i].reshape(Jacobians_y[i].channels(), 1);
        J_xc_x[i] = cv::Mat(J_xc_x[i].channels(), 1, CV_32F);
        J_xc_y[i] = cv::Mat(J_xc_y[i].channels(), 1, CV_32F);
    }

    for (int y = 0; y < T.rows; ++y)
    {
        for (int x = 0; x < T.cols; ++x)
        {
            cv::Mat jacobian_i = cv::Mat(2, 8, CV_32F);
            getJacobianElement(dw_dp_0, y, x, jacobian_i);
            jacobians.push_back(jacobian_i);
        }
    }

}

void ESMTracker::trackObject(cv::Mat &newImage)
{
    // (0) Convert current image to Gray Level
    cv::cvtColor(newImage, I, CV_RGB2GRAY);

    // Current dp
    double dp_magnitude = 0.0;
    int iterations = 0;

    cv::Mat algorithmTimes = cv::Mat(1, 10, CV_64F);
    algorithmTimes.setTo(0);
    //timespec timeStart;
    //timespec timeEnd;
    do
    {
        // (1) Warp I with W(x; p) to compute I(W(x; p))
        int currentCell = 0;
        // Mark 1
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);

        SET_TRANSFORM_MAT(W_3x3, parameters);
        cv::warpPerspective(I, I_w, W_3x3, I_w.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // (3a) Warp the gradient \nabla I with W (x; p)
        // Mark 2
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);

        cv::Sobel(I_w, I_w_gradient_x, I_w_gradient_x.depth(), 1, 0, 3, 0.125, 0, cv::BORDER_REFLECT_101);
        cv::Sobel(I_w, I_w_gradient_y, I_w_gradient_y.depth(), 0, 1, 3, 0.125, 0, cv::BORDER_REFLECT_101);
        I_w_gradient_x = I_w_gradient_x.reshape(I_w_gradient_x.channels(), 1);
        I_w_gradient_y = I_w_gradient_y.reshape(I_w_gradient_y.channels(), 1);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Calculation of J_xc
        // Mark 3 ... SLOW
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        for (int i = 0; i < 8; ++i)
        {
            cv::multiply(I_w_gradient_x, Jacobians_x[i], J_xc_x[i], 1, CV_32F);
            cv::multiply(I_w_gradient_y, Jacobians_y[i], J_xc_y[i], 1, CV_32F);
            cv::add(J_xc_x[i], J_xc_y[i], J_xc.row(i));
        }

        cv::subtract(I_w, T, delta_s, cv::Mat(), CV_32F);
        cv::multiply(delta_s, templateWeight[0], delta_s);
        delta_s = delta_s.reshape(delta_s.channels(), 1);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 4
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::add(J_e, J_xc, J_sum);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 5 ... SLOW
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gemm(J_sum, J_sum, 1, Mat(), 0, J_sum_transpose_times_J_sum, GEMM_2_T); // Before: GEMM_1_T
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //   algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 6
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::invert(J_sum_transpose_times_J_sum, J_sum_transpose_times_J_sum, CV_LU);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 7 ... SLOW
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gemm(J_sum_transpose_times_J_sum, J_sum, 1, Mat(), 0, J_sum_inv); // Before: GEMM_2_T
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 8
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gemm(J_sum_inv, delta_s, -2, Mat(), 0, delta_x, GEMM_2_T); // Before: None
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // (9) Update the warp
        // Mark 9
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        updateWarpCompositional(delta_x, parameters);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        GET_MAGNITUDE_8(delta_x, dp_magnitude);
        ++iterations;
        ++currentSubIteration;

        if (TrackingCommon::delayTracker1sInEachIteration)
        {
            //sleep(1);
            TrackingCommon::frame.copyTo(TrackingCommon::currentFrame);
            highlightObject(TrackingCommon::currentFrame);
            TrackingCommon::mainController->displayImageWindow(std::string("Intermedium"), TrackingCommon::currentFrame);
        }
        //TrackingCommon::mainController->displayData("ESM_Times", algorithmTimes, DataDisplayer::APPEND);

    }
    while (dp_magnitude > epsilon  && iterations < maxIterations);

    SET_TRANSFORM_MAT(W_3x3, parameters);
    if (TrackingCommon::showLKParameters)
    {
        TrackingCommon::mainController->displayData("W_3x3", W_3x3, DataDisplayer::FULL);
    }

    ++currentIteration;
}

void ESMTracker::getJacobianElement(cv::Mat X[2][8], int y, int x, cv::Mat &W)
{
    W.at<float>(0, 0) = (float)(X[0][0].at<float>(y, x));
    W.at<float>(0, 1) = (float)(X[0][1].at<float>(y, x));
    W.at<float>(0, 2) = (float)(X[0][2].at<float>(y, x));
    W.at<float>(0, 3) = (float)(X[0][3].at<float>(y, x));
    W.at<float>(0, 4) = (float)(X[0][4].at<float>(y, x));
    W.at<float>(0, 5) = (float)(X[0][5].at<float>(y, x));
    W.at<float>(0, 6) = (float)(X[0][6].at<float>(y, x));
    W.at<float>(0, 7) = (float)(X[0][7].at<float>(y, x));
    W.at<float>(1, 0) = (float)(X[1][0].at<float>(y, x));
    W.at<float>(1, 1) = (float)(X[1][1].at<float>(y, x));
    W.at<float>(1, 2) = (float)(X[1][2].at<float>(y, x));
    W.at<float>(1, 3) = (float)(X[1][3].at<float>(y, x));
    W.at<float>(1, 4) = (float)(X[1][4].at<float>(y, x));
    W.at<float>(1, 5) = (float)(X[1][5].at<float>(y, x));
    W.at<float>(1, 6) = (float)(X[1][6].at<float>(y, x));
    W.at<float>(1, 7) = (float)(X[1][7].at<float>(y, x));
}
