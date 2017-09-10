#include "tracking/ESMTrackerGPU.h"
#include "common/TrackingCommon.h"
#include <QDebug>

ESMTrackerGPU::ESMTrackerGPU()
{
    //this->type = MALIS_ESM;
}

void ESMTrackerGPU::setTargetObject(cv::Mat &image, cv::Rect selection)
{
    qDebug() << "Calling setTarget with GPU";
    ESMTracker::setTargetObject(image, selection);

    totalPixels = T.cols * T.rows;

    for (int i = 0; i < 8; ++i)
    {
        gJacobians_x[i] = cv::gpu::GpuMat(Jacobians_x[i]);
        gJacobians_y[i] = cv::gpu::GpuMat(Jacobians_y[i]);
        gJ_xc_x[i] = cv::gpu::GpuMat(J_xc_x[i]);
        gJ_xc_y[i] = cv::gpu::GpuMat(J_xc_y[i]);
    }

    gNewImage = cv::gpu::GpuMat(image);
    gI = cv::gpu::GpuMat(image);
    gT = cv::gpu::GpuMat(T);
    gI_w = cv::gpu::GpuMat(I_w);
    gParameters = cv::gpu::GpuMat(parameters);
    gI_w_gradient_x = cv::gpu::GpuMat(I_w_gradient_x.size(), CV_32SC1);
    gI_w_gradient_y = cv::gpu::GpuMat(I_w_gradient_y.size(), CV_32SC1);
    gDelta_s = cv::gpu::GpuMat(delta_s);
    gDelta_s_continuos = cv::gpu::createContinuous(delta_s.size(), delta_s.type());
    gDelta_x = cv::gpu::GpuMat(delta_x);
    gI_w_gradient_x_continuos = cv::gpu::createContinuous(T.rows, T.cols, CV_32SC1);
    gI_w_gradient_y_continuos = cv::gpu::createContinuous(T.rows, T.cols, CV_32SC1);

    gJ_xc = cv::gpu::GpuMat(J_xc);
    gJ_sum = cv::gpu::GpuMat(J_sum);
    gJ_e = cv::gpu::GpuMat(J_e);
    gJ_sum_transpose_times_J_sum = cv::gpu::GpuMat(J_sum_transpose_times_J_sum);
    gJ_sum_inv = cv::gpu::GpuMat(J_sum_inv);
}

void ESMTrackerGPU::trackObject(cv::Mat &newImage)
{
    qDebug() << "Tracking with GPU";
    // (0) Convert current image to Gray Level
    gNewImage.upload(newImage);
    cv::gpu::cvtColor(gNewImage, gI, CV_RGB2GRAY);

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
        cv::gpu::warpPerspective(gI, gI_w, W_3x3, gI_w.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // (3a) Warp the gradient \nabla I with W (x; p)
        // Mark 2
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);

        cv::gpu::Sobel(gI_w, gI_w_gradient_x, gI_w_gradient_x.depth(), 1, 0, 3, 0.125, cv::BORDER_REFLECT_101);//0.125
        cv::gpu::Sobel(gI_w, gI_w_gradient_y, gI_w_gradient_y.depth(), 0, 1, 3, 0.125, cv::BORDER_REFLECT_101);

        gI_w_gradient_x_continuos = cv::gpu::createContinuous(gI_w_gradient_x.size(), gI_w_gradient_x.type());
        gI_w_gradient_y_continuos = cv::gpu::createContinuous(gI_w_gradient_x.size(), gI_w_gradient_x.type());
        gI_w_gradient_x.copyTo(gI_w_gradient_x_continuos);
        gI_w_gradient_y.copyTo(gI_w_gradient_y_continuos);
        gI_w_gradient_x_continuos = gI_w_gradient_x_continuos.reshape(gI_w_gradient_x_continuos.channels(), 1);
        gI_w_gradient_y_continuos = gI_w_gradient_y_continuos.reshape(gI_w_gradient_y_continuos.channels(), 1);

        gI_w_gradient_x_continuos.convertTo(gI_w_gradient_x_float, CV_32FC1);
        gI_w_gradient_y_continuos.convertTo(gI_w_gradient_y_float, CV_32FC1);

        gI_w_gradient_x_continuos.reshape(gI_w_gradient_x_continuos.channels(), 1);
        gI_w_gradient_y_continuos.reshape(gI_w_gradient_y_continuos.channels(), 1);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Calculation of J_xc
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gpu::GpuMat gJ_xc_row;
        for (int i = 0; i < 8; ++i)
        {
            cv::gpu::multiply(gI_w_gradient_x_float, gJacobians_x[i], gJ_xc_x[i], 1, CV_32FC1);
            cv::gpu::multiply(gI_w_gradient_y_float, gJacobians_y[i], gJ_xc_y[i], 1, CV_32FC1);
            gJ_xc_row = gJ_xc.row(i);
            cv::gpu::add(gJ_xc_x[i], gJ_xc_y[i], gJ_xc_row);
        }

        cv::gpu::subtract(gI_w, gT, gDelta_s, cv::gpu::GpuMat(), CV_32F);
        gDelta_s_continuos = cv::gpu::createContinuous(gDelta_s.size(), gDelta_s.type());
        gDelta_s.copyTo(gDelta_s_continuos);
        gDelta_s_continuos = gDelta_s_continuos.reshape(gDelta_s_continuos.channels(), 1);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 4
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gpu::add(gJ_e, gJ_xc, gJ_sum);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 5 ... SLOW
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gpu::gemm(gJ_sum, gJ_sum, 1, cv::gpu::GpuMat(), 0, gJ_sum_transpose_times_J_sum, GEMM_2_T); // Before: GEMM_1_T
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 6
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        gJ_sum_transpose_times_J_sum.download(J_sum_transpose_times_J_sum);
        cv::invert(J_sum_transpose_times_J_sum, J_sum_transpose_times_J_sum, CV_LU);
        gJ_sum_transpose_times_J_sum.upload(J_sum_transpose_times_J_sum);
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 7 ... SLOW
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gpu::gemm(gJ_sum_transpose_times_J_sum, gJ_sum, 1, cv::gpu::GpuMat(), 0, gJ_sum_inv); // Before: GEMM_2_T
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // Mark 8
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);
        cv::gpu::gemm(gJ_sum_inv, gDelta_s_continuos, -2, cv::gpu::GpuMat(), 0, gDelta_x, GEMM_2_T); // Before: None
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        // (9) Update the warp
        // Mark 9
        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeStart);

        gDelta_x.download(delta_x);
        updateWarpCompositional(delta_x, parameters);

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timeEnd);
        //if (timeEnd.tv_nsec - timeStart.tv_nsec > 0)
        //    algorithmTimes.at<double>(0, currentCell++) = (timeEnd.tv_nsec - timeStart.tv_nsec) / 1000.0;

        GET_MAGNITUDE_8(delta_x, dp_magnitude);
        ++iterations;
        ++currentSubIteration;

        if (TrackingCommon::showLKParameters)
        {
            //TrackingCommon::mainController->displayData("ESM_Times", algorithmTimes, DataDisplayer::APPEND);
            //TrackingCommon::mainController->displayData("ESM_W", delta_x, DataDisplayer::FULL);
            //TrackingCommon::mainController->displayData("Parameters", parameters, DataDisplayer::FULL);
        }

    }
    while (dp_magnitude > epsilon  && iterations < maxIterations);

    // Flag to indicate whether the algorithm converged before iterations were over
    converged = iterations < maxIterations && dp_magnitude <= epsilon;

    lastError = dp_magnitude;

    SET_TRANSFORM_MAT(W_3x3, parameters);
    if (TrackingCommon::showLKParameters)
    {
        TrackingCommon::mainController->displayData("W_3x3", W_3x3, DataDisplayer::FULL);
    }

    ++currentIteration;
}

