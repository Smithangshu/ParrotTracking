#ifndef ESMTRACKERGPU_H
#define ESMTRACKERGPU_H

#include "ESMTracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <vector>

class ESMTrackerGPU : public ESMTracker
{
public:
    int totalPixels;
    cv::gpu::GpuMat gNewImage;
    cv::gpu::GpuMat gI;
    cv::gpu::GpuMat gT;
    cv::gpu::GpuMat gI_w;
    cv::gpu::GpuMat gParameters;
    cv::gpu::GpuMat gI_w_gradient_x;
    cv::gpu::GpuMat gI_w_gradient_y;
    cv::gpu::GpuMat gI_w_gradient_x_continuos;
    cv::gpu::GpuMat gI_w_gradient_y_continuos;
    cv::gpu::GpuMat gI_w_gradient_x_float;
    cv::gpu::GpuMat gI_w_gradient_y_float;

    cv::gpu::GpuMat gJ_e;
    cv::gpu::GpuMat gJ_xc;
    cv::gpu::GpuMat gDelta_s;
    cv::gpu::GpuMat gDelta_s_continuos;
    cv::gpu::GpuMat gDelta_x;
    cv::gpu::GpuMat gJ_sum;
    cv::gpu::GpuMat gJ_sum_inv;
    cv::gpu::GpuMat gJ_sum_transpose_times_J_sum;

    cv::gpu::GpuMat gJ_xc_x[8];
    cv::gpu::GpuMat gJ_xc_y[8];
    cv::gpu::GpuMat gJacobians_x[8];
    cv::gpu::GpuMat gJacobians_y[8];

    ESMTrackerGPU();
    void setTargetObject(cv::Mat &image, cv::Rect selection);
    void trackObject(cv::Mat &newImage);
};

#endif // ESMTRACKERGPU_H
