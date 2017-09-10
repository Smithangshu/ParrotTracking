#ifndef ESMTRACKERPYRAMIDAL_H
#define ESMTRACKERPYRAMIDAL_H

#include "ESMTracker.h"
#include "LucasKanadeTracker.h"
#include "opencv2/core/core.hpp"

using namespace cv;

class ESMTrackerPyramidal : public ESMTracker
{
    public:
        cv::Mat templateGradientX[pyramids];
        cv::Mat templateGradientY[pyramids];
        cv::Mat dWdP[pyramids][2][8];
        cv::Mat Je[pyramids];
        cv::Mat Jc[pyramids];
        cv::Mat delta_s[pyramids];
        cv::Mat delta_x[pyramids];
        cv::Mat J_sum[pyramids];
        cv::Mat J_sum_inv[pyramids];
        cv::Mat J_sum_transpose_times_J_sum[pyramids];
        cv::Mat JacobiansX[pyramids][8];
        cv::Mat JacobiansY[pyramids][8];
        cv::Mat J_xc[pyramids];
        cv::Mat J_xc_x[pyramids][8];
        cv::Mat J_xc_y[pyramids][8];

        ESMTrackerPyramidal();
        void setTargetObject(cv::Mat &image, cv::Rect selection);
        void trackObject(cv::Mat &newImage);
};

#endif // ESMTRACKERPYRAMIDAL_H
