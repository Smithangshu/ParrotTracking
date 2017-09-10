#ifndef ESMTRACKERCONSTRAINED_H
#define ESMTRACKERCONSTRAINED_H

#include "ESMTrackerPyramidal.h"
#include "LucasKanadeTracker.h"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class ESMTrackerConstrained : public ESMTrackerPyramidal
{
    public:
        const static int totalParameters = 6;
        cv::Mat templateGradientX[pyramids];
        cv::Mat templateGradientY[pyramids];
        cv::Mat dWdP[pyramids][2][totalParameters];
        cv::Mat Je[pyramids];
        cv::Mat Jc[pyramids];
        cv::Mat delta_s[pyramids];
        cv::Mat delta_x[pyramids];
        cv::Mat J_sum[pyramids];
        cv::Mat J_sum_inv[pyramids];
        cv::Mat J_sum_transpose_times_J_sum[pyramids];
        cv::Mat JacobiansX[pyramids][totalParameters];
        cv::Mat JacobiansY[pyramids][totalParameters];
        cv::Mat J_xc[pyramids];
        cv::Mat J_xc_x[pyramids][totalParameters];
        cv::Mat J_xc_y[pyramids][totalParameters];

        ESMTrackerConstrained();
        void setTargetObject(cv::Mat &image, cv::Rect selection);
        void trackObject(cv::Mat &newImage);
        void calculateJacobians(cv::Mat dw_dp[2][totalParameters], cv::Mat &parameters);
        void getJacobianElement(cv::Mat X[2][totalParameters], int y, int x, cv::Mat &W);
        void setTransformFromParameters(cv::Mat &W_3x3, cv::Mat &parameters);
        void setParametersFromTransform(cv::Mat &parameters, cv::Mat &W_3x3);
        void updateWarpCompositional(cv::Mat &dp, cv::Mat &W_dest);
};

#endif // ESMTRACKERCONSTRAINED_H
