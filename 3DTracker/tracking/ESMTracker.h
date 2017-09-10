#ifndef ESM_TRACKER_H
#define ESM_TRACKER_H

#include "LucasKanadeTracker.h"
#include "opencv2/core/core.hpp"
#include <vector>

using namespace std;
using namespace cv;

class ESMTracker : public LucasKanadeTracker
{
    public:

        cv::Mat dw_dp_0[2][8];
        cv::Mat T_gradient_x_0;
        cv::Mat T_gradient_y_0;

        cv::Mat J_e;
        cv::Mat jacobian_pk;
        cv::Mat I_element;
        cv::Mat r_k;
        cv::Mat J_xc;
        cv::Mat delta_s;
        cv::Mat delta_x;
        cv::Mat J_sum;
        cv::Mat J_sum_inv;
        cv::Mat J_sum_transpose_times_J_sum;
        vector<cv::Mat> jacobians;

        cv::Mat J_xc_x[8];
        cv::Mat J_xc_y[8];
        cv::Mat Jacobians_x[8];
        cv::Mat Jacobians_y[8];

        ESMTracker();
        void setTargetObject(cv::Mat &image, cv::Rect selection);
        void trackObject(cv::Mat &newImage);

        void getJacobianElement(cv::Mat X[2][8], int y, int x, cv::Mat &W);
};

#endif
