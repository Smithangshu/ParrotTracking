#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "opencv2/core/core.hpp"

class Filter {
private:
    bool inited;

public:
    cv::Mat K;
    cv::Mat I;
    cv::Mat Q;
    cv::Mat H;
    cv::Mat A;
    cv::Mat R;
    cv::Mat P;
    cv::Mat FPose;

    Filter();
    void initialize(int dimensions, float QScale = 1.0);
    void setState(cv::Mat &Pose);
    void filter(cv::Mat &Pose);
    bool isInited();
    void setScale(float scale);
};

#endif // KALMANFILTER_H
