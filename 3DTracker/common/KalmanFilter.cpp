#include "KalmanFilter.h"

Filter::Filter()
{
    inited = false;
}

void Filter::initialize(int dimensions, float QScale)
{
    Q = cv::Mat(dimensions, dimensions, CV_32F);
    cv::setIdentity(Q, cv::Scalar(1));

    H = cv::Mat::eye(dimensions, dimensions, CV_32F);
    A = cv::Mat::eye(dimensions, dimensions, CV_32F);

    R = cv::Mat(dimensions, dimensions, CV_32F);
    cv::setIdentity(R, cv::Scalar(QScale));

    P = cv::Mat::eye(dimensions, dimensions, CV_32F);
    I = cv::Mat::eye(dimensions, dimensions, CV_32F);

    FPose = cv::Mat(dimensions, 1, CV_32F);
    FPose.setTo(0);

    inited = true;
}

void Filter::setState(cv::Mat &Pose)
{
    Pose.copyTo(FPose);
}

void Filter::filter(cv::Mat &Pose)
{
    FPose = A * FPose;
    P = A * P * A.t() + Q;
    K = P * H * (H * P * H.t() + R).inv();
    FPose = FPose + K * (Pose - H * FPose);
    P = (I - K * H) * P;
}

bool Filter::isInited()
{
    return inited;
}

void Filter::setScale(float scale)
{
    cv::setIdentity(R, cv::Scalar(scale));
}
