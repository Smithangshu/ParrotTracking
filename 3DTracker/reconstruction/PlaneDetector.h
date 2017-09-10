#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "common/KalmanFilter.h"
#include <vector>
#include <deque>

struct Plane {
    cv::Mat homography;
    cv::Size dimensions;
};

class PlaneDetector {
public:
    int faces;
    std::vector<Plane> planes;
    enum PlaneDetectionMethods {FLOW_HOMOGRAPHY, FEATURE_MATCHING, BINARY_SEARCH, BINARY_SEARCH_DISTINCT} planeDetectionMethod;
    std::deque<cv::Mat> images;

    cv::Mat foundHomography;
    cv::Mat foundTransform;

    Filter filter;

    int frameSkip;

    // For Binary Search
    bool modelFound;
    cv::Point linePoints[4];

    PlaneDetector();
    void addImage(cv::Mat image);
    void setMethod(PlaneDetectionMethods planeDetectionMethod);
    bool findPlanes(cv::Mat &dst);
    bool findPlanesBackProjection(cv::Mat &dst);
    bool findPlanesFlowHomography(cv::Mat &dst);
    bool findPlanesFlowHomographyCorrected(cv::Mat &dst);
    bool findPlanesFeatures(cv::Mat &dst);
    bool findPlanesBinary(cv::Mat &dst);
    bool findPlanesBinaryDistinctColors(cv::Mat &dst);
    cv::Point2f calculateVanishingPoint(std::vector<cv::Vec4i> lines, int largestLines[2]);
    bool estimateDenseFlowCPU(cv::Mat &flwX, cv::Mat &flwY);
    bool estimateDenseFlowGPU(cv::Mat &flwX, cv::Mat &flwY);
    bool filterKeypointsWithDenseFlow(std::vector<cv::KeyPoint> &keypoints, std::vector<cv::KeyPoint> &keypointsFiltered, cv::Mat &flwX, cv::Mat &flwY, cv::Mat &dst, float motionThreshold = 20.0);
    void reconstructPlanes();
    void updateModel();
    void trackObject();
    void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double scale, const cv::Scalar &color);
};

#endif // PLANEDETECTOR_H
