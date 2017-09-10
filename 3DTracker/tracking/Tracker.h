#ifndef TRACKER_H
#define TRACKER_H

#include "opencv2/core/core.hpp"
#include "common/KalmanFilter.h"
#include "models/FeatureBasedModel.h"
#include <vector>

class Tracker
{
	public:
        int currentIteration;
        int currentSubIteration;
        cv::Rect trackingWindow;
        cv::Rect originalWindow;
        cv::Point2f trackingWindowCenter;
        cv::Point2f trackingWindowCenterFiltered;
        cv::Mat targetObject;
        cv::Scalar highlightColor;
        int highlightLineWidth;
        double lastError;
        double lastAverageError;
        float lastTime;
        // Used to indicate if the algorithm converged on last iteration
        bool converged;
        enum TrackerTypes {TYPE_GENERIC, TYPE_MEAN_SHIFT, TYPE_ESM, TYPE_ESM_PYRAMIDAL, TYPE_CUBOID, TYPE_CUBOID_SINGLE_FACE} type;

        Filter filter;

        FeatureBasedModel featureBasedModel;

        virtual void trackObject(cv::Mat &newImage) = 0;
        virtual void highlightObject(cv::Mat &targetImage) = 0;
        virtual cv::Point2f getObjectCenter();
        virtual void setTargetObject(cv::Mat &image, cv::Rect selection);
        virtual void setTargetObjectSelection(cv::Rect selection);
};

#endif
