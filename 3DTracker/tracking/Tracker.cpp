#include "Tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <QDebug>

void Tracker::setTargetObject(cv::Mat &image, cv::Rect selection)
{
    type = TYPE_GENERIC;
	currentIteration = 0;
    currentSubIteration = 0;
    lastError = 0.0;
    lastTime = 0.0;
    converged = false;

	trackingWindow = selection;
	originalWindow = selection;

    cv::Mat newTarget = image(selection);
    newTarget.copyTo(targetObject);

    // Store model for recovering
    cv::SurfFeatureDetector surfDetector(400);
    surfDetector.detect(targetObject, featureBasedModel.keypoints);
    qDebug() << "Found " << featureBasedModel.keypoints.size();
}

cv::Point2f Tracker::getObjectCenter()
{
    return cv::Point2f(trackingWindow.x + trackingWindow.width / 2, trackingWindow.y + trackingWindow.height / 2);
}

void Tracker::setTargetObjectSelection(cv::Rect selection)
{
    trackingWindow = selection;
    originalWindow = selection;
}
