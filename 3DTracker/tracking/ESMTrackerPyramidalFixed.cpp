#include "ESMTrackerPyramidalFixed.h"
#include <QDebug>

ESMTrackerPyramidalFixed::ESMTrackerPyramidalFixed()
{
    qDebug() << "Constructor ESM Pyr Fixed";
    ESMTracker();
}

void ESMTrackerPyramidalFixed::setTargetObject(cv::Mat &image, cv::Rect selection)
{
    ESMTracker::setTargetObject(image, selection);
}

void ESMTrackerPyramidalFixed::trackObject(cv::Mat &newImage)
{
    ESMTracker::trackObject(newImage);
}
