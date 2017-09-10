#ifndef ESMTRACKERPYRAMIDALFIXED_H
#define ESMTRACKERPYRAMIDALFIXED_H

#include "ESMTracker.h"
#include "opencv2/core/core.hpp"

class ESMTrackerPyramidalFixed : public ESMTracker
{
    public:
        ESMTrackerPyramidalFixed();
        void setTargetObject(cv::Mat &image, cv::Rect selection);
        void trackObject(cv::Mat &newImage);
};

#endif // ESMTRACKERPYRAMIDALFIXED_H
