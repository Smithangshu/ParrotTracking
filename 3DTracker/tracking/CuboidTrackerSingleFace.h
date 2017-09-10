#ifndef CUBOID_TRACKER_SINGLE_FACE_H
#define CUBOID_TRACKER_SINGLE_FACE_H

#include "CuboidTracker.h"

class CuboidTrackerSingleFace : public CuboidTracker
{
	public:
		int calculateFaceWithLeastError();
		int calculateFaceWithNormalMostAligned();

        CuboidTrackerSingleFace();
        void trackObject(cv::Mat &newImage);
        void highlightObject(cv::Mat &targetImage);
};

#endif
