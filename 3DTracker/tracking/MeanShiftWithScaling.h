#ifndef MEAN_SHIFT_WITH_SCALING_H
#define MEAN_SHIFT_WITH_SCALING_H

#include "opencv2/core/core.hpp"
#include "tracking/MeanShiftTracker.h"
#include "reconstruction/PositionEstimator.h"

class MeanShiftWithScaling : public MeanShiftTracker
{
	protected:
		void updateScale();

	public:
		MeanShiftWithScaling();
        void trackObject(cv::Mat &newImage);
};

#endif
