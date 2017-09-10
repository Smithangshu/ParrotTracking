#ifndef MEAN_SHIFT_H
#define MEAN_SHIFT_H

#include "opencv2/core/core.hpp"
#include "opencv/highgui.h"
#include "Tracker.h"

using namespace std;
using namespace cv;

class MeanShiftTracker : public Tracker
{
	public:
        static const int colorBins = 16;

		cv::Rect frameSize;

		cv::Point H;
		int iteration;
		double scalingFactor;
		double bhattaCoefficient;

		double qu[colorBins][colorBins][colorBins];
		double originalQu[colorBins][colorBins][colorBins];

		double bhattaEpsilon;
		int maxIterations;

		// Scale adaptation test
		bool enableWindowScaling;
		double scalingFactorTest;
		double maxWindowChange;

        // Manuel > Current scale for tracking window
        double currentScale;
		cv::Rect maxWindow;
		cv::Rect minWindow;

		// Model Update
		bool enableModelUpdate;
		double modelUpdateFactor;
		double modelUpdateEpsilon;

		// Display variables
		CvScalar frameColor;

		MeanShiftTracker();
        void setTargetObject(cv::Mat &Img, cv::Rect trackingWindow);
        void trackObject(cv::Mat &newImage);
        void highlightObject(cv::Mat &outputImage);
        cv::Point2f getObjectCenter();

        void getQu(cv::Mat &Img);
        void getPu(cv::Mat &Img, double pu[colorBins][colorBins][colorBins], cv::Point testPoint);
        cv::Point MeanShift(cv::Mat &Img, double pu[colorBins][colorBins][colorBins], cv::Point testPoint);
		double getBhattaCoefficient(double q[colorBins][colorBins][colorBins], double pu[colorBins][colorBins][colorBins]);
		cv::Point getWindowCenter();
        void updateTrackingWindowFromCenter(cv::Point windowCenter);
		cv::Rect calculateTrackingWindow(cv::Point windowCenter);
        void updateModel(cv::Mat &bufferedImage, cv::Point p_trackingWindowCenter);

};

#endif
