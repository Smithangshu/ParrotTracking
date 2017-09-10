#ifndef LUCAS_KANADE_TRACKER_H
#define LUCAS_KANADE_TRACKER_H

#include "Tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class LucasKanadeTracker : public Tracker
{
	public:
		double epsilon;
		int maxIterations;

        cv::Mat T; // Target object converted to grayscale
        cv::Mat I;
        cv::Mat I_gradient_x;
        cv::Mat I_gradient_y;
        cv::Mat I_w;
        cv::Mat I_w_gradient_x;
        cv::Mat I_w_gradient_y;
        cv::Mat warped_image;
        cv::Mat error_image;

        cv::Mat parameters;
        cv::Mat W_3x3;
        cv::Mat H_;
        cv::Mat H_inv;
        cv::Mat dp_right_operand;
        cv::Mat dp;

        cv::Mat dw_dp[2][8];
        cv::Mat steepest_descend_images[8];

        // For pyramids
        static const int pyramids = 3;
        cv::Mat templates[pyramids];
        cv::Mat imagePyramids[pyramids];
        cv::Mat imagePyramidsDisplay[pyramids];
        cv::Mat warpedImages[pyramids];
        cv::Mat warpedImagesGradientX[pyramids];
        cv::Mat warpedImagesGradientY[pyramids];
        cv::Mat scaleTransform;
        cv::Mat restoreTransform;
        cv::Mat newTransform;
        int topPyramid;

        // For color
        bool useColor;
        int imageFormat8UCX;
        int imageFormat16SCX;
        int imageFormat32FCX;

        // Difference weightning
        cv::Mat templateWeight[pyramids];
        bool enableTemplateWeighting;

        // For transforming between parameters and transform mat
        cv::Mat fromTransformToParameters;
        cv::Mat fromParametersToTransform;

		LucasKanadeTracker();
        void setTargetObject(cv::Mat &image, cv::Rect selection);
        void trackObject(cv::Mat &newImage);
        void highlightObject(cv::Mat &targetImage);

        void calculateJacobians(cv::Mat dw_dp[2][8], cv::Mat &W);
        double computeErrorImage(cv::Mat &image_1, cv::Mat &image_2, cv::Mat &error_image);
        void computeSteepestDescendImages(cv::Mat &gradient_x, cv::Mat &gradient_y, cv::Mat jacobians[2][8], cv::Mat sd_images[8]);
        void computeHessian(cv::Mat sd_images[8], cv::Mat &H);
        void computeSteepestDescendUpdate(cv::Mat sd_images[8], cv::Mat &error_image, cv::Mat &dp_right_operand);
        void setTransformFromParameters(cv::Mat &W_3x3, cv::Mat &parameters);
        void setParametersFromTransform(cv::Mat &parameters, cv::Mat &W_3x3);
        double getMagnitude(cv::Mat &columnMatrix);

		// Warping functions
        void updateWarpAdditive(cv::Mat &dp, cv::Mat &W_dest);
        void updateWarpCompositional(cv::Mat &dp, cv::Mat &W_dest);
        void updateWarpInverseCompositional(cv::Mat &dp, cv::Mat &W_dest);

		// Auxiliary
        void drawWarpedRect(cv::Mat &pImage);
        void drawWarpedRect(cv::Mat &pImage, cv::Mat &W, int width, int height);
        cv::Point pointTransformPerspective(cv::Point point, cv::Mat &W);
		double computeSSDBetweenTemplateAndWarpedImage();
        void getHomography(cv::Mat &target_mat);
        void setHomography(cv::Mat &homography, bool convertMatFormat = true);

        // Pyramids
        void setupPyramids();
        cv::Mat changeTransformScale(cv::Mat transform, cv::Rect templateSize, float factor);
        void setTopPyramid(int topPyramid);

        // Mask
        void enableMaskedTemplate(bool enable);

        // Color
        void enableColor(bool enable);

};

#define SET_VECTOR(X, u, v)\
	CV_MAT_ELEM(*(X), float, 0, 0) = (float)(u);\
	CV_MAT_ELEM(*(X), float, 1, 0) = (float)(v);\
	CV_MAT_ELEM(*(X), float, 2, 0) = 1.0f;

#define SET_VECTOR_8(X, p1, p2, p3, p4, p5, p6, p7, p8)\
    X.at<float>(0, 0) = (float)(p1);\
    X.at<float>(1, 0) = (float)(p2);\
    X.at<float>(2, 0) = (float)(p3);\
    X.at<float>(3, 0) = (float)(p4);\
    X.at<float>(4, 0) = (float)(p5);\
    X.at<float>(5, 0) = (float)(p6);\
    X.at<float>(6, 0) = (float)(p7);\
    X.at<float>(7, 0) = (float)(p8);

#define SET_TRANSFORM_MAT(X, W)\
    X.at<float>(0, 0) = (float)(1.0 + W.at<float>(0, 0));\
    X.at<float>(1, 0) = (float)(W.at<float>(1, 0));\
    X.at<float>(2, 0) = (float)(W.at<float>(2, 0));\
    X.at<float>(0, 1) = (float)(W.at<float>(3, 0));\
    X.at<float>(1, 1) = (float)(1.0 + W.at<float>(4, 0));\
    X.at<float>(2, 1) = (float)(W.at<float>(5, 0));\
    X.at<float>(0, 2) = (float)(W.at<float>(6, 0));\
    X.at<float>(1, 2) = (float)(W.at<float>(7, 0));\
    X.at<float>(2, 2) = 1.0;

#define SET_TRANSFORM_MAT_2(X, W)\
    X.at<float>(0, 0) = (float)(0.5 + W.at<float>(0, 0));\
    X.at<float>(1, 0) = (float)(W.at<float>(1, 0));\
    X.at<float>(2, 0) = (float)(W.at<float>(2, 0));\
    X.at<float>(0, 1) = (float)(W.at<float>(3, 0));\
    X.at<float>(1, 1) = (float)(0.5 + W.at<float>(4, 0));\
    X.at<float>(2, 1) = (float)(W.at<float>(5, 0));\
    X.at<float>(0, 2) = (float)(W.at<float>(6, 0));\
    X.at<float>(1, 2) = (float)(W.at<float>(7, 0));\
    X.at<float>(2, 2) = 1.0;

#define SET_TRANSFORM_VECTOR_8(X, W)\
    X.at<float>(0, 0) = (float)(W.at<float>(0, 0) - 1.0);\
    X.at<float>(1, 0) = (float)(W.at<float>(1, 0));\
    X.at<float>(2, 0) = (float)(W.at<float>(2, 0));\
    X.at<float>(3, 0) = (float)(W.at<float>(0, 1));\
    X.at<float>(4, 0) = (float)(W.at<float>(1, 1) - 1.0);\
    X.at<float>(5, 0) = (float)(W.at<float>(2, 1));\
    X.at<float>(6, 0) = (float)(W.at<float>(0, 2));\
    X.at<float>(7, 0) = (float)(W.at<float>(1, 2));

#define SET_PARAMETERS(X, p1, p2, p3, p4, p5, p6, p7, p8)\
    X.at<float>(0, 0) = (float) p1;\
    X.at<float>(1, 0) = (float) p2;\
    X.at<float>(2, 0) = (float) p3;\
    X.at<float>(3, 0) = (float) p4;\
    X.at<float>(4, 0) = (float) p5;\
    X.at<float>(5, 0) = (float) p6;\
    X.at<float>(6, 0) = (float) p7;\
    X.at<float>(7, 0) = (float) p8;

#define GET_VECTOR_8(X, p1, p2, p3, p4, p5, p6, p7, p8)\
    (p1) = X.at<float>(0, 0);\
    (p2) = X.at<float>(1, 0);\
    (p3) = X.at<float>(2, 0);\
    (p4) = X.at<float>(3, 0);\
    (p5) = X.at<float>(4, 0);\
    (p6) = X.at<float>(5, 0);\
    (p7) = X.at<float>(6, 0);\
    (p8) = X.at<float>(7, 0);

#define GET_INT_VECTOR_8(X, p1, p2, p3, p4, p5, p6, p7, p8)\
    (p1) = (int)X.at<float>(0, 0);\
    (p2) = (int)X.at<float>(1, 0);\
    (p3) = (int)X.at<float>(2, 0);\
    (p4) = (int)X.at<float>(3, 0);\
    (p5) = (int)X.at<float>(4, 0);\
    (p6) = (int)X.at<float>(5, 0);\
    (p7) = (int)X.at<float>(6, 0);\
    (p8) = (int)X.at<float>(7, 0);

#define PRINT_VECTOR_8(X)\
    cout << X.at<float>(0, 0) << " ";;\
    cout << X.at<float>(1, 0) << " ";;\
    cout << X.at<float>(2, 0) << " ";;\
    cout << X.at<float>(3, 0) << " ";;\
    cout << X.at<float>(4, 0) << " ";;\
    cout << X.at<float>(5, 0) << " ";;\
    cout << X.at<float>(6, 0) << " ";\
    cout << X.at<float>(7, 0) << "\r\n";

#define PRINT_TO_FILE_VECTOR_8(X, file_stream)\
    file_stream << X.at<float>(0, 0) << " ";\
    file_stream << X.at<float>(1, 0) << " ";\
    file_stream << X.at<float>(2, 0) << " ";\
    file_stream << X.at<float>(3, 0) << " ";\
    file_stream << X.at<float>(4, 0) << " ";\
    file_stream << X.at<float>(5, 0) << " ";\
    file_stream << X.at<float>(6, 0) << " ";\
    file_stream << X.at<float>(7, 0) << "\r\n";

#define GET_MAGNITUDE_8(X, mag)\
    (mag) = pow((double)pow(X.at<float>(0, 0), 2) + pow(X.at<float>(1, 0), 2) + pow(X.at<float>(2, 0), 2) + pow(X.at<float>(3, 0), 2) + pow(X.at<float>(4, 0), 2) + pow(X.at<float>(5, 0), 2) + pow(X.at<float>(6, 0), 2) + pow(X.at<float>(7, 0), 2), 0.5);


#endif
