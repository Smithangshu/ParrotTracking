#ifndef PLANE_DRAWING_H
#define PLANE_DRAWING_H

#include "drawing/Drawing3D.h"
#include "tracking/ESMTracker.h"
#include "reconstruction/HomographyDecomposer.h"
#include "opencv2/core/core.hpp"
#include "drawing/Drawing3D.h"
#include <sstream>

class PlaneDrawing : public Drawing3D
{
	private:
		cv::Point3f P1, P2, P3, P4;
        bool useTracker;

	protected:
		void drawScene(int index);
		void drawPlane();
        GLuint planeTexture;

	public:
        cv::Mat W;
        cv::Mat texture;
		HomographyDecomposer homographyDecomposer;
		int planeWidth;
		int planeHeight;
        ESMTracker* tracker;

		PlaneDrawing();
        void setTextures();
        void setTexture(Mat &image);
        void refreshView();
        void updateHomography();
        void setTracker(ESMTracker* tracker);
        void enableTracker(bool enable);
        void setGLParameters(enum DrawingModes drawingMode, std::string drawingTitle, cv::Mat &texture);
};

#endif
