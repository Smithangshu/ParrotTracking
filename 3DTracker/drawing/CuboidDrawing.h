#ifndef CUBOID_DRAWING_H
#define CUBOID_DRAWING_H

#include "opencv2/core/core.hpp"
#include "drawing/Drawing3D.h"
#include "tracking/CuboidTrackerSingleFace.h"
#include "tracking/CuboidTrackerMultipleFace.h"
#include "GL/glut.h"

class CuboidDrawing : public Drawing3D
{
	private:
		cv::Point3f P1, P2, P3, P4;
		cv::Mat transforms[6];
		cv::Size2i sizes[6];
		GLuint planeTextures[6];
        cv::Mat textures[6];
        int totalTextures;

	protected:
		void drawScene(int index);
		void drawCuboid();

	public:
		CuboidTracker* cuboidTracker;

		CuboidDrawing();
		void setTransforms(cv::Mat transforms[6]);
		void setTransform(cv::Mat transform, int index);
        void setTextures();
		void setTextures(cv::Mat &image);
		void setTextures(cv::Mat &frontImage, cv::Mat &topImage, cv::Mat &sidetImage);
        void setTextures(cv::Mat &frontImage, cv::Mat &topImage, cv::Mat &behindImage, cv::Mat &bottomImage, cv::Mat &leftImage, cv::Mat &rightImage);
        void prepareTextures(cv::Mat &image);
        void prepareTextures(cv::Mat &frontImage, cv::Mat &topImage, cv::Mat &sidetImage);
        void prepareTextures(cv::Mat &frontImage, cv::Mat &topImage, cv::Mat &behindImage, cv::Mat &bottomImage, cv::Mat &leftImage, cv::Mat &rightImage);
        void refreshView();
        CuboidTracker* getCuboidTracker();
        void setCuboidTracker(CuboidTracker* tracker);
};

#endif
