#ifndef DRAWING_3D_H
#define DRAWING_3D_H

#include "opencv2/core/core.hpp"
#include "GL/freeglut.h"
#include <sstream>

class Drawing3D
{
	private:
		float cameraSettings[4][9];
        void updateCameraSettings();

	protected:
		void drawReferential(float alpha = 0.7);
		void initScene();
		virtual void drawScene(int index);

	public:
        bool parametersReady;
        enum DrawingModes {SINGLE_VIEW, MULTI_VIEW} drawingMode;
        std::string drawingTitle;
        enum CameraViews {FRONT_VIEW, TOP_VIEW, SIDE_VIEW, PERSPECTIVE_VIEW} cameraView;

        cv::Mat objectCenter;
        cv::Mat referential;
		float referentialScale;
		bool isViewCreated;
		bool showReferential;
		int windowIndex;
		float cameraScale;

		Drawing3D();
		void setCamera(enum CameraViews cameraView, float eyeX, float eyeY, float eyeZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ);
        void setDefaultCamera();
        float getCameraScale();
        void setZero();
        void setObjectCenter(cv::Mat &objectCenter);
        void goToReferential();
		void setCameraScale(float cameraScale);
        void createView();
        void createView(enum DrawingModes drawingMode, const char *windowName);
        virtual void refreshView();
		void updateDisplay();
		void updateDisplayFront();
		void updateDisplayTop();
		void updateDisplaySide();
		void updateDisplayPerspective();
        virtual void setTextures();
        virtual void setGLParameters(enum DrawingModes drawingMode, std::string drawingTitle);
};

#endif
