#include "CuboidDrawing.h"
#include "common/TrackingCommon.h"
#include "GLDrawingController.h"
#include <iostream>
#include <QDebug>

CuboidDrawing::CuboidDrawing()
{
    if (TrackingCommon::enableMultipleFaceTracking) {
        cuboidTracker = new CuboidTrackerMultipleFace();
    } else {
        cuboidTracker = new CuboidTrackerSingleFace();
    }
	for (int i = 0; i < 6; ++i)
		transforms[i] = cv::Mat(4, 4, CV_64F);
    totalTextures = 0;
}

void CuboidDrawing::drawScene(int index)
{
	Drawing3D::drawScene(index);
    drawReferential(0.2);
    goToReferential();
	drawCuboid();
	glutSwapBuffers();
}

void CuboidDrawing::drawCuboid()
{
	int linesOffset = -referentialScale / 15;

    for (int i = 0; i < 6; ++i)
	{
		glPushMatrix();

		cv::Mat W = transforms[i];
		GLfloat faceTransform[16] = {
			W.at<double>(0, 0), W.at<double>(1, 0), W.at<double>(2, 0), W.at<double>(3, 0),
			W.at<double>(0, 1), W.at<double>(1, 1), W.at<double>(2, 1), W.at<double>(3, 1),
			W.at<double>(0, 2), W.at<double>(1, 2), W.at<double>(2, 2), W.at<double>(3, 2),
			W.at<double>(0, 3), W.at<double>(1, 3), W.at<double>(2, 3), W.at<double>(3, 3)
		};
		glMultMatrixf(faceTransform);

		if (i == cuboidTracker->face)
			drawReferential(0.7);
		
        cv::Size2f size;
//        if (cuboidTracker->useRealWorldDimensions) {
//            size = cuboidTracker->realFacesDimensions[i];
//        } else {
//            size = cuboidTracker->facesDimensions[i];
//        }
        size = cuboidTracker->facesDimensions[i];

		P1 = cvPoint3D32f(0, 0, 0);
        P2 = cvPoint3D32f(size.width, 0, 0);
        P3 = cvPoint3D32f(size.width, size.height, 0);
        P4 = cvPoint3D32f(0, size.height, 0);
		//P1 = cvPoint3D32f(-sizes[i].width / 2, -sizes[i].height / 2, 0);
		//P2 = cvPoint3D32f(sizes[i].width / 2, -sizes[i].height / 2, 0);
		//P3 = cvPoint3D32f(sizes[i].width / 2, sizes[i].height / 2, 0);
		//P4 = cvPoint3D32f(-sizes[i].width / 2, sizes[i].height / 2, 0);

		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		glBindTexture(GL_TEXTURE_2D, planeTextures[i]);
		glBegin(GL_QUADS);
		glTexCoord2d(0.0, 0.0); 
		glVertex3f(P1.x, P1.y, P1.z);
		glTexCoord2d(1.0, 0.0); 
		glVertex3f(P2.x, P2.y, P2.z);
		glTexCoord2d(1.0, 1.0);
		glVertex3f(P3.x, P3.y, P3.z);
		glTexCoord2d(0.0, 1.0);
		glVertex3f(P4.x, P4.y, P4.z);
		glEnd();
		
		if (i != cuboidTracker->face)
			glLineWidth(referentialScale / 50);
		else
			glLineWidth(referentialScale / 20);

		glColor4f(cuboidTracker->faceColors[i].val[0], cuboidTracker->faceColors[i].val[1], cuboidTracker->faceColors[i].val[2], 1.0f);
		glBegin(GL_LINE_LOOP);
		glVertex3f(P1.x, P1.y, P1.z + linesOffset);
		glVertex3f(P2.x, P2.y, P2.z + linesOffset);
		glVertex3f(P3.x, P3.y, P3.z + linesOffset);
		glVertex3f(P4.x, P4.y, P4.z + linesOffset);
		glEnd();

		glPopMatrix();
	}
}

void CuboidDrawing::setTransforms(cv::Mat transforms[6])
{
	for (int i = 0; i < 6; ++i)
	{
		transforms[i].copyTo(this->transforms[i]);
	}
}

void CuboidDrawing::setTransform(cv::Mat transform, int index)
{
	transforms[index].copyTo(this->transforms[index]);
}

void CuboidDrawing::setTextures(cv::Mat &image)
{
	cv::Mat imageTexture = image.clone();
	
	for (int i = 0; i < 6; ++i)
	{
        if (drawingMode == SINGLE_VIEW)
		{
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
		}
		else
		{
			glutSetWindow(GLDrawingController::glFrontViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
			glutSetWindow(GLDrawingController::glTopViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
			glutSetWindow(GLDrawingController::glSideViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
			glutSetWindow(GLDrawingController::glPerspectiveViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
		}
	}
}

void CuboidDrawing::setTextures(cv::Mat &frontImage, cv::Mat &topImage, cv::Mat &sidetImage)
{
	setTextures(frontImage, topImage, frontImage, topImage, sidetImage, sidetImage);
}

void CuboidDrawing::setTextures(cv::Mat &frontImage, cv::Mat &topImage, cv::Mat &behindImage, cv::Mat &bottomImage, cv::Mat &leftImage, cv::Mat &rightImage)
{
	cv::Mat imageTextures[6];
	imageTextures[0] = frontImage;
	imageTextures[1] = topImage;
	imageTextures[2] = behindImage;
	imageTextures[3] = bottomImage;
	imageTextures[4] = leftImage;
	imageTextures[5] = rightImage;
	
	for (int i = 0; i < 6; ++i)
	{
		cv::Mat imageTexture = imageTextures[i];
        if (drawingMode == SINGLE_VIEW)
		{
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
		}
		else
		{
			glutSetWindow(GLDrawingController::glFrontViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
			glutSetWindow(GLDrawingController::glTopViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
			glutSetWindow(GLDrawingController::glSideViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
			glutSetWindow(GLDrawingController::glPerspectiveViews[windowIndex]);
			GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTextures[i]);
		}
	}
}

CuboidTracker* CuboidDrawing::getCuboidTracker()
{
	return cuboidTracker;
}

void CuboidDrawing::setTextures()
{
    setTextures(textures[0], textures[1], textures[2], textures[3], textures[4], textures[5]);
}

void CuboidDrawing::prepareTextures(Mat &image)
{
    prepareTextures(image, image, image, image, image, image);
}

void CuboidDrawing::prepareTextures(Mat &frontImage, Mat &topImage, Mat &sidetImage)
{
    prepareTextures(frontImage, topImage, sidetImage);
}

void CuboidDrawing::prepareTextures(Mat &frontImage, Mat &topImage, Mat &behindImage, Mat &bottomImage, Mat &leftImage, Mat &rightImage)
{
    textures[0] = frontImage.clone();
    textures[1] = topImage.clone();
    textures[2] = behindImage.clone();
    textures[3] = bottomImage.clone();
    textures[4] = leftImage.clone();
    textures[5] = rightImage.clone();
}

void CuboidDrawing::refreshView()
{
    this->setTransforms(cuboidTracker->transformations);
    Drawing3D::refreshView();
}

void CuboidDrawing::setCuboidTracker(CuboidTracker *tracker)
{
    this->cuboidTracker = tracker;
}
