#include "drawing/PlaneDrawing.h"
#include "drawing/GLDrawingController.h"
#include "common/TrackingCommon.h"
#include <iostream>

PlaneDrawing::PlaneDrawing()
{
	planeWidth = 100;
	planeHeight = 100;

	W = cv::Mat(4, 4, CV_64F);
	
	homographyDecomposer = HomographyDecomposer();
    TrackingCommon::loadCameraParameters("../3ds35mmR.yml");
    homographyDecomposer.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);
    parametersReady = false;
    useTracker = true;
}

void PlaneDrawing::refreshView()
{
    updateHomography();
    Drawing3D::refreshView();
}

void PlaneDrawing::updateHomography()
{
    if (useTracker)
    {
        // Tracker now works only with floats (CV_32F), while decomposer doubles
        cv::Mat newHomography;
        tracker->W_3x3.convertTo(newHomography, CV_64F);
        homographyDecomposer.setHomography(newHomography);
        homographyDecomposer.decompose();
        homographyDecomposer.transform.copyTo(W);
    }
}

void PlaneDrawing::drawScene(int index)
{
	Drawing3D::drawScene(index);
	drawReferential(0.2);
	drawPlane();
    glutSwapBuffers();
}

void PlaneDrawing::drawPlane()
{
    glPushMatrix();

    GLfloat decompositionTransform[16] = {
        W.at<double>(0, 0), W.at<double>(1, 0), W.at<double>(2, 0), W.at<double>(3, 0),
        W.at<double>(0, 1), W.at<double>(1, 1), W.at<double>(2, 1), W.at<double>(3, 1),
        W.at<double>(0, 2), W.at<double>(1, 2), W.at<double>(2, 2), W.at<double>(3, 2),
        W.at<double>(0, 3), W.at<double>(1, 3), W.at<double>(2, 3), W.at<double>(3, 3)
    };
    glMultMatrixf(decompositionTransform);

    drawReferential(0.7);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glBindTexture(GL_TEXTURE_2D, planeTexture);
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

    glPopMatrix();
}

void PlaneDrawing::setTextures()
{
    qDebug() << "SetTexture called";
    setTexture(texture);
}

void PlaneDrawing::setTexture(cv::Mat &image)
{
	planeWidth = image.cols;
	planeHeight = image.rows;

	cv::Mat imageTexture = image.clone();

    if (drawingMode == SINGLE_VIEW)
	{
        GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTexture);
	}
	else
	{
        glutSetWindow(GLDrawingController::glFrontViews[windowIndex]);
        GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTexture);
        glutSetWindow(GLDrawingController::glTopViews[windowIndex]);
        GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTexture);
        glutSetWindow(GLDrawingController::glSideViews[windowIndex]);
        GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTexture);
        glutSetWindow(GLDrawingController::glPerspectiveViews[windowIndex]);
        GLDrawingController::loadTextureWithOpenCV(imageTexture, &planeTexture);
	}

    // Use this if can't see plane (offset it)
    if (useTracker)
    {
        P1 = cv::Point3f(0, 0, 0);
        P2 = cv::Point3f(planeWidth, 0, 0);
        P3 = cv::Point3f(planeWidth, planeHeight, 0);
        P4 = cv::Point3f(0, planeHeight, 0);
    }
    else
    {
        P1 = cv::Point3f(-planeWidth / 2, -planeHeight / 2, 0);
        P2 = cv::Point3f(planeWidth / 2, -planeHeight / 2, 0);
        P3 = cv::Point3f(planeWidth / 2, planeHeight / 2, 0);
        P4 = cv::Point3f(-planeWidth / 2, planeHeight / 2, 0);
    }
}

void PlaneDrawing::setTracker(ESMTracker* tracker)
{
    this->tracker = tracker;
}

void PlaneDrawing::enableTracker(bool enable)
{
    useTracker = enable;
}

void PlaneDrawing::setGLParameters(PlaneDrawing::DrawingModes drawingMode, std::string drawingTitle, cv::Mat &texture)
{
    this->drawingMode = drawingMode;
    this->drawingTitle = drawingTitle;
    texture.copyTo(this->texture);
    parametersReady = true;
}
