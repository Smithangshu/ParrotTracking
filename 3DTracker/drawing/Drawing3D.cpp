#include "Drawing3D.h"
#include "GLDrawingController.h"
#include "reconstruction/HomographyDecomposer.h"
#include "common/TrackingCommon.h"
#include <iostream>
#include <sstream>

Drawing3D::Drawing3D()
{
    objectCenter = cv::Mat(4, 4, CV_64F);
    cv::setIdentity(objectCenter);
    referential = cv::Mat(4, 4, CV_64F);
    cv::setIdentity(referential);
	showReferential = false;
	referentialScale = 100.0f;
    cameraScale = 100.0f;
	windowIndex = 0;
	isViewCreated = false;
}

void Drawing3D::updateCameraSettings()
{
	// Front view, 0.0, 0.0, cameraScale, 0.0, 0.0, 0.0, 0.0, cameraScale, 0.0
	cameraSettings[0][0] = 0.0;
	cameraSettings[0][1] = 0.0;
    cameraSettings[0][2] = cameraScale;
	cameraSettings[0][3] = 0.0;
	cameraSettings[0][4] = 0.0;
	cameraSettings[0][5] = 0.0;
	cameraSettings[0][6] = 0.0;
	cameraSettings[0][7] = cameraScale;
	cameraSettings[0][8] = 0.0;

	// Top view, 0.0, cameraScale, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -cameraScale
	cameraSettings[1][0] = 0.0;
    cameraSettings[1][1] = cameraScale;
	cameraSettings[1][2] = 0.0;
	cameraSettings[1][3] = 0.0;
	cameraSettings[1][4] = 0.0;
	cameraSettings[1][5] = 0.0;
	cameraSettings[1][6] = 0.0;
	cameraSettings[1][7] = 0.0;
	cameraSettings[1][8] = -cameraScale;

	// Side view, cameraScale, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cameraScale, 0.0
	cameraSettings[2][0] = cameraScale;
	cameraSettings[2][1] = 0.0;
	cameraSettings[2][2] = 0.0;
	cameraSettings[2][3] = 0.0;
	cameraSettings[2][4] = 0.0;
	cameraSettings[2][5] = 0.0;
	cameraSettings[2][6] = 0.0;
	cameraSettings[2][7] = cameraScale;
	cameraSettings[2][8] = 0.0;

	// Perspective view, cameraScale * 0.7, cameraScale * 0.7, cameraScale * 0.7, 0.0, 0.0, 0.0, cameraScale, cameraScale, 0.0
	cameraSettings[3][0] = cameraScale * 0.7;
	cameraSettings[3][1] = cameraScale * 0.7;
	cameraSettings[3][2] = cameraScale * 0.7;
	cameraSettings[3][3] = 0.0;
	cameraSettings[3][4] = 0.0;
	cameraSettings[3][5] = 0.0;
	cameraSettings[3][6] = cameraScale;
	cameraSettings[3][7] = cameraScale;
	cameraSettings[3][8] = 0.0;
}

void Drawing3D::setZero()
{
    TrackingCommon::objectZero.copyTo(referential);
    qDebug() << "New referential has been set";

    // Also, set the cameras to the default position
    updateCameraSettings();
}

void Drawing3D::setObjectCenter(cv::Mat &objectCenter)
{
    objectCenter.copyTo(this->objectCenter);
}

void Drawing3D::goToReferential()
{
    GLfloat zeroTransform[16] = {
        referential.at<double>(0, 0), referential.at<double>(1, 0), referential.at<double>(2, 0), referential.at<double>(3, 0),
        referential.at<double>(0, 1), referential.at<double>(1, 1), referential.at<double>(2, 1), referential.at<double>(3, 1),
        referential.at<double>(0, 2), referential.at<double>(1, 2), referential.at<double>(2, 2), referential.at<double>(3, 2),
        referential.at<double>(0, 3), referential.at<double>(1, 3), referential.at<double>(2, 3), referential.at<double>(3, 3)
    };
    glMultMatrixf(zeroTransform);
}

void Drawing3D::setCameraScale(float cameraScale)
{
	this->cameraScale = cameraScale;
	updateCameraSettings();
}

void Drawing3D::setDefaultCamera()
{
    setCamera(Drawing3D::FRONT_VIEW, 0.0, 0.0, -cameraScale * 2, 0.0, 0.0, -cameraScale * 3, 0.0, cameraScale, 0.0);
    setCamera(Drawing3D::TOP_VIEW, 0.0, cameraScale * 2, -cameraScale * 3, 0.0, 0.0, -cameraScale * 3, 0.0, 0.0, -cameraScale);
    setCamera(Drawing3D::SIDE_VIEW, cameraScale * 2, 0.0, -cameraScale * 3, 0.0, 0.0, -cameraScale * 3, 0.0, cameraScale, 0.0);
    setCamera(Drawing3D::PERSPECTIVE_VIEW, cameraScale * 2, cameraScale * 2, -cameraScale, 0.0, 0.0, -cameraScale * 2, 0.0, cameraScale, 0.0);
}

float Drawing3D::getCameraScale()
{
	return cameraScale;
}

void Drawing3D::setCamera(enum CameraViews cameraView, float eyeX, float eyeY, float eyeZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ)
{
	cameraSettings[cameraView][0] = eyeX;
	cameraSettings[cameraView][1] = eyeY;
	cameraSettings[cameraView][2] = eyeZ;
	cameraSettings[cameraView][3] = centerX;
	cameraSettings[cameraView][4] = centerY;
	cameraSettings[cameraView][5] = centerZ;
	cameraSettings[cameraView][6] = upX;
	cameraSettings[cameraView][7] = upY;
	cameraSettings[cameraView][8] = upZ;
}

void Drawing3D::createView()
{
    createView(drawingMode, drawingTitle.c_str());
}

void Drawing3D::createView(enum DrawingModes drawingMode, const char *windowName)
{
	if (!isViewCreated)
	{
        GLDrawingController::registerWindow(this, windowName);
		this->drawingMode = drawingMode;

        if (drawingMode == MULTI_VIEW) // For multi view
		{
            GLDrawingController::registerSecondaryWindow(this, GLDrawingController::FRONT);
            GLDrawingController::registerSecondaryWindow(this, GLDrawingController::TOP);
            GLDrawingController::registerSecondaryWindow(this, GLDrawingController::SIDE);
            GLDrawingController::registerSecondaryWindow(this, GLDrawingController::PERSPECTIVE);
        }

		isViewCreated = true;
	}
}

void Drawing3D::drawReferential(float alpha)
{
	CvPoint3D32f X = cvPoint3D32f(referentialScale, 0.0f, 0.0f);
	CvPoint3D32f Y = cvPoint3D32f(0.0f, referentialScale, 0.0f);
	CvPoint3D32f Z = cvPoint3D32f(0.0f, 0.0f, referentialScale);
	CvPoint3D32f C = cvPoint3D32f(0.0f, 0.0f, 0.0f);
	float coneScale = referentialScale / 6;

	glPushMatrix();

	glLineWidth(referentialScale / 20);	

	glPushMatrix();
	glColor4f(0.6f, 0.6f, 0.6f, alpha);
	glTranslatef(C.x, C.y, C.z);
	glutSolidCube(10);
	glPopMatrix();

	glPushMatrix();
	glColor4f(0.0f, 0.0f, 1.0f, alpha); // Blue, X axis
	glTranslatef(X.x, X.y, X.z);
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	glutSolidCone(coneScale, 2 * coneScale, 20, 20);
	glPopMatrix();
	glBegin(GL_LINES);
	glVertex3f(C.x, C.y, C.z);
	glVertex3f(X.x, X.y, X.z);
	glEnd();

	glPushMatrix(); // Green, Y axis
	glColor4f(0.0f, 1.0f, 0.0f, alpha);
	glTranslatef(Y.x, Y.y, Y.z);
	glRotatef(90.0f, -1.0f, 0.0f, 0.0f);
	glutSolidCone(coneScale, 2 * coneScale, 20, 20);
	glPopMatrix();
	glBegin(GL_LINES);
	glVertex3f(C.x, C.y, C.z);
	glVertex3f(Y.x, Y.y, Y.z);
	glEnd();

	glPushMatrix(); // Red, Z axis
	glColor4f(1.0f, 0.0f, 0.0f, alpha);
	glTranslatef(Z.x, Z.y, Z.z);
	glutSolidCone(coneScale, 2 * coneScale, 20, 20);
	glPopMatrix();
	glBegin(GL_LINES);
	glVertex3f(C.x, C.y, C.z);
	glVertex3f(Z.x, Z.y, Z.z);
	glEnd();

	glPopMatrix();
}

void Drawing3D::initScene()
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90, 1.333, 1.0, 2000.0);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Drawing3D::drawScene(int index)
{
	// X pops, Z up and Y right
	//glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
	//glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);

	// X unchanged, Y and Z mirrored
	glRotatef(180.0f, 1.0f, 0.0f, 0.0f);

    //if (showReferential == true)
    //    drawReferential(0.9);
}

void Drawing3D::refreshView()
{
	if (isViewCreated)
	{
		GLDrawingController::updateWindow(windowIndex);
	}
}

void Drawing3D::updateDisplay()
{
    if (drawingMode == SINGLE_VIEW)
	{
		updateDisplayFront();
	}
	else
	{
        glutSwapBuffers();
	}
}

void Drawing3D::updateDisplayFront()
{
	initScene();
	gluLookAt(cameraSettings[0][0], cameraSettings[0][1], cameraSettings[0][2], cameraSettings[0][3], cameraSettings[0][4], cameraSettings[0][5], cameraSettings[0][6], cameraSettings[0][7], cameraSettings[0][8]);
	drawScene(0);
}

void Drawing3D::updateDisplayTop()
{
	initScene();
	gluLookAt(cameraSettings[1][0], cameraSettings[1][1], cameraSettings[1][2], cameraSettings[1][3], cameraSettings[1][4], cameraSettings[1][5], cameraSettings[1][6], cameraSettings[1][7], cameraSettings[1][8]);
	drawScene(1);
}

void Drawing3D::updateDisplaySide()
{
	initScene();
	gluLookAt(cameraSettings[2][0], cameraSettings[2][1], cameraSettings[2][2], cameraSettings[2][3], cameraSettings[2][4], cameraSettings[2][5], cameraSettings[2][6], cameraSettings[2][7], cameraSettings[2][8]);
	drawScene(2);
}

void Drawing3D::updateDisplayPerspective()
{
    glutSetWindow(5);
    initScene();
    gluLookAt(cameraSettings[3][0], cameraSettings[3][1], cameraSettings[3][2], cameraSettings[3][3], cameraSettings[3][4], cameraSettings[3][5], cameraSettings[3][6], cameraSettings[3][7], cameraSettings[3][8]);
    drawScene(3);
}

void Drawing3D::setTextures()
{

}

void Drawing3D::setGLParameters(DrawingModes drawingMode, std::string drawingTitle)
{
    this->drawingMode = drawingMode;
    this->drawingTitle = drawingTitle;
    parametersReady = true;
}
