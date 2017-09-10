#include "drawing/GLDrawingController.h";
#include "common/TrackingCommon.h";
#include <QDebug>
//#include <QApplication>
//#include <QDesktopWidget>
#include <iostream>
#include <sstream>
//#include "drawing/window.h"

namespace GLDrawingController
{
	int totalGlWindows = 0;
	bool glutIsReady = false;
	int nextDisplayWindow = 0;
	int windowBorder = 4;
	int windowWidth = 800;
	int windowHeight = 600;
	int subwindowWidth = windowWidth / 2 - 1;
	int subwindowHeight = windowHeight / 2 - 1;

	Drawing3D* drawingWindows[20];
	int glWindows[20];
	int glWindowModes[20];
	int glFrontViews[20];
	int glTopViews[20];
	int glSideViews[20];
	int glPerspectiveViews[20];
}

void GLDrawingController::init()
{
    int size = 0;
    char* t;
    glutInit(&size, &t);
    glutIsReady = true;

//    Window window;
//    window.resize(window.sizeHint());
//    int desktopArea = QApplication::desktop()->width() *
//                     QApplication::desktop()->height();
//    int widgetArea = window.width() * window.height();
//    if (((float)widgetArea / (float)desktopArea) < 0.75f)
//        window.show();
//    else
//        window.showMaximized();
}

void GLDrawingController::destroyAllWindows()
{
    for (int i = 0; i < totalGlWindows; ++i)
    {
        glutSetWindow(glWindows[i]);
        glutHideWindow();
        glutDestroyWindow(glWindows[i]);
    }
    totalGlWindows = 0;
}


void GLDrawingController::registerWindow(Drawing3D* drawingWindow, const char *windowTitle)
{
    qDebug() << "Registering main window";
	if (glutIsReady == false)
		init();

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(windowWidth, windowHeight);

    drawingWindow->windowIndex = totalGlWindows;
    glWindows[totalGlWindows] = glutCreateWindow(windowTitle);
    drawingWindows[totalGlWindows] = drawingWindow;
	
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glutDisplayFunc(GLDrawingController::display);
    glutReshapeFunc(GLDrawingController::reshape);
    glutIdleFunc(GLDrawingController::display);

	glWindowModes[totalGlWindows] = 0;
	++totalGlWindows;
}

void GLDrawingController::registerSecondaryWindow(Drawing3D* drawingWindow, enum windowPositions position)
{
	int mainWindow = glWindows[drawingWindow->windowIndex];

    switch (position)
    {
        case FRONT:
            glFrontViews[drawingWindow->windowIndex] = glutCreateSubWindow(mainWindow, windowBorder, 2 * windowBorder + subwindowHeight, subwindowWidth, subwindowHeight);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glutDisplayFunc(displayFront);
            break;
        case TOP:
            glTopViews[drawingWindow->windowIndex] = glutCreateSubWindow(mainWindow, windowBorder, windowBorder, subwindowWidth, subwindowHeight);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glutDisplayFunc(displayTop);
            break;
        case SIDE:
            glSideViews[drawingWindow->windowIndex] = glutCreateSubWindow(mainWindow, 2 * windowBorder + subwindowWidth, windowBorder, subwindowWidth, subwindowHeight);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glutDisplayFunc(displaySide);
            break;
        case PERSPECTIVE:
            glPerspectiveViews[drawingWindow->windowIndex] = glutCreateSubWindow(mainWindow, 2 * windowBorder + subwindowWidth, 2 * windowBorder + subwindowHeight, subwindowWidth, subwindowHeight);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glutDisplayFunc(displayPerspective);
            break;
    }

	// This will override drawing mode of parent window
	glWindowModes[drawingWindow->windowIndex] = 1;
}

void GLDrawingController::loadTextureWithOpenCV(cv::Mat &image, GLuint *text)
{
    if (image.empty()) return;
   
    glGenTextures(1, text);

    glBindTexture(GL_TEXTURE_2D, *text );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
    unsigned char *imageArray;
    imageArray = new unsigned char[image.cols * image.rows * 3];
    for (int y = 0; y < image.rows; ++y)
    {
        unsigned char *destinationImageRow = imageArray + 3 * y * image.cols;
        unsigned char *sourceImageRow = image.ptr(y);
        for (int x = 0; x < image.cols; ++x)
        {
            *(destinationImageRow + 3 * x) = *(sourceImageRow + 3 * x);
            *(destinationImageRow + 3 * x + 1) = *(sourceImageRow + 3 * x + 1);
            *(destinationImageRow + 3 * x + 2) = *(sourceImageRow + 3 * x + 2);
        }
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imageArray);
    delete[] imageArray;
}

void GLDrawingController::updateWindow(int windowIndex)
{
    glutMainLoopEvent();
    nextDisplayWindow = windowIndex;

    //qDebug() << "Windows to refresh: " << glWindows[windowIndex] << "," << glFrontViews[windowIndex] << "," << glTopViews[windowIndex] << "," << glSideViews[windowIndex] << "," << glPerspectiveViews[windowIndex];
    glutPostWindowRedisplay(glWindows[windowIndex]);
    if (glWindowModes[windowIndex] != 0) // Multi View
    {
        glutPostWindowRedisplay(glFrontViews[windowIndex]);
        glutPostWindowRedisplay(glTopViews[windowIndex]);
        glutPostWindowRedisplay(glSideViews[windowIndex]);
        glutPostWindowRedisplay(glPerspectiveViews[windowIndex]);
    }
    glutMainLoopEvent();
}

void GLDrawingController::display()
{
    GLDrawingController::drawingWindows[nextDisplayWindow]->updateDisplay();
}

void GLDrawingController::displayFront()
{
	GLDrawingController::drawingWindows[nextDisplayWindow]->updateDisplayFront();
}

void GLDrawingController::displayTop()
{
	GLDrawingController::drawingWindows[nextDisplayWindow]->updateDisplayTop();
}

void GLDrawingController::displaySide()
{
	GLDrawingController::drawingWindows[nextDisplayWindow]->updateDisplaySide();
}

void GLDrawingController::displayPerspective()
{
    GLDrawingController::drawingWindows[nextDisplayWindow]->updateDisplayPerspective();
}

void GLDrawingController::reshape(int width, int height)
{
//    windowWidth = width;
//    windowHeight = height;
//    glViewport(0, 0, (GLsizei)width, (GLsizei)height);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    gluPerspective(90, (GLfloat)width / (GLfloat)height, 1.0, 1000.0);
//    glMatrixMode(GL_MODELVIEW);
}

void GLThread::run()
{
    glThreadStatus = STARTED;
    while (glThreadStatus == STARTED)
    {
        // Display 3D reconstructions
        if (TrackingCommon::show3DReconstruction)
        {
            for (int i = 0; i < TrackingCommon::totalDrawing3D; ++i)
            {
                if (!TrackingCommon::drawings3D[i]->isViewCreated && TrackingCommon::drawings3D[i]->parametersReady)
                {
                    TrackingCommon::drawings3D[i]->createView();
                    TrackingCommon::drawings3D[i]->setTextures();
                    qDebug() << "View " << i << " created";
                }

                if (TrackingCommon::drawings3D[i]->isViewCreated)
                {
                    // Taking into account camera motion
                    //((cv::Mat) (TrackingCommon::objectZero * TrackingCommon::getCurrentCameraRotationMatrix())).copyTo(TrackingCommon::drawings3D[i]->referential);
                    // For fixed camera
                    TrackingCommon::objectZero.copyTo(TrackingCommon::drawings3D[i]->referential);
                    //TrackingCommon::objectZero.copyTo(TrackingCommon::drawings3D[i]->referential);
                    TrackingCommon::drawings3D[i]->setObjectCenter( ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->centerTransformAlternate);

                    // Testing
                    //TrackingCommon::drawings3D[i]->setObjectCenter( ((CuboidDrawing*)TrackingCommon::drawings3D[i])->cuboidTracker->homographyDecomposer.transform );
                    TrackingCommon::drawings3D[i]->refreshView();
                }

            }
        }
        msleep(TrackingCommon::glDelay);
    }
    GLDrawingController::destroyAllWindows();
}

void GLThread::setZeroSlot()
{
    for (int i = 0; i < TrackingCommon::totalDrawing3D; ++i)
    {
        TrackingCommon::drawings3D[i]->setZero();
    }
}
