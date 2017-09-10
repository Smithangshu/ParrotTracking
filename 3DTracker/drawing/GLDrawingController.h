#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Drawing3D.h"
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <QThread>

namespace GLDrawingController
{
	extern int totalGlWindows;
	extern bool glutIsReady;
	extern int nextDisplayWindow;
	enum windowPositions {FRONT, TOP, SIDE, PERSPECTIVE};
	extern int windowBorder;
	extern int windowWidth;
	extern int windowHeight;
	extern int subwindowWidth;
	extern int subwindowHeight;

	extern Drawing3D* drawingWindows[];
	extern int glWindows[];
	extern int glWindowModes[];
	extern int glFrontViews[];
	extern int glTopViews[];
	extern int glSideViews[];
	extern int glPerspectiveViews[];

	void registerWindow(Drawing3D* drawingWindow, const char *windowTitle);
	void registerSecondaryWindow(Drawing3D* drawingWindow, enum windowPositions position);
	void updateWindow(int windowIndex);
	void updateAllWindows();
    void init();
    void destroyAllWindows();

	void display();
	void displayFront();
	void displayTop();
	void displaySide();
	void displayPerspective();
	void reshape(int width, int height);
    void loadTextureWithOpenCV(cv::Mat &image, GLuint *text);
};

class GLThread : public QThread
{
    Q_OBJECT
public:
    enum GLThreadStatus {STARTED, STOPPED} glThreadStatus;
    void run();
public slots:
    void setZeroSlot();
};

#endif
