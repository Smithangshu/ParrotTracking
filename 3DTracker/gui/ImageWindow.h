#ifndef IMAGEWINDOW_H
#define IMAGEWINDOW_H

#include <QWidget>
#include <QLabel>
#include "opencv2/core/core.hpp"
#include "WindowCallbacks.h"

namespace Ui {
class ImageWindow;
}

class ImageLabel : public QLabel
{
private:
    WindowCallBack* windowCallback;
    int constraintMouseX(int x);
    int constraintMouseY(int y);
public:
    void setWindowCallback(WindowCallBack* windowCallback);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
};

class ImageWindow : public QWidget
{
    Q_OBJECT
    
public:
    explicit ImageWindow(QWidget *parent = 0);
    ~ImageWindow();
    void displayImage(const QImage &image);
    void setWindowCallback(WindowCallBack* windowCallback);

private:
    Ui::ImageWindow *ui;
    ImageLabel* imageLabel;
};

#endif // IMAGEWINDOW_H
