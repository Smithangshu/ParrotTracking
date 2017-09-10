#include "ImageWindow.h"
#include "ui_ImageWindow.h"
#include <QImage>
#include <QDebug>
#include <QVBoxLayout>
#include <QMouseEvent>

ImageWindow::ImageWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageWindow)
{
    ui->setupUi(this);
    imageLabel = new ImageLabel();
    imageLabel->setMouseTracking(true);
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setMargin(0);
    mainLayout->addWidget(imageLabel);
    this->setLayout(mainLayout);
    WindowCallBack* windowCallBack = new WindowCallBack();
    imageLabel->setWindowCallback(windowCallBack);
}

ImageWindow::~ImageWindow()
{
    delete ui;
}

void ImageWindow::displayImage(const QImage &image)
{
    // display on label
    if (image.format() == QImage::Format_Mono)
    {
        imageLabel->setPixmap(QPixmap::fromImage(image));
    }
    else
    {
        imageLabel->setPixmap(QPixmap::fromImage(image.rgbSwapped(), Qt::AutoColor));
    }
    // resize the label and form to fit the image
    this->resize(imageLabel->pixmap()->size());
    imageLabel->resize(imageLabel->pixmap()->size());
}

void ImageWindow::setWindowCallback(WindowCallBack *windowCallback)
{
    imageLabel->setWindowCallback(windowCallback);
}

int ImageLabel::constraintMouseX(int x)
{
    int newX = x;

    if (x > this->width())
        newX = this->width();
    else if (x < 0)
        newX = 0;

    return newX;
}

int ImageLabel::constraintMouseY(int y)
{
    int newY = y;

    if (y > this->height())
        newY = this->height();
    else if (y < 0)
        newY = 0;

    return newY;
}

void ImageLabel::mousePressEvent(QMouseEvent *event)
{
    qDebug() << "Mouse pressed at " << event->x() << ", " << event->y();
    windowCallback->mousePress(constraintMouseX(event->x()), constraintMouseY(event->y()));
}

void ImageLabel::mouseMoveEvent(QMouseEvent *event)
{
    windowCallback->mouseMove(constraintMouseX(event->x()), constraintMouseY(event->y()));
}

void ImageLabel::mouseReleaseEvent(QMouseEvent *event)
{
    qDebug() << "Mouse released at " << event->x() << ", " << event->y();
    windowCallback->mouseRelease(constraintMouseX(event->x()), constraintMouseY(event->y()));
}

void ImageLabel::setWindowCallback(WindowCallBack *windowCallback)
{
    qDebug() << "Call back set";
    this->windowCallback = windowCallback;
}

