#-------------------------------------------------
#
# Project created by QtCreator 2012-02-24T08:39:52
#
#-------------------------------------------------

QT += core gui opengl phonon network

CONFIG += qwt

TARGET = 3DTracker
TEMPLATE = app

SOURCES += main.cpp\
    common/TrackingCommon.cpp \
    common/TestsController.cpp \
    common/MainController.cpp \
    common/PTZCameraController.cpp \
    common/KalmanFilter.cpp \
    drawing/PlaneDrawing.cpp \
    drawing/GLDrawingController.cpp \
    drawing/Drawing3D.cpp \
    drawing/CuboidDrawing.cpp \
    gui/MainWindow.cpp \
    gui/WindowCallbacks.cpp \
    gui/DataDisplayer.cpp \
    gui/DataDisplayerWindow.cpp \
    gui/HomographyCaptureWindow.cpp \
    gui/ImageWindow.cpp \
    reconstruction/PositionEstimator.cpp \
    reconstruction/HomographyExtender.cpp \
    reconstruction/HomographyDecomposer.cpp \
    reconstruction/PositionEstimatorController.cpp \
    reconstruction/PlaneDetector.cpp \
    tracking/Tracker.cpp \
    tracking/MeanShiftWithScaling.cpp \
    tracking/MeanShiftTracker.cpp \
    tracking/LucasKanadeTracker.cpp \
    tracking/ESMTracker.cpp \
    tracking/CuboidTracker.cpp \
    tracking/CuboidTrackerSingleFace.cpp \
    tracking/ESMTrackerGPU.cpp \
    tracking/ESMTrackerPyramidal.cpp \
    tracking/ESMTrackerConstrained.cpp \
    tracking/ESMTrackerPyramidalFixed.cpp \
    models/ObjectModel.cpp \
    models/FeatureBasedModel.cpp \
    control/JoypadController.cpp \
    control/ParrotController.cpp \
    gui/PlottingController.cpp \
    tracking/CuboidTrackerMultipleFace.cpp \
    common/SocketsController.cpp

HEADERS  +=  common/calibration.h \
    common/TrackingCommon.h \
    common/TestsController.h \
    common/MainController.h \
    common/PTZCameraController.h \
    common/KalmanFilter.h \
    drawing/PlaneDrawing.h \
    drawing/GLDrawingController.h \
    drawing/Drawing3D.h \
    drawing/CuboidDrawing.h \
    gui/MainWindow.h \
    gui/WindowCallbacks.h \
    gui/DataDisplayer.h \
    gui/DataDisplayerWindow.h \
    gui/HomographyCaptureWindow.h \
    gui/ImageWindow.h \
    reconstruction/PositionEstimator.h \
    reconstruction/HomographyExtender.h \
    reconstruction/HomographyDecomposer.h \
    reconstruction/PositionEstimatorController.h \
    reconstruction/PlaneDetector.h \
    tracking/Tracker.h \
    tracking/MeanShiftWithScaling.h \
    tracking/MeanShiftTracker.h \
    tracking/LucasKanadeTracker.h \
    tracking/ESMTracker.h \
    tracking/CuboidTracker.h \
    tracking/CuboidTrackerSingleFace.h \
    tracking/ESMTrackerGPU.h \
    tracking/ESMTrackerPyramidal.h \
    tracking/ESMTrackerConstrained.h \
    tracking/ESMTrackerPyramidalFixed.h \
    models/ObjectModel.h \
    models/FeatureBasedModel.h \
    control/JoypadController.h \
    control/ParrotController.h \
    gui/PlottingController.h \
    drawing/GLWidget.h \
    tracking/CuboidTrackerMultipleFace.h \
    common/SocketsController.h

FORMS    += MainWindow.ui \
    DataDisplayerWindow.ui \
    HomographyCaptureWindow.ui \
    ImageWindow.ui

win32 {
    INCLUDEPATH += C:\3DTrackerLibraries\opencv\build\include
    INCLUDEPATH += C:\3DTrackerLibraries\freeglut\include
    INCLUDEPATH += C:\3DTrackerLibraries\glew\include
    INCLUDEPATH += C:\3DTrackerLibraries\SDL-1.2.15\include
    INCLUDEPATH += "$$(GLC_LIB_DIR)/include"

    LIBS += -LC:\3DTrackerLibraries\opencv\build\x86\vc10\lib
    LIBS += -LC:\3DTrackerLibraries\freeglut\lib
    LIBS += -LC:\3DTrackerLibraries\glew\lib
    LIBS += -lfreeglut -lglew32
    LIBS += -LC:\3DTrackerLibraries\SDL-1.2.15\lib\x86 -lSDL
    LIBS += -L"$$(GLC_LIB_DIR)/lib" -lGLC_lib2

    LIBS += -lopencv_core242d \
    -lopencv_highgui242d \
    -lopencv_calib3d242d \
    -lopencv_features2d242d \
    -lopencv_flann242d \
    -lopencv_imgproc242d \
    -lopencv_nonfree242d \
    -lopencv_video242d \
    -lopencv_gpu242d

    DEFINES += SDL_WIN
}

unix {
    LIBS += -lglut -lGL -lGLU -lSDL
    LIBS += -lopencv_core \
    -lopencv_highgui \
    -lopencv_calib3d \
    -lopencv_features2d \
    -lopencv_flann \
    -lopencv_imgproc \
    -lopencv_nonfree \
    -lopencv_video \
    -lopencv_gpu
}

include(3rdparty/qextserialport/src/qextserialport.pri)
include(3rdparty/qparrot/qparrot.pri)
