HEADERS += \
    3rdparty/qparrot/Parrot.h

SOURCES +=\
    3rdparty/qparrot/Parrot.cpp

win32 {

    HEADERS += \
        3rdparty/qparrot/CHeliWin.h

    SOURCES +=\
        3rdparty/qparrot/CHeliWin.cpp
}

unix {
    HEADERS += \
        3rdparty/qparrot/Parrot.h \
        3rdparty/qparrot/CHeli.h \
        3rdparty/qparrot/app.h \
        3rdparty/qparrot/vlib.h \
        3rdparty/qparrot/video.h \
        3rdparty/qparrot/CRecognition.h \
        3rdparty/qparrot/CRawImage.h \
        3rdparty/qparrot/CGui.h

    SOURCES += \
        3rdparty/qparrot/CHeli.cpp \
        3rdparty/qparrot/app.cpp \
        3rdparty/qparrot/vlib.cpp \
        3rdparty/qparrot/video.cpp \
        3rdparty/qparrot/stream.cpp \
        3rdparty/qparrot/navdata.cpp \
        3rdparty/qparrot/default.cpp \
        3rdparty/qparrot/CRecognition.cpp \
        3rdparty/qparrot/CRawImage.cpp \
        3rdparty/qparrot/at_cmds.cpp \
        3rdparty/qparrot/CGui.cpp
}

LIBS += -lSDL
