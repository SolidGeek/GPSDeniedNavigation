QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

#openCV
INCLUDEPATH += /usr/local/include/opencv2

LIBS += -L/usr/local/lib/aarch64-linux-gnu
LIBS += -lopencv_core
LIBS += -lopencv_dnn
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_highgui
LIBS += -lopencv_videoio
LIBS += -lopencv_cudaimgproc
LIBS += -lopencv_cudawarping
LIBS += -lopencv_cudaarithm
LIBS += -lopencv_cudafilters

#Project
SOURCES += main.cpp \
    serial.cpp

HEADERS += serial.h
