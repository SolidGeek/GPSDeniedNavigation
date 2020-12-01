QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

#openCV
INCLUDEPATH += /usr/local/include/opencv4 \
    mavlink/include/c_library_v2/common \
    src \

LIBS += -L/usr/local/lib/aarch64-linux-gnu
LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_dnn


LIBS += -lopencv_videoio


#Project
SOURCES += main.cpp \
    serial.cpp \
    camera.cpp \
    telemetry.cpp \
    src/feature_tracker.cpp \
    src/visual_odemetry.cpp \
    timing.cpp

HEADERS += serial.h \
    camera.h \
    telemetry.h \
    src/feature_tracker.h \
    src/visual_odemetry.h \
    timing.h
