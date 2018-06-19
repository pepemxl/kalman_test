#-------------------------------------------------
#
# Project created by QtCreator 2018-06-05T16:02:13
#
#-------------------------------------------------

QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Kalman_test
TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv

LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_videoio -lopencv_imgproc -lopencv_calib3d -lopencv_features2d -lopencv_xfeatures2d \
        -lopencv_flann -lopencv_stitching -lopencv_objdetect -lopencv_video -lopencv_tracking \
        -lopencv_cudawarping -lopencv_cudacodec -lopencv_cudabgsegm -lopencv_cudastereo -lopencv_cudalegacy -lopencv_cudaobjdetect -lopencv_cudaarithm -fopenmp


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

