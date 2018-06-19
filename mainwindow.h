#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QRect>
#include <QString>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QWidget>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlRecord>
#include <QSqlField>
#include <QSqlError>
#include <QEventLoop>

#include <iostream>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <ratio>
#include <iomanip>
#include <fstream>
#include <stack>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <vector>
#include <exception>


#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/cudacodec.hpp"
#include "opencv2/cudaimgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
//#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/core/gpumat.hpp"
//#include "opencv2/gpu/gpumat.hpp"
//#include "opencv2/gpu/gpu.hpp"
//#include "cvx_defs.h"
#include "opencv2/video/tracking.hpp"
#include <omp.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    int lSide;
    float radio;
    char code;
    bool flagStopAll;
    std::vector<bool> vectorFlagTestRunning;
    int currentNumberOfTest;
    bool flagStopTest01;
    bool flagStopTest02;
    unsigned int timeSleep;
    float media;
    float desviacionEstandar;
    float varianza;
    float dt;


    
    cv::KalmanFilter KF;
    double processNoiseCov=.0001;
    double measurementNoiseCov=.001;
    double errorCovPost=.01;

    void test_kalman_01();
    void udpateLabelTest01(cv::Mat &img);
    bool getFlagStopTest01() const;
    void setFlagStopTest01(bool value);

    char getCode() const;
    void setCode(char value);

    unsigned int getTimeSleep() const;
    void setTimeSleep(unsigned int value);

    float getMedia() const;
    void setMedia(float value);

    float getVarianza() const;
    void setVarianza(float value);

    float getDesviacionEstandar() const;
    void setDesviacionEstandar(float value);

    float getDt() const;
    void setDt(float value);

    int getLSide() const;
    void setLSide(int value);

    float getRadio() const;
    void setRadio(float value);

    bool getFlagStopAll() const;
    void setFlagStopAll(bool value);

    int getCurrentNumberOfTest() const;
    void setCurrentNumberOfTest(int value);

    bool getFlagStopTest02() const;
    void setFlagStopTest02(bool value);

    void udpateLabelTest02(cv::Mat &img);
    void test_kalman_02();
private slots:
    void on_pushButtonTest01_clicked();

    void on_spinBoxTimeSleep_valueChanged(int arg1);

    void on_doubleSpinBoxMedia_valueChanged(double arg1);

    void on_doubleSpinBoxDesviacionEstandar_valueChanged(double arg1);

    void on_doubleSpinBoxDt_valueChanged(double arg1);

    void on_doubleSpinBoxRadio_valueChanged(double arg1);

    void on_pushButtonTest02_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
