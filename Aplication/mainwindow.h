#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
// QGraphics View
#include <QtDebug>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <QMessageBox>
#include <QFileDialog>
#include "omp.h"

#include "opencv2/opencv.hpp"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    void OpenCamera();
    bool SelectFrames(cv::VideoCapture video, std::vector<cv::Mat> & frames, std::vector<std::vector<cv::Vec2f>> & CentersPatternsToCalibrate, cv::Size imResolution, const int numFrames, int nPatternCenters);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent * event);

private slots:
    void on_pushButton_clicked();

    void on_btnCalibrate_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsPixmapItem pixmapRow;
    QGraphicsPixmapItem pixmapGauss;
    QGraphicsPixmapItem pixmapThres;
    QGraphicsPixmapItem pixmapPat;
    cv::VideoCapture video;
    cv::Mat MatrixCamera;
    cv::Mat DistCoeff;
    std::vector<cv::Mat> RVecs;
    std::vector<cv::Mat> TVecs;
    bool calibrated = false;
};

#endif // MAINWINDOW_H
