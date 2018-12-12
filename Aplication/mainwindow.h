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
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent * event);

private slots:
    void on_pushButton_clicked();
    void executeTask();

private:
    Ui::MainWindow *ui;
    QGraphicsPixmapItem pixmapRow;
    QGraphicsPixmapItem pixmapGauss;
    QGraphicsPixmapItem pixmapThres;
    QGraphicsPixmapItem pixmapPat;
    cv::VideoCapture video;
};

#endif // MAINWINDOW_H
