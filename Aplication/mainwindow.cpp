#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "processimage.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    ui->graphicsView->setScene(new QGraphicsScene(this));
    ui->graphicsView->scene()->addItem(&pixmapRow);
    ui->graphicsViewGauss->setScene(new QGraphicsScene(this));
    ui->graphicsViewGauss->scene()->addItem(&pixmapGauss);
    ui->graphicsViewThres->setScene(new QGraphicsScene(this));
    ui->graphicsViewThres->scene()->addItem(&pixmapThres);
    ui->graphicsViewPat->setScene(new QGraphicsScene(this));
    ui->graphicsViewPat->scene()->addItem(&pixmapPat);
}
void MainWindow::closeEvent(QCloseEvent * event){
    if(video.isOpened()){
        QMessageBox::warning(this, "warning","Stop the video before closing the app");
        event->ignore();
    }
    else
        event->accept();
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::executeTask(){

}
void MainWindow::OpenCamera(){
    bool isCamera;
    QString filename = QFileDialog::getOpenFileName(this, tr("choose"), "", tr("Images (*.avi)"));
    int cameraIndex = ui->videoEdit->text().toInt(&isCamera);
    if(isCamera){
        if(!video.open(cameraIndex)){
            QMessageBox::critical(this, "Camera error", "Make Sure you entered a correct Camera index");
            return;
        }
    }
    else{
        if(!video.open(filename.toStdString())){
            QMessageBox::critical(this, "Video Error opening", "Make sure you entered a video supported by opencv");
            return;
        }
    }
    // Video Processing
    cv::Mat frame;
    bool keep = true;
    double acumm = 0.0;
    unsigned long counter = 0;
    while (keep) {
        video>>frame;
        if(!frame.empty()){
            counter++;
            if(counter % 120 == 0){
                double time = acumm/120.0;
            }
            cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame;
            ProccessImage(frame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame, 30, acumm);
            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            QImage qimgG(blurGaussFrame.data, blurGaussFrame.cols, blurGaussFrame.rows, blurGaussFrame.step, QImage::Format_Grayscale8);
            QImage qimgT(thresholdFrame.data, thresholdFrame.cols, thresholdFrame.rows, thresholdFrame.step, QImage::Format_Grayscale8);
            QImage qimgP(integralFrame.data, integralFrame.cols, integralFrame.rows, integralFrame.step, QImage::Format_Grayscale8);
            //cv::imshow("Fr", frame);
            pixmapRow.setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
            pixmapGauss.setPixmap(QPixmap::fromImage(qimgG.rgbSwapped()));
            pixmapThres.setPixmap(QPixmap::fromImage(qimgT.rgbSwapped()));
            pixmapPat.setPixmap(QPixmap::fromImage(qimgP.rgbSwapped()));
            ui->graphicsView->fitInView(&pixmapRow, Qt::KeepAspectRatio);
            ui->graphicsViewGauss->fitInView(&pixmapGauss, Qt::KeepAspectRatio);
            ui->graphicsViewThres->fitInView(&pixmapThres, Qt::KeepAspectRatio);
            ui->graphicsViewPat->fitInView(&pixmapPat, Qt::KeepAspectRatio);

            //ui->graphicsView->scene()->addItem(&pixmap);
            cv::waitKey(0);
        }
        else{
            keep = false;
        }
    }
}

void MainWindow::on_pushButton_clicked()
{
    OpenCamera();
}
