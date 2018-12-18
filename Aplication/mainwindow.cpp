#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "processimage.h"
#include <string>
#include <QString>
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
    /*if(video.isOpened()){
        QMessageBox::warning(this, "warning","Stop the video before closing the app");
        event->ignore();
    }
    else
        event->accept();*/
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::executeTask(){

}
void MainWindow::OpenCamera(){
    bool isCamera = false;
    bool ok;
    int nPatternCenters = 12;
    int ncenters = ui->textnCenters->toPlainText().toInt(&ok);
    if(ok)
        nPatternCenters = ncenters;
    else{
        ui->plainTextEditLog->appendPlainText("Insert a integer number of Centers\n");
        return;
    }
    QString filename = QFileDialog::getOpenFileName(this, tr("choose"), "", tr("Images (*.avi)"));
    int cameraIndex = 0; //ui->videoEdit->text().toInt(&isCamera);

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
    unsigned long nframes = 0;
    unsigned long nfails = 0;

    int idVector[nPatternCenters+20];
    std::vector<cv::Point> CentersPrev;
    float szpromEllipse = 1000.0;
    //Define the bounding box
    int Xmax = 1000.0;
    int Ymax = 1000.0;
    int Xmin = 0.0;
    int Ymin = 0.0;
    bool reassign = false;
    memset(idVector, -1, (nPatternCenters+20)*sizeof (int));
    while (keep) {
        video>>frame;
        if(!frame.empty()){
            nframes++;
            //if(nframes % 120 == 0){
                double time = acumm/nframes;
                ui->labelTime->setText(QString::fromUtf8(std::to_string(time).c_str()));
                //acumm = 0.0;
            //}
            cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame;
            ProccessImage(frame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame, nPatternCenters, idVector, CentersPrev, reassign, acumm, szpromEllipse, Xmax, Ymax, Xmin, Ymin);
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
            if(CentersPrev.size() != nPatternCenters)
                nfails++;
            float accurc = (float)(nframes - nfails)*100.0/(float)nframes;
            ui->labelAccuracy->setText(QString::fromUtf8(std::to_string(accurc).c_str()));
            //ui->graphicsView->scene()->addItem(&pixmap);
            cv::waitKey(0);
        }
        else{
            keep = false;
        }
    }
    ui->plainTextEditLog->appendPlainText(QString::fromUtf8(std::to_string(nframes).c_str()));
    ui->plainTextEditLog->appendPlainText("Finished Successfully!");

}

void MainWindow::on_pushButton_clicked()
{

    OpenCamera();
}
