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
    int nPatternCenters;
    Grid grid(ui->spinBoxWidthPlay->value(), ui->spinBoxHeightPlay->value());
    nPatternCenters = grid.height*grid.width;
    if(nPatternCenters == 0){
        ui->plainTextEditLog->appendPlainText("Insert the Grid of Pattern Properly\n");
        return;
    }
    QString filename = QFileDialog::getOpenFileName(this, tr("choose"), "", tr("Images (*.avi *.mp4)"));
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
    std::vector<cv::Point3f> pointsRealImage;
    if(calibrated){
        float screenSpace = float(video.get(cv::CAP_PROP_FRAME_WIDTH))/float(grid.width);
        screenSpace = screenSpace < float(video.get(cv::CAP_PROP_FRAME_HEIGHT))/float(grid.height)? screenSpace: float(video.get(cv::CAP_PROP_FRAME_HEIGHT))/float(grid.height);
        for(int i = 0; i < grid.height; i++){
            for(int j = 0; j < grid.width; j++){
                pointsRealImage.push_back(cv::Point3f(float(j*screenSpace + screenSpace/2), float(i*screenSpace + screenSpace/2), 0.0f));
            }
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
    bool reassign = true;
    memset(idVector, -1, (nPatternCenters+20)*sizeof (int));
    cv::Size frameSize;
    cv::Mat frontoImg;
    while (keep) {
        video>>frame;
        if(!frame.empty()){
            if(nframes == 0)
                frameSize = frame.size();
            nframes++;
            //if(nframes % 120 == 0){
                double time = acumm/nframes;
                ui->labelTime->setText(QString::fromUtf8(std::to_string(time).c_str()));
                //acumm = 0.0;
            //}
            cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame;
            rowFrame = frame.clone();
            ProccessImage(frame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame, nPatternCenters, idVector, CentersPrev, reassign, acumm, szpromEllipse, Xmax, Ymax, Xmin, Ymin);
            if(calibrated){
                cv::Mat temp = frame.clone();
                cv::Mat frontoPL;
                cv::undistort(temp, frontoPL, MatrixCamera, DistCoeff);
                QImage qimgFP(frontoPL.data, frontoPL.cols, frontoPL.rows, frontoPL.step, QImage::Format_RGB888);
                pixmapThres.setPixmap(QPixmap::fromImage(qimgFP.rgbSwapped()));
                ui->graphicsViewThres->fitInView(&pixmapThres, Qt::KeepAspectRatio);

            }
            else {
                QImage qimgT(thresholdFrame.data, thresholdFrame.cols, thresholdFrame.rows, thresholdFrame.step, QImage::Format_Grayscale8);
                pixmapThres.setPixmap(QPixmap::fromImage(qimgT.rgbSwapped()));
                ui->graphicsViewThres->fitInView(&pixmapThres, Qt::KeepAspectRatio);
            }

            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            QImage qimgG(blurGaussFrame.data, blurGaussFrame.cols, blurGaussFrame.rows, blurGaussFrame.step, QImage::Format_Grayscale8);
            //QImage qimgT(thresholdFrame.data, thresholdFrame.cols, thresholdFrame.rows, thresholdFrame.step, QImage::Format_Grayscale8);
            QImage qimgP(integralFrame.data, integralFrame.cols, integralFrame.rows, integralFrame.step, QImage::Format_Grayscale8);
            //cv::imshow("Fr", frame);
            pixmapRow.setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
            pixmapGauss.setPixmap(QPixmap::fromImage(qimgG.rgbSwapped()));
            //pixmapThres.setPixmap(QPixmap::fromImage(qimgT.rgbSwapped()));
            pixmapPat.setPixmap(QPixmap::fromImage(qimgP.rgbSwapped()));
            ui->graphicsView->fitInView(&pixmapRow, Qt::KeepAspectRatio);
            if(calibrated && CentersPrev.size() == nPatternCenters && !reassign){
                //std::vector<cv::Point2f> centersOrd(nPatternCenters);
                //for(int tt = 0; tt < nPatternCenters; tt++){
                //    centersOrd[tt] = cv::Point2f(float(CentersPrev[idVector[tt]].x), float(CentersPrev[idVector[tt]].y));
                //}
                //cv::Mat homography = cv::findHomography(centersOrd, pointsRealImage);
                cv::Mat FrontParImg;
                if(ComputeFrontoParallel(rowFrame, MatrixCamera, DistCoeff, frameSize, nPatternCenters, grid, FrontParImg)){
                    frontoImg = FrontParImg.clone();
                }
                //cv::warpPerspective(inImage, FrontParImg, homography, frameSize);
                QImage qimgPers(FrontParImg.data, FrontParImg.cols, FrontParImg.rows, FrontParImg.step, QImage::Format_RGB888);
                pixmapGauss.setPixmap(QPixmap::fromImage(qimgPers.rgbSwapped()));
                ui->graphicsViewGauss->fitInView(&pixmapGauss, Qt::KeepAspectRatio);

            }
            else if (calibrated) {
                QImage qimgPers(frontoImg.data, frontoImg.cols, frontoImg.rows, frontoImg.step, QImage::Format_RGB888);
                pixmapGauss.setPixmap(QPixmap::fromImage(qimgPers.rgbSwapped()));
                ui->graphicsViewGauss->fitInView(&pixmapGauss, Qt::KeepAspectRatio);
            }
            else
                ui->graphicsViewGauss->fitInView(&pixmapGauss, Qt::KeepAspectRatio);
            //ui->graphicsViewThres->fitInView(&pixmapThres, Qt::KeepAspectRatio);
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


void MainWindow::on_btnCalibrate_clicked()
{
    bool isCamera = false;
    bool ok;
    int nPatternCenters;
    Grid grid(ui->spinBoxWidthCal->value(),ui->spinBoxHeightCal->value());
    nPatternCenters = grid.width*grid.height;
    float spaceValGrid =  float(ui->doubleSpinBoxSpaceCenters->value());
    int nIterationsCalibration = ui->spinBoxIt->value();

    int nFramesToCalibrate = 60;
    int getFrameMultipleBy = 50;


    nFramesToCalibrate = ui->spinBoxPatternsToCalibrate->value();

    if(nFramesToCalibrate == 0 || spaceValGrid < 0.01 || nPatternCenters == 0){
        ui->plainTextEditLog->appendPlainText(QString("Set Calibration parameters!!"));
        return;
    }
    QString filename = QFileDialog::getOpenFileName(this, tr("choose"), "", tr("Images (*.avi *mp4)"));
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

    ui->progressBarCalibrate->setMaximum(nFramesToCalibrate + 1);
    ui->progressBarCalibrate->setValue(0);
    std::vector<std::vector<cv::Vec2f>> CentersPatternsToCalibrate;
    std::vector<cv::Mat> framesCalibration;
    cv::Size frameSize;
    ui->plainTextEditLog->appendPlainText(QString(" - Loading Centers to Calibrate ...!"));
    while (CentersPatternsToCalibrate.size() < nFramesToCalibrate) {
        video>>frame;
        if(!frame.empty()){
            if(nframes == 0)
                frameSize = frame.size();
            nframes++;
            //if(nframes % 120 == 0){
                double time = acumm/nframes;
                ui->labelTime->setText(QString::fromUtf8(std::to_string(time).c_str()));
                //acumm = 0.0;
            //}
            cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame;
            rowFrame = frame.clone();
            ProccessImage(frame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame, nPatternCenters, idVector, CentersPrev, reassign, acumm, szpromEllipse, Xmax, Ymax, Xmin, Ymin);

            if(CentersPrev.size() != nPatternCenters)
                nfails++;
            float accurc = (float)(nframes - nfails)*100.0/(float)nframes;
            ui->labelAccuracy->setText(QString::fromUtf8(std::to_string(accurc).c_str()));
            if(!reassign && nframes % getFrameMultipleBy == 0){

                framesCalibration.push_back(rowFrame);
                cv::imwrite("im2.png", rowFrame);
                std::vector<cv::Vec2f> centersTemp(0);
                for(int ppp = 0; ppp < nPatternCenters; ppp++){
                    centersTemp.push_back(cv::Vec2f((float)CentersPrev[idVector[ppp]].x, (float)CentersPrev[idVector[ppp]].y));
                }
                CentersPatternsToCalibrate.push_back(centersTemp);
                // Draw the ChessBoard!
                //for(int ttt = 0; ttt < centersTemp.size(); ttt++){
                //    cv::putText(frame, std::to_string(ttt), cv::Point(centersTemp[ttt].val[0], centersTemp[ttt].val[1]), 1, 2, cv::Scalar(255, 0, 0), 2, 8);
                //}

                /*for(int m = 0; m < CenterPoints.size(); m++){
                       cv::putText(rowFrame, std::to_string(m), CenterPoints[idVector[m]], 1, 2, cv::Scalar(255, 0, 0), 2, 8);
                }*/
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
                ui->progressBarCalibrate->setValue(CentersPatternsToCalibrate.size());
            }
            //ui->graphicsView->scene()->addItem(&pixmap);
            cv::waitKey(0);
        }
        else{
            keep = false;
        }
        if(CentersPatternsToCalibrate.size() == nFramesToCalibrate)
            ui->plainTextEditLog->appendPlainText(QString(" - Calibrating ...!"));
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat distCoeff = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    std::string logComm;

    double rms = RunIterativeCameraCalibration(framesCalibration, CentersPatternsToCalibrate, frameSize, cameraMatrix, distCoeff, rvecs, tvecs, grid, spaceValGrid, nIterationsCalibration, nPatternCenters, logComm);
    //double rms = RunCalibrateCamera(CentersPatternsToCalibrate, frameSize, cameraMatrix, distCoeff, rvecs, tvecs, grid, spaceValGrid);
    ui->progressBarCalibrate->setValue(nFramesToCalibrate + 1);
    ui->plainTextEditLog->appendPlainText(QString(logComm.c_str()));
    ui->plainTextEditLog->appendPlainText(QString(" - Parameters Obtained Successfully!\n"));
    ui->plainTextEditLog->appendPlainText(QString("RMS = ") + QString::number(rms) + QString("\n"));
    ui->plainTextEditLog->appendPlainText(QString("Fx = ") + QString::number(cameraMatrix.at<double>(0,0)));
    ui->plainTextEditLog->appendPlainText(QString("Fy = ") + QString::number(cameraMatrix.at<double>(1,1)));
    ui->plainTextEditLog->appendPlainText(QString("Cx = ") + QString::number(cameraMatrix.at<double>(0,2)));
    ui->plainTextEditLog->appendPlainText(QString("Cy = ") + QString::number(cameraMatrix.at<double>(1,2)));

    MatrixCamera = cameraMatrix;
    DistCoeff = distCoeff;
    RVecs = rvecs;
    TVecs = tvecs;
    calibrated = true;

    /*std::cout<<"RMS> "<<rms<<std::endl;
    for ( int ii=0;ii<3;ii++) {
        for ( int jj=0;jj<3;jj++) {
            std::cout<<cameraMatrix.at<double>(ii,jj)<<" ";
        }
        std::cout<< std::endl;
    }
    int x = 21;
    */
}
