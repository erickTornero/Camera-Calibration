#ifndef PROCESSIMAGE_H
#define PROCESSIMAGE_H
# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>
# include <string.h>
# include <stdlib.h>
# include <chrono>
//# include "IntegrarThreshold.hpp"
#include <limits.h>
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;

    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);
    //cv::imshow("int", sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    int S = MAX(nRows, nCols)/8;
    //double T = 0.15;
    double T = 0.1;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;

    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;

        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for ( int j = 0; j < nCols; ++j)
        {
            x1 = j-s2;
            x2 = j+s2;

            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }

            count = (x2-x1)*(y2-y1);

            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 255;
            else
                p_outputMat[j] = 0;
        }
    }
}
void ProccessImage(cv::Mat & rowFrame, cv::Mat & grayRowFrame, cv::Mat & blurGaussFrame, cv::Mat & thresholdFrame, cv::Mat & integralFrame, int nPatternCenters, int * idVector, std::vector<cv::Point> & CentersPrev, bool & reassign ,double & acumTime, float & szpromEllipse, int & Xmax, int & Ymax, int & Xmin, int & Ymin){
    auto t1 = std::chrono::high_resolution_clock::now();
    int PatternSIZE = 60;
    double epsilon = 1.0;
    // Epsilon Bounding box comparison with previous bounding box, must be scaled
    double epsilonBB = 100.0;
    // Epsilon for diferences of Sizes:
    float epsilonSZEl = 60.0;
    int ncicles = 120;

    //float szpromEllipse = 1000.0;
    //Define the bounding box
    //int Xmax = 1000.0;
    //int Ymax = 1000.0;
    //int Xmin = 0.0;
    //int Ymin = 0.0;

    //std::vector<cv::Point> CentersPrev;


    //bool reassign = false;

    cv::cvtColor(rowFrame, grayRowFrame, CV_RGB2GRAY);
    GaussianBlur( grayRowFrame, blurGaussFrame, cv::Size( 5, 5 ), 0, 0 );
    cv::threshold(blurGaussFrame, thresholdFrame, 100, 255, CV_THRESH_BINARY);
    blurGaussFrame.copyTo(integralFrame);
    thresholdIntegral(blurGaussFrame, integralFrame);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(integralFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        //first pass, avoid all points with no father and childs
    std::vector<int> points;
    for(int a = 0; a < contours.size(); a++){
        if((hierarchy[a][2] >= 0 || hierarchy[a][3] >= 0) && contours[a].size() > 5)
            points.push_back(a);
    }

    std::vector<int> points2;
    //Get the MBB:
    int xmin = 3000;
    int ymin = 3000;
    int xmax = 0;
    int ymax = 0;
    double szprom = 0.0;
    bool isElipseComputed[contours.size()];
    memset(isElipseComputed, false, contours.size()*sizeof (bool));
    std::vector<cv::RotatedRect> minEllipse( contours.size() );
    std::vector<cv::Point> CenterPoints;

    for(int a = 0; a < points.size(); a++){
        int poschild = points[a];
        if(hierarchy[poschild][2] == -1 ){
            if(!isElipseComputed[poschild] && contours[poschild].size() > 5){
                minEllipse[poschild] = cv::fitEllipse(cv::Mat(contours[poschild]));
                isElipseComputed[poschild] = true;
            }
            int posfather = hierarchy[points[a]][3];
            if(!isElipseComputed[posfather] && contours[posfather].size() > 5){
                minEllipse[posfather] = cv::fitEllipse(cv::Mat(contours[posfather]));
                isElipseComputed[posfather] = true;
            }
            double val = cv::norm(minEllipse[poschild].center - minEllipse[posfather].center);
            if(val < epsilon){
                double currsz = (minEllipse[posfather].size.height + minEllipse[posfather].size.height)/2.0;
                if(currsz - szpromEllipse < epsilonSZEl ){
                    //Apply BB Heuristic:
                    cv::Point curPoint = minEllipse[poschild].center;
                    if((curPoint.x - Xmax < epsilonBB) &&(Xmin - curPoint.x < epsilonBB) && (curPoint.y - Ymax < epsilonBB) && (Ymin - curPoint.y < epsilonBB) ){
                        points2.push_back(poschild);
                        points2.push_back(posfather);
                        // Save the points in previous mesh.
                        CenterPoints.push_back(minEllipse[poschild].center);
                        // Compute current Mbb;
                        if(minEllipse[poschild].center.x < xmin){
                            xmin = minEllipse[poschild].center.x;
                        }
                        if(minEllipse[poschild].center.x > xmax)
                            xmax = minEllipse[poschild].center.x;
                        if(minEllipse[poschild].center.y < ymin)
                            ymin = minEllipse[poschild].center.y;
                        if(minEllipse[poschild].center.y > ymax)
                            ymax = minEllipse[poschild].center.y;

                        szprom += (minEllipse[poschild].size.height + minEllipse[poschild].size.height)/2.0;
                    }

                }
            }
        }
    }

    szpromEllipse = szprom/(float)points2.size();
    if(points2.size() != 0){
        Xmax = xmax;
        Xmin = xmin;
        Ymax = ymax;
        Ymin = ymin;
    }
    // Restart initial conditions
    if(points2.size() == 0){
        szpromEllipse = 1000.0;
        //Xmax = 1000.0;
        //Ymax = 1000.0;
        //Xmin = 0.0;
        //Ymin = 0.0;
    }

    bool indexesUses[CenterPoints.size()];
    memset(indexesUses, false, CenterPoints.size()*sizeof(bool));
    if(reassign && CenterPoints.size() == nPatternCenters){
        //Initialize ID
        for(int i = 0; i < nPatternCenters; i++){
            idVector[i] = i;
        }
        reassign = false;

    }
    else{
        for(int k = 0; k < CentersPrev.size(); k++){
            cv::Point P1 = CentersPrev[idVector[k]];
            double minDistance = 100.0;
            int position = 0;
            for(int i = 0; i < CenterPoints.size(); i++){
                double distance = cv::norm(P1 - CenterPoints[i]);
                if(distance < minDistance){
                    position = i;
                    minDistance = distance;
                }
            }
            if(!indexesUses[position]){
                idVector[k] = position;
                indexesUses[position] = true;
            }
            else
                reassign = true;
        }
    }
    CentersPrev = CenterPoints;
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    acumTime += (double)duration;
    cv::rectangle(rowFrame, cv::Point(Xmin, Ymin), cv::Point(Xmax, Ymax), cv::Scalar(0, 0, 255), 4, 8);
    for(int k = 0; k < points2.size(); k++){
                //boundRect[points2[k]] = cv::minAreaRect(cv::Mat(contours[points2[k]]));
                //if( contours[points2[k]].size() > 5 )
                //    minEllipse[points2[k]] = cv::fitEllipse( cv::Mat(contours[points2[k]]) );

                cv::ellipse( rowFrame, minEllipse[points2[k]], cv::Scalar(0,255,0), 2, 8 );
                cv::putText(rowFrame, std::to_string(k), CenterPoints[idVector[k]], 1, 2, cv::Scalar(255, 0, 0));
                // Uncomment bellow to drawBounding boxes
     }
    for(int m = 0; m < CenterPoints.size(); m++){
           cv::putText(rowFrame, std::to_string(m), CenterPoints[idVector[m]], 1, 2, cv::Scalar(255, 0, 0), 2, 8);
    }
    cv::putText(rowFrame, std::to_string(points2.size()), cv::Point(50, 20), 1, 2, cv::Scalar(0, 0, 255), 2, 8);
}

#endif // PROCESSIMAGE_H
