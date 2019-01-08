#ifndef PROCESSIMAGE_H
#define PROCESSIMAGE_H
# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>
# include <queue>
# include <string.h>
# include <stdlib.h>
# include <chrono>
//# include "IntegrarThreshold.hpp"
# include "bilinealtransform.h"
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

    int S = MAX(nRows, nCols)/16;
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
struct indxval{
    int index;
    float dist;
    indxval(int in, float v):index(in), dist(v){}
};
float distanceRectToPoint(float A, float B, float C, cv::Point P1){
    float den = A*P1.x + B*P1.y + C;
    float num = sqrtf(A*A + B*B);

    return den/num;
}
void SortVector(std::vector<indxval> & vec){
    int i = 0;
    while(i < vec.size()){
        float x = vec[i].dist;
        int j = i - 1;
        while (j >= 0 ) {
            if(vec[j].dist > x){
                vec[j + 1] = vec[j];
                j = j - 1;
            }
        }
        vec[j + 1] = vec[i];
        i = i + 1;
    }
}

bool ReassingIdx(int * idVector, std::vector<cv::Point> CenterPoints, int nPatternCenters, cv::Mat & im, float eps){
    if(CenterPoints.size() != nPatternCenters)
        return false;

    // Step 1:
    // Get the Index of Points that define the MBB
    int p1 = -1, p2 = -1, p3 = -1, p4 = -1;
    int epsilon = 5;
    int xmax = 0, xmin = 3000, ymax = 0, ymin = 3000;
    int yofxmax = 0, yofxmin = 3000, xofymax = 0, xofymin = 3000;
    /*
     * (p4) *****************  (p3)
     *     *               *
     *    *               *  ymin
     *   ***************** (p2)
     *  (p1)
     *
     */
    cv::RotatedRect rect = cv::minAreaRect(CenterPoints);
    cv::Point2f rec_points[4];
    rect.points(rec_points);

    for(size_t j = 0; j < 4; j++){
        cv::line(im, rec_points[j], rec_points[(j+1)%4], cv::Scalar(255,0,0), 4, 8);
    }
    int position[4] = {-1, -1, -1, -1};
    //float eps = 20.0;
    //int maxnComponents = 20;
    //std::vector<std::priority_queue<int>> indexPossible(4);
    for(int k = 0; k < 4; k++){
        cv::Point P1 = rec_points[k];
        double minDistance = 1000.0;

        for(int i = 0; i < CenterPoints.size(); i++){
            double distance = cv::norm(P1 - CenterPoints[i]);
            if(distance < minDistance && distance < eps){
                position[k] = i;
                minDistance = distance;
                //minDistance = distance;
                //indexPossible[k].push(i);
                //if(indexPossible[k].size() > maxnComponents)
                //    indexPossible[k].pop();
            }
        }
    }
    for(int k = 0; k < 4; k++){
        if(position[k] == -1){
            int idx1 = (k + 1)%4, idx2 = (k + 3)%4;
            if(position[idx1] == -1 || position[idx2] == -1)
                return false;
            float pend = (float)(CenterPoints[position[idx2]].y - CenterPoints[position[idx1]].y)/(float)(CenterPoints[position[idx2]].x - CenterPoints[position[idx1]].x);
            float c = (float)(CenterPoints[position[idx2]].y - pend*CenterPoints[position[idx2]].x);
            cv::line(im, CenterPoints[position[idx2]], CenterPoints[position[idx1]], cv::Scalar(255,0,255), 4, 8);

            std::cout<<"y = "<<pend<<" x + " <<c<<std::endl;
            float maxd = 0.0;

            for(int m = 0; m < CenterPoints.size(); m++){
                cv::Point P1 = CenterPoints[m];
                float di = distanceRectToPoint(pend, -1.0, c, P1);
                if(di > maxd){
                    position[k] = m;
                    maxd = di;
                }
            }
            float pend2 = -1/pend;
            cv::Point Pm = CenterPoints[position[k]];
            float c2 = Pm.y - pend2*Pm.x;
            cv::Point intersection = cv::Point((c - c2)/(pend2 - pend), pend*(c - c2)/(pend2-pend) + c);
            cv::line(im, Pm, intersection, cv::Scalar(255,255,0), 4, 8);

            if(position[(k + 2)%4] == -1 || position[(k + 2)%4] == position[k]){
                float mind = 10000.0;
                for(int m = 0; m < CenterPoints.size(); m++){
                    cv::Point P1 = CenterPoints[m];
                    float di = distanceRectToPoint(pend, -1.0, c, P1);
                    if(di < mind){
                        position[(k + 2)%4] = m;
                        mind = di;
                    }
                }
            }
            break;
        }
    }
    cv::Point centerRect = rect.center;
    /*for(int k = 0; k < 4; k++){
        double maxCenterDistance = 0.0;
        for(int i = 0; i < indexPossible[k].size(); i++){
            int idx = indexPossible[k].top();
            double distCenter = cv::norm(centerRect - CenterPoints[idx]) ;
            cv::circle(im, CenterPoints[idx], 5, cv::Scalar(80*k, 0, 80*k), 4, 8);
            if(distCenter > maxCenterDistance){
                maxCenterDistance = distCenter;
                //position[k] = idx;
            }
            indexPossible[k].pop();
        }
    }*/

    for(size_t j = 0; j < 4; j++){
        cv::line(im, CenterPoints[position[j]], CenterPoints[position[(j+1)%4]], cv::Scalar(0,0,255), 4, 8);
    }
    return true;

}
int ccc = 0;
void ProccessImage(cv::Mat & rowFrame, cv::Mat & grayRowFrame, cv::Mat & blurGaussFrame, cv::Mat & thresholdFrame, cv::Mat & integralFrame, int nPatternCenters, int * idVector, std::vector<cv::Point> & CentersPrev, bool & reassign ,double & acumTime, float & szpromEllipse, int & Xmax, int & Ymax, int & Xmin, int & Ymin){
    auto t1 = std::chrono::high_resolution_clock::now();
    int PatternSIZE = 60;
    double epsilon = 2.0;

    //    epsilon = 4.0;
    // Epsilon Bounding box comparison with previous bounding box, must be scaled
    double epsilonBB = 60.0; //Second Pattern: 80, First pattern: 85
    //epsilon to compute corners in reassingCorners Function
    float eps = 20.0;
    // Epsilon for diferences of Sizes:
    float epsilonSZEl = 95.0; //Second Pattern, Fist Pattern: 80, 95 /obs 55 or 95 of this parameter not provide difference
    if (szpromEllipse > 17){
        epsilon = 3.0;
        epsilonBB = 45;
        eps = 30.0;
    }
    else if (szpromEllipse < 9) {
        epsilonBB = 20;
        eps = 16.0;
    }
    else{
        epsilon = 2.0;
        epsilonBB = 45;
        eps = 20.0;
    }
    int ncicles = 120;

    //float szpromEllipse = 1000.0;
    //Define the bounding box
    //int Xmax = 1000.0;
    //int Ymax = 1000.0;
    //int Xmin = 0.0;
    //int Ymin = 0.0;
    //std::cout<<szpromEllipse<<std::endl;
    //std::vector<cv::Point> CentersPrev;


    //bool reassign = false;

    // Convert to Gray Scale
    cv::cvtColor(rowFrame, grayRowFrame, CV_RGB2GRAY);
    // Apply an Gaussian blur filter
    GaussianBlur( grayRowFrame, blurGaussFrame, cv::Size( 5, 5 ), 0, 0 );
    cv::threshold(blurGaussFrame, thresholdFrame, 100, 255, CV_THRESH_BINARY);
    blurGaussFrame.copyTo(integralFrame);
    //cv::adaptiveThreshold(blurGaussFrame, integralFrame, 125, cv::ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 11, 12);
    // Apply an Integral threshold to the blured image
    thresholdIntegral(blurGaussFrame, integralFrame);

    // Find contours!
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(integralFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    // First Heuristic, avoid all contours with no father and childs
    std::vector<int> points;    // Store the index of possible contours.
    /*
     * [{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
     */
    for(int a = 0; a < contours.size(); a++){
        if((hierarchy[a][2] >= 0 || hierarchy[a][3] >= 0) && contours[a].size() > 5)
            points.push_back(a);
    }

    // To store index of contours after apply the heuristics
    std::vector<int> points2;
    // Initialize the MBB
    // Crazy values, will be change in first execution
    int xmin = 3000;
    int ymin = 3000;
    int xmax = 0;
    int ymax = 0;
    // For heuristic of weighted size of elipse.
    double szprom = 0.0;

    // To avoid to compute more than once the 'fitEllipseMethod'
    bool isElipseComputed[contours.size()];
    memset(isElipseComputed, false, contours.size()*sizeof (bool));

    // The ellipse of Each contour, not necessariment will be calculated all ellipses.
    std::vector<cv::RotatedRect> minEllipse( contours.size() );

    // To Store the Center of each Pattern
    std::vector<cv::Point> CenterPoints;

    // Iterate over all possible contours.
    for(int a = 0; a < points.size(); a++){
        int poschild = points[a];
        // [2] == -1 ? means that contour doesn't have child
        if(hierarchy[poschild][2] == -1 ){
            // Compute child Ellipse if it is not computed yet.
            if(!isElipseComputed[poschild] && contours[poschild].size() > 5){
                minEllipse[poschild] = cv::fitEllipse(cv::Mat(contours[poschild]));
                isElipseComputed[poschild] = true;
            }
            // compute father Ellipse if it is not computed yet.
            int posfather = hierarchy[points[a]][3];
            if(!isElipseComputed[posfather] && contours[posfather].size() > 5){
                minEllipse[posfather] = cv::fitEllipse(cv::Mat(contours[posfather]));
                isElipseComputed[posfather] = true;
            }
            // Heuristic: Is the center of child, near to center of father?
            double val = cv::norm(minEllipse[poschild].center - minEllipse[posfather].center);
            if(val < epsilon){
                // Ponderate size of father.
                double currsz = (minEllipse[posfather].size.height + minEllipse[posfather].size.width)/2.0;
                double curMinSZ = (minEllipse[poschild].size.height + minEllipse[poschild].size.width)/2.0;
                // Heuristic: if size of father is too bigger in comparizon to child. 2.9 setted experimentally
                if(currsz/curMinSZ < 2.9){
                    // Heuristic: If Size of father ellipse increasse dramatically in comparison with previous frame.
                    if(currsz - szpromEllipse < epsilonSZEl ){
                        //Heuristic: M. Bounding Box Heuristic:
                        cv::Point curPoint = minEllipse[poschild].center;
                        if((curPoint.x - Xmax < epsilonBB) &&(Xmin - curPoint.x < epsilonBB) && (curPoint.y - Ymax < epsilonBB) && (Ymin - curPoint.y < epsilonBB) ){
                            // Here it is the centerpoint of pattern.
                            points2.push_back(poschild);
                            points2.push_back(posfather);
                            // Save the Center point
                            CenterPoints.push_back(minEllipse[poschild].center);
                            // Compute the MBB of Current Frame;
                            if(minEllipse[poschild].center.x < xmin){
                                xmin = minEllipse[poschild].center.x;
                            }
                            if(minEllipse[poschild].center.x > xmax)
                                xmax = minEllipse[poschild].center.x;
                            if(minEllipse[poschild].center.y < ymin)
                                ymin = minEllipse[poschild].center.y;
                            if(minEllipse[poschild].center.y > ymax)
                                ymax = minEllipse[poschild].center.y;

                            // Acumm Size of ellipse prom
                            //szprom += (minEllipse[poschild].size.height + minEllipse[poschild].size.width)/2.0;
                            szprom += currsz;
                        }

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
        if(ReassingIdx(idVector, CenterPoints, nPatternCenters, rowFrame, eps)){
            std::string s= "Exim";
            s.append(std::to_string(ccc));
            s.append(".png");
            ccc++;
            cv::imwrite(s, rowFrame);
            reassign = false;
        }
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
    //cv::rectangle(rowFrame, cv::Point(Xmin, Ymin), cv::Point(Xmax, Ymax), cv::Scalar(0, 0, 255), 4, 8);
    for(int k = 0; k < points2.size(); k++){
                //boundRect[points2[k]] = cv::minAreaRect(cv::Mat(contours[points2[k]]));
                //if( contours[points2[k]].size() > 5 )
                //    minEllipse[points2[k]] = cv::fitEllipse( cv::Mat(contours[points2[k]]) );

                cv::ellipse( rowFrame, minEllipse[points2[k]], cv::Scalar(0,255,0), 2, 8 );
                //cv::putText(rowFrame, std::to_string(k), CenterPoints[idVector[k]], 1, 2, cv::Scalar(255, 0, 0));
                // Uncomment bellow to drawBounding boxes
     }
    for(int m = 0; m < CenterPoints.size(); m++){
           cv::putText(rowFrame, std::to_string(m), CenterPoints[idVector[m]], 1, 2, cv::Scalar(255, 0, 0), 2, 8);
    }
    cv::putText(rowFrame, std::to_string(points2.size()), cv::Point(50, 20), 1, 2, cv::Scalar(0, 0, 255), 2, 8);
}

#endif // PROCESSIMAGE_H
