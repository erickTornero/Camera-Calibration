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

struct Grid;
bool GetCenterPoints(cv::Mat & frame, int nPatternCenters, std::vector<cv::Point2f> & Centers);
std::vector<cv::Point2f> RecomputeFrontoParallel(std::vector<cv::Point2f>, Grid );

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
struct Grid{
    int width;
    int height;
    Grid(int w, int h):width(w), height(h){}
    void set(int w, int h){
        width = w;
        height = h;
    }
};

/*
 *      Compute distance from Point to a Rect
 *      keep the sign of distance for future porpose
 */
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
void SortIndexes(const std::vector<cv::Point> & Centers, int * indexes, int sizeIndex = 4){

    int indMinY = 0;
    int minY = Centers[indexes[0]].y;

    for(int i = 1; i < 4;i++){
        if(Centers[indexes[i]].y < minY){
            indMinY = i;
            minY = Centers[indexes[i]].y;
        }
    }
    int tmp = indexes[indMinY];
    indexes[indMinY] = indexes[0];
    indexes[0] = tmp;
    // Compute the second minY
    int secondMinY = 1;
    int minY2 = Centers[indexes[1]].y;
    for(int i = 2; i < 4;i++){
        if(Centers[indexes[i]].y < minY2){
            secondMinY = i;
            minY2 = Centers[indexes[i]].y;
        }
    }
    tmp = indexes[secondMinY];
    indexes[secondMinY] = indexes[1];
    indexes[1] = tmp;

    if(Centers[indexes[0]].x > Centers[indexes[1]].x){
        tmp = indexes[0];
        indexes[0] = indexes[1];
        indexes[1] = tmp;
    }
    if(Centers[indexes[2]].x < Centers[indexes[3]].x){
        tmp = indexes[2];
        indexes[2] = indexes[3];
        indexes[3] = tmp;
    }
    /*
    while(i < sizeIndex){
        float x = Centers[indexes[i]].y;
        int curIdx = indexes[i];
        int j = i - 1;
        while (j >= 0 ) {
            if(Centers[indexes[j]].y > x){
                indexes[j + 1] = indexes[j];
            }
            j = j - 1;
        }
        indexes[j + 1] = curIdx;
        i = i + 1;
    }*/
    /*
    std::cout<<"Y ordered";
    for(int i = 0; i < 4; i++){
        std::cout<<"("<<Centers[indexes[i]].x<<", "<<Centers[indexes[i]].y<<") --";
    }
    std::cout<<std::endl;
    */
    /*
    int minY1Indx = 0, minY2Indx = 1;
    //find Y1:
    int minY1 = Centers[indexes[0]].y;
    for(int i = 1; i < sizeIndex; i++){
        if(Centers[indexes[i]].y < minY1){
            minY1Indx = i;
            minY1 = Centers[indexes[i]].y;
        }
    }*/
}

int getIndexTable(cv::Point p, int space, Grid grid, float errorlimit = 0.1f){
    float dx = (float) p.x/(float)space;
    float dy = (float) p.y/(float)space;

    float dxI = (float)(int(dx + 0.5));
    float dyI = (float)(int(dy + 0.5));
    float ex2 = (dxI - dx)*(dxI - dx);
    float ey2 = (dyI - dy)*(dyI - dy);
    if(sqrtf(ex2 + ey2) < errorlimit){
        int y = grid.width*int(dy + 0.5);
        int x = int(dx + 0.5);
        return x + y;
    }
    return -1;
}

bool ComputeIndexes(int * idVector, const std::vector<cv::Point> & CenterPoints, int nPatternCenters, float * coefX, float * coefY, Grid grid){
    bool indexesUsed[nPatternCenters];
    memset(indexesUsed, false, nPatternCenters*sizeof (bool));
    //auto t2 = std::chrono::high_resolution_clock::now();
    for(int kk = 0; kk < CenterPoints.size(); kk++){
        float dd[4] = {(float)CenterPoints[kk].x, (float)CenterPoints[kk].y, (float)CenterPoints[kk].y*CenterPoints[kk].x, 1.0};
        cv::Point PTransform(DotProduct(coefX, dd, 4),DotProduct(coefY, dd, 4));

        int indexCenter = getIndexTable(PTransform, 20, grid, 0.5f);
        //std::cout<<"Map->"<<" ("<<CenterPoints[kk].x<<", "<<CenterPoints[kk].y<<") --> "<<"( "<<PTransform.x<<", "<<PTransform.y<<") -->"<<indexCenter<<std::endl;

        if(!indexesUsed[indexCenter] && indexCenter != -1){
            indexesUsed[indexCenter] = true;
            idVector[indexCenter] = kk;
        }
        else
            return false;
        //if(indexCenter == -1){
        //    cv::putText(im, "-1", cv::Point(100,100), 1, 2, cv::Scalar(0, 100, 255));
        //}
    }
    return true;
}

bool ComputeCoefficients(const std::vector<cv::Point> & CenterPoints, int widthBound, int heightBound, int * position, float * coefX, float * coefY){
    int X1[4] = {CenterPoints[position[0]].x, CenterPoints[position[1]].x, CenterPoints[position[2]].x, CenterPoints[position[3]].x};
    int Y1[4] = {CenterPoints[position[0]].y, CenterPoints[position[1]].y, CenterPoints[position[2]].y, CenterPoints[position[3]].y};
    int X2[4] = {0, widthBound, widthBound, 0};
    int Y2[4] = {0, 0, heightBound, heightBound};
    //auto t1 = std::chrono::high_resolution_clock::now();
    ComputeBilinearCoeff(X1, Y1, X2, Y2, coefX, coefY, 4);
    return true;
}
static double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Vec2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors)
{
    std::vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

        err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);
        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }
    return std::sqrt(totalErr/totalPoints);
}
double RunCalibrateCamera(const std::vector<std::vector<cv::Vec2f>> & centersInImage, cv::Size imResolution, cv::Mat & cameraMatrix, cv::Mat & distCoeff, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, Grid grid, float spaceSize, bool solvpnp){
    /*
     * First Iteration
     */
    // Compute the Real Grid Image:
    std::vector<std::vector<cv::Point3f>> pointsRealImage(1);
    for(int i = 0; i < grid.height; i++){
        for(int j = 0; j < grid.width; j++){
            pointsRealImage[0].push_back(cv::Point3f(float(j*spaceSize), float(i*spaceSize), 0.0f));
        }
    }

    // Create many vec as images to calibrate there are
    pointsRealImage.resize(centersInImage.size(), pointsRealImage[0]);
    //cameraMatrix.at<double>(0,0) = 1.0;
    //double rms = cv::calibrateCamera(pointsRealImage, centersInImage, imResolution, cameraMatrix, distCoeff, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    //double rms = cv::calibrateCamera(pointsRealImage, centersInImage, imResolution, cameraMatrix, distCoeff, rvecs, tvecs, cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    double rms;
    if(!solvpnp)
        rms = cv::calibrateCamera(pointsRealImage, centersInImage, imResolution, cameraMatrix, distCoeff, rvecs, tvecs, 0);
    else{
        cv::solvePnP(pointsRealImage, centersInImage, cameraMatrix, distCoeff, rvecs, tvecs);

        std::vector<float> reperrperit;
        rms = computeReprojectionErrors(pointsRealImage, centersInImage, rvecs, tvecs, cameraMatrix, distCoeff, reperrperit);
    }

    return rms;
}

/*
 *      |    ReassingIdx    |
 *      ---------------------
 *      Assing Idx when is the first time or when the pattern is lost
 *
 *      Step 1: Get Corners:
 *          - Aproximate to a rectangle: cv::minAreaRect(CenterPoints)
 *          - Get At least 2 diagonal corners, this corners will be corners that have minimal distance with centerPoints (pattern) & corners of minAreaRect given an 'epsilon'
 *          - if corners getted is in the range: 2<= #Corners < 4, then apply the following method:
 *              -- Trace the diagonal with the two Points obtained
 *              -- Compute the distance to this Rect, the minimal distance & the maximal distance will be the another corners
 *      Step 2: Sort the corners:
 *          - In such way that this is always in the same pseudo order
 *
 *      Step 3: Apply Bilinear transformation
 *      Step 4: Map to each Index.
 */
bool ReassingIdx(int * idVector, std::vector<cv::Point> CenterPoints, int nPatternCenters, cv::Mat & im, float eps, Grid grid){
    if(CenterPoints.size() != nPatternCenters)
        return false;

    /*
     * (p4) *****************  (p3)
     *     *               *
     *    *               *  ymin
     *   ***************** (p2)
     *  (p1)
     *
     */
    /*
     * Step 1: Get Corners
     *  - Aproximate to a rectangle: cv::minAreaRect(CenterPoints)
     */
    cv::RotatedRect rect = cv::minAreaRect(CenterPoints);
    cv::Point2f rec_points[4];
    rect.points(rec_points);

    /*for(size_t j = 0; j < 4; j++){
        cv::line(im, rec_points[j], rec_points[(j+1)%4], cv::Scalar(255,0,0), 4, 8);
    }*/
    int position[4] = {-1, -1, -1, -1};
    //float eps = 20.0;
    //int maxnComponents = 20;
    //std::vector<std::priority_queue<int>> indexPossible(4);

    // Get At least 2 diagonal corners, this corners will be corners that have minimal distance with centerPoints (pattern) & corners of minAreaRect given an 'epsilon'
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

    //- if corners getted is in the range: 2<= #Corners < 4, then apply the following method:
    //              -- Trace the diagonal with the two Points obtained
    //              -- Compute the distance to this Rect, the minimal distance & the maximal distance will be the another corners
    for(int k = 0; k < 4; k++){
        if(position[k] == -1){
            // Get the Index of diagonal Rect
            int idx1 = (k + 1)%4, idx2 = (k + 3)%4;
            // If there's not defined the diagonal rect return false
            if(position[idx1] == -1 || position[idx2] == -1)
                return false;
            // Compute the Diagonal Rect.
            float pend = (float)(CenterPoints[position[idx2]].y - CenterPoints[position[idx1]].y)/(float)(CenterPoints[position[idx2]].x - CenterPoints[position[idx1]].x);
            float c = (float)(CenterPoints[position[idx2]].y - pend*CenterPoints[position[idx2]].x);
            //cv::line(im, CenterPoints[position[idx2]], CenterPoints[position[idx1]], cv::Scalar(255,0,255), 4, 8);

            //std::cout<<"y = "<<pend<<" x + " <<c<<std::endl;
            // Get the Center of maximal distance to this Rect.
            float maxd = 0.0;
            for(int m = 0; m < CenterPoints.size(); m++){
                cv::Point P1 = CenterPoints[m];
                float di = distanceRectToPoint(pend, -1.0, c, P1);
                if(di > maxd){
                    position[k] = m;
                    maxd = di;
                }
            }
            //float pend2 = -1/pend;
            //cv::Point Pm = CenterPoints[position[k]];
            //float c2 = Pm.y - pend2*Pm.x;
            //cv::Point intersection = cv::Point((c - c2)/(pend2 - pend), pend*(c - c2)/(pend2-pend) + c);
            //cv::line(im, Pm, intersection, cv::Scalar(255,255,0), 4, 8);

            // If the other corner is not defined or if it is equals to the previous corner then compute the minimal distance
            // Current this is the maximal distance but with negative signal, so this is the minimal distance
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
    // Step 2: Sort the corners
    SortIndexes(CenterPoints, position, 4);


    /*for(int k = 0; k < 4; k++){
        std::string pname = "P";
        pname.append(std::to_string(k+1));
        cv::putText(im, pname, CenterPoints[position[k]], 1, 2, cv::Scalar(255, 255, 0), 2, 8);
    }*/

    int widthBound, heightBound;
    widthBound = (grid.width-1)*20;
    heightBound = (grid.height-1)*20;

    float * coefX = new float[4];
    float * coefY = new float[4];
    /*int X1[4] = {CenterPoints[position[0]].x, CenterPoints[position[1]].x, CenterPoints[position[2]].x, CenterPoints[position[3]].x};
    int Y1[4] = {CenterPoints[position[0]].y, CenterPoints[position[1]].y, CenterPoints[position[2]].y, CenterPoints[position[3]].y};
    int X2[4] = {0, widthBound, widthBound, 0};
    int Y2[4] = {0, 0, heightBound, heightBound};
    */
    ComputeCoefficients(CenterPoints, widthBound, heightBound, position, coefX, coefY);
    //auto t1 = std::chrono::high_resolution_clock::now();
    //ComputeBilinearCoeff(X1, Y1, X2, Y2, coefX, coefY, 4);
    if(!ComputeIndexes(idVector, CenterPoints, nPatternCenters, coefX, coefY, grid)){
        //ComputeIndexes(idVector, CenterPoints, nPatternCenters, coefX, coefY, Grid(grid.height, grid.width));
        int temp = position[0];
        for(int pp = 0; pp < 4-1; pp++){
            position[pp] = position[pp+1];
        }
        position[3] = temp;
        ComputeCoefficients(CenterPoints, widthBound, heightBound, position, coefX, coefY);
        if(!ComputeIndexes(idVector, CenterPoints, nPatternCenters, coefX, coefY, grid))
            return false;
    }
    /*bool indexesUsed[nPatternCenters];
    memset(indexesUsed, false, nPatternCenters*sizeof (bool));
    //auto t2 = std::chrono::high_resolution_clock::now();
    for(int kk = 0; kk < CenterPoints.size(); kk++){
        float dd[4] = {(float)CenterPoints[kk].x, (float)CenterPoints[kk].y, (float)CenterPoints[kk].y*CenterPoints[kk].x, 1.0};
        cv::Point PTransform(DotProduct(coefX, dd, 4),DotProduct(coefY, dd, 4));

        int indexCenter = getIndexTable(PTransform, 20, grid, 0.5f);
        std::cout<<"Map->"<<" ("<<CenterPoints[kk].x<<", "<<CenterPoints[kk].y<<") --> "<<"( "<<PTransform.x<<", "<<PTransform.y<<") -->"<<indexCenter<<std::endl;
        idVector[indexCenter] = kk;
        if(indexCenter == -1){
            cv::putText(im, "-1", cv::Point(100,100), 1, 2, cv::Scalar(0, 100, 255));
        }
    }*/
    delete [] coefX;
    delete [] coefY;
    //cv::Point centerRect = rect.center;

    /*for(size_t j = 0; j < 4; j++){
        cv::line(im, CenterPoints[position[j]], CenterPoints[position[(j+1)%4]], cv::Scalar(0,0,255), 4, 8);
    }*/
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

    Grid grid(0,0);
    if(nPatternCenters == 20){
        grid.set(5, 4);
    }
    else if (nPatternCenters == 12) {
        grid.set(4, 3);
    }

    // If Pattern is well detected then allow to Draw
    bool allowDraw = false;
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
                            CenterPoints.push_back((minEllipse[poschild].center + minEllipse[posfather].center)/2.0);
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
        if(ReassingIdx(idVector, CenterPoints, nPatternCenters, rowFrame, eps, grid)){
            //std::string s= "Exim";
            //s.append(std::to_string(ccc));
            //s.append(".png");
            //ccc++;
            //cv::imwrite(s, rowFrame);
            allowDraw = true;
            reassign = false;
        }
    }
    else{
        // Perform the Tracking
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
        // Allow to draw if there's not problem in Tracking!
        if(!reassign)
            allowDraw = true;
    }

    CentersPrev = CenterPoints;
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    acumTime += (double)duration;
    //cv::rectangle(rowFrame, cv::Point(Xmin, Ymin), cv::Point(Xmax, Ymax), cv::Scalar(0, 0, 255), 4, 8);
    if(allowDraw){
        for(int k = 0; k < points2.size(); k++){
                    //boundRect[points2[k]] = cv::minAreaRect(cv::Mat(contours[points2[k]]));
                    //if( contours[points2[k]].size() > 5 )
                    //    minEllipse[points2[k]] = cv::fitEllipse( cv::Mat(contours[points2[k]]) );

                    cv::ellipse( rowFrame, minEllipse[points2[k]], cv::Scalar(0,255,0), 2, 8 );
                    //cv::putText(rowFrame, std::to_string(k), CenterPoints[idVector[k]], 1, 2, cv::Scalar(255, 0, 0));
                    // Uncomment bellow to drawBounding boxes
        }
        // Draw the ChessBoard!
        if(CenterPoints.size() == nPatternCenters){
            for(int nnr = 0; nnr < grid.height; nnr++){
                int wherep= nnr*grid.width;
                cv::line(rowFrame, CenterPoints[idVector[wherep]], CenterPoints[idVector[wherep + grid.width - 1]], cv::Scalar(94, 218, 250), 2, 8 );
                if(wherep != 0){
                    cv::line(rowFrame, CenterPoints[idVector[wherep]], CenterPoints[idVector[wherep - 1]], cv::Scalar(120, 10, 10), 2, 8);
                }
            }
        }

        for(int m = 0; m < CenterPoints.size(); m++){
               cv::putText(rowFrame, std::to_string(m), CenterPoints[idVector[m]], 1, 2, cv::Scalar(255, 0, 0), 2, 8);
        }

    }
    cv::putText(rowFrame, std::to_string(points2.size()), cv::Point(50, 20), 1, 2, cv::Scalar(0, 0, 255), 2, 8);

}
// Compute Fronto Parallel
bool ComputeFrontoParallel(cv::Mat & frame, cv::Mat & cameraMatrix, const cv::Mat & distCoeff, cv::Size ImResolution, int nPatternCenters, Grid grid, cv::Mat & FrontoParallelUndistortedOut){
    std::vector<std::vector<cv::Vec2f>> OutPoints(0);
    std::vector<cv::Point3f> pointsRealImage;
    float screenSpace = 20.0f;
    for(int i = 0; i < grid.height; i++){
        for(int j = 0; j < grid.width; j++){
            pointsRealImage.push_back(cv::Point3f(float(j*screenSpace + screenSpace/2), float(i*screenSpace + screenSpace/2), 0.0f));
        }
    }
    cv::Mat UndistortedImage;
    std::vector<cv::Point2f> UndistortedCenters;
    cv::undistort(frame, UndistortedImage, cameraMatrix, distCoeff);
    if(GetCenterPoints(UndistortedImage, nPatternCenters, UndistortedCenters)){
        cv::Mat homography = cv::findHomography(UndistortedCenters, pointsRealImage);
        FrontoParallelUndistortedOut = cv::Mat::zeros(screenSpace * grid.width, screenSpace * grid.height, CV_8UC3);
        cv::warpPerspective(UndistortedImage, FrontoParallelUndistortedOut, homography, cv::Size(screenSpace * grid.width, screenSpace * grid.height));
        return true;
    }
    return false;
}


// For Camera calibration
bool GetCenterPoints(cv::Mat & frame, int nPatternCenters, std::vector<cv::Point2f> & Centers){
    cv::Mat grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame;
    cv::Mat rowFrame = frame.clone();
    int idVector[nPatternCenters + 20];
    memset(idVector, -1, (nPatternCenters+20)*sizeof (int));
    float szpromEllipse = 1000.0;
    std::vector<cv::Point> CentersPrev;
    //Define the bounding box
    int Xmax = 1000.0;
    int Ymax = 1000.0;
    int Xmin = 0.0;
    int Ymin = 0.0;
    bool reassign = true;
    double acumm = 0.0;
    Centers.clear();
    ProccessImage(rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, integralFrame, nPatternCenters, idVector, CentersPrev, reassign, acumm, szpromEllipse, Xmax, Ymax, Xmin, Ymin);


    for(int ppp = 0; ppp < CentersPrev.size(); ppp++){
        Centers.push_back(cv::Point2f((float)CentersPrev[idVector[ppp]].x, (float)CentersPrev[idVector[ppp]].y));
    }
    if(Centers.size() == nPatternCenters && !reassign){
        cv::imwrite("imcent.png", frame);
        return true;
    }
    else
        return false;
}

std::vector<std::vector<cv::Vec2f>> GetRectifiedCenters(std::vector<cv::Mat> & frames, cv::Size ImResolution, std::vector<std::vector<cv::Vec2f>> & CentersInImage, const cv::Mat & cameraMatrix, const cv::Mat & distCoeff, const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs, const Grid grid, float spaceSize, int nPatternCenters){
    std::vector<std::vector<cv::Vec2f>> OutPoints(0);
    std::vector<cv::Point3f> pointsRealImage;
    /*float screenSpace = float(ImResolution.width)/float(grid.width);
    screenSpace = screenSpace < float(ImResolution.height)/float(grid.height)? screenSpace: float(ImResolution.height)/float(grid.height);
    for(int i = 0; i < grid.height; i++){
        for(int j = 0; j < grid.width; j++){
            pointsRealImage.push_back(cv::Point3f(float(j*screenSpace + screenSpace/2), float(i*screenSpace + screenSpace/2), 0.0f));
        }
    }*/
    float screenSpace = 20.0f;
    for(int i = 0; i < grid.height; i++){
        for(int j = 0; j < grid.width; j++){
            pointsRealImage.push_back(cv::Point3f(float(j*screenSpace + screenSpace/2), float(i*screenSpace + screenSpace/2), 0.0f));
        }
    }
    std::vector<cv::Mat> framesLocal;
    std::vector<std::vector<cv::Vec2f>> CentersInImageLocal;

    /*
     * Iterate over selected frames
     */
    for(int i = 0; i < frames.size(); i++){
        /*
         * Undistort Frame
         */
        cv::Mat copyFrame = frames[i].clone();
        cv::Mat UndistortedImage;
        std::vector<cv::Point2f> UndistortedCenters;
        cv::undistort(frames[i], UndistortedImage, cameraMatrix, distCoeff);
        if(GetCenterPoints(UndistortedImage, nPatternCenters, UndistortedCenters)){
            cv::Mat homography = cv::findHomography(UndistortedCenters, pointsRealImage);
            cv::Mat inv_homography = cv::findHomography(pointsRealImage, UndistortedCenters);
            cv::Mat FrontoParallelUndistorted = cv::Mat::zeros(screenSpace * grid.width, screenSpace * grid.height, CV_8UC3);
            cv::warpPerspective(UndistortedImage, FrontoParallelUndistorted, homography, cv::Size(screenSpace * grid.width, screenSpace * grid.height));
            std::string namex = "fronto";
            namex.append(std::to_string(i+1));
            namex.append(".png");
            cv::imwrite(namex, FrontoParallelUndistorted);
            std::vector<cv::Point2f> FrontoParallelCentersN;
            if(GetCenterPoints(FrontoParallelUndistorted, nPatternCenters, FrontoParallelCentersN)){
                std::vector<cv::Point2f> canonical;
                std::vector<cv::Point2f> FrontoParallelCenters = RecomputeFrontoParallel(FrontoParallelCentersN, grid);
                // Refinement ideal
                for(int tt = 0; tt < nPatternCenters; tt++){
                    canonical.push_back(cv::Point2f((pointsRealImage[tt].x + FrontoParallelCenters[tt].x)/2.0, (pointsRealImage[tt].y + FrontoParallelCenters[tt].y)/2.0));
                }
                std::vector<cv::Point2f> pointreprojected(nPatternCenters);
                cv::perspectiveTransform(canonical, pointreprojected, inv_homography);
                // Refinement Points, use dis points reprojected and the undistort points.
                for(int tt = 0; tt < pointreprojected.size(); tt++){
                    pointreprojected[tt].x = 0.5 * UndistortedCenters[tt].x + 0.5 * pointreprojected[tt].x;
                    pointreprojected[tt].y = 0.5 * UndistortedCenters[tt].y + 0.5 * pointreprojected[tt].y;
                }
                std::vector<cv::Vec2f> distortedReprojectedPoints;//(nPatternCenters);
                // Distort each Point Reprojected with no distortion
                // This is explanined in the OpenCV camera calibration tutorial
                std::vector<cv::Point3f> pointreprV(FrontoParallelCenters.size());
                for(int tttt = 0; tttt < pointreprV.size(); tttt++)
                    pointreprV[tttt] = cv::Point3f(canonical[tttt].x, canonical[tttt].y, 0.0f);

                cv::projectPoints(pointreprV, rvecs[i], tvecs[i], cameraMatrix, distCoeff, distortedReprojectedPoints);
                /*double Fx = cameraMatrix.at<double>(0, 0);
                double Fy = cameraMatrix.at<double>(1, 1);
                double Cx = cameraMatrix.at<double>(0, 2);
                double Cy = cameraMatrix.at<double>(1, 2);
                //double K1 = distCoeff.at<double>(0, 0);
                //double K2 = distCoeff.at<double>(0, 1);
                //double P1 = distCoeff.at<double>(0, 2);
                //double P2 = distCoeff.at<double>(0, 3);
                //double K3 = distCoeff.at<double>(0, 4);

                double K1 = distCoeff.at<double>(0);
                double K2 = distCoeff.at<double>(1);
                double P1 = distCoeff.at<double>(2);
                double P2 = distCoeff.at<double>(3);
                double K3 = distCoeff.at<double>(4);


                double xx;
                double yy;
                double r2;
                double Xcorrected;
                double Ycorrected;
                for(int tt = 0; tt < pointreprojected.size(); tt++){
                    xx = (pointreprojected[tt].x - Cx) / Fx;
                    yy = (pointreprojected[tt].y - Cy) / Fy;
                    r2 = xx * xx + yy * yy;

                    // Fix Radial distorsion
                    Xcorrected = xx * (1 + K1 * r2 + K2 * pow(r2, 2) + K3 * pow(r2, 3));
                    Ycorrected = yy * (1 + K1 * r2 + K2 * pow(r2, 2) + K3 * pow(r2, 3));

                    // Fix Tangential distorsion
                    Xcorrected = Xcorrected + (2 * P1 * xx * yy + P2 * (r2 + 2 * xx * xx));
                    Ycorrected = Ycorrected + (P1 * (r2 + 2 * yy * yy) + 2 * P2 * xx * yy);

                    Xcorrected = Xcorrected * Fx + Cx;
                    Ycorrected = Ycorrected * Fy + Cy;
                    //distortedReprojectedPoints[tt] = cv::Vec2f(Xcorrected, Ycorrected);
                }*/
                // Compute the original points
                //std::vector<cv::Point2f> originalimagepoints;
                //if(GetCenterPoints(copyFrame, nPatternCenters, originalimagepoints)){
                    std::vector<cv::Vec2f> ansxx(0);
                    for(int aaa = 0; aaa < distortedReprojectedPoints.size(); aaa++)
                        ansxx.push_back(cv::Vec2f(float(distortedReprojectedPoints[aaa][0] + CentersInImage[i][aaa][0])/2.0f, float(distortedReprojectedPoints[aaa][1] + CentersInImage[i][aaa][1])/2.0f));

                    // Finis distorting points
                    OutPoints.push_back(ansxx);
                    framesLocal.push_back(frames[i]);
                    CentersInImageLocal.push_back(CentersInImage[i]);
                //}

            }
        }
    }
    frames = framesLocal;
    CentersInImage = CentersInImageLocal;
    return OutPoints;
}

double RunIterativeCameraCalibration(std::vector<cv::Mat> & frames, std::vector<std::vector<cv::Vec2f>> & centersInImage, cv::Size imResolution, cv::Mat & cameraMatrix, cv::Mat & distCoeff, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, Grid grid, float spaceSize, int nIterations, int nPatternCenters, std::string & logComm){
    cv::Mat cameraMatrixLocal = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrixLocal.at<double>(0, 0) = 1.7778;
    cv::Mat distCoeffLocal = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecsLocal;
    std::vector<cv::Mat> tvecsLocal;

    double rms_ = RunCalibrateCamera(centersInImage, imResolution, cameraMatrixLocal, distCoeffLocal, rvecsLocal, tvecsLocal, grid, spaceSize, false);
    logComm = "****************\nIterations 0\n****************\nrms> ";
    logComm.append(std::to_string(rms_) + "\n");
    logComm.append("Fx > " + std::to_string(cameraMatrixLocal.at<double>(0, 0)) + "\n");
    logComm.append("Fy > " + std::to_string(cameraMatrixLocal.at<double>(1, 1)) + "\n");
    logComm.append("Cx > " + std::to_string(cameraMatrixLocal.at<double>(0, 2)) + "\n");
    logComm.append("Cy > " + std::to_string(cameraMatrixLocal.at<double>(1, 2)) + "\n");

    //double diffcenterX = (imResolution.width/2 - cameraMatrixLocal.at<double>(0,2));
    //double diffcenterY = (imResolution.height/2 - cameraMatrixLocal.at<double>(1,2));
    //double difCenter = diffcenterX*diffcenterX + diffcenterY*diffcenterY;

    cameraMatrix = cameraMatrixLocal.clone();
    distCoeff = distCoeffLocal.clone();
    //std::cout<<"rvecs SZ:>"<<rvecsLocal.size()<<std::endl;
    //std::cout<<"tvecs SZ:>"<<tvecsLocal.size()<<std::endl;
    rvecs.clear();
    tvecs.clear();
    for(int ttt = 0; ttt < rvecsLocal.size(); ttt++){
        rvecs.push_back(rvecsLocal[ttt].clone());
    }
    for(int ttt = 0; ttt < tvecsLocal.size(); ttt++){
        tvecs.push_back(tvecsLocal[ttt].clone());
    }
    double rms_true = rms_;
    //std::cout<<"RMS> It - "<<0<<" ->"<<rms_<<" --diff ->"<<difCenter<<std::endl;
    std::cout<<rms_true<<std::endl;
    for(int i = 0; i < nIterations; i++){
        // Recompute Centers
        std::vector<std::vector<cv::Vec2f>> newCentersInImage = GetRectifiedCenters(frames, imResolution, centersInImage, cameraMatrixLocal, distCoeffLocal, rvecsLocal, tvecsLocal, grid, spaceSize, nPatternCenters);
        // Init Parameters
        // std::cout<< "# New Centers SZ>"<<newCentersInImage.size()<<std::endl;
        cameraMatrixLocal = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrixLocal.at<double>(0, 0) = 1.7778;
        distCoeffLocal = cv::Mat::zeros(8, 1, CV_64F);
        rvecsLocal.clear();
        tvecsLocal.clear();
        rms_ = RunCalibrateCamera(newCentersInImage, imResolution, cameraMatrixLocal, distCoeffLocal, rvecsLocal, tvecsLocal, grid, spaceSize, false);
        //cv::solvePnP(newCentersInImage, )
        //logComm.append("\n"+std::to_string(i+1)+"rms> "+std::to_string(rms_));
        logComm.append("****************\nIterations " + std::to_string(i+1) + "\n****************\n");
        logComm.append("rms> " + std::to_string(rms_) + "\n");
        logComm.append("Fx > " + std::to_string(cameraMatrixLocal.at<double>(0, 0)) + "\n");
        logComm.append("Fy > " + std::to_string(cameraMatrixLocal.at<double>(1, 1)) + "\n");
        logComm.append("Cx > " + std::to_string(cameraMatrixLocal.at<double>(0, 2)) + "\n");
        logComm.append("Cy > " + std::to_string(cameraMatrixLocal.at<double>(1, 2)) + "\n");


        //diffcenterX = (imResolution.width/2 - cameraMatrixLocal.at<double>(0,2));
        //diffcenterY = (imResolution.height/2 - cameraMatrixLocal.at<double>(1,2));
        //double difCenterIt = diffcenterX*diffcenterX + diffcenterY*diffcenterY;

        //if(difCenterIt < difCenter){
            cameraMatrix = cameraMatrixLocal.clone();
            distCoeff = distCoeffLocal.clone();
            rvecs.clear();
            tvecs.clear();
            //std::cout<<"rvecs SZ:>"<<rvecsLocal.size()<<std::endl;
            //std::cout<<"tvecs SZ:>"<<tvecsLocal.size()<<std::endl;
            for(int ttt = 0; ttt < rvecsLocal.size(); ttt++){
                rvecs.push_back(rvecsLocal[ttt].clone());
            }
            for(int ttt = 0; ttt < tvecsLocal.size(); ttt++){
                tvecs.push_back(tvecsLocal[ttt].clone());
            }
            rms_true = rms_;
            //difCenter = difCenterIt;
        //}
        //centersInImage = newCentersInImage;
        std::cout<<rms_<<std::endl;
        //std::cout<<"RMS> It - "<<i + 1<<" ->"<<rms_<<" --diff ->"<<difCenterIt<<" --CxMat> "<<cameraMatrix.at<double>(0,2)<<std::endl;

    }

    return rms_true;
}
std::vector<cv::Point2f> RecomputeFrontoParallel(std::vector<cv::Point2f> centersFronto, Grid grid){
    std::vector<cv::Point2f> newCenters(centersFronto.size());

    std::vector<float> yprom(grid.height, 0.0f);
    std::vector<float> xprom(grid.width, 0.0f);
    for(int h = 0; h < grid.height; h++){
        for(int w = 0; w < grid.width; w++){
            yprom[h] += centersFronto[w + h*grid.width].y;
        }
        yprom[h] /= float(grid.width);
    }
    for(int w = 0; w < grid.width; w++){
        for(int h = 0; h < grid.height; h++){
            xprom[w] += centersFronto[w + h*grid.width].x;
        }
        xprom[w] /= float(grid.height);
    }

    for(int h = 0; h < grid.height; h++){
        for(int w = 0; w < grid.width; w++){
            newCenters[w + h * grid.width] = cv::Point2f((xprom[w] + centersFronto[w + h * grid.width].x)/2.0f, (yprom[h] + centersFronto[w + h * grid.width].y)/2.0f);
        }
    }
    for(int h = 0; h < grid.height; h++){
        for(int w = 0; w < grid.width; w++){
            newCenters[w + h * grid.width] = cv::Point2f((xprom[w] + centersFronto[w + h * grid.width].x)/2.0f, (yprom[h] + centersFronto[w + h * grid.width].y)/2.0f);
            std::cout << "("<<centersFronto[w + h * grid.width].x <<", "<<centersFronto[w + h * grid.width].y<<") --> ("<< newCenters[w + h * grid.width].x <<", "<< newCenters[w + h * grid.width].y<<")"<<std::endl;
        }
    }
    return newCenters;
}

#endif // PROCESSIMAGE_H
