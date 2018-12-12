# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>
# include <stdio.h>
# include <string.h>
# include <stdlib.h>
# include <chrono>
# include "IntegrarThreshold.hpp"
#include <limits.h>
void getBoundBox(const std::vector<cv::RotatedRect> & minEllipse, const std::vector<int> & points, int &xmin, int &ymin, int & xmax, int & ymax){
  xmax = 0, ymax = 0, xmin = std::numeric_limits<int>::max(), ymin = std::numeric_limits<double>::max();

  for(size_t i = 0; i < points.size(); i++){
        xmax = xmax < minEllipse[points[i]].center.x ? minEllipse[points[i]].center.x : xmax;
        ymax = ymax < minEllipse[points[i]].center.y ? minEllipse[points[i]].center.y : ymax;
        xmin = xmin > minEllipse[points[i]].center.x ? minEllipse[points[i]].center.x : xmin;
        ymin = ymin > minEllipse[points[i]].center.y ? minEllipse[points[i]].center.y : ymin;
  }

}
int main(){
    //cv::VideoCapture video("/home/erick/Documentos/MsCS/Images/VideosPrueba/VideosPrueba/PadronAnillos_01.avi");  
    cv::VideoCapture video("/home/erick/Documentos/MsCS/Images/videos/padron1.avi");
    // Epsilon to define the distance to center 
    int PatternSIZE = 60;
    double epsilon = 1.0;
    // Epsilon Bounding box comparison with previous bounding box, must be scaled
    double epsilonBB = 100.0;
    // Epsilon for diferences of Sizes:
    float epsilonSZEl = 50.0;
    int ncicles = 120;
    if(!video.isOpened()){
        printf("Can't open video or webcam\n");
        return -1;
    }
    std::chrono::high_resolution_clock::time_point t1,t2;
    unsigned long counter = 0;
    t1 = std::chrono::high_resolution_clock::now();
    double acumTime = 0.0;
    float szpromEllipse = 1000.0;
    //Define the bounding box
    int Xmax = 1000.0;
    int Ymax = 1000.0;
    int Xmin = 0.0;
    int Ymin = 0.0;
    while(true){

        counter++;
        if(counter % ncicles == 0){
            std::cout<<acumTime/(double)ncicles<<std::endl;
            acumTime = 0.0;
        }
        t1 = std::chrono::high_resolution_clock::now();
        cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, cannyFrame, joinImages, integralFrame;
        video >> rowFrame;
        cv::cvtColor(rowFrame, grayRowFrame, CV_RGB2GRAY);
        GaussianBlur( grayRowFrame, blurGaussFrame, cv::Size( 5, 5 ), 0, 0 );
        cv::threshold(blurGaussFrame, thresholdFrame, 100, 255, CV_THRESH_BINARY);
        blurGaussFrame.copyTo(integralFrame);
        thresholdIntegral(blurGaussFrame, integralFrame);
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        //cv::findContours(thresholdFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(integralFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
            //first pass, avoid all points with no father and childs
        std::vector<int> points;
        for(int a = 0; a < contours.size(); a++){
            if((hierarchy[a][2] >= 0 || hierarchy[a][3] >= 0) && contours[a].size() > 5)
                points.push_back(a);
        }
        //run for hole methods. if center of father are near, then add current point
        /*std::vector<cv::RotatedRect> minEllipse( contours.size() );
        std::vector<int> points2;
        for(int a = 0; a < points.size(); a++){
            if(hierarchy[points[a]][2] == -1 ){
                cv::RotatedRect elipseChild = cv::fitEllipse( cv::Mat(contours[points[a]]) );
                cv::RotatedRect elipseFat = cv::fitEllipse(cv::Mat(contours[hierarchy[points[a]][3]]));
                double val = cv::norm(elipseChild.center - elipseFat.center);
                if(epsilon > val){
                    points2.push_back(points[a]);
                    points2.push_back(hierarchy[points[a]][3]);
                }
            }
        }*/
        std::vector<int> points2;
        //Get the MBB:
        int xmin = 3000;
        int ymin = 3000;
        int xmax = 0;
        int ymax = 0;
        double szprom = 0.0;
        bool isElipseComputed[contours.size()] = {false};
        std::vector<cv::RotatedRect> minEllipse( contours.size() );
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
        //ymax = 300;
        //getBoundBox(minEllipse, points2, xmin, ymin, xmax, ymax);
        
        //cv::line(rowFrame, cv::Point(Xmin, Ymin), cv::Point(Xmax, Ymin), cv::Scalar(0, 0, 255), 4, 8);
        //cv::line(rowFrame, cv::Point(Xmax, Ymin), cv::Point(Xmax, Ymax), cv::Scalar(0, 0, 255), 4, 8);
        //cv::line(rowFrame, cv::Point(Xmin, Ymax), cv::Point(Xmax, Ymax), cv::Scalar(0, 0, 255), 4, 8);
        //cv::line(rowFrame, cv::Point(Xmin, Ymax), cv::Point(Xmin, Ymin), cv::Scalar(0, 0, 255), 4, 8);
        //cv::line( rowFrame, rect_points[j], , cv::Scalar(0, 0,255), 4, 8 );
        // For heuristic Elimination
        szpromEllipse = szprom/(float)points2.size();
        if(points2.size() != 0){
            Xmax = xmax;
            Xmin = xmin;
            Ymax = ymax;
            Ymin = ymin;
        }
        cv::rectangle(rowFrame, cv::Point(Xmin, Ymin), cv::Point(Xmax, Ymax), cv::Scalar(0, 0, 255), 4, 8);
        //else{
            
        //}
        
        // Restart initial conditions
        if(points2.size() == 0){
            szpromEllipse = 1000.0;
            //Xmax = 1000.0;
            //Ymax = 1000.0;
            //Xmin = 0.0;
            //Ymin = 0.0;
        }
        t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        acumTime += (double)duration;

        // Draw Elipses:
        for(int k = 0; k < points2.size(); k++){
            //boundRect[points2[k]] = cv::minAreaRect(cv::Mat(contours[points2[k]]));
            //if( contours[points2[k]].size() > 5 )
            //    minEllipse[points2[k]] = cv::fitEllipse( cv::Mat(contours[points2[k]]) );
            
            cv::ellipse( rowFrame, minEllipse[points2[k]], cv::Scalar(0,255,0), 2, 8 );
            cv::putText(rowFrame, std::to_string(points2[k]), minEllipse[points2[k]].center, 1, 2, cv::Scalar(255, 0, 0));
            // Uncomment bellow to drawBounding boxes
        }
        cv::putText(rowFrame, std::to_string(points2.size()), cv::Point(50, 20), 1, 2, cv::Scalar(0, 0, 255));
        
        //cv::hconcat(thresholdFrame, rowFrame, joinImages);
        cv::imshow("Contours", rowFrame);
        cv::imshow("Integral", integralFrame);
        if((char)cv::waitKey(25) == 27)
            break;
    }
    video.release();
    cv::destroyAllWindows();
}