# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>
# include <stdio.h>
# include <string.h>
# include <stdlib.h>
# include <chrono>
int main(){
    //cv::VideoCapture video("/home/erick/Documentos/MsCS/Images/VideosPrueba/VideosPrueba/PadronAnillos_01.avi");  
    cv::VideoCapture video("/home/erick/Documentos/MsCS/Images/videos/padron1.avi");
    double epsilon = 1.0;
    int ncicles = 120;
    if(!video.isOpened()){
        printf("Can't open video or webcam\n");
        return -1;
    }
    std::chrono::high_resolution_clock::time_point t1,t2;
    unsigned long counter = 0;
    t1 = std::chrono::high_resolution_clock::now();
    double acumTime = 0.0;
    while(true){

        counter++;
        if(counter % ncicles == 0){
            std::cout<<acumTime/(double)ncicles<<std::endl;
            acumTime = 0.0;
        }
        t1 = std::chrono::high_resolution_clock::now();
        cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, cannyFrame, joinImages;
        video >> rowFrame;
        cv::cvtColor(rowFrame, grayRowFrame, CV_RGB2GRAY);
        GaussianBlur( grayRowFrame, blurGaussFrame, cv::Size( 3, 3 ), 0, 0 );
        cv::threshold(blurGaussFrame, thresholdFrame, 100, 255, CV_THRESH_BINARY);
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thresholdFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
            //first pass, avoid all points with no father and childs
        std::vector<int> points;
        for(int a = 0; a < contours.size(); a++){
            if(hierarchy[a][2] >= 0 || hierarchy[a][3] >= 0 && contours[a].size() > 20)
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
        bool isElipseComputed[contours.size()] = {false};
        std::vector<cv::RotatedRect> minEllipse( contours.size() );
        for(int a = 0; a < points.size(); a++){
            int poschild = points[a];
            if(hierarchy[poschild][2] == -1 ){
                if(!isElipseComputed[poschild]){
                    minEllipse[poschild] = cv::fitEllipse(cv::Mat(contours[poschild]));
                    isElipseComputed[poschild] = true;
                }
                int posfather = hierarchy[points[a]][3];
                if(!isElipseComputed[posfather]){
                    minEllipse[posfather] = cv::fitEllipse(cv::Mat(contours[posfather]));
                    isElipseComputed[posfather] = true;
                }
                double val = cv::norm(minEllipse[poschild].center - minEllipse[posfather].center);
                if(epsilon > val){
                    points2.push_back(poschild);
                    points2.push_back(posfather);
                }
            }
        }


        // Draw Elipses:
        for(int k = 0; k < points2.size(); k++){
            //boundRect[points2[k]] = cv::minAreaRect(cv::Mat(contours[points2[k]]));
            //if( contours[points2[k]].size() > 5 )
            //    minEllipse[points2[k]] = cv::fitEllipse( cv::Mat(contours[points2[k]]) );
            
            cv::ellipse( rowFrame, minEllipse[points2[k]], cv::Scalar(0,255,0), 2, 8 );
            cv::putText(rowFrame, std::to_string(points2[k]), minEllipse[points2[k]].center, 1, 2, cv::Scalar(255, 0, 0));
            // Uncomment bellow to drawBounding boxes
        }
        t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        acumTime += (double)duration;
        
        //cv::hconcat(thresholdFrame, rowFrame, joinImages);
        cv::imshow("Contours", rowFrame);
        cv::imshow("Contours", thresholdFrame);
        if((char)cv::waitKey(25) == 27)
            break;
    }
    video.release();
    cv::destroyAllWindows();
}