#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>
#include <iostream>
#include <vector>
#include <limits>

using namespace cv;
using namespace std;

double getRay(vector<cv::KeyPoint> *keypoints, vector<cv::KeyPoint> *raypoints){
  double x_sum = 0, y_sum = 0, xy_mult = 0, x_power = 0;
  for(size_t i = 0; i < keypoints->size(); i++){
        x_sum += (*keypoints)[i].pt.x;
        y_sum += (*keypoints)[i].pt.y;
        xy_mult += (*keypoints)[i].pt.x * (*keypoints)[i].pt.y;
        x_power += (*keypoints)[i].pt.x * (*keypoints)[i].pt.x;
  }

  double m = (xy_mult- ((x_sum * y_sum) / keypoints->size())) / (x_power - ((x_sum*x_sum) / keypoints->size()));
  double mean_x = x_sum / keypoints->size(), mean_y = y_sum / keypoints->size();

  double x_1 = 0.0;
  double x_2 = 1000.0;
  double b = mean_y - (m*mean_x);

  double y_1 = m * x_1 + b;
  double y_2 = m * x_2 + b;

  raypoints->push_back( KeyPoint {x_1, y_1, 2} );
  raypoints->push_back( KeyPoint {x_2, y_2, 2} );

  double mean_dis = 0;
  for(size_t i = 0; i < keypoints->size(); i++)
        mean_dis += (m * (*keypoints)[i].pt.x+ (*keypoints)[i].pt.y + b) / sqrt(m*m+ 1);

  mean_dis = mean_dis / keypoints->size();
  return mean_dis;
  //std::cout << "mean = " << mean_dis << std::endl;
}

void getBoundBox(vector<cv::KeyPoint> *keypoints, vector<cv::KeyPoint> *boundpoints){
  double max_x = 0, max_y = 0, min_x = std::numeric_limits<double>::max(), min_y = std::numeric_limits<double>::max();

  for(size_t i = 0; i < keypoints->size(); i++){
        max_x = max_x < (*keypoints)[i].pt.x ? (*keypoints)[i].pt.x : max_x;
        max_y = max_y < (*keypoints)[i].pt.y ? (*keypoints)[i].pt.y : max_y;
        min_x = min_x > (*keypoints)[i].pt.x ? (*keypoints)[i].pt.x : min_x;
        min_y = min_y > (*keypoints)[i].pt.y ? (*keypoints)[i].pt.y : min_y;
  }

  boundpoints->push_back( KeyPoint {min_x, max_y, 2} );
  boundpoints->push_back( KeyPoint {max_x, min_y, 2} );

}

void deleteNoise(vector<cv::KeyPoint> *keypoints, double mean){
  
}

int main(){
 
  VideoCapture cap(0);

  if(!cap.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }
  
  cv::SimpleBlobDetector::Params params_1;
  params_1.minThreshold = 10;
  params_1.maxThreshold = 200;
  // Filter by Area.
	params_1.filterByArea = true;
	params_1.minArea = 100;
  // Filter by Circularity
  params_1.filterByCircularity = true;
  params_1.minCircularity = 0.8;
  // Filter by Convexity
  params_1.filterByConvexity = true;
  params_1.minConvexity = 0.9;
  // Filter by Inertia
  params_1.filterByInertia = true;
  params_1.minInertiaRatio = 0.01;


  cv::SimpleBlobDetector::Params params_2;
  params_2.minThreshold = 10;
  params_2.maxThreshold = 200;

  while(1){
 
    Mat frame;
    Mat dest_gray, border_gray, laplacian, src;

    cap >> frame;

    cv::cvtColor(frame, dest_gray, cv::COLOR_BGR2GRAY);
    //imshow( "cvtColor", dest_gray );
    cv::Canny( dest_gray, border_gray, 200, 300, 3 );
    //imshow( "Canny", border_gray );
    cv::Laplacian( border_gray, laplacian, CV_8U );
    //imshow( "Laplacian", laplacian );
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::KeyPoint> keypoints_wp;
    std::vector<cv::KeyPoint> raypoints;
    std::vector<cv::KeyPoint> boundpoints;
    
    /* Detection */
    cv::Ptr<cv::SimpleBlobDetector> detector_1 = cv::SimpleBlobDetector::create(params_1);
    cv::Ptr<cv::SimpleBlobDetector> detector_2 = cv::SimpleBlobDetector::create(params_2);
    detector_1->detect(laplacian, keypoints);
    detector_2->detect(laplacian, keypoints_wp);
    
    Mat drawI = border_gray.clone();

    double mean = getRay(&keypoints, &raypoints);
    //std::cout << " KeyPoint = " << raypoints[0].pt << std::endl;
    //std::cout << " KeyPoint = " << raypoints[1].pt << std::endl;
    line(frame, raypoints[0].pt, raypoints[1].pt, cv::Scalar(255,0,0), 5);
    KeyPoint punto{20, 20, 2};
    putText(frame, "mean: " + std::to_string(mean), punto.pt, cv::FONT_HERSHEY_SIMPLEX , 0.7, (255,255,255), 1.5, cv::LINE_AA);

    getBoundBox(&keypoints, &boundpoints);
    rectangle(frame, boundpoints[0].pt, boundpoints[1].pt, cv::Scalar(255,180,0),3);

    deleteNoise(&keypoints, mean);
    //std::cout << "KeyPoint size = " << keypoints_wp.size() << std::endl;
    for(size_t i = 0; i < keypoints_wp.size(); i++)
        cv::circle(frame, keypoints_wp[i].pt, 5, cv::Scalar(180,180,0), 3);
        
    for(size_t i = 0; i < keypoints.size(); i++){
        cv::circle(frame, keypoints[i].pt, 5, cv::Scalar(180,180,0), 3);
        //std::cout << i << " KeyPoint = " << keypoints[i].pt << std::endl;
        cv::putText(frame, std::to_string(i), keypoints[i].pt, cv::FONT_HERSHEY_SIMPLEX , 0.5, (255,255,255), 1, cv::LINE_AA);
    }
    if (frame.empty())
      break;
 
    // Display the resulting frame
    imshow( "Frame", frame );
 
    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  
  // When everything done, release the video capture object
  cap.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}
