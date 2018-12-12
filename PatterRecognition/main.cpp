#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d.hpp>
#include <iostream>
#include <vector>

using namespace cv;



int main(){
 
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  //VideoCapture cap(0);
  VideoCapture cap("/home/erick/Documentos/MsCS/Images/VideosPrueba/VideosPrueba/PadronAnillos_02.avi");
  // Check if camera opened successfully
  if(!cap.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }
     
  while(1){
 
    Mat frame;
    Mat dest_gray, border_gray, gaussian_gray;
    
    // Capture frame-by-frame
    cap >> frame;
    cv::cvtColor(frame, dest_gray, cv::COLOR_BGR2GRAY);
    //cv::Canny( dest_gray, border_gray, 200, 300, 3 );
    
    cv::threshold(dest_gray, dest_gray, 80, 255, 3);
    //cv::adaptiveThreshold(dest_gray, dest_gray, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,21, 0);
    cv::medianBlur(dest_gray, dest_gray, 3);
    std::vector<std::vector<cv::Point> > contours;
    //cv::findContours(dest_gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    cv::Canny( dest_gray, border_gray, 200, 300, 3 );
    std::vector<cv::KeyPoint> keypoints;
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(border_gray, keypoints);
    // If the frame is empty, break immediately

    Mat drawI = border_gray.clone();
    for(size_t i = 0; i < keypoints.size(); i++){
        cv::circle(frame, keypoints[i].pt, keypoints[i].size, cv::Scalar(180, 0, 0), -1);

    }
    for (size_t idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(frame, contours, idx, cv::Scalar(0, 0, 255));
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