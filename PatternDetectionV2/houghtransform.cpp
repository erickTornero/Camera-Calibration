# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>

const int approach=1;

int main(){

    cv::VideoCapture video("/home/erick/Documentos/MSCS/Images/VideosPrueba/PadronAnillos_01.avi");  

    if(!video.isOpened()){
        printf("Can't open video or webcam\n");
        return -1;
    }
    //cv::namedWindow("Frame", CV_WINDOW_AUTOSIZE);
    for( ; ; ){
        cv::Mat rowFrame, grayRowFrame, blurGaussFrame, thresholdFrame, cannyFrame;
        video >> rowFrame;
        cv::cvtColor(rowFrame, grayRowFrame, CV_RGB2GRAY);
        if(approach == 1){
            for ( int i = 1; i < 5; i = i + 2 ){ 
                GaussianBlur( grayRowFrame, blurGaussFrame, cv::Size( i, i ), 0, 0 );
            }
            cv::threshold(blurGaussFrame, thresholdFrame, 100, 255, CV_THRESH_BINARY);
            cv::imshow("GaussianBlur", blurGaussFrame);
            cv::imshow("Thresholded", thresholdFrame);
            //cv::Canny(blurGaussFrame, thresholdFrame, 200, 300);
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            
            cv::findContours(thresholdFrame, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
            std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
            std::vector<cv::RotatedRect> boundRect( contours.size() );
            std::vector<cv::RotatedRect> minEllipse( contours.size() );
            //minRect[i] = minAreaRect( Mat(contours[i]) );
       //if( contours[i].size() > 5 )
         //{ minEllipse[i] = fitEllipse( Mat(contours[i]) ); }

            for( int i = 0; i< contours.size(); i++ ){
                boundRect[i] = cv::minAreaRect(cv::Mat(contours[i]));
                if( contours[i].size() > 5 )
                    minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) );
                //boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
                //cv::rectangle( rowFrame, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(100,100,0), 2, 8, 0 );
                //cv::drawContours( rowFrame, contours, i, cv::Scalar(100,100,0), 4, 8, hierarchy, 0, cv::Point() );
                cv::ellipse( rowFrame, minEllipse[i], cv::Scalar(0,255,0), 2, 8 );
                cv::Point2f rect_points[4]; boundRect[i].points( rect_points );
                for( int j = 0; j < 4; j++ )
                    cv::line( rowFrame, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 0,255), 4, 8 );
            }
            //cv::minAreaRect(contours);
            cv::imshow("gaussian", blurGaussFrame);
            cv::imshow("Contours", rowFrame);
        }
        else if(approach == 2){

        }
        
        
        //cv::threshold(grayRowFrame, thresholdFrame, 80, 255, CV_THRESH_BINARY_INV);
        //cv::Canny(thresholdFrame, cannyFrame, 150, 300);
        //cv::imshow("ROW", rowFrame);
        //cv::imshow("Thresholding", thresholdFrame);
        //cv::imshow("Canny", cannyFrame);
        

        //ESC Keyboard
        if((char)cv::waitKey(25) == 27)
            break;

    }
    video.release();
    cv::destroyAllWindows();
}
