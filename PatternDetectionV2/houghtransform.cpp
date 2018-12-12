# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>

# include "IntegrarThreshold.cpp"

#define r_cols 420
#define r_rows 315

using namespace cv;
using namespace std;


const int approach=1;




int main(){

    cv::VideoCapture video("../../files/padron1.avi");  

    if(!video.isOpened()){
        printf("Can't open video or webcam\n");
        return -1;
    }
    //cv::namedWindow("Frame", CV_WINDOW_AUTOSIZE);
    char i = 0;
    while(i < 13){
        cv::Mat rowFrame, grayRowFrame, blurGaussFrame, cannyFrame, erosionFrame, closeFrame;
        //video >> rowFrame;
        rowFrame = cv::imread("../../files/imagen"+to_string(i++)+".png");

        //if((char)cv::waitKey(25) == 27){
        //        vector<int> compression_params;
        //        compression_params.push_back(IMWRITE_PNG_COMPRESSION);
        //        compression_params.push_back(0);
        //        compression_params.push_back(IMWRITE_PNG_STRATEGY);
        //        compression_params.push_back(IMWRITE_PNG_STRATEGY_DEFAULT);
        //        char name[] = {};
        //        imwrite("../../files/imagen"+to_string(i++)+".png", rowFrame, compression_params);
        //    }

        cv::cvtColor(rowFrame, grayRowFrame, CV_RGB2GRAY);
        if(approach == 1){
            //for ( int i = 3; i < 7; i +=2 ){ 
                GaussianBlur( grayRowFrame, blurGaussFrame, cv::Size( 3, 3 ), 0, 0 );
            //    grayRowFrame = blurGaussFrame;
            //}
            //cv::threshold(blurGaussFrame, thresholdFrame, 31, 255, CV_THRESH_BINARY);
            cv::Mat element = getStructuringElement( MORPH_CROSS, Size( 3, 3 ), Point( 0, 0 ) );

            /// Apply the erosion operation
            cv::erode( blurGaussFrame, erosionFrame, element );
            //cv::imshow("erode", erosionFrame);

            cv::Mat integralThresholdFrame = cv::Mat::zeros(erosionFrame.size(), CV_8UC1);
            thresholdIntegral(erosionFrame, integralThresholdFrame);

            /* close */
            cv::Mat element_morf = getStructuringElement( MORPH_CROSS,
                                       Size( 3, 3 ),
                                       Point( 0, 0 ) );

            Canny( integralThresholdFrame, closeFrame, 200, 200, 5 );
            //morphologyEx( integralThresholdFrame, closeFrame, MORPH_BLACKHAT, element_morf ); 
            //morphologyEx( closeFrame, closeFrame, MORPH_CLOSE, element_morf ); 

            //grayRowFrame.convertTo(grayRowFrame, CV_8UC1);
            //cv::integral(grayRowFrame, thresholdFrame);
            
            //cv::Mat sumMat;
            //cv::integral(blurGaussFrame, sumMat, 100);
            

            //thresholdFrame = bw2;
            //cv::threshold(bw2, thresholdFrame, 125, 255, CV_THRESH_BINARY);
            //cv::imshow("Integral", bw2);

            //cv::imshow("GaussianBlur", blurGaussFrame);
            //cv::imshow("Thresholded", thresholdFrame);
            //cv::Canny(blurGaussFrame, thresholdFrame, 200, 300);
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            
            cv::findContours(closeFrame, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
            std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
            std::vector<cv::RotatedRect> boundRect( contours.size() );
            std::vector<cv::RotatedRect> minEllipse( contours.size() );
            //minRect[i] = minAreaRect( Mat(contours[i]) );
       //if( contours[i].size() > 5 )
         //{ minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
            //cout << "elipse: " << contours.size() << endl;
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
            
            cv::imshow("Contours", rowFrame);

            


            cv::Size size(r_cols, r_rows);
            resize(grayRowFrame, grayRowFrame,size);
            resize(integralThresholdFrame, integralThresholdFrame,size);
            resize(erosionFrame, erosionFrame,size);
            resize(closeFrame, closeFrame,size);

            int dstWidth = r_cols*4;
            int dstHeight = r_rows;

            cv::Mat dst = cv::Mat(dstHeight, dstWidth, grayRowFrame.depth(), cv::Scalar(0,0,0));
            //cv::Rect roi(cv::Rect(0,0,image1.cols, image1.rows));
            cv::Mat targetROI = dst(cv::Rect(0, 0, r_cols, r_rows));
            grayRowFrame.copyTo(targetROI);
            targetROI = dst(cv::Rect(r_cols, 0, r_cols, r_rows));
            erosionFrame.copyTo(targetROI);
            targetROI = dst(cv::Rect(r_cols * 2, 0, r_cols, r_rows));
            integralThresholdFrame.copyTo(targetROI);
            targetROI = dst(cv::Rect(r_cols * 3, 0, r_cols, r_rows));
            closeFrame.copyTo(targetROI);

            cv::namedWindow("OpenCV Window");
            cv::imshow("OpenCV Window", dst);
            //cv::waitKey(5000);

            
        }
        else if(approach == 2){

        }
        
        
        //cv::threshold(grayRowFrame, thresholdFrame, 80, 255, CV_THRESH_BINARY_INV);
        //cv::Canny(thresholdFrame, cannyFrame, 150, 300);
        //cv::imshow("ROW", rowFrame);
        //cv::imshow("Thresholding", thresholdFrame);
        //cv::imshow("Canny", cannyFrame);
        

        //ESC Keyboard
        cv::waitKey(3000);

    }
    video.release();
    cv::destroyAllWindows();
}