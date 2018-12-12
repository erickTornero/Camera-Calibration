# include <opencv2/opencv.hpp>
# include <stdio.h>
# include <vector>
# include <stdio.h>
# include <string.h>
# include <stdlib.h>

void printhierarquy(const std::vector<cv::Vec4i> & h ){
    for(size_t i = 0; i < h.size(); i++){
        printf("%i ->", i);
        for(size_t j = 0; j < 4; j++){
            printf(" %i", h[i][j]);            
        }
        printf("\n");
    }
}
void printhierarquyPoints(const std::vector<cv::Vec4i> & h, std::vector<int> points ){
    for(size_t i = 0; i < points.size(); i++){
        printf("%i ->", points[i]);
        for(size_t j = 0; j < 4; j++){
            printf(" %i", h[points[i]][j]);            
        }
        printf("\n");
    }
}
int main(){
    double epsilon = 0.8;
    cv::Mat rowImage = cv::imread("patternimg.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat grayRowImage, blurGaussImage, thresholdImage, cannyImage;
    cv::Mat showImage;
    cv::Mat rowImCpy;
    rowImage.copyTo(rowImCpy);

    // Gray Color conversion:
    cv::cvtColor(rowImage, grayRowImage, CV_RGB2GRAY);
    GaussianBlur( grayRowImage, blurGaussImage, cv::Size( 3, 3 ),0, 0);
    cv::threshold(blurGaussImage, thresholdImage, 100, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    printhierarquy(hierarchy);
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::RotatedRect> boundRect( contours.size() );
    

    //first pass, avoid all points with no father and childs
    std::vector<int> points;
    for(int a = 0; a < contours.size(); a++){
        if(hierarchy[a][2] >= 0 || hierarchy[a][3] >= 0 && contours[a].size() > 20)
            points.push_back(a);
    }
    //run for hole methods. if center of father are near, then add current point
    std::vector<cv::RotatedRect> minEllipse( contours.size() );
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
    }
    //std::vector<int> points{50, 81, 82, 83, 84};
    for(int k = 0; k < points2.size(); k++){
        boundRect[points2[k]] = cv::minAreaRect(cv::Mat(contours[points2[k]]));
        if( contours[points2[k]].size() > 5 )
            minEllipse[points2[k]] = cv::fitEllipse( cv::Mat(contours[points2[k]]) );
        
        cv::ellipse( rowImage, minEllipse[points2[k]], cv::Scalar(0,255,0), 2, 8 );
        cv::putText(rowImage, std::to_string(points2[k]), cv::Point(boundRect[points2[k]].center.x, boundRect[points2[k]].center.y + rand()%5), 1, 2, cv::Scalar(255, 0, 0));
        // Uncomment bellow to drawBounding boxes
    }
    for( int i = 0; i< contours.size(); i++ ){
        boundRect[i] = cv::minAreaRect(cv::Mat(contours[i]));
        if( contours[i].size() > 5 )
            minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) );
        //boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
        //cv::rectangle( rowFrame, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(100,100,0), 2, 8, 0 );
        //cv::drawContours( rowFrame, contours, i, cv::Scalar(100,100,0), 4, 8, hierarchy, 0, cv::Point() );
        cv::ellipse( rowImCpy, minEllipse[i], cv::Scalar(0,255,0), 2, 8 );
        //cv::putText(rowImage, std::to_string(i), cv::Point(boundRect[i].center.x, boundRect[i].center.y + rand()%20), 1, 4, cv::Scalar(255, 0, 0));
        // Uncomment bellow to drawBounding boxes
        //cv::Point2f rect_points[4]; boundRect[i].points( rect_points );
        //for( int j = 0; j < 4; j++ )
        //cv::imshow("Contours", rowImage);
        int xx = 0;
        //    cv::line( rowImage, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 0,255), 4, 8 );
    }
    cv::putText(rowImage, std::to_string(points2.size()), cv::Point(50, 20), 1, 2, cv::Scalar(0, 0, 255));
    cv::putText(rowImCpy, std::to_string(contours.size()), cv::Point(50, 20), 1, 2, cv::Scalar(0, 0, 255));
    //cv::minAreaRect(contours);
    //cv::imshow("gaussian", blurGaussImage);
    //cv::imshow("ThresholdedImg", thresholdImage);
    cv::hconcat(rowImCpy, rowImage, showImage);
    cv::resize(showImage, showImage, cv::Size(), 0.5, 0.5);
    cv::imshow("Contours", showImage);
    //cv::im
    cv::waitKey(0);
}