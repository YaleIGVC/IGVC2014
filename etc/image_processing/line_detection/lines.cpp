#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <vector>

using namespace cv;

/// Global variables

Mat src, blueSrc, medianImg, thresholdImg, lineImg;
Mat channel[3];
int thresholdValue = 200;
int maxval = 255;
vector<Vec4i> lines;

/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
    src = imread( argv[1] );
    split(src, channel);
    blueSrc = channel[0];
    if( !src.data )
    { return -1; }
    
    medianBlur(blueSrc, medianImg, 3);
    threshold(medianImg, thresholdImg, thresholdValue, 255, THRESH_BINARY);
    //cvtColor(medianImg, hsvImg, CV_BGR2HSV);
    //inRange(hsvImg, Scalar(Hlow, Slow, Vlow), Scalar(Hhigh, Shigh, Vhigh), inRangeImg);
    HoughLinesP( thresholdImg, lines, 1, CV_PI/180, 80, 30, 10 );
    thresholdImg.copyTo(lineImg);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( lineImg, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(122, 122, 122), 3, 8 );
    }
    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", lineImg );
    namedWindow( "Lines", 1 );
    imshow( "Lines", thresholdImg );

    waitKey(0);

    return 0;
}