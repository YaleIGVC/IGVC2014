
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <vector>

using namespace cv;

/// Global variables

Mat src, blueSrc, medianImg, thresholdImg;
Mat channel[3];
int thresholdValue = 200;
int maxval = 255;

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
    imshow("Lines", thresholdImg);

    waitKey(0);

    return 0;
}