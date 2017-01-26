#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;

/** @function main */
int main(int argc, char** argv)
{
  Mat inSrc, src, src_gray;

  /// Read the image
  // inSrc = imread( argv[1], 1 );
  inSrc = imread("inside3.jpg", 1 );

  cv::resize(inSrc, src, cv::Size(), 0.25, 0.25);

  if( !src.data )
    { return -1; }

  /// Convert it to gray
  cvtColor( src, src_gray, CV_BGR2GRAY );

  // namedWindow( "gray", CV_WINDOW_AUTOSIZE );
  // imshow( "gray", src_gray );

  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );


  waitKey(0);

  vector<Vec3f> circles;

  std::cout << "rows: " << src_gray.rows << ". rows divided: " << src_gray.rows/90 << std::endl;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/90, 200, 100, 0, 0 );

  std::cout << "There are " << circles.size() << "." << std::endl;

  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }

  /// Show your results
  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform Demo", src );

  waitKey(0);
  return 0;
}