#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



using namespace cv;



/** @function main */
int main(int argc, char** argv)
{


	/*

	The input is the image message pointer, as well as an optional encoding argument. 
	The encoding refers to the destination CvImage.

	toCvCopy creates a copy of the image data from the ROS message, even when the source 
	and destination encodings match. However, you are free to modify the returned CvImage.

	toCvShare will point the returned cv::Mat at the ROS message data, avoiding a copy, 
	if the source and destination encodings match. As long as you hold a copy of the 
	returned CvImage, the ROS message data will not be freed. If the encodings do not 
	match, it will allocate a new buffer and perform the conversion. You are not 
	permitted to modify the returned CvImage, as it may share data with the ROS image 
	message, which in turn may be shared with other callbacks. Note: the second overload 
	of toCvShare is more convenient when you have a pointer to some other message type 
	(e.g. stereo_msgs/DisparityImage) that contains a sensor_msgs/Image you want to convert. 


	*/
	// Case 1: Always copy, returning a mutable CvImage
	CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
	                	const std::string& encoding = std::string());
	CvImagePtr toCvCopy(const sensor_msgs::Image& source,
	                	const std::string& encoding = std::string());

}