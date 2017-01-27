#include "ros/ros.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "monkeys.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "hello_world");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  cv::Mat_<unsigned char> blank = cv::Mat_<unsigned char>::zeros(100, 100);

  cv::namedWindow("foo");
  cv::imshow("foo", blank);

  std::cout << "the monkeys is: " << get_monkeys() << "\n";

  while (ros::ok()) {
    std::cout << "Hello, world!\n";
    cv::waitKey(1);
    loop_rate.sleep();
  }

  return 0;

}
