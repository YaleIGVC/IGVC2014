#include "ros/ros.h"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;

    // Set calibration parameters
    std::vector<double> polynomial = std::vector<double>();
    polynomial.push_back(1);
    polynomial.push_back(.01);
    polynomial.push_back(.001);
    nh.setParam("/image_unwarp/center_x", 512);
    nh.setParam("/image_unwarp/center_y", 512);
    nh.setParam("/image_unwarp/width", 512);
    nh.setParam("/image_unwarp/height", 512);
    nh.setParam("/image_unwarp/polynomial", polynomial);
}
