#include "ros/ros.h"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;

    // Set calibration parameters
    std::vector<double> polynomial = std::vector<double>();
    polynomial.push_back(5);
    polynomial.push_back(25);
    polynomial.push_back(3);
    nh.setParam("/image_unwarp/center_x", 512);
    nh.setParam("/image_unwarp/center_y", 512);
    nh.setParam("/image_unwarp/polynomial", polynomial);
}
