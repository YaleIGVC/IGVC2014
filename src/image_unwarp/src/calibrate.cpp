#include "ros/ros.h"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;

    // Set calibration parameters
    std::vector<double> polynomial = std::vector<double>();
    polynomial.push_back(5);
    polynomial.push_back(0.001); // Linear term
    polynomial.push_back(5);
    nh.setParam("/image_unwarp/center_x", 1024);
    nh.setParam("/image_unwarp/center_y", 544);
    nh.setParam("/image_unwarp/width", 2048);
    nh.setParam("/image_unwarp/height", 1088);
    nh.setParam("/image_unwarp/polynomial", polynomial);
}
