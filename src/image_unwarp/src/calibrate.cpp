#include "ros/ros.h"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;

    // Set calibration parameters
    std::vector<double> polynomial = std::vector<double>();
    polynomial.push_back(0.0);
    polynomial.push_back(1.1);
    polynomial.push_back(2.2);
    nh.setParam("center_x", 0);
    nh.setParam("center_y", 0);
    nh.setParam("polynomial", polynomial);
}
