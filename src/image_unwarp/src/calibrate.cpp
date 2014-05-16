#include "ros/ros.h"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;

    // Set calibration parameters
    std::vector<double> polynomial = std::vector<double>();
    polynomial.push_back(-0.08001114767);
    polynomial.push_back(1.274507708);
    polynomial.push_back(-0.001088811456);
    polynomial.push_back(0.0000001119609517);
    polynomial.push_back(0.0000000002040845432);
    nh.setParam("/image_unwarp/center_x", 999);
    nh.setParam("/image_unwarp/center_y", 571);
    nh.setParam("/image_unwarp/width", 2048);
    nh.setParam("/image_unwarp/height", 1088);
    nh.setParam("/image_unwarp/polynomial", polynomial);
}
