#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
    ImageConverter() : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
    }

    ~ImageConverter() {}

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "image_unwarp");
    ros::NodeHandle nh;

    // Get calibration parameters
    if (!(nh.hasParam("center_x") && nh.hasParam("center_y")
            && nh.hasParam("polynomial"))) {
        ROS_ERROR("Could not find calibration parameters. Did you run image"
                "calibrate?");
    }
    int center_x, center_y;
    std::vector<double> polynomial;
    nh.getParam("center_x", center_x);
    nh.getParam("center_y", center_y);
    nh.getParam("polynomial", polynomial);

    image_transport::ImageTransport it(nh);

    ros::spin();
    return 0;
}
