#include <cstdlib>
#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;

ros::NodeHandle nh("~");
image_transport::ImageTransport it(nh);

// Configuration parameters
string input_topic, output_topic;

// Calibration parameters
int center_x, center_y;
vector<double> polynomial;

/**
 * Find the cartesian distance between two points.
 *
 * Args:
 *  x1: first x coordinate
 *  y1: first y coordinate
 *  x2: second x coordinate
 *  y2: second y coordinate
 *
 * Returns:
 *  Cartesian distance between (x1, y1) and (x2, y2)
 */
double cart_dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * Find the cartesian angle between two points.
 *
 * Args: x1: first x coordinate
 * y1: first y coordinate
 * x2: second x coordinate
 * y2: second y coordinate
 *
 * Returns:
 *  Cartesian angle between (x1, y1) and (x2, y2)
 */
double cart_angle(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1);
}

/**
 * Calculate the polynomial function of a given set of coefficients and value.
 *
 * Args:
 *  polynomial: vector of coefficients, starting with the 0th (constant) term
 *  x: value to compute the polynomial function of
 *
 * Returns:
 *  f(x) where f is defined by the coefficients in polynomial
 */
double polynomial_function(vector<double> polynomial, double x) {
    double sum = 0;
    double temp_x = 1;
    for (size_t i = 0; i < polynomial.size(); i++) {
        sum += polynomial[i] * temp_x;
        temp_x *= x;
    }
    return sum;
}

/**
 * Callback function to unwarp a ROS image, then publish it to a new topic.
 *
 * Args:
 *  msg: ROS message containing the image to process
 */
void process_image(const sensor_msgs::ImageConstPtr &msg) {
    // Create an image publisher
    image_transport::Publisher image_pub = it.advertise(
            "/image_unwarp/output_video", 1);

    // Convert the message image to openCV format
    cv_bridge::CvImagePtr input_image;
    try {
        input_image = cv_bridge::toCvCopy(msg,
                sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Initialize an output image
    cv_bridge::CvImagePtr output_image;
    output_image->header = input_image->header;
    output_image->encoding = input_image->encoding;

    // Fill the output image based on input_image and the calibration parameters
    int width = input_image->image.cols;
    int height = input_image->image.rows;
    int channels = input_image->image.channels();
    uint8_t *input_data = (uint8_t *)input_image->image.data;
    uint8_t *output_data = (uint8_t *)output_image->image.data;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            // Calculate the distance and angle from (i, j) to the center
            double dist = cart_dist(i, j, center_x, center_y);
            double theta = cart_angle(i, j, center_x, center_y);

            // Calculate the (x, y) of the point that should be at (i, j)
            double new_dist = polynomial_function(polynomial, dist);
            int x = round(center_x + new_dist * cos(theta));
            int y = round(center_y + new_dist * sin(theta));

            // If that point is defined in the input, propogate it to the output
            if (x >= 0 && x < width && y >= 0 && y < height) {
                uchar *pixel = input_data + (x + y * width) * channels;
                uchar *new_pixel = output_data + (i + j * width) * channels;
                memcpy(new_pixel, pixel, channels * sizeof(uchar));
            }
        }
    }

    // Publish the image
    image_pub.publish(output_image->toImageMsg());
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "image_unwarp");

    // Get private configuration parameters
    nh.param<string>("input_topic", input_topic, "/camera/image_raw");
    nh.param<string>("output_topic", output_topic,
            "/image_unwarp/output_video");

    // Get calibration parameters
    if (!(nh.hasParam("center_x") && nh.hasParam("center_y")
            && nh.hasParam("polynomial"))) {
        ROS_ERROR("Could not find calibration parameters. Did you run image"
                "calibrate?");
        return EXIT_FAILURE;
    }
    nh.getParam("/image_unwarp/center_x", center_x);
    nh.getParam("/image_unwarp/center_y", center_y);
    nh.getParam("/image_unwarp/polynomial", polynomial);

    // Subscribe to incoming images
    image_transport::Subscriber image_sub = it.subscribe(input_topic, 1,
            process_image);

    ros::spin();
    return EXIT_SUCCESS;
}
