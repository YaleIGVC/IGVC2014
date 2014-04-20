#include <cstdlib>
#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <ctime>
#include <sys/time.h>

#include <ros/console.h>

using namespace std;

class ImageUnwarper {
    // ROS node and image transport handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // ROS subscriber and publisher
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // Configuration parameters
    string input_topic_, output_topic_;

    // Calibration parameters
    int center_x_, center_y_;
    vector<double> polynomial_;

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

    uint64 get_time() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64 ret = tv.tv_usec;
        ret /= 1000;
        ret += (tv.tv_sec * 1000);
        return ret;
    }

    public:
    ImageUnwarper() : nh_("~"), it_(nh_) {}

    bool setup() {
        // Get private configuration parameters
        nh_.param<string>("input_topic", input_topic_, "/raw_image");
        nh_.param<string>("output_topic", output_topic_,
                "/image_unwarp/output_video");

        // Get calibration parameters
        if (!(nh_.hasParam("/image_unwarp/center_x") && nh_.hasParam("/image_unwarp/center_y")
                && nh_.hasParam("/image_unwarp/polynomial"))) {
            ROS_ERROR("Could not find calibration parameters. Did you run "
                    "image_unwarp calibrate?");
            return false;
        }
        nh_.getParam("/image_unwarp/center_x", center_x_);
        nh_.getParam("/image_unwarp/center_y", center_y_);
        nh_.getParam("/image_unwarp/polynomial", polynomial_);


        // Subscribe to incoming images and publish unwarped images
        image_sub_ = it_.subscribe(input_topic_, 1,
                &ImageUnwarper::process_image, this);
        image_pub_ = it_.advertise(output_topic_, 1);
        return true;
    }

    /**
     * Callback function to unwarp a ROS image, then publish it to a new topic.
     *
     * Args:
     *  msg: ROS message containing the image to process
     */
    void process_image(const sensor_msgs::ImageConstPtr &msg) {
        // Convert the message image to openCV format (7ms)
        cv_bridge::CvImageConstPtr input_image;
        try {
            input_image = cv_bridge::toCvShare(msg,
                    sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Initialize an output image (7ms)
        cv_bridge::CvImagePtr output_image;
        try {
            output_image = cv_bridge::toCvCopy(msg,
                    sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Fill the output image based on input_image and the calibration parameters (680ms)
        uint64 t1 = get_time();
        int width = input_image->image.cols;
        int height = input_image->image.rows;
        int channels = input_image->image.channels();
        ROS_DEBUG("Width %d, height %d\n",width, height);
        uint8_t *input_data = (uint8_t *)input_image->image.data;
        uint8_t *output_data = (uint8_t *)output_image->image.data;
        uint64 t2 = get_time();
        cout << t1 << " " << t2 << " " << t2 - t1 << endl;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                // Calculate the distance and angle from (i, j) to the center
                double dist = cart_dist(i, j, center_x_, center_y_); // (44ms)
                double theta = cart_angle(i, j, center_x_, center_y_); // (98ms)

                // Calculate the (x, y) of the point that should be at (i, j)
                double new_dist = polynomial_function(polynomial_, dist); // (341ms)
                int x = round(center_x_ + new_dist * cos(theta)); // (75ms)
                int y = round(center_y_ + new_dist * sin(theta)); // (75ms)

                // If that point is defined in the input, propogate it to the output (<7ms)
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    uchar *pixel = input_data + (x + y * width) * channels;
                    uchar *new_pixel = output_data + (i + j * width) * channels;
                    memcpy(new_pixel, pixel, channels * sizeof(uchar));
                }
            }
        }

        // Publish the image (6ms)
        image_pub_.publish(output_image->toImageMsg());
    }
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "image_unwarp");

    ImageUnwarper iu;
    if (!iu.setup())
        return EXIT_FAILURE;

    ros::spin();
    return EXIT_SUCCESS;
}
