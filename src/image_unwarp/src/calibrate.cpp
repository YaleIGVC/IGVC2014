
#include <cstdlib>
#include <iostream>
#include <string>
#include <stack>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static void on_mouse_zoom(int event, int x, int y, int, void *);

class ImageUnwarpCalibrator {
    string WINDOW_NAME_ = "Calibration Image";

    // ROS node and image transport handles
    ros::NodeHandle *nh_;

    // Calibration parameters
    int center_x_, center_y_, width_, height_;
    string calibration_image_name_;
    vector<double> polynomial_;

    // Calibration variables used for user input
    stack<Mat> zoom_stack;
    vector<Point2f> points_;
    bool setup_complete_, help_showing_;

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

    public:
    ImageUnwarpCalibrator() : setup_complete_(false) {}

    ~ImageUnwarpCalibrator() {
        delete nh_;
    }

    /**
     * Setup the parameters for the node.
     *
     * Returns:sfdsfdsf
     *  Whether or not setup was successful
     */
    bool setup() {
        // Get calibration parameters
        nh_ = new ros::NodeHandle("~");
        if (!(nh_->hasParam("/calibrate/center_x")
                && nh_->hasParam("/calibrate/center_y")
                && nh_->hasParam("/calibrate/calibration_image"))) {
            ROS_ERROR("Could not find calibration parameters. Did you pass"
                    "them as ROS parameters?");
            return false;
        }
        nh_->getParam("/calibrate/center_x", center_x_);
        nh_->getParam("/calibrate/center_y", center_y_);
        nh_->getParam("/calibrate/calibration_image",
                calibration_image_name_);

        // Open the calibration image
        zoom_stack.push(imread(calibration_image_name_, CV_LOAD_IMAGE_COLOR));
        if (zoom_stack.top().empty()) {
            ROS_ERROR("Could not find image '%s'",
                    calibration_image_name_.c_str());
            return false;
        }

        // Open a window and set its mouse callback
        namedWindow(WINDOW_NAME_, CV_WINDOW_NORMAL);
        setMouseCallback(WINDOW_NAME_, on_mouse_zoom, NULL);

        setup_complete_ = true;
        return true;
    }

    /**
     */
    bool calibrate() {
        // Wait until the Escape Key is hit
        int last_key;
        help_showing_ = true;
        redraw();
        while ((last_key = waitKey(0)) != 27) {
            switch (last_key) {
                case 'h':
                    help_showing_ = !help_showing_;
                    redraw();
                    break;
            }
        }

        // Clean up
        destroyWindow(WINDOW_NAME_);

        return true;
    }

    /**
     */
    Mat get_help_image(Mat input_image) {
        Mat help_image = input_image.clone();
        putText(help_image, "HELP", Point(10, 10), FONT_HERSHEY_SIMPLEX, 2, Scalar::all(255), 3, 8);
        return help_image;
    }

    /**
     */
    void redraw() {
        if (help_showing_)
            imshow(WINDOW_NAME_, get_help_image(zoom_stack.top()));
        else
            imshow(WINDOW_NAME_, zoom_stack.top());
    }

    /**
     */
    void zoom_in(int x, int y) {
        //
        Mat original_image = zoom_stack.top();
        int w = original_image.size().width;
        int h = original_image.size().height;
        if (x < 0 || y < 0 || x >= w || y >= h)
            return;

        //
        int min_x = x - w / 4;
        int max_x = x + w / 4;
        int min_y = y - h / 4;
        int max_y = y + h / 4;
        if (min_x < 0) {
            min_x = 0;
            max_x = w / 2;
        } else if (max_x >= w) {
            max_x = w;
            min_x = w - w / 2;
        }
        if (min_y < 0) {
            min_y = 0;
            max_y = h / 2;
        } else if (max_y >= h) {
            max_y = h;
            min_y = h - h / 2;
        }

        //
        if (min_x < max_x && min_y < max_y) {
            Mat zoomed_image = original_image.clone();
            Rect cropping_dimensions(min_x, min_y, max_x - min_x, max_y - min_y);
            zoom_stack.push(zoomed_image(cropping_dimensions));
            redraw();
        }
    }

    /**
     */
    void zoom_out(int x, int y) {
        // If only the original image remains we can't zoom out any further
        if (zoom_stack.size() <= 1)
            return;

        // Zoom out by going to the last saved image to retain resolution
        zoom_stack.pop();
        redraw();
    }

    /**
     */
    void add_point(int x, int y) {
    }

    /**
     */
    void pop_point() {
    }
} calibrator;

/**
 */
static void on_mouse_zoom(int event, int x, int y, int flags, void *) {
    switch (event) {
        case EVENT_LBUTTONDOWN:
            if (flags == EVENT_FLAG_CTRLKEY)
                calibrator.zoom_in(x, y);
            else
                calibrator.add_point(x, y);
            break;
        case EVENT_RBUTTONDOWN:
            if (flags == EVENT_FLAG_CTRLKEY)
                calibrator.zoom_out(x, y);
            else
                calibrator.pop_point();
            break;
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");

    if (!calibrator.setup())
        return EXIT_FAILURE;
    if (!calibrator.calibrate())
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
