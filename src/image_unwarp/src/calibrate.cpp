#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <stack>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static void on_mouse_action(int event, int x, int y, int, void *);

class ImageUnwarpCalibrator {
    string WINDOW_NAME_ = "Calibration Image";
    int MAX_POINT_RADIUS_ = 4;

    // ROS node and image transport handles
    ros::NodeHandle *nh_;

    // Calibration parameters
    int center_x_, center_y_, width_, height_;
    string calibration_image_name_;
    vector<double> polynomial_;

    // Calibration variables used for user input
    stack<Mat> zoom_stack_;
    vector<Rect> crop_vector_;
    vector<Point> points_;
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
        zoom_stack_.push(imread(calibration_image_name_, CV_LOAD_IMAGE_COLOR));
        if (zoom_stack_.top().empty()) {
            ROS_ERROR("Could not find image '%s'",
                    calibration_image_name_.c_str());
            return false;
        }
        crop_vector_.push_back(Rect(0, 0, zoom_stack_.top().size().width,
                zoom_stack_.top().size().height));

        // Open a window and set its mouse callback
        namedWindow(WINDOW_NAME_, CV_WINDOW_NORMAL);
        setMouseCallback(WINDOW_NAME_, on_mouse_action, NULL);

#include <algorithm>
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
    void redraw() {
        Mat image;
        if (zoom_stack_.size() <= 1) {
            if (help_showing_)
                image = get_image(zoom_stack_.top(), true, true);
            else
                image = get_image(zoom_stack_.top(), true, false);
        } else {
            image = get_image(zoom_stack_.top(), false, false);
        }
        image = draw_points(image, std::max(0,
                (int)(MAX_POINT_RADIUS_ - zoom_stack_.size())));
        imshow(WINDOW_NAME_, image);
    }

    /**
     */
    Mat get_image(Mat input_image, bool show_toggle, bool show_help) {
        Mat help_image = input_image.clone();
        int font = FONT_HERSHEY_SIMPLEX;
        int size = 1;
        Scalar color(255, 255, 0);
        int thickness = 3;
        int line = 8;

        //
        if (show_toggle) {
            putText(help_image, "Toggle help: 'h'", Point(10, 40), font, size,
                    color, thickness, line);
            stringstream converter;
            converter << "Points added: " << points_.size();
            putText(help_image, converter.str(), Point(885, 40), font, size,
                    color, thickness, line);
        }

        //
        if (show_help) {
            putText(help_image, "Add point: left click", Point(10, 80), font,
                    size, color, thickness, line);
            putText(help_image, "Remove point: right click", Point(10, 120),
                    font, size, color, thickness, line);
            putText(help_image, "Zoom in: CTRL + left click", Point(10, 160),
                    font, size, color, thickness, line);
            putText(help_image, "Zoom out: CTRL + right click", Point(10, 200),
                    font, size, color, thickness, line);
            putText(help_image, "Finalize selection: ESC", Point(10, 240), font,
                    size, color, thickness, line);
            putText(help_image, "*Help text disappears while zoomed in",
                    Point(10, 280), font, size, color, thickness, line);
        }
        return help_image;
    }

    /**
     */
    Mat draw_points(Mat image, int circle_radius) {
        //
        Rect current_crop;
        current_crop.x = current_crop.y = 0;
        for (int i = 0; i < crop_vector_.size(); i++) {
            current_crop.x += crop_vector_[i].x;
            current_crop.y += crop_vector_[i].y;
        }
        current_crop.width = crop_vector_.back().width;
        current_crop.height = crop_vector_.back().height;

        //
        for (int i = 0; i < points_.size(); i++) {
            if (points_[i].x >= current_crop.x
                    && points_[i].x < current_crop.x + current_crop.width
                    && points_[i].y >= current_crop.y
                    && points_[i].y < current_crop.y + current_crop.height) {
                Point point(points_[i].x - current_crop.x,
                        points_[i].y - current_crop.y);
                Scalar color(255, 255, 0);
                if (circle_radius > 0)
                    circle(image, point, circle_radius, color);
                else
                    line(image, point, point, color);
            }
        }
        return image;
    }

    /**
     */
    void zoom_in(int x, int y) {
        //
        Mat original_image = zoom_stack_.top();
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
            Rect crop_dimensions(min_x, min_y, max_x - min_x, max_y - min_y);
            crop_vector_.push_back(crop_dimensions);
            zoom_stack_.push(zoomed_image(crop_dimensions));
            redraw();
        }
    }

    /**
     */
    void zoom_out(int x, int y) {
        // If only the original image remains we can't zoom out any further
        if (crop_vector_.size() <= 1 || zoom_stack_.size() <= 1)
            return;

        // Zoom out by going to the last saved image to retain resolution
        crop_vector_.pop_back();
        zoom_stack_.pop();
        redraw();
    }

    /**
     */
    void add_point(int x, int y) {
        //
        while (crop_vector_.size() > 1) {
            Rect crop = crop_vector_.back();
            x += crop.x;
            y += crop.y;
            crop_vector_.pop_back();
            zoom_stack_.pop();
        }

        //
        points_.push_back(Point(x, y));
        redraw();
    }

    /**
     */
    void pop_point() {
        if (points_.size() > 0) {
            points_.pop_back();
            redraw();
        }
    }
} calibrator;

/**
 */
static void on_mouse_action(int event, int x, int y, int flags, void *) {
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

    //ros::NodeHandle nh;

    //// Set calibration parameters
    //std::vector<double> polynomial = std::vector<double>();
    //polynomial.push_back(-0.08001114767);
    //polynomial.push_back(1.274507708);
    //polynomial.push_back(-0.001088811456);
    //polynomial.push_back(0.0000001119609517);
    //polynomial.push_back(0.0000000002040845432);
    //nh.setParam("/image_unwarp/center_x", 999);
    //nh.setParam("/image_unwarp/center_y", 571);
    //nh.setParam("/image_unwarp/width", 2048);
    //nh.setParam("/image_unwarp/height", 1088);
    //nh.setParam("/image_unwarp/polynomial", polynomial);
}
