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

#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace cv;

static void on_mouse_action(int event, int x, int y, int, void *);

class ImageUnwarpCalibrator {
    string WINDOW_NAME_ = "Calibration Image";
    int MAX_POINT_RADIUS_ = 4;
    int DEFAULT_DEGREE_ = 4;

    // ROS node and image transport handles
    ros::NodeHandle *nh_;

    // Calibration parameters
    Point center_;
    int width_, height_, degree_;
    string calibration_image_name_;
    vector<double> polynomial_;

    // Calibration variables used for user input
    stack<Mat> zoom_stack_;
    vector<Rect> crop_vector_;
    vector<Point> points_;
    vector<Point> sizing_points_;
    bool setup_complete_, help_showing_;

    public:
    ImageUnwarpCalibrator() : setup_complete_(false) {}

    ~ImageUnwarpCalibrator() {
        delete nh_;
    }

    /**
     * Setup the parameters for the node.
     *
     * Returns:
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
        nh_->getParam("/calibrate/center_x", center_.x);
        nh_->getParam("/calibrate/center_y", center_.y);
        nh_->getParam("/calibrate/calibration_image",
                calibration_image_name_);
        nh_->param("/calibrate/degree", degree_, DEFAULT_DEGREE_);

        // Open the calibration image
        zoom_stack_.push(imread(calibration_image_name_, CV_LOAD_IMAGE_COLOR));
        if (zoom_stack_.top().empty()) {
            ROS_ERROR("Could not find image '%s'",
                    calibration_image_name_.c_str());
            return false;
        }
        crop_vector_.push_back(Rect(0, 0, zoom_stack_.top().size().width,
                zoom_stack_.top().size().height));

        // Set parameters for image_unwarp node
        nh_->setParam("/image_unwarp/center_x", center_.x);
        nh_->setParam("/image_unwarp/center_y", center_.y);
        nh_->setParam("/image_unwarp/width", zoom_stack_.top().size().width);
        nh_->setParam("/image_unwarp/height", zoom_stack_.top().size().height);

        // Open a window and set its mouse callback
        namedWindow(WINDOW_NAME_, CV_WINDOW_NORMAL);
        setMouseCallback(WINDOW_NAME_, on_mouse_action, NULL);

        setup_complete_ = true;
        return true;
    }

    /**
     * Calibrate based on the image provided in the parameters.
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

        // Validate that enough points were found
        if (points_.size() < degree_ + 1) {
            ROS_ERROR("Not enough points selected for calibration");
            return false;
        }

        // Validate that the 3 sizing points were chosen
        if (sizing_points_.size() != 3) {
            ROS_ERROR("3 sizing points must be chosen");
            return false;
        }

        // Get the distance offset to the center and the distance between the
        // sizing points
        double min_dist = cart_dist(points_[0], center_);
        double size1 = cart_dist(sizing_points_[0], sizing_points_[1]);
        double size2 = cart_dist(sizing_points_[1], sizing_points_[2]);
        double size3 = cart_dist(sizing_points_[1], points_[0]);
        cout << size1 << " " << size2 << " " << size3 << endl;
        double size = (size1 + size2 + size2) / 3;

        // Get a warped image -> calibrated image mapping of distance
        vector<Point2f> scaled_points;
        scaled_points.push_back(Point(0, 0));
        for (int i = 0; i < points_.size(); i++)
            scaled_points.push_back(Point(min_dist + size * i,
                    cart_dist(points_[i], center_)));

        // Transform the mapping of distance into matrices for regression
        // using least squares (see Wikipedia "polynomial regression")
        MatrixXd x(scaled_points.size(), degree_ + 1);
        VectorXd y(scaled_points.size());
        double value;
        for (int i = 0; i < scaled_points.size(); i++) {
            value = 1;
            for (int j = 0; j <= degree_; j++) {
                x(i, j) = value;
                value *= scaled_points[i].x;
            }
            y[i] = scaled_points[i].y;
        }

        // Calculate the coefficients
        MatrixXd x_transposed = x.transpose();
        VectorXd a = (x_transposed * x).inverse() * x_transposed * y;

        // Make the coefficients ROS parameters for the image_unwarp node
        vector<double> coefficients;
        for (int i = 0; i < a.size(); i++)
            coefficients.push_back(a[i]);
        nh_->setParam("/image_unwarp/polynomial", coefficients);

        // Clean up
        destroyWindow(WINDOW_NAME_);
        return true;
    }

    /**
     * Get the cartesian distance between two points.
     *
     * Args:
     *  a: first point
     *  b: second point
     *
     * Returns:
     *  Cartesian distance between two points
     */
    double cart_dist(Point a, Point b) {
        return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
    }

    /**
     * Redraw the image in the window.
     */
    void redraw() {
        Mat image;
        if (zoom_stack_.size() <= 1) {
            if (help_showing_)
                image = get_image(zoom_stack_.top(), false, true);
            else
                image = get_image(zoom_stack_.top(), false, false);
        } else {
            image = get_image(zoom_stack_.top(), true, false);
        }
        image = draw_points(image, std::max(0,
                (int)(MAX_POINT_RADIUS_ - zoom_stack_.size())));
        imshow(WINDOW_NAME_, image);
    }

    /**
     * Adds text to the calibration image based on user input.
     *
     * Args:
     *  input_image: calibration image that might be zoomed
     *  is_zoomed: whether or not the camera is zoomed
     */
    Mat get_image(Mat input_image, bool is_zoomed, bool show_help) {
        Mat help_image = input_image.clone();
        int font = FONT_HERSHEY_SIMPLEX;
        int size = 1;
        Scalar color(255, 255, 0);
        int thickness = 3;
        int line = 8;

        // If not zoomed in show the help messages
        if (!is_zoomed) {
            putText(help_image, "Toggle help: h", Point(10, 40), font, size,
                    color, thickness, line);
            stringstream converter;
            converter << "Points added: " << points_.size();
            putText(help_image, converter.str(), Point(885, 40), font, size,
                    color, thickness, line);
            converter.str("");
            converter << "Sizing points added: " << sizing_points_.size();
            putText(help_image, converter.str(), Point(1685, 40), font, size,
                    color, thickness, line);
        }

        // If the user specifies show the extra help messages
        if (show_help) {
            putText(help_image, "Zoom in: left click", Point(10, 80),
                    font, size, color, thickness, line);
            putText(help_image, "Zoom out: right click", Point(10, 120),
                    font, size, color, thickness, line);
            putText(help_image, "Add point: CTRL + left click", Point(10, 160),
                    font, size, color, thickness, line);
            putText(help_image, "Remove point: CTRL + right click",
                    Point(10, 200), font, size, color, thickness, line);
            putText(help_image, "Add sizing point: SHIFT + left click",
                    Point(10, 240), font,
                    size, color, thickness, line);
            putText(help_image, "Remove sizing point: SHIFT + right click",
                    Point(10, 280), font, size, color, thickness, line);
            putText(help_image, "Finalize selection: ESC", Point(10, 320), font,
                    size, color, thickness, line);
            putText(help_image, "Hints:", Point(10, 380), font, size, color,
                    thickness, line);
            putText(help_image, "* Help text disappears while zoomed in",
                    Point(10, 420), font, size, color, thickness, line);
            putText(help_image, "* Click the upper left of pixels when adding",
                    Point(10, 460), font, size, color, thickness, line);
            putText(help_image,
                    "* The points MUST be ordered by proximity to the center",
                    Point(10, 500), font, size, color, thickness, line);
            putText(help_image,
                    "* The sizing points must be the three closest points to",
                    Point(10, 540), font, size, color, thickness, line);
            putText(help_image,
                    "  the center that are perpindicular to regular points",
                    Point(10, 580), font, size, color, thickness, line);
            putText(help_image,
                    "  where the center point is picked second (order does matter)",
                    Point(10, 620), font, size, color, thickness, line);
        }
        return help_image;
    }

    /**
     * Draws all of the points that the user has selected and sizes them based
     * on zoom level.
     *
     * Args:
     *  image: image to draw on
     *  circle_radius: radius of the points
     *
     * Returns:
     *  Image with circles drawn on it
     */
    Mat draw_points(Mat image, int circle_radius) {
        // Determine the crop offset if the image is zoomed
        Rect current_crop;
        current_crop.x = current_crop.y = 0;
        for (int i = 0; i < crop_vector_.size(); i++) {
            current_crop.x += crop_vector_[i].x;
            current_crop.y += crop_vector_[i].y;
        }
        current_crop.width = crop_vector_.back().width;
        current_crop.height = crop_vector_.back().height;

        // Draw the center
        Point center_point = center_;
        int center_circle_radius = max(1, circle_radius + 1);
        center_point.x -= current_crop.x;
        center_point.y -= current_crop.y;
        Scalar center_color(0, 0, 255);
        circle(image, center_point, center_circle_radius, center_color,
                CV_FILLED);

        // Draw the points
        for (int i = 0; i < points_.size(); i++) {
            if (points_[i].x >= current_crop.x
                    && points_[i].x < current_crop.x + current_crop.width
                    && points_[i].y >= current_crop.y
                    && points_[i].y < current_crop.y + current_crop.height) {
                Point point(points_[i].x - current_crop.x,
                        points_[i].y - current_crop.y);
                Scalar color(255, 255, 0);
                if (circle_radius > 0)
                    circle(image, point, circle_radius, color, CV_FILLED);
                else
                    line(image, point, point, color);
            }
        }

        // Draw the sizing points
        for (int i = 0; i < sizing_points_.size(); i++) {
            if (sizing_points_[i].x >= current_crop.x
                    && sizing_points_[i].x < current_crop.x + current_crop.width
                    && sizing_points_[i].y >= current_crop.y
                    && sizing_points_[i].y < current_crop.y + current_crop.height) {
                Point point(sizing_points_[i].x - current_crop.x,
                        sizing_points_[i].y - current_crop.y);
                Scalar color(0, 255, 0);
                if (circle_radius > 0)
                    circle(image, point, circle_radius, color, CV_FILLED);
                else
                    line(image, point, point, color);
            }
        }
        return image;
    }



    /**
     * Zoom into the image based on a user selected center point.
     *
     * Args:
     *  x: x coordinate of zoom center
     *  y: y coordinate of zoom center
     */
    void zoom_in(int x, int y) {
        // Validate the center point
        Mat original_image = zoom_stack_.top();
        int w = original_image.size().width;
        int h = original_image.size().height;
        if (x < 0 || y < 0 || x >= w || y >= h)
            return;

        // Calculate the crop dimensions
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

        // Crop the image if the crop dimensions are valid
        if (min_x < max_x && min_y < max_y) {
            Mat zoomed_image = original_image.clone();
            Rect crop_dimensions(min_x, min_y, max_x - min_x, max_y - min_y);
            crop_vector_.push_back(crop_dimensions);
            zoom_stack_.push(zoomed_image(crop_dimensions));
            redraw();
        }
    }

    /**
     * Zoom out of the calibration image.
     */
    void zoom_out() {
        // If only the original image remains we can't zoom out any further
        if (crop_vector_.size() <= 1 || zoom_stack_.size() <= 1)
            return;

        // Zoom out by going to the last saved image to retain resolution
        crop_vector_.pop_back();
        zoom_stack_.pop();
        redraw();
    }

    /**
     * Add a point to the points array.
     *
     * Args:
     *  x: x coordinate of the point to be added
     *  y: y coordinate of the point to be added
     */
    void add_point(int x, int y, bool is_sizing_point) {
        // Calculate the zoom offset
        Rect crop;
        for (int i = 0; i < crop_vector_.size(); i++) {
            crop = crop_vector_[i];
            x += crop.x;
            y += crop.y;
        }

        // Add the point and redraw so that it is displayed
        if (is_sizing_point)
            sizing_points_.push_back(Point(x, y));
        else
            points_.push_back(Point(x, y));
        redraw();
    }

    /**
     * Remoave a point from the points array.
     */
    void pop_point(bool is_sizing_point) {
        if (is_sizing_point && sizing_points_.size() > 0)
            sizing_points_.pop_back();
        else if (!is_sizing_point && points_.size() > 0)
            points_.pop_back();
        redraw();
    }
} calibrator;

/**
 * Call calibrator functions based on mouse events.
 *
 * Args:
 *  event: mouse event that occured
 *  x: x coordinate of event
 *  y: y coordinate of event
 *  flags: extra information about the event
 *  void *: unused argument
 */
static void on_mouse_action(int event, int x, int y, int flags, void *) {
    switch (event) {
        case EVENT_LBUTTONDOWN:
            if (flags == EVENT_FLAG_CTRLKEY)
                calibrator.add_point(x, y, false);
            else if (flags == EVENT_FLAG_SHIFTKEY)
                calibrator.add_point(x, y, true);
            else
                calibrator.zoom_in(x, y);
            break;
        case EVENT_RBUTTONDOWN:
            if (flags == EVENT_FLAG_CTRLKEY)
                calibrator.pop_point(false);
            else if (flags == EVENT_FLAG_SHIFTKEY)
                calibrator.pop_point(true);
            else
                calibrator.zoom_out();
            break;
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "calibrate");

    // Setup and run the calibrator
    if (!calibrator.setup())
        return EXIT_FAILURE;
    if (!calibrator.calibrate())
        return EXIT_FAILURE;

    // Write the calibration data to file
    system("cd ~/IGVC2014/launch_files/rosparams; rosparam get /image_unwarp > calibration.yaml");

    return EXIT_SUCCESS;
    ros::NodeHandle nh;
}
