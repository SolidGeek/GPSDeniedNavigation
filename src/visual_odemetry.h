#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include "feature_tracker.h"

#define M_PI 3.14159265358979323846

class VisualOdemetry
{

public:

    /**
     * Constructor
    */
    VisualOdemetry();

    void config( int frame_width, int frame_height, int interval = 20, float scale = 1.0 );

    /*
     * Calculate optical flow in pixels
     */
    void compute_dense_flow( cv::Mat frame, float dt = 0 );

    void compute_sparse_flow( cv::Mat frame, float dt = 0 );

    /*
     * Get the frame with visualised flow
     */
    cv::Mat get_frame( void );

    /*
     * Calculate pixel velocity (px/s) and velocity (m/s) if distance is supplied
     */
    float get_vel_x( float distance = 0 );
    float get_vel_y( float distance = 0 );

private:

    FeatureTracker * tracker = NULL; 

    cv::Mat frame;
    cv::Mat frame_gfx;
    cv::Mat frame_prev;

    cv::Size size;
    int interval;

    float of_x;
    float of_y;

    float vo_x;
    float vo_y;

    // Parameters used for featuretracker
    std::vector<int> status;
    std::vector<cv::Point2f> features_current, features_previous;

    /*
     * Process the raw frame according to scale and/or color
     */
    void process_frame( cv::Mat raw, cv::Mat &output );
    
    /*
     * Compute optical flow in pixels / second. Sum is the total pixel change while count is the total number of points for the frame.
    */
    float compute_pixel_velocity( float sum, int count, float dt );

    float compute_velocity( float flow, int axis_length, float distance );

    /*
     * Parameters for opencv's calcOpticalFlowFarneback
    */
    const float pyr_scale = 0.5;
    const int levels      = 5;
    const int winsize     = 13;
    const int iterations  = 10;
    const int poly_n      = 5;
    const float poly_sigma  = 1.1;

};

#endif
