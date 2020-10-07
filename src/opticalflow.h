#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include "featureTracker.h"

#define M_PI 3.14159265358979323846

class OpticalFlow
{

public:

    /**
     * Constructor
    */
    OpticalFlow( int frame_width, int frame_height, int scaledown = 1, int interval = 16, float fov = 64.0 );

    /**
     * Calculate pixel flow and convert to velocity (m/s) if distance is supplied
    */
    cv::Point2f compute_dense_flow( cv::Mat frame, float dt, float distance = 0 );

    cv::Point2f compute_sparse_flow( cv::Mat raw, float dt, float distance = 0 ); 

    cv::Mat get_frame( void );

    cv::Point2f get_flow( void );

    cv::Size get_size( void );

private:

    FeatureTracker * tracker = NULL; 

    cv::Mat grid; 

    cv::Mat frame;
    cv::Mat prev_gray;
    cv::Size size;
    cv::Point2f flow;
    cv::Point2f velocity;

    std::vector<int> status;
    std::vector<cv::Point2f> features_current, features_previous, features_tmp, useless;

    int interval;
    float fov;
    
    void process_frame( cv::Mat raw, cv::Mat &output );
    
    /**
     * Compute optical flow in pixels / second. Sum is the total pixel change while count is the total number of points for the frame.
    */
    float compute_pixel_velocity( float sum, int count, float dt );

    float compute_velocity( float flow, int axis_length, float distance );

    /**
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