#include "opticalflow.h"

OpticalFlow::OpticalFlow(){
}

OpticalFlow::OpticalFlow( int frame_width, int frame_height, int scaledown, int interval )
{
    int scale_width = frame_width/scaledown;
    int scale_height = frame_height/scaledown;

    // Calculate scaled size
    this->size = cv::Size(scale_width, scale_height);

    this->interval = interval;
}

cv::Point2f OpticalFlow::process_frame( cv::Mat frame, float dt, float distance )
{
    cv::Point2f vel; 
    cv::Mat gray;
    cv::Mat flow;

    float xsum, ysum = 0;
    float xflow, yflow;
    int count = 0;

    // Resize the frame to specified size
    cv::resize(frame, this->frame, this->size);

    // Convert to grayscale (and use this to perform flow analysis)
    cv::cvtColor(this->frame, gray, cv::COLOR_BGR2GRAY);

    if( ! this->prev_frame.empty() ){

        // Used opencv to calculate flow between new frame and previous frame
        cv::calcOpticalFlowFarneback( this->prev_frame, gray, flow, this->pyr_scale, this->levels, this->winsize, this->iterations, this->poly_n, this->poly_sigma, 0);

        for (int y = 0; y < flow.rows; y+=this->interval)
        {
            for (int x = 0; x < flow.cols; x+=this->interval)
            {
                // Extract the vector the flow in this 
                cv::Point2f vector = flow.at<cv::Point2f>(y,x);

                xsum += vector.x;
                ysum += vector.y;

                // Visualize the flow in the frame

                cv::line(this->frame, cv::Point(x,y), cv::Point(x+int(vector.x),y+int(vector.y)), cv::Scalar(0, 255, 0) );
                cv::circle(this->frame, cv::Point(x,y), 1, cv::Scalar(0, 255, 0) );

                count++;
            }
        }

        // Calculate flow from sums
        vel.x = this->compute_flow( xsum, count, dt);
        vel.y = this->compute_flow( ysum, count, dt);
    }

    if( distance != 0 ){

        vel.x = this->compute_velocity( vel.x, gray.cols, distance );
        vel.y = this->compute_velocity( vel.y, gray.rows, distance );
    }

    this->prev_frame = gray;

    return vel;
}

float OpticalFlow::compute_flow( float sum, int count, float dt ){
    // Calculate pixels per second
    return (sum / (float)count) / dt;
}

float OpticalFlow::compute_velocity( float flow, int axis_length, float distance ){

    int center_distance = axis_length/2;

    float pixel_per_length = (center_distance / (float)distance);

    return (flow / pixel_per_length);

}

cv::Mat OpticalFlow::get_frame( void ){
    return this->frame;
}