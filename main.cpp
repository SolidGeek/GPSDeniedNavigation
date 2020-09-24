#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "src/opticalflow.h"

float time_passed(){
    static std::chrono::_V2::system_clock::time_point last_time;
    std::chrono::_V2::system_clock::time_point current_time;
    std::chrono::duration<float> delta_time;

    // Get current time
    current_time = std::chrono::high_resolution_clock::now();
    
    // Calculate time passed
    delta_time = current_time - last_time;

    // Save current time for next time
    last_time = current_time;

    return delta_time.count();
}

int main( int argc, char** argv )
{
    // Settings
    const int camera_id = 0;
    const int scaledown   = 2;
    const int interval    = 20; // 
    const float fov       = 64; // Field of view

    // Params
    cv::VideoCapture camera;
    cv::Mat frame;
    cv::Point2f velocity; 
    int width  = 0;
    int height = 0;
    float xpos = 0;
    float ypos = 0;
    
    // Open camera feed
    camera.open(camera_id, cv::CAP_ANY);
    if (!camera.isOpened()) {
        return -1;
    }

    // Get feed size
    width   = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height  = camera.get(cv::CAP_PROP_FRAME_HEIGHT);

    // Initiate an OpticalFlow object 
    OpticalFlow flowsense( width, height, scaledown, interval, fov );

    while (1) {
        // Read frame from camera
        camera.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            printf("Empty frame");
            break;
        }

        float dt = time_passed();
        // Process frame using opencv
        velocity = flowsense.compute_sparse_flow(frame, dt, 0.7);

        //  Calcualte moved distance
        xpos += velocity.x * dt;
        ypos += velocity.y * dt;

        printf("x%+.3f y%+.3f m/s \n", velocity.x, velocity.y );
        printf("x%+.3f y%+.3f m \n", xpos, ypos );
        printf("Time passed: %.4f s", dt);

        // Show live feed
        cv::imshow("Camera feed sparse", flowsense.get_frame() );
        
        if (cv::waitKey(10) >= 0)
            break;
    }
    return 0;
}