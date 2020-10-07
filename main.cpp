#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "src/opticalflow.h"
#include <fstream>

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
    const int scaledown   = 1;
    const int interval    = 40; // 
    const float fov       = 64; // Field of view

    // Params
    // cv::VideoCapture camera;
    cv::VideoCapture video("video.avi");
    cv::Mat frame;
    cv::Point2f velocity;
    float xvel = 0;
    float yvel = 0;
    float beta = 0;
    int width  = 0;
    int height = 0;
    float xpos = 0;
    float ypos = 0;
    
    // Open camera feed
    /* camera.open(camera_id, cv::CAP_ANY);
    if (!camera.isOpened()) {
        return -1;
    }*/

    // Get feed size
    width   = video.get(cv::CAP_PROP_FRAME_WIDTH);
    height  = video.get(cv::CAP_PROP_FRAME_HEIGHT);

    // Initiate an OpticalFlow object 
    OpticalFlow flowsense( width, height, scaledown, interval, fov );

    std::ofstream log;
    log.open ("log.csv");

    int framecount = 0;

    while (1) {
        // Read frame from camera
        video.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            printf("Empty frame");
            break;
        }

        float dt = time_passed();
        // Process frame using opencv
        velocity = flowsense.compute_sparse_flow(frame, dt);

        log << framecount << ';' << velocity.x << ';' << velocity.y << std::endl;
        framecount++;

        if(framecount == 15401){
            break;
        }

        /* float sigma = 0.2;
        xvel = (sigma * velocity.x) + (1.0 - sigma) * xvel;
        yvel = (sigma * velocity.y) + (1.0 - sigma) * yvel;

        //  Calcualte moved distance
        xpos += xvel * dt;
        ypos += yvel * dt;

        // Calculate side-slipe angle
        if( abs(xvel) > 0.05 || abs(yvel) > 0.05 ){
            beta = atan2(yvel, xvel)*180.0/M_PI;
        }else{
            beta = 0.0f;
        }
        
        cv::Point2f flow = flowsense.get_flow();

        printf("x%+.3f y%+.3f px/s \n", flow.x, flow.y ); */

        // printf("x%+.3f y%+.3f m/s \n", xvel, yvel );
        // printf("x%+.3f y%+.3f m \n", xpos, ypos );
        // printf("Slide-slip angle: %.0f deg \n", beta);
        // printf("Time passed: %.4f s", dt);

        // Show live feed
        cv::imshow("Camera feed sparse", flowsense.get_frame() );
        
        if (cv::waitKey(10) >= 0)
            break;
    }
    return 0;
}