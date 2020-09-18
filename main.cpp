#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "src/opticalflow.h"

cv::VideoCapture camera;
cv::Mat frame;
cv::Point2f velocity; 

int camera_id = 0;

float dt = 0;

float xpos = 0;
float ypos = 0;

int width       = 0;
int height      = 0;
int scaledown   = 2;
int interval    = 20;

std::chrono::duration<float> duration;
std::chrono::_V2::system_clock::time_point now;
std::chrono::_V2::system_clock::time_point last_time;

int main( int argc, char** argv )
{

    camera.open(camera_id, cv::CAP_ANY);
    if (!camera.isOpened()) {
        printf("Could not open camera");
        return -1;
    }

    // camera.set(cv::CAP_PROP_SETTINGS, 0); // Open manufactorer settings
    // camera.set(cv::CAP_PROP_GAIN, 16); 
    // camera.set(cv::CAP_PROP_AUTO_EXPOSURE, false);
    // camera.set(cv::CAP_PROP_EXPOSURE, -8); 

    width   = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height  = camera.get(cv::CAP_PROP_FRAME_HEIGHT);


    OpticalFlow flowsense_dense( width, height, scaledown, interval );
    OpticalFlow flowsense_sparse( width, height, scaledown, interval );

    cv::Size size = flowsense_dense.get_size();

    printf("W: %d H: %d \n", size.width, size.height);
    int index = 0;

    while (1) {
        

        // Read frame from camera
        camera.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            printf("Empty frame");
            break;
        }

        // Calculate time passed
        now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = now - last_time;

        dt = duration.count();

        auto time = std::chrono::high_resolution_clock::now();

        // Process frame using opencv
        velocity = flowsense_dense.compute_dense_flow(frame, dt, 0.8);
        velocity = flowsense_sparse.compute_flow_features(frame, dt, 0.8);

        std::chrono::duration<float> delta = std::chrono::high_resolution_clock::now() - time;
        float passed = (float)delta.count();
        printf("%.8f \n", passed);


        last_time = std::chrono::high_resolution_clock::now();


        //  Calcualte moved distance
        // xpos += velocity.x * dt;
        // ypos += velocity.y * dt;

        // printf("x%+.3f y%+.3f m/s \n", velocity.x, velocity.y );
        // printf("x%+.3f y%+.3f m \n", xpos, ypos );

        // show live and wait for a key with timeout long enough to show images
        cv::imshow("Camera feed dense", flowsense_dense.get_frame() );
        cv::imshow("Camera feed sparse", flowsense_sparse.get_frame() );

        static int count = 0;

        if( index % 5 == 0 ){
            
            char sparse[30];
            char dense[30];
            sprintf( dense, "images/dense/img-%d.png", count );
            sprintf( sparse, "images/sparse/img-%d.png", count );

            cv::imwrite(dense, flowsense_dense.get_frame());
            cv::imwrite(sparse, flowsense_sparse.get_frame());
            count++;
        } 
        
        
        index++;

        if (cv::waitKey(10) >= 0)
            break;
    }
    return 0;
}