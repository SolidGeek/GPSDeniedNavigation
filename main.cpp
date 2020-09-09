#include <iostream>

#include <opencv2/opencv.hpp>

int camera_id = 1;

int main( int argc, char** argv )
{
    printf("OpenCV Flow Sensor");

    cv::VideoCapture camera;

    camera.open(camera_id, cv::CAP_ANY);

    if (!camera.isOpened()) {
        printf("Could not open camera");
        return -1;
    }

    // this will contain the image from the webcam
    cv::Mat frame;
    
    while (1) {

        camera.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            printf("Empty frame");
            break;
        }

        // show live and wait for a key with timeout long enough to show images
        cv::imshow("Camera feed", frame);

        if (cv::waitKey(10) >= 0)
            break;
    }
    return 0;
}