#include <iostream>

#include <opencv2/opencv.hpp>

using namespace cv;

int main( int argc, char** argv )
{
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(1);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam", cv::WINDOW_AUTOSIZE);

    // this will contain the image from the webcam
    cv::Mat frame;
        
    
    
    // display the frame until you press a key
    while (1) {

        // capture the next frame from the webcam
        camera >> frame;
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(10) >= 0)
            break;
    }
    return 0;
}