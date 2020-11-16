#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

#define CAMERA_ID_MAIN 0

class Camera
{
public:

    Camera( int index = CAMERA_ID_MAIN, int method = cv::CAP_V4L2 );

    bool read();
    void stream();
    void config( int width, int height );

    int height;
    int width;

    cv::Mat frame;

private:

    cv::VideoCapture camera;
};

#endif // CAMERA_H
