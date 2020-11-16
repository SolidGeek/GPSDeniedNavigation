#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera
{
public:

    Camera( int index, int method = cv::CAP_V4L2, int scale = 1);

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
