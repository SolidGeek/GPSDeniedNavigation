#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

#define CAMERA_ID_MAIN 0

class Camera
{
public:

    Camera( int index = CAMERA_ID_MAIN, int method = cv::CAP_V4L2 );

    bool read( void );
    bool stream( void );
    bool show( cv::Mat frame );
    void config( int width, int height );
    void stop( void );

    cv::Mat get( void );

    int height;
    int width;

    cv::Mat frame;

    bool frame_rdy;
    int frame_count;

private:

    cv::VideoCapture camera;
};

#endif // CAMERA_H
