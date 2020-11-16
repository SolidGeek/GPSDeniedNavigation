#include "camera.h"

Camera::Camera( int index, int method )
{
    camera.open( index , method );
    width  = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void Camera::config( int _width, int _height )
{
    width = _width;
    height = _height;

    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);    // 640
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);  // 480
}

bool Camera::read()
{
    camera.read( frame );
    if (!frame.empty())
        return false;

    return true;
}

void Camera::stream(){
    while(1){

        if( ! read() ){
            printf("No frames availble");
            break;
        }

        cv::imshow("Camera feed sparse", frame );

        if (cv::waitKey(10) >= 0)
            break;
    }
}
