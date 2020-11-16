#include "camera.h"

Camera::Camera( int index, int method, int scale )
{
    camera.open( index , method );
    width  = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void Camera::config()
{
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640); // 640 - 1280 - 2592 / 640 - 1920 - 2432 - 4896
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
}

bool Camera::read()
{
    camera.read( frame );
    if (!frame.empty())
        return false;

    return true;
}
