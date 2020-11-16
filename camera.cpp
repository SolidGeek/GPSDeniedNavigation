#include "camera.h"

Camera::Camera( int index, int method )
{
    printf("Opening camera \n");
    camera.open( index , method );
    camera.set(cv::CAP_PROP_CONVERT_RGB, 0);

    printf("Reading properties \n");
    width  = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void Camera::config( int _width, int _height )
{
    width = _width;
    height = _height;

    printf("Setting properties \n");
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);    // 640
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);  // 480

    // force initial exposure to avoid orange hue
    camera.read(frame);
    // system("v4l2-ctl -c exposure=100");

}

bool Camera::read()
{
    camera.read( frame );
    if (frame.empty())
        return false;

    return true;
}

void Camera::stream(){
    while(1){

        if( ! read() ){
            printf("No frame \n");
        }

        cv::cvtColor(frame,frame, cv::COLOR_BayerGR2RGB);
        cv::convertScaleAbs(frame, frame, 0.25, 0);

        cv::imshow("Camera feed sparse", frame );

        if (cv::waitKey(10) >= 0)
            break;
    }
    camera.release();
}


