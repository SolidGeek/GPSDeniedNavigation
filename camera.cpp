#include "camera.h"

Camera::Camera( int index, int method )
{
    printf("Opening camera \n");
    camera.open( index , method );



    // width  = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    //height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void Camera::config( int _width, int _height )
{

    int format = cv::VideoWriter::fourcc('G','R','E','Y');

    width = _width;
    height = _height;

    printf("Setting properties \n");


    camera.set(cv::CAP_PROP_FOURCC, format);
    // camera.set(cv::CAP_PROP_CONVERT_RGB, false);

    // camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);    // 640
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);  // 480

    // force initial exposure to avoid orange hue
    // camera.read(frame);
    //system("v4l2-ctl -c exposure=2000");
    system("v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=0");
}

bool Camera::read()
{
    camera.read( frame );
    if (frame.empty())
        return false;

    // Convert to RGB
    // cv::cvtColor(frame,frame, cv::COLOR_BayerGR2RGB);
    //cv::convertScaleAbs(frame, frame, 0.25, 0);

    return true;
}

bool Camera::show( cv::Mat frame ){
    cv::imshow("Camera feed sparse", frame );

    if (cv::waitKey(10) >= 0)
        return false;

    return true;
}

bool Camera::stream(){
    if( ! read() ){
        printf("No frame \n");
    }

    cv::imshow("Camera feed sparse", frame );

    if (cv::waitKey(10) >= 0)
        return false;

    return true;
}

void Camera::stop()
{
    camera.release();
}

