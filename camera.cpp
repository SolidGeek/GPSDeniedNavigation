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
    frame_rdy = false;
    frame_count = 0;

    printf("Setting properties \n");
    camera.set(cv::CAP_PROP_FOURCC, format);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);    // 640
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);  // 480
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Set exposure and gain. A frame is read before setting params, otherwise settings wont be saved.
    camera.read(frame);
    system("v4l2-ctl -c exposure=3000");
    system("v4l2-ctl -c gain=250");
}

bool Camera::read()
{
    cv::Mat temp;
    cv::Size size(width, height);

    camera.read( temp );

    if (temp.empty())
        return false;

    cv::resize(temp, frame, size);
    frame_rdy = true;

    return true;
}

cv::Mat Camera::get(){
    frame_rdy = false;
    frame_count++;
    return frame;
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

