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
    int format = cv::VideoWriter::fourcc('B','A','1','0'); // 1: BA10, 2: GREY

    // Allocate size of frame
    frame = cv::Mat(width, height, CV_8UC3);

    width = _width;
    height = _height;
    frame_rdy = false;
    frame_count = 0;

    printf("Setting properties \n");
    // camera.set(cv::CAP_PROP_FOURCC, format);
    camera.set(cv::CAP_PROP_CONVERT_RGB, 0);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);    // 640
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);  // 480
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Set exposure and gain. A frame is read before setting params, otherwise settings wont be saved.
    camera.read(frame);
    system("v4l2-ctl -c exposure=2000"); // 500 = grey
    system("v4l2-ctl -c gain=100");
}


bool Camera::read()
{
    // Input buffer (with room for 10bit bayer).
    cv::Mat buffer(width, height, CV_16UC1);
    cv::Mat temp(width, height, CV_16UC3);

    camera.read( buffer );

    if (buffer.empty())
        return false;

    // Decode the Bayer data to RGB but keep using 16 bits per channel
    cv::cvtColor( buffer, temp, cv::COLOR_BayerGR2RGB );

    // Scale 10bit to 8bit: 1024/256 = 0.25
    cv::convertScaleAbs( temp, frame, 0.25 );

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

