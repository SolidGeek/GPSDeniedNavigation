#include <iostream>
#include <unistd.h>
#include <string>
#include "timing.h"
#include "serial.h"
#include "camera.h"
#include "telemetry.h"
#include "visual_odemetry.h"

#include <math.h>
#include <thread>


#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define CAMERA_SAMPLE_TIME 3.333e4 // in us, given a rate of 30 Hz

// Init of serial port
Serial px4_uart("/dev/ttyTHS1", SERIAL_READ);

// Serial servo_uart("/dev/ttyUSB0", SERIAL_WRITE);

Telemetry tlm;
Camera cam;
VisualOdemetry vo;

float dt;
uint32_t last_time_quat;
uint32_t last_time_imu;
uint32_t last_time_pos;
uint32_t last_time_cam;

cv::Point2f velocity;

// For mavlink packages
uint8_t byte;
uint32_t msgid;

// For uart communication
char write_buf[10];

float pos_x = 300;
float pos_y = 300;

// Data from mavlink
float qw = 0, qx = 0, qy = 0, qz = 0, roll = 0, yaw = 0;

cv::Mat map = cv::Mat::zeros(cv::Size(600,600),CV_8UC1);

void mavlink_thread(){

    px4_uart.setup( SERIAL_TYPE_THS, B115200 );

    while(1){
        if( px4_uart.read_char( &byte ) ){

            msgid = tlm.process( byte );

            switch( msgid ){
                case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
                    printf("Heartbeat");
                break;

                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:

                    qw = tlm.data.attitude.q1;
                    qx = tlm.data.attitude.q2;
                    qy = tlm.data.attitude.q3;
                    qz = tlm.data.attitude.q4;

                    roll = atan2( -2*(qy*qz - qx*qw), pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2) );
                    yaw = atan2( -2*(qx*qy - qz*qw), pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2) );

                    dt = (float)(micros()-last_time_quat)/1e6;
                    last_time_quat = micros();
                    //printf("Yaw: %+.4f \n", yaw);

                break;

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                    dt = (float)(micros()-last_time_imu)/1e6;
                    last_time_imu = micros();
                    //printf("IMU: %+.2f \n", 1/dt);
                break;

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    // Log position
                break;
            }
        }
    }
}

void camera_thread(){

    cv::Mat display;

    cam.config( IMAGE_WIDTH, IMAGE_HEIGHT );
    vo.config( IMAGE_WIDTH, IMAGE_HEIGHT, 40 );

    // Sample camera
    while(1){
        if( micros() - last_time_cam > CAMERA_SAMPLE_TIME ){

            dt = (float)(micros()-last_time_cam)/1e6;
            last_time_cam = micros();

            cam.read();

            vo.compute_sparse_flow( cam.frame, dt );

            float dN = dt * (vo.get_vel_y() * cos(yaw) - vo.get_vel_x() * sin(yaw));
            float dE = dt * (vo.get_vel_y() * sin(yaw) + vo.get_vel_x() * cos(yaw));

            pos_x += dN;
            pos_y += dE;

            cv::Point pos((int)pos_x, (int)pos_y);
            cv::circle(map, pos, 1, cv::Scalar(255, 255, 255));

            display = vo.get_frame();

            int length = 100;
            cv::Point P1(display.cols/2,display.rows/2);
            cv::Point P2;

            P2.x =  (int)round(P1.x + length * cos(yaw));
            P2.y =  (int)round(P1.y + length * sin(yaw));
            cv::line(display, P1, P2, cv::Scalar(255, 255, 255), 2);

            cam.show( display );

            cv::imshow("Map", map);

            printf("CAM: %+.2f X: %+.2f Y: %+.2f \n", 1.0/dt, dN, dE );

        }
    }

    cam.stop();
}

int main()
{

    std::thread t1( camera_thread );
    std::thread t2( mavlink_thread );

    // usb.setup( SERIAL_TYPE_USB, B115200 );
    // servo_uart.setup( SERIAL_TYPE_USB, B115200);

    // while( true ){


        // sprintf(buf, "A%.2f \n", angle);
        // usb.write_string(buf); */
    // }

    t1.join();
    t2.join();

    return 0;
}
