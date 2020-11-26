#include <iostream>
#include <unistd.h>
#include <string>
#include <time.h>
#include "serial.h"
#include "camera.h"
#include "telemetry.h"
#include "visual_odemetry.h"
#include <math.h>

#include <stack>
#include <ctime>

std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define CAMERA_SAMPLE_TIME 2e4 // in us, given a rate of 50 Hz

// Init of serial port
Serial px4_uart("/dev/ttyTHS1", SERIAL_READ);

// Serial servo_uart("/dev/ttyUSB0", SERIAL_WRITE);

Telemetry tlm;
Camera cam;
VisualOdemetry vo;


uint32_t dt;
uint32_t last_time;
bool streaming = true;
cv::Point2f velocity;

uint8_t byte;
uint32_t msgid;
char buf[10] = {'\0'};
uint8_t receive_buf[100];


uint32_t get_time() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec*1000 + now.tv_nsec/1.0e6);
}

uint32_t micros(){

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    static uint32_t start_time = 0;

    if( start_time == 0 ){
        start_time = (now.tv_sec*1e6 + now.tv_nsec/1e3);
    }

    uint32_t now_time = (now.tv_sec*1e6 + now.tv_nsec/1e3);

    return now_time - start_time;

}

int main()
{
    cam.config( IMAGE_WIDTH, IMAGE_HEIGHT );
    vo.config( IMAGE_WIDTH, IMAGE_HEIGHT, 2, 16);
    // usb.setup( SERIAL_TYPE_USB, B115200 );

    px4_uart.setup( SERIAL_TYPE_THS, B115200 );
    // servo_uart.setup( SERIAL_TYPE_USB, B115200);


    while( true ){

        if( micros() - last_time > CAMERA_SAMPLE_TIME ){

            dt = micros() - last_time;
            last_time = micros();

            cam.read();

            velocity = vo.compute_sparse_flow( cam.frame);
            cam.show( vo.get_frame() );


            printf("Hz: %.2f \n", 1.0/((float)dt/1.0e6)  );
        }else{
            if( px4_uart.read_char( &byte ) ){

                msgid = tlm.process( byte );

                switch( msgid ){

                    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:

                        float qw = tlm.data.attitude.q1;
                        float qx = tlm.data.attitude.q2;
                        float qy = tlm.data.attitude.q3;
                        float qz = tlm.data.attitude.q4;

                        float roll = atan2( -2*(qy*qz - qx*qw), pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2) ) * 180.0/M_PI;

                        // float yaw = atan2( -2*(qx*qy - qz*qw), pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2) );

                        // float dt = (float)(get_time()-last_time)/1000.0;
                        // printf("roll: %+.4f \n", roll);
                        // last_time = get_time();

                        sprintf(buf, "A%.2f \n", roll);
                        printf("%s", buf);
                        // servo_uart.write_string(buf);

                    break;

                }
            }
        }



        // sprintf(buf, "A%.2f \n", angle);
        // usb.write_string(buf);
    }

    // cam.stop();


    /*
    while (true) {
        uint8_t byte;
        uint32_t msgid;

        if( uart.read_char( &byte ) ){

            msgid = tlm.process( byte );

            switch( msgid ){

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                    float dt = (float)(get_time()-last_time)/1000.0;
                    printf("dt:%.3f gx:%+.4f \n", dt, tlm.data.imu.xgyro);
                    last_time = get_time();
                break;

            }
        }
    }*/
    return 0;
}
