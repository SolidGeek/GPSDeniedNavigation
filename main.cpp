#include <iostream>
#include <unistd.h>
#include <string>
#include <time.h>
#include "serial.h"
#include "camera.h"
#include "telemetry.h"

// Init of serial port
// Serial uart(B115200);

Serial usb("/dev/ttyUSB0", SERIAL_WRITE);

// Telemetry tlm;
Camera cam;

uint32_t time_ms;
uint32_t start_time;
uint32_t last_time;
bool streaming = true;

char buf[10] = {'\0'};

uint32_t get_time() {

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec*1000 + now.tv_nsec/1.0e6);
}

int main()
{

    start_time = get_time();

    cam.config( 640, 480 );
    // cam.stream();

    float angle = 0;

    usb.setup( SERIAL_TYPE_USB, B115200 );

    while( streaming ){

        streaming = cam.stream();

        time_ms = get_time() - start_time;

        angle = 30 * sin (2 * M_PI * time_ms/1000);

        sprintf(buf, "A%.2f \n", angle);

        usb.write_string(buf);
        //usleep(10000);

    }

    cam.stop();


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
