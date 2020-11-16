#include <iostream>
#include <string>
#include <time.h>
#include "serial.h"
#include "telemetry.h"

// Init of serial port
Serial uart(B115200);

Telemetry tlm;

uint32_t last_time;

uint32_t get_time() {

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec*1000 + now.tv_nsec/1.0e6);
}

int main()
{
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
    }
    return 0;
}
