#include <iostream>
#include <string>
#include <chrono>
#include <fstream>
#include "serial.h"
#include "mavlink.h"
#include <time.h>

Serial uart(B115200);

uint32_t get_time() {

    struct timespec now;
    clock_gettime(CLOCK_BOOTTIME, &now);

    return (now.tv_sec*1000 + now.tv_nsec/1.0e6);
}

int main()
{
    uint32_t last;

    mavlink_status_t status;
    mavlink_message_t msg;
    int ch = MAVLINK_COMM_0;

    mavlink_highres_imu_t imu;

    while (true) {

        unsigned char byte;

        if(uart.read_char( &byte )){
            if( mavlink_parse_char( ch, byte, &msg, &status ) ){

                switch( msg.msgid ){
                    case MAVLINK_MSG_ID_HIGHRES_IMU:
                        mavlink_msg_highres_imu_decode(&msg, &imu);

                        float freq = (float)(get_time() - last)/1000.0;

                        printf("%+.3f \n", freq);

                        last = get_time();
                    break;
                }
            }
        }
    }

    return 0;
}
