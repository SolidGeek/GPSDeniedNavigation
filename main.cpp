#include <iostream>
#include <string>
#include "serial.h"
#include "mavlink/include/c_library_v2/common/mavlink.h"

Serial uart(B115200);

int main()
{

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

                        std::cout << "gx: " << imu.xgyro << " gy:" << imu.ygyro << " gz:" <<imu.zgyro << std::endl;
                    break;
                }
            }
        }
    }

    return 0;
}
