#include <iostream>
#include <unistd.h>
#include <string>
#include <math.h>
#include <thread>

#include "timing.h"
#include "serial.h"
#include "camera.h"
#include "telemetry.h"
#include "logger.h"
#include "visual_odemetry.h"

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define LOG_SAMPLE_TIME     2e4     // in us, a rate of 50Hz
#define CAMERA_SAMPLE_TIME  3.333e4 // in us, given a rate of 30 Hz

const std::string path = "/home/dev/GPSDeniedNavigation/log";

// Initialization of classes
Serial px4_uart("/dev/ttyTHS1", SERIAL_READ);
Camera cam;
VisualOdemetry vo;
Telemetry tlm;
Logger data_log(path);

// Serial servo_uart("/dev/ttyUSB0", SERIAL_WRITE);

// Global flags
static bool log_data_flag = false;

void mavlink_thread(){

    // For mavlink packages
    uint8_t byte;
    uint32_t msgid;
    float dt;

    // UART communication setup
    px4_uart.setup( SERIAL_TYPE_THS, B115200 );

    while(1){
        if( px4_uart.read_char( &byte ) ){

            msgid = tlm.process( byte );

            switch( msgid ){
                case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
                    if( tlm.state.landed_state == 2 ){
                        log_data_flag = true;
                    }
                break;

                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                    dt = (float)(micros()-tlm.att_time)/1e6;

                    data_log.data.quat_rdy   = true;
                    data_log.data.q1         = tlm.att.q1;
                    data_log.data.q2         = tlm.att.q2;
                    data_log.data.q3         = tlm.att.q3;
                    data_log.data.q4         = tlm.att.q4;
                break;

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                    dt = (float)(micros()-tlm.imu_time)/1e6;

                    data_log.data.imu_rdy        = true;
                    data_log.data.ax             = tlm.imu.xacc;
                    data_log.data.ay             = tlm.imu.yacc;
                    data_log.data.az             = tlm.imu.zacc;
                    data_log.data.gx             = tlm.imu.xgyro;
                    data_log.data.gy             = tlm.imu.ygyro;
                    data_log.data.gz             = tlm.imu.zgyro;
                    data_log.data.imu_alt        = tlm.imu.pressure_alt;
                    data_log.data.imu_temp       = tlm.imu.temperature;
                    data_log.data.imu_abs_pres   = tlm.imu.abs_pressure;
                break;

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    dt = (float)(micros()-tlm.pos_time)/1e6;

                    data_log.data.pos_rdy    = true;
                    data_log.data.pos_lat    = tlm.pos.lat;
                    data_log.data.pos_lon    = tlm.pos.lon;
                    data_log.data.pos_alt    = tlm.pos.alt;
                    data_log.data.pos_vx     = tlm.pos.vx;
                    data_log.data.pos_vy     = tlm.pos.vy;
                    data_log.data.pos_vz     = tlm.pos.vz;
                break;

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    dt = (float)(micros()-tlm.gps_time)/1e6;

                    data_log.data.gps_rdy    = true;
                    data_log.data.gps_lat    = tlm.gps.lat;
                    data_log.data.gps_lon    = tlm.gps.lon;
                    data_log.data.gps_alt    = tlm.gps.alt;
                    data_log.data.gps_cog    = tlm.gps.cog;
                    data_log.data.gps_v      = tlm.gps.vel;
                break;
            }
        }
    }
}

void camera_thread(){

    float dt = 0;
    uint64_t last_time = 0;

    cam.config( IMAGE_WIDTH, IMAGE_HEIGHT );
    vo.config( IMAGE_WIDTH, IMAGE_HEIGHT, 40 );

    while(1){
        dt = (float)(micros()-last_time)/1e6;
        last_time = micros();

        cam.read();
        vo.compute_sparse_flow( cam.frame, dt );

        cam.show( vo.get_frame() );

        printf("CAM: %+.2f \n", 1.0/dt);
        usleep(CAMERA_SAMPLE_TIME);
    }

    cam.stop();
}

void logger_thread(){

    float dt = 0;
    uint64_t last_time = 0;

    // Clear old log
    data_log.clear();

    while( log_data_flag ){
        dt = (float)(micros()-last_time)/1e6;
        last_time = micros();

        printf("SAVE: %+.2f \n", 1.0/dt);

        data_log.save_queue();

        // Save to log every 10th line
        usleep(LOG_SAMPLE_TIME*10);
    }
}

void data_thread(){

    float dt = 0;
    uint64_t last_time = 0;

    while( log_data_flag ){
        dt = (float)(micros()-last_time)/1e6;
        last_time = micros();

        // Add a new line to the logging queue.
        std::string line = data_log.build_line();
        data_log.add_to_queue( line );

        // Sleep the thread until next sample.
        usleep(LOG_SAMPLE_TIME);
    }
}

int main()
{
    // usb.setup( SERIAL_TYPE_USB, B115200 );
    // servo_uart.setup( SERIAL_TYPE_USB, B115200);

    log_data_flag = true;

    std::thread t1( camera_thread );
    std::thread t2( mavlink_thread );
    std::thread t3( data_thread );
    std::thread t4( logger_thread );

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    // while( true ){
        // sprintf(buf, "A%.2f \n", angle);
        // usb.write_string(buf); */
    // }

    return 0;
}
