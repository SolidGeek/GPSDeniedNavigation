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

#define IMAGE_WIDTH 800 // 1600/2
#define IMAGE_HEIGHT 650 // 1300/2

#define LOG_SAMPLE_TIME     2e4 // in us, a rate of 50Hz
#define CAMERA_SAMPLE_TIME  3.333e4 // in us, given a rate of 20 Hz

const std::string path = "/home/dev/GPSDeniedNavigation/log";

// Initialization of classes
Serial px4_uart("/dev/ttyTHS1", SERIAL_READ);
Camera cam;
VisualOdemetry vo;
Telemetry tlm;
Logger data_log(path);

// Serial servo_uart("/dev/ttyUSB0", SERIAL_WRITE);

// Global flags
static bool logging_data_flag = false;
static bool cam_save_flag = true;

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
    uint64_t timer = 0;
    uint64_t process_time = 0;

    cam.config( IMAGE_WIDTH, IMAGE_HEIGHT );
    vo.config( IMAGE_WIDTH, IMAGE_HEIGHT, 40 );

    while(1){
        dt = (float)(micros()-last_time)/1e6;
        last_time = micros();

        timer = micros();
        cam.read();

        // vo.compute_sparse_flow( cam.frame, dt );
        // cam.show( vo.get_frame() );

        process_time = micros() - timer;

        printf("CAM: %+.2f \n", 1.0/dt);

        usleep(CAMERA_SAMPLE_TIME - process_time);
    }

    cam.stop();
}

void video_thread(){

    int format = cv::VideoWriter::fourcc('M', 'P', 'E', 'G');
    cv::VideoWriter video(path + "/video.avi", format, 30, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

    while( cam.frame_count < 100 ){
        // Only write a new frame, if the camera thread has read an new frame
        if( cam.frame_rdy ){
            video.write( cam.get() );
            printf("NEW FRAME %d \n", cam.frame_count);
        }

    }

    video.release();

}

void sync_thread(){

    while( 1 ){
        // Only sync log if is logging
        if( logging_data_flag ){
            data_log.mem_sync_file();
        }

        // Save to file every 100th line
        usleep(LOG_SAMPLE_TIME*100);
    }

    /*
    // Clear old log
    data_log.clear();

    while( log_data_flag ){
        dt = (float)(micros()-last_time)/1e6;
        last_time = micros();

        printf("SAVE: %+.2f \n", 1.0/dt);

        data_log.save_queue();

        // Save to log every 10th line
        usleep(LOG_SAMPLE_TIME*10);
    }*/
}

void log_thread(){

    bool logging_started = false;

    while( 1 ){

        // Only log while in air
        if( tlm.state.landed_state == MAV_LANDED_STATE_IN_AIR ){

            if( !logging_started )
                data_log.mem_log_start();

            // Set time of sample time
            data_log.data.time = micros();

            data_log.mem_add_line();

            // Reset ready states
            data_log.data.imu_rdy = false;
            data_log.data.gps_rdy = false;
            data_log.data.pos_rdy = false;
            data_log.data.quat_rdy = false;

            logging_data_flag = true;
        }else {
            logging_data_flag = false;
        }

        usleep(LOG_SAMPLE_TIME);
    }

    printf("Logging done");
    data_log.mem_close_file();

    /* while( log_data_flag ){
        dt = (float)(micros()-last_time)/1e6;
        last_time = micros();

        // Add a new line to the logging queue.
        std::string line = data_log.build_line();
        data_log.add_to_queue( line );

        // Sleep the thread until next sample.
        usleep(LOG_SAMPLE_TIME);
    }*/
}

int main()
{

    printf("Size of struct %d \n", sizeof(Logger::log_data_t) );

    // usb.setup( SERIAL_TYPE_USB, B115200 );
    // servo_uart.setup( SERIAL_TYPE_USB, B115200);

    log_data_flag = true;

    std::thread t1( camera_thread );
    std::thread t2( video_thread );
    std::thread t3( mavlink_thread );
    std::thread t4( log_thread );
    std::thread t5( sync_thread );

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();

    // while( true ){
        // sprintf(buf, "A%.2f \n", angle);
        // usb.write_string(buf); */
    // }

    return 0;
}
