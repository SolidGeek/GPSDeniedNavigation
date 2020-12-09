#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <string>
#include <cstring>
#include <queue>
#include <iostream>
#include <fstream>
#include <errno.h>

#define LOG_DELIMETER ";"

class Logger
{
public:

    std::queue<std::string> queue;

    // structure to hold data temporarily
    struct
    {
        uint32_t time;
        // Highress IMU data
        bool imu_rdy;
        float ax, ay, az;
        float gx, gy, gz;
        float imu_alt;
        float imu_temp;
        float imu_abs_pres;

        // Quaternion attitude
        bool quat_rdy;
        float q1, q2, q3, q4;

        // GPS data
        bool gps_rdy;
        float gps_lat, gps_lon, gps_alt, gps_v, gps_cog;

        // Global position estimate
        bool pos_rdy;
        float pos_lat, pos_lon, pos_alt;
        float pos_vx, pos_vy, pos_vz;

        // Camera timestamps
        uint32_t frame_time;
        uint16_t frame_count;
    } data;


    Logger( const std::string path );

    void clear( void );

    void save_queue( void );

    void add_to_queue( std::string str );

    std::string build_line( void );

private:

    std::string log_path;
    std::ofstream file;

};

#endif // LOGGER_H
