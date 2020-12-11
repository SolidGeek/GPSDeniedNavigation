#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <string>
#include <cstring>
#include <queue>
#include <iostream>
#include <fstream>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/types.h>

#define LOG_DELIMETER ";"

class Logger
{
public:

    std::queue<std::string> queue;

    // structure to hold data temporarily
    struct log_data_t
    {
        uint64_t time;   // 8

        uint8_t imu_rdy; // 1
        uint8_t quat_rdy; // 1
        uint8_t gps_rdy;  // 1
        uint8_t pos_rdy;  // 1

        // Highress IMU data
        float ax, ay, az; // 12
        float gx, gy, gz; // 12
        float imu_alt; // 4
        float imu_temp; // 4

        // Quaternion attitude
        float q1, q2, q3, q4; // 16

        // GPS data
        int32_t gps_lat, gps_lon, gps_alt; // 12
        uint16_t gps_v, gps_cog; // 8

        // Global position estimate
        int32_t pos_lat, pos_lon, pos_alt, pos_alt_rel; // 16
        int16_t pos_vx, pos_vy, pos_vz; // 12

        // Camera timestamps
        uint64_t frame_time; // 8
        uint32_t frame_count; // 4
    }__attribute__((packed));

    log_data_t data;

    Logger( const std::string path ); // 4096b * 250 = 10.24mb

    void clear( void );

    void save_queue( void );

    void add_to_queue( std::string str );

    std::string build_line( void );

    // Memory mapping instead of CSV file
    log_data_t * mem_log;
    uint32_t mem_index;
    int mem_file;

    void mem_add_line( void );
    void mem_sync_file( void );
    void mem_close_file( void );
    void mem_log_start( int log_size = 2500 );

private:

    std::string log_path;
    std::ofstream file;

};

#endif // LOGGER_H
