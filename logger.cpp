#include "logger.h"

Logger::Logger( const std::string path )
{
    log_path = (std::string)path;
}

void Logger::clear(){

    file.open(log_path + "/log.txt", std::ios::trunc);
    file.close();
}

void Logger::save_queue(){

    std::string temp;

    // Open log file for writing
    file.open(log_path + "/log.txt", std::ios::app);

    if(!file.is_open()){
        printf("Log error %i: %s\n", errno, strerror(errno));
    }

    // Loop through the queue, and add each line to log
    while( ! queue.empty() ){
        // Write line to log
        temp = queue.front();
        temp.append("\n");

        // Write to file
        file << temp;

        // Remove line from queue
        queue.pop();
    }

    // Close and save log
    file.flush();
    file.close();
}


void Logger::add_to_queue( std::string str ){
    // Push line to queue, to save when there is time
    queue.push( str );
}

std::string Logger::build_line(){

    std::string line = "";

    line += std::to_string( data.time );

    // IMU Data
    line += std::to_string( data.imu_rdy ) + LOG_DELIMETER;
    line += std::to_string( data.ax ) + LOG_DELIMETER;
    line += std::to_string( data.ay ) + LOG_DELIMETER;
    line += std::to_string( data.az ) + LOG_DELIMETER;
    line += std::to_string( data.gx ) + LOG_DELIMETER;
    line += std::to_string( data.gy ) + LOG_DELIMETER;
    line += std::to_string( data.gz ) + LOG_DELIMETER;
    line += std::to_string( data.imu_alt ) + LOG_DELIMETER;
    line += std::to_string( data.imu_temp ) + LOG_DELIMETER;
    line += std::to_string( data.imu_abs_pres ) + LOG_DELIMETER;

    // Quaternion data
    line += std::to_string( data.quat_rdy ) + LOG_DELIMETER;
    line += std::to_string( data.q1 ) + LOG_DELIMETER;
    line += std::to_string( data.q2 ) + LOG_DELIMETER;
    line += std::to_string( data.q3 ) + LOG_DELIMETER;
    line += std::to_string( data.q4 ) + LOG_DELIMETER;

    // GPS data
    line += std::to_string( data.gps_rdy ) + LOG_DELIMETER;
    line += std::to_string( data.gps_lat ) + LOG_DELIMETER;
    line += std::to_string( data.gps_lon ) + LOG_DELIMETER;
    line += std::to_string( data.gps_alt ) + LOG_DELIMETER;
    line += std::to_string( data.gps_v ) + LOG_DELIMETER;

    // Global position (groundtruth)
    line += std::to_string( data.pos_rdy ) + LOG_DELIMETER;
    line += std::to_string( data.pos_lat ) + LOG_DELIMETER;
    line += std::to_string( data.pos_lon ) + LOG_DELIMETER;
    line += std::to_string( data.pos_alt ) + LOG_DELIMETER;
    line += std::to_string( data.pos_vx ) + LOG_DELIMETER;
    line += std::to_string( data.pos_vy ) + LOG_DELIMETER;
    line += std::to_string( data.pos_vz );

    return line;
}
