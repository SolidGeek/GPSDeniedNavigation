#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "mavlink.h"
#include "timing.h"

class Telemetry
{
public:

    // Structures to hold data
    mavlink_highres_imu_t imu;              // Raw IMU
    mavlink_gps_raw_int_t gps;              // Raw GPS
    mavlink_global_position_int_t pos;      // Global position
    mavlink_attitude_quaternion_t att;      // Attitude
    mavlink_extended_sys_state_t state;    // System status

    // Timestamps for packages (us)
    uint64_t imu_time; // In us
    uint64_t gps_time;
    uint64_t pos_time;
    uint64_t att_time;
    uint64_t state_time;

    Telemetry();

    bool parse( mavlink_message_t msg );

    int16_t process( uint8_t byte );

private:

    // Mavlink structures to hold messages
    mavlink_status_t status;
    mavlink_message_t msg;
};

#endif // TELEMETRY_H
