#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "mavlink.h"

class Telemetry
{
public:

    // Structure to hold data
    struct {
        mavlink_highres_imu_t imu;
        mavlink_gps_raw_int_t gps;
        mavlink_global_position_int_t estimator;
        mavlink_attitude_quaternion_t attitude;
        mavlink_extended_sys_state_t status;
    } data;

    Telemetry();

    bool parse( mavlink_message_t msg );

    int32_t process( uint8_t byte );

private:

    // Mavlink structures to hold messages
    mavlink_status_t status;
    mavlink_message_t msg;
};

#endif // TELEMETRY_H
