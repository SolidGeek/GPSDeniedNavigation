#include "telemetry.h"

Telemetry::Telemetry()
{

}

int32_t Telemetry::process( uint8_t byte ){
    // Feed each byte to mavlink parser
    if( mavlink_parse_char( MAVLINK_COMM_0, byte, &msg, &status ) ){
        // If mavlink returns true, a complete packet was received.
        if( parse( msg ) )
            return msg.msgid;
    }

    return -1;
}

bool Telemetry::parse( mavlink_message_t msg ){
    // Switch on the mavlink message ID
    switch( msg.msgid ){
        case MAVLINK_MSG_ID_HIGHRES_IMU: mavlink_msg_highres_imu_decode(&msg, &data.imu); break;
        case MAVLINK_MSG_ID_GPS_RAW_INT: mavlink_msg_gps_raw_int_decode(&msg, &data.gps); break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: mavlink_msg_global_position_int_decode(&msg, &data.estimator); break;
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: mavlink_msg_attitude_quaternion_decode(&msg, &data.attitude); break;
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: mavlink_msg_extended_sys_state_decode(&msg, &data.status); break;
        default:
            // If the packet was not recognised, disregard the data.
            return false;
        break;
    }
    // A valid packet was received
    return true;
}
