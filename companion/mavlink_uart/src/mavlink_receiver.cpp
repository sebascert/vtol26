#include "mavlink_receiver.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <common/mavlink.h>

bool MAVLinkReceiver::init(const char* port)
{
    uart_fd = open(
        port,
        O_RDWR | O_NOCTTY | O_NDELAY
    );

    if(uart_fd == -1)
        return false;

    termios options;

    tcgetattr(uart_fd, &options);

    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);

    options.c_cflag = CS8 | CLOCAL | CREAD;

    tcsetattr(
        uart_fd,
        TCSANOW,
        &options
    );

    return true;
}

bool MAVLinkReceiver::receive(DroneState& state)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    uint8_t byte;

    while(read(uart_fd, &byte, 1) > 0)
    {
        if(mavlink_parse_char(
            MAVLINK_COMM_0,
            byte,
            &msg,
            &status))
        {
            switch(msg.msgid)
            {
                // ATTITUDE
                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    mavlink_attitude_t att;

                    mavlink_msg_attitude_decode(
                        &msg,
                        &att
                    );

                    state.roll  = att.roll;
                    state.pitch = att.pitch;
                    state.yaw   = att.yaw;

                    break;
                }

                // GPS
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    mavlink_gps_raw_int_t gps;

                    mavlink_msg_gps_raw_int_decode(
                        &msg,
                        &gps
                    );

                    state.lat = gps.lat / 1e7;
                    state.lon = gps.lon / 1e7;
                    state.alt = gps.alt / 1000.0;

                    break;
                }

                // IMU
                case MAVLINK_MSG_ID_RAW_IMU:
                {
                    mavlink_raw_imu_t imu;

                    mavlink_msg_raw_imu_decode(
                        &msg,
                        &imu
                    );

                    state.accel_x = imu.xacc;
                    state.accel_y = imu.yacc;
                    state.accel_z = imu.zacc;

                    state.gyro_x = imu.xgyro;
                    state.gyro_y = imu.ygyro;
                    state.gyro_z = imu.zgyro;

                    break;
                }

                // BAROMETRO
                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                {
                    mavlink_scaled_pressure_t baro;

                    mavlink_msg_scaled_pressure_decode(
                        &msg,
                        &baro
                    );

                    state.pressure = baro.press_abs;

                    break;
                }

                // RC
                case MAVLINK_MSG_ID_RC_CHANNELS:
                {
                    mavlink_rc_channels_t rc;

                    mavlink_msg_rc_channels_decode(
                        &msg,
                        &rc
                    );

                    state.rc_channels[0] = rc.chan1_raw;
                    state.rc_channels[1] = rc.chan2_raw;
                    state.rc_channels[2] = rc.chan3_raw;
                    state.rc_channels[3] = rc.chan4_raw;

                    break;
                }

                case MAVLINK_MSG_ID_VFR_HUD:
            {
                mavlink_vfr_hud_t hud;

                mavlink_msg_vfr_hud_decode(
                    &msg,
                    &hud
                );

                state.airspeed = hud.airspeed;

                break;
            }

                // TIMESTAMP
                case MAVLINK_MSG_ID_SYSTEM_TIME:
                {
                    mavlink_system_time_t t;

                    mavlink_msg_system_time_decode(
                        &msg,
                        &t
                    );

                    state.timestamp_us = t.time_unix_usec;

                    break;
                }

                case MAVLINK_MSG_ID_HEARTBEAT:

                {
                    mavlink_heartbeat_t hb;

                    mavlink_msg_heartbeat_decode(
                        &msg,
                        &hb
                    );

                    state.flight_mode = hb.custom_mode;

                    break;
            }

                default:
                    break;
            }
        }
    }

    return true;
}