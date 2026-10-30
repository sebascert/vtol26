#include "mavlink_receiver.hpp"
#include "serial_port.hpp"
#include <cerrno>
#include <cmath>

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <common/mavlink.h>

MAVLinkReceiver::~MAVLinkReceiver() { if (uart_fd >= 0) close(uart_fd); }

bool MAVLinkReceiver::init(const char* port, int baud_rate) {
    uart_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    int error = 0;
    if (uart_fd < 0) return false;
    if (!configureSerialPort(uart_fd, baud_rate, error)) {
        close(uart_fd); uart_fd = -1; errno = error; return false;
    }
    return true;
}

bool MAVLinkReceiver::receive(DroneState& state)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    uint8_t byte;

    ssize_t received = 0;
    size_t budget = 8192;
    while(budget-- > 0 && (received = read(uart_fd, &byte, 1)) > 0)
    {
        if(mavlink_parse_char(
            MAVLINK_COMM_0,
            byte,
            &msg,
            &status))
        {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t hb{};
                mavlink_msg_heartbeat_decode(&msg, &hb);
                if (hb.autopilot != MAV_AUTOPILOT_PX4) continue;
                if (source_system_ == 0) { source_system_ = msg.sysid; source_component_ = msg.compid; }
            }
            if (source_system_ == 0 || msg.sysid != source_system_ || msg.compid != source_component_) continue;
            const auto now = DroneState::Clock::now();
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

                    state.attitude_received = {};
                    if (!std::isfinite(att.roll) || !std::isfinite(att.pitch) || !std::isfinite(att.yaw) ||
                        !std::isfinite(att.rollspeed) || !std::isfinite(att.pitchspeed) || !std::isfinite(att.yawspeed)) break;
                    state.attitude_received = now;
                    state.gyro_x = att.rollspeed;
                    state.gyro_y = att.pitchspeed;
                    state.gyro_z = att.yawspeed;
                    state.roll  = att.roll;
                    state.pitch = att.pitch;
                    state.yaw   = att.yaw;

                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                    mavlink_local_position_ned_t pos{};
                    mavlink_msg_local_position_ned_decode(&msg, &pos);
                    state.position_received = {};
                    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z) ||
                        !std::isfinite(pos.vx) || !std::isfinite(pos.vy) || !std::isfinite(pos.vz)) break;
                    state.local_x = pos.x; state.local_y = pos.y; state.local_z = pos.z;
                    state.velocity_x = pos.vx; state.velocity_y = pos.vy; state.velocity_z = pos.vz;
                    state.position_received = now;
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

                    // Angular rates come from ATTITUDE in rad/s, not RAW_IMU millirad/s.

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

                state.airspeed_received = {};
                if (!std::isfinite(hud.airspeed) || hud.airspeed < 0) break;
                state.airspeed = hud.airspeed;
                state.airspeed_received = now;

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
                    state.heartbeat_received = now;

                    break;
            }

                default:
                    break;
            }
        }
    }

    return received >= 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
}
