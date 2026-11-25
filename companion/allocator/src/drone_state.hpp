// Esto representa el estado actual del dron que llega desde PX4:

#pragma once

#include "pso_runtime.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <initializer_list>

struct DroneState {
  float local_x = 0, local_y = 0, local_z = 0; // LOCAL_POSITION_NED [m]
  float velocity_x = 0, velocity_y = 0, velocity_z = 0; // NED [m/s]
  using Clock = std::chrono::steady_clock;
  Clock::time_point heartbeat_received{}, attitude_received{},
      position_received{};
  Clock::time_point position_target_received{}, attitude_target_received{};
  pso_runtime::PoseTarget target;
  uint32_t time_boot_ms = 0, position_target_boot_ms = 0,
           attitude_target_boot_ms = 0;
  uint32_t position_boot_ms = 0, attitude_boot_ms = 0;
  bool ready(Clock::time_point now, std::chrono::milliseconds timeout) const {
    for (auto time : {heartbeat_received, attitude_received, position_received,
                      position_target_received, attitude_target_received})
      if (time == Clock::time_point{} || now - time > timeout)
        return false;
    for (auto stamp : {position_target_boot_ms, attitude_target_boot_ms}) {
      const uint32_t difference = time_boot_ms - stamp;
      if (std::min(difference, uint32_t(0) - difference) > timeout.count())
        return false;
    }
    return true;
  }
  // ATTITUDE
  float roll = 0;
  float pitch = 0;
  float yaw = 0;

  // GPS
  float lat = 0;
  float lon = 0;
  float alt = 0;

  // BAROMETER
  float pressure = 0;

  // IMU
  float accel_x = 0;
  float accel_y = 0;
  float accel_z = 0;

  // PITOT
  float airspeed = 0;

  // GYRO Vel. Ang. Dron.
  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;

  // RC
  uint16_t rc_channels[16] = {0};

  // Flight mode
  uint32_t flight_mode = 0;

  // Time Stamp
  uint64_t timestamp_us = 0;
};
