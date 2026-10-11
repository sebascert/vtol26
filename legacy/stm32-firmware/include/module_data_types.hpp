#pragma once
#include <cstdint>
#include <cmath>

namespace stallion {

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    void normalize() {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n > 0.0001f) {
            w/=n; x/=n; y/=n; z/=n;
        }
    }

    void to_euler(float& roll, float& pitch, float& yaw) const {
        roll  = atan2f(2*(w*x+y*z), 1-2*(x*x+y*y));
        pitch = asinf(2*(w*y - z*x));
        yaw   = atan2f(2*(w*z+x*y), 1-2*(y*y+z*z));
    }
};

struct StateEstimate {
    Quaternion q;

    float euler_roll_rad = 0;
    float euler_pitch_rad = 0;
    float euler_yaw_rad = 0;

    float rate_roll_rads = 0;
    float rate_pitch_rads = 0;
    float rate_yaw_rads = 0;

    float accel_x_mps2 = 0;
    float accel_y_mps2 = 0;
    float accel_z_mps2 = 0;

    float altitude_m = 0;
    float altitude_rate_mps = 0;

    float pos_north_m = 0;
    float pos_east_m = 0;

    float vel_north_mps = 0;
    float vel_east_mps = 0;

    float airspeed_mps = 0;

    bool is_healthy = false;
    uint32_t timestamp_us = 0;
};

struct AttitudeSetpoint {
    float roll_rad = 0;
    float pitch_rad = 0;
    float yaw_rad = 0;
    float thrust_normalized = 0;
};

struct ControlOutput {
    float motor_cmd[4] = {0,0,0,0};
    float torque_roll = 0;
    float torque_pitch = 0;
    float torque_yaw = 0;
    float thrust = 0;
    uint32_t timestamp_us = 0;
    bool valid = false;
};

struct IMURawData {
    float accel_x = 0, accel_y = 0, accel_z = 0;  // m/s²
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;     // rad/s
    float temp_c = 0;
    uint32_t timestamp_us = 0;
};

struct BaroRawData {
    float pressure_pa = 0;
    float temperature_c = 0;
    float altitude_m = 0;
    uint32_t timestamp_us = 0;
};

struct MagRawData {
    float mag_x = 0, mag_y = 0, mag_z = 0;
    uint32_t timestamp_us = 0;
};

struct SensorData {
    float imu_acx = 0, imu_acy = 0, imu_acz = 0;
    float imu_gx = 0, imu_gy = 0, imu_gz = 0;
    
    float mag_x = 0, mag_y = 0, mag_z = 0;
    bool mag_valid = false;
    
    float baro_alt_m = 0;
    float baro_temp_c = 0;
    
    bool gps_valid = false;
    double gps_lat = 0, gps_lon = 0;
    float gps_alt_m = 0;
    
    uint32_t timestamp_us = 0;
};

struct ControlSetpoint {
    float roll_rad = 0;
    float pitch_rad = 0;
    float yaw_rad = 0;
    float thrust = 0;
};

} // namespace stallion