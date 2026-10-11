#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include "module_data_types.hpp"
#include "module_interfaces.hpp"

namespace stallion {

class StateEstimator : public IEstimator {
public:
    StateEstimator();
    ~StateEstimator();

    bool update(uint64_t now_us) override;
    StateEstimate state() const override;
    bool is_healthy() const override;
    void reset() override;

    bool feed_imu(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  uint32_t timestamp_us);
    bool feed_baro(float pressure_pa, float temperature_c,
                   uint32_t timestamp_us);
    bool feed_mag(float mx, float my, float mz,
                  uint32_t timestamp_us);
    bool feed_gps(double lat_deg, double lon_deg,
                  float alt_m, float speed_mps,
                  uint32_t timestamp_us);
    bool feed_pitot(float airspeed_mps,
                    uint32_t timestamp_us);

    void set_accel_weight(float w) { accel_weight_ = w; }
    void set_mag_weight(float w) { mag_weight_ = w; }
    void set_baro_weight(float w) { baro_weight_ = w; }

private:
    StateEstimate state_;
    uint64_t last_update_us_;
    bool is_healthy_;

    float accel_x_, accel_y_, accel_z_;
    float gyro_x_, gyro_y_, gyro_z_;
    float mag_x_, mag_y_, mag_z_;

    float baro_pressure_pa_;
    float baro_temperature_c_;

    double gps_lat_;
    double gps_lon_;
    float gps_alt_;
    float gps_speed_;

    float pitot_airspeed_;

    uint32_t last_imu_time_us_;
    uint32_t last_baro_time_us_;
    uint32_t last_gps_time_us_;
    uint32_t last_pitot_time_us_;

    float baro_alt_reference_m_;
    bool baro_reference_set_;

    float accel_weight_;
    float mag_weight_;
    float baro_weight_;

    float alt_ekf_state_;
    float alt_ekf_covariance_;
    float alt_ekf_process_noise_;
    float alt_ekf_measurement_noise_;
    bool alt_ekf_initialized_;

    float roll_rad_;
    float pitch_rad_;
    float yaw_rad_;
    bool attitude_initialized_;
    static constexpr float kComplementaryAlpha = 0.98f;

    void predict_attitude(float dt);
    void correct_attitude_accel();
    void correct_attitude_mag();
    void update_attitude(float dt);
    void update_altitude_ekf(float baro_alt, float accel, float dt);
    void update_gps_position();
    void update_airspeed();

    void quaternion_integrate(float gx, float gy, float gz, float dt);
    void quaternion_normalize();
    void quaternion_to_euler();
    void quaternion_to_dcm(float dcm[3][3]) const;
};

} // namespace stallion
