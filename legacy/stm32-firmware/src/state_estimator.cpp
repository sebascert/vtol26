#include "state_estimator.hpp"
#include <Arduino.h>

namespace stallion {

StateEstimator::StateEstimator()
    : last_update_us_(0),
      is_healthy_(false),
      accel_x_(0), accel_y_(0), accel_z_(0),
      gyro_x_(0), gyro_y_(0), gyro_z_(0),
      mag_x_(0), mag_y_(0), mag_z_(0),
      baro_pressure_pa_(101325), baro_temperature_c_(15),
      gps_lat_(0), gps_lon_(0), gps_alt_(0), gps_speed_(0),
      pitot_airspeed_(0),
      last_imu_time_us_(0), last_baro_time_us_(0),
      last_gps_time_us_(0), last_pitot_time_us_(0),
      baro_alt_reference_m_(0), baro_reference_set_(false),
      accel_weight_(0.04f),  // 4% corrección de aceleración para menor drift
      mag_weight_(0.04f),    // 4% corrección de magnetómetro para yaw más estable
      baro_weight_(0.05f),   // 5% para altitud baro
      alt_ekf_state_(0), alt_ekf_covariance_(1.0f),
      alt_ekf_process_noise_(0.01f), alt_ekf_measurement_noise_(0.1f),
      alt_ekf_initialized_(false),
      roll_rad_(0), pitch_rad_(0), yaw_rad_(0),
      attitude_initialized_(false) {
    memset(&state_, 0, sizeof(state_));
    state_.q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
}

StateEstimator::~StateEstimator() {}

bool StateEstimator::update(uint64_t now_us) {
    if (!is_healthy_) {
        return false;
    }

    float dt = 0.0f;
    if (last_update_us_ > 0) {
        dt = (now_us - last_update_us_) / 1e6f;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;
        }
    }
    last_update_us_ = now_us;

    update_attitude(dt);

    if (last_baro_time_us_ > 0) {
        float vertical_accel = -accel_z_;  // aproximación (NED)
        update_altitude_ekf(state_.altitude_m, vertical_accel, dt);
    }

    update_gps_position();
    update_airspeed();

    state_.timestamp_us = now_us;
    state_.is_healthy = is_healthy_;

    return true;
}

StateEstimate StateEstimator::state() const {
    return state_;
}

bool StateEstimator::is_healthy() const {
    return is_healthy_;
}

void StateEstimator::reset() {
    state_ = StateEstimate();
    state_.q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    last_update_us_ = 0;
    alt_ekf_initialized_ = false;
    attitude_initialized_ = false;
    roll_rad_ = 0.0f;
    pitch_rad_ = 0.0f;
    yaw_rad_ = 0.0f;
    is_healthy_ = false;
}

// ============================================================================
// MÉTODOS DE ALIMENTACIÓN DE DATOS
// ============================================================================

bool StateEstimator::feed_imu(float ax, float ay, float az, float gx, float gy, float gz, uint32_t timestamp_us) {
    accel_x_ = ax;
    accel_y_ = ay;
    accel_z_ = az;
    gyro_x_ = gx;
    gyro_y_ = gy;
    gyro_z_ = gz;
    
    last_imu_time_us_ = timestamp_us;
    is_healthy_ = true;
    return true;
}

static void euler_to_quaternion(float roll, float pitch, float yaw, Quaternion& q);

void StateEstimator::update_attitude(float dt) {
    float accel_roll = atan2f(accel_y_, accel_z_);
    float accel_pitch = atan2f(-accel_x_, sqrtf(accel_y_ * accel_y_ + accel_z_ * accel_z_));

    if (!attitude_initialized_) {
        roll_rad_ = accel_roll;
        pitch_rad_ = accel_pitch;
        yaw_rad_ = 0.0f;
        attitude_initialized_ = true;
    } else {
        roll_rad_ = kComplementaryAlpha * (roll_rad_ + gyro_x_ * dt) + (1.0f - kComplementaryAlpha) * accel_roll;
        pitch_rad_ = kComplementaryAlpha * (pitch_rad_ + gyro_y_ * dt) + (1.0f - kComplementaryAlpha) * accel_pitch;
        yaw_rad_ += gyro_z_ * dt;
    }

    state_.euler_roll_rad = roll_rad_;
    state_.euler_pitch_rad = pitch_rad_;
    state_.euler_yaw_rad = yaw_rad_;
    state_.rate_roll_rads = gyro_x_;
    state_.rate_pitch_rads = gyro_y_;
    state_.rate_yaw_rads = gyro_z_;
    state_.accel_x_mps2 = accel_x_;
    state_.accel_y_mps2 = accel_y_;
    state_.accel_z_mps2 = accel_z_;

    // Mantener el quaternion en sincronía con el filtro de actitud
    euler_to_quaternion(roll_rad_, pitch_rad_, yaw_rad_, state_.q);
    state_.q.normalize();

    // Corregir drift yaw si hay magnetómetro disponible
    if (mag_x_ != 0.0f || mag_y_ != 0.0f || mag_z_ != 0.0f) {
        correct_attitude_mag();
        state_.q.normalize();
    }
}


static void euler_to_quaternion(float roll, float pitch, float yaw, Quaternion& q) {
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = cy * sp * cr + sy * cp * sr;
    q.z = sy * cp * cr - cy * sp * sr;
}

bool StateEstimator::feed_baro(float pressure_pa, float temperature_c, uint32_t timestamp_us) {
    baro_pressure_pa_ = pressure_pa;
    baro_temperature_c_ = temperature_c;
    
    // Calcular altitud usando fórmula barométrica internacional
    // h = T0/L * (P0/P)^(R*L/g) - T0
    const float P0 = 101325.0f;  // Pa (presión a nivel del mar)
    const float T0 = 288.15f;    // K (temperatura estándar)
    const float L = -0.0065f;    // K/m (lapse rate)
    const float R_specific = 287.05f;  // J/(kg·K)
    const float g = 9.81f;
    
    float pressure_ratio = pressure_pa / P0;
    float exponent = (R_specific * L) / g;
    float altitude_m = (T0 / L) * (powf(pressure_ratio, exponent) - 1.0f);
    
    // Establecer referencia en primera lectura
    if (!baro_reference_set_) {
        baro_alt_reference_m_ = altitude_m;
        baro_reference_set_ = true;
    }
    
    state_.altitude_m = altitude_m - baro_alt_reference_m_;
    last_baro_time_us_ = timestamp_us;
    
    return true;
}

bool StateEstimator::feed_mag(float mx, float my, float mz, uint32_t timestamp_us) {
    // Normalizar magnémetro
    float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
    if (mag_norm > 0.01f) {
        mag_x_ = mx / mag_norm;
        mag_y_ = my / mag_norm;
        mag_z_ = mz / mag_norm;
    }
    return true;
}

bool StateEstimator::feed_gps(double lat_deg, double lon_deg, float alt_m, float speed_mps, uint32_t timestamp_us) {
    gps_lat_ = lat_deg;
    gps_lon_ = lon_deg;
    gps_alt_ = alt_m;
    gps_speed_ = speed_mps;
    last_gps_time_us_ = timestamp_us;
    return true;
}

bool StateEstimator::feed_pitot(float airspeed_mps, uint32_t timestamp_us) {
    pitot_airspeed_ = airspeed_mps;
    last_pitot_time_us_ = timestamp_us;
    return true;
}

// ============================================================================
// PREDICCIÓN: INTEGRACIÓN DEL GIROSCOPIO
// ============================================================================

void StateEstimator::predict_attitude(float dt) {
    quaternion_integrate(gyro_x_, gyro_y_, gyro_z_, dt);
    quaternion_normalize();
}

void StateEstimator::quaternion_integrate(float gx, float gy, float gz, float dt) {
    // Método: Integración directa (simple pero estable para dt pequeños)
    // dq/dt = 0.5 * q * ω
    // donde ω = [0, gx, gy, gz]
    
    Quaternion q = state_.q;
    float dq_w = 0.5f * (-q.x * gx - q.y * gy - q.z * gz) * dt;
    float dq_x = 0.5f * (q.w * gx + q.y * gz - q.z * gy) * dt;
    float dq_y = 0.5f * (q.w * gy - q.x * gz + q.z * gx) * dt;
    float dq_z = 0.5f * (q.w * gz + q.x * gy - q.y * gx) * dt;
    
    state_.q.w += dq_w;
    state_.q.x += dq_x;
    state_.q.y += dq_y;
    state_.q.z += dq_z;
}

void StateEstimator::quaternion_normalize() {
    state_.q.normalize();
}

// ============================================================================
// CORRECCIÓN: ACELERÓMETRO
// ============================================================================

void StateEstimator::correct_attitude_accel() {
    // Estimar actitud desde aceleración
    // Asumir: a_body = R(q) * [0, 0, -g]
    // Invertir para obtener corrección
    
    float ax = accel_x_;
    float ay = accel_y_;
    float az = accel_z_;
    
    float accel_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (accel_norm < 0.1f || accel_norm > 40.0f) {
        return;  // Datos inválidos
    }
    
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;
    
    // Matriz de rotación actual desde quaternión
    float dcm[3][3];
    quaternion_to_dcm(dcm);
    
    // Aceleración esperada en frame inercial: [0, 0, -g]
    // Convertir a body frame: a_expected = R_T * [0, 0, -g]
    float a_expected_x = -dcm[2][0];
    float a_expected_y = -dcm[2][1];
    float a_expected_z = -dcm[2][2];
    
    // Error: diferencia entre medido y esperado
    float error_x = ay * a_expected_z - az * a_expected_y;
    float error_y = az * a_expected_x - ax * a_expected_z;
    float error_z = ax * a_expected_y - ay * a_expected_x;
    
    // Aplicar corrección (complementary filter)
    float K_accel = accel_weight_;
    state_.q.x += error_x * K_accel;
    state_.q.y += error_y * K_accel;
    state_.q.z += error_z * K_accel;
}

// ============================================================================
// CORRECCIÓN: MAGNETÓMETRO (HEADING)
// ============================================================================

void StateEstimator::correct_attitude_mag() {
    if (mag_x_ == 0 && mag_y_ == 0 && mag_z_ == 0) {
        return;  // Datos no disponibles
    }
    
    // Matriz DCM
    float dcm[3][3];
    quaternion_to_dcm(dcm);
    
    // Proyectar magnetómetro a frame horizontal
    // B_body = [mx, my, mz]
    // B_earth_horizontal = [dcm[0][0]*mx + dcm[1][0]*my + dcm[2][0]*mz,
    //                       dcm[0][1]*mx + dcm[1][1]*my + dcm[2][1]*mz]
    
    float b_north = dcm[0][0]*mag_x_ + dcm[1][0]*mag_y_ + dcm[2][0]*mag_z_;
    float b_east = dcm[0][1]*mag_x_ + dcm[1][1]*mag_y_ + dcm[2][1]*mag_z_;
    
    // Heading esperado (norte) - Yaw correction
    float heading_error = atan2f(b_east, b_north);
    
    // Corregir yaw del quaternión
    float K_mag = mag_weight_;
    state_.q.z += heading_error * K_mag;
}

// ============================================================================
// ALTITUD: EKF SIMPLIFICADO
// ============================================================================

void StateEstimator::update_altitude_ekf(float baro_alt, float vertical_accel, float dt) {
    if (!alt_ekf_initialized_) {
        alt_ekf_state_ = baro_alt;
        alt_ekf_initialized_ = true;
        return;
    }
    
    // EKF predición (modelo simple: integración de aceleración vertical)
    float alt_pred = alt_ekf_state_ + state_.altitude_rate_mps * dt + 0.5f * vertical_accel * dt * dt;
    
    // Predicción de covarianza
    float P_pred = alt_ekf_covariance_ + alt_ekf_process_noise_;
    
    // Ganancia de Kalman
    float K = P_pred / (P_pred + alt_ekf_measurement_noise_);
    
    // Actualización de estado
    alt_ekf_state_ = alt_pred + K * (baro_alt - alt_pred);
    
    // Actualización de covarianza
    alt_ekf_covariance_ = (1.0f - K) * P_pred;
    
    // Actualizar estado
    state_.altitude_m = alt_ekf_state_;
    
    // Estimar velocidad vertical (diferencia con lectura anterior)
    static float alt_prev = alt_ekf_state_;
    state_.altitude_rate_mps = (alt_ekf_state_ - alt_prev) / dt;
    alt_prev = alt_ekf_state_;
}

// ============================================================================
// GPS Y AIRSPEED
// ============================================================================

void StateEstimator::update_gps_position() {
    // Actualizar posición NED si GPS disponible
    // (simplificado - no incluye proyección a UTM)
    static double lat_ref = 0, lon_ref = 0;
    static bool ref_set = false;
    
    if (!ref_set && gps_lat_ != 0) {
        lat_ref = gps_lat_;
        lon_ref = gps_lon_;
        ref_set = true;
    }
    
    if (ref_set) {
        // Distancia aproximada usando pequeño ángulo
        const float R_earth = 6371000.0f;  // metros
        state_.pos_north_m = (gps_lat_ - lat_ref) * (M_PI / 180.0f) * R_earth;
        state_.pos_east_m = (gps_lon_ - lon_ref) * (M_PI / 180.0f) * R_earth * cosf(gps_lat_ * M_PI / 180.0f);
        state_.vel_north_mps = gps_speed_ * cosf(atan2f(state_.pos_east_m, state_.pos_north_m));
        state_.vel_east_mps = gps_speed_ * sinf(atan2f(state_.pos_east_m, state_.pos_north_m));
    }
}

void StateEstimator::update_airspeed() {
    state_.airspeed_mps = pitot_airspeed_;
}

// ============================================================================
// CONVERSIONES DE QUATERNIÓN
// ============================================================================

void StateEstimator::quaternion_to_euler() {
    state_.q.to_euler(state_.euler_roll_rad, state_.euler_pitch_rad, state_.euler_yaw_rad);
    state_.rate_roll_rads = gyro_x_;
    state_.rate_pitch_rads = gyro_y_;
    state_.rate_yaw_rads = gyro_z_;
}

void StateEstimator::quaternion_to_dcm(float dcm[3][3]) const {
    // Matriz de cosenos directores desde quaternión
    // Basado en: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    
    float q0 = state_.q.w;
    float q1 = state_.q.x;
    float q2 = state_.q.y;
    float q3 = state_.q.z;
    
    dcm[0][0] = 1 - 2*(q2*q2 + q3*q3);
    dcm[0][1] = 2*(q1*q2 - q0*q3);
    dcm[0][2] = 2*(q1*q3 + q0*q2);
    
    dcm[1][0] = 2*(q1*q2 + q0*q3);
    dcm[1][1] = 1 - 2*(q1*q1 + q3*q3);
    dcm[1][2] = 2*(q2*q3 - q0*q1);
    
    dcm[2][0] = 2*(q1*q3 - q0*q2);
    dcm[2][1] = 2*(q2*q3 + q0*q1);
    dcm[2][2] = 1 - 2*(q1*q1 + q2*q2);
}

} // namespace stallion
