#include "attitude_complementary_filter.hpp"
#include <Arduino.h>

namespace stallion {

AttitudeComplementaryFilter::AttitudeComplementaryFilter()
    : last_update_us_(0),
      is_healthy_(false),
      accel_weight_(0.02f),
      mag_weight_(0.01f),
      accel_x_(0), accel_y_(0), accel_z_(0),
      mag_x_(0), mag_y_(0), mag_z_(0) {
    
    state_.q.w = 1.0f;
    state_.q.x = 0.0f;
    state_.q.y = 0.0f;
    state_.q.z = 0.0f;
    
    quaternion_to_euler();
}

AttitudeComplementaryFilter::~AttitudeComplementaryFilter() {}

bool AttitudeComplementaryFilter::update(uint64_t now_us) {
    if (last_update_us_ == 0) {
        last_update_us_ = now_us;
        return true;
    }
    
    accel_correct();
    mag_correct();
    quaternion_normalize();
    quaternion_to_euler();
    
    state_.timestamp_us = now_us;
    last_update_us_ = now_us;
    
    is_healthy_ = true;
    return true;
}

bool AttitudeComplementaryFilter::predict(float dt, float gx, float gy, float gz) {
    quaternion_integrate(gx, gy, gz, dt);
    quaternion_normalize();
    quaternion_to_euler();
    return true;
}

bool AttitudeComplementaryFilter::update_accel(float ax, float ay, float az) {
    accel_x_ = ax;
    accel_y_ = ay;
    accel_z_ = az;
    return true;
}

bool AttitudeComplementaryFilter::update_mag(float mx, float my, float mz) {
    mag_x_ = mx;
    mag_y_ = my;
    mag_z_ = mz;
    return true;
}

StateEstimate AttitudeComplementaryFilter::state() const {
    return state_;
}

bool AttitudeComplementaryFilter::is_healthy() const {
    return is_healthy_;
}

void AttitudeComplementaryFilter::reset() {
    state_.q.w = 1.0f;
    state_.q.x = 0.0f;
    state_.q.y = 0.0f;
    state_.q.z = 0.0f;
    
    quaternion_to_euler();
    
    last_update_us_ = 0;
    is_healthy_ = false;
}

void AttitudeComplementaryFilter::quaternion_integrate(float gx, float gy, float gz, float dt) {
    float qw = state_.q.w;
    float qx = state_.q.x;
    float qy = state_.q.y;
    float qz = state_.q.z;
    
    float q_dot_w = 0.5f * (-qx * gx - qy * gy - qz * gz);
    float q_dot_x = 0.5f * ( qw * gx - qz * gy + qy * gz);
    float q_dot_y = 0.5f * ( qz * gx + qw * gy - qx * gz);
    float q_dot_z = 0.5f * (-qy * gx + qx * gy + qw * gz);
    
    qw += q_dot_w * dt;
    qx += q_dot_x * dt;
    qy += q_dot_y * dt;
    qz += q_dot_z * dt;
    
    state_.q.w = qw;
    state_.q.x = qx;
    state_.q.y = qy;
    state_.q.z = qz;
}

void AttitudeComplementaryFilter::quaternion_normalize() {
    float norm = sqrtf(state_.q.w * state_.q.w + 
                      state_.q.x * state_.q.x + 
                      state_.q.y * state_.q.y + 
                      state_.q.z * state_.q.z);
    
    if (norm > 0.0001f) {
        state_.q.w /= norm;
        state_.q.x /= norm;
        state_.q.y /= norm;
        state_.q.z /= norm;
    }
}

void AttitudeComplementaryFilter::accel_correct() {
    float ax = accel_x_;
    float ay = accel_y_;
    float az = accel_z_;
    
    float accel_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (accel_norm < 0.1f) return;
    
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;
    
    float qw = state_.q.w;
    float qx = state_.q.x;
    float qy = state_.q.y;
    float qz = state_.q.z;
    
    float g_body_x = 2.0f * (qx*qz - qw*qy);
    float g_body_y = 2.0f * (qw*qx + qy*qz);
    float g_body_z = qw*qw - qx*qx - qy*qy + qz*qz;
    
    float ex = g_body_y * az - g_body_z * ay;
    float ey = g_body_z * ax - g_body_x * az;
    float ez = g_body_x * ay - g_body_y * ax;
    
    float kp = accel_weight_;
    
    qw += kp * (-qx * ex - qy * ey - qz * ez);
    qx += kp * ( qw * ex - qz * ey + qy * ez);
    qy += kp * ( qz * ex + qw * ey - qx * ez);
    qz += kp * (-qy * ex + qx * ey + qw * ez);
    
    state_.q.w = qw;
    state_.q.x = qx;
    state_.q.y = qy;
    state_.q.z = qz;
}

void AttitudeComplementaryFilter::mag_correct() {
    if (fabsf(mag_x_) < 0.1f && fabsf(mag_y_) < 0.1f && fabsf(mag_z_) < 0.1f) {
        return;
    }
}

void AttitudeComplementaryFilter::quaternion_to_euler() {
    state_.q.to_euler(state_.euler_roll_rad, state_.euler_pitch_rad, state_.euler_yaw_rad);
}

} // namespace stallion