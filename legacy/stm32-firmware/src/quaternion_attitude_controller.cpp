#include "quaternion_attitude_controller.hpp"
#include <Arduino.h>

namespace stallion {

QuaternionAttitudeController::QuaternionAttitudeController()
    : is_healthy_(false),
      kp_attitude_(4.5f),
      kp_rate_(0.15f),
      thrust_max_(1.0f),
      last_roll_sp_(0),
      last_pitch_sp_(0),
      last_yaw_sp_(0) {
    memset(&output_, 0, sizeof(output_));
}

bool QuaternionAttitudeController::update(const StateEstimate& state, 
                                          const ControlSetpoint& setpoint, 
                                          float dt) {
    if (!state.is_healthy) {
        is_healthy_ = false;
        return false;
    }

    // Convertir setpoint Euler a cuaternión deseado
    euler_to_quaternion(setpoint.roll_rad, setpoint.pitch_rad, setpoint.yaw_rad, q_desired_);

    // Calcular error cuaterniónico
    Quaternion q_error = quaternion_error(q_desired_, state.q);

    // Convertir error a torques
    float torque_roll = 0, torque_pitch = 0, torque_yaw = 0;
    quaternion_error_to_torques(q_error, dt, torque_roll, torque_pitch, torque_yaw);

    // Limitar torques
    const float MAX_TORQUE = 0.5f;
    torque_roll = fmaxf(-MAX_TORQUE, fminf(MAX_TORQUE, torque_roll));
    torque_pitch = fmaxf(-MAX_TORQUE, fminf(MAX_TORQUE, torque_pitch));
    torque_yaw = fmaxf(-MAX_TORQUE, fminf(MAX_TORQUE, torque_yaw));

    // Preparar salida
    output_.torque_roll = torque_roll;
    output_.torque_pitch = torque_pitch;
    output_.torque_yaw = torque_yaw;
    output_.thrust = fmaxf(0, fminf(thrust_max_, setpoint.thrust));
    output_.timestamp_us = state.timestamp_us;
    output_.valid = true;

    is_healthy_ = true;

    last_roll_sp_ = setpoint.roll_rad;
    last_pitch_sp_ = setpoint.pitch_rad;
    last_yaw_sp_ = setpoint.yaw_rad;

    return true;
}

ControlOutput QuaternionAttitudeController::get_output() const {
    return output_;
}

void QuaternionAttitudeController::reset() {
    memset(&output_, 0, sizeof(output_));
    q_desired_ = Quaternion(1, 0, 0, 0);
    last_roll_sp_ = 0;
    last_pitch_sp_ = 0;
    last_yaw_sp_ = 0;
    is_healthy_ = false;
}

void QuaternionAttitudeController::euler_to_quaternion(float roll, float pitch, float yaw, 
                                                       Quaternion& q) {
    // Convertir ángulos de Euler a cuaternión
    // Convención: ZYX (yaw-pitch-roll)
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

    quaternion_normalize(q);
}

Quaternion QuaternionAttitudeController::quaternion_error(const Quaternion& q_desired,
                                                          const Quaternion& q_current) {
    // q_error = q_desired * inv(q_current)
    Quaternion q_current_inv = quaternion_inverse(q_current);
    return quaternion_multiply(q_desired, q_current_inv);
}

void QuaternionAttitudeController::quaternion_error_to_torques(const Quaternion& q_error, 
                                                               float dt,
                                                               float& torque_roll, 
                                                               float& torque_pitch, 
                                                               float& torque_yaw) {
    // Convertir componentes imaginarios del error a torques
    // El vector imaginario de q_error representa el eje de rotación * sin(angle/2)
    // Multiplicar por ganancia proporcional para obtener torques

    torque_roll = kp_attitude_ * q_error.x;
    torque_pitch = kp_attitude_ * q_error.y;
    torque_yaw = kp_attitude_ * q_error.z;
}

Quaternion QuaternionAttitudeController::quaternion_inverse(const Quaternion& q) {
    // Para un cuaternión unitario: inv(q) = conj(q)
    return Quaternion(q.w, -q.x, -q.y, -q.z);
}

Quaternion QuaternionAttitudeController::quaternion_multiply(const Quaternion& q1, 
                                                             const Quaternion& q2) {
    // q1 * q2 = [w1*w2 - v1·v2, w1*v2 + w2*v1 + v1×v2]
    float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

    return Quaternion(w, x, y, z);
}

void QuaternionAttitudeController::quaternion_normalize(Quaternion& q) {
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 0.0001f) {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    }
}

} // namespace stallion
