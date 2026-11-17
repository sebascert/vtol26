#include "attitude_pid_controller.hpp"
#include <cmath>

namespace stallion {

AttitudePIDController::AttitudePIDController()
    : is_healthy_(false),
      sp_roll_rad_(0), sp_pitch_rad_(0), sp_yaw_rad_(0),
      sp_roll_rate_rads_(0), sp_pitch_rate_rads_(0), sp_yaw_rate_rads_(0) {
    
    PIDGains default_rate = {1.0f, 0.1f, 0.05f, 0.5f, -0.5f};
    PIDGains default_att = {2.0f, 0.0f, 0.1f, 0.3f, -0.3f};
    
    set_rate_gains(default_rate, default_rate, default_rate);
    set_attitude_gains(default_att, default_att, default_att);
    
    reset_pid_states();
}

bool AttitudePIDController::update(const StateEstimate& state, const ControlSetpoint& setpoint, float dt) {
    if (dt <= 0 || dt > 0.1f) {
        is_healthy_ = false;
        return false;
    }
    
    sp_roll_rad_ = constrain(setpoint.roll_rad, -MAX_ATT_SP, MAX_ATT_SP);
    sp_pitch_rad_ = constrain(setpoint.pitch_rad, -MAX_ATT_SP, MAX_ATT_SP);
    sp_yaw_rad_ = setpoint.yaw_rad;
    
    float att_error_roll = sp_roll_rad_ - state.euler_roll_rad;
    float att_error_pitch = sp_pitch_rad_ - state.euler_pitch_rad;
    float att_error_yaw = sp_yaw_rad_ - state.euler_yaw_rad;
    
    while (att_error_yaw > M_PI) att_error_yaw -= 2*M_PI;
    while (att_error_yaw < -M_PI) att_error_yaw += 2*M_PI;
    
    sp_roll_rate_rads_ = pid_update(att_roll_, att_roll_state_, att_error_roll, dt);
    sp_pitch_rate_rads_ = pid_update(att_pitch_, att_pitch_state_, att_error_pitch, dt);
    sp_yaw_rate_rads_ = pid_update(att_yaw_, att_yaw_state_, att_error_yaw, dt);
    
    sp_roll_rate_rads_ = constrain(sp_roll_rate_rads_, -MAX_RATE_SP, MAX_RATE_SP);
    sp_pitch_rate_rads_ = constrain(sp_pitch_rate_rads_, -MAX_RATE_SP, MAX_RATE_SP);
    sp_yaw_rate_rads_ = constrain(sp_yaw_rate_rads_, -MAX_RATE_SP, MAX_RATE_SP);
    
    float rate_error_roll = sp_roll_rate_rads_ - state.rate_roll_rads;
    float rate_error_pitch = sp_pitch_rate_rads_ - state.rate_pitch_rads;
    float rate_error_yaw = sp_yaw_rate_rads_ - state.rate_yaw_rads;
    
    output_.torque_roll = pid_update(rate_roll_, rate_roll_state_, rate_error_roll, dt);
    output_.torque_pitch = pid_update(rate_pitch_, rate_pitch_state_, rate_error_pitch, dt);
    output_.torque_yaw = pid_update(rate_yaw_, rate_yaw_state_, rate_error_yaw, dt);
    
    output_.torque_roll = constrain(output_.torque_roll, -1.0f, 1.0f);
    output_.torque_pitch = constrain(output_.torque_pitch, -1.0f, 1.0f);
    output_.torque_yaw = constrain(output_.torque_yaw, -1.0f, 1.0f);
    
    output_.thrust = setpoint.thrust;
    output_.valid = true;
    output_.timestamp_us = state.timestamp_us;
    is_healthy_ = true;
    
    return true;
}

ControlOutput AttitudePIDController::get_output() const {
    return output_;
}

bool AttitudePIDController::is_healthy() const {
    return is_healthy_;
}

void AttitudePIDController::reset() {
    reset_pid_states();
    output_ = ControlOutput();
    output_.valid = false;
    is_healthy_ = false;
}

void AttitudePIDController::set_rate_gains(PIDGains roll_rate, PIDGains pitch_rate, PIDGains yaw_rate) {
    rate_roll_ = roll_rate;
    rate_pitch_ = pitch_rate;
    rate_yaw_ = yaw_rate;
}

void AttitudePIDController::set_attitude_gains(PIDGains roll_att, PIDGains pitch_att, PIDGains yaw_att) {
    att_roll_ = roll_att;
    att_pitch_ = pitch_att;
    att_yaw_ = yaw_att;
}

float AttitudePIDController::pid_update(PIDGains& gains, PIDState& state, float error, float dt) {
    float p_term = gains.kp * error;
    
    state.integral += error * dt;
    state.integral = constrain(state.integral, gains.i_min, gains.i_max);
    float i_term = gains.ki * state.integral;
    
    float derivative = (error - state.prev_error) / dt;
    derivative = 0.1f * derivative + 0.9f * state.prev_derivative;
    float d_term = gains.kd * derivative;
    
    float output = p_term + i_term + d_term;
    
    state.prev_error = error;
    state.prev_derivative = derivative;
    
    return output;
}

void AttitudePIDController::reset_pid_states() {
    rate_roll_state_ = {0, 0, 0};
    rate_pitch_state_ = {0, 0, 0};
    rate_yaw_state_ = {0, 0, 0};
    att_roll_state_ = {0, 0, 0};
    att_pitch_state_ = {0, 0, 0};
    att_yaw_state_ = {0, 0, 0};
}

} // namespace stallion
