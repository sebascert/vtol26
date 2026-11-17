#pragma once

#include "module_interfaces.hpp"
#include "module_data_types.hpp"
#include <Arduino.h>

namespace stallion {

/**
 * @brief Controlador PID para actitud (pitch/roll/yaw)
 * 
 * Implementa control PID cascado: attitude outer loop, rate inner loop
 */
class AttitudePIDController : public IAttitudeController {
public:
    AttitudePIDController();
    ~AttitudePIDController() override = default;
    
    bool update(const StateEstimate& state, const ControlSetpoint& setpoint, float dt) override;
    ControlOutput get_output() const override;
    bool is_healthy() const override;
    void reset() override;
    
    struct PIDGains {
        float kp = 0;
        float ki = 0;
        float kd = 0;
        float i_max = 0;
        float i_min = 0;
    };
    
    void set_rate_gains(PIDGains roll_rate, PIDGains pitch_rate, PIDGains yaw_rate);
    void set_attitude_gains(PIDGains roll_att, PIDGains pitch_att, PIDGains yaw_att);

private:
    ControlOutput output_;
    bool is_healthy_;
    
    float sp_roll_rad_;
    float sp_pitch_rad_;
    float sp_yaw_rad_;
    float sp_roll_rate_rads_;
    float sp_pitch_rate_rads_;
    float sp_yaw_rate_rads_;
    
    PIDGains rate_roll_;
    PIDGains rate_pitch_;
    PIDGains rate_yaw_;
    PIDGains att_roll_;
    PIDGains att_pitch_;
    PIDGains att_yaw_;
    
    struct PIDState {
        float integral = 0;
        float prev_error = 0;
        float prev_derivative = 0;
    };
    
    PIDState rate_roll_state_;
    PIDState rate_pitch_state_;
    PIDState rate_yaw_state_;
    PIDState att_roll_state_;
    PIDState att_pitch_state_;
    PIDState att_yaw_state_;
    
    float pid_update(PIDGains& gains, PIDState& state, float error, float dt);
    void reset_pid_states();
    
    static constexpr float MAX_RATE_SP = 6.28f;
    static constexpr float MAX_ATT_SP = 0.52f;
};

} // namespace stallion
