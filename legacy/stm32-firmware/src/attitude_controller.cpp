#include "attitude_controller.hpp"

namespace stallion {

Controller::Controller() {}

bool Controller::update(const StateEstimate& state,
                        const AttitudeSetpoint& sp,
                        float dt) {

    output_.motor_cmd[0] = sp.thrust_normalized;
    output_.motor_cmd[1] = sp.thrust_normalized;
    output_.motor_cmd[2] = sp.thrust_normalized;
    output_.motor_cmd[3] = sp.thrust_normalized;

    output_.timestamp_us = state.timestamp_us;
    output_.valid = true;

    return true;
}

ControlOutput Controller::get_output() const {
    return output_;
}

bool Controller::is_healthy() const {
    return true;
}

}