#pragma once

#include "module_interfaces.hpp"

namespace stallion {

class Controller : public IController {
public:
    Controller();

    bool update(const StateEstimate& state,
                const AttitudeSetpoint& sp,
                float dt) override;

    ControlOutput get_output() const override;
    bool is_healthy() const override;

private:
    ControlOutput output_;
};

}