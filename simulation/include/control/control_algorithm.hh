#pragma once

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace pvtol {

struct PvtolState {
    gz::math::Vector3d positionError{0.0, 0.0, 0.0};
    gz::math::Vector3d linearVelocity{0.0, 0.0, 0.0};
    gz::math::Quaterniond orientation{};
    gz::math::Vector3d angularVelocity{0.0, 0.0, 0.0};
    double dt{0.0};
};

struct PvtolInput {
    double thrust{0.0};
    gz::math::Vector3d bodyTorque{0.0, 0.0, 0.0};
    bool valid{false};
};

struct PvtolParameters {
    double mass{1.3};
    double gravity{9.81};
    gz::math::Vector3d inertia{0.0325, 0.0325, 0.0325};
};

class ControlAlgorithm {
   public:
    virtual ~ControlAlgorithm() = default;

    // Paper's requested world-frame acceleration, before actuator policy.
    virtual gz::math::Vector3d DesiredAcceleration(
        const PvtolState& state) const = 0;

    // Convert the acceleration accepted by the adapter into commands.
    virtual PvtolInput Compute(
        const PvtolState& state,
        const gz::math::Vector3d& commandedAcceleration) = 0;
};

}  // namespace pvtol
