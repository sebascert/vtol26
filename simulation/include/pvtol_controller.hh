#pragma once

#include <memory>

#include <gz/math/Vector3.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

#include "control/control_algorithm.hh"

namespace pvtol {

struct ActuatorLimits {
    double minUpwardAcceleration{0.5};  // m/s^2
    double maxTiltAngle{0.6108652382};  // 35 degrees
    double maxThrust{30.0};             // N
    double maxTorque{3.0};              // N m per body axis
};

class PvtolController final : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate {
   public:
    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager&) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm) override;

   private:
    gz::math::Vector3d LimitAcceleration(
        gz::math::Vector3d desired) const;

    gz::sim::Entity linkEntity_{gz::sim::kNullEntity};
    std::unique_ptr<ControlAlgorithm> algorithm_;

    PvtolParameters parameters_{};
    ActuatorLimits actuatorLimits_{};
    gz::math::Vector3d target_{0.0, 0.0, 2.0};
};

}  // namespace pvtol
