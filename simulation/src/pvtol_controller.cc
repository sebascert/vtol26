#include "pvtol_controller.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

#include "control/quaternion_algorithm.hh"

namespace pvtol {
namespace {

bool Finite(const gz::math::Vector3d& v) {
    return std::isfinite(v.X()) &&
           std::isfinite(v.Y()) &&
           std::isfinite(v.Z());
}

}  // namespace

void PvtolController::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) {

    const gz::sim::Model model(entity);
    const auto linkName =
        sdf->Get<std::string>("link_name", "body").first;

    linkEntity_ = model.LinkByName(ecm, linkName);

    if (linkEntity_ == gz::sim::kNullEntity) {
        return;
    }

    gz::sim::Link(linkEntity_).EnableVelocityChecks(ecm);

    target_ = sdf->Get<gz::math::Vector3d>(
        "target", gz::math::Vector3d(0.0, 0.0, 2.0)).first;

    algorithm_ = std::make_unique<QuaternionAlgorithm>(parameters_);
}

gz::math::Vector3d PvtolController::LimitAcceleration(
    gz::math::Vector3d desired) const {

    // Adapter policy: restrict requested motion to upward thrust
    // and the available tilt envelope.
    desired.Z(std::max(
        actuatorLimits_.minUpwardAcceleration,
        desired.Z()));

    const double horizontal =
        std::hypot(desired.X(), desired.Y());
    const double maxHorizontal =
        desired.Z() * std::tan(actuatorLimits_.maxTiltAngle);

    if (horizontal > maxHorizontal && horizontal > 1e-9) {
        const double scale = maxHorizontal / horizontal;
        desired.X(desired.X() * scale);
        desired.Y(desired.Y() * scale);
    }

    return desired;
}

void PvtolController::PreUpdate(
    const gz::sim::UpdateInfo& info,
    gz::sim::EntityComponentManager& ecm) {

    if (info.paused ||
        linkEntity_ == gz::sim::kNullEntity ||
        !algorithm_) {
        return;
    }

    const gz::sim::Link link(linkEntity_);
    const auto pose = link.WorldPose(ecm);
    const auto linearVelocity =
        link.WorldLinearVelocity(ecm);
    const auto worldAngularVelocity =
        link.WorldAngularVelocity(ecm);

    if (!pose || !linearVelocity || !worldAngularVelocity) {
        return;
    }

    const gz::math::Quaterniond orientation = pose->Rot();

    const PvtolState state{
        pose->Pos() - target_,
        *linearVelocity,
        orientation,
        orientation.RotateVectorReverse(
            *worldAngularVelocity),
        std::chrono::duration<double>(info.dt).count()
    };

    const auto desiredAcceleration =
        algorithm_->DesiredAcceleration(state);

    if (!Finite(desiredAcceleration)) {
        return;
    }

    const auto commandedAcceleration =
        LimitAcceleration(desiredAcceleration);

    const PvtolInput input =
        algorithm_->Compute(state, commandedAcceleration);

    if (!input.valid) {
        return;
    }

    const double thrust = std::clamp(
        input.thrust,
        0.0,
        actuatorLimits_.maxThrust);

    const gz::math::Vector3d bodyTorque(
        std::clamp(input.bodyTorque.X(),
                   -actuatorLimits_.maxTorque,
                   actuatorLimits_.maxTorque),
        std::clamp(input.bodyTorque.Y(),
                   -actuatorLimits_.maxTorque,
                   actuatorLimits_.maxTorque),
        std::clamp(input.bodyTorque.Z(),
                   -actuatorLimits_.maxTorque,
                   actuatorLimits_.maxTorque));

    link.AddWorldWrench(
        ecm,
        orientation.RotateVector(
            gz::math::Vector3d(0.0, 0.0, thrust)),
        orientation.RotateVector(bodyTorque));
}

}  // namespace pvtol
