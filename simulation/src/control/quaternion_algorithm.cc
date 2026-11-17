#include "control/quaternion_algorithm.hh"

#include <cmath>

namespace pvtol {
namespace {

using gz::math::Quaterniond;
using gz::math::Vector3d;

bool Finite(const Vector3d& v) {
    return std::isfinite(v.X()) &&
           std::isfinite(v.Y()) &&
           std::isfinite(v.Z());
}

bool Finite(const Quaterniond& q) {
    return std::isfinite(q.W()) &&
           std::isfinite(q.X()) &&
           std::isfinite(q.Y()) &&
           std::isfinite(q.Z());
}

Vector3d RotationVector(Quaterniond q) {
    q.Normalize();

    if (q.W() < 0.0) {
        q = Quaterniond(-q.W(), -q.X(), -q.Y(), -q.Z());
    }

    const Vector3d v(q.X(), q.Y(), q.Z());
    const double length = v.Length();

    if (length < 1e-9) {
        return v * 2.0;
    }

    return v * (2.0 * std::atan2(length, q.W()) / length);
}

}  // namespace

QuaternionAlgorithm::QuaternionAlgorithm(PvtolParameters parameters)
    : parameters_(parameters) {
}

Vector3d QuaternionAlgorithm::DesiredAcceleration(
    const PvtolState& state) const {

    // Paper (17): u_pos = -K_pos * ([p; p_dot] - [p_d; 0]).
    const Vector3d uPos(
        -kpPosition_.X() * state.positionError.X() -
            kdPosition_.X() * state.linearVelocity.X(),
        -kpPosition_.Y() * state.positionError.Y() -
            kdPosition_.Y() * state.linearVelocity.Y(),
        -kpPosition_.Z() * state.positionError.Z() -
            kdPosition_.Z() * state.linearVelocity.Z());

    // Paper (15)-(16): u_pd = u_pos - g_vector.
    // Here g_vector = (0, 0, -gravity).
    return uPos + Vector3d(0.0, 0.0, parameters_.gravity);
}

PvtolInput QuaternionAlgorithm::Compute(
    const PvtolState& state,
    const Vector3d& commandedAcceleration) {

    if (!Finite(state.orientation) ||
        !Finite(state.angularVelocity) ||
        !Finite(commandedAcceleration)) {
        return {};
    }

    const double accelerationMagnitude =
        commandedAcceleration.Length();

    if (!std::isfinite(accelerationMagnitude) ||
        accelerationMagnitude < 1e-9) {
        return {};
    }

    // Paper (14): construct the quaternion that rotates the body thrust
    // axis b onto the commanded world-frame acceleration direction.
    const Vector3d b(0.0, 0.0, 1.0);
    const Vector3d direction =
        commandedAcceleration / accelerationMagnitude;
    const double scalarPart = 1.0 + b.Dot(direction);
    const Vector3d vectorPart = b.Cross(direction);

    if (scalarPart < 1e-9) {
        return {};
    }

    Quaterniond desiredOrientation(
        scalarPart,
        vectorPart.X(),
        vectorPart.Y(),
        vectorPart.Z());
    desiredOrientation.Normalize();

    Vector3d desiredAngularVelocity(0.0, 0.0, 0.0);

    if (havePreviousDesired_ &&
        std::isfinite(state.dt) &&
        state.dt > 0.0 &&
        state.dt <= 0.1) {

        const Quaterniond change =
            previousDesired_.Inverse() * desiredOrientation;
        desiredAngularVelocity =
            RotationVector(change) / state.dt;
    }

    previousDesired_ = desiredOrientation;
    havePreviousDesired_ = true;

    Quaterniond orientationError =
        desiredOrientation.Inverse() * state.orientation;
    orientationError.Normalize();

    const Vector3d thetaError =
        RotationVector(orientationError);
    const Vector3d desiredRateInCurrentBody =
        orientationError.RotateVectorReverse(
            desiredAngularVelocity);
    const Vector3d rateError =
        state.angularVelocity - desiredRateInCurrentBody;

    // Paper (11)-(12): u = -K_att * [theta_error; omega_error].
    const Vector3d u(
        -kpAttitude_.X() * thetaError.X() -
            kdAttitude_.X() * rateError.X(),
        -kpAttitude_.Y() * thetaError.Y() -
            kdAttitude_.Y() * rateError.Y(),
        -kpAttitude_.Z() * thetaError.Z() -
            kdAttitude_.Z() * rateError.Z());

    const Vector3d Jomega(
        parameters_.inertia.X() * state.angularVelocity.X(),
        parameters_.inertia.Y() * state.angularVelocity.Y(),
        parameters_.inertia.Z() * state.angularVelocity.Z());

    // Paper (10)-(11): tau = J*u + omega cross (J*omega).
    const Vector3d bodyTorque(
        parameters_.inertia.X() * u.X(),
        parameters_.inertia.Y() * u.Y(),
        parameters_.inertia.Z() * u.Z());

    const Vector3d compensatedTorque =
        bodyTorque + state.angularVelocity.Cross(Jomega);

    if (!Finite(compensatedTorque)) {
        return {};
    }

    // Paper (14): F_th = m * ||u_p||.
    return {
        parameters_.mass * accelerationMagnitude,
        compensatedTorque,
        true
    };
}

}  // namespace pvtol
