#pragma once

#include "control/control_algorithm.hh"

namespace pvtol {

class QuaternionAlgorithm final : public ControlAlgorithm {
   public:
    explicit QuaternionAlgorithm(PvtolParameters parameters = {});

    gz::math::Vector3d DesiredAcceleration(
        const PvtolState& state) const override;

    PvtolInput Compute(
        const PvtolState& state,
        const gz::math::Vector3d& commandedAcceleration) override;

   private:
    PvtolParameters parameters_;

    gz::math::Vector3d kpPosition_{3.1622, 3.1622, 3.1622};
    gz::math::Vector3d kdPosition_{2.7063, 2.7063, 2.7063};

    gz::math::Vector3d kpAttitude_{100.0, 100.0, 40.0};
    gz::math::Vector3d kdAttitude_{14.142, 14.142, 10.0};

    bool havePreviousDesired_{false};
    gz::math::Quaterniond previousDesired_{};
};

}  // namespace pvtol
