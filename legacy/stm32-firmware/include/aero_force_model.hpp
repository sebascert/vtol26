#pragma once

#include "math_types.hpp"

namespace stallion {

struct AeroModelParams {
  float rho {1.225f};

  float area_wing {0.0f};
  float cl0 {0.0f};
  float cl_alpha {0.0f};
  float cd0 {0.0f};
  float wing_k {0.0f};

  float area_tail {0.0f};
  float cl_gamma {0.0f};
  float cd_gamma {0.0f};
  float tail_cant_angle_rad {0.0f};

  float area_ruddervator {0.0f};
  float cl_rud {0.0f};
  float cd_rud {0.0f};
  float rud_cant_angle_rad {0.0f};

  float area_aileron {0.0f};
  float cl_beta {0.0f};
  float cd_beta {0.0f};
};

struct AeroModelInput {
  float alpha_rad {0.0f};
  float airspeed_mps {0.0f};
  float thrust_n {0.0f};
  float motor_tilt_rad {0.0f};

  float gamma_left_rad {0.0f};
  float gamma_right_rad {0.0f};

  float alpha_rud_left_rad {0.0f};
  float gamma_rud_left_rad {0.0f};
  float alpha_rud_right_rad {0.0f};
  float gamma_rud_right_rad {0.0f};

  float aileron_beta_rad {0.0f};
};

struct ForceBreakdown {
  Vec3 wing {};
  Vec3 motor {};
  Vec3 aileron_left {};
  Vec3 aileron_right {};
  Vec3 tail_left {};
  Vec3 tail_right {};
  Vec3 ruddervator_left {};
  Vec3 ruddervator_right {};
  Vec3 total {};
};

// Modelo de fuerzas traducido desde el script simbolico de Matlab/Simulink.
class AeroForceModel {
 public:
  explicit AeroForceModel(AeroModelParams params) : params_(params) {}

  ForceBreakdown compute(const AeroModelInput& input) const;

 private:
  AeroModelParams params_;
};

}  // namespace stallion
