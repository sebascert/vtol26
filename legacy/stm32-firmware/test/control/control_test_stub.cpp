#include "aero_force_model.hpp"

int main() {
  stallion::AeroModelParams params {};
  params.rho = 1.225f;
  params.area_wing = 0.22f;
  params.cl_alpha = 5.5f;

  stallion::AeroForceModel model(params);
  stallion::AeroModelInput in {};
  in.alpha_rad = 0.1f;
  in.airspeed_mps = 12.0f;
  const auto out = model.compute(in);

  return (out.total.x == out.total.x) ? 0 : 1;
}
