#include "aero_force_model.hpp"

#include <cmath>

namespace stallion {

namespace {

float dynamicPressure(float rho, float airspeed_mps) {
  return 0.5f * rho * airspeed_mps * airspeed_mps;
}

}  // namespace

ForceBreakdown AeroForceModel::compute(const AeroModelInput& input) const {
  ForceBreakdown out {};
  const float q_dyn = dynamicPressure(params_.rho, input.airspeed_mps);

  // Ala principal.
  const float cl = params_.cl0 + params_.cl_alpha * input.alpha_rad;
  const float cd = params_.cd0 + params_.wing_k * cl * cl;
  const float d_wing = -cd * q_dyn * params_.area_wing;
  const float l_wing = cl * q_dyn * params_.area_wing;
  const Quaternion q_wind_to_body = quatFromAxisAngleY(-input.alpha_rad);
  out.wing = rotateVectorByQuat({0.0f, d_wing, l_wing}, q_wind_to_body);

  // Alerones.
  const float l_ail_l = -q_dyn * params_.cl_beta * input.aileron_beta_rad * params_.area_aileron;
  const float d_ail_l = -q_dyn * params_.cd_beta * std::fabs(input.aileron_beta_rad) * params_.area_aileron;
  const float l_ail_r = q_dyn * params_.cl_beta * input.aileron_beta_rad * params_.area_aileron;
  const float d_ail_r = -q_dyn * params_.cd_beta * std::fabs(input.aileron_beta_rad) * params_.area_aileron;
  out.aileron_left = rotateVectorByQuat({0.0f, d_ail_l, l_ail_l}, q_wind_to_body);
  out.aileron_right = rotateVectorByQuat({0.0f, d_ail_r, l_ail_r}, q_wind_to_body);

  // Motor (empuje + tilt en eje Y).
  const Quaternion q_motor = quatFromAxisAngleY(input.motor_tilt_rad);
  out.motor = rotateVectorByQuat({0.0f, 0.0f, input.thrust_n}, q_motor);

  // Cola izquierda y derecha (cant en eje X).
  const float d_tail_l = -q_dyn * params_.cd_gamma * std::fabs(input.gamma_left_rad) * params_.area_tail;
  const float l_tail_l = q_dyn * params_.cl_gamma * input.gamma_left_rad * params_.area_tail;
  const float d_tail_r = -q_dyn * params_.cd_gamma * std::fabs(input.gamma_right_rad) * params_.area_tail;
  const float l_tail_r = q_dyn * params_.cl_gamma * input.gamma_right_rad * params_.area_tail;
  const Quaternion q_tail_left {
      std::cos(-params_.tail_cant_angle_rad), std::sin(-params_.tail_cant_angle_rad), 0.0f, 0.0f};
  const Quaternion q_tail_right {
      std::cos(params_.tail_cant_angle_rad), std::sin(params_.tail_cant_angle_rad), 0.0f, 0.0f};
  out.tail_left = rotateVectorByQuat({0.0f, d_tail_l, l_tail_l}, q_tail_left);
  out.tail_right = rotateVectorByQuat({0.0f, d_tail_r, l_tail_r}, q_tail_right);

  // Ruddervators izquierdo y derecho.
  const float eff_l = input.alpha_rud_left_rad + input.gamma_rud_left_rad;
  const float eff_r = input.alpha_rud_right_rad + input.gamma_rud_right_rad;
  const float d_rud_l = -q_dyn * params_.cd_rud * std::fabs(eff_l) * params_.area_ruddervator;
  const float l_rud_l = q_dyn * params_.cl_rud * eff_l * params_.area_ruddervator;
  const float d_rud_r = -q_dyn * params_.cd_rud * std::fabs(eff_r) * params_.area_ruddervator;
  const float l_rud_r = q_dyn * params_.cl_rud * eff_r * params_.area_ruddervator;
  const Quaternion q_rud_left {
      std::cos(-params_.rud_cant_angle_rad), std::sin(-params_.rud_cant_angle_rad), 0.0f, 0.0f};
  const Quaternion q_rud_right {
      std::cos(params_.rud_cant_angle_rad), std::sin(params_.rud_cant_angle_rad), 0.0f, 0.0f};
  out.ruddervator_left = rotateVectorByQuat({0.0f, d_rud_l, l_rud_l}, q_rud_left);
  out.ruddervator_right = rotateVectorByQuat({0.0f, d_rud_r, l_rud_r}, q_rud_right);

  out.total = out.wing + out.motor + out.aileron_left + out.aileron_right +
              out.tail_left + out.tail_right + out.ruddervator_left + out.ruddervator_right;

  return out;
}

}  // namespace stallion
