#include "pso_interface.hpp"

PSOInterface::PSOInterface(const std::string &config_path,
                           pso_runtime::RuntimeLimits limits,
                           const std::string &vehicle_path)
    : optimizer_(config_path, vehicle_path), limits_(limits),
      center_(optimizer_.initialActuatorState()) {}

void PSOInterface::reset() { center_ = optimizer_.initialActuatorState(); }

pso_runtime::Result PSOInterface::runPSO(const DroneState &state) {
  pso_runtime::VehicleState vehicle;
  // PX4 and the predictor share NED world coordinates and FRD body axes.
  vehicle.position = {state.local_x, state.local_y, state.local_z};
  vehicle.velocity = {state.velocity_x, state.velocity_y, state.velocity_z};
  vehicle.orientation = {state.roll, state.pitch, state.yaw};
  vehicle.angular_rate = {state.gyro_x, state.gyro_y, state.gyro_z};
  auto result = optimizer_.solve(vehicle, state.target, center_, limits_);
  center_ = result.actuators;
  return result;
}
