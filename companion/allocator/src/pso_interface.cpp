#include "pso_interface.hpp"

PSOInterface::PSOInterface(const std::string& config_path, pso_runtime::RuntimeLimits limits)
    : optimizer_(config_path), limits_(limits), center_(optimizer_.initialActuatorState()) {}

void PSOInterface::reset() { center_ = optimizer_.initialActuatorState(); }

pso_runtime::Result PSOInterface::runPSO(const DroneState& state) {
    pso_runtime::VehicleState vehicle;
    // shortcut: preserve the experiment's positive-up model axes; validate the full
    // body/world frame convention before using these allocations for actuation.
    vehicle.position = {state.local_x, state.local_y, -state.local_z};
    vehicle.velocity = {state.velocity_x, state.velocity_y, -state.velocity_z};
    vehicle.orientation = {state.roll, state.pitch, state.yaw};
    vehicle.angular_rate = {state.gyro_x, state.gyro_y, state.gyro_z};
    vehicle.airspeed_m_s = state.airspeed;
    auto result = optimizer_.solve(vehicle, center_, limits_);
    center_ = result.actuators;
    return result;
}
