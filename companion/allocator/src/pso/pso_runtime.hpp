#pragma once

#include "config_loader.hpp"
#include "pso_state.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace pso_runtime {

struct VehicleState {
    // Model frame uses a positive-up Z axis. The companion currently converts
    // LOCAL_POSITION_NED Z and VZ to this convention before calling the solver.
    std::array<double, 3> position{};
    std::array<double, 3> orientation{};  // roll, pitch, yaw [rad]
    std::array<double, 3> velocity{};     // model frame [m/s]
    std::array<double, 3> angular_rate{}; // roll, pitch, yaw rates [rad/s]
    double airspeed_m_s = 0.0;
};

struct RuntimeLimits {
    std::size_t particles = 16;
    int iterations = 5;
    int external_cycles = 1;
};

struct Result {
    std::array<double, DIMENSION_ESTADO> actuators{};
    double fitness = 0.0;
    double position_error_m = 0.0;
    bool position_converged = false;
    std::size_t objective_evaluations = 0;
};

class Optimizer {
public:
    explicit Optimizer(const std::string& config_path);

    const std::array<double, DIMENSION_ESTADO>& initialActuatorState() const;
    Result solve(const VehicleState& state,
                 const std::array<double, DIMENSION_ESTADO>& search_center,
                 const RuntimeLimits& limits) const;

private:
    CargadorConfiguracion::Configuracion config_;
    std::array<double, DIMENSION_ESTADO> initial_actuator_state_{};
};

} // namespace pso_runtime
