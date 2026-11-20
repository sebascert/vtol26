#pragma once

#include "config_loader.hpp"
#include "pso_state.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace pso_runtime {

struct VehicleState {
    std::array<double, 3> position{};    // Local NED [m], positive Z down.
    std::array<double, 3> orientation{}; // FRD-body-to-NED roll, pitch, yaw [rad].
    std::array<double, 3> velocity{};    // Local NED [m/s].
    std::array<double, 3> angular_rate{}; // FRD body angular velocity [rad/s].
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
