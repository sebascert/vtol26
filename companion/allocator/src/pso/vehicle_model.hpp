#pragma once
#include "config_loader.hpp"
#include "pso_state.hpp"
#include <array>

namespace pso_runtime {

struct BodyWrench {
  std::array<double, 3>
      force{}; // FRD, N; gravity is applied by the integrator.
  std::array<double, 3> moment{}; // FRD, N m, about the CG.
};

void validateActuators(const CargadorConfiguracion::Configuracion &config,
                       const std::array<double, 9> &command);
BodyWrench bodyWrench(const CargadorConfiguracion::Configuracion &config,
                      const std::array<double, 3> &body_velocity_frd,
                      const std::array<double, 9> &command);

} // namespace pso_runtime
