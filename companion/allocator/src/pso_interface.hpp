#pragma once

#include "drone_state.hpp"
#include "pso_runtime.hpp"

class PSOInterface {
public:
  explicit PSOInterface(const std::string &config_path,
                        pso_runtime::RuntimeLimits limits = {},
                        const std::string &vehicle_path =
                            vehicleConfigDirectory() + "/vehicle.json");
  pso_runtime::Result runPSO(const DroneState &state);
  void reset();

private:
  pso_runtime::Optimizer optimizer_;
  pso_runtime::RuntimeLimits limits_;
  std::array<double, DIMENSION_ESTADO> center_;
};
