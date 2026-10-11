#pragma once

#include "drone_state.hpp"
#include "pso/pso_state.hpp"

class PSOInterface
{
public:

    std::array<double, DIMENSION_ESTADO>
    runPSO(const DroneState& state);
};