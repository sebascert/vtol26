#include "pso_interface.hpp"
#include <iostream>

std::array<double, DIMENSION_ESTADO>
PSOInterface::runPSO(const DroneState& state)
{
    std::array<double, DIMENSION_ESTADO> solution;

    solution[0] = 1000;
    solution[1] = 1000;
    solution[2] = 1000;

    solution[3] = 0.1;
    solution[4] = 0.1;

    solution[5] = 0.0;
    solution[6] = 0.0;

    solution[7] = 0.0;
    solution[8] = 0.0;

    std::cout << "PSO ejecutado" << std::endl;

    return solution;
}