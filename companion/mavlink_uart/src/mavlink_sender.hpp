#pragma once

#include <array>
#include "pso/pso_state.hpp"

class MAVLinkSender
{
public:

    bool init(const char* port="/dev/serial0");

    bool sendHeartbeat();

    bool sendPSOCommand(const std::array<double, DIMENSION_ESTADO>& cmd);

private:

    int uart_fd = -1;
};