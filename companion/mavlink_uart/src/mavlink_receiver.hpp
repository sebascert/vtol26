#pragma once

#include "drone_state.hpp"

class MAVLinkReceiver
{
public:
    bool init(const char* port="/dev/serial0");

    bool receive(DroneState& state);

private:
    int uart_fd = -1;
};