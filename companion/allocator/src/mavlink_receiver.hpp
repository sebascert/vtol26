#pragma once

#include "drone_state.hpp"

class MAVLinkReceiver
{
public:
    ~MAVLinkReceiver();
    bool init(const char* port="/dev/serial0", int baud_rate=57600);

    bool receive(DroneState& state);

private:
    int uart_fd = -1;
    unsigned source_system_ = 0, source_component_ = 0;
};