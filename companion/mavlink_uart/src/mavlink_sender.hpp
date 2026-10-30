#pragma once

#include "pso_runtime.hpp"
#include <common/mavlink.h>

class MAVLinkSender {
public:
    ~MAVLinkSender();
    bool init(const char* port="/dev/serial0", int baud_rate=57600);
    bool sendHeartbeat();
    bool sendPSOCommand(const pso_runtime::Result& result);
private:
    bool send(const mavlink_message_t& message);
    int uart_fd = -1;
};
