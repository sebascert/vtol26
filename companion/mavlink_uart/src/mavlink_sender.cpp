#include "mavlink_sender.hpp"
#include "mavlink_identity.hpp"
#include "serial_port.hpp"

#include <chrono>
#include <cmath>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

MAVLinkSender::~MAVLinkSender() { if (uart_fd >= 0) close(uart_fd); }

bool MAVLinkSender::init(const char* port, int baud_rate) {
    uart_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    int error = 0;
    if (uart_fd < 0) return false;
    if (!configureSerialPort(uart_fd, baud_rate, error)) {
        close(uart_fd); uart_fd = -1; errno = error; return false;
    }
    return true;
}

bool MAVLinkSender::send(const mavlink_message_t& message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const auto size = mavlink_msg_to_send_buffer(buffer, &message);
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (offset < size && std::chrono::steady_clock::now() < deadline) {
        const auto written = write(uart_fd, buffer + offset, size - offset);
        if (written > 0) { offset += written; continue; }
        if (written < 0 && errno == EINTR) continue;
        if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        pollfd descriptor{uart_fd, POLLOUT, 0};
        if (poll(&descriptor, 1, 10) < 0 && errno != EINTR) return false;
    }
    return offset == size;
}

bool MAVLinkSender::sendHeartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(MAVLINK_COMPANION_SYSTEM_ID, MAVLINK_COMPANION_COMPONENT_ID, &message,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    return send(message);
}

bool MAVLinkSender::sendPSOCommand(const pso_runtime::Result& result) {
    float data[58]{};
    for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
        if (!std::isfinite(result.actuators[i])) return false;
        data[i] = static_cast<float>(result.actuators[i]);
        if (!std::isfinite(data[i])) return false;
    }
    data[9] = static_cast<float>(result.fitness);
    data[10] = static_cast<float>(result.position_error_m);
    if (!std::isfinite(data[9]) || !std::isfinite(data[10])) return false;
    data[11] = result.position_converged ? 1.0f : 0.0f;
    data[12] = static_cast<float>(result.objective_evaluations);
    mavlink_message_t message{};
    const auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    // DEBUG_FLOAT_ARRAY carries all nine model values without pretending they are PWM setpoints.
    mavlink_msg_debug_float_array_pack(MAVLINK_COMPANION_SYSTEM_ID, MAVLINK_COMPANION_COMPONENT_ID,
        &message, time_us, "PSO_ALLOC", 0, data);
    return send(message);
}
