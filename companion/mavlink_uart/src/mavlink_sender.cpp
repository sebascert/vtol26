#include "mavlink_sender.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

#include <common/mavlink.h>

bool MAVLinkSender::init(const char* port)
{
    uart_fd = open(
        port,
        O_RDWR | O_NOCTTY | O_NDELAY
    );

    if(uart_fd == -1)
        return false;

    termios options;

    tcgetattr(uart_fd,&options);

    cfsetispeed(&options,B57600);
    cfsetospeed(&options,B57600);

    options.c_cflag =
        CS8 | CLOCAL | CREAD;

    tcsetattr(
        uart_fd,
        TCSANOW,
        &options
    );

    return true;
}


bool MAVLinkSender::sendHeartbeat()
{
    mavlink_message_t msg;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        255,
        0,
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE
    );

    uint16_t len =
        mavlink_msg_to_send_buffer(
            buffer,
            &msg
        );

    write(
        uart_fd,
        buffer,
        len
    );

    return true;
}

bool MAVLinkSender::sendPSOCommand(
    const std::array<double, DIMENSION_ESTADO>& cmd
)
{
    std::cout << "Enviando PSO: ";

    for(auto v : cmd)
        std::cout << v << " ";

    std::cout << std::endl;

    return true;
}