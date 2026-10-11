#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#include <common/mavlink.h>

int main() {

    int uart = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);

    if (uart == -1) {
        std::cerr << "No se pudo abrir UART\n";
        return -1;
    }

    struct termios options;
    tcgetattr(uart, &options);

    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);

    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;

    tcsetattr(uart, TCSANOW, &options);

    std::cout << "UART + MAVLink listo\n";

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    while (true) {

        mavlink_msg_heartbeat_pack(
            1, 200,
            &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_INVALID,
            0, 0,
            MAV_STATE_ACTIVE
        );

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        write(uart, buf, len);

        std::cout << "Heartbeat enviado\n";

        usleep(1000000);
    }
}
