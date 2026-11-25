#pragma once

#include <cerrno>
#include <termios.h>

inline bool serialBaudSpeed(int baud_rate, speed_t &speed) {
  switch (baud_rate) {
  case 9600:
    speed = B9600;
    break;
  case 19200:
    speed = B19200;
    break;
  case 38400:
    speed = B38400;
    break;
  case 57600:
    speed = B57600;
    break;
  case 115200:
    speed = B115200;
    break;
#ifdef B230400
  case 230400:
    speed = B230400;
    break;
#endif
  default:
    return false;
  }

  return true;
}

inline bool configureSerialPort(int fd, int baud_rate, int &error_number) {
  speed_t speed{};
  if (!serialBaudSpeed(baud_rate, speed)) {
    error_number = EINVAL;
    return false;
  }

  termios options{};
  if (tcgetattr(fd, &options) == -1) {
    error_number = errno;
    return false;
  }

  cfmakeraw(&options);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8 | CLOCAL | CREAD;
  options.c_cflag &= ~(PARENB | CSTOPB);
#ifdef CRTSCTS
  options.c_cflag &= ~CRTSCTS;
#endif
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (cfsetispeed(&options, speed) == -1 ||
      cfsetospeed(&options, speed) == -1 ||
      tcsetattr(fd, TCSANOW, &options) == -1) {
    error_number = errno;
    return false;
  }

  error_number = 0;
  return true;
}
