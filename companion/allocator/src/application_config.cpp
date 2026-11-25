#include "application_config.hpp"
#include "serial_port.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool parseInteger(const char *text, int minimum, int maximum, int &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (*end != '\0' || parsed < minimum || parsed > maximum) {
    return false;
  }

  value = static_cast<int>(parsed);
  return true;
}

} // namespace

bool parseCommandLine(int argc, char *argv[], ApplicationConfig &config,
                      bool &show_help, std::string &error) {
  show_help = false;
  error.clear();

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      show_help = true;
      return true;
    }

    if (i + 1 >= argc) {
      error = "Missing value for " + argument;
      return false;
    }

    const char *value = argv[++i];
    if (argument == "--port") {
      if (*value == '\0') {
        error = "UART port cannot be empty";
        return false;
      }
      config.uart_port = value;
    } else if (argument == "--baud") {
      int baud_rate = 0;
      speed_t speed{};
      if (!parseInteger(value, 1200, 1000000, baud_rate) ||
          !serialBaudSpeed(baud_rate, speed)) {
        error = "Unsupported baud rate: " + std::string(value);
        return false;
      }
      config.baud_rate = baud_rate;
    } else if (argument == "--loop-ms") {
      if (!parseInteger(value, 1, 1000, config.loop_period_ms)) {
        error = "Invalid loop period: " + std::string(value);
        return false;
      }
    } else if (argument == "--heartbeat-timeout-ms") {
      if (!parseInteger(value, 100, 60000, config.heartbeat_timeout_ms)) {
        error = "Invalid heartbeat timeout: " + std::string(value);
        return false;
      }
    } else if (argument == "--pso-config") {
      if (*value == '\0') {
        error = "PSO config path cannot be empty";
        return false;
      }
      config.pso_config_path = value;
    } else if (argument == "--pso-period-ms") {
      if (!parseInteger(value, 10, 60000, config.pso_period_ms)) {
        error = "Invalid PSO period: " + std::string(value);
        return false;
      }
    } else if (argument == "--pso-max-particles") {
      if (!parseInteger(value, 2, 256, config.pso_max_particles)) {
        error = "Invalid PSO particle cap: " + std::string(value);
        return false;
      }
    } else if (argument == "--pso-max-iterations") {
      if (!parseInteger(value, 1, 100, config.pso_max_iterations)) {
        error = "Invalid PSO iteration cap: " + std::string(value);
        return false;
      }
    } else if (argument == "--pso-external-cycles") {
      if (!parseInteger(value, 1, 30, config.pso_external_cycles)) {
        error = "Invalid PSO cycle cap: " + std::string(value);
        return false;
      }
    } else {
      error = "Unknown argument: " + argument;
      return false;
    }
  }

  return true;
}

void printUsage(const char *program_name) {
  const char *name =
      (program_name != nullptr) ? program_name : "vtol_allocator";
  std::cout
      << "Usage: " << name << " [options]\n"
      << "  --port PATH                  UART device (default: /dev/serial0)\n"
      << "  --baud RATE                  UART baud rate (default: 57600)\n"
      << "  --loop-ms N                  Main loop period (default: 20)\n"
      << "  --heartbeat-timeout-ms N     PX4 heartbeat timeout (default: "
         "3000)\n"
      << "  --pso-config PATH            PSO tuning (default: "
         "src/pso/config.json)\n"
      << "  --pso-period-ms N            Allocation period (default: 1000)\n"
      << "  --pso-max-particles N        Runtime cap (default: 16)\n"
      << "  --pso-max-iterations N       Runtime cap (default: 5)\n"
      << "  --pso-external-cycles N      Runtime cap (default: 1)\n"
      << "  --help                       Show this help\n";
}
