#pragma once

#include <string>

struct ApplicationConfig {
  std::string uart_port = "/dev/serial0";
  int baud_rate = 57600;
  int loop_period_ms = 20;
  int heartbeat_timeout_ms = 3000;
  std::string pso_config_path = "src/pso/config.json";
  int pso_period_ms = 1000;
  int pso_max_particles = 16;
  int pso_max_iterations = 5;
  int pso_external_cycles = 1;
};

bool parseCommandLine(int argc, char *argv[], ApplicationConfig &config,
                      bool &show_help, std::string &error);

void printUsage(const char *program_name);
