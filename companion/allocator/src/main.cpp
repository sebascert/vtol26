#include "application_config.hpp"
#include "mavlink_receiver.hpp"
#include "mavlink_sender.hpp"
#include "pso_interface.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
} // namespace

int main(int argc, char *argv[]) {
  ApplicationConfig config;
  bool help = false;
  std::string error;
  if (!parseCommandLine(argc, argv, config, help, error)) {
    std::cerr << error << '\n';
    return 1;
  }
  if (help) {
    printUsage(argv[0]);
    return 0;
  }
  try {
    PSOInterface pso(config.pso_config_path,
                     {static_cast<size_t>(config.pso_max_particles),
                      config.pso_max_iterations, config.pso_external_cycles});
    std::cerr << "Vehicle configuration: "
              << vehicleConfigDirectory() + "/vehicle.json" << '\n';
    MAVLinkReceiver receiver;
    MAVLinkSender sender;
    if (!receiver.init(config.uart_port.c_str(), config.baud_rate) ||
        !sender.init(config.uart_port.c_str(), config.baud_rate)) {
      std::cerr << "Cannot configure UART: " << config.uart_port << '\n';
      return 1;
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    DroneState state;
    auto next_heartbeat = DroneState::Clock::now();
    auto next_pso = next_heartbeat;
    bool was_ready = false;
    std::cerr << "Waiting for fresh PX4 state and pose targets.\n";
    while (running) {
      if (!receiver.receive(state)) {
        std::cerr << "UART receive failed\n";
        return 1;
      }
      const auto now = DroneState::Clock::now();
      if (now >= next_heartbeat) {
        if (!sender.sendHeartbeat()) {
          std::cerr << "Heartbeat write failed\n";
          return 1;
        }
        next_heartbeat = now + std::chrono::seconds(1);
      }
      const bool ready = state.ready(
          now, std::chrono::milliseconds(config.heartbeat_timeout_ms));
      if (!ready && was_ready) {
        pso.reset();
        std::cerr << "State or target stale/invalid; allocations suspended\n";
      }
      was_ready = ready;
      if (ready && now >= next_pso) {
        const auto inputs = state;
        const auto solve_started = DroneState::Clock::now();
        const auto solution = pso.runPSO(inputs);
        // Refresh the link after optimization; never publish a result
        // after its inputs expire.
        if (!receiver.receive(state))
          return 1;
        if (inputs.ready(
                DroneState::Clock::now(),
                std::chrono::milliseconds(config.heartbeat_timeout_ms)) &&
            state.ready(
                DroneState::Clock::now(),
                std::chrono::milliseconds(config.heartbeat_timeout_ms)) &&
            inputs.target.position == state.target.position &&
            inputs.target.orientation == state.target.orientation) {
          if (!sender.sendPSOCommand(solution, &inputs)) {
            std::cerr << "Allocation write failed\n";
            return 1;
          }
          std::cout << std::setprecision(12) << "{\"actuators\":[";
          for (size_t i = 0; i < solution.actuators.size(); ++i)
            std::cout << (i ? "," : "") << solution.actuators[i];
          std::cout << "],\"fitness\":" << solution.fitness
                    << ",\"position_error_m\":" << solution.position_error_m
                    << ",\"position_converged\":"
                    << (solution.position_converged ? "true" : "false")
                    << ",\"objective_evaluations\":"
                    << solution.objective_evaluations;
          std::cout << ",\"source_state_boot_ms\":"
                    << std::min(inputs.position_boot_ms,
                                inputs.attitude_boot_ms)
                    << ",\"solve_duration_ms\":"
                    << std::chrono::duration<double, std::milli>(
                           DroneState::Clock::now() - solve_started)
                           .count();
          std::cout << ",\"state_position_ned\":[" << inputs.local_x << ','
                    << inputs.local_y << ',' << inputs.local_z
                    << "],\"target_position_ned\":["
                    << inputs.target.position[0] << ','
                    << inputs.target.position[1] << ','
                    << inputs.target.position[2]
                    << "],\"target_orientation_rpy\":["
                    << inputs.target.orientation[0] << ','
                    << inputs.target.orientation[1] << ','
                    << inputs.target.orientation[2]
                    << "],\"position_target_boot_ms\":"
                    << inputs.position_target_boot_ms
                    << ",\"attitude_target_boot_ms\":"
                    << inputs.attitude_target_boot_ms << "}" << std::endl;
        }
        next_pso = DroneState::Clock::now() +
                   std::chrono::milliseconds(config.pso_period_ms);
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(config.loop_period_ms));
    }
  } catch (const std::exception &exception) {
    std::cerr << "Allocator error: " << exception.what() << '\n';
    return 1;
  }
  return 0;
}
