#include "pso_interface.hpp"
#include "mavlink_receiver.hpp"
#include "mavlink_sender.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <pty.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <unistd.h>

#define CHECK(condition) do { if (!(condition)) { std::cerr << "Check failed: " #condition << '\n'; std::exit(1); } } while (0)

int main(int argc, char** argv) {
    CHECK(argc == 3);
    pso_runtime::Optimizer optimizer(argv[1]);
    pso_runtime::VehicleState vehicle;
    const auto initial = optimizer.initialActuatorState();
    auto result = optimizer.solve(vehicle, initial, {16,5,1});
    CHECK(result.objective_evaluations == 97);
    CHECK(std::isfinite(result.fitness));
    CHECK(!result.position_converged); // Ten metres away in a 0.1-second model step.
    const auto config = CargadorConfiguracion::cargarDesdeJson(argv[1]);
    for (size_t i = 0; i < 9; ++i) {
        CHECK(result.actuators[i] >= config.actuator_min[i]);
        CHECK(result.actuators[i] <= config.actuator_max[i]);
    }
    auto again = optimizer.solve(vehicle, result.actuators, {16,5,1});
    CHECK(again.fitness <= result.fitness);
    vehicle.position[2] = 10;
    CHECK(optimizer.solve(vehicle, initial, {16,5,1}).position_converged);
    vehicle.position[0] = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try { optimizer.solve(vehicle, initial, {}); } catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
    DroneState state;
    CHECK(!state.ready(DroneState::Clock::now(), std::chrono::seconds(1)));

    int master, slave;
    char port[128];
    CHECK(openpty(&master, &slave, port, nullptr, nullptr) == 0);
    MAVLinkReceiver receiver;
    MAVLinkSender sender;
    CHECK(receiver.init(port)); CHECK(sender.init(port));
    auto inject = [&](const mavlink_message_t& message) {
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
        const auto size = mavlink_msg_to_send_buffer(bytes, &message);
        CHECK(write(master, bytes, size) == size);
        usleep(1000);
        CHECK(receiver.receive(state));
    };
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(1,1,&msg,MAV_TYPE_VTOL_TILTROTOR,MAV_AUTOPILOT_PX4,0,123456,MAV_STATE_ACTIVE);
    inject(msg); CHECK(state.flight_mode == 123456);
    mavlink_msg_attitude_pack(1,1,&msg,100,0.1f,0.2f,0.3f,0.4f,0.5f,0.6f);
    inject(msg); CHECK(state.gyro_x == 0.4f);
    mavlink_msg_local_position_ned_pack(1,1,&msg,100,1,2,-3,4,5,-6); inject(msg);
    mavlink_msg_vfr_hud_pack(1,1,&msg,10,0,0,0,0,0); inject(msg);
    CHECK(state.ready(DroneState::Clock::now(), std::chrono::seconds(1)));
    mavlink_msg_attitude_pack(2,1,&msg,100,9,9,9,9,9,9); inject(msg);
    CHECK(state.roll == 0.1f); // Other vehicles cannot overwrite the selected source.
    PSOInterface interface(argv[1]);
    auto allocation = interface.runPSO(state);
    CHECK(sender.sendPSOCommand(allocation));
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto size = read(master, bytes, sizeof(bytes));
    CHECK(size > 0);
    bool decoded = false;
    mavlink_status_t status{};
    for (ssize_t i = 0; i < size; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, bytes[i], &msg, &status)) {
            CHECK(msg.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY);
            mavlink_debug_float_array_t data{};
            mavlink_msg_debug_float_array_decode(&msg,&data);
            for (size_t d = 0; d < 9; ++d) CHECK(data.data[d] == static_cast<float>(allocation.actuators[d]));
            decoded = true;
        }
    }
    CHECK(decoded);
    CHECK(!state.ready(DroneState::Clock::now() + std::chrono::seconds(2), std::chrono::seconds(1)));
    // Invalid telemetry must invalidate freshness immediately.
    mavlink_msg_attitude_pack(1,1,&msg,100,std::numeric_limits<float>::quiet_NaN(),0,0,0,0,0);
    inject(msg);
    CHECK(!state.ready(DroneState::Clock::now(), std::chrono::seconds(1)));
    close(master); close(slave);

    bool bad_budget = false;
    try { optimizer.solve({}, initial, {0,5,1}); } catch (const std::invalid_argument&) { bad_budget = true; }
    CHECK(bad_budget);
    std::ifstream input(argv[1]);
    std::stringstream contents; contents << input.rdbuf();
    auto invalid = contents.str();
    const auto location = invalid.find("1000"); CHECK(location != std::string::npos);
    invalid.replace(location, 4, "9000"); // Initial rotor value exceeds maximum.
    const auto invalid_path = std::filesystem::path(argv[2]).parent_path() / "invalid_config.json";
    { std::ofstream output(invalid_path); output << invalid; }
    bool bad_config = false;
    try { pso_runtime::Optimizer bad(invalid_path.string()); } catch (const std::invalid_argument&) { bad_config = true; }
    CHECK(bad_config);
    std::filesystem::remove(invalid_path);

    // Exercise the actual UART executable, including startup, output and loss of telemetry.
    CHECK(openpty(&master, &slave, port, nullptr, nullptr) == 0);
    int output_pipe[2]; CHECK(pipe(output_pipe) == 0);
    const auto child = fork(); CHECK(child >= 0);
    if (child == 0) {
        close(master); close(slave); close(output_pipe[0]);
        dup2(output_pipe[1], STDOUT_FILENO); close(output_pipe[1]);
        execl(argv[2], argv[2], "--port", port, "--pso-config", argv[1],
              "--pso-period-ms", "100", "--heartbeat-timeout-ms", "200", "--loop-ms", "5", nullptr);
        _exit(127);
    }
    close(output_pipe[1]);
    pollfd output{output_pipe[0], POLLIN, 0};
    CHECK(poll(&output, 1, 50) == 0); // No allocation from default zero telemetry.
    auto transmit = [&](const mavlink_message_t& packet) {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        auto length = mavlink_msg_to_send_buffer(buffer, &packet);
        CHECK(write(master, buffer, length) == length);
    };
    mavlink_msg_heartbeat_pack(1,1,&msg,MAV_TYPE_VTOL_TILTROTOR,MAV_AUTOPILOT_PX4,0,0,MAV_STATE_ACTIVE); transmit(msg);
    mavlink_msg_attitude_pack(1,1,&msg,100,0,0,0,0,0,0); transmit(msg);
    mavlink_msg_local_position_ned_pack(1,1,&msg,100,0,0,0,0,0,0); transmit(msg);
    mavlink_msg_vfr_hud_pack(1,1,&msg,0,0,0,0,0,0); transmit(msg);
    CHECK(poll(&output, 1, 1000) > 0);
    char line[2048]{};
    CHECK(read(output_pipe[0], line, sizeof(line)-1) > 0);
    CHECK(std::string(line).find("\"actuators\":[") != std::string::npos);
    usleep(300000);
    while (poll(&output, 1, 0) > 0) CHECK(read(output_pipe[0], line, sizeof(line)) > 0);
    CHECK(poll(&output, 1, 150) == 0); // No cached allocation after the telemetry timeout.
    kill(child, SIGTERM);
    int status_code; CHECK(waitpid(child, &status_code, 0) == child);
    CHECK(WIFEXITED(status_code) && WEXITSTATUS(status_code) == 0);
    close(master); close(slave); close(output_pipe[0]);
}
