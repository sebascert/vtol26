#include "pso_interface.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#define CHECK(condition) do { if (!(condition)) { std::cerr << "Line " << __LINE__ << ": " #condition << '\n'; std::exit(1); } } while (0)

using Vector = std::array<double, 3>;
using Actuators = std::array<double, DIMENSION_ESTADO>;
const double pi = std::acos(-1.0);
const char* no_surfaces = R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})";

template<class Values>
void writeVector(std::ostream& out, const Values& values) {
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << values[i];
    out << ']';
}

void configure(const char* path, const Actuators& actuators, const Vector& target,
               const Vector& attitude, const char* surfaces) {
    std::ofstream out(path);
    CHECK(out.good());
    out << std::setprecision(17)
        << R"({"physics":{"rho":1,"g":10,"mass":1.1,"inertia":[1,1,1]},)"
        << R"("propulsion":{"thrust_coefficient":1},)" << surfaces
        << R"(,"cost_function":{"dt":0.1,"k_dynamics":0,"k_pos":1,"k_att":1},)"
        << R"("target_pose":{"position":)";
    writeVector(out, target);
    out << R"(,"orientation":)";
    writeVector(out, attitude);
    out << R"(},"actuators":{"min":)";
    writeVector(out, actuators);
    out << R"(,"max":)";
    writeVector(out, actuators);
    out << R"(,"initial":)";
    writeVector(out, actuators);
    out << "}}";
    out.close();
    CHECK(out.good());
}

void checkPrediction(const char* path, const Actuators& actuators,
                     const pso_runtime::VehicleState& state, const Vector& position,
                     const Vector& attitude = {}, const char* surfaces = no_surfaces) {
    // Lock all inputs so random search cannot conceal a wrong force, frame or moment sign.
    configure(path, actuators, position, attitude, surfaces);
    pso_runtime::Optimizer optimizer(path);
    const auto result = optimizer.solve(state, actuators, {2,1,1});
    CHECK(result.position_error_m < 1e-10);
    CHECK(result.fitness < 1e-10);
}

int main(int argc, char** argv) {
    CHECK(argc == 2);
    const auto path = argv[1];
    Actuators zero{};
    pso_runtime::VehicleState state;

    // NED free fall: gravity increases down position; velocity is already in NED.
    state.position = {1,2,-3};
    state.velocity = {4,5,6};
    checkPrediction(path, zero, state, {1.4,2.5,-2.35});
    state = {};

    // Balanced 11 N thrust supports 1.1 kg at g=10; front/rear pitch moments cancel.
    Actuators hover{3,3,5,0,0,0,0,0,0};
    checkPrediction(path, hover, state, {});
    state.orientation = {pi/2,0,0};
    checkPrediction(path, hover, state, {0,0.05,0.05}, state.orientation);

    // Positive body pitch rotates upward thrust toward negative north.
    state.orientation = {0,pi/6,0};
    checkPrediction(path, hover, state, {-0.025,0,0.05*(1-std::cos(pi/6))}, state.orientation);

    // Fully tilted front rotors point forward; yaw=90 degrees sends that force east.
    state.orientation = {0,0,pi/2};
    Actuators forward{3,3,0,pi/2,pi/2,0,0,0,0};
    checkPrediction(path, forward, state, {0,0.03/1.1,0.05}, state.orientation);
    state = {};

    // r cross F in FRD: left upward thrust gives positive roll, front gives positive pitch.
    checkPrediction(path, {6,0,5,0,0,0,0,0,0}, state, {}, {0.0075,0,0});
    checkPrediction(path, {3,3,0,0,0,0,0,0,0}, state, {0,0,0.05-0.03/1.1}, {0,0.015,0});
    checkPrediction(path, {2.2,0,0,pi/2,0,0,0,0,0}, state, {0.01,0,0.05}, {0,0,0.00275});

    // Body yaw rate at roll=90 degrees changes Euler pitch, not Euler yaw.
    state.orientation = {pi/2,0,0};
    state.angular_rate = {0,0,0.2};
    checkPrediction(path, zero, state, {0,0,0.05}, {pi/2,-0.02,0});
    state = {};

    // Wing lift is negative body Z; drag is negative X. Its forward arm pitches up.
    state.airspeed_m_s = 10;
    checkPrediction(path, zero, state, {-0.25/1.1,0,0.05-0.25/1.1}, {0,0.15,0},
        R"("wing":{"area":1,"cl0":1,"cl_alpha":0,"cd0":1,"k":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})");

    // Equal V-tail commands cancel side force and yaw/roll, retaining upward lift and aft pitch.
    state.airspeed_m_s = std::sqrt(2.0);
    const double cant = 0.785;
    checkPrediction(path, {0,0,0,0,0,0.1,0.1,0,0}, state,
        {-0.001/1.1,0,0.05-0.001*std::cos(cant)/1.1}, {0,-0.0007*std::cos(cant),0},
        R"("wing":{"area":0},"tail":{"area":1,"cl_gamma":1,"cd_gamma":1,"cant_angle":0.785},"ruddervator":{"area":0},"aileron":{"area":0})");
    checkPrediction(path, {0,0,0,0,0,0.1,0.1,0,0}, state,
        {-0.001/1.1,0,0.05-0.001*std::cos(cant)/1.1}, {0,-0.00075*std::cos(cant),0},
        R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":1,"cl":1,"cd":1,"cant_angle":0.785},"aileron":{"area":0})");

    // Same-sign aileron commands produce opposite lift increments and negative roll.
    checkPrediction(path, {0,0,0,0,0,0,0,0.1,0.1}, state,
        {-0.001/1.1,0,0.05}, {-0.0003,0,0},
        R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":1,"cl_beta":1,"cd_beta":1})");

    // The telemetry adapter must forward NED position/velocity and FRD attitude/rates unchanged.
    configure(path, hover, {0,0,-10}, {}, no_surfaces);
    DroneState telemetry;
    telemetry.local_x=1; telemetry.local_y=2; telemetry.local_z=-3;
    telemetry.velocity_x=4; telemetry.velocity_y=5; telemetry.velocity_z=-6;
    telemetry.roll=0.1f; telemetry.pitch=-0.2f; telemetry.yaw=0.3f;
    telemetry.gyro_x=0.4f; telemetry.gyro_y=-0.5f; telemetry.gyro_z=0.6f;
    state.position = {telemetry.local_x,telemetry.local_y,telemetry.local_z};
    state.velocity = {telemetry.velocity_x,telemetry.velocity_y,telemetry.velocity_z};
    state.orientation = {telemetry.roll,telemetry.pitch,telemetry.yaw};
    state.angular_rate = {telemetry.gyro_x,telemetry.gyro_y,telemetry.gyro_z};
    state.airspeed_m_s = telemetry.airspeed;
    const auto expected = pso_runtime::Optimizer(path).solve(state, hover, {2,1,1});
    const auto actual = PSOInterface(path, {2,1,1}).runPSO(telemetry);
    CHECK(std::abs(actual.fitness-expected.fitness) < 1e-10);
    CHECK(std::abs(actual.position_error_m-expected.position_error_m) < 1e-10);
    std::filesystem::remove(path);
}
