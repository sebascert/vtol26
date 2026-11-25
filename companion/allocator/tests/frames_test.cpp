#include "pso_interface.hpp"
#include <json-c/json.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << "Line " << __LINE__ << ": " #condition << '\n';             \
      std::exit(1);                                                            \
    }                                                                          \
  } while (0)

using Vector = std::array<double, 3>;
using Actuators = std::array<double, DIMENSION_ESTADO>;
const double pi = std::acos(-1.0);
const char *no_surfaces =
    R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})";

template <class Values>
void writeVector(std::ostream &out, const Values &values) {
  out << '[';
  for (size_t i = 0; i < values.size(); ++i)
    out << (i ? "," : "") << values[i];
  out << ']';
}

std::string vehiclePath(const char *path) {
  return std::string(path) + ".vehicle.json";
}

void configure(const char *path, const Actuators &actuators,
               const Vector &target, const Vector &attitude,
               const char *surfaces) {
  // Keep the shared vehicle schema, replacing only the terms under test.
  auto *vehicle = json_object_from_file(
      (vehicleConfigDirectory() + "/vehicle.json").c_str());
  CHECK(vehicle);
  auto object = [](json_object *parent, const char *key) {
    auto *value = json_object_object_get(parent, key);
    CHECK(value);
    return value;
  };
  auto set = [](json_object *parent, const char *key, double value) {
    json_object_object_add(parent, key, json_object_new_double(value));
  };
  auto *body = object(vehicle, "rigid_body");
  set(body, "mass_kg", 1.1);
  json_object_object_add(body, "inertia_kg_m2", json_tokener_parse("[1,1,1]"));
  set(object(vehicle, "environment"), "air_density_kg_m3", 1);
  set(object(vehicle, "environment"), "gravity_m_s2", 10);
  json_object_object_add(object(vehicle, "propulsion"), "k_t_n_per_command",
                         json_tokener_parse("[1,1,1]"));
  auto *legacy =
      json_tokener_parse(("{" + std::string(surfaces) + "}").c_str());
  const char *names[] = {"wing", "aileron", "tail", "ruddervator"};
  const char *shared[] = {"wing", "aileron", "cola", "ruddervator"};
  for (size_t i = 0; i < 4; ++i) {
    auto *old = object(legacy, names[i]);
    auto *surface = object(object(vehicle, "aerodynamics"), shared[i]);
    auto *area = json_object_object_get(old, "area");
    if (area && json_object_get_double(area) == 0) {
      if (i == 0) {
        for (const char *key :
             {"cl0", "cl_alpha_per_rad", "cd0", "induced_drag_factor"})
          set(surface, key, 0);
      } else {
        set(surface, "cl_per_rad", 0);
        set(surface, "cd_per_rad", 0);
      }
    } else {
      const char *old_keys[] = {
          "area",    "cl0",      "cl_alpha", "cd0", "k",  "cl_beta",
          "cd_beta", "cl_gamma", "cd_gamma", "cl",  "cd", "cant_angle"};
      const char *new_keys[] = {i == 0 ? "area_m2" : "area_per_side_m2",
                                "cl0",
                                "cl_alpha_per_rad",
                                "cd0",
                                "induced_drag_factor",
                                "cl_per_rad",
                                "cd_per_rad",
                                "cl_per_rad",
                                "cd_per_rad",
                                "cl_per_rad",
                                "cd_per_rad",
                                "cant_angle_rad"};
      for (size_t k = 0; k < 12; ++k) {
        auto *value = json_object_object_get(old, old_keys[k]);
        if (value)
          set(surface, new_keys[k], json_object_get_double(value));
      }
    }
  }
  json_object_put(legacy);
  auto *inputs = object(vehicle, "actuators");
  for (size_t i = 3; i < 9; ++i) {
    auto *input = json_object_array_get_idx(inputs, i);
    set(input, "min", std::min(0.0, actuators[i]) - 1);
    set(input, "max", std::max(0.0, actuators[i]) + 1);
  }
  CHECK(json_object_to_file(vehiclePath(path).c_str(), vehicle) == 0);
  json_object_put(vehicle);

  std::ofstream out(path);
  CHECK(out.good());
  out << std::setprecision(17)
      << R"({"cost_function":{"dt":0.1,"k_dynamics":0,"k_pos":1,"k_att":1},)"
      // Tiny search radius prevents optimization from concealing a wrong
      // prediction.
      << R"("pso":{"k_am":1e-12,"k_am_min":1e-12},"target_pose":{"position":)";
  writeVector(out, target);
  out << R"(,"orientation":)";
  writeVector(out, attitude);
  out << R"(},"actuators":{"initial":)";
  writeVector(out, actuators);
  out << "}}";
  out.close();
  CHECK(out.good());
}

void checkPrediction(const char *path, const Actuators &actuators,
                     const pso_runtime::VehicleState &state,
                     const Vector &position, const Vector &attitude = {},
                     const char *surfaces = no_surfaces) {
  // Negligible exploration cannot conceal a wrong force, frame or moment sign.
  configure(path, actuators, position, attitude, surfaces);
  pso_runtime::Optimizer optimizer(path, vehiclePath(path));
  const auto result =
      optimizer.solve(state, {position, attitude}, actuators, {2, 1, 1});
  CHECK(result.position_error_m < 1e-10);
  CHECK(result.fitness < 1e-10);
}

void checkConfiguredPrediction(const char *path, const Actuators &actuators,
                               const Vector &position, const Vector &attitude,
                               const std::function<void(json_object *)> &edit) {
  configure(path, actuators, position, attitude, no_surfaces);
  auto *vehicle = json_object_from_file(vehiclePath(path).c_str());
  CHECK(vehicle);
  edit(vehicle);
  CHECK(json_object_to_file(vehiclePath(path).c_str(), vehicle) == 0);
  json_object_put(vehicle);
  const auto result =
      pso_runtime::Optimizer(path, vehiclePath(path))
          .solve({}, {position, attitude}, actuators, {2, 1, 1});
  CHECK(result.position_error_m < 1e-10 && result.fitness < 1e-10);
}

int main(int argc, char **argv) {
  CHECK(argc == 2);
  const auto path = argv[1];
  Actuators zero{};
  pso_runtime::VehicleState state;

  // NED free fall: gravity increases down position; velocity is already in NED.
  state.position = {1, 2, -3};
  state.velocity = {4, 5, 6};
  checkPrediction(path, zero, state, {1.4, 2.5, -2.35});
  state = {};

  // Balanced 11 N thrust supports 1.1 kg at g=10; front/rear pitch moments
  // cancel.
  Actuators hover{3, 3, 5, 0, 0, 0, 0, 0, 0};
  checkPrediction(path, hover, state, {});
  // Three distinct coefficients must each produce the configured thrust.
  checkConfiguredPrediction(
      path, {1.5, 1, 1.25, 0, 0, 0, 0, 0, 0}, {}, {}, [](json_object *vehicle) {
        json_object_object_add(json_object_object_get(vehicle, "propulsion"),
                               "k_t_n_per_command",
                               json_tokener_parse("[2,3,4]"));
      });
  // Moving only the left motor's arm changes the predicted roll moment.
  checkConfiguredPrediction(
      path, hover, {}, {0.00375, 0, 0}, [](json_object *vehicle) {
        json_object_object_add(
            json_object_object_get(vehicle, "force_positions_body_m"),
            "motor_left", json_tokener_parse("[0.5,-0.5,0]"));
      });
  checkConfiguredPrediction(
      path, hover, {}, {0, 0, -0.0025}, [](json_object *vehicle) {
        json_object_object_add(json_object_object_get(vehicle, "propulsion"),
                               "reaction_torque_per_thrust_m",
                               json_tokener_parse("[0.2,-0.1,0.04]"));
      });
  // Reaction torque rotates with the tilted rotor axis, rather than staying on
  // Z.
  checkConfiguredPrediction(
      path, {2.2, 0, 0, pi / 2, 0, 0, 0, 0, 0}, {0.01, 0, 0.05},
      {0.0022, 0, 0.00275}, [](json_object *vehicle) {
        json_object_object_add(json_object_object_get(vehicle, "propulsion"),
                               "reaction_torque_per_thrust_m",
                               json_tokener_parse("[0.2,0,0]"));
      });
  state.orientation = {pi / 2, 0, 0};
  checkPrediction(path, hover, state, {0, 0.05, 0.05}, state.orientation);

  // Positive body pitch rotates upward thrust toward negative north.
  state.orientation = {0, pi / 6, 0};
  checkPrediction(path, hover, state,
                  {-0.025, 0, 0.05 * (1 - std::cos(pi / 6))},
                  state.orientation);

  // Fully tilted front rotors point forward; yaw=90 degrees sends that force
  // east.
  state.orientation = {0, 0, pi / 2};
  Actuators forward{3, 3, 0, pi / 2, pi / 2, 0, 0, 0, 0};
  checkPrediction(path, forward, state, {0, 0.03 / 1.1, 0.05},
                  state.orientation);
  state = {};

  // r cross F in FRD: left upward thrust gives positive roll, front gives
  // positive pitch.
  checkPrediction(path, {6, 0, 5, 0, 0, 0, 0, 0, 0}, state, {}, {0.0075, 0, 0});
  checkPrediction(path, {3, 3, 0, 0, 0, 0, 0, 0, 0}, state,
                  {0, 0, 0.05 - 0.03 / 1.1}, {0, 0.015, 0});
  checkPrediction(path, {2.2, 0, 0, pi / 2, 0, 0, 0, 0, 0}, state,
                  {0.01, 0, 0.05}, {0, 0, 0.00275});

  // Body yaw rate at roll=90 degrees changes Euler pitch, not Euler yaw.
  state.orientation = {pi / 2, 0, 0};
  state.angular_rate = {0, 0, 0.2};
  checkPrediction(path, zero, state, {0, 0, 0.05}, {pi / 2, -0.02, 0});
  state = {};

  // Wing lift is negative body Z; drag is negative X. Its forward arm pitches
  // up.
  state.velocity = {10, 0, 0};
  checkPrediction(
      path, zero, state, {1 - 0.25 / 1.1, 0, 0.05 - 0.25 / 1.1}, {0, 0.15, 0},
      R"("wing":{"area":1,"cl0":1,"cl_alpha":0,"cd0":1,"k":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})");

  // Equal V-tail commands cancel side force and yaw/roll, retaining upward lift
  // and aft pitch.
  state.velocity = {std::sqrt(2.0), 0, 0};
  const double cant = 0.785;
  checkPrediction(
      path, {0, 0, 0, 0, 0, 0.1, 0.1, 0, 0}, state,
      {0.1 * std::sqrt(2.0) - 0.001 / 1.1, 0,
       0.05 - 0.001 * std::cos(cant) / 1.1},
      {0, -0.0007 * std::cos(cant), 0},
      R"("wing":{"area":0},"tail":{"area":1,"cl_gamma":1,"cd_gamma":1,"cant_angle":0.785},"ruddervator":{"area":0},"aileron":{"area":0})");
  checkPrediction(
      path, {0, 0, 0, 0, 0, 0.1, 0.1, 0, 0}, state,
      {0.1 * std::sqrt(2.0) - 0.001 / 1.1, 0,
       0.05 - 0.001 * std::cos(cant) / 1.1},
      {0, -0.00075 * std::cos(cant), 0},
      R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":1,"cl":1,"cd":1,"cant_angle":0.785},"aileron":{"area":0})");

  // Same-sign aileron commands produce opposite lift increments and negative
  // roll.
  checkPrediction(
      path, {0, 0, 0, 0, 0, 0, 0, 0.1, 0.1}, state,
      {0.1 * std::sqrt(2.0) - 0.001 / 1.1, 0, 0.05}, {-0.0003, 0, 0},
      R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":1,"cl_beta":1,"cd_beta":1})");

  // Nonzero alpha changes wing lift/drag and their rotation, for both signs of
  // vertical airflow.
  for (double vz : {4.0, -4.0}) {
    state = {};
    state.velocity = {3, 0, vz};
    const double alpha = std::atan2(vz, 3.0);
    const double lift = 12.5 * alpha,
                 drag = 12.5; // rho=1, V=5, unit wing area/slopes.
    const double fx = -0.6 * drag + (vz / 5) * lift;
    const double fz = -(vz / 5) * drag - 0.6 * lift;
    checkPrediction(
        path, zero, state,
        {0.3 + 0.005 * fx / 1.1, 0, 0.1 * vz + 0.05 + 0.005 * fz / 1.1},
        {0, -0.003 * fz, 0},
        R"("wing":{"area":1,"cl0":0,"cl_alpha":1,"cd0":1,"k":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})");
  }

  // Pitch and yaw must transform NED velocity into body axes before alpha is
  // evaluated.
  state = {};
  state.velocity = {0, 10, 0};
  state.orientation = {0, 0.2, pi / 2};
  checkPrediction(
      path, zero, state, {0, 1, 0.05 - 0.05 / 1.1},
      {0, 0.2 + 0.03 * std::cos(0.2), pi / 2},
      R"("wing":{"area":1,"cl0":0,"cl_alpha":1,"cd0":0,"k":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})");

  // Backward motion checks atan2's quadrant and the drag direction, not
  // high-alpha validity.
  state = {};
  state.velocity = {-10, 0, 0};
  checkPrediction(
      path, zero, state, {-1 + 0.25 / 1.1, 0, 0.05 + 0.25 / 1.1}, {0, -0.15, 0},
      R"("wing":{"area":1,"cl0":1,"cl_alpha":0,"cd0":1,"k":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})");

  // Pure sideslip has alpha=0 by convention, but its speed still contributes to
  // Q.
  state.velocity = {0, 3, 0};
  checkPrediction(
      path, zero, state, {-0.0225 / 1.1, 0.3, 0.05 - 0.0225 / 1.1},
      {0, 0.0135, 0},
      R"("wing":{"area":1,"cl0":1,"cl_alpha":0,"cd0":1,"k":0},"tail":{"area":0},"ruddervator":{"area":0},"aileron":{"area":0})");

  // Ruddervator incidence acts even at neutral servos; opposite deflection
  // cancels it.
  state.velocity = {3, 0, 4};
  const double alpha = std::atan2(4.0, 3.0);
  const double tail_lift = 25 * alpha * std::cos(cant);
  const char *ruddervator_only =
      R"("wing":{"area":0},"tail":{"area":0},"ruddervator":{"area":1,"cl":1,"cd":1,"cant_angle":0.785},"aileron":{"area":0})";
  checkPrediction(
      path, zero, state,
      {0.3 - 0.125 * alpha / 1.1, 0, 0.45 - 0.005 * tail_lift / 1.1},
      {0, -0.00375 * tail_lift, 0}, ruddervator_only);
  checkPrediction(path, {0, 0, 0, 0, 0, -alpha, -alpha, 0, 0}, state,
                  {0.3, 0, 0.45}, {}, ruddervator_only);

  // Zero speed suppresses all aerodynamic terms, including deflected control
  // surfaces.
  state = {};
  checkPrediction(
      path, {0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.4}, state, {0, 0, 0.05}, {},
      R"("wing":{"area":1},"tail":{"area":1},"ruddervator":{"area":1},"aileron":{"area":1})");

  // The telemetry adapter must forward NED position/velocity and FRD
  // attitude/rates unchanged.
  configure(path, hover, {0, 0, -10}, {}, no_surfaces);
  DroneState telemetry;
  telemetry.local_x = 1;
  telemetry.local_y = 2;
  telemetry.local_z = -3;
  telemetry.velocity_x = 4;
  telemetry.velocity_y = 5;
  telemetry.velocity_z = -6;
  telemetry.roll = 0.1f;
  telemetry.pitch = -0.2f;
  telemetry.yaw = 0.3f;
  telemetry.gyro_x = 0.4f;
  telemetry.gyro_y = -0.5f;
  telemetry.gyro_z = 0.6f;
  state.position = {telemetry.local_x, telemetry.local_y, telemetry.local_z};
  state.velocity = {telemetry.velocity_x, telemetry.velocity_y,
                    telemetry.velocity_z};
  state.orientation = {telemetry.roll, telemetry.pitch, telemetry.yaw};
  state.angular_rate = {telemetry.gyro_x, telemetry.gyro_y, telemetry.gyro_z};
  telemetry.airspeed =
      999; // Optional measured airspeed cannot override the still-air model.
  telemetry.target = {{0, 0, -10}, {}};
  const auto expected = pso_runtime::Optimizer(path, vehiclePath(path))
                            .solve(state, telemetry.target, hover, {2, 1, 1});
  const auto actual =
      PSOInterface(path, {2, 1, 1}, vehiclePath(path)).runPSO(telemetry);
  CHECK(std::abs(actual.fitness - expected.fitness) < 1e-10);
  CHECK(std::abs(actual.position_error_m - expected.position_error_m) < 1e-10);
  // The same optimizer follows online targets even when the JSON target is
  // different.
  pso_runtime::Optimizer live_optimizer(path, vehiclePath(path));
  CHECK(live_optimizer.solve({}, {{0, 0, 0}, {}}, hover, {2, 1, 1})
            .position_error_m < 1e-10);
  CHECK(std::abs(live_optimizer.solve({}, {{0, 0, -2}, {}}, hover, {2, 1, 1})
                     .position_error_m -
                 2) < 1e-10);
  auto bad_target = telemetry.target;
  bad_target.orientation[0] = std::numeric_limits<double>::quiet_NaN();
  bool invalid_target = false;
  try {
    live_optimizer.solve({}, bad_target, hover, {2, 1, 1});
  } catch (const std::invalid_argument &) {
    invalid_target = true;
  }
  CHECK(invalid_target);
  state = {};
  state.velocity = {1001, 0, 0};
  bool invalid_speed = false;
  try {
    live_optimizer.solve(state, telemetry.target, hover, {2, 1, 1});
  } catch (const std::invalid_argument &) {
    invalid_speed = true;
  }
  CHECK(invalid_speed);
  std::filesystem::remove(path);
  std::filesystem::remove(vehiclePath(path));
}
