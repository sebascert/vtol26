#include "config_loader.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fstream>
#include <json-c/json.h>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

namespace {
using Json = json_object;
using Document = std::unique_ptr<Json, decltype(&json_object_put)>;
void check(bool valid, const std::string &message) {
  if (!valid)
    throw std::invalid_argument("Configuration: " + message);
}
Json *field(Json *object, const char *name) {
  Json *value = nullptr;
  check(json_object_is_type(object, json_type_object) &&
            json_object_object_get_ex(object, name, &value),
        std::string("missing field ") + name);
  return value;
}
void keys(Json *object, std::initializer_list<const char *> allowed) {
  check(json_object_is_type(object, json_type_object), "expected object");
  json_object_object_foreach(object, key, value) {
    (void)value;
    check(
        std::any_of(allowed.begin(), allowed.end(),
                    [&](const char *name) { return std::string(key) == name; }),
        std::string("unknown field ") + key);
  }
}
double number(Json *value) {
  check(json_object_is_type(value, json_type_int) ||
            json_object_is_type(value, json_type_double),
        "expected number");
  const double result = json_object_get_double(value);
  check(std::isfinite(result), "number must be finite");
  return result;
}
double scalar(Json *object, const char *name) {
  return number(field(object, name));
}
std::string string(Json *value) {
  check(json_object_is_type(value, json_type_string), "expected string");
  std::string result(json_object_get_string(value),
                     json_object_get_string_len(value));
  check(!result.empty(), "empty string");
  return result;
}
std::vector<double> vector(Json *value, size_t size) {
  check(json_object_is_type(value, json_type_array) &&
            json_object_array_length(value) == size,
        "wrong vector length");
  std::vector<double> result;
  for (size_t i = 0; i < size; ++i)
    result.push_back(number(json_object_array_get_idx(value, i)));
  return result;
}
std::array<double, 3> vector3(Json *value) {
  const auto v = vector(value, 3);
  return {v[0], v[1], v[2]};
}
Document read(const std::string &path) {
  std::ifstream input(path);
  check(input.is_open(), "cannot open " + path);
  input.seekg(0, std::ios::end);
  const auto size = input.tellg();
  check(size > 0 && size <= 1024 * 1024, "empty or oversized JSON in " + path);
  input.seekg(0);
  std::stringstream contents;
  contents << input.rdbuf();
  check(!input.bad(), "cannot read " + path);
  const auto text = contents.str();
  std::unique_ptr<json_tokener, decltype(&json_tokener_free)> tokener(
      json_tokener_new(), json_tokener_free);
  check(tokener != nullptr, "cannot allocate JSON parser");
  json_tokener_set_flags(tokener.get(),
                         JSON_TOKENER_STRICT | JSON_TOKENER_VALIDATE_UTF8);
  Document document(json_tokener_parse_ex(tokener.get(), text.c_str(),
                                          static_cast<int>(text.size())),
                    json_object_put);
  check(json_tokener_get_error(tokener.get()) == json_tokener_success &&
            document != nullptr,
        "invalid JSON in " + path);
  const auto end = json_tokener_get_parse_end(tokener.get());
  check(text.find_first_not_of(" \t\r\n", end) == std::string::npos,
        "trailing data in " + path);
  return document;
}
void optional(Json *object, const char *name, double &destination) {
  Json *value = nullptr;
  if (json_object_object_get_ex(object, name, &value))
    destination = number(value);
}
void optional(Json *object, const char *name, int &destination) {
  double value = destination;
  optional(object, name, value);
  check(value >= 0 && value <= std::numeric_limits<int>::max() &&
            value == std::floor(value),
        std::string("invalid integer ") + name);
  destination = static_cast<int>(value);
}
} // namespace

CargadorConfiguracion::Configuracion
CargadorConfiguracion::cargarVehiculo(const std::string &vehicle_path) {
  const auto validator = vehicleConfigDirectory() + "/validate_vehicle.py";
  const auto python = vehicleConfigDirectory() + "/venv/bin/python3";
  // Consume the validator's output so a file change cannot bypass validation.
  int descriptors[2];
  check(pipe(descriptors) == 0, "cannot create validator pipe");
  const pid_t child = fork();
  if (child == -1) {
    close(descriptors[0]);
    close(descriptors[1]);
    throw std::runtime_error("Configuration: cannot start vehicle validator");
  }
  if (child == 0) {
    close(descriptors[0]);
    if (dup2(descriptors[1], STDOUT_FILENO) == -1)
      _exit(127);
    close(descriptors[1]);
    execl(python.c_str(), python.c_str(), "-I", validator.c_str(),
          vehicle_path.c_str(), "--print-json", static_cast<char *>(nullptr));
    _exit(127);
  }
  close(descriptors[1]);
  std::string text;
  char buffer[4096];
  ssize_t count;
  bool read_ok = true;
  while ((count = ::read(descriptors[0], buffer, sizeof(buffer))) != 0) {
    if (count < 0) {
      if (errno == EINTR)
        continue;
      read_ok = false;
      break;
    }
    if (text.size() + count > 1024 * 1024) {
      read_ok = false;
      break;
    }
    text.append(buffer, count);
  }
  close(descriptors[0]);
  int status = 0;
  pid_t waited;
  do {
    waited = waitpid(child, &status, 0);
  } while (waited == -1 && errno == EINTR);
  check(read_ok && waited == child && WIFEXITED(status) &&
            WEXITSTATUS(status) == 0,
        "vehicle validation failed; install config/ with config/install.sh");
  Document vehicle(json_tokener_parse(text.c_str()), json_object_put);
  check(vehicle != nullptr, "vehicle validator returned invalid JSON");
  Configuracion c;
  auto *root = vehicle.get();
  auto *meta = field(root, "metadata");
  c.vehicle_id = string(field(meta, "vehicle_id"));
  c.vehicle_revision = string(field(meta, "revision"));
  c.calibration_status = string(field(meta, "calibration_status"));
  auto *body = field(root, "rigid_body");
  c.masa = scalar(body, "mass_kg");
  c.inercia = vector(field(body, "inertia_kg_m2"), 3);
  c.cg_model_m = vector3(field(body, "cg_model_m"));
  auto *env = field(root, "environment");
  c.rho = scalar(env, "air_density_kg_m3");
  c.g = scalar(env, "gravity_m_s2");
  auto *positions = field(root, "force_positions_body_m");
  size_t index = 0;
  for (const char *name :
       {"wing", "aileron_left", "aileron_right", "motor_left", "motor_right",
        "motor_rear", "cola_left", "cola_right", "ruddervator_left",
        "ruddervator_right"})
    c.force_positions[index++] = vector3(field(positions, name));
  auto *aero = field(root, "aerodynamics");
  auto *wing = field(aero, "wing");
  c.area_ala = scalar(wing, "area_m2");
  c.cl0_ala = scalar(wing, "cl0");
  c.cl_alpha_ala = scalar(wing, "cl_alpha_per_rad");
  c.cd0_ala = scalar(wing, "cd0");
  c.k_ala = scalar(wing, "induced_drag_factor");
  auto surface = [&](const char *name, double &area, double &cl, double &cd,
                     double *cant) {
    auto *object = field(aero, name);
    area = scalar(object, "area_per_side_m2");
    cl = scalar(object, "cl_per_rad");
    cd = scalar(object, "cd_per_rad");
    if (cant)
      *cant = scalar(object, "cant_angle_rad");
  };
  surface("aileron", c.area_aleron, c.cl_beta_aleron, c.cd_beta_aleron,
          nullptr);
  surface("cola", c.area_cola, c.cl_gamma_cola, c.cd_gamma_cola,
          &c.angulo_diedro_cola);
  surface("ruddervator", c.area_ruddervator, c.cl_ruddervator, c.cd_ruddervator,
          &c.angulo_diedro_ruddervator);
  auto *propulsion = field(root, "propulsion");
  c.coeficiente_empuje = vector3(field(propulsion, "k_t_n_per_command"));
  c.reaccion_motor = vector3(field(propulsion, "reaction_torque_per_thrust_m"));
  auto *actuators = field(root, "actuators");
  for (size_t i = 0; i < 9; ++i) {
    auto *actuator = json_object_array_get_idx(actuators, i);
    c.actuator_min.push_back(scalar(actuator, "min"));
    c.actuator_max.push_back(scalar(actuator, "max"));
    c.actuator_disarmed.push_back(scalar(actuator, "disarmed"));
  }
  return c;
}

CargadorConfiguracion::Configuracion
CargadorConfiguracion::cargarDesdeJson(const std::string &tuning_path,
                                       const std::string &vehicle_path) {
  auto c = cargarVehiculo(vehicle_path);
  auto tuning = read(tuning_path);
  keys(tuning.get(), {"cost_function", "pso", "actuators", "target_pose"});
  auto *cost = field(tuning.get(), "cost_function");
  keys(cost, {"omega_scale", "angle_scale", "k_w", "k_epsilon", "k_gamma",
              "k_beta", "k_dynamics", "k_pos", "k_att", "dt"});
  optional(cost, "omega_scale", c.escala_omega);
  optional(cost, "angle_scale", c.escala_angulo);
  optional(cost, "k_w", c.k_w);
  optional(cost, "k_epsilon", c.k_epsilon);
  optional(cost, "k_gamma", c.k_gamma);
  optional(cost, "k_beta", c.k_beta);
  optional(cost, "k_dynamics", c.k_dinamica);
  optional(cost, "k_pos", c.k_pos);
  optional(cost, "k_att", c.k_orientacion);
  optional(cost, "dt", c.dt);
  Json *swarm = nullptr;
  if (json_object_object_get_ex(tuning.get(), "pso", &swarm)) {
    keys(swarm, {"w_max", "w_min", "c1", "c2", "num_particles",
                 "max_iterations", "k_am", "external_cycles",
                 "k_am_contraction", "k_am_min", "convergence_threshold"});
    optional(swarm, "w_max", c.w_max);
    optional(swarm, "w_min", c.w_min);
    optional(swarm, "c1", c.c1);
    optional(swarm, "c2", c.c2);
    optional(swarm, "num_particles", c.num_particulas);
    optional(swarm, "max_iterations", c.max_iteraciones);
    optional(swarm, "k_am", c.k_am);
    optional(swarm, "external_cycles", c.ciclos_externos);
    optional(swarm, "k_am_contraction", c.contraccion_k_am);
    optional(swarm, "k_am_min", c.k_am_minimo);
    optional(swarm, "convergence_threshold", c.umbral_convergencia);
  }
  auto *initial = field(tuning.get(), "actuators");
  keys(initial, {"initial"});
  c.actuator_initial = vector(field(initial, "initial"), 9);
  Json *target = nullptr;
  if (json_object_object_get_ex(tuning.get(), "target_pose", &target)) {
    keys(target, {"position", "orientation"});
    c.posicion_objetivo = vector(field(target, "position"), 3);
    c.orientacion_objetivo = vector(field(target, "orientation"), 3);
    c.tiene_pose_objetivo = true;
  }
  return c;
}
