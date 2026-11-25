#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <array>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

inline std::string vehicleConfigDirectory() {
  const char *user_home = std::getenv("HOME");
  if (!user_home || !*user_home)
    throw std::invalid_argument(
        "HOME must identify the current user's home directory");
  return std::string(user_home) + "/.local/share/vtol26/config";
}

class CargadorConfiguracion {
public:
  struct Configuracion {
    double rho{}, g{}, masa{};
    std::vector<double> inercia;
    double area_ala{}, cl0_ala{}, cl_alpha_ala{}, cd0_ala{}, k_ala{};
    double area_cola{}, cl_gamma_cola{}, cd_gamma_cola{}, angulo_diedro_cola{};
    double area_ruddervator{}, cl_ruddervator{}, cd_ruddervator{},
        angulo_diedro_ruddervator{};
    double area_aleron{}, cl_beta_aleron{}, cd_beta_aleron{};
    std::array<double, 3> coeficiente_empuje{}, reaccion_motor{}, cg_model_m{};
    // Wing, ailerons L/R, motors L/R/rear, cola L/R, ruddervators L/R; all from
    // CG.
    std::array<std::array<double, 3>, 10> force_positions{};
    std::string vehicle_id, vehicle_revision, calibration_status;

    double escala_omega = 1000.0;
    double escala_angulo = 1.0;
    double k_w = 1.0;
    double k_epsilon = 1.0;
    double k_gamma = 1.0;
    double k_beta = 1.0;
    double k_dinamica = 1.0;
    double k_pos = 1.0;
    double k_orientacion = 1.0;
    double dt = 0.1;

    double w_max = 0.9;
    double w_min = 0.4;
    double c1 = 2.0;
    double c2 = 2.0;
    int num_particulas = 40;
    int max_iteraciones = 120;
    double k_am = 5.0;
    int ciclos_externos = 20;
    double contraccion_k_am = 0.6;
    double k_am_minimo = 0.02;
    double umbral_convergencia = 1e-4;

    std::vector<double> posicion_objetivo = {0.0, 0.0, 0.0};
    std::vector<double> orientacion_objetivo = {0.0, 0.0, 0.0};
    bool tiene_pose_objetivo = false;
    std::vector<double> actuator_min, actuator_max, actuator_initial,
        actuator_disarmed;
  };

  static Configuracion
  cargarVehiculo(const std::string &vehicle_path = vehicleConfigDirectory() +
                                                   "/vehicle.json");

  static Configuracion
  cargarDesdeJson(const std::string &tuning_path,
                  const std::string &vehicle_path = vehicleConfigDirectory() +
                                                    "/vehicle.json");
};

#endif
