#include "vehicle_model.hpp"
#include <cmath>
#include <stdexcept>

namespace pso_runtime {
namespace {
using Vector = std::array<double, 3>;
Vector sumar3(const Vector &a, const Vector &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vector escalar3(const Vector &a, double scale) {
  return {a[0] * scale, a[1] * scale, a[2] * scale};
}
Vector productoCruz3(const Vector &a, const Vector &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
Vector rotateY(const Vector &v, double angle) {
  const double c = std::cos(angle), s = std::sin(angle);
  return {c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]};
}
Vector rotateX(const Vector &v, double angle) {
  const double c = std::cos(angle), s = std::sin(angle);
  return {v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]};
}
} // namespace

void validateActuators(const CargadorConfiguracion::Configuracion &config,
                       const std::array<double, 9> &command) {
  if (config.actuator_min.size() != 9 || config.actuator_max.size() != 9)
    throw std::invalid_argument("Expected nine configured actuator limits");
  for (size_t i = 0; i < command.size(); ++i)
    if (!std::isfinite(command[i]) || command[i] < config.actuator_min[i] ||
        command[i] > config.actuator_max[i])
      throw std::invalid_argument("Actuator command outside configured limits");
}

BodyWrench
bodyWrench(const CargadorConfiguracion::Configuracion &parametrosFisicos,
           const std::array<double, 3> &velocidadCuerpo,
           const std::array<double, 9> &particula) {
  validateActuators(parametrosFisicos, particula);
  for (double value : velocidadCuerpo)
    if (!std::isfinite(value))
      throw std::invalid_argument("Nonfinite body velocity");
  BodyWrench resultado;
  const double alpha = (velocidadCuerpo[0] == 0.0 && velocidadCuerpo[2] == 0.0)
                           ? 0.0
                           : std::atan2(velocidadCuerpo[2], velocidadCuerpo[0]);
  const double V =
      std::hypot(velocidadCuerpo[0], velocidadCuerpo[1], velocidadCuerpo[2]);
  const double q = 0.5 * parametrosFisicos.rho * V * V;

  const double omega_I = particula[IND_OMEGA_I];
  const double omega_D = particula[IND_OMEGA_D];
  const double omega_T = particula[IND_OMEGA_T];
  const double epsilon_I = particula[IND_EPSILON_I];
  const double epsilon_D = particula[IND_EPSILON_D];
  const double gamma_I = particula[IND_GAMMA_I];
  const double gamma_D = particula[IND_GAMMA_D];
  const double beta_I = particula[IND_BETA_I];
  const double beta_D = particula[IND_BETA_D];

  const double CL =
      parametrosFisicos.cl0_ala + parametrosFisicos.cl_alpha_ala * alpha;
  const double CD =
      parametrosFisicos.cd0_ala + parametrosFisicos.k_ala * CL * CL;
  const double D_ala = -CD * q * parametrosFisicos.area_ala;
  const double L_ala = CL * q * parametrosFisicos.area_ala;
  const auto F_ala = rotateY({D_ala, 0.0, -L_ala}, -alpha);

  const double L_aleron_izq = -q * parametrosFisicos.cl_beta_aleron * beta_I *
                              parametrosFisicos.area_aleron;
  const double D_aleron_izq = -q * parametrosFisicos.cd_beta_aleron *
                              std::abs(beta_I) * parametrosFisicos.area_aleron;
  const double L_aleron_der = q * parametrosFisicos.cl_beta_aleron * beta_D *
                              parametrosFisicos.area_aleron;
  const double D_aleron_der = -q * parametrosFisicos.cd_beta_aleron *
                              std::abs(beta_D) * parametrosFisicos.area_aleron;
  const auto F_aleron_izq = rotateY({D_aleron_izq, 0.0, -L_aleron_izq}, -alpha);
  const auto F_aleron_der = rotateY({D_aleron_der, 0.0, -L_aleron_der}, -alpha);

  const double empuje_I = parametrosFisicos.coeficiente_empuje[0] * omega_I;
  const double empuje_D = parametrosFisicos.coeficiente_empuje[1] * omega_D;
  const double empuje_T = parametrosFisicos.coeficiente_empuje[2] * omega_T;
  const auto F_motor_izq = rotateY({0.0, 0.0, -empuje_I}, -epsilon_I);
  const auto F_motor_der = rotateY({0.0, 0.0, -empuje_D}, -epsilon_D);
  const auto F_motor_trasero = std::array<double, 3>{0.0, 0.0, -empuje_T};

  const double D_cola_izq = -q * parametrosFisicos.cd_gamma_cola *
                            std::abs(gamma_I) * parametrosFisicos.area_cola;
  const double L_cola_izq = q * parametrosFisicos.cl_gamma_cola * gamma_I *
                            parametrosFisicos.area_cola;
  const double D_cola_der = -q * parametrosFisicos.cd_gamma_cola *
                            std::abs(gamma_D) * parametrosFisicos.area_cola;
  const double L_cola_der = q * parametrosFisicos.cl_gamma_cola * gamma_D *
                            parametrosFisicos.area_cola;
  const auto F_cola_izq = rotateX({D_cola_izq, 0.0, -L_cola_izq},
                                  parametrosFisicos.angulo_diedro_cola);
  const auto F_cola_der = rotateX({D_cola_der, 0.0, -L_cola_der},
                                  -parametrosFisicos.angulo_diedro_cola);

  const double efectivo_ruddervator_izq = alpha + gamma_I;
  const double efectivo_ruddervator_der = alpha + gamma_D;
  const double D_ruddervator_izq = -q * parametrosFisicos.cd_ruddervator *
                                   std::abs(efectivo_ruddervator_izq) *
                                   parametrosFisicos.area_ruddervator;
  const double L_ruddervator_izq = q * parametrosFisicos.cl_ruddervator *
                                   efectivo_ruddervator_izq *
                                   parametrosFisicos.area_ruddervator;
  const double D_ruddervator_der = -q * parametrosFisicos.cd_ruddervator *
                                   std::abs(efectivo_ruddervator_der) *
                                   parametrosFisicos.area_ruddervator;
  const double L_ruddervator_der = q * parametrosFisicos.cl_ruddervator *
                                   efectivo_ruddervator_der *
                                   parametrosFisicos.area_ruddervator;
  const auto F_ruddervator_izq =
      rotateX({D_ruddervator_izq, 0.0, -L_ruddervator_izq},
              parametrosFisicos.angulo_diedro_ruddervator);
  const auto F_ruddervator_der =
      rotateX({D_ruddervator_der, 0.0, -L_ruddervator_der},
              -parametrosFisicos.angulo_diedro_ruddervator);

  resultado.force = F_ala;
  resultado.force = sumar3(resultado.force, F_aleron_izq);
  resultado.force = sumar3(resultado.force, F_aleron_der);
  resultado.force = sumar3(resultado.force, F_motor_izq);
  resultado.force = sumar3(resultado.force, F_motor_der);
  resultado.force = sumar3(resultado.force, F_motor_trasero);
  resultado.force = sumar3(resultado.force, F_cola_izq);
  resultado.force = sumar3(resultado.force, F_cola_der);
  resultado.force = sumar3(resultado.force, F_ruddervator_izq);
  resultado.force = sumar3(resultado.force, F_ruddervator_der);

  const auto &r_ala = parametrosFisicos.force_positions[0];
  const auto &r_aleron_izq = parametrosFisicos.force_positions[1];
  const auto &r_aleron_der = parametrosFisicos.force_positions[2];
  const auto &r_motor_izq = parametrosFisicos.force_positions[3];
  const auto &r_motor_der = parametrosFisicos.force_positions[4];
  const auto &r_motor_trasero = parametrosFisicos.force_positions[5];
  const auto &r_cola_izq = parametrosFisicos.force_positions[6];
  const auto &r_cola_der = parametrosFisicos.force_positions[7];
  const auto &r_ruddervator_izq = parametrosFisicos.force_positions[8];
  const auto &r_ruddervator_der = parametrosFisicos.force_positions[9];

  resultado.moment = productoCruz3(r_ala, F_ala);
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_aleron_izq, F_aleron_izq));
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_aleron_der, F_aleron_der));
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_motor_izq, F_motor_izq));
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_motor_der, F_motor_der));
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_motor_trasero, F_motor_trasero));
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_cola_izq, F_cola_izq));
  resultado.moment =
      sumar3(resultado.moment, productoCruz3(r_cola_der, F_cola_der));
  resultado.moment = sumar3(
      resultado.moment, productoCruz3(r_ruddervator_izq, F_ruddervator_izq));
  resultado.moment = sumar3(
      resultado.moment, productoCruz3(r_ruddervator_der, F_ruddervator_der));

  // Signed reaction torque follows each tilted motor's thrust axis.
  resultado.moment =
      sumar3(resultado.moment,
             escalar3(F_motor_izq, parametrosFisicos.reaccion_motor[0]));
  resultado.moment =
      sumar3(resultado.moment,
             escalar3(F_motor_der, parametrosFisicos.reaccion_motor[1]));
  resultado.moment =
      sumar3(resultado.moment,
             escalar3(F_motor_trasero, parametrosFisicos.reaccion_motor[2]));
  for (const auto &vector : {resultado.force, resultado.moment})
    for (double value : vector)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite vehicle wrench");
  return resultado;
}
} // namespace pso_runtime
