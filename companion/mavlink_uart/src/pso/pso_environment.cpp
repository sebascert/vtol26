 #include "pso_state.hpp"
#include "config_loader.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <string>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <utility>

namespace {

std::array<double, 3> add3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

std::array<double, 3> sub3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

std::array<double, 3> scale3(const std::array<double, 3>& v, double s) {
    return {v[0] * s, v[1] * s, v[2] * s};
}

std::array<double, 3> cross3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

std::array<double, 4> quatFromAxisAngleY(double angle) {
    double half = angle * 0.5;
    return {std::cos(half), 0.0, std::sin(half), 0.0};
}

std::array<double, 4> quatFromAxisAngleX(double angle) {
    double half = angle * 0.5;
    return {std::cos(half), std::sin(half), 0.0, 0.0};
}

std::array<double, 4> quatFromEuler(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    return {
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr
    };
}

std::array<double, 4> quatConjugate(const std::array<double, 4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

std::array<double, 4> quatMultiply(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    };
}

std::array<double, 3> rotateVectorByQuat(const std::array<double, 3>& v, const std::array<double, 4>& q) {
    std::array<double, 4> vecQuat = {0.0, v[0], v[1], v[2]};
    std::array<double, 4> qConj = quatConjugate(q);
    std::array<double, 4> rotated = quatMultiply(quatMultiply(q, vecQuat), qConj);
    return {rotated[1], rotated[2], rotated[3]};
}

double quatDot(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

double quatAngularDistance(const std::array<double, 4>& qa, const std::array<double, 4>& qb) {
    double d = std::abs(quatDot(qa, qb));
    d = std::clamp(d, -1.0, 1.0);
    return 2.0 * std::acos(d);
}

double sanitizeScalar(double v, double limit = 1e6) {
    if (!std::isfinite(v)) {
        return 0.0;
    }
    return std::clamp(v, -limit, limit);
}

std::array<double, 3> sanitizeVec3(const std::array<double, 3>& v, double limit = 1e6) {
    return {
        sanitizeScalar(v[0], limit),
        sanitizeScalar(v[1], limit),
        sanitizeScalar(v[2], limit)
    };
}

struct PhysicsParameters {
    double rho = 1.225;
    double g = 9.81;
    double mass = 12.0;
    std::array<double, 3> inertia = {0.25, 0.28, 0.35};

    double area_wing = 0.28;
    double cl0 = 0.28;
    double cl_alpha = 4.6;
    double cd0 = 0.028;
    double wing_k = 0.06;

    double area_tail = 0.05;
    double cl_gamma = 0.52;
    double cd_gamma = 0.016;
    double tail_cant_angle = 0.785;

    double area_ruddervator = 0.02;
    double cl_rud = 0.28;
    double cd_rud = 0.012;
    double ruddervator_cant_angle = 0.785;

    double area_aileron = 0.03;
    double cl_beta = 0.32;
    double cd_beta = 0.015;

    double thrust_coefficient = 2.4e-4;
    double reference_airspeed = 20.0;

    // Parámetros de la función de costo
    double OMEGA_SCALE = 1000.0;
    double ANGLE_SCALE = 1.0;
    double k_w = 1.0;
    double k_epsilon = 1.0;
    double k_gamma = 1.0;
    double k_beta = 1.0;
    double k_dynamics = 1.0;
    double k_pos = 1.0;
    double k_att = 1.0;
    double dt = 0.1;

    // Parámetros PSO
    double w_max = 0.9;
    double w_min = 0.4;
    double c1 = 2.0;
    double c2 = 2.0;

    static PhysicsParameters fromConfig(const ConfigLoader::Config& cfg) {
        PhysicsParameters p;
        p.rho = cfg.rho;
        p.g = cfg.g;
        p.mass = cfg.mass;
        if (cfg.inertia.size() == 3) {
            p.inertia[0] = cfg.inertia[0];
            p.inertia[1] = cfg.inertia[1];
            p.inertia[2] = cfg.inertia[2];
        }
        p.area_wing = cfg.wing_area;
        p.cl0 = cfg.wing_cl0;
        p.cl_alpha = cfg.wing_cl_alpha;
        p.cd0 = cfg.wing_cd0;
        p.wing_k = cfg.wing_k;
        p.area_tail = cfg.tail_area;
        p.cl_gamma = cfg.tail_cl_gamma;
        p.cd_gamma = cfg.tail_cd_gamma;
        p.tail_cant_angle = cfg.tail_cant_angle;
        p.area_ruddervator = cfg.rud_area;
        p.cl_rud = cfg.rud_cl;
        p.cd_rud = cfg.rud_cd;
        p.ruddervator_cant_angle = cfg.rud_cant_angle;
        p.area_aileron = cfg.ail_area;
        p.cl_beta = cfg.ail_cl_beta;
        p.cd_beta = cfg.ail_cd_beta;
        p.thrust_coefficient = cfg.thrust_coeff;
        p.reference_airspeed = cfg.ref_airspeed;
        p.OMEGA_SCALE = cfg.omega_scale;
        p.ANGLE_SCALE = cfg.angle_scale;
        p.k_w = cfg.k_w;
        p.k_epsilon = cfg.k_epsilon;
        p.k_gamma = cfg.k_gamma;
        p.k_beta = cfg.k_beta;
        p.k_dynamics = cfg.k_dynamics;
        p.k_pos = cfg.k_pos;
        p.k_att = cfg.k_att;
        p.dt = cfg.dt;
        p.w_max = cfg.w_max;
        p.w_min = cfg.w_min;
        p.c1 = cfg.c1;
        p.c2 = cfg.c2;
        return p;
    }
};

struct ForceMoment {
    std::array<double, 3> force = {0.0, 0.0, 0.0};
    std::array<double, 3> moment = {0.0, 0.0, 0.0};
};

} // namespace

/**
 * @file pso_environment.cpp
 * @brief Entorno de pruebas para desarrollar el algoritmo PSO
 * 
 * Este archivo implementa:
 * - Inicialización del vector de estado D
 * - Cálculo de áreas de muestreo basadas en K_AM
 * - Generación de partículas iniciales para PSO
 */

/**
 * @class PSO_Environment
 * @brief Entorno de pruebas para el algoritmo PSO
 */
class EntornoPSO {
private:
    Estado estadoActual;

    std::array<double, 3> targetPosition;
    std::array<double, 3> targetOrientation;
    bool targetPoseEnabled;
    std::array<double, 3> posicionActual;
    std::array<double, 3> orientacionActual;
    std::array<double, 3> velocidadActual;
    std::array<double, 3> velocidadAngularActual;
    PhysicsParameters physicsParams;
    std::mt19937 generadorAleatorio;

    struct Postura {
        std::array<double, 3> posicion;
        std::array<double, 3> orientacion;
    };


public:
    EntornoPSO()
        : targetPosition{0.0, 0.0, 0.0},
          targetOrientation{0.0, 0.0, 0.0},
          targetPoseEnabled(false),
          posicionActual{0.0, 0.0, 0.0},
          orientacionActual{0.0, 0.0, 0.0},
          velocidadActual{0.0, 0.0, 0.0},
          velocidadAngularActual{0.0, 0.0, 0.0},
          generadorAleatorio(std::random_device{}()) {
        estadoActual.inicializar();
    }

    // Permite inyectar parámetros físicos desde config.json
    void establecerParametrosFisicos(const PhysicsParameters& p) {
        physicsParams = p;
    }

    void establecerTargetPose(const std::array<double, 3>& posicionObjetivo,
                              const std::array<double, 3>& orientacionObjetivo) {
        targetPosition = posicionObjetivo;
        targetOrientation = orientacionObjetivo;
        targetPoseEnabled = true;
    }

    bool usaTargetPose() const {
        return targetPoseEnabled;
    }

    const std::array<double, 3>& obtenerTargetPosicion() const {
        return targetPosition;
    }

    const std::array<double, 3>& obtenerTargetOrientacion() const {
        return targetOrientation;
    }

    void establecerRadioMuestreo(double radio) {
        estadoActual.establecerRadioMuestreo(radio);
    }

    double obtenerRadioMuestreo() const {
        return estadoActual.obtenerRadioMuestreo();
    }

    /**
     * Establece el estado actual del dron
     */
    void establecerEstadoActual(const std::array<double, DIMENSION_ESTADO>& estado) {
        estadoActual.D = estado;
    }

    /**
     * Obtiene el estado actual
     */
    const Estado& obtenerEstadoActual() const {
        return estadoActual;
    }

    /**
     * Imprime el estado actual y sus rangos de muestreo
     */
    void imprimirEstadoYRangosMuestreo() const {
        std::cout << "\n=== Estado Actual (D) ===" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        
        std::cout << "\nRotores (velocidades angulares):" << std::endl;
        std::cout << "  ω_I (omega izquierdo):      " << estadoActual.D[IND_OMEGA_I] << std::endl;
        std::cout << "  ω_D (omega derecho):        " << estadoActual.D[IND_OMEGA_D] << std::endl;
        std::cout << "  ω_T (omega trasero):        " << estadoActual.D[IND_OMEGA_T] << std::endl;
        
        std::cout << "\nRotores basculantes delanteros (ángulos):" << std::endl;
        std::cout << "  ε_I (epsilon izquierdo):    " << estadoActual.D[IND_EPSILON_I] << std::endl;
        std::cout << "  ε_D (epsilon derecho):      " << estadoActual.D[IND_EPSILON_D] << std::endl;
        
        std::cout << "\nRuddervators (ángulos):" << std::endl;
        std::cout << "  γ_I (gamma izquierdo):      " << estadoActual.D[IND_GAMMA_I] << std::endl;
        std::cout << "  γ_D (gamma derecho):        " << estadoActual.D[IND_GAMMA_D] << std::endl;
        
        std::cout << "\nAlerones (ángulos):" << std::endl;
        std::cout << "  β_I (beta izquierdo):       " << estadoActual.D[IND_BETA_I] << std::endl;
        std::cout << "  β_D (beta derecho):         " << estadoActual.D[IND_BETA_D] << std::endl;
        
        std::cout << "\n=== Rangos de Muestreo (K_AM = " << estadoActual.obtenerRadioMuestreo() << ") ===" << std::endl;
        
        const char* labels[] = {
            "ω_I", "ω_D", "ω_T", 
            "ε_I", "ε_D", 
            "γ_I", "γ_D", 
            "β_I", "β_D"
        };
        
        for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
            auto [min_val, max_val] = estadoActual.obtenerRangoMuestreo(i);
            std::cout << "  " << labels[i] << ": [" << min_val << ", " << max_val << "]" << std::endl;
        }
    }

    /**
     * Genera un conjunto inicial de partículas dentro de los rangos de muestreo
     * @param numParticles Número de partículas a generar
     * @return Vector de partículas (estado inicial)
     */
    std::vector<std::array<double, DIMENSION_ESTADO>> generarParticulasIniciales(size_t numParticulas) {
        std::vector<std::array<double, DIMENSION_ESTADO>> particles;
        particles.reserve(numParticulas);
        
        for (size_t p = 0; p < numParticulas; ++p) {
            std::array<double, DIMENSION_ESTADO> particle;
            
            for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
                auto [min_val, max_val] = estadoActual.obtenerRangoMuestreo(i);
                std::uniform_real_distribution<double> distribution(min_val, max_val);
                particle[i] = distribution(generadorAleatorio);
            }
            
            particles.push_back(particle);
        }
        
        return particles;
    }

    /**
     * Predice la posición y orientación resultante R para una partícula
     * usando una integración física simple basada en fuerza, momento, masa e inercia.
     */
    ForceMoment computeForceMoment(const std::array<double, DIMENSION_ESTADO>& p) const {
        ForceMoment fm;
        const double alpha = 0.0; // Ángulo de ataque del ala fijo para este modelo inicial.
        const double V = physicsParams.reference_airspeed;
        const double q = 0.5 * physicsParams.rho * V * V;

        const double omega_I = p[IND_OMEGA_I];
        const double omega_D = p[IND_OMEGA_D];
        const double omega_T = p[IND_OMEGA_T];
        const double epsilon_I = p[IND_EPSILON_I];
        const double epsilon_D = p[IND_EPSILON_D];
        const double gamma_I = p[IND_GAMMA_I];
        const double gamma_D = p[IND_GAMMA_D];
        const double beta_I = p[IND_BETA_I];
        const double beta_D = p[IND_BETA_D];

        // Ala principal
        const double CL = physicsParams.cl0 + physicsParams.cl_alpha * alpha;
        const double CD = physicsParams.cd0 + physicsParams.wing_k * CL * CL;
        const double D_wing = -CD * q * physicsParams.area_wing;
        const double L_wing = CL * q * physicsParams.area_wing;
        const auto F_wing = rotateVectorByQuat({0.0, D_wing, L_wing}, quatFromAxisAngleY(-alpha));

        // Alerones
        const double L_ail_left = -q * physicsParams.cl_beta * beta_I * physicsParams.area_aileron;
        const double D_ail_left = -q * physicsParams.cd_beta * std::abs(beta_I) * physicsParams.area_aileron;
        const double L_ail_right = q * physicsParams.cl_beta * beta_D * physicsParams.area_aileron;
        const double D_ail_right = -q * physicsParams.cd_beta * std::abs(beta_D) * physicsParams.area_aileron;
        const auto F_ail_left = rotateVectorByQuat({0.0, D_ail_left, L_ail_left}, quatFromAxisAngleY(-alpha));
        const auto F_ail_right = rotateVectorByQuat({0.0, D_ail_right, L_ail_right}, quatFromAxisAngleY(-alpha));

        // Motores delanteros y trasero
        const double thrust_I = physicsParams.thrust_coefficient * omega_I;
        const double thrust_D = physicsParams.thrust_coefficient * omega_D;
        const double thrust_T = physicsParams.thrust_coefficient * omega_T;
        const auto F_motor_left = rotateVectorByQuat({0.0, 0.0, thrust_I}, quatFromAxisAngleY(epsilon_I));
        const auto F_motor_right = rotateVectorByQuat({0.0, 0.0, thrust_D}, quatFromAxisAngleY(epsilon_D));
        const auto F_motor_rear = std::array<double, 3>{0.0, 0.0, thrust_T};

        // Cola izquierda y derecha
        const double D_tail_left = -q * physicsParams.cd_gamma * std::abs(gamma_I) * physicsParams.area_tail;
        const double L_tail_left = q * physicsParams.cl_gamma * gamma_I * physicsParams.area_tail;
        const double D_tail_right = -q * physicsParams.cd_gamma * std::abs(gamma_D) * physicsParams.area_tail;
        const double L_tail_right = q * physicsParams.cl_gamma * gamma_D * physicsParams.area_tail;
        const auto F_tail_left = rotateVectorByQuat({0.0, D_tail_left, L_tail_left}, quatFromAxisAngleX(-physicsParams.tail_cant_angle));
        const auto F_tail_right = rotateVectorByQuat({0.0, D_tail_right, L_tail_right}, quatFromAxisAngleX(physicsParams.tail_cant_angle));

        // Ruddervators izquierdo y derecho
        const double eff_rud_left = gamma_I;
        const double eff_rud_right = gamma_D;
        const double D_rud_left = -q * physicsParams.cd_rud * std::abs(eff_rud_left) * physicsParams.area_ruddervator;
        const double L_rud_left = q * physicsParams.cl_rud * eff_rud_left * physicsParams.area_ruddervator;
        const double D_rud_right = -q * physicsParams.cd_rud * std::abs(eff_rud_right) * physicsParams.area_ruddervator;
        const double L_rud_right = q * physicsParams.cl_rud * eff_rud_right * physicsParams.area_ruddervator;
        const auto F_rud_left = rotateVectorByQuat({0.0, D_rud_left, L_rud_left}, quatFromAxisAngleX(-physicsParams.ruddervator_cant_angle));
        const auto F_rud_right = rotateVectorByQuat({0.0, D_rud_right, L_rud_right}, quatFromAxisAngleX(physicsParams.ruddervator_cant_angle));

        // Fuerza total en marco cuerpo
        fm.force = F_wing;
        fm.force = add3(fm.force, F_ail_left);
        fm.force = add3(fm.force, F_ail_right);
        fm.force = add3(fm.force, F_motor_left);
        fm.force = add3(fm.force, F_motor_right);
        fm.force = add3(fm.force, F_motor_rear);
        fm.force = add3(fm.force, F_tail_left);
        fm.force = add3(fm.force, F_tail_right);
        fm.force = add3(fm.force, F_rud_left);
        fm.force = add3(fm.force, F_rud_right);

        // Brazos de palanca respecto al CG (aproximados)
        const std::array<double, 3> r_wing = {0.6, 0.0, 0.0};
        const std::array<double, 3> r_ail_left = {0.7, -0.3, 0.0};
        const std::array<double, 3> r_ail_right = {0.7, 0.3, 0.0};
        const std::array<double, 3> r_motor_left = {0.5, -0.25, 0.0};
        const std::array<double, 3> r_motor_right = {0.5, 0.25, 0.0};
        const std::array<double, 3> r_motor_rear = {-0.6, 0.0, 0.0};
        const std::array<double, 3> r_tail_left = {-0.7, -0.25, 0.0};
        const std::array<double, 3> r_tail_right = {-0.7, 0.25, 0.0};
        const std::array<double, 3> r_rud_left = {-0.75, -0.20, 0.0};
        const std::array<double, 3> r_rud_right = {-0.75, 0.20, 0.0};

        fm.moment = cross3(r_wing, F_wing);
        fm.moment = add3(fm.moment, cross3(r_ail_left, F_ail_left));
        fm.moment = add3(fm.moment, cross3(r_ail_right, F_ail_right));
        fm.moment = add3(fm.moment, cross3(r_motor_left, F_motor_left));
        fm.moment = add3(fm.moment, cross3(r_motor_right, F_motor_right));
        fm.moment = add3(fm.moment, cross3(r_motor_rear, F_motor_rear));
        fm.moment = add3(fm.moment, cross3(r_tail_left, F_tail_left));
        fm.moment = add3(fm.moment, cross3(r_tail_right, F_tail_right));
        fm.moment = add3(fm.moment, cross3(r_rud_left, F_rud_left));
        fm.moment = add3(fm.moment, cross3(r_rud_right, F_rud_right));

        return fm;
    }

    Postura predictResultPose(const std::array<double, DIMENSION_ESTADO>& particle) const {
        ForceMoment fm = computeForceMoment(particle);
        auto bodyToInertial = quatFromEuler(orientacionActual[0], orientacionActual[1], orientacionActual[2]);
        auto F_inertial = rotateVectorByQuat(fm.force, bodyToInertial);
        const std::array<double, 3> gravity = {0.0, 0.0, -physicsParams.g};
        auto accel = add3(scale3(F_inertial, 1.0 / physicsParams.mass), gravity);

        const double dt = physicsParams.dt;
        Postura result;
        result.posicion = add3(posicionActual, add3(scale3(velocidadActual, dt), scale3(accel, 0.5 * dt * dt)));

        const std::array<double, 3> omega = velocidadAngularActual;
        std::array<double, 3> inertiaForce = {
            physicsParams.inertia[0] * omega[0],
            physicsParams.inertia[1] * omega[1],
            physicsParams.inertia[2] * omega[2]
        };
        auto omegaCross = cross3(omega, inertiaForce);
        std::array<double, 3> momentBody = sub3(fm.moment, omegaCross);
        std::array<double, 3> omegaDot = {
            momentBody[0] / physicsParams.inertia[0],
            momentBody[1] / physicsParams.inertia[1],
            momentBody[2] / physicsParams.inertia[2]
        };

        result.orientacion = add3(orientacionActual, scale3(omegaDot, dt));
        return result;
    }

    std::pair<std::array<double, 3>, std::array<double, 3>> obtenerPosePredicha(
        const std::array<double, DIMENSION_ESTADO>& particle) const {
        const Postura pose = predictResultPose(particle);
        return {sanitizeVec3(pose.posicion), sanitizeVec3(pose.orientacion)};
    }

    /**
     * Calcula el costo de posición/orientación para una partícula
     */
    double calcularCostoPosicionOrientacion(const std::array<double, DIMENSION_ESTADO>& particle) const {
        ForceMoment fm = computeForceMoment(particle);
        Postura posePredicha = predictResultPose(particle);
        std::array<double, 3> predictedPosition = sanitizeVec3(posePredicha.posicion);
        std::array<double, 3> predictedOrientation = sanitizeVec3(posePredicha.orientacion);

        double dx = predictedPosition[0] - posicionActual[0];
        double dy = predictedPosition[1] - posicionActual[1];
        double dz = predictedPosition[2] - posicionActual[2];

        double dm0 = sanitizeScalar(fm.moment[0]);
        double dm1 = sanitizeScalar(fm.moment[1]);
        double dm2 = sanitizeScalar(fm.moment[2]);

        double delta_mag_squared = dx*dx + dy*dy + dz*dz + dm0*dm0 + dm1*dm1 + dm2*dm2;

        double pose_cost = 0.0;
        if (targetPoseEnabled) {
            double ex = predictedPosition[0] - targetPosition[0];
            double ey = predictedPosition[1] - targetPosition[1];
            double ez = predictedPosition[2] - targetPosition[2];
            double pos_err_sq = ex * ex + ey * ey + ez * ez;

            auto q_pred = quatFromEuler(predictedOrientation[0], predictedOrientation[1], predictedOrientation[2]);
            auto q_target = quatFromEuler(targetOrientation[0], targetOrientation[1], targetOrientation[2]);
            double att_err_angle = quatAngularDistance(q_pred, q_target);
            double att_err_sq = att_err_angle * att_err_angle;

            pose_cost = physicsParams.k_pos * pos_err_sq + physicsParams.k_att * att_err_sq;
        }

        const auto sq = [](double x) { return x * x; };
        // Removed actuator-target penalties: optimize solely for predicted pose and dynamics
        double cost = physicsParams.k_dynamics * delta_mag_squared + pose_cost;
        return cost;
    }

    /**
     * Imprime las partículas generadas
     */
    void imprimirParticulas(const std::vector<std::array<double, DIMENSION_ESTADO>>& particles) const {
        std::cout << "\n=== Partículas Iniciales (PSO) ===" << std::endl;
        std::cout << "Total de partículas: " << particles.size() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        
        const char* labels[] = {
            "ω_I", "ω_D", "ω_T", 
            "ε_I", "ε_D", 
            "γ_I", "γ_D", 
            "β_I", "β_D"
        };
        
        for (size_t p = 0; p < particles.size() && p < 5; ++p) {  // Mostrar solo las primeras 5
            std::cout << "\nPartícula " << p << ":" << std::endl;
            for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
                std::cout << "  " << labels[i] << ": " << particles[p][i] << std::endl;
            }
            // Calcular y mostrar el valor de la función de costo f para esta partícula
            double f_val = calcularCostoPosicionOrientacion(particles[p]);
            std::cout << "  f: " << std::fixed << std::setprecision(6) << f_val << std::endl;
        }
        
        if (particles.size() > 5) {
            std::cout << "\n... (" << (particles.size() - 5) << " partículas más)" << std::endl;
        }
    }

    /**
     * Actualiza velocidades y posiciones de las partículas (PSO clásico)
     * - particles, velocities, pbest_positions, pbest_costs, gbest_position y gbest_cost
     *   deben estar inicializados y tener tamaños consistentes.
     * - iter: iteración actual (0..max_iters), max_iters: número máximo de iteraciones
     *
     * Ecuaciones usadas:
     * v(t+1) = w*v(t) + c1*r1*(pBest - x(t)) + c2*r2*(gBest - x(t))
     * x(t+1) = x(t) + v(t+1)
     * w decrece linealmente de 0.9 a 0.4 sobre max_iters.
     */
    void actualizar_enjambre(std::vector<std::array<double, DIMENSION_ESTADO>>& particles,
                             std::vector<std::array<double, DIMENSION_ESTADO>>& velocities,
                             std::vector<std::array<double, DIMENSION_ESTADO>>& pbest_positions,
                             std::vector<double>& pbest_costs,
                             std::array<double, DIMENSION_ESTADO>& gbest_position,
                             double& gbest_cost,
                             int iter, int max_iters) {
        if (particles.size() != velocities.size() || particles.size() != pbest_positions.size() || particles.size() != pbest_costs.size()) {
            throw std::runtime_error("Tamaños de vectores de partículas no coinciden");
        }

        double w = physicsParams.w_max;
        if (max_iters > 0) {
            int t = std::min(std::max(iter, 0), max_iters);
            w = physicsParams.w_max - (physicsParams.w_max - physicsParams.w_min) * (static_cast<double>(t) / static_cast<double>(max_iters));
        }

        std::uniform_real_distribution<double> dist01(0.0, 1.0);

        for (size_t i = 0; i < particles.size(); ++i) {
            for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
                double r1 = dist01(generadorAleatorio);
                double r2 = dist01(generadorAleatorio);

                velocities[i][d] = w * velocities[i][d]
                                   + physicsParams.c1 * r1 * (pbest_positions[i][d] - particles[i][d])
                                   + physicsParams.c2 * r2 * (gbest_position[d] - particles[i][d]);

                particles[i][d] += velocities[i][d];

                auto rango = estadoActual.obtenerRangoMuestreo(d);
                double minv = rango.first;
                double maxv = rango.second;
                if (particles[i][d] < minv) particles[i][d] = minv;
                if (particles[i][d] > maxv) particles[i][d] = maxv;
            }

            double cost = calcularCostoPosicionOrientacion(particles[i]);
            if (cost < pbest_costs[i]) {
                pbest_costs[i] = cost;
                pbest_positions[i] = particles[i];
            }
            if (cost < gbest_cost) {
                gbest_cost = cost;
                gbest_position = particles[i];
            }
        }
    }
};


/**
 * Función principal de prueba
 */
int main() {
    std::cout << "=== Entorno de Pruebas PSO para VTOL ===" << std::endl;
    
    ConfigLoader::Config cfg;
    try {
        cfg = ConfigLoader::loadFromJson("config.json");
        std::cout << "✓ config.json cargado exitosamente" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "⚠ Error al cargar config.json: " << e.what() << std::endl;
        std::cerr << "   Usando valores por defecto..." << std::endl;
    }
    
    EntornoPSO env;
    env.establecerParametrosFisicos(PhysicsParameters::fromConfig(cfg));

    std::array<double, DIMENSION_ESTADO> estadoInicial = {
        1000.0, 1050.0, 950.0,
        0.5, 0.5,
        0.0, 0.0,
        0.1, -0.1
    };

    std::array<double, 3> target_pose_position = {0.0, 0.0, 0.0};
    std::array<double, 3> target_pose_orientation = {0.0, 0.0, 0.0};
    if (cfg.target_pose_position.size() == 3) {
        for (size_t i = 0; i < 3; ++i) {
            target_pose_position[i] = cfg.target_pose_position[i];
        }
    }
    if (cfg.target_pose_orientation.size() == 3) {
        for (size_t i = 0; i < 3; ++i) {
            target_pose_orientation[i] = cfg.target_pose_orientation[i];
        }
    }
    if (cfg.has_target_pose) {
        env.establecerTargetPose(target_pose_position, target_pose_orientation);
    } else {
        std::cerr << "Error: config.json must define \"target_pose\". Aborting." << std::endl;
        return 1;
    }

    env.establecerEstadoActual(estadoInicial);
    env.establecerRadioMuestreo(cfg.k_am);
    env.imprimirEstadoYRangosMuestreo();

    const size_t numParticulas = static_cast<size_t>(std::max(2, cfg.num_particles));
    const int iteraciones_maximas = std::max(1, cfg.max_iterations);
    const int ciclos_externos_max = std::max(1, cfg.external_cycles);
    const double umbral_convergencia = std::max(1e-12, cfg.convergence_threshold);
    const double factor_contraccion = std::clamp(cfg.k_am_contraction, 0.01, 0.999);
    const double k_am_minimo = std::max(1e-6, cfg.k_am_min);

    std::cout << "\n=== Ejecutando PSO con meta-iteraciones ===" << std::endl;
    std::cout << "  iteraciones internas por ciclo = " << iteraciones_maximas
              << ", ciclos externos máximo = " << ciclos_externos_max << std::endl;
    // Imprimir resumen de configuración cargada (ruta absoluta y parámetros clave)
    try {
        auto abs_cfg_path = std::filesystem::absolute("config.json");
        std::cout << "Config usada: " << abs_cfg_path.string() << std::endl;
    } catch (...) {
        std::cout << "Config usada: config.json (ruta no resuelta)" << std::endl;
    }
    std::cout << "  num_particles = " << cfg.num_particles << ", max_iterations = " << cfg.max_iterations << std::endl;
    std::cout << "  k_am inicial=" << cfg.k_am << ", contracción=" << factor_contraccion
              << ", k_am mínimo=" << k_am_minimo << std::endl;
    std::cout << "  umbral de convergencia=" << umbral_convergencia << std::endl;
    std::cout << "  w_max=" << cfg.w_max << " w_min=" << cfg.w_min << " c1=" << cfg.c1 << " c2=" << cfg.c2 << std::endl;
    std::cout << "  cost: k_w=" << cfg.k_w << " k_eps=" << cfg.k_epsilon
               << " k_gamma=" << cfg.k_gamma << " k_beta=" << cfg.k_beta
               << " k_dyn=" << cfg.k_dynamics
               << " k_pos=" << cfg.k_pos
               << " k_att=" << cfg.k_att << std::endl;
    std::cout << "  target_pose position=[" << target_pose_position[0] << ", " << target_pose_position[1] << ", " << target_pose_position[2] << "]"
              << " orientation=[" << target_pose_orientation[0] << ", " << target_pose_orientation[1] << ", " << target_pose_orientation[2] << "]"
              << std::endl;

    auto distanciaActuadores = [](const std::array<double, DIMENSION_ESTADO>& a,
                                  const std::array<double, DIMENSION_ESTADO>& b) {
        double sum = 0.0;
        for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
            double d = a[i] - b[i];
            sum += d * d;
        }
        return std::sqrt(sum);
    };

    auto distanciaPosicion = [](const std::array<double, 3>& a,
                                const std::array<double, 3>& b) {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::array<double, DIMENSION_ESTADO> centro_actual = estadoInicial;
    double k_am_actual = std::max(cfg.k_am, k_am_minimo);
    std::vector<double> historial_fitness;
    std::vector<std::array<double, DIMENSION_ESTADO>> historial_centros;
    historial_centros.push_back(centro_actual);
    std::vector<std::array<double, DIMENSION_ESTADO>> particulas_finales;

    std::array<double, DIMENSION_ESTADO> gbest_global = centro_actual;
    double gbest_global_costo = std::numeric_limits<double>::infinity();
    bool converged = false;
    int global_iteration = 0;
    auto pose_inicial_predicha = env.obtenerPosePredicha(centro_actual);
    double distancia_actual_target = distanciaPosicion(pose_inicial_predicha.first, target_pose_position);

    for (int ciclo = 0; ciclo < ciclos_externos_max && !converged; ++ciclo) {
        env.establecerEstadoActual(centro_actual);
        env.establecerRadioMuestreo(k_am_actual);

        auto particulas = env.generarParticulasIniciales(numParticulas);
        std::vector<std::array<double, DIMENSION_ESTADO>> velocidades(particulas.size());
        for (auto& vel : velocidades) vel.fill(0.0);

        std::vector<std::array<double, DIMENSION_ESTADO>> pbest_posiciones = particulas;
        std::vector<double> pbest_costos(particulas.size(), 0.0);
        std::array<double, DIMENSION_ESTADO> gbest_posicion = particulas.front();
        double gbest_costo = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < particulas.size(); ++i) {
            pbest_costos[i] = env.calcularCostoPosicionOrientacion(particulas[i]);
            if (pbest_costos[i] < gbest_costo) {
                gbest_costo = pbest_costos[i];
                gbest_posicion = particulas[i];
            }
        }

        for (int iter = 0; iter < iteraciones_maximas; ++iter) {
            env.actualizar_enjambre(
                particulas, velocidades, pbest_posiciones, pbest_costos,
                gbest_posicion, gbest_costo,
                iter, iteraciones_maximas - 1
            );

            if (gbest_costo < gbest_global_costo) {
                gbest_global_costo = gbest_costo;
                gbest_global = gbest_posicion;
            }

            historial_fitness.push_back(gbest_global_costo);
            auto pose_gbest_predicha = env.obtenerPosePredicha(gbest_posicion);
            distancia_actual_target = distanciaPosicion(pose_gbest_predicha.first, target_pose_position);
            converged = (distancia_actual_target <= umbral_convergencia);

            if (iter % 25 == 0 || converged || iter == iteraciones_maximas - 1) {
                std::cout << "Ciclo " << ciclo
                          << " | Iteración " << iter
                          << " | gBest f = " << gbest_costo
                          << " | dist(target)=" << distancia_actual_target
                          << " | K_AM=" << k_am_actual
                          << std::endl;
            }

            ++global_iteration;
            if (converged) {
                break;
            }
        }

        particulas_finales = particulas;
        centro_actual = gbest_posicion;
        historial_centros.push_back(centro_actual);
        k_am_actual = std::max(k_am_minimo, k_am_actual * factor_contraccion);
    }

    env.establecerEstadoActual(centro_actual);
    env.establecerRadioMuestreo(k_am_actual);

    if (!converged) {
        std::cout << "⚠ No se alcanzó el umbral de convergencia en "
                  << ciclos_externos_max << " ciclos externos." << std::endl;
    }

    if (!particulas_finales.empty()) {
        env.imprimirParticulas(particulas_finales);
    }
    
    std::cout << "\n=== Prueba completada ===" << std::endl;
    std::cout << "Para visualizar: python visualize_particles.py" << std::endl;
    
    return 0;
}