#include "pso_state.hpp"
#include "config_loader.hpp"
#include "pso_runtime.hpp"
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

std::array<double, 3> sumar3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

std::array<double, 3> restar3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

std::array<double, 3> escalar3(const std::array<double, 3>& v, double s) {
    return {v[0] * s, v[1] * s, v[2] * s};
}

std::array<double, 3> productoCruz3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

std::array<double, 4> cuaternionDesdeAnguloEjeY(double angulo) {
    double mitad = angulo * 0.5;
    return {std::cos(mitad), 0.0, std::sin(mitad), 0.0};
}

std::array<double, 4> cuaternionDesdeAnguloEjeX(double angulo) {
    double mitad = angulo * 0.5;
    return {std::cos(mitad), std::sin(mitad), 0.0, 0.0};
}

std::array<double, 4> cuaternionDesdeEuler(double alabeo, double cabeceo, double guinada) {
    double cr = std::cos(alabeo * 0.5);
    double sr = std::sin(alabeo * 0.5);
    double cp = std::cos(cabeceo * 0.5);
    double sp = std::sin(cabeceo * 0.5);
    double cy = std::cos(guinada * 0.5);
    double sy = std::sin(guinada * 0.5);

    return {
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr
    };
}

std::array<double, 4> conjugadoCuaternion(const std::array<double, 4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

std::array<double, 4> multiplicarCuaterniones(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    };
}

std::array<double, 3> rotarVectorPorCuaternion(const std::array<double, 3>& v, const std::array<double, 4>& q) {
    std::array<double, 4> vectorCuaternion = {0.0, v[0], v[1], v[2]};
    std::array<double, 4> qConjugado = conjugadoCuaternion(q);
    std::array<double, 4> rotado = multiplicarCuaterniones(multiplicarCuaterniones(q, vectorCuaternion), qConjugado);
    return {rotado[1], rotado[2], rotado[3]};
}

double productoPuntoCuaternion(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

double distanciaAngularCuaternion(const std::array<double, 4>& qa, const std::array<double, 4>& qb) {
    double d = std::abs(productoPuntoCuaternion(qa, qb));
    d = std::clamp(d, -1.0, 1.0);
    return 2.0 * std::acos(d);
}

double sanearEscalar(double v, double limite = 1e6) {
    if (!std::isfinite(v)) {
        return 0.0;
    }
    return std::clamp(v, -limite, limite);
}

std::array<double, 3> sanearVector3(const std::array<double, 3>& v, double limite = 1e6) {
    return {
        sanearEscalar(v[0], limite),
        sanearEscalar(v[1], limite),
        sanearEscalar(v[2], limite)
    };
}

struct ParametrosFisicos {
    double rho = 1.225;
    double g = 9.81;
    double masa = 12.0;
    std::array<double, 3> inercia = {0.25, 0.28, 0.35};

    double area_ala = 0.28;
    double cl0_ala = 0.28;
    double cl_alpha_ala = 4.6;
    double cd0_ala = 0.028;
    double k_ala = 0.06;

    double area_cola = 0.05;
    double cl_gamma_cola = 0.52;
    double cd_gamma_cola = 0.016;
    double angulo_diedro_cola = 0.785;

    double area_ruddervator = 0.02;
    double cl_ruddervator = 0.28;
    double cd_ruddervator = 0.012;
    double angulo_diedro_ruddervator = 0.785;

    double area_aleron = 0.03;
    double cl_beta_aleron = 0.32;
    double cd_beta_aleron = 0.015;

    double coeficiente_empuje = 2.4e-4;
    double velocidad_aire_referencia = 20.0;

    double ESCALA_OMEGA = 1000.0;
    double ESCALA_ANGULO = 1.0;
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

    static ParametrosFisicos desdeConfiguracion(const CargadorConfiguracion::Configuracion& configuracion) {
        ParametrosFisicos parametros;
        parametros.rho = configuracion.rho;
        parametros.g = configuracion.g;
        parametros.masa = configuracion.masa;
        if (configuracion.inercia.size() == 3) {
            parametros.inercia[0] = configuracion.inercia[0];
            parametros.inercia[1] = configuracion.inercia[1];
            parametros.inercia[2] = configuracion.inercia[2];
        }
        parametros.area_ala = configuracion.area_ala;
        parametros.cl0_ala = configuracion.cl0_ala;
        parametros.cl_alpha_ala = configuracion.cl_alpha_ala;
        parametros.cd0_ala = configuracion.cd0_ala;
        parametros.k_ala = configuracion.k_ala;
        parametros.area_cola = configuracion.area_cola;
        parametros.cl_gamma_cola = configuracion.cl_gamma_cola;
        parametros.cd_gamma_cola = configuracion.cd_gamma_cola;
        parametros.angulo_diedro_cola = configuracion.angulo_diedro_cola;
        parametros.area_ruddervator = configuracion.area_ruddervator;
        parametros.cl_ruddervator = configuracion.cl_ruddervator;
        parametros.cd_ruddervator = configuracion.cd_ruddervator;
        parametros.angulo_diedro_ruddervator = configuracion.angulo_diedro_ruddervator;
        parametros.area_aleron = configuracion.area_aleron;
        parametros.cl_beta_aleron = configuracion.cl_beta_aleron;
        parametros.cd_beta_aleron = configuracion.cd_beta_aleron;
        parametros.coeficiente_empuje = configuracion.coeficiente_empuje;
        parametros.velocidad_aire_referencia = configuracion.velocidad_aire_referencia;
        parametros.ESCALA_OMEGA = configuracion.escala_omega;
        parametros.ESCALA_ANGULO = configuracion.escala_angulo;
        parametros.k_w = configuracion.k_w;
        parametros.k_epsilon = configuracion.k_epsilon;
        parametros.k_gamma = configuracion.k_gamma;
        parametros.k_beta = configuracion.k_beta;
        parametros.k_dinamica = configuracion.k_dinamica;
        parametros.k_pos = configuracion.k_pos;
        parametros.k_orientacion = configuracion.k_orientacion;
        parametros.dt = configuracion.dt;
        parametros.w_max = configuracion.w_max;
        parametros.w_min = configuracion.w_min;
        parametros.c1 = configuracion.c1;
        parametros.c2 = configuracion.c2;
        return parametros;
    }
};

struct FuerzaMomento {
    std::array<double, 3> fuerza = {0.0, 0.0, 0.0};
    std::array<double, 3> momento = {0.0, 0.0, 0.0};
};

}

class EntornoPSO {
private:
    Estado estadoActual;

    std::array<double, 3> posicionMeta;
    std::array<double, 3> orientacionMeta;
    bool poseObjetivoActiva;
    std::array<double, 3> posicionActual;
    std::array<double, 3> orientacionActual;
    std::array<double, 3> velocidadActual;
    std::array<double, 3> velocidadAngularActual;
    ParametrosFisicos parametrosFisicos;
    std::mt19937 generadorAleatorio;

    struct Postura {
        std::array<double, 3> posicion;
        std::array<double, 3> orientacion;
    };

public:
    EntornoPSO()
        : posicionMeta{0.0, 0.0, 0.0},
          orientacionMeta{0.0, 0.0, 0.0},
          poseObjetivoActiva(false),
          posicionActual{0.0, 0.0, 0.0},
          orientacionActual{0.0, 0.0, 0.0},
          velocidadActual{0.0, 0.0, 0.0},
          velocidadAngularActual{0.0, 0.0, 0.0},
          generadorAleatorio(std::random_device{}()) {
        estadoActual.inicializar();
    }

    void establecerVehiculo(const pso_runtime::VehicleState& state) {
        posicionActual = state.position;
        orientacionActual = state.orientation;
        velocidadActual = state.velocity;
        velocidadAngularActual = state.angular_rate;
        parametrosFisicos.velocidad_aire_referencia = state.airspeed_m_s;
    }

    void establecerLimites(const std::vector<double>& minimum, const std::vector<double>& maximum) {
        std::copy(minimum.begin(), minimum.end(), estadoActual.minimum.begin());
        std::copy(maximum.begin(), maximum.end(), estadoActual.maximum.begin());
    }

    void establecerParametrosFisicos(const ParametrosFisicos& parametros) {
        parametrosFisicos = parametros;
    }

    void establecerPoseObjetivo(const std::array<double, 3>& posicionObjetivo,
                                const std::array<double, 3>& orientacionObjetivo) {
        posicionMeta = posicionObjetivo;
        orientacionMeta = orientacionObjetivo;
        poseObjetivoActiva = true;
    }

    bool usaPoseObjetivo() const {
        return poseObjetivoActiva;
    }

    const std::array<double, 3>& obtenerPosicionObjetivo() const {
        return posicionMeta;
    }

    const std::array<double, 3>& obtenerOrientacionObjetivo() const {
        return orientacionMeta;
    }

    void establecerRadioMuestreo(double radio) {
        estadoActual.establecerRadioMuestreo(radio);
    }

    double obtenerRadioMuestreo() const {
        return estadoActual.obtenerRadioMuestreo();
    }

    void establecerEstadoActual(const std::array<double, DIMENSION_ESTADO>& estado) {
        estadoActual.D = estado;
    }

    const Estado& obtenerEstadoActual() const {
        return estadoActual;
    }

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

        const char* etiquetas[] = {
            "ω_I", "ω_D", "ω_T",
            "ε_I", "ε_D",
            "γ_I", "γ_D",
            "β_I", "β_D"
        };

        for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
            auto [valor_min, valor_max] = estadoActual.obtenerRangoMuestreo(i);
            std::cout << "  " << etiquetas[i] << ": [" << valor_min << ", " << valor_max << "]" << std::endl;
        }
    }

    std::vector<std::array<double, DIMENSION_ESTADO>> generarParticulasIniciales(size_t numParticulas) {
        std::vector<std::array<double, DIMENSION_ESTADO>> particulas;
        particulas.reserve(numParticulas);

        for (size_t p = 0; p < numParticulas; ++p) {
            std::array<double, DIMENSION_ESTADO> particula;

            for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
                auto [valor_min, valor_max] = estadoActual.obtenerRangoMuestreo(i);
                std::uniform_real_distribution<double> distribucion(valor_min, valor_max);
                particula[i] = distribucion(generadorAleatorio);
            }

            particulas.push_back(particula);
        }

        return particulas;
    }

    FuerzaMomento calcularFuerzaMomento(const std::array<double, DIMENSION_ESTADO>& particula) const {
        FuerzaMomento resultado;
        const double alpha = 0.0;
        const double V = parametrosFisicos.velocidad_aire_referencia;
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

        const double CL = parametrosFisicos.cl0_ala + parametrosFisicos.cl_alpha_ala * alpha;
        const double CD = parametrosFisicos.cd0_ala + parametrosFisicos.k_ala * CL * CL;
        const double D_ala = -CD * q * parametrosFisicos.area_ala;
        const double L_ala = CL * q * parametrosFisicos.area_ala;
        const auto F_ala = rotarVectorPorCuaternion({0.0, D_ala, L_ala}, cuaternionDesdeAnguloEjeY(-alpha));

        const double L_aleron_izq = -q * parametrosFisicos.cl_beta_aleron * beta_I * parametrosFisicos.area_aleron;
        const double D_aleron_izq = -q * parametrosFisicos.cd_beta_aleron * std::abs(beta_I) * parametrosFisicos.area_aleron;
        const double L_aleron_der = q * parametrosFisicos.cl_beta_aleron * beta_D * parametrosFisicos.area_aleron;
        const double D_aleron_der = -q * parametrosFisicos.cd_beta_aleron * std::abs(beta_D) * parametrosFisicos.area_aleron;
        const auto F_aleron_izq = rotarVectorPorCuaternion({0.0, D_aleron_izq, L_aleron_izq}, cuaternionDesdeAnguloEjeY(-alpha));
        const auto F_aleron_der = rotarVectorPorCuaternion({0.0, D_aleron_der, L_aleron_der}, cuaternionDesdeAnguloEjeY(-alpha));

        const double empuje_I = parametrosFisicos.coeficiente_empuje * omega_I;
        const double empuje_D = parametrosFisicos.coeficiente_empuje * omega_D;
        const double empuje_T = parametrosFisicos.coeficiente_empuje * omega_T;
        const auto F_motor_izq = rotarVectorPorCuaternion({0.0, 0.0, empuje_I}, cuaternionDesdeAnguloEjeY(epsilon_I));
        const auto F_motor_der = rotarVectorPorCuaternion({0.0, 0.0, empuje_D}, cuaternionDesdeAnguloEjeY(epsilon_D));
        const auto F_motor_trasero = std::array<double, 3>{0.0, 0.0, empuje_T};

        const double D_cola_izq = -q * parametrosFisicos.cd_gamma_cola * std::abs(gamma_I) * parametrosFisicos.area_cola;
        const double L_cola_izq = q * parametrosFisicos.cl_gamma_cola * gamma_I * parametrosFisicos.area_cola;
        const double D_cola_der = -q * parametrosFisicos.cd_gamma_cola * std::abs(gamma_D) * parametrosFisicos.area_cola;
        const double L_cola_der = q * parametrosFisicos.cl_gamma_cola * gamma_D * parametrosFisicos.area_cola;
        const auto F_cola_izq = rotarVectorPorCuaternion({0.0, D_cola_izq, L_cola_izq}, cuaternionDesdeAnguloEjeX(-parametrosFisicos.angulo_diedro_cola));
        const auto F_cola_der = rotarVectorPorCuaternion({0.0, D_cola_der, L_cola_der}, cuaternionDesdeAnguloEjeX(parametrosFisicos.angulo_diedro_cola));

        const double efectivo_ruddervator_izq = gamma_I;
        const double efectivo_ruddervator_der = gamma_D;
        const double D_ruddervator_izq = -q * parametrosFisicos.cd_ruddervator * std::abs(efectivo_ruddervator_izq) * parametrosFisicos.area_ruddervator;
        const double L_ruddervator_izq = q * parametrosFisicos.cl_ruddervator * efectivo_ruddervator_izq * parametrosFisicos.area_ruddervator;
        const double D_ruddervator_der = -q * parametrosFisicos.cd_ruddervator * std::abs(efectivo_ruddervator_der) * parametrosFisicos.area_ruddervator;
        const double L_ruddervator_der = q * parametrosFisicos.cl_ruddervator * efectivo_ruddervator_der * parametrosFisicos.area_ruddervator;
        const auto F_ruddervator_izq = rotarVectorPorCuaternion({0.0, D_ruddervator_izq, L_ruddervator_izq}, cuaternionDesdeAnguloEjeX(-parametrosFisicos.angulo_diedro_ruddervator));
        const auto F_ruddervator_der = rotarVectorPorCuaternion({0.0, D_ruddervator_der, L_ruddervator_der}, cuaternionDesdeAnguloEjeX(parametrosFisicos.angulo_diedro_ruddervator));

        resultado.fuerza = F_ala;
        resultado.fuerza = sumar3(resultado.fuerza, F_aleron_izq);
        resultado.fuerza = sumar3(resultado.fuerza, F_aleron_der);
        resultado.fuerza = sumar3(resultado.fuerza, F_motor_izq);
        resultado.fuerza = sumar3(resultado.fuerza, F_motor_der);
        resultado.fuerza = sumar3(resultado.fuerza, F_motor_trasero);
        resultado.fuerza = sumar3(resultado.fuerza, F_cola_izq);
        resultado.fuerza = sumar3(resultado.fuerza, F_cola_der);
        resultado.fuerza = sumar3(resultado.fuerza, F_ruddervator_izq);
        resultado.fuerza = sumar3(resultado.fuerza, F_ruddervator_der);

        const std::array<double, 3> r_ala = {0.6, 0.0, 0.0};
        const std::array<double, 3> r_aleron_izq = {0.7, -0.3, 0.0};
        const std::array<double, 3> r_aleron_der = {0.7, 0.3, 0.0};
        const std::array<double, 3> r_motor_izq = {0.5, -0.25, 0.0};
        const std::array<double, 3> r_motor_der = {0.5, 0.25, 0.0};
        const std::array<double, 3> r_motor_trasero = {-0.6, 0.0, 0.0};
        const std::array<double, 3> r_cola_izq = {-0.7, -0.25, 0.0};
        const std::array<double, 3> r_cola_der = {-0.7, 0.25, 0.0};
        const std::array<double, 3> r_ruddervator_izq = {-0.75, -0.20, 0.0};
        const std::array<double, 3> r_ruddervator_der = {-0.75, 0.20, 0.0};

        resultado.momento = productoCruz3(r_ala, F_ala);
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_aleron_izq, F_aleron_izq));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_aleron_der, F_aleron_der));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_motor_izq, F_motor_izq));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_motor_der, F_motor_der));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_motor_trasero, F_motor_trasero));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_cola_izq, F_cola_izq));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_cola_der, F_cola_der));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_ruddervator_izq, F_ruddervator_izq));
        resultado.momento = sumar3(resultado.momento, productoCruz3(r_ruddervator_der, F_ruddervator_der));

        return resultado;
    }

    Postura predecirPoseResultante(const std::array<double, DIMENSION_ESTADO>& particula) const {
        FuerzaMomento fm = calcularFuerzaMomento(particula);
        auto cuerpoAInercial = cuaternionDesdeEuler(orientacionActual[0], orientacionActual[1], orientacionActual[2]);
        auto F_inercial = rotarVectorPorCuaternion(fm.fuerza, cuerpoAInercial);
        const std::array<double, 3> gravedad = {0.0, 0.0, -parametrosFisicos.g};
        auto aceleracion = sumar3(escalar3(F_inercial, 1.0 / parametrosFisicos.masa), gravedad);

        const double dt = parametrosFisicos.dt;
        Postura resultado;
        resultado.posicion = sumar3(posicionActual, sumar3(escalar3(velocidadActual, dt), escalar3(aceleracion, 0.5 * dt * dt)));

        const std::array<double, 3> omega = velocidadAngularActual;
        std::array<double, 3> fuerzaInercia = {
            parametrosFisicos.inercia[0] * omega[0],
            parametrosFisicos.inercia[1] * omega[1],
            parametrosFisicos.inercia[2] * omega[2]
        };
        auto productoCruzOmega = productoCruz3(omega, fuerzaInercia);
        std::array<double, 3> momentoCuerpo = restar3(fm.momento, productoCruzOmega);
        std::array<double, 3> derivadaOmega = {
            momentoCuerpo[0] / parametrosFisicos.inercia[0],
            momentoCuerpo[1] / parametrosFisicos.inercia[1],
            momentoCuerpo[2] / parametrosFisicos.inercia[2]
        };

        // Integrate body rates through a rotation quaternion, including existing angular velocity.
        const auto delta = sumar3(escalar3(omega, dt), escalar3(derivadaOmega, 0.5 * dt * dt));
        const double angle = std::hypot(delta[0], delta[1], delta[2]);
        const double scale = angle > 1e-12 ? std::sin(angle * 0.5) / angle : 0.5;
        const auto q = multiplicarCuaterniones(cuerpoAInercial,
            {std::cos(angle * 0.5), delta[0]*scale, delta[1]*scale, delta[2]*scale});
        resultado.orientacion = {
            std::atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])),
            std::asin(std::clamp(2*(q[0]*q[2]-q[3]*q[1]), -1.0, 1.0)),
            std::atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
        };
        return resultado;
    }

    std::pair<std::array<double, 3>, std::array<double, 3>> obtenerPosePredicha(
        const std::array<double, DIMENSION_ESTADO>& particula) const {
        const Postura pose = predecirPoseResultante(particula);
        return {sanearVector3(pose.posicion), sanearVector3(pose.orientacion)};
    }

    double calcularCostoPosicionOrientacion(const std::array<double, DIMENSION_ESTADO>& particula) const {
        FuerzaMomento fm = calcularFuerzaMomento(particula);
        Postura posePredicha = predecirPoseResultante(particula);
        std::array<double, 3> posicionPredicha = sanearVector3(posePredicha.posicion);
        std::array<double, 3> orientacionPredicha = sanearVector3(posePredicha.orientacion);

        double dx = posicionPredicha[0] - posicionActual[0];
        double dy = posicionPredicha[1] - posicionActual[1];
        double dz = posicionPredicha[2] - posicionActual[2];

        double dm0 = sanearEscalar(fm.momento[0]);
        double dm1 = sanearEscalar(fm.momento[1]);
        double dm2 = sanearEscalar(fm.momento[2]);

        double magnitudDeltaCuadrado = dx*dx + dy*dy + dz*dz + dm0*dm0 + dm1*dm1 + dm2*dm2;

        double costoPose = 0.0;
        if (poseObjetivoActiva) {
            double ex = posicionPredicha[0] - posicionMeta[0];
            double ey = posicionPredicha[1] - posicionMeta[1];
            double ez = posicionPredicha[2] - posicionMeta[2];
            double errorPosicionCuadrado = ex * ex + ey * ey + ez * ez;

            auto qPredicho = cuaternionDesdeEuler(orientacionPredicha[0], orientacionPredicha[1], orientacionPredicha[2]);
            auto qObjetivo = cuaternionDesdeEuler(orientacionMeta[0], orientacionMeta[1], orientacionMeta[2]);
            double anguloErrorOrientacion = distanciaAngularCuaternion(qPredicho, qObjetivo);
            double errorOrientacionCuadrado = anguloErrorOrientacion * anguloErrorOrientacion;

            costoPose = parametrosFisicos.k_pos * errorPosicionCuadrado + parametrosFisicos.k_orientacion * errorOrientacionCuadrado;
        }

        const auto cuadrado = [](double x) { return x * x; };
        double costo = parametrosFisicos.k_dinamica * magnitudDeltaCuadrado + costoPose;
        return costo;
    }

    void imprimirParticulas(const std::vector<std::array<double, DIMENSION_ESTADO>>& particulas) const {
        std::cout << "\n=== Partículas Iniciales (PSO) ===" << std::endl;
        std::cout << "Total de partículas: " << particulas.size() << std::endl;
        std::cout << std::fixed << std::setprecision(4);

        const char* etiquetas[] = {
            "ω_I", "ω_D", "ω_T",
            "ε_I", "ε_D",
            "γ_I", "γ_D",
            "β_I", "β_D"
        };

        for (size_t p = 0; p < particulas.size() && p < 5; ++p) {
            std::cout << "\nPartícula " << p << ":" << std::endl;
            for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
                std::cout << "  " << etiquetas[i] << ": " << particulas[p][i] << std::endl;
            }
            double valorF = calcularCostoPosicionOrientacion(particulas[p]);
            std::cout << "  f: " << std::fixed << std::setprecision(6) << valorF << std::endl;
        }

        if (particulas.size() > 5) {
            std::cout << "\n... (" << (particulas.size() - 5) << " partículas más)" << std::endl;
        }
    }

    void actualizar_enjambre(std::vector<std::array<double, DIMENSION_ESTADO>>& particulas,
                             std::vector<std::array<double, DIMENSION_ESTADO>>& velocidades,
                             std::vector<std::array<double, DIMENSION_ESTADO>>& mejoresPosiciones,
                             std::vector<double>& mejoresCostos,
                             std::array<double, DIMENSION_ESTADO>& mejorPosicionGlobal,
                             double& mejorCostoGlobal,
                             int iter, int iteracionesMaximas) {
        if (particulas.size() != velocidades.size() || particulas.size() != mejoresPosiciones.size() || particulas.size() != mejoresCostos.size()) {
            throw std::runtime_error("Tamaños de vectores de partículas no coinciden");
        }

        double w = parametrosFisicos.w_max;
        if (iteracionesMaximas > 0) {
            int t = std::min(std::max(iter, 0), iteracionesMaximas);
            w = parametrosFisicos.w_max - (parametrosFisicos.w_max - parametrosFisicos.w_min) * (static_cast<double>(t) / static_cast<double>(iteracionesMaximas));
        }

        std::uniform_real_distribution<double> dist01(0.0, 1.0);

        for (size_t i = 0; i < particulas.size(); ++i) {
            for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
                double r1 = dist01(generadorAleatorio);
                double r2 = dist01(generadorAleatorio);

                velocidades[i][d] = w * velocidades[i][d]
                                   + parametrosFisicos.c1 * r1 * (mejoresPosiciones[i][d] - particulas[i][d])
                                   + parametrosFisicos.c2 * r2 * (mejorPosicionGlobal[d] - particulas[i][d]);

                particulas[i][d] += velocidades[i][d];

                auto rango = estadoActual.obtenerRangoMuestreo(d);
                double minv = rango.first;
                double maxv = rango.second;
                if (particulas[i][d] < minv) particulas[i][d] = minv;
                if (particulas[i][d] > maxv) particulas[i][d] = maxv;
            }

            double costo = calcularCostoPosicionOrientacion(particulas[i]);
            if (costo < mejoresCostos[i]) {
                mejoresCostos[i] = costo;
                mejoresPosiciones[i] = particulas[i];
            }
            if (costo < mejorCostoGlobal) {
                mejorCostoGlobal = costo;
                mejorPosicionGlobal = particulas[i];
            }
        }
    }

    void exportarPsoAJson(const std::vector<std::array<double, DIMENSION_ESTADO>>& particulas,
                          const std::array<double, DIMENSION_ESTADO>& mejorGlobal,
                          const std::vector<double>& historialAptitud,
                          const std::vector<std::array<double, DIMENSION_ESTADO>>& historialCentros,
                          int iteracion,
                          int iteracionExterna,
                          int iteracionGlobal,
                          const std::array<double, 3>& posicionObjetivo,
                          const std::array<double, 3>& orientacionObjetivo,
                          const std::array<double, 3>& posicionPredichaMejorGlobal,
                          const std::array<double, 3>& orientacionPredichaMejorGlobal,
                          double k_am_actual,
                          double distanciaAObjetivo,
                          bool convergio,
                          const CargadorConfiguracion::Configuracion& configuracionAplicada,
                          const std::string& nombreArchivo = "pso_data.json") const {
        std::stringstream flujo;
        flujo << std::setprecision(10);

        flujo << "{\"particles\":[";
        for (size_t p = 0; p < particulas.size(); ++p) {
            if (p > 0) flujo << ",";
            flujo << "[";
            for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
                if (d > 0) flujo << ",";
                flujo << particulas[p][d];
            }
            flujo << "]";
        }
        flujo << "],\"gbest\":[";

        for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
            if (d > 0) flujo << ",";
            flujo << mejorGlobal[d];
        }

        flujo << "],\"target_pose\":{\"position\":["
           << posicionObjetivo[0] << "," << posicionObjetivo[1] << "," << posicionObjetivo[2]
           << "],\"orientation\":["
           << orientacionObjetivo[0] << "," << orientacionObjetivo[1] << "," << orientacionObjetivo[2]
           << "]}";

        flujo << ",\"predicted_gbest_pose\":{\"position\":["
           << posicionPredichaMejorGlobal[0] << "," << posicionPredichaMejorGlobal[1] << "," << posicionPredichaMejorGlobal[2]
           << "],\"orientation\":["
           << orientacionPredichaMejorGlobal[0] << "," << orientacionPredichaMejorGlobal[1] << "," << orientacionPredichaMejorGlobal[2]
           << "]}";

        flujo << ",\"fitness_history\":[";
        for (size_t i = 0; i < historialAptitud.size(); ++i) {
            if (i > 0) flujo << ",";
            flujo << historialAptitud[i];
        }

        flujo << "],\"center_history\":[";
        for (size_t c = 0; c < historialCentros.size(); ++c) {
            if (c > 0) flujo << ",";
            flujo << "[";
            for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
                if (d > 0) flujo << ",";
                flujo << historialCentros[c][d];
            }
            flujo << "]";
        }

        flujo << "],\"search_center\":[";
        for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
            if (d > 0) flujo << ",";
            flujo << estadoActual.D[d];
        }

        flujo << "],\"applied_config\":{";
        flujo << "\"num_particles\":" << configuracionAplicada.num_particulas << ",";
        flujo << "\"max_iterations\":" << configuracionAplicada.max_iteraciones << ",";
        flujo << "\"external_cycles\":" << configuracionAplicada.ciclos_externos << ",";
        flujo << "\"k_am\":" << configuracionAplicada.k_am << ",";
        flujo << "\"k_am_contraction\":" << configuracionAplicada.contraccion_k_am << ",";
        flujo << "\"k_am_min\":" << configuracionAplicada.k_am_minimo << ",";
        flujo << "\"convergence_threshold\":" << configuracionAplicada.umbral_convergencia << ",";
        flujo << "\"w_max\":" << configuracionAplicada.w_max << ",";
        flujo << "\"w_min\":" << configuracionAplicada.w_min << ",";
        flujo << "\"c1\":" << configuracionAplicada.c1 << ",";
        flujo << "\"c2\":" << configuracionAplicada.c2 << ",";
        flujo << "\"thrust_coefficient\":" << configuracionAplicada.coeficiente_empuje << ",";
        flujo << "\"wing_area\":" << configuracionAplicada.area_ala << ",";
        flujo << "\"omega_scale\":" << configuracionAplicada.escala_omega << ",";
        flujo << "\"k_dynamics\":" << configuracionAplicada.k_dinamica << ",";
        flujo << "\"k_pos\":" << configuracionAplicada.k_pos << ",";
        flujo << "\"k_att\":" << configuracionAplicada.k_orientacion << ",";
        flujo << "\"target_pose_enabled\":" << (configuracionAplicada.tiene_pose_objetivo ? "true" : "false");
        flujo << "},\"iteration\":" << iteracion
           << ",\"outer_iteration\":" << iteracionExterna
           << ",\"global_iteration\":" << iteracionGlobal
           << ",\"k_am_current\":" << k_am_actual
           << ",\"distance_to_target\":" << distanciaAObjetivo
           << ",\"converged\":" << (convergio ? "true" : "false")
           << "}";

        std::ofstream archivo(nombreArchivo, std::ios::trunc);
        if (!archivo.is_open()) {
            std::cerr << "Error: No se pudo abrir el archivo " << nombreArchivo << std::endl;
            return;
        }
        archivo << flujo.str();
    }

    void imprimirVisualizacion3D(const std::vector<std::array<double, DIMENSION_ESTADO>>& particulas) const {
        std::vector<size_t> todasLasDimensiones(DIMENSION_ESTADO);
        for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
            todasLasDimensiones[i] = i;
        }

        std::mt19937 generadorTemporal = generadorAleatorio;
        std::shuffle(todasLasDimensiones.begin(), todasLasDimensiones.end(), generadorTemporal);

        std::array<size_t, 3> dimensiones = {todasLasDimensiones[0], todasLasDimensiones[1], todasLasDimensiones[2]};

        const char* etiquetas[] = {
            "ω_I", "ω_D", "ω_T",
            "ε_I", "ε_D",
            "γ_I", "γ_D",
            "β_I", "β_D"
        };

        std::cout << "\n=== Visualización 3D ASCII ===" << std::endl;
        std::cout << "Dimensiones: " << etiquetas[dimensiones[0]] << ", " << etiquetas[dimensiones[1]] << ", " << etiquetas[dimensiones[2]] << std::endl;
        std::cout << "Estado actual: (" << estadoActual.D[dimensiones[0]] << ", "
                  << estadoActual.D[dimensiones[1]] << ", " << estadoActual.D[dimensiones[2]] << ")" << std::endl;

        double min0 = estadoActual.D[dimensiones[0]], max0 = estadoActual.D[dimensiones[0]];
        double min1 = estadoActual.D[dimensiones[1]], max1 = estadoActual.D[dimensiones[1]];
        double min2 = estadoActual.D[dimensiones[2]], max2 = estadoActual.D[dimensiones[2]];

        for (const auto& particula : particulas) {
            min0 = std::min(min0, particula[dimensiones[0]]);
            max0 = std::max(max0, particula[dimensiones[0]]);
            min1 = std::min(min1, particula[dimensiones[1]]);
            max1 = std::max(max1, particula[dimensiones[1]]);
            min2 = std::min(min2, particula[dimensiones[2]]);
            max2 = std::max(max2, particula[dimensiones[2]]);
        }

        std::cout << "Rangos: " << etiquetas[dimensiones[0]] << "[" << min0 << "," << max0 << "], "
                  << etiquetas[dimensiones[1]] << "[" << min1 << "," << max1 << "], "
                  << etiquetas[dimensiones[2]] << "[" << min2 << "," << max2 << "]" << std::endl;

        std::cout << "\nPrimeras 10 partículas:" << std::endl;
        for (size_t i = 0; i < std::min(size_t(10), particulas.size()); ++i) {
            std::cout << "P" << i << ": (" << particulas[i][dimensiones[0]] << ", "
                      << particulas[i][dimensiones[1]] << ", " << particulas[i][dimensiones[2]] << ")" << std::endl;
        }
    }
};

namespace pso_runtime {
namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::invalid_argument(message);
}
template<class Values> void finiteValues(const Values& values) {
    for (double v : values) require(std::isfinite(v) && std::abs(v) <= 1e6, "Non-finite or excessive PSO input");
}
}

Optimizer::Optimizer(const std::string& config_path)
    : config_(CargadorConfiguracion::cargarDesdeJson(config_path)) {
    const auto& c = config_;
    require(c.tiene_pose_objetivo, "PSO requires target_pose");
    require(c.actuator_min.size() == DIMENSION_ESTADO && c.actuator_max.size() == DIMENSION_ESTADO &&
            c.actuator_initial.size() == DIMENSION_ESTADO, "PSO requires nine actuator min/max/initial values");
    finiteValues(c.actuator_min); finiteValues(c.actuator_max); finiteValues(c.actuator_initial);
    finiteValues(c.posicion_objetivo); finiteValues(c.orientacion_objetivo); finiteValues(c.inercia);
    require(c.inercia.size() == 3, "PSO requires three inertia values");
    for (double v : c.inercia) require(v > 0, "Inertia must be positive");
    finiteValues(std::initializer_list<double>{c.rho,c.g,c.masa,c.dt,c.area_ala,c.cl0_ala,c.cl_alpha_ala,
        c.cd0_ala,c.k_ala,c.area_cola,c.cl_gamma_cola,c.cd_gamma_cola,c.angulo_diedro_cola,
        c.area_ruddervator,c.cl_ruddervator,c.cd_ruddervator,c.angulo_diedro_ruddervator,
        c.area_aleron,c.cl_beta_aleron,c.cd_beta_aleron,c.coeficiente_empuje,
        c.k_dinamica,c.k_pos,c.k_orientacion,c.w_max,c.w_min,c.c1,c.c2,c.k_am,
        c.contraccion_k_am,c.k_am_minimo,c.umbral_convergencia});
    require(c.masa > 0 && c.dt > 0 && c.dt <= 1 && c.rho > 0 && c.g > 0 && c.coeficiente_empuje > 0,
            "Invalid PSO physics");
    require(c.k_dinamica >= 0 && c.k_pos > 0 && c.k_orientacion >= 0 && c.c1 >= 0 && c.c2 >= 0 &&
            c.w_min >= 0 && c.w_max >= c.w_min && c.k_am > 0 && c.k_am_minimo > 0 &&
            c.contraccion_k_am > 0 && c.contraccion_k_am <= 1 && c.umbral_convergencia > 0 &&
            c.num_particulas >= 2 && c.max_iteraciones >= 1 && c.ciclos_externos >= 1, "Invalid PSO parameters");
    for (size_t d = 0; d < DIMENSION_ESTADO; ++d) {
        require(c.actuator_min[d] <= c.actuator_max[d] && c.actuator_initial[d] >= c.actuator_min[d] &&
                c.actuator_initial[d] <= c.actuator_max[d], "Invalid actuator limits or initial state");
        require(d >= 3 || c.actuator_min[d] >= 0, "Negative rotor limits");
        initial_actuator_state_[d] = c.actuator_initial[d];
    }
}

const std::array<double, DIMENSION_ESTADO>& Optimizer::initialActuatorState() const {
    return initial_actuator_state_;
}

Result Optimizer::solve(const VehicleState& state,
                       const std::array<double, DIMENSION_ESTADO>& search_center,
                       const RuntimeLimits& limits) const {
    finiteValues(state.position); finiteValues(state.orientation); finiteValues(state.velocity);
    finiteValues(state.angular_rate); finiteValues(search_center);
    require(std::isfinite(state.airspeed_m_s) && state.airspeed_m_s >= 0 && state.airspeed_m_s <= 1000,
            "Invalid airspeed");
    require(limits.particles >= 2 && limits.particles <= 256 && limits.iterations >= 1 && limits.iterations <= 100 &&
            limits.external_cycles >= 1 && limits.external_cycles <= 30, "Invalid runtime budget");
    const auto& c = config_;
    for (size_t d = 0; d < DIMENSION_ESTADO; ++d)
        require(search_center[d] >= c.actuator_min[d] && search_center[d] <= c.actuator_max[d], "Search center out of bounds");
    EntornoPSO environment;
    environment.establecerParametrosFisicos(ParametrosFisicos::desdeConfiguracion(c));
    environment.establecerVehiculo(state);
    environment.establecerLimites(c.actuator_min, c.actuator_max);
    environment.establecerPoseObjetivo({c.posicion_objetivo[0],c.posicion_objetivo[1],c.posicion_objetivo[2]},
                                     {c.orientacion_objetivo[0],c.orientacion_objetivo[1],c.orientacion_objetivo[2]});
    Result result;
    result.actuators = search_center;
    result.fitness = environment.calcularCostoPosicionOrientacion(search_center);
    result.objective_evaluations = 1;
    const size_t count = std::min(limits.particles, static_cast<size_t>(c.num_particulas));
    const int iterations = std::min(limits.iterations, c.max_iteraciones);
    double radius = c.k_am;
    for (int cycle = 0; cycle < std::min(limits.external_cycles, c.ciclos_externos); ++cycle) {
        environment.establecerEstadoActual(result.actuators);
        environment.establecerRadioMuestreo(radius);
        auto particles = environment.generarParticulasIniciales(count);
        particles[0] = result.actuators; // Retain the incumbent so random sampling cannot worsen it.
        auto best = particles;
        std::vector<std::array<double, DIMENSION_ESTADO>> velocity(count);
        std::vector<double> costs(count);
        for (size_t i = 0; i < count; ++i) {
            costs[i] = environment.calcularCostoPosicionOrientacion(particles[i]);
            if (costs[i] < result.fitness) { result.fitness = costs[i]; result.actuators = particles[i]; }
        }
        result.objective_evaluations += count;
        for (int iteration = 0; iteration < iterations; ++iteration) {
            environment.actualizar_enjambre(particles, velocity, best, costs,
                result.actuators, result.fitness, iteration, iterations - 1);
            result.objective_evaluations += count;
        }
        radius = std::max(c.k_am_minimo, radius * c.contraccion_k_am);
    }
    const auto pose = environment.obtenerPosePredicha(result.actuators);
    result.position_error_m = std::hypot(pose.first[0]-c.posicion_objetivo[0],
                                       pose.first[1]-c.posicion_objetivo[1], pose.first[2]-c.posicion_objetivo[2]);
    result.position_converged = result.position_error_m <= c.umbral_convergencia;
    require(std::isfinite(result.fitness) && std::isfinite(result.position_error_m), "Invalid PSO result");
    finiteValues(result.actuators);
    return result;
}
} // namespace pso_runtime

#ifndef PSO_RUNTIME_ONLY
int main() {
    std::cout << "=== Entorno de Pruebas PSO para VTOL ===" << std::endl;

    CargadorConfiguracion::Configuracion configuracion;
    try {
        configuracion = CargadorConfiguracion::cargarDesdeJson("config.json");
        std::cout << "✓ config.json cargado exitosamente" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "⚠ Error al cargar config.json: " << e.what() << std::endl;
        std::cerr << "   Usando valores por defecto..." << std::endl;
    }

    EntornoPSO entorno;
    entorno.establecerParametrosFisicos(ParametrosFisicos::desdeConfiguracion(configuracion));

    std::array<double, DIMENSION_ESTADO> estadoInicial = {
        1000.0, 1050.0, 950.0,
        0.5, 0.5,
        0.0, 0.0,
        0.1, -0.1
    };

    std::array<double, 3> posicionPoseObjetivo = {0.0, 0.0, 0.0};
    std::array<double, 3> orientacionPoseObjetivo = {0.0, 0.0, 0.0};
    if (configuracion.posicion_objetivo.size() == 3) {
        for (size_t i = 0; i < 3; ++i) {
            posicionPoseObjetivo[i] = configuracion.posicion_objetivo[i];
        }
    }
    if (configuracion.orientacion_objetivo.size() == 3) {
        for (size_t i = 0; i < 3; ++i) {
            orientacionPoseObjetivo[i] = configuracion.orientacion_objetivo[i];
        }
    }
    if (configuracion.tiene_pose_objetivo) {
        entorno.establecerPoseObjetivo(posicionPoseObjetivo, orientacionPoseObjetivo);
    } else {
        std::cerr << "Error: config.json debe definir \"target_pose\". Abortando." << std::endl;
        return 1;
    }

    entorno.establecerEstadoActual(estadoInicial);
    entorno.establecerRadioMuestreo(configuracion.k_am);
    entorno.imprimirEstadoYRangosMuestreo();

    const size_t numParticulas = static_cast<size_t>(std::max(2, configuracion.num_particulas));
    const int iteracionesMaximas = std::max(1, configuracion.max_iteraciones);
    const int ciclosExternosMax = std::max(1, configuracion.ciclos_externos);
    const double umbralConvergencia = std::max(1e-12, configuracion.umbral_convergencia);
    const double factorContraccion = std::clamp(configuracion.contraccion_k_am, 0.01, 0.999);
    const double kAmMinimo = std::max(1e-6, configuracion.k_am_minimo);

    std::cout << "\n=== Ejecutando PSO con meta-iteraciones ===" << std::endl;
    std::cout << "  iteraciones internas por ciclo = " << iteracionesMaximas
              << ", ciclos externos máximo = " << ciclosExternosMax << std::endl;
    try {
        auto rutaConfigAbsoluta = std::filesystem::absolute("config.json");
        std::cout << "Config usada: " << rutaConfigAbsoluta.string() << std::endl;
    } catch (...) {
        std::cout << "Config usada: config.json (ruta no resuelta)" << std::endl;
    }
    std::cout << "  num_particles = " << configuracion.num_particulas << ", max_iterations = " << configuracion.max_iteraciones << std::endl;
    std::cout << "  k_am inicial=" << configuracion.k_am << ", contracción=" << factorContraccion
              << ", k_am mínimo=" << kAmMinimo << std::endl;
    std::cout << "  umbral de convergencia=" << umbralConvergencia << std::endl;
    std::cout << "  w_max=" << configuracion.w_max << " w_min=" << configuracion.w_min << " c1=" << configuracion.c1 << " c2=" << configuracion.c2 << std::endl;
    std::cout << "  cost: k_w=" << configuracion.k_w << " k_eps=" << configuracion.k_epsilon
               << " k_gamma=" << configuracion.k_gamma << " k_beta=" << configuracion.k_beta
               << " k_dyn=" << configuracion.k_dinamica
               << " k_pos=" << configuracion.k_pos
               << " k_att=" << configuracion.k_orientacion << std::endl;
    std::cout << "  target_pose position=[" << posicionPoseObjetivo[0] << ", " << posicionPoseObjetivo[1] << ", " << posicionPoseObjetivo[2] << "]"
              << " orientation=[" << orientacionPoseObjetivo[0] << ", " << orientacionPoseObjetivo[1] << ", " << orientacionPoseObjetivo[2] << "]"
              << std::endl;

    auto distanciaActuadores = [](const std::array<double, DIMENSION_ESTADO>& a,
                                  const std::array<double, DIMENSION_ESTADO>& b) {
        double suma = 0.0;
        for (size_t i = 0; i < DIMENSION_ESTADO; ++i) {
            double d = a[i] - b[i];
            suma += d * d;
        }
        return std::sqrt(suma);
    };

    auto distanciaPosicion = [](const std::array<double, 3>& a,
                                const std::array<double, 3>& b) {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::array<double, DIMENSION_ESTADO> centroActual = estadoInicial;
    double kAmActual = std::max(configuracion.k_am, kAmMinimo);
    std::vector<double> historialAptitud;
    std::vector<std::array<double, DIMENSION_ESTADO>> historialCentros;
    historialCentros.push_back(centroActual);
    std::vector<std::array<double, DIMENSION_ESTADO>> particulasFinales;

    std::array<double, DIMENSION_ESTADO> mejorGlobal = centroActual;
    double mejorCostoGlobal = std::numeric_limits<double>::infinity();
    bool convergio = false;
    int iteracionGlobal = 0;
    auto posePredichaInicial = entorno.obtenerPosePredicha(centroActual);
    double distanciaActualObjetivo = distanciaPosicion(posePredichaInicial.first, posicionPoseObjetivo);

    for (int ciclo = 0; ciclo < ciclosExternosMax && !convergio; ++ciclo) {
        entorno.establecerEstadoActual(centroActual);
        entorno.establecerRadioMuestreo(kAmActual);

        auto particulas = entorno.generarParticulasIniciales(numParticulas);
        std::vector<std::array<double, DIMENSION_ESTADO>> velocidades(particulas.size());
        for (auto& vel : velocidades) vel.fill(0.0);

        std::vector<std::array<double, DIMENSION_ESTADO>> mejoresPosiciones = particulas;
        std::vector<double> mejoresCostos(particulas.size(), 0.0);
        std::array<double, DIMENSION_ESTADO> mejorPosicionCiclo = particulas.front();
        double mejorCostoCiclo = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < particulas.size(); ++i) {
            mejoresCostos[i] = entorno.calcularCostoPosicionOrientacion(particulas[i]);
            if (mejoresCostos[i] < mejorCostoCiclo) {
                mejorCostoCiclo = mejoresCostos[i];
                mejorPosicionCiclo = particulas[i];
            }
        }

        for (int iter = 0; iter < iteracionesMaximas; ++iter) {
            entorno.actualizar_enjambre(
                particulas, velocidades, mejoresPosiciones, mejoresCostos,
                mejorPosicionCiclo, mejorCostoCiclo,
                iter, iteracionesMaximas - 1
            );

            if (mejorCostoCiclo < mejorCostoGlobal) {
                mejorCostoGlobal = mejorCostoCiclo;
                mejorGlobal = mejorPosicionCiclo;
            }

            historialAptitud.push_back(mejorCostoGlobal);
            auto posePredichaMejorCiclo = entorno.obtenerPosePredicha(mejorPosicionCiclo);
            distanciaActualObjetivo = distanciaPosicion(posePredichaMejorCiclo.first, posicionPoseObjetivo);
            convergio = (distanciaActualObjetivo <= umbralConvergencia);

            entorno.exportarPsoAJson(
                particulas, mejorPosicionCiclo, historialAptitud, historialCentros,
                iter, ciclo, iteracionGlobal,
                posicionPoseObjetivo, orientacionPoseObjetivo,
                posePredichaMejorCiclo.first, posePredichaMejorCiclo.second,
                kAmActual, distanciaActualObjetivo, convergio, configuracion
            );

            if (iter % 25 == 0 || convergio || iter == iteracionesMaximas - 1) {
                std::cout << "Ciclo " << ciclo
                          << " | Iteración " << iter
                          << " | gBest f = " << mejorCostoCiclo
                          << " | dist(target)=" << distanciaActualObjetivo
                          << " | K_AM=" << kAmActual
                          << std::endl;
            }

            ++iteracionGlobal;
            if (convergio) {
                break;
            }
        }

        particulasFinales = particulas;
        centroActual = mejorPosicionCiclo;
        historialCentros.push_back(centroActual);
        kAmActual = std::max(kAmMinimo, kAmActual * factorContraccion);
    }

    entorno.establecerEstadoActual(centroActual);
    entorno.establecerRadioMuestreo(kAmActual);

    if (!convergio) {
        std::cout << "⚠ No se alcanzó el umbral de convergencia en "
                  << ciclosExternosMax << " ciclos externos." << std::endl;
    }

    if (!particulasFinales.empty()) {
        entorno.imprimirVisualizacion3D(particulasFinales);
        entorno.imprimirParticulas(particulasFinales);
    }

    std::cout << "\n=== Prueba completada ===" << std::endl;
    std::cout << "Para visualizar: python visualize_particles.py" << std::endl;

    return 0;
}

#endif
