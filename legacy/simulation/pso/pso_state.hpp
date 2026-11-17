#ifndef PSO_ESTADO_HPP
#define PSO_ESTADO_HPP

#include <array>
#include <algorithm>
#include <limits>
#include <cmath>
#include <stdexcept>

constexpr size_t DIMENSION_ESTADO = 9;

constexpr size_t IND_OMEGA_I = 0;
constexpr size_t IND_OMEGA_D = 1;
constexpr size_t IND_OMEGA_T = 2;

constexpr size_t IND_EPSILON_I = 3;
constexpr size_t IND_EPSILON_D = 4;

constexpr size_t IND_GAMMA_I = 5;
constexpr size_t IND_GAMMA_D = 6;

constexpr size_t IND_BETA_I = 7;
constexpr size_t IND_BETA_D = 8;

constexpr std::array<double, DIMENSION_ESTADO> FACTOR_MUESTREO_DIM = {
    1.0, 1.0, 1.0,
    0.02, 0.02,
    0.02, 0.02,
    0.02, 0.02
};

constexpr double K_AM_DEFAULT = 5;

struct Estado {
    std::array<double, DIMENSION_ESTADO> D;
    double radioMuestreo = K_AM_DEFAULT;
    std::array<double, DIMENSION_ESTADO> minimum = [] {
        std::array<double, DIMENSION_ESTADO> v{};
        v.fill(-std::numeric_limits<double>::infinity()); return v;
    }();
    std::array<double, DIMENSION_ESTADO> maximum = [] {
        std::array<double, DIMENSION_ESTADO> v{};
        v.fill(std::numeric_limits<double>::infinity()); return v;
    }();

    std::pair<double, double> obtenerRangoMuestreo(size_t indice) const {
        if (indice >= DIMENSION_ESTADO) {
            throw std::out_of_range("Índice fuera del rango del estado");
        }
        const double amplitud = radioMuestreo * FACTOR_MUESTREO_DIM[indice];
        return {std::max(minimum[indice], D[indice] - amplitud),
                std::min(maximum[indice], D[indice] + amplitud)};
    }

    void establecerRadioMuestreo(double nuevoRadio) {
        radioMuestreo = (nuevoRadio > 0.0) ? nuevoRadio : 1e-6;
    }

    double obtenerRadioMuestreo() const {
        return radioMuestreo;
    }

    void inicializar() {
        D.fill(0.0);
    }
};

#endif
