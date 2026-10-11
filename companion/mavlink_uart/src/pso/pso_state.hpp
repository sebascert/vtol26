#ifndef PSO_ESTADO_HPP
#define PSO_ESTADO_HPP

#include <array>
#include <cmath>
#include <stdexcept>

/**
 * @file pso_state.hpp
 * @brief Define el estado del VTOL y parámetros para el algoritmo PSO
 *
 * Vector de estado D (magnitud 9):
 * - Índices 0-2: Velocidades angulares de rotores [ω_I, ω_D, ω_T]
 * - Índices 3-4: Ángulos rotores basculantes delanteros [ε_I, ε_D]
 * - Índices 5-6: Ángulos ruddervators [γ_I, γ_D]
 * - Índices 7-8: Ángulos alerones [β_I, β_D]
 */

constexpr size_t DIMENSION_ESTADO = 9;

// Índices del vector de estado (constantes propias)
constexpr size_t IND_OMEGA_I = 0;      // Velocidad angular rotor izquierdo
constexpr size_t IND_OMEGA_D = 1;      // Velocidad angular rotor derecho
constexpr size_t IND_OMEGA_T = 2;      // Velocidad angular rotor trasero

constexpr size_t IND_EPSILON_I = 3;    // Ángulo rotor basculante izquierdo
constexpr size_t IND_EPSILON_D = 4;    // Ángulo rotor basculante derecho

constexpr size_t IND_GAMMA_I = 5;      // Ángulo ruddervator izquierdo
constexpr size_t IND_GAMMA_D = 6;      // Ángulo ruddervator derecho

constexpr size_t IND_BETA_I = 7;       // Ángulo alerón izquierdo
constexpr size_t IND_BETA_D = 8;       // Ángulo alerón derecho

// Escala por dimensión para el radio de muestreo global.
// Rotores mantienen el radio completo; superficies usan una fracción.
constexpr std::array<double, DIMENSION_ESTADO> FACTOR_MUESTREO_DIM = {
    1.0, 1.0, 1.0,   // ω_I, ω_D, ω_T
    0.02, 0.02,      // ε_I, ε_D
    0.02, 0.02,      // γ_I, γ_D
    0.02, 0.02       // β_I, β_D
};

/**
 * Constante K_AM: Define la magnitud del área de muestreo
 * Para cada elemento D[i], el rango de muestreo será: [D[i] - K_AM, D[i] + K_AM]
 */
constexpr double K_AM_DEFAULT = 5;  // Ajustar según necesidades

/**
 * @struct Estado
 * @brief Representa el estado actual del dron VTOL
 */
struct Estado {
    std::array<double, DIMENSION_ESTADO> D;
    double radioMuestreo = K_AM_DEFAULT;

    /**
     * Obtiene el rango de muestreo para una dimensión específica
     * @param indice Índice de la dimensión
     * @return Par {min, max} del rango de muestreo
     */
    std::pair<double, double> obtenerRangoMuestreo(size_t indice) const {
        if (indice >= DIMENSION_ESTADO) {
            throw std::out_of_range("Índice fuera del rango del estado");
        }
        const double span = radioMuestreo * FACTOR_MUESTREO_DIM[indice];
        return {D[indice] - span, D[indice] + span};
    }

    void establecerRadioMuestreo(double nuevoRadio) {
        radioMuestreo = (nuevoRadio > 0.0) ? nuevoRadio : 1e-6;
    }

    double obtenerRadioMuestreo() const {
        return radioMuestreo;
    }

    /**
     * Inicializa el estado con valores por defecto
     */
    void inicializar() {
        D.fill(0.0);
    }
};

#endif // PSO_ESTADO_HPP