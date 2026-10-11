#pragma once

#include "module_interfaces.hpp"
#include "module_data_types.hpp"
#include <cmath>

namespace stallion {

/**
 * @brief Controlador de actitud basado en cuaterniones (inspirado en PX4)
 * 
 * Implementa control geométrico:
 * - Error cuaterniónico: q_error = q_desired * inverse(q_current)
 * - Conversión a torques mediante ganancia proporcional
 * - Soporte para control de velocidades angulares (opcional)
 */
class QuaternionAttitudeController : public IAttitudeController {
public:
    QuaternionAttitudeController();
    ~QuaternionAttitudeController() override = default;

    /**
     * @brief Actualizar controlador
     * @param state Estado actual del sistema (incluye quaternion)
     * @param setpoint Setpoint de control (roll, pitch, yaw, thrust)
     * @param dt Tiempo desde última actualización (s)
     */
    bool update(const StateEstimate& state, const ControlSetpoint& setpoint, float dt) override;

    /**
     * @brief Obtener salida de control (torques + thrust)
     */
    ControlOutput get_output() const override;

    bool is_healthy() const override { return is_healthy_; }
    void reset() override;

    /**
     * @brief Configurar ganancias de control
     */
    void set_attitude_gain(float kp) { kp_attitude_ = kp; }
    void set_rate_gain(float kp) { kp_rate_ = kp; }
    void set_thrust_max(float max_thrust) { thrust_max_ = max_thrust; }

private:
    ControlOutput output_;
    bool is_healthy_;

    // Ganancias de control
    float kp_attitude_;  // Ganancia para error de actitud
    float kp_rate_;      // Ganancia para error de velocidad angular

    // Límites
    float thrust_max_;

    // Estado interno
    Quaternion q_desired_;
    float last_roll_sp_;
    float last_pitch_sp_;
    float last_yaw_sp_;

    /**
     * @brief Convertir setpoint Euler a cuaternión deseado
     */
    void euler_to_quaternion(float roll, float pitch, float yaw, Quaternion& q);

    /**
     * @brief Calcular error cuaterniónico
     * q_error representa la rotación necesaria para pasar de q_current a q_desired
     */
    Quaternion quaternion_error(const Quaternion& q_desired, const Quaternion& q_current);

    /**
     * @brief Convertir quaternion error a torques
     */
    void quaternion_error_to_torques(const Quaternion& q_error, float dt,
                                     float& torque_roll, float& torque_pitch, float& torque_yaw);

    /**
     * @brief Inverso de un cuaternión
     */
    Quaternion quaternion_inverse(const Quaternion& q);

    /**
     * @brief Producto de cuaterniones: q1 * q2
     */
    Quaternion quaternion_multiply(const Quaternion& q1, const Quaternion& q2);

    /**
     * @brief Normalizar cuaternión
     */
    void quaternion_normalize(Quaternion& q);
};

} // namespace stallion
