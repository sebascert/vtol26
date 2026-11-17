#pragma once

#include "module_data_types.hpp"
#include <cstdint>

namespace stallion {

/**
 * @brief Allocator de control PSO para VTOL tricóptero
 * 
 * Recibe:
 * - torque_roll (Nm)
 * - torque_pitch (Nm)
 * - torque_yaw (Nm)
 * - thrust (N o normalizado)
 * 
 * Resuelve el problema de asignación de control usando PSO para encontrar:
 * - Velocidades de motores (RPM)
 * - Ángulos de los servos tiltrotor
 * - Desviaciones de superficies de control
 * 
 * El PSO es un solver de optimización que distribuye los comandos
 * entre los actuadores de forma óptima.
 */
class PSO_Allocator {
public:
    PSO_Allocator();
    ~PSO_Allocator() = default;

    /**
     * @brief Resolver asignación de control
     * @param torque_roll Torque deseado en roll (Nm)
     * @param torque_pitch Torque deseado en pitch (Nm)
     * @param torque_yaw Torque deseado en yaw (Nm)
     * @param thrust Empuje deseado (N)
     * @param dt Tiempo desde última actualización
     * @return true si se resolvió correctamente
     */
    bool solve(float torque_roll, float torque_pitch, float torque_yaw, 
               float thrust, float dt);

    /**
     * @brief Obtener velocidades de motores (RPM)
     */
    const float* get_motor_speeds() const { return motor_speeds_; }

    /**
     * @brief Obtener ángulos de servo tiltrotor
     * Formato: [motor_0_angle, motor_1_angle, motor_2_angle] en radianes
     */
    const float* get_servo_angles() const { return servo_angles_; }

    /**
     * @brief Obtener estado de validez
     */
    bool is_valid() const { return is_valid_; }

    /**
     * @brief Resetear allocator
     */
    void reset();

    /**
     * @brief Establecer límites de velocidad de motor
     */
    void set_motor_speed_limits(float min_rpm, float max_rpm) {
        motor_min_rpm_ = min_rpm;
        motor_max_rpm_ = max_rpm;
    }

    /**
     * @brief Establecer límites de ángulo de servo
     */
    void set_servo_angle_limits(float min_rad, float max_rad) {
        servo_min_rad_ = min_rad;
        servo_max_rad_ = max_rad;
    }

private:
    // Salidas
    float motor_speeds_[3];  // RPM para 3 motores (tricóptero)
    float servo_angles_[3];  // Ángulos en radianes para 3 servos tiltrotor
    bool is_valid_;

    // Límites operacionales
    float motor_min_rpm_;
    float motor_max_rpm_;
    float servo_min_rad_;
    float servo_max_rad_;

    /**
     * @brief Solver PSO simplificado para fases de desarrollo
     * En versión final, esto llamará al PSO real del sistema
     */
    bool solve_simple(float torque_roll, float torque_pitch, float torque_yaw, 
                      float thrust);

    /**
     * @brief Mixer clásico de cuatrirrotor (para referencia, no se usa en VTOL)
     * Se mantiene para debugging
     */
    void mix_quadrotor_style(float torque_roll, float torque_pitch, 
                             float torque_yaw, float thrust);
};

} // namespace stallion
