#include "pso_allocator.hpp"
#include <cstring>
#include <cmath>
#include <Arduino.h>

namespace stallion {

PSO_Allocator::PSO_Allocator()
    : is_valid_(false),
      motor_min_rpm_(1000.0f),
      motor_max_rpm_(8000.0f),
      servo_min_rad_(-0.349f),  // -20 grados
      servo_max_rad_(0.349f) {  // +20 grados
    memset(motor_speeds_, 0, sizeof(motor_speeds_));
    memset(servo_angles_, 0, sizeof(servo_angles_));
}

bool PSO_Allocator::solve(float torque_roll, float torque_pitch, float torque_yaw,
                          float thrust, float dt) {
    if (dt <= 0) {
        is_valid_ = false;
        return false;
    }

    // Usar solver simple por ahora
    // En futuro, esto llamará al PSO real
    bool success = solve_simple(torque_roll, torque_pitch, torque_yaw, thrust);
    is_valid_ = success;
    return success;
}

bool PSO_Allocator::solve_simple(float torque_roll, float torque_pitch, float torque_yaw,
                                 float thrust) {
    /**
     * Estrategia simple de allocación para tricóptero VTOL tiltrotor:
     * 
     * - Motor 0: Frontal (norte)
     * - Motor 1: Trasero-Izq (suroeste)
     * - Motor 2: Trasero-Der (sureste)
     * 
     * Configuración de simetría:
     * - Motor 0 genera thrust principalmente vertical (eje Z)
     * - Motores 1 y 2 generan thrust con componentes en X, Y, Z
     * - Servos tiltrotor ajustan ángulos para generar torques
     */

    // Thrust base distribuido
    // En un tricóptero simétrico: ~1/3 por motor en hover
    float thrust_base = thrust / 3.0f;

    // Ganancias de acoplamiento
    const float K_ROLL = 0.5f;
    const float K_PITCH = 0.5f;
    const float K_YAW = 0.3f;

    // Calcular velocidades base (mapear thrust a RPM)
    // Relación lineal simplificada: RPM = (thrust * 5000) + 1000
    float motor_cmd[3];
    motor_cmd[0] = motor_min_rpm_ + (thrust_base * (motor_max_rpm_ - motor_min_rpm_));
    motor_cmd[1] = motor_min_rpm_ + (thrust_base * (motor_max_rpm_ - motor_min_rpm_));
    motor_cmd[2] = motor_min_rpm_ + (thrust_base * (motor_max_rpm_ - motor_min_rpm_));

    // Modular velocidades con torques
    // Roll: diferenciar motores 1 y 2 (alas)
    motor_cmd[1] += K_ROLL * torque_roll * 1000.0f;
    motor_cmd[2] -= K_ROLL * torque_roll * 1000.0f;

    // Pitch: combinar todos los motores
    motor_cmd[0] -= K_PITCH * torque_pitch * 800.0f;
    motor_cmd[1] += K_PITCH * torque_pitch * 400.0f;
    motor_cmd[2] += K_PITCH * torque_pitch * 400.0f;

    // Yaw: copter style - rotor 0 vs 1,2 contrarotadores
    motor_cmd[0] += K_YAW * torque_yaw * 1000.0f;
    motor_cmd[1] -= K_YAW * torque_yaw * 500.0f;
    motor_cmd[2] -= K_YAW * torque_yaw * 500.0f;

    // Limitar velocidades
    for (int i = 0; i < 3; i++) {
        motor_cmd[i] = fmaxf(motor_min_rpm_, fminf(motor_max_rpm_, motor_cmd[i]));
    }

    // Ángulos de servo tiltrotor
    // Los servos se ajustan según los torques para mejorar coupling
    servo_angles_[0] = 0.0f;  // Motor frontal sin tilt (o minimal)
    servo_angles_[1] = K_PITCH * torque_pitch * 0.1f;  // Tilt trasero-izq
    servo_angles_[2] = K_PITCH * torque_pitch * 0.1f;  // Tilt trasero-der

    // Limitar ángulos
    for (int i = 0; i < 3; i++) {
        servo_angles_[i] = fmaxf(servo_min_rad_, fminf(servo_max_rad_, servo_angles_[i]));
        motor_speeds_[i] = motor_cmd[i];
    }

    return true;
}

void PSO_Allocator::mix_quadrotor_style(float torque_roll, float torque_pitch,
                                        float torque_yaw, float thrust) {
    /**
     * Mixer X-quad clásico (solo con fines de referencia)
     * Formato: [NW, NE, SE, SW]
     * 
     * En tricóptero no se usa, pero se deja para debugging
     */
    float cmd[4] = {
        thrust - torque_roll - torque_pitch + torque_yaw,
        thrust + torque_roll - torque_pitch - torque_yaw,
        thrust + torque_roll + torque_pitch + torque_yaw,
        thrust - torque_roll + torque_pitch - torque_yaw
    };

    // Mapear a RPM (solo para debug)
    const float MIN_CMD = 1000.0f;
    const float MAX_CMD = 8000.0f;
    for (int i = 0; i < 4; i++) {
        cmd[i] = MIN_CMD + (cmd[i] + 0.5f) * (MAX_CMD - MIN_CMD);
    }
}

void PSO_Allocator::reset() {
    memset(motor_speeds_, 0, sizeof(motor_speeds_));
    memset(servo_angles_, 0, sizeof(servo_angles_));
    is_valid_ = false;
}

} // namespace stallion
