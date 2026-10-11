#include "scheduler.hpp"
#include "quaternion_attitude_controller.hpp"
#include <Arduino.h>

namespace stallion {

CoordinadorDeTareas::CoordinadorDeTareas()
    : sensores_(std::make_unique<SensorManager>()),
      controlador_(std::make_unique<QuaternionAttitudeController>()),
      allocator_(std::make_unique<PSO_Allocator>()),
      last_update_us_(0),
      allocator_valid_(false),
      inicializado_(false) {
  setpoint_ = ControlSetpoint{};
  control_output_ = ControlOutput{};
  memset(allocator_motor_speeds_, 0, sizeof(allocator_motor_speeds_));
  memset(allocator_servo_angles_, 0, sizeof(allocator_servo_angles_));
}

CoordinadorDeTareas::~CoordinadorDeTareas() {
  detener();
}

void CoordinadorDeTareas::inicializar(uint64_t ahora_us) {
  (void)ahora_us;
  sensores_->inicializar();
  controlador_->reset();
  last_update_us_ = ahora_us;
  inicializado_ = true;
}

void CoordinadorDeTareas::ejecutar(uint64_t ahora_us) {
  if (!inicializado_) {
    return;
  }

  float dt = 0.0f;
  if (last_update_us_ != 0 && ahora_us > last_update_us_) {
    dt = static_cast<float>(ahora_us - last_update_us_) * 1e-6f;
  }
  last_update_us_ = ahora_us;

  sensores_->update(ahora_us);
  estado_actual_ = sensores_->state();

  if (controlador_->update(estado_actual_, setpoint_, dt)) {
    control_output_ = controlador_->get_output();
  }

  if (allocator_) {
    allocator_valid_ = allocator_->solve(
        control_output_.torque_roll,
        control_output_.torque_pitch,
        control_output_.torque_yaw,
        control_output_.thrust,
        dt);
    if (allocator_valid_) {
      const float* speeds = allocator_->get_motor_speeds();
      const float* angles = allocator_->get_servo_angles();
      for (int i = 0; i < 3; ++i) {
        allocator_motor_speeds_[i] = speeds[i];
        allocator_servo_angles_[i] = angles[i];
      }
    }
  }
}

void CoordinadorDeTareas::detener() {
  inicializado_ = false;
  if (controlador_) {
    controlador_->reset();
  }
}

void CoordinadorDeTareas::diagnostico() {
  Serial.print("[DIAG] SensorManager healthy=");
  Serial.println(sensores_->is_healthy() ? "YES" : "NO");

  Serial.print("[DIAG] Pose q=");
  Serial.print(estado_actual_.q.w, 4);
  Serial.print(",");
  Serial.print(estado_actual_.q.x, 4);
  Serial.print(",");
  Serial.print(estado_actual_.q.y, 4);
  Serial.print(",");
  Serial.println(estado_actual_.q.z, 4);

  Serial.print("[DIAG] Rates rad/s = ");
  Serial.print(estado_actual_.rate_roll_rads, 4);
  Serial.print(", ");
  Serial.print(estado_actual_.rate_pitch_rads, 4);
  Serial.print(", ");
  Serial.println(estado_actual_.rate_yaw_rads, 4);

  Serial.print("[DIAG] BARO P=");
  Serial.print(sensores_->baro_data().pressure_pa, 1);
  Serial.print(" Pa, T=");
  Serial.print(sensores_->baro_data().temperature_c, 2);
  Serial.print(" C, Alt=");
  Serial.print(sensores_->baro_data().altitude_m, 2);
  Serial.println(" m");

  sensores_->debug_serial();

  Serial.print("[EST] q=");
  Serial.print(estado_actual_.q.w, 4);
  Serial.print(",");
  Serial.print(estado_actual_.q.x, 4);
  Serial.print(",");
  Serial.print(estado_actual_.q.y, 4);
  Serial.print(",");
  Serial.print(estado_actual_.q.z, 4);
  Serial.print(" | RPY=");
  Serial.print(estado_actual_.euler_roll_rad * 57.2957795f, 2);
  Serial.print(",");
  Serial.print(estado_actual_.euler_pitch_rad * 57.2957795f, 2);
  Serial.print(",");
  Serial.print(estado_actual_.euler_yaw_rad * 57.2957795f, 2);
  Serial.println(" deg");

  Serial.print("[CTL] torque=");
  Serial.print(control_output_.torque_roll, 4);
  Serial.print(",");
  Serial.print(control_output_.torque_pitch, 4);
  Serial.print(",");
  Serial.print(control_output_.torque_yaw, 4);
  Serial.print(" thrust=");
  Serial.println(control_output_.thrust, 4);

  if (allocator_valid_) {
    Serial.print("[ALLOC] motors=");
    Serial.print(allocator_motor_speeds_[0], 1);
    Serial.print(",");
    Serial.print(allocator_motor_speeds_[1], 1);
    Serial.print(",");
    Serial.print(allocator_motor_speeds_[2], 1);
    Serial.print(" RPM | servos=");
    Serial.print(allocator_servo_angles_[0], 4);
    Serial.print(",");
    Serial.print(allocator_servo_angles_[1], 4);
    Serial.print(",");
    Serial.println(allocator_servo_angles_[2], 4);
  } else {
    Serial.println("[ALLOC] invalid");
  }
}

void CoordinadorDeTareas::set_attitude_setpoint(float roll_rad, float pitch_rad, float yaw_rad, float thrust) {
  setpoint_.roll_rad = roll_rad;
  setpoint_.pitch_rad = pitch_rad;
  setpoint_.yaw_rad = yaw_rad;
  setpoint_.thrust = thrust;
}

}  // namespace stallion
