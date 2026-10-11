#pragma once

#include <cstdint>
#include <memory>
#include "module_data_types.hpp"
#include "module_interfaces.hpp"
#include "pso_allocator.hpp"
#include "sensor_manager.hpp"

namespace stallion {

class CoordinadorDeTareas {
 public:
  CoordinadorDeTareas();
  ~CoordinadorDeTareas();

  void inicializar(uint64_t ahora_us);
  void ejecutar(uint64_t ahora_us);
  void detener();
  void diagnostico();
  void set_attitude_setpoint(float roll_rad, float pitch_rad, float yaw_rad, float thrust);

 private:
  std::unique_ptr<SensorManager> sensores_;
  std::unique_ptr<IAttitudeController> controlador_;
  std::unique_ptr<PSO_Allocator> allocator_;

  StateEstimate estado_actual_;
  ControlSetpoint setpoint_;
  ControlOutput control_output_;
  bool allocator_valid_;
  float allocator_motor_speeds_[3];
  float allocator_servo_angles_[3];
  uint64_t last_update_us_;
  bool inicializado_;
};

} // namespace stallion
