#include "drivers/mpu6000_driver.hpp"
#include <Arduino.h>

namespace stallion {

MPU6000Driver::MPU6000Driver() = default;

bool MPU6000Driver::initialize() {
  // TODO: Configure SPI (MOSI, MISO, CLK, CS pins)
  // TODO: Write initialization registers to MPU6000
  // For now, just return true
  return true;
}

bool MPU6000Driver::read(IMURawData& out_data) {
  // TODO: Read from SPI and populate out_data
  // For now, return dummy values
  out_data.timestamp_us = micros();
  out_data.accel_raw = {0.0f, 0.0f, -9.81f};  // 1G down
  out_data.gyro_raw = {0.0f, 0.0f, 0.0f};
  out_data.temp_c = 25.0f;
  out_data.valid = true;
  return true;
}

}  // namespace stallion
