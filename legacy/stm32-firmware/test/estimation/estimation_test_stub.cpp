#include "module_data_types.hpp"

int main() {
  stallion::StateEstimate s {};
  s.timestamp_us = 1000;
  s.imu_valid = true;
  s.airspeed_valid = true;
  return (s.imu_valid && s.airspeed_valid) ? 0 : 1;
}
