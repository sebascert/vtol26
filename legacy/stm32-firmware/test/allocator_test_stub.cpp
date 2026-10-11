#include "module_data_types.hpp"

int main() {
  stallion::AllocatorOutput out {};
  out.motor_cmd[0] = 0.4f;
  out.motor_cmd[1] = 0.4f;
  out.motor_cmd[2] = 0.4f;
  return out.saturated ? 1 : 0;
}
