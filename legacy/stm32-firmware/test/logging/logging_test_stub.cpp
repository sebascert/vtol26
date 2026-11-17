#include "module_data_types.hpp"

int main() {
  stallion::LogSample sample {};
  sample.timestamp_us = 123456;
  sample.failsafe.active = false;
  return (sample.timestamp_us > 0) ? 0 : 1;
}
