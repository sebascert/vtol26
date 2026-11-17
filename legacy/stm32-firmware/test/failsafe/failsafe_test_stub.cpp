#include "module_data_types.hpp"

int main() {
  stallion::FailsafeStatus fs {};
  fs.active = true;
  fs.level = stallion::FailsafeLevel::LevelHold;
  fs.reason = stallion::FailsafeReason::RcLoss;
  return (fs.active && fs.level == stallion::FailsafeLevel::LevelHold) ? 0 : 1;
}
