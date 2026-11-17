#include "module_data_types.hpp"

int main() {
  stallion::HilFrame frame {};
  frame.seq = 1;
  frame.valid = true;
  return (frame.valid && frame.seq == 1) ? 0 : 1;
}
