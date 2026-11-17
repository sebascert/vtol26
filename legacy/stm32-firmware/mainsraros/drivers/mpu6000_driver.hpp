#include "imu_driver.hpp"

namespace stallion {

class MPU6000Driver : public IIMUDriver {
 public:
  MPU6000Driver();
  ~MPU6000Driver() override = default;

  bool initialize() override;
  bool read(IMURawData& out_data) override;
  uint16_t sample_rate_hz() const override { return 200; }

 private:
  // TODO: SPI configuration and read registers
  uint64_t last_read_us_{0};
};

}  // namespace stallion
