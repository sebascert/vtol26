#include <Arduino.h>
#include "serial_interface.hpp"

namespace stallion {

class ArduinoSerialImpl : public ISerialInterface {
 public:
  bool initialize(uint32_t baudrate) override;
  bool send(const uint8_t* data, size_t length) override;
  bool printf(const char* format, ...) override;
  void close() override;
};

bool ArduinoSerialImpl::initialize(uint32_t baudrate) {
  Serial.begin(baudrate);
  delay(100);  // Wait for Serial to stabilize
  return true;
}

bool ArduinoSerialImpl::send(const uint8_t* data, size_t length) {
  if (!data || length == 0) return false;
  Serial.write(data, length);
  return true;
}

bool ArduinoSerialImpl::printf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  if (len > 0 && len < (int)sizeof(buffer)) {
    Serial.write((uint8_t*)buffer, len);
    return true;
  }
  return false;
}

void ArduinoSerialImpl::close() {
  Serial.end();
}

}  // namespace stallion
