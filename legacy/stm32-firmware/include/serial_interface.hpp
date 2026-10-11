#pragma once

#include <cstdint>
#include <cstring>

namespace stallion {

class ISerialInterface {
 public:
  virtual ~ISerialInterface() = default;
  
  // Initialize serial port
  virtual bool initialize(uint32_t baudrate) = 0;
  
  // Send raw bytes
  virtual bool send(const uint8_t* data, size_t length) = 0;
  
  // Send formatted string
  virtual bool printf(const char* format, ...) = 0;
  
  // Close/shutdown
  virtual void close() = 0;
};

}  // namespace stallion
