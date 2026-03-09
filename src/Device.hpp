#ifndef TMC2209_DEVICE_HPP
#define TMC2209_DEVICE_HPP

#include "Result.hpp"
#include "UartBus.hpp"
#include "UartParameters.hpp"

namespace tmc2209
{

class Device
{
public:
  Device () = default;
  Device (UartBus &bus, uint8_t serial_address);
  Device (UartBus &bus, const UartParameters &parameters);

  void bind (UartBus &bus, uint8_t serial_address);
  void bind (UartBus &bus, const UartParameters &parameters);

  bool isBound () const;
  uint8_t serialAddress () const;
  UartParameters parameters () const;

  Result<uint32_t> readRegister (uint8_t register_address) const;
  Result<void> writeRegister (uint8_t register_address, uint32_t data) const;

  Result<void> startRead (uint8_t register_address) const;
  Result<void> startWrite (uint8_t register_address, uint32_t data) const;
  void poll () const;
  bool busy () const;
  bool resultReady () const;
  Result<uint32_t> takeReadResult () const;
  Result<void> takeWriteResult () const;

  UartError lastError () const;
  void clearLastError () const;

private:
  Result<void> notInitialized_ () const;

  UartBus *bus_{ nullptr };
  UartParameters parameters_{};
};

} // namespace tmc2209

#endif // TMC2209_DEVICE_HPP
