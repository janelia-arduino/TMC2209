#ifndef TMC2209_UART_BUS_HPP
#define TMC2209_UART_BUS_HPP

#include <Arduino.h>

#include "Result.hpp"
#include "TMC2209/UartEngine.hpp"
#include "UartBusParameters.hpp"

#if !defined(ESP32) && !defined(ARDUINO_ARCH_SAMD) && !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_SAM_DUE) && !defined(ARDUINO_ARCH_RENESAS)
#define TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED true
#else
#define TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED false
#endif
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
#include <SoftwareSerial.h>
#endif

namespace tmc2209
{

class UartBus : private UartEngineIo
{
public:
  UartBus ();

  void setParameters (const UartBusParameters &parameters);
  UartBusParameters parameters () const;

#if !defined(ARDUINO_ARCH_RENESAS)
  void setup (HardwareSerial &serial);
#endif
#if defined(ESP32)
  void setup (HardwareSerial &serial);
#elif defined(ARDUINO_ARCH_RP2040)
  void setup (SerialUART &serial);
#elif defined(ARDUINO_ARCH_RENESAS)
  void setup (UART &serial);
#endif

#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  void setup (SoftwareSerial &serial);
#endif

  bool isConfigured () const;

  Result<uint32_t> readRegister (uint8_t serial_address,
                                 uint8_t register_address);
  Result<void> writeRegister (uint8_t serial_address,
                              uint8_t register_address,
                              uint32_t data);

  Result<void> startRead (uint8_t serial_address, uint8_t register_address);
  Result<void> startWrite (uint8_t serial_address,
                           uint8_t register_address,
                           uint32_t data);
  void poll ();
  bool busy () const;
  bool busy (uint8_t serial_address) const;
  bool resultReady () const;
  bool resultReady (uint8_t serial_address) const;
  Result<uint32_t> takeReadResult ();
  Result<uint32_t> takeReadResult (uint8_t serial_address);
  Result<void> takeWriteResult ();
  Result<void> takeWriteResult (uint8_t serial_address);

  UartError lastError () const;
  void clearLastError ();

private:
  static constexpr uint8_t INVALID_SERIAL_ADDRESS = 0xFFu;

  bool serialTransportConfigured () const;
  int serialAvailable ();
  int serialRead ();
  size_t serialWrite (uint8_t c);
  void serialFlush ();

  int uartAvailable () override;
  int uartRead () override;
  size_t uartWrite (uint8_t c) override;
  void uartFlush () override;

  HardwareSerial *hardware_serial_ptr_;
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  SoftwareSerial *software_serial_ptr_;
#endif

  UartBusParameters parameters_;
  UartError last_uart_error_;
  UartEngine uart_engine_;
  uint8_t active_serial_address_;
  uint8_t completed_serial_address_;
};

} // namespace tmc2209

#endif // TMC2209_UART_BUS_HPP
