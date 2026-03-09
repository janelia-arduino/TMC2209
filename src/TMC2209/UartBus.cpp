#include "UartBus.hpp"

namespace tmc2209
{

UartBus::UartBus ()
  : hardware_serial_ptr_ (nullptr),
    last_uart_error_ (UartError::None),
    active_serial_address_ (INVALID_SERIAL_ADDRESS),
    completed_serial_address_ (INVALID_SERIAL_ADDRESS)
{
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  software_serial_ptr_ = nullptr;
#endif
  uart_engine_.attach (this);
}

void
UartBus::setParameters (const UartBusParameters &parameters)
{
  parameters_ = parameters;
}

UartBusParameters
UartBus::parameters () const
{
  return parameters_;
}

#if !defined(ARDUINO_ARCH_RENESAS)
void
UartBus::setup (HardwareSerial &serial)
{
  hardware_serial_ptr_ = &serial;
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  software_serial_ptr_ = nullptr;
#endif
  uart_engine_.reset ();
  last_uart_error_ = UartError::None;
  active_serial_address_ = INVALID_SERIAL_ADDRESS;
  completed_serial_address_ = INVALID_SERIAL_ADDRESS;
}
#endif

#if defined(ESP32)
void
UartBus::setup (HardwareSerial &serial)
{
  hardware_serial_ptr_ = &serial;
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  software_serial_ptr_ = nullptr;
#endif
  uart_engine_.reset ();
  last_uart_error_ = UartError::None;
  active_serial_address_ = INVALID_SERIAL_ADDRESS;
  completed_serial_address_ = INVALID_SERIAL_ADDRESS;
}
#elif defined(ARDUINO_ARCH_RP2040)
void
UartBus::setup (SerialUART &serial)
{
  hardware_serial_ptr_ = &serial;
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  software_serial_ptr_ = nullptr;
#endif
  uart_engine_.reset ();
  last_uart_error_ = UartError::None;
  active_serial_address_ = INVALID_SERIAL_ADDRESS;
  completed_serial_address_ = INVALID_SERIAL_ADDRESS;
}
#elif defined(ARDUINO_ARCH_RENESAS)
void
UartBus::setup (UART &serial)
{
  hardware_serial_ptr_ = &serial;
  uart_engine_.reset ();
  last_uart_error_ = UartError::None;
  active_serial_address_ = INVALID_SERIAL_ADDRESS;
  completed_serial_address_ = INVALID_SERIAL_ADDRESS;
}
#endif

#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
void
UartBus::setup (SoftwareSerial &serial)
{
  software_serial_ptr_ = &serial;
  hardware_serial_ptr_ = nullptr;
  uart_engine_.reset ();
  last_uart_error_ = UartError::None;
  active_serial_address_ = INVALID_SERIAL_ADDRESS;
  completed_serial_address_ = INVALID_SERIAL_ADDRESS;
}
#endif

bool
UartBus::isConfigured () const
{
  return serialTransportConfigured ();
}

Result<uint32_t>
UartBus::readRegister (uint8_t serial_address, uint8_t register_address)
{
  Result<uint32_t> result;

  const auto start_result = startRead (serial_address, register_address);
  if (!start_result.ok ())
    {
      result.error = start_result.error;
      return result;
    }

  while (!resultReady (serial_address))
    {
      poll ();
      if (!resultReady (serial_address))
        {
          delayMicroseconds (1);
        }
    }

  result = takeReadResult (serial_address);
  last_uart_error_ = result.error;
  return result;
}

Result<void>
UartBus::writeRegister (uint8_t serial_address,
                        uint8_t register_address,
                        uint32_t data)
{
  Result<void> result;

  const auto start_result = startWrite (serial_address, register_address, data);
  if (!start_result.ok ())
    {
      result.error = start_result.error;
      return result;
    }

  while (!resultReady (serial_address))
    {
      poll ();
      if (!resultReady (serial_address))
        {
          delayMicroseconds (1);
        }
    }

  result = takeWriteResult (serial_address);
  last_uart_error_ = result.error;
  return result;
}

Result<void>
UartBus::startRead (uint8_t serial_address, uint8_t register_address)
{
  Result<void> result;

  if (!serialTransportConfigured ())
    {
      result.error = UartError::NotInitialized;
      last_uart_error_ = result.error;
      return result;
    }

  result = uart_engine_.startRead (serial_address, register_address);
  if (result.ok ())
    {
      active_serial_address_ = serial_address;
      completed_serial_address_ = INVALID_SERIAL_ADDRESS;
    }
  last_uart_error_ = result.error;
  return result;
}

Result<void>
UartBus::startWrite (uint8_t serial_address,
                     uint8_t register_address,
                     uint32_t data)
{
  Result<void> result;

  if (!serialTransportConfigured ())
    {
      result.error = UartError::NotInitialized;
      last_uart_error_ = result.error;
      return result;
    }

  result = uart_engine_.startWrite (serial_address, register_address, data);
  if (result.ok ())
    {
      active_serial_address_ = serial_address;
      completed_serial_address_ = INVALID_SERIAL_ADDRESS;
    }
  last_uart_error_ = result.error;
  return result;
}

void
UartBus::poll ()
{
  const bool was_ready = uart_engine_.resultReady ();
  uart_engine_.poll ();
  if ((!was_ready) && uart_engine_.resultReady ())
    {
      last_uart_error_ = uart_engine_.lastError ();
      completed_serial_address_ = active_serial_address_;
    }
}

bool
UartBus::busy () const
{
  return uart_engine_.busy ();
}

bool
UartBus::busy (uint8_t serial_address) const
{
  return uart_engine_.busy () && (active_serial_address_ == serial_address);
}

bool
UartBus::resultReady () const
{
  return uart_engine_.resultReady ();
}

bool
UartBus::resultReady (uint8_t serial_address) const
{
  return uart_engine_.resultReady ()
         && (completed_serial_address_ == serial_address);
}

Result<uint32_t>
UartBus::takeReadResult ()
{
  Result<uint32_t> result = uart_engine_.takeReadResult ();
  last_uart_error_ = result.error;
  if (!uart_engine_.resultReady ())
    {
      completed_serial_address_ = INVALID_SERIAL_ADDRESS;
      active_serial_address_ = INVALID_SERIAL_ADDRESS;
    }
  return result;
}

Result<uint32_t>
UartBus::takeReadResult (uint8_t serial_address)
{
  if (!resultReady (serial_address))
    {
      Result<uint32_t> result;
      result.error = UartError::Busy;
      return result;
    }
  return takeReadResult ();
}

Result<void>
UartBus::takeWriteResult ()
{
  Result<void> result = uart_engine_.takeWriteResult ();
  last_uart_error_ = result.error;
  if (!uart_engine_.resultReady ())
    {
      completed_serial_address_ = INVALID_SERIAL_ADDRESS;
      active_serial_address_ = INVALID_SERIAL_ADDRESS;
    }
  return result;
}

Result<void>
UartBus::takeWriteResult (uint8_t serial_address)
{
  if (!resultReady (serial_address))
    {
      Result<void> result;
      result.error = UartError::Busy;
      return result;
    }
  return takeWriteResult ();
}

UartError
UartBus::lastError () const
{
  return last_uart_error_;
}

void
UartBus::clearLastError ()
{
  last_uart_error_ = UartError::None;
}

bool
UartBus::serialTransportConfigured () const
{
  return (hardware_serial_ptr_ != nullptr)
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
         || (software_serial_ptr_ != nullptr)
#endif
      ;
}

int
UartBus::serialAvailable ()
{
  if (hardware_serial_ptr_ != nullptr)
    {
      return hardware_serial_ptr_->available ();
    }
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  if (software_serial_ptr_ != nullptr)
    {
      return software_serial_ptr_->available ();
    }
#endif
  return 0;
}

int
UartBus::serialRead ()
{
  if (hardware_serial_ptr_ != nullptr)
    {
      return hardware_serial_ptr_->read ();
    }
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  if (software_serial_ptr_ != nullptr)
    {
      return software_serial_ptr_->read ();
    }
#endif
  return -1;
}

size_t
UartBus::serialWrite (uint8_t c)
{
  if (hardware_serial_ptr_ != nullptr)
    {
      return hardware_serial_ptr_->write (c);
    }
#if TMC2209_UARTBUS_SOFTWARE_SERIAL_INCLUDED
  if (software_serial_ptr_ != nullptr)
    {
      return software_serial_ptr_->write (c);
    }
#endif
  return 0;
}

void
UartBus::serialFlush ()
{
  if (hardware_serial_ptr_ != nullptr)
    {
      hardware_serial_ptr_->flush ();
      return;
    }
}

int
UartBus::uartAvailable ()
{
  return serialAvailable ();
}

int
UartBus::uartRead ()
{
  return serialRead ();
}

size_t
UartBus::uartWrite (uint8_t c)
{
  return serialWrite (c);
}

void
UartBus::uartFlush ()
{
  serialFlush ();
}

} // namespace tmc2209
