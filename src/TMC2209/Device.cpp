#include "Device.hpp"

namespace tmc2209
{

Device::Device (UartBus &bus, uint8_t serial_address)
{
  bind (bus, serial_address);
}

Device::Device (UartBus &bus, const UartParameters &parameters)
{
  bind (bus, parameters);
}

void
Device::bind (UartBus &bus, uint8_t serial_address)
{
  bus_ = &bus;
  parameters_.serial_address = serial_address;
}

void
Device::bind (UartBus &bus, const UartParameters &parameters)
{
  bus_ = &bus;
  parameters_ = parameters;
}

void
Device::setParameters (const UartParameters &parameters)
{
  parameters_ = parameters;
}

bool
Device::isBound () const
{
  return bus_ != nullptr;
}

uint8_t
Device::serialAddress () const
{
  return parameters_.serial_address;
}

UartParameters
Device::parameters () const
{
  return parameters_;
}

Result<uint32_t>
Device::readRegister (uint8_t register_address) const
{
  if (bus_ == nullptr)
    {
      Result<uint32_t> result;
      result.error = UartError::NotInitialized;
      return result;
    }
  return bus_->readRegister (parameters_, register_address);
}

Result<void>
Device::writeRegister (uint8_t register_address, uint32_t data) const
{
  if (bus_ == nullptr)
    {
      return notInitialized_ ();
    }
  return bus_->writeRegister (parameters_, register_address, data);
}

Result<void>
Device::startRead (uint8_t register_address) const
{
  if (bus_ == nullptr)
    {
      return notInitialized_ ();
    }
  return bus_->startRead (parameters_.serial_address, register_address);
}

Result<void>
Device::startWrite (uint8_t register_address, uint32_t data) const
{
  if (bus_ == nullptr)
    {
      return notInitialized_ ();
    }
  return bus_->startWrite (parameters_.serial_address, register_address, data);
}

void
Device::poll () const
{
  if (bus_ != nullptr)
    {
      bus_->poll ();
    }
}

bool
Device::busy () const
{
  return (bus_ != nullptr) && bus_->busy ();
}

bool
Device::resultReady () const
{
  return (bus_ != nullptr) && bus_->resultReady (parameters_.serial_address);
}

Result<uint32_t>
Device::takeReadResult () const
{
  if (bus_ == nullptr)
    {
      Result<uint32_t> result;
      result.error = UartError::NotInitialized;
      return result;
    }
  return bus_->takeReadResult (parameters_.serial_address);
}

Result<void>
Device::takeWriteResult () const
{
  if (bus_ == nullptr)
    {
      return notInitialized_ ();
    }
  return bus_->takeWriteResult (parameters_.serial_address);
}

UartError
Device::lastError () const
{
  if (bus_ == nullptr)
    {
      return UartError::NotInitialized;
    }
  return bus_->lastError ();
}

void
Device::clearLastError () const
{
  if (bus_ != nullptr)
    {
      bus_->clearLastError ();
    }
}

Result<void>
Device::notInitialized_ () const
{
  Result<void> result;
  result.error = UartError::NotInitialized;
  return result;
}

} // namespace tmc2209
