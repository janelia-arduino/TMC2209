#include "Registers.hpp"

namespace tmc2209
{

Registers::Registers (Device &device)
{
  bind (device);
}

void
Registers::bind (Device &device)
{
  device_ = &device;
}

bool
Registers::isBound () const
{
  return device_ != nullptr;
}

Result<uint32_t>
Registers::readRaw (uint8_t register_address) const
{
  if (device_ == nullptr)
    {
      Result<uint32_t> result;
      result.error = UartError::NotInitialized;
      return result;
    }
  return device_->readRegister (register_address);
}

Result<void>
Registers::writeRaw (uint8_t register_address, uint32_t raw_value) const
{
  if (device_ == nullptr)
    {
      Result<void> result;
      result.error = UartError::NotInitialized;
      return result;
    }
  return device_->writeRegister (register_address, raw_value);
}

Result<reg::GCONF>
Registers::readGconf () const
{
  return readTyped_<reg::GCONF> (reg_addr::GCONF);
}

Result<void>
Registers::writeGconf (const reg::GCONF &value) const
{
  return writeTyped_ (reg_addr::GCONF, value);
}

Result<reg::GSTAT>
Registers::readGstat () const
{
  return readTyped_<reg::GSTAT> (reg_addr::GSTAT);
}

Result<void>
Registers::writeGstat (const reg::GSTAT &value) const
{
  return writeTyped_ (reg_addr::GSTAT, value);
}

Result<reg::IOIN>
Registers::readIoin () const
{
  return readTyped_<reg::IOIN> (reg_addr::IOIN);
}

Result<reg::IHOLD_IRUN>
Registers::readIholdIrun () const
{
  return readTyped_<reg::IHOLD_IRUN> (reg_addr::IHOLD_IRUN);
}

Result<void>
Registers::writeIholdIrun (const reg::IHOLD_IRUN &value) const
{
  return writeTyped_ (reg_addr::IHOLD_IRUN, value);
}

Result<reg::COOLCONF>
Registers::readCoolconf () const
{
  return readTyped_<reg::COOLCONF> (reg_addr::COOLCONF);
}

Result<void>
Registers::writeCoolconf (const reg::COOLCONF &value) const
{
  return writeTyped_ (reg_addr::COOLCONF, value);
}

Result<reg::CHOPCONF>
Registers::readChopconf () const
{
  return readTyped_<reg::CHOPCONF> (reg_addr::CHOPCONF);
}

Result<void>
Registers::writeChopconf (const reg::CHOPCONF &value) const
{
  return writeTyped_ (reg_addr::CHOPCONF, value);
}

Result<reg::DRV_STATUS>
Registers::readDrvStatus () const
{
  return readTyped_<reg::DRV_STATUS> (reg_addr::DRV_STATUS);
}

Result<reg::PWMCONF>
Registers::readPwmconf () const
{
  return readTyped_<reg::PWMCONF> (reg_addr::PWMCONF);
}

Result<void>
Registers::writePwmconf (const reg::PWMCONF &value) const
{
  return writeTyped_ (reg_addr::PWMCONF, value);
}

Result<reg::PWM_SCALE>
Registers::readPwmScale () const
{
  return readTyped_<reg::PWM_SCALE> (reg_addr::PWM_SCALE);
}

Result<reg::PWM_AUTO>
Registers::readPwmAuto () const
{
  return readTyped_<reg::PWM_AUTO> (reg_addr::PWM_AUTO);
}

Result<uint8_t>
Registers::readIfcnt () const
{
  Result<uint8_t> result;
  const auto raw = readRaw (reg_addr::IFCNT);
  if (!raw.ok ())
    {
      result.error = raw.error;
      return result;
    }
  result.value = static_cast<uint8_t> (raw.value & 0xFFu);
  return result;
}

Result<uint32_t>
Registers::readTstep () const
{
  return readRaw (reg_addr::TSTEP);
}

Result<uint16_t>
Registers::readSgResult () const
{
  Result<uint16_t> result;
  const auto raw = readRaw (reg_addr::SG_RESULT);
  if (!raw.ok ())
    {
      result.error = raw.error;
      return result;
    }
  result.value = static_cast<uint16_t> (raw.value & 0x03FFu);
  return result;
}

Result<uint16_t>
Registers::readMscnt () const
{
  Result<uint16_t> result;
  const auto raw = readRaw (reg_addr::MSCNT);
  if (!raw.ok ())
    {
      result.error = raw.error;
      return result;
    }
  result.value = static_cast<uint16_t> (raw.value & 0x03FFu);
  return result;
}

} // namespace tmc2209
