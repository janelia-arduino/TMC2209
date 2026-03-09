#ifndef TMC2209_REGISTERS_HPP
#define TMC2209_REGISTERS_HPP

#include "Device.hpp"
#include "Result.hpp"
#include "tmc2209_registers.hpp"

namespace tmc2209
{
namespace reg_addr
{
constexpr uint8_t GCONF = 0x00;
constexpr uint8_t GSTAT = 0x01;
constexpr uint8_t IFCNT = 0x02;
constexpr uint8_t REPLYDELAY = 0x03;
constexpr uint8_t IOIN = 0x06;
constexpr uint8_t IHOLD_IRUN = 0x10;
constexpr uint8_t TPOWERDOWN = 0x11;
constexpr uint8_t TSTEP = 0x12;
constexpr uint8_t TPWMTHRS = 0x13;
constexpr uint8_t TCOOLTHRS = 0x14;
constexpr uint8_t VACTUAL = 0x22;
constexpr uint8_t SGTHRS = 0x40;
constexpr uint8_t SG_RESULT = 0x41;
constexpr uint8_t COOLCONF = 0x42;
constexpr uint8_t MSCNT = 0x6A;
constexpr uint8_t MSCURACT = 0x6B;
constexpr uint8_t CHOPCONF = 0x6C;
constexpr uint8_t DRV_STATUS = 0x6F;
constexpr uint8_t PWMCONF = 0x70;
constexpr uint8_t PWM_SCALE = 0x71;
constexpr uint8_t PWM_AUTO = 0x72;
} // namespace reg_addr

class Registers
{
public:
  Registers () = default;
  explicit Registers (Device &device);

  void bind (Device &device);
  bool isBound () const;

  Result<uint32_t> readRaw (uint8_t register_address) const;
  Result<void> writeRaw (uint8_t register_address, uint32_t raw_value) const;

  Result<reg::GCONF> readGconf () const;
  Result<void> writeGconf (const reg::GCONF &value) const;

  Result<reg::GSTAT> readGstat () const;
  Result<void> writeGstat (const reg::GSTAT &value) const;

  Result<reg::IOIN> readIoin () const;
  Result<reg::IHOLD_IRUN> readIholdIrun () const;
  Result<void> writeIholdIrun (const reg::IHOLD_IRUN &value) const;

  Result<reg::COOLCONF> readCoolconf () const;
  Result<void> writeCoolconf (const reg::COOLCONF &value) const;

  Result<reg::CHOPCONF> readChopconf () const;
  Result<void> writeChopconf (const reg::CHOPCONF &value) const;

  Result<reg::DRV_STATUS> readDrvStatus () const;

  Result<reg::PWMCONF> readPwmconf () const;
  Result<void> writePwmconf (const reg::PWMCONF &value) const;

  Result<reg::PWM_SCALE> readPwmScale () const;
  Result<reg::PWM_AUTO> readPwmAuto () const;

  Result<uint8_t> readIfcnt () const;
  Result<uint32_t> readTstep () const;
  Result<uint16_t> readSgResult () const;
  Result<uint16_t> readMscnt () const;

private:
  template <typename RegisterT>
  Result<RegisterT> readTyped_ (uint8_t register_address) const
  {
    Result<RegisterT> result;
    if (device_ == nullptr)
      {
        result.error = UartError::NotInitialized;
        return result;
      }

    const auto raw = device_->readRegister (register_address);
    if (!raw.ok ())
      {
        result.error = raw.error;
        return result;
      }

    result.value.raw = raw.value;
    result.error = UartError::None;
    return result;
  }

  template <typename RegisterT>
  Result<void> writeTyped_ (uint8_t register_address,
                            const RegisterT &value) const
  {
    if (device_ == nullptr)
      {
        Result<void> result;
        result.error = UartError::NotInitialized;
        return result;
      }
    return device_->writeRegister (register_address, value.raw);
  }

  Device *device_{ nullptr };
};

} // namespace tmc2209

#endif // TMC2209_REGISTERS_HPP
