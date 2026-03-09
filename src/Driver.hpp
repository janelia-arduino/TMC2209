#ifndef TMC2209_DRIVER_HPP
#define TMC2209_DRIVER_HPP

#include <Arduino.h>

#include "Registers.hpp"
#include "Result.hpp"

namespace tmc2209
{

class Driver
{
public:
  Driver () = default;
  Driver (Device &device, Registers &registers);

  void bind (Device &device, Registers &registers);
  bool isBound () const;

  Result<void> initialize ();

  Result<void> enable ();
  Result<void> disable ();

  Result<void> setMicrostepsPerStep (uint16_t microsteps_per_step);
  Result<uint16_t> getMicrostepsPerStep () const;

  Result<void> setRunCurrent (uint8_t percent);
  Result<void> setHoldCurrent (uint8_t percent);
  Result<void> setHoldDelay (uint8_t percent);
  Result<void> setAllCurrentValues (uint8_t run_current_percent,
                                    uint8_t hold_current_percent,
                                    uint8_t hold_delay_percent);
  Result<void> setRMSCurrent (uint16_t mA,
                              float rSense = 0.11f,
                              float holdMultiplier = 0.5f);

  Result<void> enableInverseMotorDirection ();
  Result<void> disableInverseMotorDirection ();

  Result<void> enableStealthChop ();
  Result<void> disableStealthChop ();

  Result<void> enableAutomaticCurrentScaling ();
  Result<void> disableAutomaticCurrentScaling ();
  Result<void> enableAutomaticGradientAdaptation ();
  Result<void> disableAutomaticGradientAdaptation ();

  Result<void> setPwmOffset (uint8_t pwm_offset);
  Result<void> setPwmGradient (uint8_t pwm_gradient);

  Result<uint8_t> getVersion () const;

private:
  static constexpr uint8_t PERCENT_MIN = 0;
  static constexpr uint8_t PERCENT_MAX = 100;
  static constexpr uint8_t CURRENT_SETTING_MIN = 0;
  static constexpr uint8_t CURRENT_SETTING_MAX = 31;
  static constexpr uint8_t HOLD_DELAY_MIN = 0;
  static constexpr uint8_t HOLD_DELAY_MAX = 15;

  static constexpr uint8_t IHOLD_DEFAULT = 16;
  static constexpr uint8_t IRUN_DEFAULT = 31;
  static constexpr uint8_t IHOLDDELAY_DEFAULT = 1;

  static constexpr uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053u;
  static constexpr uint8_t TBL_DEFAULT = 0b10;
  static constexpr uint8_t HEND_DEFAULT = 0;
  static constexpr uint8_t HSTART_DEFAULT = 5;
  static constexpr uint8_t TOFF_DEFAULT = 3;
  static constexpr uint8_t TOFF_DISABLE = 0;
  static constexpr uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024u;
  static constexpr uint32_t COOLCONF_DEFAULT = 0u;
  static constexpr uint8_t TPOWERDOWN_DEFAULT = 20u;
  static constexpr uint32_t TPWMTHRS_DEFAULT = 0u;
  static constexpr int32_t VACTUAL_DEFAULT = 0;
  static constexpr uint32_t TCOOLTHRS_DEFAULT = 0u;
  static constexpr uint32_t SGTHRS_DEFAULT = 0u;
  static constexpr uint8_t VERSION = 0x21u;

  Result<void> notInitialized_ () const;
  uint8_t percentToCurrentSetting_ (uint8_t percent) const;
  uint8_t percentToHoldDelaySetting_ (uint8_t percent) const;
  reg::Mres microstepsToMres_ (uint16_t microsteps_per_step) const;
  uint16_t mresToMicrosteps_ (reg::Mres mres) const;

  Device *device_{ nullptr };
  Registers *registers_{ nullptr };
  uint8_t toff_{ TOFF_DEFAULT };
};

} // namespace tmc2209

#endif // TMC2209_DRIVER_HPP
