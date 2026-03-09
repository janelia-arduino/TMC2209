#include "Driver.hpp"

namespace tmc2209
{

Driver::Driver (Device &device, Registers &registers)
{
  bind (device, registers);
}

void
Driver::bind (Device &device, Registers &registers)
{
  device_ = &device;
  registers_ = &registers;
}

bool
Driver::isBound () const
{
  return (device_ != nullptr) && (registers_ != nullptr);
}

Result<void>
Driver::initialize ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  reg::GCONF gconf;
  gconf.raw = 0u;
  gconf.i_scale_analog (false);
  gconf.pdn_disable (true);
  gconf.mstep_reg_select (true);
  gconf.multistep_filt (true);
  auto result = registers_->writeGconf (gconf);
  if (!result.ok ())
    {
      return result;
    }

  reg::IHOLD_IRUN ihold_irun;
  ihold_irun.raw = 0u;
  ihold_irun.ihold (IHOLD_DEFAULT);
  ihold_irun.irun (IRUN_DEFAULT);
  ihold_irun.iholddelay (IHOLDDELAY_DEFAULT);
  result = registers_->writeIholdIrun (ihold_irun);
  if (!result.ok ())
    {
      return result;
    }

  reg::CHOPCONF chopconf;
  chopconf.raw = CHOPPER_CONFIG_DEFAULT;
  chopconf.tbl (TBL_DEFAULT);
  chopconf.hend (HEND_DEFAULT);
  chopconf.hstart (HSTART_DEFAULT);
  chopconf.toff (TOFF_DEFAULT);
  toff_ = TOFF_DEFAULT;
  result = registers_->writeChopconf (chopconf);
  if (!result.ok ())
    {
      return result;
    }

  reg::PWMCONF pwmconf;
  pwmconf.raw = PWM_CONFIG_DEFAULT;
  result = registers_->writePwmconf (pwmconf);
  if (!result.ok ())
    {
      return result;
    }

  result = registers_->writeRaw (reg_addr::COOLCONF, COOLCONF_DEFAULT);
  if (!result.ok ())
    {
      return result;
    }

  result = registers_->writeRaw (reg_addr::TPOWERDOWN, TPOWERDOWN_DEFAULT);
  if (!result.ok ())
    {
      return result;
    }
  result = registers_->writeRaw (reg_addr::TPWMTHRS, TPWMTHRS_DEFAULT);
  if (!result.ok ())
    {
      return result;
    }
  result = registers_->writeRaw (reg_addr::VACTUAL,
                                 static_cast<uint32_t> (VACTUAL_DEFAULT));
  if (!result.ok ())
    {
      return result;
    }
  result = registers_->writeRaw (reg_addr::TCOOLTHRS, TCOOLTHRS_DEFAULT);
  if (!result.ok ())
    {
      return result;
    }
  result = registers_->writeRaw (reg_addr::SGTHRS, SGTHRS_DEFAULT);
  if (!result.ok ())
    {
      return result;
    }

  reg::GSTAT gstat;
  gstat.raw = 0u;
  gstat.drv_err (true);
  result = registers_->writeGstat (gstat);
  if (!result.ok ())
    {
      return result;
    }

  ihold_irun.ihold (0u);
  ihold_irun.irun (0u);
  result = registers_->writeIholdIrun (ihold_irun);
  if (!result.ok ())
    {
      return result;
    }

  chopconf.toff (TOFF_DISABLE);
  result = registers_->writeChopconf (chopconf);
  if (!result.ok ())
    {
      return result;
    }

  pwmconf.pwm_autoscale (false);
  pwmconf.pwm_autograd (false);
  return registers_->writePwmconf (pwmconf);
}

Result<void>
Driver::enable ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto chopconf = registers_->readChopconf ();
  if (!chopconf.ok ())
    {
      Result<void> result;
      result.error = chopconf.error;
      return result;
    }

  chopconf.value.toff (toff_);
  return registers_->writeChopconf (chopconf.value);
}

Result<void>
Driver::disable ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto chopconf = registers_->readChopconf ();
  if (!chopconf.ok ())
    {
      Result<void> result;
      result.error = chopconf.error;
      return result;
    }

  if (chopconf.value.toff () > 0u)
    {
      toff_ = static_cast<uint8_t> (chopconf.value.toff ());
    }
  chopconf.value.toff (TOFF_DISABLE);
  return registers_->writeChopconf (chopconf.value);
}

Result<void>
Driver::setMicrostepsPerStep (uint16_t microsteps_per_step)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto chopconf = registers_->readChopconf ();
  if (!chopconf.ok ())
    {
      Result<void> result;
      result.error = chopconf.error;
      return result;
    }

  chopconf.value.mres (microstepsToMres_ (microsteps_per_step));
  return registers_->writeChopconf (chopconf.value);
}

Result<uint16_t>
Driver::getMicrostepsPerStep () const
{
  Result<uint16_t> result;
  if (!isBound ())
    {
      result.error = UartError::NotInitialized;
      return result;
    }

  const auto chopconf = registers_->readChopconf ();
  if (!chopconf.ok ())
    {
      result.error = chopconf.error;
      return result;
    }

  result.value = mresToMicrosteps_ (chopconf.value.mres ());
  return result;
}

Result<void>
Driver::setRunCurrent (uint8_t percent)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto current = registers_->readIholdIrun ();
  if (!current.ok ())
    {
      Result<void> result;
      result.error = current.error;
      return result;
    }

  current.value.irun (percentToCurrentSetting_ (percent));
  return registers_->writeIholdIrun (current.value);
}

Result<void>
Driver::setHoldCurrent (uint8_t percent)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto current = registers_->readIholdIrun ();
  if (!current.ok ())
    {
      Result<void> result;
      result.error = current.error;
      return result;
    }

  current.value.ihold (percentToCurrentSetting_ (percent));
  return registers_->writeIholdIrun (current.value);
}

Result<void>
Driver::setHoldDelay (uint8_t percent)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto current = registers_->readIholdIrun ();
  if (!current.ok ())
    {
      Result<void> result;
      result.error = current.error;
      return result;
    }

  current.value.iholddelay (percentToHoldDelaySetting_ (percent));
  return registers_->writeIholdIrun (current.value);
}

Result<void>
Driver::setAllCurrentValues (uint8_t run_current_percent,
                             uint8_t hold_current_percent,
                             uint8_t hold_delay_percent)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto current = registers_->readIholdIrun ();
  if (!current.ok ())
    {
      Result<void> result;
      result.error = current.error;
      return result;
    }

  current.value.irun (percentToCurrentSetting_ (run_current_percent));
  current.value.ihold (percentToCurrentSetting_ (hold_current_percent));
  current.value.iholddelay (percentToHoldDelaySetting_ (hold_delay_percent));
  return registers_->writeIholdIrun (current.value);
}

Result<void>
Driver::setRMSCurrent (uint16_t mA, float rSense, float holdMultiplier)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }

  auto chopconf = registers_->readChopconf ();
  if (!chopconf.ok ())
    {
      Result<void> result;
      result.error = chopconf.error;
      return result;
    }

  auto current = registers_->readIholdIrun ();
  if (!current.ok ())
    {
      Result<void> result;
      result.error = current.error;
      return result;
    }

  auto clampCurrentScale = [] (int value) -> uint8_t {
    if (value < 0)
      {
        return 0;
      }
    if (value > 31)
      {
        return 31;
      }
    return static_cast<uint8_t> (value);
  };

  int current_scale = static_cast<int> (
      32.0f * 1.41421f * mA / 1000.0f * (rSense + 0.02f) / 0.325f - 1.0f);

  if (current_scale < 16)
    {
      chopconf.value.vsense (true);
      current_scale = static_cast<int> (
          32.0f * 1.41421f * mA / 1000.0f * (rSense + 0.02f) / 0.180f
          - 1.0f);
    }
  else
    {
      chopconf.value.vsense (false);
    }

  const uint8_t clamped_scale = clampCurrentScale (current_scale);
  const int hold_current = static_cast<int> (clamped_scale * holdMultiplier);

  current.value.irun (clamped_scale);
  current.value.ihold (clampCurrentScale (hold_current));

  auto result = registers_->writeChopconf (chopconf.value);
  if (!result.ok ())
    {
      return result;
    }
  return registers_->writeIholdIrun (current.value);
}

Result<void>
Driver::enableInverseMotorDirection ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto gconf = registers_->readGconf ();
  if (!gconf.ok ())
    {
      Result<void> result;
      result.error = gconf.error;
      return result;
    }
  gconf.value.shaft (true);
  return registers_->writeGconf (gconf.value);
}

Result<void>
Driver::disableInverseMotorDirection ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto gconf = registers_->readGconf ();
  if (!gconf.ok ())
    {
      Result<void> result;
      result.error = gconf.error;
      return result;
    }
  gconf.value.shaft (false);
  return registers_->writeGconf (gconf.value);
}

Result<void>
Driver::enableStealthChop ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto gconf = registers_->readGconf ();
  if (!gconf.ok ())
    {
      Result<void> result;
      result.error = gconf.error;
      return result;
    }
  gconf.value.enable_spread_cycle (false);
  return registers_->writeGconf (gconf.value);
}

Result<void>
Driver::disableStealthChop ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto gconf = registers_->readGconf ();
  if (!gconf.ok ())
    {
      Result<void> result;
      result.error = gconf.error;
      return result;
    }
  gconf.value.enable_spread_cycle (true);
  return registers_->writeGconf (gconf.value);
}

Result<void>
Driver::enableAutomaticCurrentScaling ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto pwmconf = registers_->readPwmconf ();
  if (!pwmconf.ok ())
    {
      Result<void> result;
      result.error = pwmconf.error;
      return result;
    }
  pwmconf.value.pwm_autoscale (true);
  return registers_->writePwmconf (pwmconf.value);
}

Result<void>
Driver::disableAutomaticCurrentScaling ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto pwmconf = registers_->readPwmconf ();
  if (!pwmconf.ok ())
    {
      Result<void> result;
      result.error = pwmconf.error;
      return result;
    }
  pwmconf.value.pwm_autoscale (false);
  return registers_->writePwmconf (pwmconf.value);
}

Result<void>
Driver::enableAutomaticGradientAdaptation ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto pwmconf = registers_->readPwmconf ();
  if (!pwmconf.ok ())
    {
      Result<void> result;
      result.error = pwmconf.error;
      return result;
    }
  pwmconf.value.pwm_autograd (true);
  return registers_->writePwmconf (pwmconf.value);
}

Result<void>
Driver::disableAutomaticGradientAdaptation ()
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto pwmconf = registers_->readPwmconf ();
  if (!pwmconf.ok ())
    {
      Result<void> result;
      result.error = pwmconf.error;
      return result;
    }
  pwmconf.value.pwm_autograd (false);
  return registers_->writePwmconf (pwmconf.value);
}

Result<void>
Driver::setPwmOffset (uint8_t pwm_offset)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto pwmconf = registers_->readPwmconf ();
  if (!pwmconf.ok ())
    {
      Result<void> result;
      result.error = pwmconf.error;
      return result;
    }
  pwmconf.value.pwm_offset (pwm_offset);
  return registers_->writePwmconf (pwmconf.value);
}

Result<void>
Driver::setPwmGradient (uint8_t pwm_gradient)
{
  if (!isBound ())
    {
      return notInitialized_ ();
    }
  auto pwmconf = registers_->readPwmconf ();
  if (!pwmconf.ok ())
    {
      Result<void> result;
      result.error = pwmconf.error;
      return result;
    }
  pwmconf.value.pwm_grad (pwm_gradient);
  return registers_->writePwmconf (pwmconf.value);
}

Result<uint8_t>
Driver::getVersion () const
{
  Result<uint8_t> result;
  if (!isBound ())
    {
      result.error = UartError::NotInitialized;
      return result;
    }

  const auto ioin = registers_->readIoin ();
  if (!ioin.ok ())
    {
      result.error = ioin.error;
      return result;
    }

  result.value = static_cast<uint8_t> (ioin.value.version ());
  if (result.value != VERSION)
    {
      // The transport succeeded; the version byte can still be surfaced to the
      // caller for diagnostics, so keep UartError::None.
    }
  return result;
}

Result<void>
Driver::notInitialized_ () const
{
  Result<void> result;
  result.error = UartError::NotInitialized;
  return result;
}

uint8_t
Driver::percentToCurrentSetting_ (uint8_t percent) const
{
  const uint8_t constrained_percent = constrain (percent, PERCENT_MIN,
                                                 PERCENT_MAX);
  return static_cast<uint8_t> (map (constrained_percent, PERCENT_MIN,
                                    PERCENT_MAX, CURRENT_SETTING_MIN,
                                    CURRENT_SETTING_MAX));
}

uint8_t
Driver::percentToHoldDelaySetting_ (uint8_t percent) const
{
  const uint8_t constrained_percent = constrain (percent, PERCENT_MIN,
                                                 PERCENT_MAX);
  return static_cast<uint8_t> (map (constrained_percent, PERCENT_MIN,
                                    PERCENT_MAX, HOLD_DELAY_MIN,
                                    HOLD_DELAY_MAX));
}

reg::Mres
Driver::microstepsToMres_ (uint16_t microsteps_per_step) const
{
  uint16_t shifted = constrain (microsteps_per_step,
                                static_cast<uint16_t> (1u),
                                static_cast<uint16_t> (256u));
  shifted = static_cast<uint16_t> (shifted >> 1);

  uint8_t exponent = 0u;
  while (shifted > 0u)
    {
      shifted = static_cast<uint16_t> (shifted >> 1);
      ++exponent;
    }

  switch (exponent)
    {
    case 0:
      return reg::Mres::M1;
    case 1:
      return reg::Mres::M2;
    case 2:
      return reg::Mres::M4;
    case 3:
      return reg::Mres::M8;
    case 4:
      return reg::Mres::M16;
    case 5:
      return reg::Mres::M32;
    case 6:
      return reg::Mres::M64;
    case 7:
      return reg::Mres::M128;
    case 8:
    default:
      return reg::Mres::M256;
    }
}

uint16_t
Driver::mresToMicrosteps_ (reg::Mres mres) const
{
  switch (mres)
    {
    case reg::Mres::M1:
      return 1u;
    case reg::Mres::M2:
      return 2u;
    case reg::Mres::M4:
      return 4u;
    case reg::Mres::M8:
      return 8u;
    case reg::Mres::M16:
      return 16u;
    case reg::Mres::M32:
      return 32u;
    case reg::Mres::M64:
      return 64u;
    case reg::Mres::M128:
      return 128u;
    case reg::Mres::M256:
    default:
      return 256u;
    }
}

} // namespace tmc2209
