// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC2209.h"


TMC2209::TMC2209 ()
{
  facade_device_.bind (facade_bus_, 0u);
  registers.bind (facade_device_);
  driver.bind (facade_device_, registers);

  hardware_enable_pin_ = -1;
  cool_step_enabled_ = false;
  last_uart_error_ = UartError::None;
  mirror_resync_required_ = false;
}

#if !defined(ARDUINO_ARCH_RENESAS)
void
TMC2209::setup (HardwareSerial &serial, SerialAddress serial_address)
{
  facade_bus_.setup (serial);
  facade_device_.bind (facade_bus_, static_cast<uint8_t> (serial_address));
  last_uart_error_ = UartError::None;

  initialize (serial_address);
}
#endif
#if defined(ESP32)
void
TMC2209::setup (HardwareSerial &serial, SerialAddress serial_address)
{
  facade_bus_.setup (serial);
  facade_device_.bind (facade_bus_, static_cast<uint8_t> (serial_address));
  last_uart_error_ = UartError::None;
  initialize (serial_address);
}
#elif defined(ARDUINO_ARCH_RP2040)
void
TMC2209::setup (SerialUART &serial, SerialAddress serial_address)
{
  facade_bus_.setup (serial);
  facade_device_.bind (facade_bus_, static_cast<uint8_t> (serial_address));
  last_uart_error_ = UartError::None;
  initialize (serial_address);
}
#elif defined(ARDUINO_ARCH_RENESAS)
void
TMC2209::setup (UART &serial, SerialAddress serial_address)
{
  facade_bus_.setup (serial);
  facade_device_.bind (facade_bus_, static_cast<uint8_t> (serial_address));
  last_uart_error_ = UartError::None;
  initialize (serial_address);
}
#endif

#if SOFTWARE_SERIAL_INCLUDED
void
TMC2209::setup (SoftwareSerial &serial, SerialAddress serial_address)
{
  facade_bus_.setup (serial);
  facade_device_.bind (facade_bus_, static_cast<uint8_t> (serial_address));
  last_uart_error_ = UartError::None;
  initialize (serial_address);
}
#endif

// unidirectional methods

void
TMC2209::setHardwareEnablePin (uint8_t hardware_enable_pin)
{
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode (hardware_enable_pin_, OUTPUT);
  digitalWrite (hardware_enable_pin_, HIGH);
}

void
TMC2209::enable ()
{
  if (hardware_enable_pin_ >= 0)
    {
      digitalWrite (hardware_enable_pin_, LOW);
    }
  chopconf_.toff (toff_);
  writeStoredChopperConfig ();
}

void
TMC2209::disable ()
{
  if (hardware_enable_pin_ >= 0)
    {
      digitalWrite (hardware_enable_pin_, HIGH);
    }
  chopconf_.toff (TOFF_DISABLE);
  writeStoredChopperConfig ();
}

void
TMC2209::setMicrostepsPerStep (uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted = constrain_ (
      microsteps_per_step, MICROSTEPS_PER_STEP_MIN, MICROSTEPS_PER_STEP_MAX);
  // Shift the constrained value (not the raw input) so out-of-range values
  // don't produce an incorrect exponent.
  microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
  uint16_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
    {
      microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
      ++exponent;
    }
  setMicrostepsPerStepPowerOfTwo (exponent);
}

void
TMC2209::setMicrostepsPerStepPowerOfTwo (uint8_t exponent)
{
  switch (exponent)
    {
    case 0:
      {
        chopconf_.mres (tmc2209::reg::Mres::M1);
        break;
      }
    case 1:
      {
        chopconf_.mres (tmc2209::reg::Mres::M2);
        break;
      }
    case 2:
      {
        chopconf_.mres (tmc2209::reg::Mres::M4);
        break;
      }
    case 3:
      {
        chopconf_.mres (tmc2209::reg::Mres::M8);
        break;
      }
    case 4:
      {
        chopconf_.mres (tmc2209::reg::Mres::M16);
        break;
      }
    case 5:
      {
        chopconf_.mres (tmc2209::reg::Mres::M32);
        break;
      }
    case 6:
      {
        chopconf_.mres (tmc2209::reg::Mres::M64);
        break;
      }
    case 7:
      {
        chopconf_.mres (tmc2209::reg::Mres::M128);
        break;
      }
    case 8:
    default:
      {
        chopconf_.mres (tmc2209::reg::Mres::M256);
        break;
      }
    }
  writeStoredChopperConfig ();
}

void
TMC2209::setRunCurrent (uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting (percent);
  ihold_irun_.irun (run_current);
  writeStoredDriverCurrent ();
}

void
TMC2209::setHoldCurrent (uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting (percent);

  ihold_irun_.ihold (hold_current);
  writeStoredDriverCurrent ();
}

void
TMC2209::setHoldDelay (uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting (percent);

  ihold_irun_.iholddelay (hold_delay);
  writeStoredDriverCurrent ();
}

void
TMC2209::setAllCurrentValues (uint8_t run_current_percent,
                              uint8_t hold_current_percent,
                              uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting (run_current_percent);
  uint8_t hold_current = percentToCurrentSetting (hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting (hold_delay_percent);

  ihold_irun_.irun (run_current);
  ihold_irun_.ihold (hold_current);
  ihold_irun_.iholddelay (hold_delay);
  writeStoredDriverCurrent ();
}

void
TMC2209::setRMSCurrent (uint16_t mA, float rSense, float holdMultiplier)
{
  // Taken from
  // https://github.com/teemuatlut/TMCStepper/blob/74e8e6881adc9241c2e626071e7328d7652f361a/src/source/TMCStepper.cpp#L41.

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

  int CS = static_cast<int> (
      32.0f * 1.41421f * mA / 1000.0f * (rSense + 0.02f) / 0.325f - 1.0f);

  // If Current Scale is too low, turn on high sensitivity R_sense and
  // calculate again.
  if (CS < 16)
    {
      enableVSense ();
      CS = static_cast<int> (
          32.0f * 1.41421f * mA / 1000.0f * (rSense + 0.02f) / 0.180f
          - 1.0f);
    }
  else
    { // If CS >= 16, turn off high_sense_r
      disableVSense ();
    }

  const uint8_t clamped_cs = clampCurrentScale (CS);
  int hold_current = static_cast<int> (clamped_cs * holdMultiplier);
  const uint8_t clamped_hold_current = clampCurrentScale (hold_current);

  ihold_irun_.irun (clamped_cs);
  ihold_irun_.ihold (clamped_hold_current);
  writeStoredDriverCurrent ();
}

void
TMC2209::enableDoubleEdge ()
{
  chopconf_.double_edge (true);
  writeStoredChopperConfig ();
}

void
TMC2209::disableDoubleEdge ()
{
  chopconf_.double_edge (false);
  writeStoredChopperConfig ();
}

void
TMC2209::enableVSense ()
{
  chopconf_.vsense (true);
  writeStoredChopperConfig ();
}

void
TMC2209::disableVSense ()
{
  chopconf_.vsense (false);
  writeStoredChopperConfig ();
}

void
TMC2209::enableInverseMotorDirection ()
{
  gconf_.shaft (true);
  writeStoredGlobalConfig ();
}

void
TMC2209::disableInverseMotorDirection ()
{
  gconf_.shaft (false);
  writeStoredGlobalConfig ();
}

void
TMC2209::setStandstillMode (TMC2209::StandstillMode mode)
{
  pwmconf_.freewheel (static_cast<uint32_t> (mode));
  writeStoredPwmConfig ();
}

void
TMC2209::enableAutomaticCurrentScaling ()
{
  pwmconf_.pwm_autoscale (true);
  writeStoredPwmConfig ();
}

void
TMC2209::disableAutomaticCurrentScaling ()
{
  pwmconf_.pwm_autoscale (false);
  writeStoredPwmConfig ();
}

void
TMC2209::enableAutomaticGradientAdaptation ()
{
  pwmconf_.pwm_autograd (true);
  writeStoredPwmConfig ();
}

void
TMC2209::disableAutomaticGradientAdaptation ()
{
  pwmconf_.pwm_autograd (false);
  writeStoredPwmConfig ();
}

void
TMC2209::setPwmOffset (uint8_t pwm_amplitude)
{
  pwmconf_.pwm_offset (pwm_amplitude);
  writeStoredPwmConfig ();
}

void
TMC2209::setPwmGradient (uint8_t pwm_amplitude)
{
  pwmconf_.pwm_grad (pwm_amplitude);
  writeStoredPwmConfig ();
}

void
TMC2209::setPowerDownDelay (uint8_t power_down_delay)
{
  write (ADDRESS_TPOWERDOWN, power_down_delay);
}

void
TMC2209::setReplyDelay (uint8_t reply_delay)
{
  if (reply_delay > REPLY_DELAY_MAX)
    {
      reply_delay = REPLY_DELAY_MAX;
    }

  tmc2209::reg::REPLYDELAY reply_delay_data;
  reply_delay_data.raw = 0;
  reply_delay_data.replydelay (reply_delay);

  write (ADDRESS_REPLYDELAY, reply_delay_data.raw);
}

void
TMC2209::moveAtVelocity (int32_t microsteps_per_period)
{
  write (ADDRESS_VACTUAL, microsteps_per_period);
}

void
TMC2209::moveUsingStepDirInterface ()
{
  write (ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void
TMC2209::enableStealthChop ()
{
  gconf_.enable_spread_cycle (false);
  writeStoredGlobalConfig ();
}

void
TMC2209::disableStealthChop ()
{
  gconf_.enable_spread_cycle (true);
  writeStoredGlobalConfig ();
}

void
TMC2209::setCoolStepDurationThreshold (uint32_t duration_threshold)
{
  write (ADDRESS_TCOOLTHRS, duration_threshold);
}

void
TMC2209::setStealthChopDurationThreshold (uint32_t duration_threshold)
{
  write (ADDRESS_TPWMTHRS, duration_threshold);
}

void
TMC2209::setStallGuardThreshold (uint8_t stall_guard_threshold)
{
  write (ADDRESS_SGTHRS, stall_guard_threshold);
}

void
TMC2209::enableCoolStep (uint8_t lower_threshold, uint8_t upper_threshold)
{
  lower_threshold = constrain_ (lower_threshold, SEMIN_MIN, SEMIN_MAX);
  coolconf_.semin (lower_threshold);
  upper_threshold = constrain_ (upper_threshold, SEMAX_MIN, SEMAX_MAX);
  coolconf_.semax (upper_threshold);
  write (ADDRESS_COOLCONF, coolconf_.raw);
  cool_step_enabled_ = true;
}

void
TMC2209::disableCoolStep ()
{
  coolconf_.semin (SEMIN_OFF);
  write (ADDRESS_COOLCONF, coolconf_.raw);
  cool_step_enabled_ = false;
}

void
TMC2209::setCoolStepCurrentIncrement (CurrentIncrement current_increment)
{
  coolconf_.seup (static_cast<uint32_t> (current_increment));
  write (ADDRESS_COOLCONF, coolconf_.raw);
}

void
TMC2209::setCoolStepMeasurementCount (MeasurementCount measurement_count)
{
  coolconf_.sedn (static_cast<uint32_t> (measurement_count));
  write (ADDRESS_COOLCONF, coolconf_.raw);
}

void
TMC2209::enableAnalogCurrentScaling ()
{
  gconf_.i_scale_analog (true);
  writeStoredGlobalConfig ();
}

void
TMC2209::disableAnalogCurrentScaling ()
{
  gconf_.i_scale_analog (false);
  writeStoredGlobalConfig ();
}

void
TMC2209::useExternalSenseResistors ()
{
  gconf_.internal_rsense (false);
  writeStoredGlobalConfig ();
}

void
TMC2209::useInternalSenseResistors ()
{
  gconf_.internal_rsense (true);
  writeStoredGlobalConfig ();
}

// bidirectional methods

uint8_t
TMC2209::getVersion ()
{
  const auto result = driver.getVersion ();
  last_uart_error_ = result.error;
  if (!result.ok ())
    {
      return 0u;
    }
  return result.value;
}

TMC2209::Result<uint32_t>
TMC2209::readRegister (uint8_t register_address)
{
  Result<uint32_t> result = facade_device_.readRegister (register_address);
  last_uart_error_ = result.error;
  return result;
}

TMC2209::Result<void>
TMC2209::writeRegister (uint8_t register_address, uint32_t data)
{
  Result<void> result = facade_device_.writeRegister (register_address, data);
  last_uart_error_ = result.error;
  return result;
}

TMC2209::Result<void>
TMC2209::startRead (uint8_t register_address)
{
  Result<void> result = facade_device_.startRead (register_address);
  last_uart_error_ = result.error;
  return result;
}

TMC2209::Result<void>
TMC2209::startWrite (uint8_t register_address, uint32_t data)
{
  Result<void> result = facade_device_.startWrite (register_address, data);
  last_uart_error_ = result.error;
  return result;
}

void
TMC2209::poll ()
{
  facade_device_.poll ();
  if (facade_device_.resultReady ())
    {
      last_uart_error_ = facade_device_.lastError ();
    }
}

bool
TMC2209::busy () const
{
  return facade_bus_.busy (facade_device_.serialAddress ());
}

bool
TMC2209::resultReady () const
{
  return facade_device_.resultReady ();
}

bool
TMC2209::done () const
{
  return resultReady ();
}

TMC2209::Result<uint32_t>
TMC2209::takeReadResult ()
{
  Result<uint32_t> result = facade_device_.takeReadResult ();
  last_uart_error_ = result.error;
  return result;
}

TMC2209::Result<void>
TMC2209::takeWriteResult ()
{
  Result<void> result = facade_device_.takeWriteResult ();
  last_uart_error_ = result.error;
  return result;
}

TMC2209::UartError
TMC2209::getLastUartError () const
{
  const auto bus_error = facade_bus_.lastError ();
  if (bus_error != UartError::None)
    {
      return bus_error;
    }
  return last_uart_error_;
}

void
TMC2209::clearLastUartError ()
{
  last_uart_error_ = UartError::None;
  facade_device_.clearLastError ();
}

void
TMC2209::enableWriteVerification ()
{
  UartParameters parameters = facade_device_.parameters ();
  parameters.verify_writes = true;
  facade_device_.setParameters (parameters);
}

void
TMC2209::disableWriteVerification ()
{
  UartParameters parameters = facade_device_.parameters ();
  parameters.verify_writes = false;
  facade_device_.setParameters (parameters);
}

bool
TMC2209::writeVerificationEnabled () const
{
  return facade_device_.parameters ().verify_writes;
}

bool
TMC2209::isCommunicating ()
{
  return (getVersion () == VERSION);
}

bool
TMC2209::isSetupAndCommunicating ()
{
  return serialOperationMode ();
}

bool
TMC2209::isCommunicatingButNotSetup ()
{
  return (isCommunicating () && (not isSetupAndCommunicating ()));
}

bool
TMC2209::hardwareDisabled ()
{
  tmc2209::reg::IOIN input;
  input.raw = read (ADDRESS_IOIN);

  return input.enn ();
}

uint16_t
TMC2209::getMicrostepsPerStep ()
{
  if (facade_bus_.isConfigured ())
    {
      const auto chopconf = registers.readChopconf ();
      if (chopconf.ok ())
        {
          chopconf_ = chopconf.value;
          if (chopconf_.toff () > 0u)
            {
              toff_ = static_cast<uint8_t> (chopconf_.toff ());
            }
          last_uart_error_ = UartError::None;
        }
      else
        {
          last_uart_error_ = chopconf.error;
        }
    }

  uint16_t microsteps_per_step_exponent;
  switch (chopconf_.mres ())
    {
    case tmc2209::reg::Mres::M1:
      {
        microsteps_per_step_exponent = 0;
        break;
      }
    case tmc2209::reg::Mres::M2:
      {
        microsteps_per_step_exponent = 1;
        break;
      }
    case tmc2209::reg::Mres::M4:
      {
        microsteps_per_step_exponent = 2;
        break;
      }
    case tmc2209::reg::Mres::M8:
      {
        microsteps_per_step_exponent = 3;
        break;
      }
    case tmc2209::reg::Mres::M16:
      {
        microsteps_per_step_exponent = 4;
        break;
      }
    case tmc2209::reg::Mres::M32:
      {
        microsteps_per_step_exponent = 5;
        break;
      }
    case tmc2209::reg::Mres::M64:
      {
        microsteps_per_step_exponent = 6;
        break;
      }
    case tmc2209::reg::Mres::M128:
      {
        microsteps_per_step_exponent = 7;
        break;
      }
    case tmc2209::reg::Mres::M256:
    default:
      {
        microsteps_per_step_exponent = 8;
        break;
      }
    }
  return 1 << microsteps_per_step_exponent;
}

TMC2209::Settings
TMC2209::getSettings ()
{
  Settings settings;
  settings.is_communicating = isCommunicating ();

  if (settings.is_communicating)
    {
      readAndStoreRegisters ();

      settings.is_setup = gconf_.pdn_disable ();
      settings.software_enabled = (chopconf_.toff () > TOFF_DISABLE);
      settings.microsteps_per_step = getMicrostepsPerStep ();
      settings.inverse_motor_direction_enabled = gconf_.shaft ();
      settings.stealth_chop_enabled = not gconf_.enable_spread_cycle ();
      settings.standstill_mode = static_cast<uint8_t> (pwmconf_.freewheel ());
      settings.irun_percent = currentSettingToPercent (ihold_irun_.irun ());
      settings.irun_register_value = ihold_irun_.irun ();
      settings.ihold_percent = currentSettingToPercent (ihold_irun_.ihold ());
      settings.ihold_register_value = ihold_irun_.ihold ();
      settings.iholddelay_percent
          = holdDelaySettingToPercent (ihold_irun_.iholddelay ());
      settings.iholddelay_register_value = ihold_irun_.iholddelay ();
      settings.automatic_current_scaling_enabled = pwmconf_.pwm_autoscale ();
      settings.automatic_gradient_adaptation_enabled
          = pwmconf_.pwm_autograd ();
      settings.pwm_offset = static_cast<uint8_t> (pwmconf_.pwm_offset ());
      settings.pwm_gradient = static_cast<uint8_t> (pwmconf_.pwm_grad ());
      settings.cool_step_enabled = cool_step_enabled_;
      settings.analog_current_scaling_enabled = gconf_.i_scale_analog ();
      settings.internal_sense_resistors_enabled
          = gconf_.internal_rsense ();
    }
  else
    {
      settings.is_setup = false;
      settings.software_enabled = false;
      settings.microsteps_per_step = 0;
      settings.inverse_motor_direction_enabled = false;
      settings.stealth_chop_enabled = false;
      settings.standstill_mode = static_cast<uint8_t> (pwmconf_.freewheel ());
      settings.irun_percent = 0;
      settings.irun_register_value = 0;
      settings.ihold_percent = 0;
      settings.ihold_register_value = 0;
      settings.iholddelay_percent = 0;
      settings.iholddelay_register_value = 0;
      settings.automatic_current_scaling_enabled = false;
      settings.automatic_gradient_adaptation_enabled = false;
      settings.pwm_offset = 0;
      settings.pwm_gradient = 0;
      settings.cool_step_enabled = false;
      settings.analog_current_scaling_enabled = false;
      settings.internal_sense_resistors_enabled = false;
    }

  return settings;
}

TMC2209::Status
TMC2209::getStatus ()
{
  tmc2209::reg::DRV_STATUS drive_status;
  drive_status.raw = read (ADDRESS_DRV_STATUS);

  Status status{};
  status.over_temperature_warning = drive_status.over_temperature_warning ();
  status.over_temperature_shutdown = drive_status.over_temperature_shutdown ();
  status.short_to_ground_a = drive_status.short_to_ground_a ();
  status.short_to_ground_b = drive_status.short_to_ground_b ();
  status.low_side_short_a = drive_status.low_side_short_a ();
  status.low_side_short_b = drive_status.low_side_short_b ();
  status.open_load_a = drive_status.open_load_a ();
  status.open_load_b = drive_status.open_load_b ();
  status.over_temperature_120c = drive_status.over_temperature_120c ();
  status.over_temperature_143c = drive_status.over_temperature_143c ();
  status.over_temperature_150c = drive_status.over_temperature_150c ();
  status.over_temperature_157c = drive_status.over_temperature_157c ();
  status.current_scaling
      = static_cast<uint8_t> (drive_status.current_scaling ());
  status.stealth_chop_mode = drive_status.stealth_chop_mode ();
  status.standstill = drive_status.standstill ();

  return status;
}

TMC2209::GlobalStatus
TMC2209::getGlobalStatus ()
{
  tmc2209::reg::GSTAT gstat;
  gstat.raw = read (ADDRESS_GSTAT);

  GlobalStatus status{};
  status.reset = gstat.reset ();
  status.drv_err = gstat.drv_err ();
  status.uv_cp = gstat.uv_cp ();

  return status;
}

void
TMC2209::clearReset ()
{
  tmc2209::reg::GSTAT gstat;
  gstat.raw = 0;
  gstat.reset (true);
  write (ADDRESS_GSTAT, gstat.raw);
}

void
TMC2209::clearDriveError ()
{
  tmc2209::reg::GSTAT gstat;
  gstat.raw = 0;
  gstat.drv_err (true);
  write (ADDRESS_GSTAT, gstat.raw);
}

TMC2209::HealthStatus
TMC2209::readHealthStatus ()
{
  HealthStatus status;
  const GlobalStatus global_status = getGlobalStatus ();
  status.communication_ok = isCommunicating ();
  status.setup_ok = isSetupAndCommunicating ();
  status.reset = global_status.reset;
  status.driver_error = global_status.drv_err;
  status.charge_pump_undervoltage = global_status.uv_cp;
  status.mirror_resync_required = mirrorResyncRequired ();
  return status;
}

void
TMC2209::notePossibleMirrorDrift ()
{
  mirror_resync_required_ = true;
}

bool
TMC2209::mirrorResyncRequired () const
{
  return mirror_resync_required_;
}

bool
TMC2209::reinitialize ()
{
  if (!isCommunicating ())
    {
      notePossibleMirrorDrift ();
      return false;
    }

  if (!replayCachedConfiguration_ ())
    {
      notePossibleMirrorDrift ();
      return false;
    }

  mirror_resync_required_ = false;
  return isSetupAndCommunicating ();
}

bool
TMC2209::recoverFromDeviceReset ()
{
  if (!isCommunicating ())
    {
      notePossibleMirrorDrift ();
      return false;
    }

  const GlobalStatus global_status = getGlobalStatus ();
  if (global_status.reset || global_status.drv_err || global_status.uv_cp
      || mirrorResyncRequired ())
    {
      notePossibleMirrorDrift ();
      return reinitialize ();
    }

  mirror_resync_required_ = false;
  return true;
}

bool
TMC2209::recoverIfNeeded ()
{
  if (!mirrorResyncRequired ())
    {
      return true;
    }
  return recoverFromDeviceReset ();
}

bool
TMC2209::recoverIfUnhealthy ()
{
  const HealthStatus status = readHealthStatus ();
  if (!status.communication_ok || !status.setup_ok || status.reset
      || status.driver_error || status.charge_pump_undervoltage)
    {
      notePossibleMirrorDrift ();
    }
  return recoverIfNeeded ();
}

bool
TMC2209::resyncReadableConfiguration ()
{
  if (!isCommunicating ())
    {
      notePossibleMirrorDrift ();
      return false;
    }

  const auto gconf = registers.readGconf ();
  if (!gconf.ok ())
    {
      last_uart_error_ = gconf.error;
      notePossibleMirrorDrift ();
      return false;
    }
  gconf_ = gconf.value;

  const auto ihold_irun = registers.readIholdIrun ();
  if (!ihold_irun.ok ())
    {
      last_uart_error_ = ihold_irun.error;
      notePossibleMirrorDrift ();
      return false;
    }
  ihold_irun_ = ihold_irun.value;

  const auto coolconf = registers.readCoolconf ();
  if (!coolconf.ok ())
    {
      last_uart_error_ = coolconf.error;
      notePossibleMirrorDrift ();
      return false;
    }
  coolconf_ = coolconf.value;
  cool_step_enabled_ = (coolconf_.semin () != SEMIN_OFF);

  const auto chopconf = registers.readChopconf ();
  if (!chopconf.ok ())
    {
      last_uart_error_ = chopconf.error;
      notePossibleMirrorDrift ();
      return false;
    }
  chopconf_ = chopconf.value;
  if (chopconf_.toff () > 0u)
    {
      toff_ = static_cast<uint8_t> (chopconf_.toff ());
    }

  const auto pwmconf = registers.readPwmconf ();
  if (!pwmconf.ok ())
    {
      last_uart_error_ = pwmconf.error;
      notePossibleMirrorDrift ();
      return false;
    }
  pwmconf_ = pwmconf.value;

  const auto reply_delay = readRegister (ADDRESS_REPLYDELAY);
  if (!reply_delay.ok ())
    {
      notePossibleMirrorDrift ();
      return false;
    }
  reply_delay_raw_ = reply_delay.value;

  const auto tpowerdown = readRegister (ADDRESS_TPOWERDOWN);
  if (!tpowerdown.ok ())
    {
      notePossibleMirrorDrift ();
      return false;
    }
  tpowerdown_raw_ = tpowerdown.value;

  const auto tpwmthrs = readRegister (ADDRESS_TPWMTHRS);
  if (!tpwmthrs.ok ())
    {
      notePossibleMirrorDrift ();
      return false;
    }
  tpwmthrs_raw_ = tpwmthrs.value;

  const auto vactual = readRegister (ADDRESS_VACTUAL);
  if (!vactual.ok ())
    {
      notePossibleMirrorDrift ();
      return false;
    }
  vactual_raw_ = vactual.value;

  const auto tcoolthrs = readRegister (ADDRESS_TCOOLTHRS);
  if (!tcoolthrs.ok ())
    {
      notePossibleMirrorDrift ();
      return false;
    }
  tcoolthrs_raw_ = tcoolthrs.value;

  const auto sgthrs = readRegister (ADDRESS_SGTHRS);
  if (!sgthrs.ok ())
    {
      notePossibleMirrorDrift ();
      return false;
    }
  sgthrs_raw_ = sgthrs.value;

  last_uart_error_ = UartError::None;
  mirror_resync_required_ = false;
  return true;
}

uint8_t
TMC2209::getInterfaceTransmissionCounter ()
{
  return read (ADDRESS_IFCNT);
}

uint32_t
TMC2209::getInterstepDuration ()
{
  return read (ADDRESS_TSTEP);
}

uint16_t
TMC2209::getStallGuardResult ()
{
  return read (ADDRESS_SG_RESULT);
}

uint8_t
TMC2209::getPwmScaleSum ()
{
  tmc2209::reg::PWM_SCALE pwm_scale;
  pwm_scale.raw = read (ADDRESS_PWM_SCALE);

  return static_cast<uint8_t> (pwm_scale.pwm_scale_sum ());
}

int16_t
TMC2209::getPwmScaleAuto ()
{
  tmc2209::reg::PWM_SCALE pwm_scale;
  pwm_scale.raw = read (ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_auto_signed ();
}

uint8_t
TMC2209::getPwmOffsetAuto ()
{
  tmc2209::reg::PWM_AUTO pwm_auto;
  pwm_auto.raw = read (ADDRESS_PWM_AUTO);

  return static_cast<uint8_t> (pwm_auto.pwm_offset_auto ());
}

uint8_t
TMC2209::getPwmGradientAuto ()
{
  tmc2209::reg::PWM_AUTO pwm_auto;
  pwm_auto.raw = read (ADDRESS_PWM_AUTO);

  return static_cast<uint8_t> (pwm_auto.pwm_gradient_auto ());
}

uint16_t
TMC2209::getMicrostepCounter ()
{
  return read (ADDRESS_MSCNT);
}

// private
void
TMC2209::initialize (SerialAddress serial_address)
{
  mirror_resync_required_ = false;
  setOperationModeToSerial (serial_address);
  setRegistersToDefaults ();
  clearDriveError ();

  minimizeMotorCurrent ();
  disable ();
  disableAutomaticCurrentScaling ();
  disableAutomaticGradientAdaptation ();
}

void
TMC2209::setOperationModeToSerial (SerialAddress serial_address)
{
  (void)serial_address;

  gconf_.raw = 0;
  gconf_.i_scale_analog (false);
  gconf_.pdn_disable (true);
  gconf_.mstep_reg_select (true);
  gconf_.multistep_filt (true);

  writeStoredGlobalConfig ();
}

void
TMC2209::setRegistersToDefaults ()
{
  ihold_irun_.raw = 0;
  ihold_irun_.ihold (IHOLD_DEFAULT);
  ihold_irun_.irun (IRUN_DEFAULT);
  ihold_irun_.iholddelay (IHOLDDELAY_DEFAULT);
  write (ADDRESS_IHOLD_IRUN, ihold_irun_.raw);

  chopconf_.raw = CHOPPER_CONFIG_DEFAULT;
  chopconf_.tbl (TBL_DEFAULT);
  chopconf_.hend (HEND_DEFAULT);
  chopconf_.hstart (HSTART_DEFAULT);
  chopconf_.toff (TOFF_DEFAULT);
  write (ADDRESS_CHOPCONF, chopconf_.raw);

  pwmconf_.raw = PWM_CONFIG_DEFAULT;
  write (ADDRESS_PWMCONF, pwmconf_.raw);

  coolconf_.raw = COOLCONF_DEFAULT;
  write (ADDRESS_COOLCONF, coolconf_.raw);

  write (ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  write (ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
  write (ADDRESS_VACTUAL, VACTUAL_DEFAULT);
  write (ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
  write (ADDRESS_SGTHRS, SGTHRS_DEFAULT);
  write (ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

void
TMC2209::readAndStoreRegisters ()
{
  const auto gconf = registers.readGconf ();
  if (gconf.ok ())
    {
      gconf_ = gconf.value;
      last_uart_error_ = UartError::None;
    }

  const auto ihold_irun = registers.readIholdIrun ();
  if (ihold_irun.ok ())
    {
      ihold_irun_ = ihold_irun.value;
      last_uart_error_ = UartError::None;
    }

  const auto coolconf = registers.readCoolconf ();
  if (coolconf.ok ())
    {
      coolconf_ = coolconf.value;
      cool_step_enabled_ = (coolconf_.semin () != SEMIN_OFF);
      last_uart_error_ = UartError::None;
    }

  const auto chopconf = registers.readChopconf ();
  if (chopconf.ok ())
    {
      chopconf_ = chopconf.value;
      if (chopconf_.toff () > 0u)
        {
          toff_ = static_cast<uint8_t> (chopconf_.toff ());
        }
      last_uart_error_ = UartError::None;
    }

  const auto pwmconf = registers.readPwmconf ();
  if (pwmconf.ok ())
    {
      pwmconf_ = pwmconf.value;
      last_uart_error_ = UartError::None;
    }
}

bool
TMC2209::serialOperationMode ()
{
  tmc2209::reg::GCONF gconf;
  gconf.raw = readGlobalConfigBytes ();

  return gconf.pdn_disable ();
}

void
TMC2209::minimizeMotorCurrent ()
{
  ihold_irun_.irun (CURRENT_SETTING_MIN);
  ihold_irun_.ihold (CURRENT_SETTING_MIN);
  writeStoredDriverCurrent ();
}

void
TMC2209::write (uint8_t register_address, uint32_t data)
{
  switch (register_address)
    {
    case ADDRESS_REPLYDELAY:
      reply_delay_raw_ = data;
      break;
    case ADDRESS_TPOWERDOWN:
      tpowerdown_raw_ = data;
      break;
    case ADDRESS_TPWMTHRS:
      tpwmthrs_raw_ = data;
      break;
    case ADDRESS_VACTUAL:
      vactual_raw_ = data;
      break;
    case ADDRESS_TCOOLTHRS:
      tcoolthrs_raw_ = data;
      break;
    case ADDRESS_SGTHRS:
      sgthrs_raw_ = data;
      break;
    default:
      break;
    }
  (void)writeRegister (register_address, data);
}

uint32_t
TMC2209::read (uint8_t register_address)
{
  const auto result = readRegister (register_address);
  if (!result.ok ())
    {
      return 0;
    }
  return result.value;
}

uint8_t
TMC2209::percentToCurrentSetting (uint8_t percent)
{
  uint8_t constrained_percent = constrain_ (percent, PERCENT_MIN, PERCENT_MAX);
  uint8_t current_setting = map (constrained_percent, PERCENT_MIN, PERCENT_MAX,
                                 CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t
TMC2209::currentSettingToPercent (uint8_t current_setting)
{
  uint8_t percent = map (current_setting, CURRENT_SETTING_MIN,
                         CURRENT_SETTING_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

uint8_t
TMC2209::percentToHoldDelaySetting (uint8_t percent)
{
  uint8_t constrained_percent = constrain_ (percent, PERCENT_MIN, PERCENT_MAX);
  uint8_t hold_delay_setting
      = map (constrained_percent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN,
             HOLD_DELAY_MAX);
  return hold_delay_setting;
}

uint8_t
TMC2209::holdDelaySettingToPercent (uint8_t hold_delay_setting)
{
  uint8_t percent = map (hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX,
                         PERCENT_MIN, PERCENT_MAX);
  return percent;
}

void
TMC2209::writeStoredGlobalConfig ()
{
  write (ADDRESS_GCONF, gconf_.raw);
}

uint32_t
TMC2209::readGlobalConfigBytes ()
{
  return read (ADDRESS_GCONF);
}

void
TMC2209::writeStoredDriverCurrent ()
{
  write (ADDRESS_IHOLD_IRUN, ihold_irun_.raw);

  if (ihold_irun_.irun () >= SEIMIN_UPPER_CURRENT_LIMIT)
    {
      coolconf_.seimin ((SEIMIN_UPPER_SETTING) != 0);
    }
  else
    {
      coolconf_.seimin ((SEIMIN_LOWER_SETTING) != 0);
    }
  if (cool_step_enabled_)
    {
      write (ADDRESS_COOLCONF, coolconf_.raw);
    }
}

void
TMC2209::writeStoredChopperConfig ()
{
  write (ADDRESS_CHOPCONF, chopconf_.raw);
}

uint32_t
TMC2209::readChopperConfigBytes ()
{
  return read (ADDRESS_CHOPCONF);
}

void
TMC2209::writeStoredPwmConfig ()
{
  write (ADDRESS_PWMCONF, pwmconf_.raw);
}

uint32_t
TMC2209::readPwmConfigBytes ()
{
  return read (ADDRESS_PWMCONF);
}

bool
TMC2209::replayCachedConfiguration_ ()
{
  writeStoredGlobalConfig ();
  if (getLastUartError () != UartError::None)
    {
      return false;
    }

  writeStoredDriverCurrent ();
  if (getLastUartError () != UartError::None)
    {
      return false;
    }

  writeStoredChopperConfig ();
  if (getLastUartError () != UartError::None)
    {
      return false;
    }

  writeStoredPwmConfig ();
  if (getLastUartError () != UartError::None)
    {
      return false;
    }

  write (ADDRESS_COOLCONF, coolconf_.raw);
  if (getLastUartError () != UartError::None)
    {
      return false;
    }

  write (ADDRESS_REPLYDELAY, reply_delay_raw_);
  write (ADDRESS_TPOWERDOWN, tpowerdown_raw_);
  write (ADDRESS_TPWMTHRS, tpwmthrs_raw_);
  write (ADDRESS_VACTUAL, vactual_raw_);
  write (ADDRESS_TCOOLTHRS, tcoolthrs_raw_);
  write (ADDRESS_SGTHRS, sgthrs_raw_);
  clearReset ();
  clearDriveError ();
  return getLastUartError () == UartError::None;
}

uint32_t
TMC2209::constrain_ (uint32_t value, uint32_t low, uint32_t high)
{
  return ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)));
}
