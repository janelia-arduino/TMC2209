// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC2209.h"


TMC2209::TMC2209()
{
  serial_ptr_ = nullptr;
  enable_pin_ = -1;
  uart_address_ = UART_ADDRESS_0;

  global_config_.bytes = 0;

  driver_current_.bytes = 0;

  chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;

  pwm_config_.bytes = PWM_CONFIG_DEFAULT;

}

void TMC2209::setEnablePin(size_t enable_pin)
{
  enable_pin_ = enable_pin;

  pinMode(enable_pin_,OUTPUT);
  disable();
}

void TMC2209::setup(HardwareSerial & serial,
  UartAddress uart_address)
{
  setOperationModeToUart(serial,uart_address);
}

bool TMC2209::communicating()
{
  return (getVersion() == VERSION);
}

uint8_t TMC2209::getVersion()
{
  Input input;
  input.bytes = read(ADDRESS_IOIN);

  return input.version;
}

void TMC2209::enable()
{
  if (enable_pin_ >= 0)
  {
    digitalWrite(enable_pin_,LOW);
  }
}

void TMC2209::disable()
{
  if (enable_pin_ >= 0)
  {
    digitalWrite(enable_pin_,HIGH);
  }
}

void TMC2209::setMicrostepsPerStep(uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted = constrain(microsteps_per_step,
    MICROSTEPS_PER_STEP_MIN,
    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step >> 1;
  uint16_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  setMicrostepsPerStepPowerOfTwo(exponent);
}

uint16_t TMC2209::getMicrostepsPerStep()
{
  uint16_t microsteps_per_step_exponent;
  switch (chopper_config_.mres)
  {
    case MRES_001:
    {
      microsteps_per_step_exponent = 0;
      break;
    }
    case MRES_002:
    {
      microsteps_per_step_exponent = 1;
      break;
    }
    case MRES_004:
    {
      microsteps_per_step_exponent = 2;
      break;
    }
    case MRES_008:
    {
      microsteps_per_step_exponent = 3;
      break;
    }
    case MRES_016:
    {
      microsteps_per_step_exponent = 4;
      break;
    }
    case MRES_032:
    {
      microsteps_per_step_exponent = 5;
      break;
    }
    case MRES_064:
    {
      microsteps_per_step_exponent = 6;
      break;
    }
    case MRES_128:
    {
      microsteps_per_step_exponent = 7;
      break;
    }
    case MRES_256:
    default:
    {
      microsteps_per_step_exponent = 8;
      break;
    }
  }
  return 1 << microsteps_per_step_exponent;
}

void TMC2209::setRunCurrent(uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);

  driver_current_.irun = run_current;
  setDriverCurrent();
}

void TMC2209::setHoldCurrent(uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);

  driver_current_.ihold = hold_current;
  setDriverCurrent();
}

void TMC2209::setHoldDelay(uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);

  driver_current_.iholddelay = hold_delay;
  setDriverCurrent();
}

void TMC2209::setAllCurrentValues(uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  driver_current_.irun = run_current;
  driver_current_.ihold = hold_current;
  driver_current_.iholddelay = hold_delay;
  setDriverCurrent();
}

TMC2209::Status TMC2209::getStatus()
{
  DriveStatus drive_status;
  drive_status.bytes = read(ADDRESS_DRV_STATUS);
  return drive_status.status;
}

void TMC2209::enableInverseMotorDirection()
{
  global_config_.shaft = 1;
  setGlobalConfig();
}

void TMC2209::disableInverseMotorDirection()
{
  global_config_.shaft = 0;
  setGlobalConfig();
}

void TMC2209::enableStealthChop()
{
  global_config_.enable_spread_cycle = 0;
  setGlobalConfig();
}

void TMC2209::enableSpreadCycle()
{
  global_config_.enable_spread_cycle = 1;
  setGlobalConfig();
}

void TMC2209::setZeroHoldCurrentMode(TMC2209::ZeroHoldCurrentMode mode)
{
  pwm_config_.freewheel = mode;
  setPwmConfig();
}

TMC2209::Settings TMC2209::getSettings()
{
  getGlobalConfig();
  getChopperConfig();
  getPwmConfig();

  Settings settings;
  settings.microsteps_per_step = getMicrostepsPerStep();
  settings.inverse_motor_direction_enabled = global_config_.shaft;
  settings.spread_cycle_enabled = global_config_.enable_spread_cycle;
  settings.zero_hold_current_mode = pwm_config_.freewheel;
  settings.irun = currentSettingToPercent(driver_current_.irun);
  settings.ihold = currentSettingToPercent(driver_current_.ihold);
  settings.iholddelay = holdDelaySettingToPercent(driver_current_.iholddelay);

  return settings;
}

void TMC2209::setPwmThreshold(uint32_t value)
{
  write(ADDRESS_TPWMTHRS,value);
}

// private
void TMC2209::setOperationModeToUart(HardwareSerial & serial,
  UartAddress uart_address)
{
  serial_ptr_ = &serial;
  uart_address_ = uart_address;

  serial_ptr_->begin(SERIAL_BAUD_RATE);

  global_config_.i_scale_analog = 0;
  global_config_.internal_rsense = 0;
  global_config_.enable_spread_cycle = 0;
  global_config_.pdn_disable = 1;
  global_config_.mstep_reg_select = 1;

  setGlobalConfig();
}

void TMC2209::setOperationModeToStandalone()
{
  serial_ptr_ = nullptr;

  global_config_.i_scale_analog = 1;
  global_config_.internal_rsense = 1;
  global_config_.enable_spread_cycle = 0;
  global_config_.pdn_disable = 0;
  global_config_.mstep_reg_select = 0;

  setGlobalConfig();
}

void TMC2209::setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
{
  switch (exponent)
  {
    case 0:
    {
      chopper_config_.mres = MRES_001;
      break;
    }
    case 1:
    {
      chopper_config_.mres = MRES_002;
      break;
    }
    case 2:
    {
      chopper_config_.mres = MRES_004;
      break;
    }
    case 3:
    {
      chopper_config_.mres = MRES_008;
      break;
    }
    case 4:
    {
      chopper_config_.mres = MRES_016;
      break;
    }
    case 5:
    {
      chopper_config_.mres = MRES_032;
      break;
    }
    case 6:
    {
      chopper_config_.mres = MRES_064;
      break;
    }
    case 7:
    {
      chopper_config_.mres = MRES_128;
      break;
    }
    case 8:
    default:
    {
      chopper_config_.mres = MRES_256;
      break;
    }
  }
  setChopperConfig();
}

uint32_t TMC2209::reverseData(uint32_t data)
{
  uint32_t reversed_data = 0;
  uint8_t right_shift;
  uint8_t left_shift;
  for (uint8_t i=0; i<DATA_SIZE; ++i)
  {
    right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
    left_shift = i * BITS_PER_BYTE;
    reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
  }
  return reversed_data;
}

template<typename Datagram>
uint8_t TMC2209::calculateCrc(Datagram & datagram,
  uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t byte;
  for (uint8_t i=0; i<(datagram_size - 1); ++i)
  {
    byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    for (uint8_t j=0; j<BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

template<typename Datagram>
void TMC2209::sendDatagram(Datagram & datagram,
  uint8_t datagram_size)
{
  uint8_t byte;
  for (uint8_t i=0; i<datagram_size; ++i)
  {
    byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    serial_ptr_->write(byte);
  }

  // wait for bytes sent out on TX line to be echoed on RX line
  uint16_t echo_delay = 0;
  while ((serial_ptr_->available() <= 0) && (echo_delay++ < ECHO_DELAY_MAX_VALUE))
  {
    delay(1);
  }

  // clear RX buffer of echo bytes
  uint8_t bytes_read = 0;
  while ((serial_ptr_->available() > 0) && (bytes_read++ < datagram_size))
  {
    byte = serial_ptr_->read();
  }
}

void TMC2209::write(uint8_t register_address,
  uint32_t data)
{
  WriteReadReplyDatagram write_datagram;
  write_datagram.bytes = 0;
  write_datagram.sync = SYNC;
  write_datagram.uart_address = uart_address_;
  write_datagram.register_address = register_address;
  write_datagram.rw = RW_WRITE;
  write_datagram.data = reverseData(data);
  write_datagram.crc = calculateCrc(write_datagram,WRITE_READ_REPLY_DATAGRAM_SIZE);

  sendDatagram(write_datagram,WRITE_READ_REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209::read(uint8_t register_address)
{
  ReadRequestDatagram read_request_datagram;
  read_request_datagram.bytes = 0;
  read_request_datagram.sync = SYNC;
  read_request_datagram.uart_address = uart_address_;
  read_request_datagram.register_address = register_address;
  read_request_datagram.rw = RW_READ;
  read_request_datagram.crc = calculateCrc(read_request_datagram,READ_REQUEST_DATAGRAM_SIZE);

  sendDatagram(read_request_datagram,READ_REQUEST_DATAGRAM_SIZE);
  uint16_t reply_delay = 0;
  while ((serial_ptr_->available() <= 0) && (reply_delay++ < REPLY_DELAY_MAX_VALUE))
  {
    delay(1);
  }
  uint64_t byte;
  uint8_t byte_count = 0;
  WriteReadReplyDatagram read_reply_datagram;
  read_reply_datagram.bytes = 0;
  while (serial_ptr_->available() > 0)
  {
    byte = serial_ptr_->read();
    read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
  }
  return reverseData(read_reply_datagram.data);
}

uint8_t TMC2209::percentToCurrentSetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t TMC2209::currentSettingToPercent(uint8_t current_setting)
{
  uint8_t percent = map(current_setting,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX,
    PERCENT_MIN,
    PERCENT_MAX);
  return percent;
}

uint8_t TMC2209::percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t hold_delay_setting = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX);
  return hold_delay_setting;
}

uint8_t TMC2209::holdDelaySettingToPercent(uint8_t hold_delay_setting)
{
  uint8_t percent = map(hold_delay_setting,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX,
    PERCENT_MIN,
    PERCENT_MAX);
  return percent;
}

void TMC2209::setGlobalConfig()
{
  write(ADDRESS_GCONF,global_config_.bytes);
}

void TMC2209::getGlobalConfig()
{
  global_config_.bytes = read(ADDRESS_GCONF);
}

void TMC2209::setDriverCurrent()
{
  write(ADDRESS_IHOLD_IRUN,driver_current_.bytes);
}

void TMC2209::setChopperConfig()
{
  write(ADDRESS_CHOPCONF,chopper_config_.bytes);
}

void TMC2209::getChopperConfig()
{
  chopper_config_.bytes = read(ADDRESS_CHOPCONF);
}

void TMC2209::setPwmConfig()
{
  write(ADDRESS_PWMCONF,pwm_config_.bytes);
}

void TMC2209::getPwmConfig()
{
  pwm_config_.bytes = read(ADDRESS_PWMCONF);
}
