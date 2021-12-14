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
  debug_stream_ptr_ = nullptr;

  global_config_.bytes = 0;

  driver_current_.bytes = 0;

  chopper_config_.bytes = 0;
  chopper_config_.toff = TOFF_DEFAULT;
  chopper_config_.hstrt = HSTRT_DEFAULT;
  chopper_config_.hend = HEND_DEFAULT;
  microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;

  pwm_config_.bytes = 0;
  pwm_config_.pwm_ampl = PWM_AMPL_DEFAULT;
  pwm_config_.pwm_grad = PWM_GRAD_DEFAULT;
  pwm_config_.pwm_freq = PWM_FREQ_DEFAULT;
  pwm_config_.pwm_autoscale = PWM_AUTOSCALE_DEFAULT;
}

void TMC2209::setEnablePin(size_t enable_pin)
{
  enable_pin_ = enable_pin;

  pinMode(enable_pin_,OUTPUT);
  disable();
}

void TMC2209::setOperationModeToUart(HardwareSerial & serial,
  UartAddress uart_address)
{
  serial_ptr_ = &serial;
  uart_address_ = uart_address;

  serial_ptr_->begin(SERIAL_BAUD_RATE);

  global_config_.i_scale_analog = 0;
  global_config_.internal_rsense = 0;
  global_config_.pdn_disable = 1;
  global_config_.mstep_reg_select = 1;

  setGlobalConfig();
}

void TMC2209::setOperationModeToStandalone()
{
  serial_ptr_ = nullptr;

  global_config_.i_scale_analog = 1;
  global_config_.internal_rsense = 1;
  global_config_.pdn_disable = 0;
  global_config_.mstep_reg_select = 0;

  setGlobalConfig();
}

void TMC2209::setDebugOn(Stream & debug_stream)
{
  debug_stream_ptr_ = &debug_stream;
}

void TMC2209::setDebugOff()
{
  debug_stream_ptr_ = nullptr;
}

void TMC2209::test()
{
  DriverCurrent driver_current;
  driver_current.ihold = 0x05;
  driver_current.irun = 0x14;
  driver_current.iholddelay = 0x01;
  write(ADDRESS_IHOLD_IRUN,driver_current.bytes);
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

void TMC2209::initialize()
{
  setMicrostepsPerStep(256);
  enableStealthChop();
  setPwmThreshold(TPWMTHRS_DEFAULT);
  setPwmConfig();
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

void TMC2209::setMicrostepsPerStep(size_t microsteps_per_step)
{
  size_t microsteps_per_step_shifted = constrain(microsteps_per_step,
    MICROSTEPS_PER_STEP_MIN,
    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step >> 1;
  size_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  setMicrostepsPerStepPowerOfTwo(exponent);
}

size_t TMC2209::getMicrostepsPerStep()
{
  return 1 << microsteps_per_step_exponent_;
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
  drive_status.uint32 = read(ADDRESS_DRV_STATUS);
  return drive_status.fields.status;
}

void TMC2209::enableAnalogInputCurrentScaling()
{
  global_config_.i_scale_analog = 1;
  setGlobalConfig();
}

void TMC2209::disableAnalogInputCurrentScaling()
{
  global_config_.i_scale_analog = 0;
  setGlobalConfig();
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
  global_config_.en_pwm_mode = 1;
  setGlobalConfig();
}

void TMC2209::disableStealthChop()
{
  global_config_.en_pwm_mode = 0;
  setGlobalConfig();
}

void TMC2209::enableAutomaticCurrentScaling()
{
  pwm_config_.pwm_autoscale = PWM_AUTOSCALE_ENABLED;
  pwm_config_.pwm_ampl = PWM_AMPL_DEFAULT;
  pwm_config_.pwm_grad = PWM_GRAD_DEFAULT;
  setPwmConfig();
}

void TMC2209::disableAutomaticCurrentScaling()
{
  pwm_config_.pwm_autoscale = PWM_AUTOSCALE_DISABLED;
  pwm_config_.pwm_ampl = PWM_AMPL_MIN;
  pwm_config_.pwm_grad = PWM_GRAD_MIN;
  setPwmConfig();
}

void TMC2209::setZeroHoldCurrentMode(TMC2209::ZeroHoldCurrentMode mode)
{
  pwm_config_.freewheel = mode;
  setPwmConfig();
}

void TMC2209::setPwmOffset(uint8_t pwm_amplitude)
{
  uint8_t pwm_ampl = pwmAmplitudeToPwmAmpl(pwm_amplitude);
  pwm_config_.pwm_ampl = pwm_ampl;
  setPwmConfig();
}

void TMC2209::setPwmGradient(uint8_t pwm_amplitude)
{
  uint8_t pwm_grad = pwmAmplitudeToPwmGrad(pwm_amplitude);
  pwm_config_.pwm_grad = pwm_grad;
  setPwmConfig();
}

uint8_t TMC2209::getPwmScale()
{
  return read(ADDRESS_PWM_SCALE);
}

TMC2209::Settings TMC2209::getSettings()
{
  Settings settings;
  settings.stealth_chop_enabled = global_config_.en_pwm_mode;
  settings.automatic_current_scaling_enabled = pwm_config_.pwm_autoscale;
  settings.zero_hold_current_mode = pwm_config_.freewheel;
  settings.pwm_offset = pwm_config_.pwm_ampl;
  settings.pwm_gradient = pwm_config_.pwm_grad;
  settings.irun = driver_current_.irun;
  settings.ihold = driver_current_.ihold;
  settings.iholddelay = driver_current_.iholddelay;

  return settings;
}

// private
// void TMC2209::setStepDirInput()
// {
// }

// void TMC2209::setSpiInput()
// {
// }

void TMC2209::setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
{
  microsteps_per_step_exponent_ = exponent;

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
      microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;
      chopper_config_.mres = MRES_256;
      break;
    }
  }
  setChopperConfig();
}

// uint32_t TMC2209::sendReceivePrevious(TMC2209::WriteReadReplyDatagram & mosi_datagram)
// {
  // MisoDatagram miso_datagram;
  // miso_datagram.uint64 = 0;

  // spiBeginTransaction();
  // for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  // {
  //   uint8_t byte_write = (mosi_datagram.uint64 >> (8*i)) & 0xff;
  //   uint8_t byte_read = SPI.transfer(byte_write);
  //   miso_datagram.uint64 |= byte_read << (8*i);
  // }
  // spiEndTransaction();

  // noInterrupts();
  // spi_status_ = miso_datagram.fields.spi_status;
  // interrupts();

  // return miso_datagram.fields.data;
//   return 0;
// }

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
  uint8_t current_byte;
  for (uint8_t i=0; i<(datagram_size - 1); ++i)
  {
    current_byte = (datagram.bytes >> (i*BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    for (uint8_t j=0; j<BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (current_byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      current_byte = current_byte >> 1;
    }
  }
  return crc;
}

template<typename Datagram>
void TMC2209::sendDatagram(Datagram & datagram,
  uint8_t datagram_size)
{
  uint8_t current_byte;
  for (uint8_t i=0; i<datagram_size; ++i)
  {
    current_byte = (datagram.bytes >> (i*BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    // serial_ptr_->write(current_byte);
    if (debug_stream_ptr_)
    {
      debug_stream_ptr_->println(current_byte,BIN);
    }
  }
}

uint32_t TMC2209::write(uint8_t register_address,
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
  return 0;
}

uint32_t TMC2209::read(uint8_t register_address)
{
  // WriteReadReplyDatagram mosi_datagram;
  // mosi_datagram.uint64 = 0;
  // mosi_datagram.fields.rw = RW_READ;
  // mosi_datagram.fields.address = address;

  // // must read twice to get value at address
  // sendReceivePrevious(mosi_datagram);
  // uint32_t data = sendReceivePrevious(mosi_datagram);
  // return data;
  return 0;
}

uint8_t TMC2209::percentToCurrentSetting(uint8_t percent)
{
  uint8_t current_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map(current_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t TMC2209::percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t hold_delay_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t hold_delay = map(hold_delay_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX);
  return hold_delay;
}

uint8_t TMC2209::pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude)
{
  uint8_t pwm_ampl = pwm_amplitude;
  if (pwm_config_.pwm_autoscale)
  {
    pwm_ampl = constrain(pwm_ampl,
      PWM_AMPL_AUTOSCALE_MIN,
      PWM_AMPL_AUTOSCALE_MAX);
  }
  return pwm_ampl;
}

uint8_t TMC2209::pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude)
{
  uint8_t pwm_grad = pwm_amplitude;
  if (pwm_config_.pwm_autoscale)
  {
    pwm_grad = constrain(pwm_grad,
      PWM_GRAD_AUTOSCALE_MIN,
      PWM_GRAD_AUTOSCALE_MAX);
  }
  return pwm_grad;
}

void TMC2209::setGlobalConfig()
{
  write(ADDRESS_GCONF,global_config_.bytes);
}

void TMC2209::setDriverCurrent()
{
  write(ADDRESS_IHOLD_IRUN,driver_current_.bytes);
}

void TMC2209::setChopperConfig()
{
  write(ADDRESS_CHOPCONF,chopper_config_.bytes);
}

void TMC2209::setPwmThreshold(uint32_t value)
{
  write(ADDRESS_TPWMTHRS,value);
}

void TMC2209::setPwmConfig()
{
  write(ADDRESS_PWMCONF,pwm_config_.bytes);
}

void TMC2209::enableClockSelect()
{
  // digitalWrite(chip_select_pin_,LOW);
}

void TMC2209::disableClockSelect()
{
  // digitalWrite(chip_select_pin_,HIGH);
}
void TMC2209::spiBeginTransaction()
{
  // SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  // enableClockSelect();
}

void TMC2209::spiEndTransaction()
{
  // disableClockSelect();
  // SPI.endTransaction();
}
