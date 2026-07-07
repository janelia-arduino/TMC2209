// ----------------------------------------------------------------------------
// tmc2209.c
//
// Portable C17 implementation of the TMC2209 stepper motor driver library.
//
// Authors:
// Peter Polidoro peter@polidoro.io (original Arduino C++ implementation)
// ----------------------------------------------------------------------------

#include "tmc2209.h"

// Serial settings
#define TMC2209_BYTE_MAX_VALUE 0xFFu
#define TMC2209_BITS_PER_BYTE 8u

#define TMC2209_ECHO_DELAY_INC_MICROSECONDS 1u
#define TMC2209_ECHO_DELAY_MAX_MICROSECONDS 4000u

#define TMC2209_REPLY_DELAY_INC_MICROSECONDS 1u
#define TMC2209_REPLY_DELAY_MAX_MICROSECONDS 10000u

#define TMC2209_STEPPER_DRIVER_FEATURE_OFF 0u
#define TMC2209_STEPPER_DRIVER_FEATURE_ON 1u

#define TMC2209_MAX_READ_RETRIES 5u
#define TMC2209_READ_RETRY_DELAY_MS 20u

// Datagrams
//
// Write datagram (8 bytes):
// sync, serial address, register address | write bit, data MSB first, crc
// Read request datagram (4 bytes):
// sync, serial address, register address, crc
// Read reply datagram (8 bytes):
// sync, 0xFF, register address, data MSB first, crc
#define TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE 8u
#define TMC2209_READ_REQUEST_DATAGRAM_SIZE 4u

#define TMC2209_SYNC 0x05u
#define TMC2209_RW_WRITE_BIT 0x80u

#define TMC2209_DATAGRAM_INDEX_SYNC 0u
#define TMC2209_DATAGRAM_INDEX_SERIAL_ADDRESS 1u
#define TMC2209_DATAGRAM_INDEX_REGISTER_ADDRESS 2u
#define TMC2209_DATAGRAM_INDEX_DATA 3u

// General Configuration Registers
#define TMC2209_ADDRESS_GCONF 0x00u
#define TMC2209_ADDRESS_GSTAT 0x01u
#define TMC2209_ADDRESS_IFCNT 0x02u
#define TMC2209_ADDRESS_REPLYDELAY 0x03u
#define TMC2209_ADDRESS_IOIN 0x06u

#define TMC2209_VERSION 0x21u

#define TMC2209_REPLYDELAY_SHIFT 8u
#define TMC2209_IOIN_ENN_MASK 0x00000001u
#define TMC2209_IOIN_VERSION_SHIFT 24u

// Velocity Dependent Driver Feature Control Register Set
#define TMC2209_ADDRESS_IHOLD_IRUN 0x10u
#define TMC2209_PERCENT_MIN 0u
#define TMC2209_PERCENT_MAX 100u
#define TMC2209_CURRENT_SETTING_MIN 0u
#define TMC2209_CURRENT_SETTING_MAX 31u
#define TMC2209_HOLD_DELAY_MIN 0u
#define TMC2209_HOLD_DELAY_MAX 15u
#define TMC2209_IHOLD_DEFAULT 16u
#define TMC2209_IRUN_DEFAULT 31u
#define TMC2209_IHOLDDELAY_DEFAULT 1u

#define TMC2209_ADDRESS_TPOWERDOWN 0x11u
#define TMC2209_TPOWERDOWN_DEFAULT 20u

#define TMC2209_ADDRESS_TSTEP 0x12u

#define TMC2209_ADDRESS_TPWMTHRS 0x13u
#define TMC2209_TPWMTHRS_DEFAULT 0u

#define TMC2209_ADDRESS_VACTUAL 0x22u
#define TMC2209_VACTUAL_DEFAULT 0
#define TMC2209_VACTUAL_STEP_DIR_INTERFACE 0

// CoolStep and StallGuard Control Register Set
#define TMC2209_ADDRESS_TCOOLTHRS 0x14u
#define TMC2209_TCOOLTHRS_DEFAULT 0u
#define TMC2209_ADDRESS_SGTHRS 0x40u
#define TMC2209_SGTHRS_DEFAULT 0u
#define TMC2209_ADDRESS_SG_RESULT 0x41u
#define TMC2209_ADDRESS_COOLCONF 0x42u
#define TMC2209_COOLCONF_DEFAULT 0u

#define TMC2209_SEIMIN_UPPER_CURRENT_LIMIT 20u
#define TMC2209_SEIMIN_LOWER_SETTING 0u
#define TMC2209_SEIMIN_UPPER_SETTING 1u
#define TMC2209_SEMIN_OFF 0u
#define TMC2209_SEMIN_MIN 1u
#define TMC2209_SEMIN_MAX 15u
#define TMC2209_SEMAX_MIN 0u
#define TMC2209_SEMAX_MAX 15u

// Microstepping Control Register Set
#define TMC2209_ADDRESS_MSCNT 0x6Au
#define TMC2209_ADDRESS_MSCURACT 0x6Bu

// Driver Register Set
#define TMC2209_ADDRESS_CHOPCONF 0x6Cu
#define TMC2209_CHOPPER_CONFIG_DEFAULT 0x10000053u
#define TMC2209_TBL_DEFAULT 0x2u
#define TMC2209_HEND_DEFAULT 0u
#define TMC2209_HSTART_DEFAULT 5u
#define TMC2209_TOFF_DEFAULT 3u
#define TMC2209_TOFF_DISABLE 0u
#define TMC2209_MRES_256 0x0u
#define TMC2209_MRES_128 0x1u
#define TMC2209_MRES_064 0x2u
#define TMC2209_MRES_032 0x3u
#define TMC2209_MRES_016 0x4u
#define TMC2209_MRES_008 0x5u
#define TMC2209_MRES_004 0x6u
#define TMC2209_MRES_002 0x7u
#define TMC2209_MRES_001 0x8u
#define TMC2209_DOUBLE_EDGE_DISABLE 0u
#define TMC2209_DOUBLE_EDGE_ENABLE 1u
#define TMC2209_VSENSE_DISABLE 0u
#define TMC2209_VSENSE_ENABLE 1u

#define TMC2209_MICROSTEPS_PER_STEP_MIN 1u
#define TMC2209_MICROSTEPS_PER_STEP_MAX 256u

#define TMC2209_ADDRESS_DRV_STATUS 0x6Fu

#define TMC2209_ADDRESS_PWMCONF 0x70u
#define TMC2209_PWM_CONFIG_DEFAULT 0xC10D0024u

#define TMC2209_ADDRESS_PWM_SCALE 0x71u
#define TMC2209_PWM_SCALE_SUM_MASK 0xFFu
#define TMC2209_PWM_SCALE_AUTO_SHIFT 16u
#define TMC2209_PWM_SCALE_AUTO_MASK 0x1FFu
#define TMC2209_PWM_SCALE_AUTO_SIGN_BIT 0x100u

#define TMC2209_ADDRESS_PWM_AUTO 0x72u
#define TMC2209_PWM_OFFSET_AUTO_MASK 0xFFu
#define TMC2209_PWM_GRADIENT_AUTO_SHIFT 16u
#define TMC2209_PWM_GRADIENT_AUTO_MASK 0xFFu

// GSTAT bits
#define TMC2209_GSTAT_RESET 0x1u
#define TMC2209_GSTAT_DRV_ERR 0x2u

// ----------------------------------------------------------------------------
// Private helper functions
// ----------------------------------------------------------------------------

static uint32_t tmc2209_constrain(uint32_t value, uint32_t low, uint32_t high)
{
  return (value < low) ? low : ((value > high) ? high : value);
}

static uint32_t tmc2209_map(uint32_t x,
  uint32_t in_min,
  uint32_t in_max,
  uint32_t out_min,
  uint32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void tmc2209_serial_write(tmc2209_t * tmc2209,
  uint8_t const * data,
  size_t size)
{
  tmc2209->hal.serial_write(tmc2209->hal.context, data, size);
}

static size_t tmc2209_serial_available(tmc2209_t * tmc2209)
{
  return tmc2209->hal.serial_available(tmc2209->hal.context);
}

static int16_t tmc2209_serial_read(tmc2209_t * tmc2209)
{
  return tmc2209->hal.serial_read(tmc2209->hal.context);
}

static void tmc2209_serial_flush(tmc2209_t * tmc2209)
{
  tmc2209->hal.serial_flush(tmc2209->hal.context);
}

static void tmc2209_delay_microseconds(tmc2209_t * tmc2209,
  uint32_t microseconds)
{
  tmc2209->hal.delay_microseconds(tmc2209->hal.context, microseconds);
}

static void tmc2209_delay_milliseconds(tmc2209_t * tmc2209,
  uint32_t milliseconds)
{
  tmc2209->hal.delay_milliseconds(tmc2209->hal.context, milliseconds);
}

// CRC8-ATM, polynomial 0x07, as specified in the TMC2209 datasheet.
// The CRC covers every datagram byte except the final CRC byte itself.
static uint8_t tmc2209_calculate_crc(uint8_t const * datagram,
  uint8_t datagram_size)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < (uint8_t)(datagram_size - 1u); ++i)
  {
    uint8_t byte = datagram[i];
    for (uint8_t j = 0; j < TMC2209_BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7u) ^ (byte & 0x01u))
      {
        crc = (uint8_t)((crc << 1u) ^ 0x07u);
      }
      else
      {
        crc = (uint8_t)(crc << 1u);
      }
      byte = byte >> 1u;
    }
  }
  return crc;
}

static void tmc2209_send_datagram_unidirectional(tmc2209_t * tmc2209,
  uint8_t const * datagram,
  uint8_t datagram_size)
{
  tmc2209_serial_write(tmc2209, datagram, datagram_size);
}

static void tmc2209_send_datagram_bidirectional(tmc2209_t * tmc2209,
  uint8_t const * datagram,
  uint8_t datagram_size)
{
  // Wait for the transmission of outgoing serial data to complete
  tmc2209_serial_flush(tmc2209);

  // clear the serial receive buffer if necessary
  while (tmc2209_serial_available(tmc2209) > 0u)
  {
    (void)tmc2209_serial_read(tmc2209);
  }

  // write datagram
  tmc2209_serial_write(tmc2209, datagram, datagram_size);

  // Wait for the transmission of outgoing serial data to complete
  tmc2209_serial_flush(tmc2209);

  // wait for bytes sent out on TX line to be echoed on RX line
  uint32_t echo_delay = 0;
  while ((tmc2209_serial_available(tmc2209) < datagram_size) &&
    (echo_delay < TMC2209_ECHO_DELAY_MAX_MICROSECONDS))
  {
    tmc2209_delay_microseconds(tmc2209, TMC2209_ECHO_DELAY_INC_MICROSECONDS);
    echo_delay += TMC2209_ECHO_DELAY_INC_MICROSECONDS;
  }

  if (echo_delay >= TMC2209_ECHO_DELAY_MAX_MICROSECONDS)
  {
    return;
  }

  // clear RX buffer of echo bytes
  for (uint8_t i = 0; i < datagram_size; ++i)
  {
    (void)tmc2209_serial_read(tmc2209);
  }
}

static void tmc2209_write_register(tmc2209_t * tmc2209,
  uint8_t register_address,
  uint32_t data)
{
  uint8_t datagram[TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE];
  datagram[TMC2209_DATAGRAM_INDEX_SYNC] = TMC2209_SYNC;
  datagram[TMC2209_DATAGRAM_INDEX_SERIAL_ADDRESS] = tmc2209->serial_address;
  datagram[TMC2209_DATAGRAM_INDEX_REGISTER_ADDRESS] =
    (uint8_t)(register_address | TMC2209_RW_WRITE_BIT);
  datagram[TMC2209_DATAGRAM_INDEX_DATA + 0u] = (uint8_t)(data >> 24u);
  datagram[TMC2209_DATAGRAM_INDEX_DATA + 1u] = (uint8_t)(data >> 16u);
  datagram[TMC2209_DATAGRAM_INDEX_DATA + 2u] = (uint8_t)(data >> 8u);
  datagram[TMC2209_DATAGRAM_INDEX_DATA + 3u] = (uint8_t)data;
  datagram[TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE - 1u] =
    tmc2209_calculate_crc(datagram, TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE);

  tmc2209_send_datagram_unidirectional(tmc2209,
    datagram,
    TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE);
}

static uint32_t tmc2209_read_register(tmc2209_t * tmc2209,
  uint8_t register_address)
{
  uint8_t request_datagram[TMC2209_READ_REQUEST_DATAGRAM_SIZE];
  request_datagram[TMC2209_DATAGRAM_INDEX_SYNC] = TMC2209_SYNC;
  request_datagram[TMC2209_DATAGRAM_INDEX_SERIAL_ADDRESS] =
    tmc2209->serial_address;
  request_datagram[TMC2209_DATAGRAM_INDEX_REGISTER_ADDRESS] =
    register_address;
  request_datagram[TMC2209_READ_REQUEST_DATAGRAM_SIZE - 1u] =
    tmc2209_calculate_crc(request_datagram,
      TMC2209_READ_REQUEST_DATAGRAM_SIZE);

  for (uint8_t retry = 0; retry < TMC2209_MAX_READ_RETRIES; ++retry)
  {
    tmc2209_send_datagram_bidirectional(tmc2209,
      request_datagram,
      TMC2209_READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_delay = 0;
    while ((tmc2209_serial_available(tmc2209) <
      TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE) &&
      (reply_delay < TMC2209_REPLY_DELAY_MAX_MICROSECONDS))
    {
      tmc2209_delay_microseconds(tmc2209,
        TMC2209_REPLY_DELAY_INC_MICROSECONDS);
      reply_delay += TMC2209_REPLY_DELAY_INC_MICROSECONDS;
    }

    if (reply_delay >= TMC2209_REPLY_DELAY_MAX_MICROSECONDS)
    {
      return 0;
    }

    uint8_t reply_datagram[TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE];
    for (uint8_t i = 0; i < TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
    {
      reply_datagram[i] = (uint8_t)tmc2209_serial_read(tmc2209);
    }

    uint8_t crc = tmc2209_calculate_crc(reply_datagram,
      TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE);
    if (crc == reply_datagram[TMC2209_WRITE_READ_REPLY_DATAGRAM_SIZE - 1u])
    {
      return ((uint32_t)reply_datagram[TMC2209_DATAGRAM_INDEX_DATA + 0u] << 24u) |
        ((uint32_t)reply_datagram[TMC2209_DATAGRAM_INDEX_DATA + 1u] << 16u) |
        ((uint32_t)reply_datagram[TMC2209_DATAGRAM_INDEX_DATA + 2u] << 8u) |
        (uint32_t)reply_datagram[TMC2209_DATAGRAM_INDEX_DATA + 3u];
    }

    tmc2209_delay_milliseconds(tmc2209, TMC2209_READ_RETRY_DELAY_MS);
  }

  return 0;
}

static uint8_t tmc2209_percent_to_current_setting(uint8_t percent)
{
  uint8_t constrained_percent = (uint8_t)tmc2209_constrain(percent,
    TMC2209_PERCENT_MIN,
    TMC2209_PERCENT_MAX);
  return (uint8_t)tmc2209_map(constrained_percent,
    TMC2209_PERCENT_MIN,
    TMC2209_PERCENT_MAX,
    TMC2209_CURRENT_SETTING_MIN,
    TMC2209_CURRENT_SETTING_MAX);
}

static uint8_t tmc2209_current_setting_to_percent(uint8_t current_setting)
{
  return (uint8_t)tmc2209_map(current_setting,
    TMC2209_CURRENT_SETTING_MIN,
    TMC2209_CURRENT_SETTING_MAX,
    TMC2209_PERCENT_MIN,
    TMC2209_PERCENT_MAX);
}

static uint8_t tmc2209_percent_to_hold_delay_setting(uint8_t percent)
{
  uint8_t constrained_percent = (uint8_t)tmc2209_constrain(percent,
    TMC2209_PERCENT_MIN,
    TMC2209_PERCENT_MAX);
  return (uint8_t)tmc2209_map(constrained_percent,
    TMC2209_PERCENT_MIN,
    TMC2209_PERCENT_MAX,
    TMC2209_HOLD_DELAY_MIN,
    TMC2209_HOLD_DELAY_MAX);
}

static uint8_t tmc2209_hold_delay_setting_to_percent(
  uint8_t hold_delay_setting)
{
  return (uint8_t)tmc2209_map(hold_delay_setting,
    TMC2209_HOLD_DELAY_MIN,
    TMC2209_HOLD_DELAY_MAX,
    TMC2209_PERCENT_MIN,
    TMC2209_PERCENT_MAX);
}

static void tmc2209_write_stored_global_config(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_GCONF,
    tmc2209->global_config.bytes);
}

static uint32_t tmc2209_read_global_config_bytes(tmc2209_t * tmc2209)
{
  return tmc2209_read_register(tmc2209, TMC2209_ADDRESS_GCONF);
}

static void tmc2209_write_stored_driver_current(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_IHOLD_IRUN,
    tmc2209->driver_current.bytes);

  if (tmc2209->driver_current.irun >= TMC2209_SEIMIN_UPPER_CURRENT_LIMIT)
  {
    tmc2209->cool_config.seimin = TMC2209_SEIMIN_UPPER_SETTING;
  }
  else
  {
    tmc2209->cool_config.seimin = TMC2209_SEIMIN_LOWER_SETTING;
  }
  if (tmc2209->cool_step_enabled)
  {
    tmc2209_write_register(tmc2209,
      TMC2209_ADDRESS_COOLCONF,
      tmc2209->cool_config.bytes);
  }
}

static void tmc2209_write_stored_chopper_config(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_CHOPCONF,
    tmc2209->chopper_config.bytes);
}

static uint32_t tmc2209_read_chopper_config_bytes(tmc2209_t * tmc2209)
{
  return tmc2209_read_register(tmc2209, TMC2209_ADDRESS_CHOPCONF);
}

static void tmc2209_write_stored_pwm_config(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_PWMCONF,
    tmc2209->pwm_config.bytes);
}

static uint32_t tmc2209_read_pwm_config_bytes(tmc2209_t * tmc2209)
{
  return tmc2209_read_register(tmc2209, TMC2209_ADDRESS_PWMCONF);
}

static void tmc2209_set_operation_mode_to_serial(tmc2209_t * tmc2209,
  tmc2209_serial_address_t serial_address)
{
  tmc2209->serial_address = (uint8_t)serial_address;

  tmc2209->global_config.bytes = 0;
  tmc2209->global_config.i_scale_analog = 0;
  tmc2209->global_config.pdn_disable = 1;
  tmc2209->global_config.mstep_reg_select = 1;
  tmc2209->global_config.multistep_filt = 1;

  tmc2209_write_stored_global_config(tmc2209);
}

static void tmc2209_set_registers_to_defaults(tmc2209_t * tmc2209)
{
  tmc2209->driver_current.bytes = 0;
  tmc2209->driver_current.ihold = TMC2209_IHOLD_DEFAULT;
  tmc2209->driver_current.irun = TMC2209_IRUN_DEFAULT;
  tmc2209->driver_current.iholddelay = TMC2209_IHOLDDELAY_DEFAULT;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_IHOLD_IRUN,
    tmc2209->driver_current.bytes);

  tmc2209->chopper_config.bytes = TMC2209_CHOPPER_CONFIG_DEFAULT;
  tmc2209->chopper_config.tbl = TMC2209_TBL_DEFAULT;
  tmc2209->chopper_config.hend = TMC2209_HEND_DEFAULT;
  tmc2209->chopper_config.hstart = TMC2209_HSTART_DEFAULT;
  tmc2209->chopper_config.toff = TMC2209_TOFF_DEFAULT;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_CHOPCONF,
    tmc2209->chopper_config.bytes);

  tmc2209->pwm_config.bytes = TMC2209_PWM_CONFIG_DEFAULT;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_PWMCONF,
    tmc2209->pwm_config.bytes);

  tmc2209->cool_config.bytes = TMC2209_COOLCONF_DEFAULT;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_COOLCONF,
    tmc2209->cool_config.bytes);

  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_TPOWERDOWN,
    TMC2209_TPOWERDOWN_DEFAULT);
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_TPWMTHRS,
    TMC2209_TPWMTHRS_DEFAULT);
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_VACTUAL,
    (uint32_t)TMC2209_VACTUAL_DEFAULT);
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_TCOOLTHRS,
    TMC2209_TCOOLTHRS_DEFAULT);
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_SGTHRS,
    TMC2209_SGTHRS_DEFAULT);
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_COOLCONF,
    TMC2209_COOLCONF_DEFAULT);
}

static void tmc2209_read_and_store_registers(tmc2209_t * tmc2209)
{
  tmc2209->global_config.bytes = tmc2209_read_global_config_bytes(tmc2209);
  tmc2209->chopper_config.bytes = tmc2209_read_chopper_config_bytes(tmc2209);
  tmc2209->pwm_config.bytes = tmc2209_read_pwm_config_bytes(tmc2209);
}

static bool tmc2209_serial_operation_mode(tmc2209_t * tmc2209)
{
  tmc2209_global_config_t global_config;
  global_config.bytes = tmc2209_read_global_config_bytes(tmc2209);

  return global_config.pdn_disable;
}

static void tmc2209_minimize_motor_current(tmc2209_t * tmc2209)
{
  tmc2209->driver_current.irun = TMC2209_CURRENT_SETTING_MIN;
  tmc2209->driver_current.ihold = TMC2209_CURRENT_SETTING_MIN;
  tmc2209_write_stored_driver_current(tmc2209);
}

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------

void tmc2209_setup(tmc2209_t * tmc2209,
  tmc2209_hal_t const * hal,
  tmc2209_serial_address_t serial_address)
{
  tmc2209->hal = *hal;
  tmc2209->serial_address = (uint8_t)serial_address;
  tmc2209->cool_step_enabled = false;
  tmc2209->toff = TMC2209_TOFF_DEFAULT;
  tmc2209->global_config.bytes = 0;
  tmc2209->driver_current.bytes = 0;
  tmc2209->cool_config.bytes = 0;
  tmc2209->chopper_config.bytes = 0;
  tmc2209->pwm_config.bytes = 0;

  tmc2209_set_operation_mode_to_serial(tmc2209, serial_address);
  tmc2209_set_registers_to_defaults(tmc2209);
  tmc2209_clear_drive_error(tmc2209);

  tmc2209_minimize_motor_current(tmc2209);
  tmc2209_disable(tmc2209);
  tmc2209_disable_automatic_current_scaling(tmc2209);
  tmc2209_disable_automatic_gradient_adaptation(tmc2209);
}

// ----------------------------------------------------------------------------
// Unidirectional methods
// ----------------------------------------------------------------------------

void tmc2209_enable(tmc2209_t * tmc2209)
{
  if (tmc2209->hal.set_hardware_enable_pin != NULL)
  {
    tmc2209->hal.set_hardware_enable_pin(tmc2209->hal.context, true);
  }
  tmc2209->chopper_config.toff = tmc2209->toff;
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_disable(tmc2209_t * tmc2209)
{
  if (tmc2209->hal.set_hardware_enable_pin != NULL)
  {
    tmc2209->hal.set_hardware_enable_pin(tmc2209->hal.context, false);
  }
  tmc2209->chopper_config.toff = TMC2209_TOFF_DISABLE;
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_set_microsteps_per_step(tmc2209_t * tmc2209,
  uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted =
    (uint16_t)(tmc2209_constrain(microsteps_per_step,
      TMC2209_MICROSTEPS_PER_STEP_MIN,
      TMC2209_MICROSTEPS_PER_STEP_MAX) >> 1u);
  uint8_t exponent = 0;
  while (microsteps_per_step_shifted > 0u)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1u;
    ++exponent;
  }
  tmc2209_set_microsteps_per_step_power_of_two(tmc2209, exponent);
}

void tmc2209_set_microsteps_per_step_power_of_two(tmc2209_t * tmc2209,
  uint8_t exponent)
{
  switch (exponent)
  {
    case 0:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_001;
      break;
    }
    case 1:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_002;
      break;
    }
    case 2:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_004;
      break;
    }
    case 3:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_008;
      break;
    }
    case 4:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_016;
      break;
    }
    case 5:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_032;
      break;
    }
    case 6:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_064;
      break;
    }
    case 7:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_128;
      break;
    }
    case 8:
    default:
    {
      tmc2209->chopper_config.mres = TMC2209_MRES_256;
      break;
    }
  }
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_set_run_current(tmc2209_t * tmc2209, uint8_t percent)
{
  tmc2209->driver_current.irun = tmc2209_percent_to_current_setting(percent);
  tmc2209_write_stored_driver_current(tmc2209);
}

void tmc2209_set_hold_current(tmc2209_t * tmc2209, uint8_t percent)
{
  tmc2209->driver_current.ihold = tmc2209_percent_to_current_setting(percent);
  tmc2209_write_stored_driver_current(tmc2209);
}

void tmc2209_set_hold_delay(tmc2209_t * tmc2209, uint8_t percent)
{
  tmc2209->driver_current.iholddelay =
    tmc2209_percent_to_hold_delay_setting(percent);
  tmc2209_write_stored_driver_current(tmc2209);
}

void tmc2209_set_all_current_values(tmc2209_t * tmc2209,
  uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent)
{
  tmc2209->driver_current.irun =
    tmc2209_percent_to_current_setting(run_current_percent);
  tmc2209->driver_current.ihold =
    tmc2209_percent_to_current_setting(hold_current_percent);
  tmc2209->driver_current.iholddelay =
    tmc2209_percent_to_hold_delay_setting(hold_delay_percent);
  tmc2209_write_stored_driver_current(tmc2209);
}

void tmc2209_set_rms_current(tmc2209_t * tmc2209,
  uint16_t milliamps,
  float r_sense,
  float hold_multiplier)
{
  // Taken from https://github.com/teemuatlut/TMCStepper/blob/74e8e6881adc9241c2e626071e7328d7652f361a/src/source/TMCStepper.cpp#L41.

  float cs_float =
    32.0f * 1.41421f * (float)milliamps / 1000.0f * (r_sense + 0.02f) / 0.325f - 1.0f;
  // If Current Scale is too low, turn on high sensitivity R_sense and
  // calculate again
  if (cs_float < 16.0f)
  {
    tmc2209_enable_vsense(tmc2209);
    cs_float =
      32.0f * 1.41421f * (float)milliamps / 1000.0f * (r_sense + 0.02f) / 0.180f - 1.0f;
  }
  else
  {
    // If CS >= 16, turn off high_sense_r
    tmc2209_disable_vsense(tmc2209);
  }

  if (cs_float < 0.0f)
  {
    cs_float = 0.0f;
  }
  if (cs_float > 31.0f)
  {
    cs_float = 31.0f;
  }
  uint8_t cs = (uint8_t)cs_float;

  tmc2209->driver_current.irun = cs;
  tmc2209->driver_current.ihold = (uint8_t)((float)cs * hold_multiplier);
  tmc2209_write_stored_driver_current(tmc2209);
}

void tmc2209_enable_double_edge(tmc2209_t * tmc2209)
{
  tmc2209->chopper_config.double_edge = TMC2209_DOUBLE_EDGE_ENABLE;
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_disable_double_edge(tmc2209_t * tmc2209)
{
  tmc2209->chopper_config.double_edge = TMC2209_DOUBLE_EDGE_DISABLE;
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_enable_vsense(tmc2209_t * tmc2209)
{
  tmc2209->chopper_config.vsense = TMC2209_VSENSE_ENABLE;
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_disable_vsense(tmc2209_t * tmc2209)
{
  tmc2209->chopper_config.vsense = TMC2209_VSENSE_DISABLE;
  tmc2209_write_stored_chopper_config(tmc2209);
}

void tmc2209_enable_inverse_motor_direction(tmc2209_t * tmc2209)
{
  tmc2209->global_config.shaft = 1;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_disable_inverse_motor_direction(tmc2209_t * tmc2209)
{
  tmc2209->global_config.shaft = 0;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_set_standstill_mode(tmc2209_t * tmc2209,
  tmc2209_standstill_mode_t mode)
{
  tmc2209->pwm_config.freewheel = (uint32_t)mode;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_enable_automatic_current_scaling(tmc2209_t * tmc2209)
{
  tmc2209->pwm_config.pwm_autoscale = TMC2209_STEPPER_DRIVER_FEATURE_ON;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_disable_automatic_current_scaling(tmc2209_t * tmc2209)
{
  tmc2209->pwm_config.pwm_autoscale = TMC2209_STEPPER_DRIVER_FEATURE_OFF;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_enable_automatic_gradient_adaptation(tmc2209_t * tmc2209)
{
  tmc2209->pwm_config.pwm_autograd = TMC2209_STEPPER_DRIVER_FEATURE_ON;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_disable_automatic_gradient_adaptation(tmc2209_t * tmc2209)
{
  tmc2209->pwm_config.pwm_autograd = TMC2209_STEPPER_DRIVER_FEATURE_OFF;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_set_pwm_offset(tmc2209_t * tmc2209, uint8_t pwm_amplitude)
{
  tmc2209->pwm_config.pwm_offset = pwm_amplitude;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_set_pwm_gradient(tmc2209_t * tmc2209, uint8_t pwm_amplitude)
{
  tmc2209->pwm_config.pwm_grad = pwm_amplitude;
  tmc2209_write_stored_pwm_config(tmc2209);
}

void tmc2209_set_power_down_delay(tmc2209_t * tmc2209,
  uint8_t power_down_delay)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_TPOWERDOWN,
    power_down_delay);
}

void tmc2209_set_reply_delay(tmc2209_t * tmc2209, uint8_t reply_delay)
{
  if (reply_delay > TMC2209_REPLY_DELAY_MAX)
  {
    reply_delay = TMC2209_REPLY_DELAY_MAX;
  }
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_REPLYDELAY,
    (uint32_t)reply_delay << TMC2209_REPLYDELAY_SHIFT);
}

void tmc2209_move_at_velocity(tmc2209_t * tmc2209,
  int32_t microsteps_per_period)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_VACTUAL,
    (uint32_t)microsteps_per_period);
}

void tmc2209_move_using_step_dir_interface(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_VACTUAL,
    (uint32_t)TMC2209_VACTUAL_STEP_DIR_INTERFACE);
}

void tmc2209_enable_stealth_chop(tmc2209_t * tmc2209)
{
  tmc2209->global_config.enable_spread_cycle = 0;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_disable_stealth_chop(tmc2209_t * tmc2209)
{
  tmc2209->global_config.enable_spread_cycle = 1;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_set_stealth_chop_duration_threshold(tmc2209_t * tmc2209,
  uint32_t duration_threshold)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_TPWMTHRS,
    duration_threshold);
}

void tmc2209_set_stall_guard_threshold(tmc2209_t * tmc2209,
  uint8_t stall_guard_threshold)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_SGTHRS,
    stall_guard_threshold);
}

void tmc2209_enable_cool_step(tmc2209_t * tmc2209,
  uint8_t lower_threshold,
  uint8_t upper_threshold)
{
  lower_threshold = (uint8_t)tmc2209_constrain(lower_threshold,
    TMC2209_SEMIN_MIN,
    TMC2209_SEMIN_MAX);
  tmc2209->cool_config.semin = lower_threshold;
  upper_threshold = (uint8_t)tmc2209_constrain(upper_threshold,
    TMC2209_SEMAX_MIN,
    TMC2209_SEMAX_MAX);
  tmc2209->cool_config.semax = upper_threshold;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_COOLCONF,
    tmc2209->cool_config.bytes);
  tmc2209->cool_step_enabled = true;
}

void tmc2209_disable_cool_step(tmc2209_t * tmc2209)
{
  tmc2209->cool_config.semin = TMC2209_SEMIN_OFF;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_COOLCONF,
    tmc2209->cool_config.bytes);
  tmc2209->cool_step_enabled = false;
}

void tmc2209_set_cool_step_current_increment(tmc2209_t * tmc2209,
  tmc2209_current_increment_t current_increment)
{
  tmc2209->cool_config.seup = (uint32_t)current_increment;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_COOLCONF,
    tmc2209->cool_config.bytes);
}

void tmc2209_set_cool_step_measurement_count(tmc2209_t * tmc2209,
  tmc2209_measurement_count_t measurement_count)
{
  tmc2209->cool_config.sedn = (uint32_t)measurement_count;
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_COOLCONF,
    tmc2209->cool_config.bytes);
}

void tmc2209_set_cool_step_duration_threshold(tmc2209_t * tmc2209,
  uint32_t duration_threshold)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_TCOOLTHRS,
    duration_threshold);
}

void tmc2209_enable_analog_current_scaling(tmc2209_t * tmc2209)
{
  tmc2209->global_config.i_scale_analog = 1;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_disable_analog_current_scaling(tmc2209_t * tmc2209)
{
  tmc2209->global_config.i_scale_analog = 0;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_use_external_sense_resistors(tmc2209_t * tmc2209)
{
  tmc2209->global_config.internal_rsense = 0;
  tmc2209_write_stored_global_config(tmc2209);
}

void tmc2209_use_internal_sense_resistors(tmc2209_t * tmc2209)
{
  tmc2209->global_config.internal_rsense = 1;
  tmc2209_write_stored_global_config(tmc2209);
}

// ----------------------------------------------------------------------------
// Bidirectional methods
// ----------------------------------------------------------------------------

uint8_t tmc2209_get_version(tmc2209_t * tmc2209)
{
  uint32_t input = tmc2209_read_register(tmc2209, TMC2209_ADDRESS_IOIN);

  return (uint8_t)(input >> TMC2209_IOIN_VERSION_SHIFT);
}

bool tmc2209_is_communicating(tmc2209_t * tmc2209)
{
  return (tmc2209_get_version(tmc2209) == TMC2209_VERSION);
}

bool tmc2209_is_setup_and_communicating(tmc2209_t * tmc2209)
{
  return tmc2209_serial_operation_mode(tmc2209);
}

bool tmc2209_is_communicating_but_not_setup(tmc2209_t * tmc2209)
{
  return (tmc2209_is_communicating(tmc2209) &&
    (!tmc2209_is_setup_and_communicating(tmc2209)));
}

bool tmc2209_hardware_disabled(tmc2209_t * tmc2209)
{
  uint32_t input = tmc2209_read_register(tmc2209, TMC2209_ADDRESS_IOIN);

  return ((input & TMC2209_IOIN_ENN_MASK) != 0u);
}

uint16_t tmc2209_get_microsteps_per_step(tmc2209_t * tmc2209)
{
  uint16_t microsteps_per_step_exponent;
  switch (tmc2209->chopper_config.mres)
  {
    case TMC2209_MRES_001:
    {
      microsteps_per_step_exponent = 0;
      break;
    }
    case TMC2209_MRES_002:
    {
      microsteps_per_step_exponent = 1;
      break;
    }
    case TMC2209_MRES_004:
    {
      microsteps_per_step_exponent = 2;
      break;
    }
    case TMC2209_MRES_008:
    {
      microsteps_per_step_exponent = 3;
      break;
    }
    case TMC2209_MRES_016:
    {
      microsteps_per_step_exponent = 4;
      break;
    }
    case TMC2209_MRES_032:
    {
      microsteps_per_step_exponent = 5;
      break;
    }
    case TMC2209_MRES_064:
    {
      microsteps_per_step_exponent = 6;
      break;
    }
    case TMC2209_MRES_128:
    {
      microsteps_per_step_exponent = 7;
      break;
    }
    case TMC2209_MRES_256:
    default:
    {
      microsteps_per_step_exponent = 8;
      break;
    }
  }
  return (uint16_t)(1u << microsteps_per_step_exponent);
}

tmc2209_settings_t tmc2209_get_settings(tmc2209_t * tmc2209)
{
  tmc2209_settings_t settings;
  settings.is_communicating = tmc2209_is_communicating(tmc2209);

  if (settings.is_communicating)
  {
    tmc2209_read_and_store_registers(tmc2209);

    settings.is_setup = tmc2209->global_config.pdn_disable;
    settings.software_enabled =
      (tmc2209->chopper_config.toff > TMC2209_TOFF_DISABLE);
    settings.microsteps_per_step = tmc2209_get_microsteps_per_step(tmc2209);
    settings.inverse_motor_direction_enabled = tmc2209->global_config.shaft;
    settings.stealth_chop_enabled =
      !tmc2209->global_config.enable_spread_cycle;
    settings.standstill_mode = (uint8_t)tmc2209->pwm_config.freewheel;
    settings.irun_percent =
      tmc2209_current_setting_to_percent((uint8_t)tmc2209->driver_current.irun);
    settings.irun_register_value = (uint8_t)tmc2209->driver_current.irun;
    settings.ihold_percent =
      tmc2209_current_setting_to_percent((uint8_t)tmc2209->driver_current.ihold);
    settings.ihold_register_value = (uint8_t)tmc2209->driver_current.ihold;
    settings.iholddelay_percent =
      tmc2209_hold_delay_setting_to_percent((uint8_t)tmc2209->driver_current.iholddelay);
    settings.iholddelay_register_value =
      (uint8_t)tmc2209->driver_current.iholddelay;
    settings.automatic_current_scaling_enabled =
      tmc2209->pwm_config.pwm_autoscale;
    settings.automatic_gradient_adaptation_enabled =
      tmc2209->pwm_config.pwm_autograd;
    settings.pwm_offset = (uint8_t)tmc2209->pwm_config.pwm_offset;
    settings.pwm_gradient = (uint8_t)tmc2209->pwm_config.pwm_grad;
    settings.cool_step_enabled = tmc2209->cool_step_enabled;
    settings.analog_current_scaling_enabled =
      tmc2209->global_config.i_scale_analog;
    settings.internal_sense_resistors_enabled =
      tmc2209->global_config.internal_rsense;
  }
  else
  {
    settings.is_setup = false;
    settings.software_enabled = false;
    settings.microsteps_per_step = 0;
    settings.inverse_motor_direction_enabled = false;
    settings.stealth_chop_enabled = false;
    settings.standstill_mode = (uint8_t)tmc2209->pwm_config.freewheel;
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

tmc2209_status_t tmc2209_get_status(tmc2209_t * tmc2209)
{
  union
  {
    tmc2209_status_t status;
    uint32_t bytes;
  } drive_status;
  drive_status.bytes = tmc2209_read_register(tmc2209,
    TMC2209_ADDRESS_DRV_STATUS);
  return drive_status.status;
}

tmc2209_global_status_t tmc2209_get_global_status(tmc2209_t * tmc2209)
{
  union
  {
    tmc2209_global_status_t global_status;
    uint32_t bytes;
  } global_status_union;
  global_status_union.bytes = tmc2209_read_register(tmc2209,
    TMC2209_ADDRESS_GSTAT);
  return global_status_union.global_status;
}

void tmc2209_clear_reset(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209, TMC2209_ADDRESS_GSTAT, TMC2209_GSTAT_RESET);
}

void tmc2209_clear_drive_error(tmc2209_t * tmc2209)
{
  tmc2209_write_register(tmc2209,
    TMC2209_ADDRESS_GSTAT,
    TMC2209_GSTAT_DRV_ERR);
}

uint8_t tmc2209_get_interface_transmission_counter(tmc2209_t * tmc2209)
{
  return (uint8_t)tmc2209_read_register(tmc2209, TMC2209_ADDRESS_IFCNT);
}

uint32_t tmc2209_get_interstep_duration(tmc2209_t * tmc2209)
{
  return tmc2209_read_register(tmc2209, TMC2209_ADDRESS_TSTEP);
}

uint16_t tmc2209_get_stall_guard_result(tmc2209_t * tmc2209)
{
  return (uint16_t)tmc2209_read_register(tmc2209, TMC2209_ADDRESS_SG_RESULT);
}

uint8_t tmc2209_get_pwm_scale_sum(tmc2209_t * tmc2209)
{
  uint32_t pwm_scale = tmc2209_read_register(tmc2209,
    TMC2209_ADDRESS_PWM_SCALE);

  return (uint8_t)(pwm_scale & TMC2209_PWM_SCALE_SUM_MASK);
}

int16_t tmc2209_get_pwm_scale_auto(tmc2209_t * tmc2209)
{
  uint32_t pwm_scale = tmc2209_read_register(tmc2209,
    TMC2209_ADDRESS_PWM_SCALE);
  uint32_t pwm_scale_auto =
    (pwm_scale >> TMC2209_PWM_SCALE_AUTO_SHIFT) & TMC2209_PWM_SCALE_AUTO_MASK;

  // sign extend the 9 bit two's complement register value
  if ((pwm_scale_auto & TMC2209_PWM_SCALE_AUTO_SIGN_BIT) != 0u)
  {
    pwm_scale_auto |= ~(uint32_t)TMC2209_PWM_SCALE_AUTO_MASK;
  }
  return (int16_t)(int32_t)pwm_scale_auto;
}

uint8_t tmc2209_get_pwm_offset_auto(tmc2209_t * tmc2209)
{
  uint32_t pwm_auto = tmc2209_read_register(tmc2209,
    TMC2209_ADDRESS_PWM_AUTO);

  return (uint8_t)(pwm_auto & TMC2209_PWM_OFFSET_AUTO_MASK);
}

uint8_t tmc2209_get_pwm_gradient_auto(tmc2209_t * tmc2209)
{
  uint32_t pwm_auto = tmc2209_read_register(tmc2209,
    TMC2209_ADDRESS_PWM_AUTO);

  return (uint8_t)((pwm_auto >> TMC2209_PWM_GRADIENT_AUTO_SHIFT) &
    TMC2209_PWM_GRADIENT_AUTO_MASK);
}

uint16_t tmc2209_get_microstep_counter(tmc2209_t * tmc2209)
{
  return (uint16_t)tmc2209_read_register(tmc2209, TMC2209_ADDRESS_MSCNT);
}
