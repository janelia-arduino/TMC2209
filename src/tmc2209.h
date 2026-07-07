// ----------------------------------------------------------------------------
// tmc2209.h
//
// TMC2209 stepper motor driver library.
//
// Portable C17 implementation with no platform dependencies. All hardware
// access (UART, delays, optional hardware enable pin) is performed through a
// user supplied hardware abstraction layer (tmc2209_hal_t), which makes the
// library usable on any microcontroller, including STM32 devices using only
// CMSIS.
//
// Authors:
// Peter Polidoro peter@polidoro.io (original Arduino C++ implementation)
// ----------------------------------------------------------------------------

#ifndef TMC2209_H
#define TMC2209_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// Hardware abstraction layer
// ----------------------------------------------------------------------------

// The library talks to the TMC2209 over a half duplex UART configured as
// 8 data bits, no parity, 1 stop bit. Configure the UART peripheral before
// calling tmc2209_setup(); the library never (re)configures the port itself.
//
// All callbacks receive the user supplied context pointer as their first
// argument.
typedef struct tmc2209_hal
{
  // Transmit size bytes. May block until the bytes are queued or sent.
  void (*serial_write)(void * context, uint8_t const * data, size_t size);

  // Number of received bytes immediately available for reading.
  size_t (*serial_available)(void * context);

  // Read one received byte. Return -1 if no byte is available.
  int16_t (*serial_read)(void * context);

  // Block until all queued outgoing bytes have physically left the wire.
  void (*serial_flush)(void * context);

  // Busy wait for the given number of microseconds.
  void (*delay_microseconds)(void * context, uint32_t microseconds);

  // Wait for the given number of milliseconds.
  void (*delay_milliseconds)(void * context, uint32_t milliseconds);

  // Optional, may be NULL. Drive the TMC2209 hardware enable (ENN) input.
  // enable == true must pull ENN low (driver enabled),
  // enable == false must pull ENN high (driver disabled).
  void (*set_hardware_enable_pin)(void * context, bool enable);

  // Opaque pointer passed unchanged to every callback.
  void * context;
} tmc2209_hal_t;

// ----------------------------------------------------------------------------
// Public types
// ----------------------------------------------------------------------------

typedef enum tmc2209_serial_address
{
  TMC2209_SERIAL_ADDRESS_0 = 0,
  TMC2209_SERIAL_ADDRESS_1 = 1,
  TMC2209_SERIAL_ADDRESS_2 = 2,
  TMC2209_SERIAL_ADDRESS_3 = 3,
} tmc2209_serial_address_t;

typedef enum tmc2209_standstill_mode
{
  TMC2209_NORMAL = 0,
  TMC2209_FREEWHEELING = 1,
  TMC2209_STRONG_BRAKING = 2,
  TMC2209_BRAKING = 3,
} tmc2209_standstill_mode_t;

typedef enum tmc2209_current_increment
{
  TMC2209_CURRENT_INCREMENT_1 = 0,
  TMC2209_CURRENT_INCREMENT_2 = 1,
  TMC2209_CURRENT_INCREMENT_4 = 2,
  TMC2209_CURRENT_INCREMENT_8 = 3,
} tmc2209_current_increment_t;

typedef enum tmc2209_measurement_count
{
  TMC2209_MEASUREMENT_COUNT_32 = 0,
  TMC2209_MEASUREMENT_COUNT_8 = 1,
  TMC2209_MEASUREMENT_COUNT_2 = 2,
  TMC2209_MEASUREMENT_COUNT_1 = 3,
} tmc2209_measurement_count_t;

typedef struct tmc2209_settings
{
  bool is_communicating;
  bool is_setup;
  bool software_enabled;
  uint16_t microsteps_per_step;
  bool inverse_motor_direction_enabled;
  bool stealth_chop_enabled;
  uint8_t standstill_mode;
  uint8_t irun_percent;
  uint8_t irun_register_value;
  uint8_t ihold_percent;
  uint8_t ihold_register_value;
  uint8_t iholddelay_percent;
  uint8_t iholddelay_register_value;
  bool automatic_current_scaling_enabled;
  bool automatic_gradient_adaptation_enabled;
  uint8_t pwm_offset;
  uint8_t pwm_gradient;
  bool cool_step_enabled;
  bool analog_current_scaling_enabled;
  bool internal_sense_resistors_enabled;
} tmc2209_settings_t;

typedef struct tmc2209_status
{
  uint32_t over_temperature_warning : 1;
  uint32_t over_temperature_shutdown : 1;
  uint32_t short_to_ground_a : 1;
  uint32_t short_to_ground_b : 1;
  uint32_t low_side_short_a : 1;
  uint32_t low_side_short_b : 1;
  uint32_t open_load_a : 1;
  uint32_t open_load_b : 1;
  uint32_t over_temperature_120c : 1;
  uint32_t over_temperature_143c : 1;
  uint32_t over_temperature_150c : 1;
  uint32_t over_temperature_157c : 1;
  uint32_t reserved0 : 4;
  uint32_t current_scaling : 5;
  uint32_t reserved1 : 9;
  uint32_t stealth_chop_mode : 1;
  uint32_t standstill : 1;
} tmc2209_status_t;

#define TMC2209_CURRENT_SCALING_MAX 31u

typedef struct tmc2209_global_status
{
  uint32_t reset : 1;
  uint32_t drv_err : 1;
  uint32_t uv_cp : 1;
  uint32_t reserved : 29;
} tmc2209_global_status_t;

// Maximum value accepted by tmc2209_set_reply_delay().
// A minimum of 2 is required when using multiple serial addresses in
// bidirectional communication.
#define TMC2209_REPLY_DELAY_MAX 15u

// ----------------------------------------------------------------------------
// Internal register images (do not access directly)
// ----------------------------------------------------------------------------

typedef union tmc2209_global_config
{
  struct
  {
    uint32_t i_scale_analog : 1;
    uint32_t internal_rsense : 1;
    uint32_t enable_spread_cycle : 1;
    uint32_t shaft : 1;
    uint32_t index_otpw : 1;
    uint32_t index_step : 1;
    uint32_t pdn_disable : 1;
    uint32_t mstep_reg_select : 1;
    uint32_t multistep_filt : 1;
    uint32_t test_mode : 1;
    uint32_t reserved : 22;
  };
  uint32_t bytes;
} tmc2209_global_config_t;

typedef union tmc2209_driver_current
{
  struct
  {
    uint32_t ihold : 5;
    uint32_t reserved_0 : 3;
    uint32_t irun : 5;
    uint32_t reserved_1 : 3;
    uint32_t iholddelay : 4;
    uint32_t reserved_2 : 12;
  };
  uint32_t bytes;
} tmc2209_driver_current_t;

typedef union tmc2209_cool_config
{
  struct
  {
    uint32_t semin : 4;
    uint32_t reserved_0 : 1;
    uint32_t seup : 2;
    uint32_t reserved_1 : 1;
    uint32_t semax : 4;
    uint32_t reserved_2 : 1;
    uint32_t sedn : 2;
    uint32_t seimin : 1;
    uint32_t reserved_3 : 16;
  };
  uint32_t bytes;
} tmc2209_cool_config_t;

typedef union tmc2209_chopper_config
{
  struct
  {
    uint32_t toff : 4;
    uint32_t hstart : 3;
    uint32_t hend : 4;
    uint32_t reserved_0 : 4;
    uint32_t tbl : 2;
    uint32_t vsense : 1;
    uint32_t reserved_1 : 6;
    uint32_t mres : 4;
    uint32_t interpolation : 1;
    uint32_t double_edge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
  };
  uint32_t bytes;
} tmc2209_chopper_config_t;

typedef union tmc2209_pwm_config
{
  struct
  {
    uint32_t pwm_offset : 8;
    uint32_t pwm_grad : 8;
    uint32_t pwm_freq : 2;
    uint32_t pwm_autoscale : 1;
    uint32_t pwm_autograd : 1;
    uint32_t freewheel : 2;
    uint32_t reserved : 2;
    uint32_t pwm_reg : 4;
    uint32_t pwm_lim : 4;
  };
  uint32_t bytes;
} tmc2209_pwm_config_t;

// Driver instance. Treat all members as private; initialize with
// tmc2209_setup().
typedef struct tmc2209
{
  tmc2209_hal_t hal;
  uint8_t serial_address;
  bool cool_step_enabled;
  uint8_t toff;
  tmc2209_global_config_t global_config;
  tmc2209_driver_current_t driver_current;
  tmc2209_cool_config_t cool_config;
  tmc2209_chopper_config_t chopper_config;
  tmc2209_pwm_config_t pwm_config;
} tmc2209_t;

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------

// Initialize the driver instance and put the TMC2209 into UART operation
// mode with safe default register values. The UART referenced by the HAL
// must already be configured (8N1) before this call.
void tmc2209_setup(tmc2209_t * tmc2209,
  tmc2209_hal_t const * hal,
  tmc2209_serial_address_t serial_address);

// ----------------------------------------------------------------------------
// Unidirectional methods (work without an RX connection)
// ----------------------------------------------------------------------------

// The driver is disabled by default and must be enabled before use.
// If the HAL provides set_hardware_enable_pin it is driven as well.
void tmc2209_enable(tmc2209_t * tmc2209);
void tmc2209_disable(tmc2209_t * tmc2209);

// valid values = 1,2,4,8,...128,256, other values get rounded down
void tmc2209_set_microsteps_per_step(tmc2209_t * tmc2209,
  uint16_t microsteps_per_step);

// valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
void tmc2209_set_microsteps_per_step_power_of_two(tmc2209_t * tmc2209,
  uint8_t exponent);

// range 0-100
void tmc2209_set_run_current(tmc2209_t * tmc2209, uint8_t percent);
// range 0-100
void tmc2209_set_hold_current(tmc2209_t * tmc2209, uint8_t percent);
// range 0-100
void tmc2209_set_hold_delay(tmc2209_t * tmc2209, uint8_t percent);
// range 0-100
void tmc2209_set_all_current_values(tmc2209_t * tmc2209,
  uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent);
void tmc2209_set_rms_current(tmc2209_t * tmc2209,
  uint16_t milliamps,
  float r_sense,
  float hold_multiplier);

void tmc2209_enable_double_edge(tmc2209_t * tmc2209);
void tmc2209_disable_double_edge(tmc2209_t * tmc2209);

void tmc2209_enable_vsense(tmc2209_t * tmc2209);
void tmc2209_disable_vsense(tmc2209_t * tmc2209);

void tmc2209_enable_inverse_motor_direction(tmc2209_t * tmc2209);
void tmc2209_disable_inverse_motor_direction(tmc2209_t * tmc2209);

void tmc2209_set_standstill_mode(tmc2209_t * tmc2209,
  tmc2209_standstill_mode_t mode);

void tmc2209_enable_automatic_current_scaling(tmc2209_t * tmc2209);
void tmc2209_disable_automatic_current_scaling(tmc2209_t * tmc2209);
void tmc2209_enable_automatic_gradient_adaptation(tmc2209_t * tmc2209);
void tmc2209_disable_automatic_gradient_adaptation(tmc2209_t * tmc2209);
// range 0-255
void tmc2209_set_pwm_offset(tmc2209_t * tmc2209, uint8_t pwm_amplitude);
// range 0-255
void tmc2209_set_pwm_gradient(tmc2209_t * tmc2209, uint8_t pwm_amplitude);

// default = 20
// minimum of 2 for StealthChop auto tuning
void tmc2209_set_power_down_delay(tmc2209_t * tmc2209,
  uint8_t power_down_delay);

// clamped to TMC2209_REPLY_DELAY_MAX
void tmc2209_set_reply_delay(tmc2209_t * tmc2209, uint8_t reply_delay);

void tmc2209_move_at_velocity(tmc2209_t * tmc2209,
  int32_t microsteps_per_period);
void tmc2209_move_using_step_dir_interface(tmc2209_t * tmc2209);

void tmc2209_enable_stealth_chop(tmc2209_t * tmc2209);
void tmc2209_disable_stealth_chop(tmc2209_t * tmc2209);

void tmc2209_set_stealth_chop_duration_threshold(tmc2209_t * tmc2209,
  uint32_t duration_threshold);

void tmc2209_set_stall_guard_threshold(tmc2209_t * tmc2209,
  uint8_t stall_guard_threshold);

// lower_threshold: min = 1, max = 15
// upper_threshold: min = 0, max = 15, 0-2 recommended
void tmc2209_enable_cool_step(tmc2209_t * tmc2209,
  uint8_t lower_threshold,
  uint8_t upper_threshold);
void tmc2209_disable_cool_step(tmc2209_t * tmc2209);
void tmc2209_set_cool_step_current_increment(tmc2209_t * tmc2209,
  tmc2209_current_increment_t current_increment);
void tmc2209_set_cool_step_measurement_count(tmc2209_t * tmc2209,
  tmc2209_measurement_count_t measurement_count);
void tmc2209_set_cool_step_duration_threshold(tmc2209_t * tmc2209,
  uint32_t duration_threshold);

void tmc2209_enable_analog_current_scaling(tmc2209_t * tmc2209);
void tmc2209_disable_analog_current_scaling(tmc2209_t * tmc2209);

void tmc2209_use_external_sense_resistors(tmc2209_t * tmc2209);
void tmc2209_use_internal_sense_resistors(tmc2209_t * tmc2209);

// ----------------------------------------------------------------------------
// Bidirectional methods (require RX and TX connections)
// ----------------------------------------------------------------------------

uint8_t tmc2209_get_version(tmc2209_t * tmc2209);

// if the driver is not communicating, check power and communication
// connections
bool tmc2209_is_communicating(tmc2209_t * tmc2209);

// check to make sure the TMC2209 is properly setup and communicating
bool tmc2209_is_setup_and_communicating(tmc2209_t * tmc2209);

// the driver may be communicating but not setup if driver power is lost
// then restored after setup, so that defaults are loaded instead of setup
// options
bool tmc2209_is_communicating_but_not_setup(tmc2209_t * tmc2209);

// the driver may also be disabled by the hardware enable input pin
// this pin must be grounded or disconnected before the driver may be enabled
bool tmc2209_hardware_disabled(tmc2209_t * tmc2209);

uint16_t tmc2209_get_microsteps_per_step(tmc2209_t * tmc2209);

tmc2209_settings_t tmc2209_get_settings(tmc2209_t * tmc2209);

tmc2209_status_t tmc2209_get_status(tmc2209_t * tmc2209);

tmc2209_global_status_t tmc2209_get_global_status(tmc2209_t * tmc2209);
void tmc2209_clear_reset(tmc2209_t * tmc2209);
void tmc2209_clear_drive_error(tmc2209_t * tmc2209);

uint8_t tmc2209_get_interface_transmission_counter(tmc2209_t * tmc2209);

uint32_t tmc2209_get_interstep_duration(tmc2209_t * tmc2209);

uint16_t tmc2209_get_stall_guard_result(tmc2209_t * tmc2209);

uint8_t tmc2209_get_pwm_scale_sum(tmc2209_t * tmc2209);
int16_t tmc2209_get_pwm_scale_auto(tmc2209_t * tmc2209);
uint8_t tmc2209_get_pwm_offset_auto(tmc2209_t * tmc2209);
uint8_t tmc2209_get_pwm_gradient_auto(tmc2209_t * tmc2209);

uint16_t tmc2209_get_microstep_counter(tmc2209_t * tmc2209);

#ifdef __cplusplus
}
#endif

#endif // TMC2209_H
