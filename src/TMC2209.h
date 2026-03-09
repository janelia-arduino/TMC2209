// ----------------------------------------------------------------------------
// TMC2209.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC2209_H
#define TMC2209_H
#include <Arduino.h>

#include "Result.hpp"

#include "Device.hpp"
#include "Driver.hpp"
#include "Registers.hpp"
#include "UartBus.hpp"
#include "UartBusParameters.hpp"
#include "UartParameters.hpp"

#include "TMC2209/UartEngine.hpp"
#include "tmc2209_registers.hpp"

#if !defined(ESP32) && !defined(ARDUINO_ARCH_SAMD) && !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_SAM_DUE) && !defined(ARDUINO_ARCH_RENESAS)
#define SOFTWARE_SERIAL_INCLUDED true
#else
#define SOFTWARE_SERIAL_INCLUDED false
#endif
#if SOFTWARE_SERIAL_INCLUDED
#include <SoftwareSerial.h>
#endif

class TMC2209 : private tmc2209::UartEngineIo
{
public:
  TMC2209 ();

  // Expose transport result/error helpers without requiring users to type the
  // namespace.
  using UartError = tmc2209::UartError;
  template <typename T>
  using Result = tmc2209::Result<T>;
  using UartBus = tmc2209::UartBus;
  using Device = tmc2209::Device;
  using Driver = tmc2209::Driver;
  using Registers = tmc2209::Registers;
  using UartParameters = tmc2209::UartParameters;
  using UartBusParameters = tmc2209::UartBusParameters;

  Driver driver;
  Registers registers;

  enum SerialAddress
  {
    SERIAL_ADDRESS_0 = 0,
    SERIAL_ADDRESS_1 = 1,
    SERIAL_ADDRESS_2 = 2,
    SERIAL_ADDRESS_3 = 3,
  };
  // Identify which microcontroller serial port is connected to the TMC2209 e.g.
  // Serial1, Serial2, etc. Optionally identify which serial address is assigned
  // to the TMC2209 if not the default of SERIAL_ADDRESS_0.
#if !defined(ARDUINO_ARCH_RENESAS)
  void setup (HardwareSerial &serial,
              SerialAddress serial_address = SERIAL_ADDRESS_0);
#endif
#if defined(ESP32)
  void setup (HardwareSerial &serial,
              SerialAddress serial_address);
#elif defined(ARDUINO_ARCH_RP2040)
  void setup (SerialUART &serial,
              SerialAddress serial_address);
#elif defined(ARDUINO_ARCH_RENESAS)
  void setup (UART &serial,
              SerialAddress serial_address = SERIAL_ADDRESS_0);
#endif

#if SOFTWARE_SERIAL_INCLUDED
  // Software serial ports should only be used for unidirectional communication
  // The RX pin does not need to be connected, but it must be specified when
  // creating an instance of a SoftwareSerial object
  void setup (SoftwareSerial &serial,
              SerialAddress serial_address = SERIAL_ADDRESS_0);
#endif

  // unidirectional methods

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin (uint8_t hardware_enable_pin);
  void enable ();
  void disable ();

  // valid values = 1,2,4,8,...128,256, other values get rounded down
  void setMicrostepsPerStep (uint16_t microsteps_per_step);

  // valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
  // https://en.wikipedia.org/wiki/Power_of_two
  void setMicrostepsPerStepPowerOfTwo (uint8_t exponent);

  // range 0-100
  void setRunCurrent (uint8_t percent);
  // range 0-100
  void setHoldCurrent (uint8_t percent);
  // range 0-100
  void setHoldDelay (uint8_t percent);
  // range 0-100
  void setAllCurrentValues (uint8_t run_current_percent,
                            uint8_t hold_current_percent,
                            uint8_t hold_delay_percent);
  void setRMSCurrent (uint16_t mA,
                      float rSense,
                      float holdMultiplier = 0.5f);

  void enableDoubleEdge ();
  void disableDoubleEdge ();

  void enableVSense ();
  void disableVSense ();

  void enableInverseMotorDirection ();
  void disableInverseMotorDirection ();

  enum StandstillMode
  {
    NORMAL = 0,
    FREEWHEELING = 1,
    STRONG_BRAKING = 2,
    BRAKING = 3,
  };
  void setStandstillMode (StandstillMode mode);

  void enableAutomaticCurrentScaling ();
  void disableAutomaticCurrentScaling ();
  void enableAutomaticGradientAdaptation ();
  void disableAutomaticGradientAdaptation ();
  // range 0-255
  void setPwmOffset (uint8_t pwm_amplitude);
  // range 0-255
  void setPwmGradient (uint8_t pwm_amplitude);

  // default = 20
  // mimimum of 2 for StealthChop auto tuning
  void setPowerDownDelay (uint8_t power_down_delay);

  // mimimum of 2 when using multiple serial addresses
  // in bidirectional communication
  const static uint8_t REPLY_DELAY_MAX = 15;
  void setReplyDelay (uint8_t delay);

  void moveAtVelocity (int32_t microsteps_per_period);
  void moveUsingStepDirInterface ();

  void enableStealthChop ();
  void disableStealthChop ();

  void setStealthChopDurationThreshold (uint32_t duration_threshold);

  void setStallGuardThreshold (uint8_t stall_guard_threshold);

  // lower_threshold: min = 1, max = 15
  // upper_threshold: min = 0, max = 15, 0-2 recommended
  void enableCoolStep (uint8_t lower_threshold = 1,
                       uint8_t upper_threshold = 0);
  void disableCoolStep ();
  enum CurrentIncrement
  {
    CURRENT_INCREMENT_1 = 0,
    CURRENT_INCREMENT_2 = 1,
    CURRENT_INCREMENT_4 = 2,
    CURRENT_INCREMENT_8 = 3,
  };
  void setCoolStepCurrentIncrement (CurrentIncrement current_increment);
  enum MeasurementCount
  {
    MEASUREMENT_COUNT_32 = 0,
    MEASUREMENT_COUNT_8 = 1,
    MEASUREMENT_COUNT_2 = 2,
    MEASUREMENT_COUNT_1 = 3,
  };
  void setCoolStepMeasurementCount (MeasurementCount measurement_count);
  void setCoolStepDurationThreshold (uint32_t duration_threshold);

  void enableAnalogCurrentScaling ();
  void disableAnalogCurrentScaling ();

  void useExternalSenseResistors ();
  void useInternalSenseResistors ();

  // bidirectional methods
  uint8_t getVersion ();

  // Explicit register access with error reporting.
  Result<uint32_t> readRegister (uint8_t register_address);
  Result<void> writeRegister (uint8_t register_address, uint32_t data);

  // Non-blocking register transactions backed by the shared UART engine.
  Result<void> startRead (uint8_t register_address);
  Result<void> startWrite (uint8_t register_address, uint32_t data);
  void poll ();
  bool busy () const;
  bool resultReady () const;
  Result<uint32_t> takeReadResult ();
  Result<void> takeWriteResult ();

  // Retrieve and clear the last UART error observed by the library.
  UartError getLastUartError () const;
  void clearLastUartError ();

  // if driver is not communicating, check power and communication connections
  bool isCommunicating ();

  // check to make sure TMC2209 is properly setup and communicating
  bool isSetupAndCommunicating ();

  // driver may be communicating but not setup if driver power is lost then
  // restored after setup so that defaults are loaded instead of setup options
  bool isCommunicatingButNotSetup ();

  // driver may also be disabled by the hardware enable input pin
  // this pin must be grounded or disconnected before driver may be enabled
  bool hardwareDisabled ();

  uint16_t getMicrostepsPerStep ();

  struct Settings
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
  };
  Settings getSettings ();

  struct Status
  {
    bool over_temperature_warning;
    bool over_temperature_shutdown;
    bool short_to_ground_a;
    bool short_to_ground_b;
    bool low_side_short_a;
    bool low_side_short_b;
    bool open_load_a;
    bool open_load_b;
    bool over_temperature_120c;
    bool over_temperature_143c;
    bool over_temperature_150c;
    bool over_temperature_157c;
    uint8_t current_scaling;
    bool stealth_chop_mode;
    bool standstill;
  };
  const static uint8_t CURRENT_SCALING_MAX = 31;
  Status getStatus ();

  struct GlobalStatus
  {
    bool reset;
    bool drv_err;
    bool uv_cp;
  };
  GlobalStatus getGlobalStatus ();
  void clearReset ();
  void clearDriveError ();

  uint8_t getInterfaceTransmissionCounter ();

  uint32_t getInterstepDuration ();

  uint16_t getStallGuardResult ();

  uint8_t getPwmScaleSum ();
  int16_t getPwmScaleAuto ();
  uint8_t getPwmOffsetAuto ();
  uint8_t getPwmGradientAuto ();

  uint16_t getMicrostepCounter ();

private:
  tmc2209::UartBus facade_bus_;
  tmc2209::Device facade_device_;

  HardwareSerial *hardware_serial_ptr_;
#if SOFTWARE_SERIAL_INCLUDED
  SoftwareSerial *software_serial_ptr_;
#endif
  uint8_t serial_address_;
  int16_t hardware_enable_pin_;

  UartError last_uart_error_;
  tmc2209::UartEngine uart_engine_;

  void initialize (SerialAddress serial_address = SERIAL_ADDRESS_0);
  bool serialTransportConfigured () const;
  int serialAvailable ();
  size_t serialWrite (uint8_t c);
  int serialRead ();
  void serialFlush ();

  int uartAvailable () override;
  int uartRead () override;
  size_t uartWrite (uint8_t c) override;
  void uartFlush () override;

  const static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
  const static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

  // General Configuration Registers
  const static uint8_t ADDRESS_GCONF = 0x00;
  tmc2209::reg::GCONF gconf_;

  const static uint8_t ADDRESS_GSTAT = 0x01;

  const static uint8_t ADDRESS_IFCNT = 0x02;

  const static uint8_t ADDRESS_REPLYDELAY = 0x03;

  const static uint8_t ADDRESS_IOIN = 0x06;
  const static uint8_t VERSION = 0x21;

  // Velocity Dependent Driver Feature Control Register Set
  const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
  tmc2209::reg::IHOLD_IRUN ihold_irun_;
  const static uint8_t PERCENT_MIN = 0;
  const static uint8_t PERCENT_MAX = 100;
  const static uint8_t CURRENT_SETTING_MIN = 0;
  const static uint8_t CURRENT_SETTING_MAX = 31;
  const static uint8_t HOLD_DELAY_MIN = 0;
  const static uint8_t HOLD_DELAY_MAX = 15;
  const static uint8_t IHOLD_DEFAULT = 16;
  const static uint8_t IRUN_DEFAULT = 31;
  const static uint8_t IHOLDDELAY_DEFAULT = 1;

  const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
  const static uint8_t TPOWERDOWN_DEFAULT = 20;

  const static uint8_t ADDRESS_TSTEP = 0x12;

  const static uint8_t ADDRESS_TPWMTHRS = 0x13;
  const static uint32_t TPWMTHRS_DEFAULT = 0;

  const static uint8_t ADDRESS_VACTUAL = 0x22;
  const static int32_t VACTUAL_DEFAULT = 0;
  const static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;

  // CoolStep and StallGuard Control Register Set
  const static uint8_t ADDRESS_TCOOLTHRS = 0x14;
  const static uint8_t TCOOLTHRS_DEFAULT = 0;
  const static uint8_t ADDRESS_SGTHRS = 0x40;
  const static uint8_t SGTHRS_DEFAULT = 0;
  const static uint8_t ADDRESS_SG_RESULT = 0x41;

  const static uint8_t ADDRESS_COOLCONF = 0x42;
  const static uint8_t COOLCONF_DEFAULT = 0;
  tmc2209::reg::COOLCONF coolconf_;
  bool cool_step_enabled_;
  const static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
  const static uint8_t SEIMIN_LOWER_SETTING = 0;
  const static uint8_t SEIMIN_UPPER_SETTING = 1;
  const static uint8_t SEMIN_OFF = 0;
  const static uint8_t SEMIN_MIN = 1;
  const static uint8_t SEMIN_MAX = 15;
  const static uint8_t SEMAX_MIN = 0;
  const static uint8_t SEMAX_MAX = 15;

  // Microstepping Control Register Set
  const static uint8_t ADDRESS_MSCNT = 0x6A;
  const static uint8_t ADDRESS_MSCURACT = 0x6B;

  // Driver Register Set
  const static uint8_t ADDRESS_CHOPCONF = 0x6C;
  tmc2209::reg::CHOPCONF chopconf_;
  const static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
  const static uint8_t TBL_DEFAULT = 0b10;
  const static uint8_t HEND_DEFAULT = 0;
  const static uint8_t HSTART_DEFAULT = 5;
  const static uint8_t TOFF_DEFAULT = 3;
  const static uint8_t TOFF_DISABLE = 0;
  uint8_t toff_ = TOFF_DEFAULT;

  const static size_t MICROSTEPS_PER_STEP_MIN = 1;
  const static size_t MICROSTEPS_PER_STEP_MAX = 256;

  const static uint8_t ADDRESS_DRV_STATUS = 0x6F;

  const static uint8_t ADDRESS_PWMCONF = 0x70;
  tmc2209::reg::PWMCONF pwmconf_;
  const static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
  const static uint8_t PWM_OFFSET_MIN = 0;
  const static uint8_t PWM_OFFSET_MAX = 255;
  const static uint8_t PWM_OFFSET_DEFAULT = 0x24;
  const static uint8_t PWM_GRAD_MIN = 0;
  const static uint8_t PWM_GRAD_MAX = 255;
  const static uint8_t PWM_GRAD_DEFAULT = 0x14;

  const static uint8_t ADDRESS_PWM_SCALE = 0x71;

  const static uint8_t ADDRESS_PWM_AUTO = 0x72;

  void setOperationModeToSerial (SerialAddress serial_address);

  void setRegistersToDefaults ();
  void readAndStoreRegisters ();

  bool serialOperationMode ();

  void minimizeMotorCurrent ();

  void write (uint8_t register_address,
              uint32_t data);
  uint32_t read (uint8_t register_address);

  uint8_t percentToCurrentSetting (uint8_t percent);
  uint8_t currentSettingToPercent (uint8_t current_setting);
  uint8_t percentToHoldDelaySetting (uint8_t percent);
  uint8_t holdDelaySettingToPercent (uint8_t hold_delay_setting);

  uint8_t pwmAmplitudeToPwmAmpl (uint8_t pwm_amplitude);
  uint8_t pwmAmplitudeToPwmGrad (uint8_t pwm_amplitude);

  void writeStoredGlobalConfig ();
  uint32_t readGlobalConfigBytes ();
  void writeStoredDriverCurrent ();
  void writeStoredChopperConfig ();
  uint32_t readChopperConfigBytes ();
  void writeStoredPwmConfig ();
  uint32_t readPwmConfigBytes ();

  uint32_t constrain_ (uint32_t value, uint32_t low, uint32_t high);
};

#endif
