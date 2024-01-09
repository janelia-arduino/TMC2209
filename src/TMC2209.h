// ----------------------------------------------------------------------------
// TMC2209.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC2209_H
#define TMC2209_H
#include <Arduino.h>

#if !defined(ESP32) && !defined(ARDUINO_ARCH_SAMD) && !defined(ARDUINO_RASPBERRY_PI_PICO) && !defined(ARDUINO_SAM_DUE)
#  define SOFTWARE_SERIAL_IMPLEMENTED true
#else
#  define SOFTWARE_SERIAL_IMPLEMENTED false
#endif
#if SOFTWARE_SERIAL_IMPLEMENTED
#  include <SoftwareSerial.h>
#endif


class TMC2209
{
public:
  TMC2209();

  enum SerialAddress
  {
    SERIAL_ADDRESS_0=0,
    SERIAL_ADDRESS_1=1,
    SERIAL_ADDRESS_2=2,
    SERIAL_ADDRESS_3=3,
  };
  // Alternate rx and tx pins may be specified for certain microcontrollers e.g.
  // ESP32
  #ifdef ESP32
  void setup(HardwareSerial & serial,
    long serial_baud_rate=115200,
    SerialAddress serial_address=SERIAL_ADDRESS_0,
    int16_t alternate_rx_pin=-1,
    int16_t alternate_tx_pin=-1);
  #else
  // Identify which microcontroller serial port is connected to the TMC2209 e.g.
  // Serial1, Serial2, etc. Optionally identify which serial address is assigned
  // to the TMC2209 if not the default of SERIAL_ADDRESS_0.
  void setup(HardwareSerial & serial,
    long serial_baud_rate=115200,
    SerialAddress serial_address=SERIAL_ADDRESS_0);
  #endif

#if SOFTWARE_SERIAL_IMPLEMENTED
  // Software serial ports should only be used for unidirectional communication
  // The RX pin does not need to be connected, but it must be specified when
  // creating an instance of a SoftwareSerial object
  void setup(SoftwareSerial & serial,
    long serial_baud_rate=9600,
    SerialAddress serial_address=SERIAL_ADDRESS_0);
#endif

  // unidirectional methods

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin(uint8_t hardware_enable_pin);
  void enable();
  void disable();

  // valid values = 1,2,4,8,...128,256, other values get rounded down
  void setMicrostepsPerStep(uint16_t microsteps_per_step);

  // valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
  // https://en.wikipedia.org/wiki/Power_of_two
  void setMicrostepsPerStepPowerOfTwo(uint8_t exponent);

  // range 0-100
  void setRunCurrent(uint8_t percent);
  // range 0-100
  void setHoldCurrent(uint8_t percent);
  // range 0-100
  void setHoldDelay(uint8_t percent);
  // range 0-100
  void setAllCurrentValues(uint8_t run_current_percent,
    uint8_t hold_current_percent,
    uint8_t hold_delay_percent);

  void enableInverseMotorDirection();
  void disableInverseMotorDirection();

  enum StandstillMode
  {
    NORMAL=0,
    FREEWHEELING=1,
    STRONG_BRAKING=2,
    BRAKING=3,
  };
  void setStandstillMode(StandstillMode mode);

  void enableAutomaticCurrentScaling();
  void disableAutomaticCurrentScaling();
  void enableAutomaticGradientAdaptation();
  void disableAutomaticGradientAdaptation();
  // range 0-255
  void setPwmOffset(uint8_t pwm_amplitude);
  // range 0-255
  void setPwmGradient(uint8_t pwm_amplitude);

  // default = 20
  // mimimum of 2 for StealthChop auto tuning
  void setPowerDownDelay(uint8_t power_down_delay);

  // mimimum of 2 when using multiple serial addresses
  // in bidirectional communication
  const static uint8_t REPLY_DELAY_MAX = 15;
  void setReplyDelay(uint8_t delay);

  void moveAtVelocity(int32_t microsteps_per_period);
  void moveUsingStepDirInterface();

  void enableStealthChop();
  void disableStealthChop();

  void setStealthChopDurationThreshold(uint32_t duration_threshold);

  void setStallGuardThreshold(uint8_t stall_guard_threshold);

  // lower_threshold: min = 1, max = 15
  // upper_threshold: min = 0, max = 15, 0-2 recommended
  void enableCoolStep(uint8_t lower_threshold=1,
    uint8_t upper_threshold=0);
  void disableCoolStep();
  enum CurrentIncrement
  {
    CURRENT_INCREMENT_1=0,
    CURRENT_INCREMENT_2=1,
    CURRENT_INCREMENT_4=2,
    CURRENT_INCREMENT_8=3,
  };
  void setCoolStepCurrentIncrement(CurrentIncrement current_increment);
  enum MeasurementCount
  {
    MEASUREMENT_COUNT_32=0,
    MEASUREMENT_COUNT_8=1,
    MEASUREMENT_COUNT_2=2,
    MEASUREMENT_COUNT_1=3,
  };
  void setCoolStepMeasurementCount(MeasurementCount measurement_count);
  void setCoolStepDurationThreshold(uint32_t duration_threshold);

  void enableAnalogCurrentScaling();
  void disableAnalogCurrentScaling();

  void useExternalSenseResistors();
  void useInternalSenseResistors();

  // bidirectional methods
  uint8_t getVersion();

  // if driver is not communicating, check power and communication connections
  bool isCommunicating();

  // check to make sure TMC2209 is properly setup and communicating
  bool isSetupAndCommunicating();

  // driver may be communicating but not setup if driver power is lost then
  // restored after setup so that defaults are loaded instead of setup options
  bool isCommunicatingButNotSetup();

  // driver may also be disabled by the hardware enable input pin
  // this pin must be grounded or disconnected before driver may be enabled
  bool hardwareDisabled();

  uint16_t getMicrostepsPerStep();

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
  Settings getSettings();

  struct Status
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
  };
  const static uint8_t CURRENT_SCALING_MAX = 31;
  Status getStatus();

  uint8_t getInterfaceTransmissionCounter();

  uint32_t getInterstepDuration();

  uint16_t getStallGuardResult();

  uint8_t getPwmScaleSum();
  int16_t getPwmScaleAuto();
  uint8_t getPwmOffsetAuto();
  uint8_t getPwmGradientAuto();

  uint16_t getMicrostepCounter();

private:
  HardwareSerial * hardware_serial_ptr_;
#if SOFTWARE_SERIAL_IMPLEMENTED
  SoftwareSerial * software_serial_ptr_;
#endif
  uint32_t serial_baud_rate_;
  uint8_t serial_address_;
  int16_t hardware_enable_pin_;

  void initialize(long serial_baud_rate=115200,
    SerialAddress serial_address=SERIAL_ADDRESS_0);
  int serialAvailable();
  size_t serialWrite(uint8_t c);
  int serialRead();
  void serialFlush();

  // Serial Settings
  const static uint8_t BYTE_MAX_VALUE = 0xFF;
  const static uint8_t BITS_PER_BYTE = 8;

  const static uint32_t ECHO_DELAY_INC_MICROSECONDS = 1;
  const static uint32_t ECHO_DELAY_MAX_MICROSECONDS = 4000;

  const static uint32_t REPLY_DELAY_INC_MICROSECONDS = 1;
  const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10000;

  const static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
  const static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

  // Datagrams
  const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
  const static uint8_t DATA_SIZE = 4;
  union WriteReadReplyDatagram
  {
    struct
    {
      uint64_t sync : 4;
      uint64_t reserved : 4;
      uint64_t serial_address : 8;
      uint64_t register_address : 7;
      uint64_t rw : 1;
      uint64_t data : 32;
      uint64_t crc : 8;
    };
    uint64_t bytes;
  };

  const static uint8_t SYNC = 0b101;
  const static uint8_t RW_READ = 0;
  const static uint8_t RW_WRITE = 1;
  const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

  const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
  union ReadRequestDatagram
  {
    struct
    {
      uint32_t sync : 4;
      uint32_t reserved : 4;
      uint32_t serial_address : 8;
      uint32_t register_address : 7;
      uint32_t rw : 1;
      uint32_t crc : 8;
    };
    uint32_t bytes;
  };

  // General Configuration Registers
  const static uint8_t ADDRESS_GCONF = 0x00;
  union GlobalConfig
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
  };
  GlobalConfig global_config_;

  const static uint8_t ADDRESS_GSTAT = 0x01;
  union GlobalStatus
  {
    struct
    {
      uint32_t reset : 1;
      uint32_t drv_err : 1;
      uint32_t uv_cp : 1;
      uint32_t reserved : 29;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_IFCNT = 0x02;

  const static uint8_t ADDRESS_REPLYDELAY = 0x03;
  union ReplyDelay
  {
    struct
    {
      uint32_t reserved_0 : 8;
      uint32_t replydelay : 4;
      uint32_t reserved_1 : 20;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_IOIN = 0x06;
  union Input
  {
    struct
    {
      uint32_t enn : 1;
      uint32_t reserved_0 : 1;
      uint32_t ms1 : 1;
      uint32_t ms2 : 1;
      uint32_t diag : 1;
      uint32_t reserved_1 : 1;
      uint32_t pdn_serial : 1;
      uint32_t step : 1;
      uint32_t spread_en : 1;
      uint32_t dir : 1;
      uint32_t reserved_2 : 14;
      uint32_t version : 8;
    };
    uint32_t bytes;
  };
  const static uint8_t VERSION = 0x21;


  // Velocity Dependent Driver Feature Control Register Set
  const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
  union DriverCurrent
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
  };
  DriverCurrent driver_current_;
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
  union CoolConfig
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
  };
  CoolConfig cool_config_;
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
  union ChopperConfig
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
  };
  ChopperConfig chopper_config_;
  const static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
  const static uint8_t TBL_DEFAULT = 0b10;
  const static uint8_t HEND_DEFAULT = 0;
  const static uint8_t HSTART_DEFAULT = 5;
  const static uint8_t TOFF_DEFAULT = 3;
  const static uint8_t TOFF_DISABLE = 0;
  uint8_t toff_ = TOFF_DEFAULT;
  const static uint8_t MRES_256 = 0b0000;
  const static uint8_t MRES_128 = 0b0001;
  const static uint8_t MRES_064 = 0b0010;
  const static uint8_t MRES_032 = 0b0011;
  const static uint8_t MRES_016 = 0b0100;
  const static uint8_t MRES_008 = 0b0101;
  const static uint8_t MRES_004 = 0b0110;
  const static uint8_t MRES_002 = 0b0111;
  const static uint8_t MRES_001 = 0b1000;

  const static size_t MICROSTEPS_PER_STEP_MIN = 1;
  const static size_t MICROSTEPS_PER_STEP_MAX = 256;

  const static uint8_t ADDRESS_DRV_STATUS = 0x6F;
  union DriveStatus
  {
    struct
    {
      Status status;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_PWMCONF = 0x70;
  union PwmConfig
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
  };
  PwmConfig pwm_config_;
  const static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
  const static uint8_t PWM_OFFSET_MIN = 0;
  const static uint8_t PWM_OFFSET_MAX = 255;
  const static uint8_t PWM_OFFSET_DEFAULT = 0x24;
  const static uint8_t PWM_GRAD_MIN = 0;
  const static uint8_t PWM_GRAD_MAX = 255;
  const static uint8_t PWM_GRAD_DEFAULT = 0x14;

  union PwmScale
  {
    struct
    {
      uint32_t pwm_scale_sum : 8;
      uint32_t reserved_0 : 8;
      uint32_t pwm_scale_auto : 9;
      uint32_t reserved_1 : 7;
    };
    uint32_t bytes;
  };
  const static uint8_t ADDRESS_PWM_SCALE = 0x71;

  union PwmAuto
  {
    struct
    {
      uint32_t pwm_offset_auto : 8;
      uint32_t reserved_0 : 8;
      uint32_t pwm_gradient_auto : 8;
      uint32_t reserved_1 : 8;
    };
    uint32_t bytes;
  };
  const static uint8_t ADDRESS_PWM_AUTO = 0x72;

  void setOperationModeToSerial(SerialAddress serial_address);

  void setRegistersToDefaults();
  void readAndStoreRegisters();

  bool serialOperationMode();

  void minimizeMotorCurrent();

  uint32_t reverseData(uint32_t data);
  template<typename Datagram>
  uint8_t calculateCrc(Datagram & datagram,
    uint8_t datagram_size);
  template<typename Datagram>
  void sendDatagramUnidirectional(Datagram & datagram,
    uint8_t datagram_size);
  template<typename Datagram>
  void sendDatagramBidirectional(Datagram & datagram,
    uint8_t datagram_size);

  void write(uint8_t register_address,
    uint32_t data);
  uint32_t read(uint8_t register_address);

  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);
  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

  uint8_t pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude);
  uint8_t pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude);

  void writeStoredGlobalConfig();
  uint32_t readGlobalConfigBytes();
  void writeStoredDriverCurrent();
  void writeStoredChopperConfig();
  uint32_t readChopperConfigBytes();
  void writeStoredPwmConfig();
  uint32_t readPwmConfigBytes();

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);
};

#endif
