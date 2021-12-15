// ----------------------------------------------------------------------------
// TMC2209.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC2209_H
#define TMC2209_H
#include <Arduino.h>
#include <Streaming.h>


class TMC2209
{
public:
  TMC2209();
  void setEnablePin(size_t enable_pin);

  enum UartAddress
  {
    UART_ADDRESS_0=0,
    UART_ADDRESS_1=1,
    UART_ADDRESS_2=2,
  };
  void setup(HardwareSerial & serial,
    UartAddress uart_address=UART_ADDRESS_0);

  bool communicating();
  uint8_t getVersion();

  void enable();
  void disable();

  // valid values = 1,2,4,8,...128,256, other values get rounded down
  void setMicrostepsPerStep(uint16_t microsteps_per_step);

  void setRunCurrent(uint8_t percent);
  void setHoldCurrent(uint8_t percent);
  void setHoldDelay(uint8_t percent);
  void setAllCurrentValues(uint8_t run_current_percent,
    uint8_t hold_current_percent,
    uint8_t hold_delay_percent);

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
    uint32_t stealth_mode : 1;
    uint32_t standstill : 1;
  };
  const static uint8_t CURRENT_SCALING_MAX = 31;
  Status getStatus();

  void enableInverseMotorDirection();
  void disableInverseMotorDirection();

  void enableStealthChop();
  void enableSpreadCycle();
  enum ZeroHoldCurrentMode
  {
    NORMAL=0,
    FREEWHEELING=1,
    STRONG_BRAKING=2,
    BRAKING=3,
  };
  void setZeroHoldCurrentMode(ZeroHoldCurrentMode mode);

  struct Settings
  {
    uint16_t microsteps_per_step;
    bool inverse_motor_direction_enabled;
    bool spread_cycle_enabled;
    uint8_t zero_hold_current_mode;
    uint8_t irun;
    uint8_t ihold;
    uint8_t iholddelay;
  };
  Settings getSettings();

  void setPwmThreshold(uint32_t value);

private:
  HardwareSerial * serial_ptr_;
  int enable_pin_;

  uint8_t uart_address_;

  // Serial Settings
  const static uint32_t SERIAL_BAUD_RATE = 250000;

  const static uint8_t BYTE_MAX_VALUE = 0xFF;
  const static uint8_t BITS_PER_BYTE = 8;

  const static uint16_t ECHO_DELAY_MAX_VALUE = 20;
  const static uint16_t REPLY_DELAY_MAX_VALUE = 100;

  // Datagrams
  const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
  const static uint8_t DATA_SIZE = 4;
  union WriteReadReplyDatagram
  {
    struct
    {
      uint64_t sync : 4;
      uint64_t reserved : 4;
      uint64_t uart_address : 8;
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
  const static uint8_t READ_REPLY_UART_ADDRESS = 0b11111111;

  const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
  union ReadRequestDatagram
  {
    struct
    {
      uint32_t sync : 4;
      uint32_t reserved : 4;
      uint32_t uart_address : 8;
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
  union InterfaceTransmissionCounter
  {
    struct
    {
      uint32_t ifcnt : 8;
      uint32_t reserved : 24;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_SENDDELAY = 0x03;
  union SendDelay
  {
    struct
    {
      uint32_t reserved_0 : 8;
      uint32_t senddelay : 8;
      uint32_t reserved_1 : 16;
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
      uint32_t pdn_uart : 1;
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

  const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
  union PowerDownDelay
  {
    struct
    {
      uint32_t value : 8;
      uint32_t reserved : 24;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_TSTEP = 0x12;

  const static uint8_t ADDRESS_TPWMTHRS = 0x13;

  const static uint8_t ADDRESS_VACTUAL = 0x22;

  // CoolStep and StallGuard Control Register Set
  const static uint8_t ADDRESS_TCOOLTHRS = 0x14;
  const static uint8_t ADDRESS_SGTHRS = 0x40;
  const static uint8_t ADDRESS_SG_RESULT = 0x41;

  const static uint8_t ADDRESS_COOLCONF = 0x42;
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
      uint32_t hstrt : 3;
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

  const static uint8_t ADDRESS_PWM_SCALE = 0x71;
  const static uint8_t ADDRESS_PWM_AUTO = 0x72;

  void setOperationModeToUart(HardwareSerial & serial,
    UartAddress uart_address=UART_ADDRESS_0);
  void setOperationModeToStandalone();

  // microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
  void setMicrostepsPerStepPowerOfTwo(uint8_t exponent);

  uint32_t reverseData(uint32_t data);
  template<typename Datagram>
  uint8_t calculateCrc(Datagram & datagram,
    uint8_t datagram_size);
  template<typename Datagram>
  void sendDatagram(Datagram & datagram,
    uint8_t datagram_size);

  void write(uint8_t register_address,
    uint32_t data);
  uint32_t read(uint8_t register_address);

  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);
  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);
  uint16_t getMicrostepsPerStep();

  void setGlobalConfig();
  void getGlobalConfig();
  void setDriverCurrent();
  void setChopperConfig();
  void getChopperConfig();
  void setPwmConfig();
  void getPwmConfig();
};

#endif
