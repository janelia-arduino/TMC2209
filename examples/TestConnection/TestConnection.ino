#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;

const uint8_t RUN_CURRENT_PERCENT = 20;
const uint8_t HOLD_CURRENT_PERCENT = 10;
const uint8_t HOLD_DELAY_PERCENT = 50;

const int LOOP_DELAY = 2000;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(BAUD);

  stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);
}

void loop()
{
  if (stepper_driver.communicating())
  {
    Serial.print("Communicating with stepper driver!\n");
    Serial.print("\n");
  }
  else
  {
    Serial.print("Not communicating with stepper driver!\n");
    return;
  }

  TMC2209::Status status = stepper_driver.getStatus();
  Serial.print("status.over_temperature_warning = ");
  Serial.print(status.over_temperature_warning);
  Serial.print("\n");
  Serial.print("status.over_temperature_shutdown = ");
  Serial.print(status.over_temperature_shutdown);
  Serial.print("\n");
  Serial.print("status.short_to_ground_a = ");
  Serial.print(status.short_to_ground_a);
  Serial.print("\n");
  Serial.print("status.short_to_ground_b = ");
  Serial.print(status.short_to_ground_b);
  Serial.print("\n");
  Serial.print("status.low_side_short_a = ");
  Serial.print(status.low_side_short_a);
  Serial.print("\n");
  Serial.print("status.low_side_short_b = ");
  Serial.print(status.low_side_short_b);
  Serial.print("\n");
  Serial.print("status.open_load_a = ");
  Serial.print(status.open_load_a);
  Serial.print("\n");
  Serial.print("status.open_load_b = ");
  Serial.print(status.open_load_b);
  Serial.print("\n");
  Serial.print("status.over_temperature_120c = ");
  Serial.print(status.over_temperature_120c);
  Serial.print("\n");
  Serial.print("status.over_temperature_143c = ");
  Serial.print(status.over_temperature_143c);
  Serial.print("\n");
  Serial.print("status.over_temperature_150c = ");
  Serial.print(status.over_temperature_150c);
  Serial.print("\n");
  Serial.print("status.over_temperature_157c = ");
  Serial.print(status.over_temperature_157c);
  Serial.print("\n");
  Serial.print("status.current_scaling = ");
  Serial.print(status.current_scaling);
  Serial.print("\n");
  Serial.print("status.stealth_mode = ");
  Serial.print(status.stealth_mode);
  Serial.print("\n");
  Serial.print("status.standstill = ");
  Serial.print(status.standstill);
  Serial.print("\n");
  Serial.print("\n");

  stepper_driver.setAllCurrentValues(RUN_CURRENT_PERCENT,HOLD_CURRENT_PERCENT,HOLD_DELAY_PERCENT);

  TMC2209::Settings settings = stepper_driver.getSettings();
  Serial.print("settings.microsteps_per_step = ");
  Serial.print(settings.microsteps_per_step);
  Serial.print("\n");
  Serial.print("settings.inverse_motor_direction_enabled = ");
  Serial.print(settings.inverse_motor_direction_enabled);
  Serial.print("\n");
  Serial.print("settings.spread_cycle_enabled = ");
  Serial.print(settings.spread_cycle_enabled);
  Serial.print("\n");
  Serial.print("settings.zero_hold_current_mode = ");
  switch (settings.zero_hold_current_mode)
  {
    case TMC2209::NORMAL:
      Serial.print("normal\n");
      break;
    case TMC2209::FREEWHEELING:
      Serial.print("freewheeling\n");
      break;
    case TMC2209::STRONG_BRAKING:
      Serial.print("strong_braking\n");
      break;
    case TMC2209::BRAKING:
      Serial.print("braking\n");
      break;
  }
  Serial.print("settings.irun = ");
  Serial.print(settings.irun);
  Serial.print("\n");
  Serial.print("settings.ihold = ");
  Serial.print(settings.ihold);
  Serial.print("\n");
  Serial.print("settings.iholddelay = ");
  Serial.print(settings.iholddelay);
  Serial.print("\n");
  Serial.print("settings.automatic_current_scaling_enabled = ");
  Serial.print(settings.automatic_current_scaling_enabled);
  Serial.print("\n");
  Serial.print("settings.pwm_offset = ");
  Serial.print(settings.pwm_offset);
  Serial.print("\n");
  Serial.print("settings.pwm_gradient = ");
  Serial.print(settings.pwm_gradient);
  Serial.print("\n");
  Serial.print("\n");

  Serial.print("\n");
  delay(LOOP_DELAY);
}
