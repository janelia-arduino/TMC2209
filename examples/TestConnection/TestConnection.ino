#include <Arduino.h>
#include <Streaming.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;

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
    Serial << "Communicating with stepper driver!\n";
    Serial << "\n";
  }
  else
  {
    Serial << "Not communicating with stepper driver!\n";
    return;
  }

  TMC2209::Status status = stepper_driver.getStatus();
  Serial << "status.over_temperature_warning = " << status.over_temperature_warning << "\n";
  Serial << "status.over_temperature_shutdown = " << status.over_temperature_shutdown << "\n";
  Serial << "status.short_to_ground_a = " << status.short_to_ground_a << "\n";
  Serial << "status.short_to_ground_b = " << status.short_to_ground_b << "\n";
  Serial << "status.low_side_short_a = " << status.low_side_short_a << "\n";
  Serial << "status.low_side_short_b = " << status.low_side_short_b << "\n";
  Serial << "status.open_load_a = " << status.open_load_a << "\n";
  Serial << "status.open_load_b = " << status.open_load_b << "\n";
  Serial << "status.over_temperature_120c = " << status.over_temperature_120c << "\n";
  Serial << "status.over_temperature_143c = " << status.over_temperature_143c << "\n";
  Serial << "status.over_temperature_150c = " << status.over_temperature_150c << "\n";
  Serial << "status.over_temperature_157c = " << status.over_temperature_157c << "\n";
  Serial << "status.current_scaling = " << status.current_scaling << "\n";
  Serial << "status.stealth_mode = " << status.stealth_mode << "\n";
  Serial << "status.standstill = " << status.standstill << "\n";
  Serial << "\n";

  stepper_driver.setAllCurrentValues(100,50,50);

  TMC2209::Settings settings = stepper_driver.getSettings();
  Serial << "settings.microsteps_per_step = " << settings.microsteps_per_step << "\n";
  Serial << "settings.inverse_motor_direction_enabled = " << settings.inverse_motor_direction_enabled << "\n";
  Serial << "settings.spread_cycle_enabled = " << settings.spread_cycle_enabled << "\n";
  Serial << "settings.zero_hold_current_mode = ";
  switch (settings.zero_hold_current_mode)
  {
    case TMC2209::NORMAL:
      Serial<< "normal\n";
      break;
    case TMC2209::FREEWHEELING:
      Serial<< "freewheeling\n";
      break;
    case TMC2209::STRONG_BRAKING:
      Serial<< "strong_braking\n";
      break;
    case TMC2209::BRAKING:
      Serial<< "braking\n";
      break;
  }
  Serial << "settings.irun = " << settings.irun << "\n";
  Serial << "settings.ihold = " << settings.ihold << "\n";
  Serial << "settings.iholddelay = " << settings.iholddelay << "\n";
  Serial << "settings.automatic_current_scaling_enabled = " << settings.automatic_current_scaling_enabled << "\n";
  Serial << "settings.pwm_offset = " << settings.pwm_offset << "\n";
  Serial << "settings.pwm_gradient = " << settings.pwm_gradient << "\n";
  Serial << "\n";

  Serial << "\n";
  delay(LOOP_DELAY);
}
