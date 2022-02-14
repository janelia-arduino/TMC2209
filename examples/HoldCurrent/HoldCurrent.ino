#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 2000;
const int32_t VELOCITY = 10000;
const int32_t VELOCITY_STOPPED = 0;
const uint8_t PERCENT_MIN = 0;
const uint8_t PERCENT_MAX = 100;
const uint8_t PERCENT_INC = 10;
const uint8_t RUN_CURRENT_PERCENT = 50;

uint8_t hold_current_percent = PERCENT_INC;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);

  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver setup and communicating!");
    Serial.println("");
  }
  else
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  stepper_driver.enable();
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
}

void loop()
{
  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  Serial.println("enable and run");
  stepper_driver.enable();
  stepper_driver.moveAtVelocity(VELOCITY);

  Serial.print("setHoldCurrent(");
  Serial.print(hold_current_percent);
  Serial.println(")");
  stepper_driver.setHoldCurrent(hold_current_percent);
  delay(DELAY);

  Serial.println("stop");
  stepper_driver.moveAtVelocity(VELOCITY_STOPPED);
  delay(DELAY);

  uint8_t pwm_scale_sum = stepper_driver.getPwmScaleSum();
  Serial.print("pwm_scale_sum = ");
  Serial.println(pwm_scale_sum);
  delay(DELAY);

  stepper_driver.disable();
  Serial.println("disable");
  Serial.println("");
  delay(DELAY);

  if (hold_current_percent == PERCENT_MAX)
  {
    hold_current_percent = PERCENT_MIN;
  }
  hold_current_percent += PERCENT_INC;
}
