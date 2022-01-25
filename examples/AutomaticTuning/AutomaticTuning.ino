#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;
const int DELAY = 4000;
const int32_t VELOCITY_STOPPED = 0;
const int32_t VELOCITY = 20000;
const uint8_t RUN_CURRENT_PERCENT = 40;
const uint8_t HOLD_CURRENT_PERCENT = 100;
const uint8_t HOLD_CURRENT_PERCENT_REDUCED = 10;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(BAUD);

  stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);

  if (stepper_driver.communicating())
  {
    Serial.println("Communicating with stepper driver!");
    Serial.println("");
  }
  else
  {
    Serial.println("Not communicating with stepper driver!");
    return;
  }

  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.enableAutomaticGradientAdaptation();
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setHoldCurrent(HOLD_CURRENT_PERCENT);
  stepper_driver.enable();
}

void loop()
{
  if (not stepper_driver.communicating())
  {
    Serial.println("Not communicating with stepper driver!");
    return;
  }

  stepper_driver.moveAtVelocity(VELOCITY_STOPPED);
  Serial.println("Holding...");
  delay(DELAY);

  uint8_t pwm_scale_sum = stepper_driver.getPwmScaleSum();
  Serial.print("pwm_scale_sum = ");
  Serial.println(pwm_scale_sum);

  stepper_driver.setHoldCurrent(HOLD_CURRENT_PERCENT_REDUCED);

  stepper_driver.moveAtVelocity(VELOCITY);
  Serial.println("moving...");
  delay(DELAY);

  pwm_scale_sum = stepper_driver.getPwmScaleSum();
  Serial.print("pwm_scale_sum = ");
  Serial.println(pwm_scale_sum);

  Serial.println("");
}
