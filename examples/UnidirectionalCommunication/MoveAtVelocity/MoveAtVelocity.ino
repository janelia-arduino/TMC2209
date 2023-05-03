#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const int32_t RUN_VELOCITY = 20000;
const int32_t STOP_VELOCITY = 0;
const int RUN_DURATION = 2000;
const int STOP_DURATION = 1000;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;


// Instantiate TMC2209
TMC2209 stepper_driver;
bool invert_direction = false;

void setup()
{
  stepper_driver.setup(serial_stream);

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();
  stepper_driver.enable();
}

void loop()
{
  stepper_driver.moveAtVelocity(STOP_VELOCITY);
  delay(STOP_DURATION);
  if (invert_direction)
  {
    stepper_driver.enableInverseMotorDirection();
  }
  else
  {
    stepper_driver.disableInverseMotorDirection();
  }
  invert_direction = not invert_direction;

  stepper_driver.moveAtVelocity(RUN_VELOCITY);

  delay(RUN_DURATION);
}
