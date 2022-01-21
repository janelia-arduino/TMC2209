#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;
const int DELAY = 4000;
const int32_t VELOCITY = 10000;
const uint8_t PERCENT_MIN = 0;
const uint8_t PERCENT_MAX = 100;
const uint8_t PERCENT_INC = 10;

uint8_t run_current_percent = PERCENT_INC;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(BAUD);

  stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);

  if (stepper_driver.communicating())
  {
    Serial.println("Communicating with stepper driver!\n");
  }
  else
  {
    Serial.println("Not communicating with stepper driver!");
    return;
  }

  stepper_driver.enable();
  stepper_driver.moveAtVelocity(VELOCITY);
}

void loop()
{
  if (not stepper_driver.communicating())
  {
    Serial.println("Not communicating with stepper driver!");
    return;
  }

  Serial.print("setRunCurrent(");
  Serial.print(run_current_percent);
  Serial.println(")");
  stepper_driver.setRunCurrent(run_current_percent);
  delay(DELAY);

  if (run_current_percent == PERCENT_MAX)
  {
    run_current_percent = PERCENT_MIN;
  }
  run_current_percent += PERCENT_INC;
}
