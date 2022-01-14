#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;

const int LOOP_DELAY = 100;

const int32_t VELOCITY_MIN = -1000;
const int32_t VELOCITY_MAX = 1000;
const uint8_t RUN_CURRENT_PERCENT = 20;
const uint8_t HOLD_CURRENT_PERCENT = 10;
const uint8_t HOLD_DELAY_PERCENT = 50;

int32_t velocity_inc = 10;
int32_t velocity = 0;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(BAUD);

  stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);
  stepper_driver.setAllCurrentValues(RUN_CURRENT_PERCENT,HOLD_CURRENT_PERCENT,HOLD_DELAY_PERCENT);

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
}

void loop()
{
  stepper_driver.moveAtVelocity(velocity);
  Serial.print("moveAtVelocity(");
  Serial.print(velocity);
  Serial.print(")\n");
  delay(LOOP_DELAY);

  if ((velocity == VELOCITY_MIN) || (velocity == VELOCITY_MAX))
  {
    velocity_inc = -velocity_inc;
  }
  velocity += velocity_inc;
}
