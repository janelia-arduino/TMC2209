#include <Arduino.h>
#include <Streaming.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;

const int LOOP_DELAY = 2000;

const int32_t velocity = 100;

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

  stepper_driver.moveAtVelocity(velocity);
  Serial << "moveAtVelocity(" << velocity << ")\n";
  delay(LOOP_DELAY);

  stepper_driver.moveAtVelocity(0);
  Serial << "moveAtVelocity(" << 0 << ")\n";
  delay(LOOP_DELAY);

  stepper_driver.moveAtVelocity(-velocity);
  Serial << "moveAtVelocity(" << -velocity << ")\n";
  delay(LOOP_DELAY);

  Serial << "\n";
}
