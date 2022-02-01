#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;
const int DELAY = 2000;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(BAUD);

  stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);
}

void loop()
{
  Serial.println("Try turning driver power off, waiting, and on again to see what happens.");
  if (stepper_driver.isCommunicating())
  {
    Serial.println("Stepper driver is communicating!");
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
  }
  if (stepper_driver.isSetup())
  {
    Serial.println("Stepper driver is setup!");
  }
  else
  {
    Serial.println("Stepper driver is not setup!");
  }
  Serial.println("");
  delay(DELAY);
}
