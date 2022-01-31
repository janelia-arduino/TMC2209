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

  // stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);
}

void loop()
{
  Serial.println("Driver not setup properly!");

  if (stepper_driver.communicating())
  {
    Serial.println("Communicating with stepper driver!");
    Serial.println("");
  }
  else
  {
    Serial.println("Not communicating with stepper driver!");
  }
  Serial.println("");
  delay(DELAY);
}
