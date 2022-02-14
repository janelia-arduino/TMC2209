#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
// const long SERIAL1_BAUD_RATE = 500000;
// const long SERIAL1_BAUD_RATE = 115200;
// const long SERIAL1_BAUD_RATE = 57600;
// const long SERIAL1_BAUD_RATE = 38400;
// const long SERIAL1_BAUD_RATE = 28800;
const long SERIAL1_BAUD_RATE = 9600;
const int DELAY = 2000;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream,SERIAL1_BAUD_RATE);
}

void loop()
{
  Serial.println("*************************");
  Serial.print("serial1_baud_rate = ");
  Serial.println(SERIAL1_BAUD_RATE);

  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver setup and communicating!");
  }
  else
  {
    Serial.println("Stepper driver not setup and communicating!");
  }

  Serial.println("*************************");
  Serial.println("");
  delay(DELAY);
}
