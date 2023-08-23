#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
}

void loop()
{
  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver is setup and communicating!");
    Serial.println("Try turning driver power off to see what happens.");
  }
  else if (stepper_driver.isCommunicatingButNotSetup())
  {
    Serial.println("Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }
  Serial.println();
  delay(DELAY);
}
