#include <TMC2209.h>

// This example is meant to test the RP2040
//
// See this reference for more details:
// https://arduino-pico.readthedocs.io/en/latest/serial.html

SerialUART & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;
const uint8_t SERIAL_RX_PIN = 9;
const uint8_t SERIAL_TX_PIN = 8;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, SERIAL_RX_PIN, SERIAL_TX_PIN);
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
