#include <TMC2209.h>

// This example is meant to test the RP2040
//
// See this reference for more details:
// https://arduino-pico.readthedocs.io/en/latest/serial.html

SerialUART & serial_stream1 = Serial1;
HardwareSerial & serial_stream2 = Serial2;
SerialPIO Serial3(27,26);

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;
const uint8_t SERIAL_RX_PIN1 = 9;
const uint8_t SERIAL_TX_PIN1 = 8;

// Instantiate TMC2209
TMC2209 stepper_driver1;
TMC2209 stepper_driver2;
TMC2209 stepper_driver3;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver1.setup(serial_stream1, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, SERIAL_RX_PIN1, SERIAL_TX_PIN1);
  stepper_driver2.setup(serial_stream2, SERIAL_BAUD_RATE);
  stepper_driver3.setup(Serial3, SERIAL_BAUD_RATE);
}

void loop()
{
  if (stepper_driver1.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver is setup and communicating!");
    Serial.println("Try turning driver power off to see what happens.");
  }
  else if (stepper_driver1.isCommunicatingButNotSetup())
  {
    Serial.println("Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    stepper_driver1.setup(serial_stream1);
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }
  Serial.println();
  delay(DELAY);
}
