#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/
//
// To make this library work with those boards, refer to this library example:
// examples/UnidirectionalCommunication/SoftwareSerial

HardwareSerial & serial_stream = Serial3;

const uint8_t STEP_PIN = 2;
const uint8_t DIRECTION_PIN = 3;
const uint32_t STEP_COUNT = 51200;
const uint16_t HALF_STEP_DURATION_MICROSECONDS = 10;
const uint16_t STOP_DURATION = 1000;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;


// Instantiate TMC2209
TMC2209 stepper_driver;

void setup()
{
  stepper_driver.setup(serial_stream);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();
  stepper_driver.enable();
}

void loop()
{
  // One step takes two iterations through the for loop
  for (uint32_t i=0; i<STEP_COUNT*2; ++i)
  {
    digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
    delayMicroseconds(HALF_STEP_DURATION_MICROSECONDS);
  }
  digitalWrite(DIRECTION_PIN, !digitalRead(DIRECTION_PIN));
  delay(STOP_DURATION);
}
