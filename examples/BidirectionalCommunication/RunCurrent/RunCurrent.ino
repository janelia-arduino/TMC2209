#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

// Identify which microcontroller serial port is connected to the TMC2209
// e.g. Serial1, Serial2...
HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 2000;
const int32_t VELOCITY = 10000;
const uint8_t PERCENT_MIN = 0;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t PERCENT_MAX = 100;
const uint8_t PERCENT_INC = 10;

uint8_t run_current_percent = PERCENT_INC;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);

  stepper_driver.enable();
  stepper_driver.moveAtVelocity(VELOCITY);
}

void loop()
{
  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  Serial.print("setRunCurrent(");
  Serial.print(run_current_percent);
  Serial.println(")");
  stepper_driver.setRunCurrent(run_current_percent);
  delay(DELAY);

  TMC2209::Status status = stepper_driver.getStatus();
  Serial.print("status.current_scaling = ");
  Serial.println(status.current_scaling);
  Serial.println();

  if (run_current_percent == PERCENT_MAX)
  {
    run_current_percent = PERCENT_MIN;
  }
  run_current_percent += PERCENT_INC;
}
