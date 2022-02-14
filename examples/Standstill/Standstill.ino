#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 4000;
const uint8_t RUN_CURRENT_PERCENT = 100;
const uint8_t HOLD_CURRENT_STANDSTILL = 0;

// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);

  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver setup and communicating!");
    Serial.println("");
  }
  else
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setHoldCurrent(HOLD_CURRENT_STANDSTILL);
  stepper_driver.enable();
}

void loop()
{
  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  Serial.println("standstill mode = NORMAL");
  stepper_driver.setStandstillMode(stepper_driver.NORMAL);
  delay(DELAY);

  Serial.println("standstill mode = FREEWHEELING");
  stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
  delay(DELAY);

  Serial.println("standstill mode = STRONG_BRAKING");
  stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);
  delay(DELAY);

  Serial.println("standstill mode = BRAKING");
  stepper_driver.setStandstillMode(stepper_driver.BRAKING);
  delay(DELAY);

  Serial.println("");
}
