#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 2000;
const int32_t VELOCITY = 20000;
const uint8_t RUN_CURRENT_PERCENT = 40;


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

  bool disabled_by_input_pin = stepper_driver.disabledByInputPin();
  TMC2209::Settings settings = stepper_driver.getSettings();
  TMC2209::Status status = stepper_driver.getStatus();

  if (disabled_by_input_pin)
  {
    Serial.println("Stepper driver is disabled by input pin!");
  }
  else if (not settings.enabled)
  {
    Serial.println("Stepper driver is disabled by firmware!");
  }
  else if ((not status.standstill))
  {
    Serial.print("Moving at velocity ");
    Serial.println(VELOCITY);

    uint32_t interstep_duration = stepper_driver.getInterstepDuration();
    Serial.print("which is equal to an interstep_duration of ");
    Serial.println(interstep_duration);
  }
  else
  {
    Serial.println("Not moving, something is wrong!");
  }

  Serial.println("");
  delay(DELAY);
}
