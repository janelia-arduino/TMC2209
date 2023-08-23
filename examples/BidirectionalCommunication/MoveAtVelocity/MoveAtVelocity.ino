#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 2000;
const int32_t VELOCITY = 20000;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
  delay(DELAY);

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();
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

  bool hardware_disabled = stepper_driver.hardwareDisabled();
  TMC2209::Settings settings = stepper_driver.getSettings();
  TMC2209::Status status = stepper_driver.getStatus();

  if (hardware_disabled)
  {
    Serial.println("Stepper driver is hardware disabled!");
  }
  else if (not settings.software_enabled)
  {
    Serial.println("Stepper driver is software disabled!");
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

  Serial.println();
  delay(DELAY);
}
