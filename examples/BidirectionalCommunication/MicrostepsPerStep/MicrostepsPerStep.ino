#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 4000;
const int32_t VELOCITY = 200;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;
const uint8_t MICROSTEPS_PER_STEP_EXPONENT_MIN = 0;
const uint8_t MICROSTEPS_PER_STEP_EXPONENT_MAX = 8;
const uint8_t MICROSTEPS_PER_STEP_EXPONENT_INC = 1;

uint8_t microsteps_per_step_exponent = MICROSTEPS_PER_STEP_EXPONENT_MIN;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);

  stepper_driver.setMicrostepsPerStepPowerOfTwo(microsteps_per_step_exponent);
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

  Serial.print("setMicrostepsPerStepPowerOfTwo(");
  Serial.print(microsteps_per_step_exponent);
  Serial.println(")");
  stepper_driver.setMicrostepsPerStepPowerOfTwo(microsteps_per_step_exponent);
  Serial.print("getMicrostepsPerStep() = ");
  Serial.println(stepper_driver.getMicrostepsPerStep());
  delay(DELAY);

  uint32_t interstep_duration = stepper_driver.getInterstepDuration();
  Serial.print("interstep_duration = ");
  Serial.println(interstep_duration);
  Serial.println();

  microsteps_per_step_exponent += MICROSTEPS_PER_STEP_EXPONENT_INC;
  if (microsteps_per_step_exponent > MICROSTEPS_PER_STEP_EXPONENT_MAX)
  {
    microsteps_per_step_exponent = MICROSTEPS_PER_STEP_EXPONENT_MIN;
  }
}
