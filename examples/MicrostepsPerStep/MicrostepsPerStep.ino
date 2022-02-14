#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 4000;
const int32_t VELOCITY = 200;
const uint8_t RUN_CURRENT_PERCENT = 60;
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
  Serial.println("");

  microsteps_per_step_exponent += MICROSTEPS_PER_STEP_EXPONENT_INC;
  if (microsteps_per_step_exponent > MICROSTEPS_PER_STEP_EXPONENT_MAX)
  {
    microsteps_per_step_exponent = MICROSTEPS_PER_STEP_EXPONENT_MIN;
  }
}
