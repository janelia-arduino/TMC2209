#include <TMC2209.h>
#include <TMC429.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/
//
// To make this library work with those boards, refer to this library example:
// examples/UnidirectionalCommunication/SoftwareSerial

const long SERIAL_BAUD_RATE = 115200;
const int LOOP_DELAY = 1000;

// Stepper driver settings
HardwareSerial & serial_stream = Serial3;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const int RUN_CURRENT_PERCENT = 100;
const int MICROSTEPS_PER_STEP = 256;

// Instantiate stepper driver
TMC2209 stepper_driver;

// Stepper controller settings
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 2;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const int ACCELERATION_MAX = MICROSTEPS_PER_REV / 2;
const long VELOCITY_MAX = REVS_PER_SEC_MAX * MICROSTEPS_PER_REV;
const long VELOCITY_MIN = 50;

// Instantiate stepper controller
TMC429 stepper_controller;

long target_velocity, actual_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.enableAutomaticGradientAdaptation();
  stepper_driver.enableCoolStep();

  stepper_controller.setup(CHIP_SELECT_PIN, CLOCK_FREQUENCY_MHZ);
  stepper_controller.disableLeftSwitchStop(MOTOR_INDEX);
  stepper_controller.disableRightSwitches();
  stepper_controller.setVelocityMode(MOTOR_INDEX);
  stepper_controller.setLimitsInHz(MOTOR_INDEX, VELOCITY_MIN, VELOCITY_MAX, ACCELERATION_MAX);

  stepper_driver.enable();
}

void loop()
{
  delay(LOOP_DELAY);

  stepper_controller.setTargetVelocityInHz(MOTOR_INDEX, VELOCITY_MAX);
  Serial.print("set target_velocity: ");
  Serial.println(target_velocity);
  Serial.print("at target_velocity: ");
  Serial.println(stepper_controller.atTargetVelocity(MOTOR_INDEX));

  target_velocity = stepper_controller.getTargetVelocityInHz(MOTOR_INDEX);

  actual_velocity = stepper_controller.getActualVelocityInHz(MOTOR_INDEX);
  Serial.print("actual_velocity: ");
  Serial.println(actual_velocity);
}
