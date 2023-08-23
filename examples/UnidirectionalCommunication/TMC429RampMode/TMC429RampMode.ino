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
const int BETWEEN_MOVE_DELAY = 2000;
const int CHECK_AT_POSITION_DELAY = 500;

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
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const long REVS_PER_MOVE = 10;
const long TARGET_POSITION = REVS_PER_MOVE * MICROSTEPS_PER_REV;
const int ACCELERATION_MAX = 2 * MICROSTEPS_PER_REV;
const long ZERO_POSITION = 0;
const long VELOCITY_MAX = 2 * MICROSTEPS_PER_REV;
const long VELOCITY_MIN = 500;

// Instantiate stepper controller
TMC429 stepper_controller;

long target_position, actual_position;

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
  stepper_controller.disableRightSwitchStop(MOTOR_INDEX);
  stepper_controller.setLimitsInHz(MOTOR_INDEX, VELOCITY_MIN, VELOCITY_MAX, ACCELERATION_MAX);
  stepper_controller.setRampMode(MOTOR_INDEX);

  stepper_controller.setActualPosition(MOTOR_INDEX, ZERO_POSITION);
  stepper_controller.setTargetPosition(MOTOR_INDEX, ZERO_POSITION);

  stepper_driver.enable();
}

void loop()
{
  actual_position = stepper_controller.getActualPosition(MOTOR_INDEX);
  Serial.print("actual position: ");
  Serial.println(actual_position);
  Serial.println();
  delay(BETWEEN_MOVE_DELAY);

  Serial.print("set target position: ");
  Serial.println(TARGET_POSITION);
  stepper_controller.setTargetPosition(MOTOR_INDEX, TARGET_POSITION);
  while (!stepper_controller.atTargetPosition(MOTOR_INDEX))
  {
    Serial.print("target position: ");
    Serial.println(stepper_controller.getTargetPosition(MOTOR_INDEX));
    Serial.print("actual position: ");
    Serial.println(stepper_controller.getActualPosition(MOTOR_INDEX));
    delay(CHECK_AT_POSITION_DELAY);
  }
  Serial.println("at target position!");
  Serial.println();
  delay(BETWEEN_MOVE_DELAY);

  Serial.print("set target position: ");
  Serial.println(ZERO_POSITION);
  stepper_controller.setTargetPosition(MOTOR_INDEX, ZERO_POSITION);
  while (!stepper_controller.atTargetPosition(MOTOR_INDEX))
  {
    Serial.print("target position: ");
    Serial.println(stepper_controller.getTargetPosition(MOTOR_INDEX));
    Serial.print("actual position: ");
    Serial.println(stepper_controller.getActualPosition(MOTOR_INDEX));
    delay(CHECK_AT_POSITION_DELAY);
  }
  Serial.println();
  delay(BETWEEN_MOVE_DELAY);
}
