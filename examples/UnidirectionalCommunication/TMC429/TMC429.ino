#include <Arduino.h>
#include <TMC2209.h>
#include <TMC429.h>

const long SERIAL_BAUD_RATE = 115200;
const int LOOP_DELAY = 1000;

// Stepper driver settings
HardwareSerial & serial_stream = Serial1;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const int RUN_CURRENT_PERCENT = 10;
const int MICROSTEPS_PER_STEP = 256;

// Instantiate stepper driver
TMC2209 stepper_driver;

// Stepper controller settings
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 2;
const int ACCELERATION_MAX = 10;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const long VELOCITY_MAX = 5000;
const long VELOCITY_MIN = 100;

// Instantiate stepper controller
TMC429 stepper_controller;

long target_velocity, actual_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();

  stepper_controller.setup(CHIP_SELECT_PIN, CLOCK_FREQUENCY_MHZ);
  stepper_controller.initialize();
  stepper_controller.disableLeftSwitchStop(MOTOR_INDEX);
  stepper_controller.disableRightSwitches();
  stepper_controller.setVelocityMode(MOTOR_INDEX);
  stepper_controller.setLimitsInHz(MOTOR_INDEX, VELOCITY_MIN, VELOCITY_MAX, ACCELERATION_MAX);

  stepper_driver.enable();
}

void loop()
{
  stepper_controller.setTargetVelocityInHz(MOTOR_INDEX, VELOCITY_MAX);
  Serial.print("set target_velocity: ");
  Serial.println(target_velocity);
  Serial.print("at target_velocity: ");
  Serial.println(stepper_controller.atTargetVelocity(MOTOR_INDEX));

  target_velocity = stepper_controller.getTargetVelocityInHz(MOTOR_INDEX);

  actual_velocity = stepper_controller.getActualVelocityInHz(MOTOR_INDEX);
  Serial.print("actual_velocity: ");
  Serial.println(actual_velocity);
  delay(LOOP_DELAY);
}
