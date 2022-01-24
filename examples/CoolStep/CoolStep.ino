#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long BAUD = 115200;
const int DELAY = 500;
const int32_t VELOCITY = 20000;
const uint8_t RUN_CURRENT_PERCENT = 60;
const uint8_t LOOPS_BEFORE_TOGGLING = 3;
const uint8_t COOL_STEP_LOWER_THRESHOLD = 1;
const uint8_t COOL_STEP_UPPER_THRESHOLD = 0;
const uint32_t COOL_STEP_DURATION_THRESHOLD = 2000;
const TMC2209::CurrentIncrementStep STEP_WIDTH = TMC2209::STEP_WIDTH_8;
const TMC2209::MeasurementsPerDecrement MEASUREMENTS = TMC2209::MEASUREMENTS_1;

uint8_t loop_count = 0;
bool cool_step_enabled = false;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(BAUD);

  stepper_driver.setup(serial_stream,TMC2209::SERIAL_ADDRESS_0);

  if (stepper_driver.communicating())
  {
    Serial.println("Communicating with stepper driver!");
    Serial.println("");
  }
  else
  {
    Serial.println("Not communicating with stepper driver!");
    return;
  }

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.disableAutomaticCurrentScaling();
  stepper_driver.disableAutomaticGradientAdaptation();
  stepper_driver.setCurrentIncrementStep(STEP_WIDTH);
  stepper_driver.setMeasurementsPerDecrement(MEASUREMENTS);
  stepper_driver.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  stepper_driver.enable();
  stepper_driver.moveAtVelocity(VELOCITY);
}

void loop()
{
  if (not stepper_driver.communicating())
  {
    Serial.println("Not communicating with stepper driver!");
    return;
  }

  if (cool_step_enabled)
  {
    Serial.println("cool step enabled");
  }
  else
  {
    Serial.println("cool step disabled");
  }

  uint32_t interstep_duration = stepper_driver.getInterstepDuration();
  Serial.print("interstep_duration = ");
  Serial.println(interstep_duration);
  delay(DELAY);

  uint16_t stall_guard_result = stepper_driver.getStallGuardResult();
  Serial.print("stall_guard_result = ");
  Serial.println(stall_guard_result);
  delay(DELAY);

  uint8_t pwm_scale_sum = stepper_driver.getPwmScaleSum();
  Serial.print("pwm_scale_sum = ");
  Serial.println(pwm_scale_sum);
  delay(DELAY);

  int16_t pwm_scale_auto = stepper_driver.getPwmScaleAuto();
  Serial.print("pwm_scale_auto = ");
  Serial.println(pwm_scale_auto);
  delay(DELAY);

  uint8_t pwm_offset_auto = stepper_driver.getPwmOffsetAuto();
  Serial.print("pwm_offset_auto = ");
  Serial.println(pwm_offset_auto);
  delay(DELAY);

  uint8_t pwm_gradient_auto = stepper_driver.getPwmGradientAuto();
  Serial.print("pwm_gradient_auto = ");
  Serial.println(pwm_gradient_auto);
  delay(DELAY);

  TMC2209::Status status = stepper_driver.getStatus();
  Serial.print("status.current_scaling = ");
  Serial.println(status.current_scaling);

  TMC2209::Settings settings = stepper_driver.getSettings();
  Serial.print("settings.irun_register_value = ");
  Serial.println(settings.irun_register_value);

  Serial.println("");

  if (++loop_count == LOOPS_BEFORE_TOGGLING)
  {
    loop_count = 0;
    cool_step_enabled = not cool_step_enabled;
    if (cool_step_enabled)
    {
      stepper_driver.enableCoolStep(COOL_STEP_LOWER_THRESHOLD,
        COOL_STEP_UPPER_THRESHOLD);
    }
    else
    {
      stepper_driver.disableCoolStep();
    }
  }
}
