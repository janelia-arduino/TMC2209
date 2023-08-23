#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 500;
const int32_t VELOCITY = 20000;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;
const uint8_t LOOPS_BEFORE_TOGGLING = 3;
const TMC2209::CurrentIncrement CURRENT_INCREMENT = TMC2209::CURRENT_INCREMENT_8;
const TMC2209::MeasurementCount MEASUREMENT_COUNT = TMC2209::MEASUREMENT_COUNT_1;
const uint32_t COOL_STEP_DURATION_THRESHOLD = 2000;
const uint8_t COOL_STEP_LOWER_THRESHOLD = 1;
const uint8_t COOL_STEP_UPPER_THRESHOLD = 0;

uint8_t loop_count = 0;
bool cool_step_enabled = false;


// Instantiate TMC2209
TMC2209 stepper_driver;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setCoolStepCurrentIncrement(CURRENT_INCREMENT);
  stepper_driver.setCoolStepMeasurementCount(MEASUREMENT_COUNT);
  stepper_driver.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
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

  Serial.println();

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
