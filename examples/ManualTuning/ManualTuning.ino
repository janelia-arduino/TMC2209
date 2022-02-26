#include <Arduino.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int32_t RUN_VELOCITY = 80000;
const int32_t STOP_VELOCITY = 0;
const int RUN_DURATION = 500;
const int STOP_DURATION = 1000;
const uint8_t RUN_CURRENT_PERCENT = 100;
const uint8_t HOLD_CURRENT_PERCENT = 0;
const uint8_t HOLD_DELAY_PERCENT = 0;
const uint8_t PWM_OFFSET = 80;
const uint8_t PWM_GRADIENT = 20;


// Instantiate TMC2209
TMC2209 stepper_driver;
bool invert_direction = false;

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

  stepper_driver.setAllCurrentValues(RUN_CURRENT_PERCENT,
    HOLD_CURRENT_PERCENT,
    HOLD_DELAY_PERCENT);
  stepper_driver.setPwmOffset(PWM_OFFSET);
  stepper_driver.setPwmGradient(PWM_GRADIENT);
  stepper_driver.enable();
}

void loop()
{
  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  stepper_driver.moveAtVelocity(STOP_VELOCITY);
  delay(STOP_DURATION);
  if (invert_direction)
  {
    stepper_driver.enableInverseMotorDirection();
  }
  else
  {
    stepper_driver.disableInverseMotorDirection();
  }
  invert_direction = not invert_direction;

  stepper_driver.moveAtVelocity(RUN_VELOCITY);

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
    if (invert_direction)
    {
      Serial.print("-");
    }
    Serial.println(RUN_VELOCITY);

    uint32_t interstep_duration = stepper_driver.getInterstepDuration();
    Serial.print("which is equal to an interstep_duration of ");
    Serial.println(interstep_duration);

    Serial.print("status.current_scaling = ");
    Serial.println(status.current_scaling);

    uint32_t pwm_offset_auto = stepper_driver.getPwmOffsetAuto();
    Serial.print("pwm_offset_auto ");
    Serial.println(pwm_offset_auto);

    uint32_t pwm_gradient_auto = stepper_driver.getPwmGradientAuto();
    Serial.print("pwm_gradient_auto ");
    Serial.println(pwm_gradient_auto);

    TMC2209::Settings settings = stepper_driver.getSettings();
    Serial.print("settings.irun_percent = ");
    Serial.println(settings.irun_percent);
    Serial.print("settings.automatic_current_scaling_enabled = ");
    Serial.println(settings.automatic_current_scaling_enabled);
    Serial.print("settings.automatic_gradient_adaptation_enabled = ");
    Serial.println(settings.automatic_gradient_adaptation_enabled);
    Serial.print("settings.pwm_offset = ");
    Serial.println(settings.pwm_offset);
    Serial.print("settings.pwm_gradient = ");
    Serial.println(settings.pwm_gradient);
    Serial.print("settings.cool_step_enabled = ");
    Serial.println(settings.cool_step_enabled);
  }
  else
  {
    Serial.println("Not moving, something is wrong!");
  }

  Serial.println("");
  delay(RUN_DURATION);
}
