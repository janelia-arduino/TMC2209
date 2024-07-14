#include <TMC2209.h>

// This example is meant to test a FYSETC-ERB board containing an RP2040 and
// two TMC2209 chips
//
// See this reference for more details:
// https://github.com/FYSETC/FYSETC-ERB


const uint8_t X_TX_PIN = 20;
const uint8_t Y_TX_PIN = 17;
const long SERIAL_BAUD_RATE = 115200;

SerialPIO x_serial(X_TX_PIN, SerialPIO::NOPIN);
SerialPIO y_serial(Y_TX_PIN, SerialPIO::NOPIN);

// Instantiate TMC2209
TMC2209 x_stepper_driver;
TMC2209 y_stepper_driver;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;

const int32_t RUN_VELOCITY = 20000;


void setup()
{
  x_stepper_driver.setup(x_serial, SERIAL_BAUD_RATE);
  x_stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  x_stepper_driver.enableCoolStep();
  x_stepper_driver.enable();
  x_stepper_driver.moveAtVelocity(RUN_VELOCITY);

  y_stepper_driver.setup(y_serial, SERIAL_BAUD_RATE);
  y_stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  y_stepper_driver.enableCoolStep();
  y_stepper_driver.enable();
  y_stepper_driver.moveAtVelocity(RUN_VELOCITY);
}

void loop()
{
}
