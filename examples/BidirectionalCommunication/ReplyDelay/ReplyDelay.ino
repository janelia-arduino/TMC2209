#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;
const uint8_t REPEAT_COUNT_MAX = 2;

// Instantiate TMC2209
TMC2209 stepper_driver;

uint8_t reply_delay;
uint8_t repeat_count;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);

  reply_delay = 0;
  stepper_driver.setReplyDelay(reply_delay);

  repeat_count = 0;
}

void loop()
{
  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver is setup and communicating!");
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
  }
  Serial.print("repeat count ");
  Serial.print(repeat_count);
  Serial.print(" out of ");
  Serial.println(REPEAT_COUNT_MAX);
  Serial.print("reply delay = ");
  Serial.println(reply_delay);
  Serial.println();
  delay(DELAY);

  if (++repeat_count >= REPEAT_COUNT_MAX)
  {
    repeat_count = 0;
    if (++reply_delay >= stepper_driver.REPLY_DELAY_MAX)
    {
      reply_delay = 0;
    }
    stepper_driver.setReplyDelay(reply_delay);
  }
}
