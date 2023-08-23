#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 6000;

const int LOOP_COUNT = 100;

unsigned long time_begin;
unsigned long time_end;
TMC2209::Status status;

typedef void (*DriverFunction)(void);

// Instantiate TMC2209
TMC2209 stepper_driver;

void driverWriteFunction()
{
  stepper_driver.enable();
}

void driverReadFunction()
{
  status = stepper_driver.getStatus();
}

void runTestLoop(DriverFunction driver_function)
{
  time_begin = millis();
  for (uint16_t i=0; i<LOOP_COUNT; ++i)
  {
    driver_function();
  }
  time_end = millis();
  Serial.print("Loop ran ");
  Serial.print(LOOP_COUNT);
  Serial.print(" times and took ");
  Serial.print(time_end - time_begin);
  Serial.println(" milliseconds");
  Serial.println();
}

void runTestLoops()
{
  Serial.println("Running driver WRITE test loop, please wait...");
  runTestLoop(driverWriteFunction);
  Serial.println("Running driver READ test loop, please wait...");
  runTestLoop(driverReadFunction);
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
}

void loop()
{
  Serial.println("*************************");
  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver is setup and communicating!");
    runTestLoops();
    Serial.println("Try turning driver power off to see what happens.");
  }
  else if (stepper_driver.isCommunicatingButNotSetup())
  {
    Serial.println("Stepper driver is communicating but not setup!");
    runTestLoops();
    Serial.println("Running setup again...");
    stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
    runTestLoops();
    Serial.println("Try turning driver power on to see what happens.");
  }
  Serial.println();
  delay(DELAY);
}
