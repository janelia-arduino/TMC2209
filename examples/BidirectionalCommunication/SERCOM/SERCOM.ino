#include <TMC2209.h>
#include <wiring_private.h>

// SERCOM (Serial Communication) is a multiplexed serial configuration used on
// the SAMD21, SAMD51 and other boards.
//
// See this reference for more details:
// https://learn.sparkfun.com/tutorials/adding-more-sercom-ports-for-samd-boards/all#adding-a-uart

// Serial2 on SERCOM4

#define PIN_SERIAL2_RX A3 // RX
#define PIN_SERIAL2_TX A2 // Tx
#define PAD_SERIAL2_RX (SERCOM_RX_PAD_1)
#define PAD_SERIAL2_TX (UART_TX_PAD_0)
const long SERIAL_BAUD_RATE = 115200;

Uart Serial2(&sercom4, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

// Instantiate TMC2209
TMC2209 stepper_driver;
HardwareSerial & serial_stream = Serial2; // this connects the Rx and Tx pins to the libary

void SERCOM4_0_Handler() // required to get sercom4 working
{
  Serial2.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial2.IrqHandler();
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Begin set-up");
  Serial2.begin(115200);
  // Assign pins 12 & 13 SERCOM functionality
  pinPeripheral(PIN_SERIAL2_RX, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SERIAL2_TX, PIO_SERCOM_ALT);

  stepper_driver.setup(serial_stream);
  Serial.println("Set-up completed");
}

void loop()
{

  Serial.println("start loop");

  if (stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver is setup and communicating!");
    Serial.println("Try turning driver power off to see what happens.");
  }
  else if (stepper_driver.isCommunicatingButNotSetup())
  {
    Serial.println("Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }
  Serial.println();
  stepper_driver.getVersion();
  stepper_driver.getSettings();
  delay(2000);
}
