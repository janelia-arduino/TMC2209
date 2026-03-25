#include <TMC2209.h>

HardwareSerial &driver_serial = Serial3;

const long USB_BAUD_RATE = 115200;
const long DRIVER_BAUD_RATE = 115200;
const uint32_t LOOP_DELAY_MS = 3000;
const uint32_t READ_TIMEOUT_MS = 250;

const uint8_t SERIAL_ADDRESS = 0u;
const uint8_t GSTAT_ADDRESS = 0x01u;
const uint8_t IFCNT_ADDRESS = 0x02u;
const uint8_t IOIN_ADDRESS = 0x06u;
const uint8_t CHOPCONF_ADDRESS = 0x6Cu;

TMC2209 stepper_driver;
static bool enable_double_edge = false;
static bool enable_double_edge_verified = true;

static void
print_result_prefix (const char *label)
{
  Serial.print ("[");
  Serial.print (label);
  Serial.print ("] ");
}

static void
print_uart_error (TMC2209::UartError error)
{
  Serial.println (static_cast<unsigned int> (error));
}

static bool
poll_until_done (TMC2209 &driver, uint32_t timeout_ms)
{
  const uint32_t start_ms = millis ();
  while (!driver.done ())
    {
      driver.poll ();
      if ((millis () - start_ms) > timeout_ms)
        {
          return false;
        }
    }
  return true;
}

static bool
poll_until_bus_done (TMC2209::UartBus &bus,
                     uint8_t serial_address,
                     uint32_t timeout_ms)
{
  const uint32_t start_ms = millis ();
  while (!bus.done (serial_address))
    {
      bus.poll ();
      if ((millis () - start_ms) > timeout_ms)
        {
          return false;
        }
    }
  return true;
}

void
setup ()
{
  Serial.begin (USB_BAUD_RATE);
  driver_serial.begin (DRIVER_BAUD_RATE);

  stepper_driver.setup (driver_serial, TMC2209::SERIAL_ADDRESS_0);
  stepper_driver.setReplyDelay (2u);
  stepper_driver.setRunCurrent (20u);
  stepper_driver.setHoldCurrent (10u);
}

void
loop ()
{
  Serial.println ("=== UART Compatibility Smoke ===");

  const uint8_t version = stepper_driver.getVersion ();
  print_result_prefix ("version");
  Serial.println (version, HEX);

  const auto blocking_read = stepper_driver.readRegister (IOIN_ADDRESS);
  print_result_prefix ("blocking read");
  if (blocking_read.ok ())
    {
      Serial.print ("ok value=0x");
      Serial.println (blocking_read.value, HEX);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (blocking_read.error);
    }

  const auto async_start = stepper_driver.startRead (IOIN_ADDRESS);
  print_result_prefix ("facade async start");
  if (!async_start.ok ())
    {
      Serial.print ("error=");
      print_uart_error (async_start.error);
    }
  else if (!poll_until_done (stepper_driver, READ_TIMEOUT_MS))
    {
      Serial.println ("timeout");
    }
  else
    {
      const auto async_result = stepper_driver.takeReadResult ();
      if (async_result.ok ())
        {
          Serial.print ("ok value=0x");
          Serial.println (async_result.value, HEX);
        }
      else
        {
          Serial.print ("error=");
          print_uart_error (async_result.error);
        }
    }

  auto &bus = stepper_driver.uartBus ();
  const auto bus_start = bus.startRead (SERIAL_ADDRESS, IOIN_ADDRESS);
  print_result_prefix ("bus async start");
  if (!bus_start.ok ())
    {
      Serial.print ("error=");
      print_uart_error (bus_start.error);
    }
  else if (!poll_until_bus_done (bus, SERIAL_ADDRESS, READ_TIMEOUT_MS))
    {
      Serial.println ("timeout");
    }
  else
    {
      const auto bus_result = bus.takeReadResult (SERIAL_ADDRESS);
      if (bus_result.ok ())
        {
          Serial.print ("ok value=0x");
          Serial.println (bus_result.value, HEX);
        }
      else
        {
          Serial.print ("error=");
          print_uart_error (bus_result.error);
        }
    }

  const auto health_before = stepper_driver.readHealthStatus ();
  print_result_prefix ("health");
  Serial.print ("comm=");
  Serial.print (health_before.communication_ok);
  Serial.print (" setup=");
  Serial.print (health_before.setup_ok);
  Serial.print (" reset=");
  Serial.print (health_before.reset);
  Serial.print (" drv_err=");
  Serial.print (health_before.driver_error);
  Serial.print (" uv_cp=");
  Serial.print (health_before.charge_pump_undervoltage);
  Serial.print (" mirror=");
  Serial.println (health_before.mirror_resync_required);

  const auto gstat_before_clear = stepper_driver.readRegister (GSTAT_ADDRESS);
  print_result_prefix ("gstat before clear");
  if (gstat_before_clear.ok ())
    {
      Serial.print ("0x");
      Serial.println (gstat_before_clear.value, HEX);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (gstat_before_clear.error);
    }

  stepper_driver.clearReset ();
  stepper_driver.clearDriveError ();

  const auto gstat_after_clear = stepper_driver.readRegister (GSTAT_ADDRESS);
  print_result_prefix ("gstat after clear");
  if (gstat_after_clear.ok ())
    {
      Serial.print ("0x");
      Serial.println (gstat_after_clear.value, HEX);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (gstat_after_clear.error);
    }

  const auto ifcnt_before_write = stepper_driver.readRegister (IFCNT_ADDRESS);
  print_result_prefix ("ifcnt before verified write");
  if (ifcnt_before_write.ok ())
    {
      Serial.println (ifcnt_before_write.value);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (ifcnt_before_write.error);
    }

  const auto chopconf_before = stepper_driver.readRegister (CHOPCONF_ADDRESS);
  print_result_prefix ("chopconf before write");
  if (chopconf_before.ok ())
    {
      Serial.print ("0x");
      Serial.println (chopconf_before.value, HEX);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (chopconf_before.error);
    }

  tmc2209::reg::CHOPCONF chopconf;
  chopconf.raw = chopconf_before.ok () ? chopconf_before.value : 0u;
  chopconf.double_edge (enable_double_edge);
  if (!enable_double_edge)
    {
      chopconf.toff (3u);
    }

  const auto manual_write = stepper_driver.writeRegister (CHOPCONF_ADDRESS,
                                                          chopconf.raw);
  print_result_prefix ("manual write");
  if (manual_write.ok ())
    {
      Serial.println ("ok");
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (manual_write.error);
    }

  const auto ifcnt_after_manual_write = stepper_driver.readRegister (IFCNT_ADDRESS);
  print_result_prefix ("ifcnt after manual write");
  if (ifcnt_after_manual_write.ok ())
    {
      Serial.println (ifcnt_after_manual_write.value);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (ifcnt_after_manual_write.error);
    }

  const auto chopconf_after_manual_write = stepper_driver.readRegister (
      CHOPCONF_ADDRESS);
  print_result_prefix ("chopconf after manual write");
  if (chopconf_after_manual_write.ok ())
    {
      Serial.print ("0x");
      Serial.println (chopconf_after_manual_write.value, HEX);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (chopconf_after_manual_write.error);
    }

  tmc2209::reg::CHOPCONF chopconf_verified = chopconf;
  chopconf_verified.double_edge (enable_double_edge_verified);
  stepper_driver.enableWriteVerification ();
  const auto verified_write = stepper_driver.writeRegister (CHOPCONF_ADDRESS,
                                                            chopconf_verified.raw);
  print_result_prefix ("verified write");
  if (verified_write.ok ())
    {
      Serial.println ("ok");
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (verified_write.error);
    }
  stepper_driver.disableWriteVerification ();

  const auto ifcnt_after_write = stepper_driver.readRegister (IFCNT_ADDRESS);
  print_result_prefix ("ifcnt after verified write");
  if (ifcnt_after_write.ok ())
    {
      Serial.println (ifcnt_after_write.value);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (ifcnt_after_write.error);
    }

  const auto chopconf_after = stepper_driver.readRegister (CHOPCONF_ADDRESS);
  print_result_prefix ("chopconf after write");
  if (chopconf_after.ok ())
    {
      Serial.print ("0x");
      Serial.println (chopconf_after.value, HEX);
    }
  else
    {
      Serial.print ("error=");
      print_uart_error (chopconf_after.error);
    }

  enable_double_edge = !enable_double_edge;
  enable_double_edge_verified = !enable_double_edge_verified;

  stepper_driver.notePossibleMirrorDrift ();
  const bool recovered = stepper_driver.recoverIfNeeded ();
  print_result_prefix ("recoverIfNeeded");
  Serial.println (recovered);

  const bool resynced = stepper_driver.resyncReadableConfiguration ();
  print_result_prefix ("resyncReadableConfiguration");
  Serial.println (resynced);

  print_result_prefix ("last uart error");
  print_uart_error (stepper_driver.getLastUartError ());

  Serial.println ();
  delay (LOOP_DELAY_MS);
}
