#include <unity.h>

#define private public
#include <TMC2209.h>
#undef private
#include <TMC2209/Protocol.hpp>
#include <tmc_bits.hpp>
#include <tmc2209_registers.hpp>

#include "FakeSerial.hpp"

// Unity hooks (optional on most platforms, but harmless)
void
setUp (void)
{
  arduino_test::reset_time ();
}
void
tearDown (void)
{
}

static void
poll_until_result_ready (TMC2209 &tmc, uint32_t timeout_us = 250000u)
{
  const uint32_t start = micros ();
  while (!tmc.resultReady ())
    {
      tmc.poll ();
      if (!tmc.resultReady ())
        {
          delayMicroseconds (1);
        }

      if ((micros () - start) > timeout_us)
        {
          TEST_FAIL_MESSAGE ("timed out waiting for UART engine result");
        }
    }
}

static void
poll_until_bus_result_ready (tmc2209::UartBus &bus,
                             uint32_t timeout_us = 250000u)
{
  const uint32_t start = micros ();
  while (!bus.resultReady ())
    {
      bus.poll ();
      if (!bus.resultReady ())
        {
          delayMicroseconds (1);
        }

      if ((micros () - start) > timeout_us)
        {
          TEST_FAIL_MESSAGE ("timed out waiting for shared UART bus result");
        }
    }
}

void
test_microsteps_powers_of_two_map_exactly ()
{
  TMC2209 tmc;

  const uint16_t cases[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
  for (uint16_t v : cases)
    {
      tmc.setMicrostepsPerStep (v);
      TEST_ASSERT_EQUAL_UINT16_MESSAGE (v, tmc.getMicrostepsPerStep (), "microsteps power-of-two mapping");
    }
}

void
test_microsteps_clamps_out_of_range_inputs ()
{
  TMC2209 tmc;

  tmc.setMicrostepsPerStep (0);
  TEST_ASSERT_EQUAL_UINT16 (1, tmc.getMicrostepsPerStep ());

  tmc.setMicrostepsPerStep (999);
  TEST_ASSERT_EQUAL_UINT16 (256, tmc.getMicrostepsPerStep ());
}

void
test_read_retries_after_reply_timeout ()
{
  FakeSerial serial;

  // Configure the virtual device: IOIN (0x06) contains VERSION in the top byte.
  // VERSION constant in the library is 0x21.
  const uint32_t ioin_value = 0x21000000u;
  serial.set_register_value (0x06, ioin_value);

  // First read request gets no reply; second read request gets a reply.
  serial.reply_after_attempt (2);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);

  // Ignore any traffic during setup.
  serial.reset ();

  const uint8_t version = tmc.getVersion ();
  TEST_ASSERT_EQUAL_UINT8_MESSAGE (0x21, version, "expected VERSION after retry");

  // The retry bug fix should cause exactly 2 read requests in this scenario.
  TEST_ASSERT_EQUAL_UINT_MESSAGE (2, serial.read_request_count (), "expected one retry after timeout");
}

void
test_readRegister_reports_timeout_error_when_no_reply ()
{
  FakeSerial serial;

  // Never reply to read requests.
  serial.reply_after_attempt (999);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  const auto res = tmc.readRegister (0x06);
  TEST_ASSERT_FALSE_MESSAGE (res.ok (), "expected readRegister to fail");
  TEST_ASSERT_EQUAL_UINT8_MESSAGE (static_cast<uint8_t> (TMC2209::UartError::ReplyTimeout),
                                   static_cast<uint8_t> (res.error),
                                   "expected ReplyTimeout error");

  TEST_ASSERT_EQUAL_UINT8_MESSAGE (static_cast<uint8_t> (TMC2209::UartError::ReplyTimeout),
                                   static_cast<uint8_t> (tmc.getLastUartError ()),
                                   "expected getLastUartError() to return ReplyTimeout");

  // Default MAX_READ_RETRIES is 5 in this library.
  TEST_ASSERT_EQUAL_UINT_MESSAGE (5, serial.read_request_count (), "expected retries on timeout");
}

void
test_readRegister_retries_after_crc_mismatch ()
{
  FakeSerial serial;

  const uint32_t ioin_value = 0x21000000u;
  serial.set_register_value (0x06, ioin_value);

  // Corrupt the first reply CRC, then send a valid reply on retry.
  serial.corrupt_crc_for_first_replies (1);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  const auto res = tmc.readRegister (0x06);
  TEST_ASSERT_TRUE_MESSAGE (res.ok (), "expected readRegister to succeed after CRC retry");
  TEST_ASSERT_EQUAL_HEX32_MESSAGE (ioin_value, res.value, "expected register value");

  TEST_ASSERT_EQUAL_UINT_MESSAGE (2, serial.read_request_count (), "expected one retry after CRC mismatch");

  TEST_ASSERT_EQUAL_UINT8_MESSAGE (static_cast<uint8_t> (TMC2209::UartError::None),
                                   static_cast<uint8_t> (tmc.getLastUartError ()),
                                   "expected getLastUartError() to be cleared on success");
}

void
test_nonblocking_startRead_poll_takeReadResult_succeeds ()
{
  FakeSerial serial;
  const uint32_t ioin_value = 0x21000000u;
  serial.set_register_value (0x06, ioin_value);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  const auto start_result = tmc.startRead (0x06);
  TEST_ASSERT_TRUE_MESSAGE (start_result.ok (), "expected startRead to succeed");
  TEST_ASSERT_TRUE_MESSAGE (tmc.busy (), "expected UART engine to be busy after startRead");

  poll_until_result_ready (tmc);

  const auto result = tmc.takeReadResult ();
  TEST_ASSERT_TRUE_MESSAGE (result.ok (), "expected takeReadResult to succeed");
  TEST_ASSERT_EQUAL_HEX32 (ioin_value, result.value);
  TEST_ASSERT_FALSE_MESSAGE (tmc.busy (), "expected UART engine to be idle after takeReadResult");
}

void
test_nonblocking_startWrite_poll_takeWriteResult_succeeds ()
{
  FakeSerial serial;

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  const auto start_result = tmc.startWrite (0x10u, 0x12345678u);
  TEST_ASSERT_TRUE_MESSAGE (start_result.ok (), "expected startWrite to succeed");

  poll_until_result_ready (tmc, 1000u);

  const auto result = tmc.takeWriteResult ();
  TEST_ASSERT_TRUE_MESSAGE (result.ok (), "expected takeWriteResult to succeed");

  const auto expected = tmc2209::protocol::WriteReadReplyDatagram::makeWrite (
      0x00u, 0x10u, 0x12345678u);
  TEST_ASSERT_EQUAL_UINT (tmc2209::protocol::WriteReadReplyDatagram::kSize,
                          serial.tx_bytes ().size ());
  for (size_t i = 0; i < serial.tx_bytes ().size (); ++i)
    {
      TEST_ASSERT_EQUAL_UINT8_MESSAGE (expected.bytes[i], serial.tx_bytes ()[i],
                                       "unexpected transmitted write byte");
    }
}

void
test_nonblocking_startRead_returns_busy_while_transaction_active ()
{
  FakeSerial serial;
  serial.set_register_value (0x06, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_TRUE (tmc.startRead (0x06).ok ());

  const auto busy_result = tmc.startWrite (0x10u, 0x12345678u);
  TEST_ASSERT_FALSE_MESSAGE (busy_result.ok (), "expected concurrent transaction start to fail");
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (TMC2209::UartError::Busy),
                           static_cast<uint8_t> (busy_result.error));
}

void
test_nonblocking_read_drains_stale_rx_bytes_before_transaction ()
{
  FakeSerial serial;
  serial.set_register_value (0x06, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  serial.queue_rx_bytes ({ 0xAAu, 0xBBu, 0xCCu });

  TEST_ASSERT_TRUE (tmc.startRead (0x06).ok ());
  poll_until_result_ready (tmc);

  const auto result = tmc.takeReadResult ();
  TEST_ASSERT_TRUE_MESSAGE (result.ok (), "expected stale RX bytes to be drained before read");
  TEST_ASSERT_EQUAL_HEX32 (0x21000000u, result.value);
  TEST_ASSERT_EQUAL_UINT_MESSAGE (1u, serial.read_request_count (),
                                  "expected a single successful request after draining RX garbage");
}

void
test_nonblocking_read_reports_echo_corruption_after_retries ()
{
  FakeSerial serial;
  serial.set_register_value (0x06, 0x21000000u);
  serial.corrupt_echo_for_first_read_requests (5);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_TRUE (tmc.startRead (0x06).ok ());
  poll_until_result_ready (tmc, 150000u);

  const auto result = tmc.takeReadResult ();
  TEST_ASSERT_FALSE_MESSAGE (result.ok (), "expected read to fail when every echo is corrupted");
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (TMC2209::UartError::RxGarbage),
                           static_cast<uint8_t> (result.error));
  TEST_ASSERT_EQUAL_UINT_MESSAGE (5u, serial.read_request_count (),
                                  "expected retries until the read attempt budget is exhausted");
}

void
test_protocol_read_request_pack_is_explicit_and_crc_valid ()
{
  const auto datagram
      = tmc2209::protocol::ReadRequestDatagram::make (0x02u, 0x6Cu);

  TEST_ASSERT_EQUAL_UINT8 (0x05u, datagram.bytes[0]);
  TEST_ASSERT_EQUAL_UINT8 (0x02u, datagram.bytes[1]);
  TEST_ASSERT_EQUAL_UINT8 (0x6Cu, datagram.bytes[2]);
  TEST_ASSERT_EQUAL_UINT8 (tmc2209::protocol::SYNC, datagram.sync ());
  TEST_ASSERT_EQUAL_UINT8 (0x02u, datagram.serialAddress ());
  TEST_ASSERT_EQUAL_UINT8 (0x6Cu, datagram.registerAddress ());
  TEST_ASSERT_EQUAL_UINT8 (tmc2209::protocol::RW_READ, datagram.rw ());
  TEST_ASSERT_TRUE (datagram.hasValidCrc ());
}

void
test_protocol_write_datagram_pack_is_explicit_and_big_endian ()
{
  const auto datagram = tmc2209::protocol::WriteReadReplyDatagram::makeWrite (
      0x03u, 0x6Cu, 0x12345678u);

  TEST_ASSERT_EQUAL_UINT8 (0x05u, datagram.bytes[0]);
  TEST_ASSERT_EQUAL_UINT8 (0x03u, datagram.bytes[1]);
  TEST_ASSERT_EQUAL_UINT8 (0xECu, datagram.bytes[2]);
  TEST_ASSERT_EQUAL_UINT8 (0x12u, datagram.bytes[3]);
  TEST_ASSERT_EQUAL_UINT8 (0x34u, datagram.bytes[4]);
  TEST_ASSERT_EQUAL_UINT8 (0x56u, datagram.bytes[5]);
  TEST_ASSERT_EQUAL_UINT8 (0x78u, datagram.bytes[6]);
  TEST_ASSERT_EQUAL_UINT8 (tmc2209::protocol::RW_WRITE, datagram.rw ());
  TEST_ASSERT_EQUAL_HEX32 (0x12345678u, datagram.data ());
  TEST_ASSERT_TRUE (datagram.hasValidCrc ());
}

void
test_protocol_crc_detects_corruption ()
{
  auto datagram = tmc2209::protocol::WriteReadReplyDatagram::makeReadReply (
      0x06u, 0x21000000u);

  TEST_ASSERT_TRUE (datagram.hasValidCrc ());
  datagram.bytes[tmc2209::protocol::WriteReadReplyDatagram::kSize - 1] ^= 0x01u;
  TEST_ASSERT_FALSE (datagram.hasValidCrc ());
}

void
test_setRMSCurrent_clamps_very_low_requests_to_zero ()
{
  TMC2209 tmc;

  tmc.setRMSCurrent (0, 0.11f, 0.5f);

  TEST_ASSERT_EQUAL_UINT32 (0u, tmc.ihold_irun_.irun ());
  TEST_ASSERT_EQUAL_UINT32 (0u, tmc.ihold_irun_.ihold ());
}

void
test_tmc_bits_bit_get_set ()
{
  uint32_t reg = 0;
  using B = tmc::bits::Bit<3>;

  TEST_ASSERT_FALSE (B::get (reg));

  B::set (reg, true);
  TEST_ASSERT_TRUE (B::get (reg));
  TEST_ASSERT_EQUAL_HEX32 (0x00000008u, reg);

  B::set (reg, false);
  TEST_ASSERT_FALSE (B::get (reg));
  TEST_ASSERT_EQUAL_HEX32 (0x00000000u, reg);
}

void
test_tmc_bits_field_get_set_and_masks ()
{
  uint32_t reg = 0;
  using F = tmc::bits::Field<8, 4>;

  F::set (reg, 0xFu);
  TEST_ASSERT_EQUAL_HEX32 (0x00000F00u, reg);
  TEST_ASSERT_EQUAL_UINT32 (0xFu, F::get (reg));

  // Value is masked to field width (4 bits) so 0xAB becomes 0xB.
  F::set (reg, 0xABu);
  TEST_ASSERT_EQUAL_HEX32 (0x00000B00u, reg);
  TEST_ASSERT_EQUAL_UINT32 (0xBu, F::get (reg));
}

void
test_tmc_bits_field_does_not_clobber_other_bits ()
{
  uint32_t reg = 0xA5A5A5A5u;
  using F = tmc::bits::Field<8, 4>;

  const uint32_t expected = reg & ~uint32_t (0x00000F00u);

  F::set (reg, 0);
  TEST_ASSERT_EQUAL_HEX32 (expected, reg);
}

void
test_reg_gconf_encodes_expected_bits ()
{
  tmc2209::reg::GCONF g;

  g.pdn_disable (true)
      .mstep_reg_select (true)
      .multistep_filt (true)
      .shaft (true);

  // Bits: PDN_DISABLE(6) + MSTEP_REG_SELECT(7) + MULTISTEP_FILT(8) + SHAFT(3)
  TEST_ASSERT_EQUAL_HEX32 (0x000001C8u, g.raw);

  TEST_ASSERT_TRUE (g.pdn_disable ());
  TEST_ASSERT_TRUE (g.mstep_reg_select ());
  TEST_ASSERT_TRUE (g.multistep_filt ());
  TEST_ASSERT_TRUE (g.shaft ());
  TEST_ASSERT_FALSE (g.enable_spread_cycle ());
}

void
test_reg_chopconf_encodes_expected_fields ()
{
  tmc2209::reg::CHOPCONF c;

  c.toff (3)
      .hstart (5)
      .hend (10)
      .tbl (2)
      .vsense (true)
      .mres (tmc2209::reg::Mres::M16)
      .interpolation (true)
      .double_edge (true)
      .diss2g (false)
      .diss2vs (false);

  TEST_ASSERT_EQUAL_HEX32 (0x34030553u, c.raw);

  TEST_ASSERT_EQUAL_UINT32 (3u, c.toff ());
  TEST_ASSERT_EQUAL_UINT32 (5u, c.hstart ());
  TEST_ASSERT_EQUAL_UINT32 (10u, c.hend ());
  TEST_ASSERT_EQUAL_UINT32 (2u, c.tbl ());
  TEST_ASSERT_TRUE (c.vsense ());
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::reg::Mres::M16),
                           static_cast<uint8_t> (c.mres ()));
  TEST_ASSERT_TRUE (c.interpolation ());
  TEST_ASSERT_TRUE (c.double_edge ());
}

void
test_reg_pwmconf_encodes_expected_fields ()
{
  tmc2209::reg::PWMCONF p;

  p.pwm_offset (0x24)
      .pwm_grad (0x14)
      .pwm_freq (2)
      .pwm_autoscale (true)
      .pwm_autograd (false)
      .freewheel (1)
      .pwm_reg (1)
      .pwm_lim (12);

  TEST_ASSERT_EQUAL_HEX32 (0xC1161424u, p.raw);

  TEST_ASSERT_EQUAL_UINT32 (0x24u, p.pwm_offset ());
  TEST_ASSERT_EQUAL_UINT32 (0x14u, p.pwm_grad ());
  TEST_ASSERT_EQUAL_UINT32 (2u, p.pwm_freq ());
  TEST_ASSERT_TRUE (p.pwm_autoscale ());
  TEST_ASSERT_FALSE (p.pwm_autograd ());
  TEST_ASSERT_EQUAL_UINT32 (1u, p.freewheel ());
  TEST_ASSERT_EQUAL_UINT32 (1u, p.pwm_reg ());
  TEST_ASSERT_EQUAL_UINT32 (12u, p.pwm_lim ());
}

void
test_reg_ihold_irun_encodes_expected_fields ()
{
  tmc2209::reg::IHOLD_IRUN r;

  r.ihold (16).irun (31).iholddelay (1);

  TEST_ASSERT_EQUAL_HEX32 (0x00011F10u, r.raw);
  TEST_ASSERT_EQUAL_UINT32 (16u, r.ihold ());
  TEST_ASSERT_EQUAL_UINT32 (31u, r.irun ());
  TEST_ASSERT_EQUAL_UINT32 (1u, r.iholddelay ());
}

void
test_reg_coolconf_encodes_expected_fields ()
{
  tmc2209::reg::COOLCONF c;

  c.semin (1).seup (2).semax (15).sedn (3).seimin (true);

  TEST_ASSERT_EQUAL_HEX32 (0x0000EF41u, c.raw);
  TEST_ASSERT_EQUAL_UINT32 (1u, c.semin ());
  TEST_ASSERT_EQUAL_UINT32 (2u, c.seup ());
  TEST_ASSERT_EQUAL_UINT32 (15u, c.semax ());
  TEST_ASSERT_EQUAL_UINT32 (3u, c.sedn ());
  TEST_ASSERT_TRUE (c.seimin ());
}



void
test_reg_gstat_encodes_expected_bits ()
{
  tmc2209::reg::GSTAT g;

  g.reset (true).drv_err (true).uv_cp (false);

  TEST_ASSERT_EQUAL_HEX32 (0x00000003u, g.raw);
  TEST_ASSERT_TRUE (g.reset ());
  TEST_ASSERT_TRUE (g.drv_err ());
  TEST_ASSERT_FALSE (g.uv_cp ());
}

void
test_reg_replydelay_encodes_expected_field ()
{
  tmc2209::reg::REPLYDELAY r;

  r.replydelay (0xFu);
  TEST_ASSERT_EQUAL_HEX32 (0x00000F00u, r.raw);
  TEST_ASSERT_EQUAL_UINT32 (0xFu, r.replydelay ());

  // Field is 4 bits wide; value is masked.
  r.raw = 0;
  r.replydelay (0xABu);
  TEST_ASSERT_EQUAL_HEX32 (0x00000B00u, r.raw);
  TEST_ASSERT_EQUAL_UINT32 (0xBu, r.replydelay ());
}

void
test_reg_ioin_encodes_expected_fields ()
{
  tmc2209::reg::IOIN ioin;
  ioin.raw = 0;

  ioin.enn (true)
      .ms1 (true)
      .ms2 (false)
      .diag (true)
      .pdn_serial (true)
      .step (false)
      .spread_en (true)
      .dir (true)
      .version (0x21);

  TEST_ASSERT_EQUAL_HEX32 (0x21000355u, ioin.raw);
  TEST_ASSERT_TRUE (ioin.enn ());
  TEST_ASSERT_TRUE (ioin.ms1 ());
  TEST_ASSERT_FALSE (ioin.ms2 ());
  TEST_ASSERT_TRUE (ioin.diag ());
  TEST_ASSERT_TRUE (ioin.pdn_serial ());
  TEST_ASSERT_FALSE (ioin.step ());
  TEST_ASSERT_TRUE (ioin.spread_en ());
  TEST_ASSERT_TRUE (ioin.dir ());
  TEST_ASSERT_EQUAL_UINT32 (0x21u, ioin.version ());
}

void
test_reg_drv_status_encodes_expected_fields ()
{
  tmc2209::reg::DRV_STATUS s;
  s.raw = 0;

  s.over_temperature_warning (true)
      .short_to_ground_a (true)
      .open_load_b (true)
      .over_temperature_150c (true)
      .current_scaling (31)
      .stealth_chop_mode (true)
      .standstill (true);

  TEST_ASSERT_EQUAL_HEX32 (0xC01F0485u, s.raw);

  TEST_ASSERT_TRUE (s.over_temperature_warning ());
  TEST_ASSERT_FALSE (s.over_temperature_shutdown ());
  TEST_ASSERT_TRUE (s.short_to_ground_a ());
  TEST_ASSERT_TRUE (s.open_load_b ());
  TEST_ASSERT_TRUE (s.over_temperature_150c ());
  TEST_ASSERT_EQUAL_UINT32 (31u, s.current_scaling ());
  TEST_ASSERT_TRUE (s.stealth_chop_mode ());
  TEST_ASSERT_TRUE (s.standstill ());
}

void
test_reg_pwm_scale_encodes_expected_fields ()
{
  tmc2209::reg::PWM_SCALE p;

  p.pwm_scale_sum (0xAA).pwm_scale_auto (0x1FF);

  TEST_ASSERT_EQUAL_HEX32 (0x01FF00AAu, p.raw);
  TEST_ASSERT_EQUAL_UINT32 (0xAAu, p.pwm_scale_sum ());
  TEST_ASSERT_EQUAL_UINT32 (0x1FFu, p.pwm_scale_auto ());
  TEST_ASSERT_EQUAL_INT16 (-1, p.pwm_scale_auto_signed ());
}

void
test_getPwmScaleAuto_decodes_signed_pwm_scale_auto_field ()
{
  FakeSerial serial;

  serial.set_register_value (0x71, 0x01FF0000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_EQUAL_INT16 (-1, tmc.getPwmScaleAuto ());
}

void
test_reg_pwm_auto_encodes_expected_fields ()
{
  tmc2209::reg::PWM_AUTO p;

  p.pwm_offset_auto (0x24).pwm_gradient_auto (0x14);

  TEST_ASSERT_EQUAL_HEX32 (0x00140024u, p.raw);
  TEST_ASSERT_EQUAL_UINT32 (0x24u, p.pwm_offset_auto ());
  TEST_ASSERT_EQUAL_UINT32 (0x14u, p.pwm_gradient_auto ());
}



void
test_uartbus_devices_read_address_specific_register_values ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);
  serial.set_register_value (0x01u, 0x06u, 0x11000000u);

  tmc2209::UartBus bus;
  bus.setup (serial);
  tmc2209::Device dev0 (bus, 0x00u);
  tmc2209::Device dev1 (bus, 0x01u);

  const auto res0 = dev0.readRegister (0x06u);
  const auto res1 = dev1.readRegister (0x06u);

  TEST_ASSERT_TRUE (res0.ok ());
  TEST_ASSERT_TRUE (res1.ok ());
  TEST_ASSERT_EQUAL_HEX32 (0x21000000u, res0.value);
  TEST_ASSERT_EQUAL_HEX32 (0x11000000u, res1.value);

  const auto req0 = tmc2209::protocol::ReadRequestDatagram::make (0x00u, 0x06u);
  const auto req1 = tmc2209::protocol::ReadRequestDatagram::make (0x01u, 0x06u);

  TEST_ASSERT_EQUAL_UINT (8u, serial.tx_bytes ().size ());
  for (size_t i = 0; i < 4u; ++i)
    {
      TEST_ASSERT_EQUAL_UINT8 (req0.bytes[i], serial.tx_bytes ()[i]);
      TEST_ASSERT_EQUAL_UINT8 (req1.bytes[i], serial.tx_bytes ()[i + 4u]);
    }
}

void
test_uartbus_shared_bus_scopes_results_to_the_started_device ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  tmc2209::UartBus bus;
  bus.setup (serial);
  tmc2209::Device dev0 (bus, 0x00u);
  tmc2209::Device dev1 (bus, 0x01u);

  TEST_ASSERT_TRUE (dev0.startRead (0x06u).ok ());

  const auto busy_result = dev1.startRead (0x06u);
  TEST_ASSERT_FALSE (busy_result.ok ());
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::UartError::Busy),
                           static_cast<uint8_t> (busy_result.error));

  poll_until_bus_result_ready (bus);

  TEST_ASSERT_TRUE (dev0.resultReady ());
  TEST_ASSERT_FALSE (dev1.resultReady ());

  const auto wrong_result = dev1.takeReadResult ();
  TEST_ASSERT_FALSE (wrong_result.ok ());
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::UartError::Busy),
                           static_cast<uint8_t> (wrong_result.error));

  const auto good_result = dev0.takeReadResult ();
  TEST_ASSERT_TRUE (good_result.ok ());
  TEST_ASSERT_EQUAL_HEX32 (0x21000000u, good_result.value);
}

void
test_registers_typed_helpers_read_ioin_version ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  tmc2209::UartBus bus;
  bus.setup (serial);
  tmc2209::Device device (bus, 0x00u);
  tmc2209::Registers registers (device);

  const auto ioin = registers.readIoin ();
  TEST_ASSERT_TRUE (ioin.ok ());
  TEST_ASSERT_EQUAL_UINT8 (0x21u,
                           static_cast<uint8_t> (ioin.value.version ()));
}

void
test_device_write_verification_succeeds_when_ifcnt_increments ()
{
  FakeSerial serial;

  tmc2209::UartBus bus;
  bus.setup (serial);

  tmc2209::UartParameters parameters;
  parameters.serial_address = 0x00u;
  parameters.verify_writes = true;
  tmc2209::Device device (bus, parameters);

  const auto result = device.writeRegister (0x10u, 0x12345678u);
  TEST_ASSERT_TRUE (result.ok ());
  TEST_ASSERT_EQUAL_UINT8 (1u, static_cast<uint8_t> (
                                   serial.register_value (0x00u,
                                                          TMC2209::ADDRESS_IFCNT)));
}

void
test_device_write_verification_reports_ifcnt_mismatch ()
{
  FakeSerial serial;
  serial.suppress_ifcnt_increment_for_first_writes (1u);

  tmc2209::UartBus bus;
  bus.setup (serial);

  tmc2209::UartParameters parameters;
  parameters.serial_address = 0x00u;
  parameters.verify_writes = true;
  tmc2209::Device device (bus, parameters);

  const auto result = device.writeRegister (0x10u, 0x12345678u);
  TEST_ASSERT_FALSE (result.ok ());
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::UartError::WriteVerifyFailed),
                           static_cast<uint8_t> (result.error));
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::UartError::WriteVerifyFailed),
                           static_cast<uint8_t> (device.lastError ()));
}

void
test_driver_initialize_configures_serial_mode_defaults ()
{
  FakeSerial serial;

  tmc2209::UartBus bus;
  bus.setup (serial);
  tmc2209::Device device (bus, 0x00u);
  tmc2209::Registers registers (device);
  tmc2209::Driver driver (device, registers);

  const auto init_result = driver.initialize ();
  TEST_ASSERT_TRUE (init_result.ok ());

  const auto gconf = registers.readGconf ();
  TEST_ASSERT_TRUE (gconf.ok ());
  TEST_ASSERT_TRUE (gconf.value.pdn_disable ());
  TEST_ASSERT_TRUE (gconf.value.mstep_reg_select ());
  TEST_ASSERT_TRUE (gconf.value.multistep_filt ());

  const auto ihold_irun = registers.readIholdIrun ();
  TEST_ASSERT_TRUE (ihold_irun.ok ());
  TEST_ASSERT_EQUAL_UINT32 (0u, ihold_irun.value.irun ());
  TEST_ASSERT_EQUAL_UINT32 (0u, ihold_irun.value.ihold ());

  const auto chopconf = registers.readChopconf ();
  TEST_ASSERT_TRUE (chopconf.ok ());
  TEST_ASSERT_EQUAL_UINT32 (0u, chopconf.value.toff ());

  const auto pwmconf = registers.readPwmconf ();
  TEST_ASSERT_TRUE (pwmconf.ok ());
  TEST_ASSERT_FALSE (pwmconf.value.pwm_autoscale ());
  TEST_ASSERT_FALSE (pwmconf.value.pwm_autograd ());
}

void
test_facade_exposes_driver_and_registers_on_internal_bus ()
{
  FakeSerial serial;
  serial.set_register_value (0x01u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_1);
  serial.reset ();

  const auto ioin = tmc.registers.readIoin ();
  TEST_ASSERT_TRUE (ioin.ok ());
  TEST_ASSERT_EQUAL_UINT8 (0x21u,
                           static_cast<uint8_t> (ioin.value.version ()));

  const auto expected_read = tmc2209::protocol::ReadRequestDatagram::make (
      0x01u, 0x06u);
  TEST_ASSERT_EQUAL_UINT (4u, serial.tx_bytes ().size ());
  for (size_t i = 0; i < 4u; ++i)
    {
      TEST_ASSERT_EQUAL_UINT8 (expected_read.bytes[i], serial.tx_bytes ()[i]);
    }

  serial.reset ();
  TEST_ASSERT_TRUE (tmc.driver.disable ().ok ());
  const auto chopconf = tmc.registers.readChopconf ();
  TEST_ASSERT_TRUE (chopconf.ok ());
  TEST_ASSERT_EQUAL_UINT32 (0u, chopconf.value.toff ());
}

void
test_driver_microsteps_follows_legacy_power_of_two_flooring ()
{
  FakeSerial serial;

  tmc2209::UartBus bus;
  bus.setup (serial);
  tmc2209::Device device (bus, 0x00u);
  tmc2209::Registers registers (device);
  tmc2209::Driver driver (device, registers);

  TEST_ASSERT_TRUE (driver.initialize ().ok ());

  TEST_ASSERT_TRUE (driver.setMicrostepsPerStep (5u).ok ());
  auto microsteps = driver.getMicrostepsPerStep ();
  TEST_ASSERT_TRUE (microsteps.ok ());
  TEST_ASSERT_EQUAL_UINT16 (4u, microsteps.value);

  TEST_ASSERT_TRUE (driver.setMicrostepsPerStep (999u).ok ());
  microsteps = driver.getMicrostepsPerStep ();
  TEST_ASSERT_TRUE (microsteps.ok ());
  TEST_ASSERT_EQUAL_UINT16 (256u, microsteps.value);
}



void
test_facade_nonblocking_and_subobjects_share_bus_state ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_TRUE (tmc.startRead (0x06u).ok ());

  const auto ioin = tmc.registers.readIoin ();
  TEST_ASSERT_FALSE_MESSAGE (
      ioin.ok (),
      "expected facade and subobjects to share one in-flight UART transaction");
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::UartError::Busy),
                           static_cast<uint8_t> (ioin.error));
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (tmc2209::UartError::Busy),
                           static_cast<uint8_t> (tmc.getLastUartError ()));

  poll_until_result_ready (tmc);

  const auto result = tmc.takeReadResult ();
  TEST_ASSERT_TRUE (result.ok ());
  TEST_ASSERT_EQUAL_HEX32 (0x21000000u, result.value);
}

void
test_facade_getSettings_tracks_subobject_register_changes ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_TRUE (tmc.driver.setRunCurrent (50u).ok ());
  TEST_ASSERT_TRUE (tmc.driver.setHoldCurrent (25u).ok ());

  tmc2209::reg::COOLCONF coolconf;
  coolconf.raw = 0u;
  coolconf.semin (3u).semax (1u);
  TEST_ASSERT_TRUE (tmc.registers.writeCoolconf (coolconf).ok ());

  const auto ihold_irun = tmc.registers.readIholdIrun ();
  TEST_ASSERT_TRUE (ihold_irun.ok ());

  const auto settings = tmc.getSettings ();
  TEST_ASSERT_TRUE (settings.is_communicating);
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (ihold_irun.value.irun ()),
                           settings.irun_register_value);
  TEST_ASSERT_EQUAL_UINT8 (static_cast<uint8_t> (ihold_irun.value.ihold ()),
                           settings.ihold_register_value);
  TEST_ASSERT_TRUE (settings.cool_step_enabled);
}

void
test_facade_getMicrostepsPerStep_tracks_driver_changes_when_configured ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_TRUE (tmc.driver.setMicrostepsPerStep (64u).ok ());
  TEST_ASSERT_EQUAL_UINT16 (64u, tmc.getMicrostepsPerStep ());
}

void
test_facade_enableWriteVerification_uses_ifcnt_verification ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_FALSE (tmc.writeVerificationEnabled ());
  tmc.enableWriteVerification ();
  TEST_ASSERT_TRUE (tmc.writeVerificationEnabled ());

  const uint8_t ifcnt_before = static_cast<uint8_t> (
      serial.register_value (0x00u, TMC2209::ADDRESS_IFCNT));
  const auto result = tmc.writeRegister (0x10u, 0x12345678u);
  TEST_ASSERT_TRUE (result.ok ());
  TEST_ASSERT_EQUAL_UINT8 (
      static_cast<uint8_t> (ifcnt_before + 1u),
      static_cast<uint8_t> (serial.register_value (0x00u,
                                                   TMC2209::ADDRESS_IFCNT)));
}

void
test_family_style_done_aliases_follow_result_ready_state ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  TEST_ASSERT_TRUE (tmc.startRead (0x06u).ok ());
  TEST_ASSERT_FALSE (tmc.done ());
  TEST_ASSERT_FALSE (tmc.uartBus ().done ());
  TEST_ASSERT_FALSE (tmc.device ().done ());

  poll_until_result_ready (tmc);

  TEST_ASSERT_TRUE (tmc.done ());
  TEST_ASSERT_TRUE (tmc.uartBus ().done ());
  TEST_ASSERT_TRUE (tmc.device ().done ());

  const auto result = tmc.takeReadResult ();
  TEST_ASSERT_TRUE (result.ok ());
  TEST_ASSERT_EQUAL_HEX32 (0x21000000u, result.value);
}

void
test_recoverFromDeviceReset_replays_cached_configuration ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  tmc.setReplyDelay (3u);
  tmc.setPowerDownDelay (7u);
  tmc.setStealthChopDurationThreshold (0x1234u);
  tmc.setCoolStepDurationThreshold (0x2345u);
  tmc.setStallGuardThreshold (0x17u);
  tmc.setMicrostepsPerStep (64u);
  tmc.setRunCurrent (61u);

  tmc2209::reg::GSTAT gstat;
  gstat.raw = 0u;
  gstat.reset (true);

  serial.set_register_value (0x00u, TMC2209::ADDRESS_GSTAT, gstat.raw);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_GCONF, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_CHOPCONF,
                             TMC2209::CHOPPER_CONFIG_DEFAULT);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_PWMCONF,
                             TMC2209::PWM_CONFIG_DEFAULT);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_IHOLD_IRUN, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_COOLCONF, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_REPLYDELAY, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_TPOWERDOWN,
                             TMC2209::TPOWERDOWN_DEFAULT);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_TPWMTHRS, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_VACTUAL, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_TCOOLTHRS, 0u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_SGTHRS, 0u);

  TEST_ASSERT_TRUE (tmc.recoverFromDeviceReset ());
  TEST_ASSERT_FALSE (tmc.mirrorResyncRequired ());

  TEST_ASSERT_EQUAL_HEX32 (tmc.gconf_.raw,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_GCONF));
  TEST_ASSERT_EQUAL_HEX32 (tmc.ihold_irun_.raw,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_IHOLD_IRUN));
  TEST_ASSERT_EQUAL_HEX32 (tmc.chopconf_.raw,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_CHOPCONF));
  TEST_ASSERT_EQUAL_HEX32 (tmc.pwmconf_.raw,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_PWMCONF));
  TEST_ASSERT_EQUAL_HEX32 (tmc.coolconf_.raw,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_COOLCONF));
  TEST_ASSERT_EQUAL_HEX32 (tmc.reply_delay_raw_,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_REPLYDELAY));
  TEST_ASSERT_EQUAL_HEX32 (tmc.tpowerdown_raw_,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_TPOWERDOWN));
  TEST_ASSERT_EQUAL_HEX32 (tmc.tpwmthrs_raw_,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_TPWMTHRS));
  TEST_ASSERT_EQUAL_HEX32 (tmc.vactual_raw_,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_VACTUAL));
  TEST_ASSERT_EQUAL_HEX32 (tmc.tcoolthrs_raw_,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_TCOOLTHRS));
  TEST_ASSERT_EQUAL_HEX32 (tmc.sgthrs_raw_,
                           serial.register_value (0x00u,
                                                  TMC2209::ADDRESS_SGTHRS));
}

void
test_resyncReadableConfiguration_refreshes_cached_state_from_device ()
{
  FakeSerial serial;
  serial.set_register_value (0x00u, 0x06u, 0x21000000u);

  TMC2209 tmc;
  tmc.setup (serial, TMC2209::SERIAL_ADDRESS_0);
  serial.reset ();

  tmc2209::reg::GCONF gconf = tmc.gconf_;
  gconf.shaft (true);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_GCONF, gconf.raw);

  tmc2209::reg::IHOLD_IRUN ihold_irun = tmc.ihold_irun_;
  ihold_irun.irun (9u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_IHOLD_IRUN,
                             ihold_irun.raw);

  tmc2209::reg::CHOPCONF chopconf = tmc.chopconf_;
  chopconf.mres (tmc2209::reg::Mres::M32);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_CHOPCONF, chopconf.raw);

  serial.set_register_value (0x00u, TMC2209::ADDRESS_REPLYDELAY, 5u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_TPOWERDOWN, 11u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_TPWMTHRS, 0x3456u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_VACTUAL, 0x4567u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_TCOOLTHRS, 0x5678u);
  serial.set_register_value (0x00u, TMC2209::ADDRESS_SGTHRS, 0x19u);

  tmc.notePossibleMirrorDrift ();
  TEST_ASSERT_TRUE (tmc.resyncReadableConfiguration ());

  TEST_ASSERT_FALSE (tmc.mirrorResyncRequired ());
  TEST_ASSERT_EQUAL_HEX32 (gconf.raw, tmc.gconf_.raw);
  TEST_ASSERT_EQUAL_HEX32 (ihold_irun.raw, tmc.ihold_irun_.raw);
  TEST_ASSERT_EQUAL_HEX32 (chopconf.raw, tmc.chopconf_.raw);
  TEST_ASSERT_EQUAL_UINT16 (32u, tmc.getMicrostepsPerStep ());
  TEST_ASSERT_EQUAL_HEX32 (5u, tmc.reply_delay_raw_);
  TEST_ASSERT_EQUAL_HEX32 (11u, tmc.tpowerdown_raw_);
  TEST_ASSERT_EQUAL_HEX32 (0x3456u, tmc.tpwmthrs_raw_);
  TEST_ASSERT_EQUAL_HEX32 (0x4567u, tmc.vactual_raw_);
  TEST_ASSERT_EQUAL_HEX32 (0x5678u, tmc.tcoolthrs_raw_);
  TEST_ASSERT_EQUAL_HEX32 (0x19u, tmc.sgthrs_raw_);
}


int
main (int argc, char **argv)
{
  (void)argc;
  (void)argv;

  UNITY_BEGIN ();

  RUN_TEST (test_microsteps_powers_of_two_map_exactly);
  RUN_TEST (test_microsteps_clamps_out_of_range_inputs);
  RUN_TEST (test_read_retries_after_reply_timeout);
  RUN_TEST (test_readRegister_reports_timeout_error_when_no_reply);
  RUN_TEST (test_readRegister_retries_after_crc_mismatch);
  RUN_TEST (test_nonblocking_startRead_poll_takeReadResult_succeeds);
  RUN_TEST (test_nonblocking_startWrite_poll_takeWriteResult_succeeds);
  RUN_TEST (test_nonblocking_startRead_returns_busy_while_transaction_active);
  RUN_TEST (test_nonblocking_read_drains_stale_rx_bytes_before_transaction);
  RUN_TEST (test_nonblocking_read_reports_echo_corruption_after_retries);
  RUN_TEST (test_protocol_read_request_pack_is_explicit_and_crc_valid);
  RUN_TEST (test_protocol_write_datagram_pack_is_explicit_and_big_endian);
  RUN_TEST (test_protocol_crc_detects_corruption);
  RUN_TEST (test_setRMSCurrent_clamps_very_low_requests_to_zero);
  RUN_TEST (test_tmc_bits_bit_get_set);
  RUN_TEST (test_tmc_bits_field_get_set_and_masks);
  RUN_TEST (test_tmc_bits_field_does_not_clobber_other_bits);
  RUN_TEST (test_reg_gconf_encodes_expected_bits);
  RUN_TEST (test_reg_chopconf_encodes_expected_fields);
  RUN_TEST (test_reg_pwmconf_encodes_expected_fields);
  RUN_TEST (test_reg_ihold_irun_encodes_expected_fields);
  RUN_TEST (test_reg_coolconf_encodes_expected_fields);
  RUN_TEST (test_reg_gstat_encodes_expected_bits);
  RUN_TEST (test_reg_replydelay_encodes_expected_field);
  RUN_TEST (test_reg_ioin_encodes_expected_fields);
  RUN_TEST (test_reg_drv_status_encodes_expected_fields);
  RUN_TEST (test_reg_pwm_scale_encodes_expected_fields);
  RUN_TEST (test_getPwmScaleAuto_decodes_signed_pwm_scale_auto_field);
  RUN_TEST (test_reg_pwm_auto_encodes_expected_fields);
  RUN_TEST (test_uartbus_devices_read_address_specific_register_values);
  RUN_TEST (test_uartbus_shared_bus_scopes_results_to_the_started_device);
  RUN_TEST (test_registers_typed_helpers_read_ioin_version);
  RUN_TEST (test_device_write_verification_succeeds_when_ifcnt_increments);
  RUN_TEST (test_device_write_verification_reports_ifcnt_mismatch);
  RUN_TEST (test_driver_initialize_configures_serial_mode_defaults);
  RUN_TEST (test_facade_exposes_driver_and_registers_on_internal_bus);
  RUN_TEST (test_driver_microsteps_follows_legacy_power_of_two_flooring);
  RUN_TEST (test_facade_nonblocking_and_subobjects_share_bus_state);
  RUN_TEST (test_facade_getSettings_tracks_subobject_register_changes);
  RUN_TEST (test_facade_getMicrostepsPerStep_tracks_driver_changes_when_configured);
  RUN_TEST (test_facade_enableWriteVerification_uses_ifcnt_verification);
  RUN_TEST (test_family_style_done_aliases_follow_result_ready_state);
  RUN_TEST (test_recoverFromDeviceReset_replays_cached_configuration);
  RUN_TEST (test_resyncReadableConfiguration_refreshes_cached_state_from_device);

  return UNITY_END ();
}
