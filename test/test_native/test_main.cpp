#include <unity.h>

#include <TMC2209.h>
#include <TMC2209/Protocol.hpp>
#include <tmc_bits.hpp>
#include <tmc2209_registers.hpp>

#include "FakeSerial.hpp"

// Unity hooks (optional on most platforms, but harmless)
void
setUp (void)
{
}
void
tearDown (void)
{
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
  RUN_TEST (test_protocol_read_request_pack_is_explicit_and_crc_valid);
  RUN_TEST (test_protocol_write_datagram_pack_is_explicit_and_big_endian);
  RUN_TEST (test_protocol_crc_detects_corruption);
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
  RUN_TEST (test_reg_pwm_auto_encodes_expected_fields);

  return UNITY_END ();
}
