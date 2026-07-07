// ----------------------------------------------------------------------------
// test_tmc2209.c
//
// Host based unit tests for the portable C17 TMC2209 library. A mock HAL
// captures transmitted datagrams and simulates the single wire UART echo and
// TMC2209 register replies so the protocol implementation can be verified
// without hardware.
//
// Build and run with: make test
// ----------------------------------------------------------------------------

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "tmc2209.h"

#define MOCK_BUFFER_SIZE 4096

typedef struct mock_serial
{
  // every byte written by the library
  uint8_t tx_log[MOCK_BUFFER_SIZE];
  size_t tx_log_size;

  // pending bytes readable by the library (echo + replies)
  uint8_t rx_buffer[MOCK_BUFFER_SIZE];
  size_t rx_head;
  size_t rx_tail;

  // simulated TMC2209 register file
  uint32_t registers[128];

  // when set, reply datagrams are queued with an invalid crc
  bool corrupt_replies;

  bool enable_pin_state;
  bool enable_pin_written;
} mock_serial_t;

static mock_serial_t mock;

static void mock_rx_push(uint8_t byte)
{
  assert(mock.rx_head < MOCK_BUFFER_SIZE);
  mock.rx_buffer[mock.rx_head++] = byte;
}

static uint8_t mock_crc(uint8_t const * datagram, uint8_t datagram_size)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < (uint8_t)(datagram_size - 1); ++i)
  {
    uint8_t byte = datagram[i];
    for (uint8_t j = 0; j < 8; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      }
      else
      {
        crc = (uint8_t)(crc << 1);
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

// Handle a complete datagram the way a real TMC2209 would: store writes in
// the register file and queue reply datagrams for reads.
static void mock_handle_datagram(uint8_t const * datagram, size_t size)
{
  if (size == 8)
  {
    // write access datagram
    uint8_t register_address = datagram[2] & 0x7F;
    uint32_t data = ((uint32_t)datagram[3] << 24) |
      ((uint32_t)datagram[4] << 16) |
      ((uint32_t)datagram[5] << 8) |
      (uint32_t)datagram[6];
    mock.registers[register_address] = data;
  }
  else if (size == 4)
  {
    // read access datagram, queue the reply after the echo
    uint8_t register_address = datagram[2] & 0x7F;
    uint32_t data = mock.registers[register_address];
    uint8_t reply[8];
    reply[0] = 0x05;
    reply[1] = 0xFF;
    reply[2] = register_address;
    reply[3] = (uint8_t)(data >> 24);
    reply[4] = (uint8_t)(data >> 16);
    reply[5] = (uint8_t)(data >> 8);
    reply[6] = (uint8_t)data;
    reply[7] = mock_crc(reply, 8);
    if (mock.corrupt_replies)
    {
      reply[7] ^= 0xFF;
    }
    for (size_t i = 0; i < 8; ++i)
    {
      mock_rx_push(reply[i]);
    }
  }
}

static void mock_serial_write(void * context,
  uint8_t const * data,
  size_t size)
{
  (void)context;
  assert(mock.tx_log_size + size <= MOCK_BUFFER_SIZE);
  memcpy(&mock.tx_log[mock.tx_log_size], data, size);
  mock.tx_log_size += size;

  // the single wire interface echoes every transmitted byte back on RX
  for (size_t i = 0; i < size; ++i)
  {
    mock_rx_push(data[i]);
  }

  mock_handle_datagram(data, size);
}

static size_t mock_serial_available(void * context)
{
  (void)context;
  return mock.rx_head - mock.rx_tail;
}

static int16_t mock_serial_read(void * context)
{
  (void)context;
  if (mock.rx_tail >= mock.rx_head)
  {
    return -1;
  }
  return (int16_t)mock.rx_buffer[mock.rx_tail++];
}

static void mock_serial_flush(void * context)
{
  (void)context;
}

static void mock_delay_microseconds(void * context, uint32_t microseconds)
{
  (void)context;
  (void)microseconds;
}

static void mock_delay_milliseconds(void * context, uint32_t milliseconds)
{
  (void)context;
  (void)milliseconds;
}

static void mock_set_hardware_enable_pin(void * context, bool enable)
{
  (void)context;
  mock.enable_pin_state = enable;
  mock.enable_pin_written = true;
}

static tmc2209_hal_t const mock_hal = {
  .serial_write = mock_serial_write,
  .serial_available = mock_serial_available,
  .serial_read = mock_serial_read,
  .serial_flush = mock_serial_flush,
  .delay_microseconds = mock_delay_microseconds,
  .delay_milliseconds = mock_delay_milliseconds,
  .set_hardware_enable_pin = mock_set_hardware_enable_pin,
  .context = NULL,
};

static void mock_reset(void)
{
  memset(&mock, 0, sizeof(mock));
  // IOIN reset value: VERSION 0x21 in bits 31:24, ENN low
  mock.registers[0x06] = 0x21000000u;
}

static size_t test_count;

#define TEST(name) \
  do \
  { \
    ++test_count; \
    printf("  %-60s", name); \
  } while (0)

#define TEST_PASS() printf("ok\n")

static void test_crc_against_known_value(void)
{
  TEST("crc matches TMC datasheet CRC8-ATM algorithm");
  // read request for IOIN (0x06) at serial address 0:
  // 0x05 0x00 0x06 crc
  uint8_t datagram[4] = {0x05, 0x00, 0x06, 0x00};
  uint8_t crc = mock_crc(datagram, 4);
  // reference value computed with the reference implementation from the
  // TMC2209 datasheet section 4.1.2 (swuart_calcCRC)
  assert(crc == 0x6F);
  TEST_PASS();
}

static void test_write_datagram_format(void)
{
  TEST("write datagram has correct framing, byte order and crc");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_2);

  mock.tx_log_size = 0;
  tmc2209_set_stall_guard_threshold(&tmc, 0xAB);

  assert(mock.tx_log_size == 8);
  assert(mock.tx_log[0] == 0x05); // sync
  assert(mock.tx_log[1] == 0x02); // serial address
  assert(mock.tx_log[2] == (0x40 | 0x80)); // SGTHRS register, write bit
  assert(mock.tx_log[3] == 0x00); // data MSB first
  assert(mock.tx_log[4] == 0x00);
  assert(mock.tx_log[5] == 0x00);
  assert(mock.tx_log[6] == 0xAB);
  assert(mock.tx_log[7] == mock_crc(mock.tx_log, 8));
  TEST_PASS();
}

static void test_read_datagram_format_and_reply(void)
{
  TEST("read request is well formed and reply data is decoded");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  mock.registers[0x41] = 0x000001F4u; // SG_RESULT = 500
  mock.tx_log_size = 0;
  uint16_t result = tmc2209_get_stall_guard_result(&tmc);

  assert(mock.tx_log_size == 4);
  assert(mock.tx_log[0] == 0x05); // sync
  assert(mock.tx_log[1] == 0x00); // serial address
  assert(mock.tx_log[2] == 0x41); // SG_RESULT register, read bit clear
  assert(mock.tx_log[3] == mock_crc(mock.tx_log, 4));
  assert(result == 500);
  TEST_PASS();
}

static void test_register_write_roundtrip(void)
{
  TEST("written register values read back identically");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_set_microsteps_per_step(&tmc, 64);
  assert(tmc2209_get_microsteps_per_step(&tmc) == 64);

  // CHOPCONF mres field (bits 27:24) must be MRES_064 = 0b0010
  assert(((mock.registers[0x6C] >> 24) & 0x0F) == 0x02);
  TEST_PASS();
}

static void test_microsteps_rounding(void)
{
  TEST("non power of two microsteps round down");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_set_microsteps_per_step(&tmc, 255);
  assert(tmc2209_get_microsteps_per_step(&tmc) == 128);
  tmc2209_set_microsteps_per_step(&tmc, 3);
  assert(tmc2209_get_microsteps_per_step(&tmc) == 2);
  tmc2209_set_microsteps_per_step(&tmc, 0);
  assert(tmc2209_get_microsteps_per_step(&tmc) == 1);
  tmc2209_set_microsteps_per_step(&tmc, 300);
  assert(tmc2209_get_microsteps_per_step(&tmc) == 256);
  TEST_PASS();
}

static void test_version_and_communication(void)
{
  TEST("version is read from IOIN and communication is detected");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  assert(tmc2209_get_version(&tmc) == 0x21);
  assert(tmc2209_is_communicating(&tmc));
  assert(tmc2209_is_setup_and_communicating(&tmc));
  assert(!tmc2209_is_communicating_but_not_setup(&tmc));
  TEST_PASS();
}

static void test_setup_configures_gconf(void)
{
  TEST("setup enables pdn_disable, mstep_reg_select and multistep_filt");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  uint32_t gconf = mock.registers[0x00];
  assert((gconf & (1u << 0)) == 0u); // i_scale_analog off
  assert((gconf & (1u << 6)) != 0u); // pdn_disable on
  assert((gconf & (1u << 7)) != 0u); // mstep_reg_select on
  assert((gconf & (1u << 8)) != 0u); // multistep_filt on
  TEST_PASS();
}

static void test_enable_disable(void)
{
  TEST("enable/disable drive the enable pin and CHOPCONF toff");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  // setup leaves the driver disabled
  assert(mock.enable_pin_written);
  assert(!mock.enable_pin_state);
  assert((mock.registers[0x6C] & 0x0Fu) == 0u); // toff = 0

  tmc2209_enable(&tmc);
  assert(mock.enable_pin_state);
  assert((mock.registers[0x6C] & 0x0Fu) == 3u); // toff = default 3

  tmc2209_disable(&tmc);
  assert(!mock.enable_pin_state);
  assert((mock.registers[0x6C] & 0x0Fu) == 0u);
  TEST_PASS();
}

static void test_current_settings(void)
{
  TEST("current percent values map to IHOLD_IRUN register fields");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_set_all_current_values(&tmc, 100, 50, 100);
  uint32_t ihold_irun = mock.registers[0x10];
  assert((ihold_irun & 0x1Fu) == 15u); // ihold = 50% -> 15
  assert(((ihold_irun >> 8) & 0x1Fu) == 31u); // irun = 100% -> 31
  assert(((ihold_irun >> 16) & 0x0Fu) == 15u); // iholddelay = 100% -> 15

  tmc2209_settings_t settings = tmc2209_get_settings(&tmc);
  assert(settings.is_communicating);
  assert(settings.is_setup);
  assert(settings.irun_register_value == 31);
  assert(settings.irun_percent == 100);
  assert(settings.ihold_register_value == 15);
  TEST_PASS();
}

static void test_status_decoding(void)
{
  TEST("DRV_STATUS bits decode into the status struct");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  // standstill (bit 31), stealth chop mode (bit 30), current_scaling = 17
  // (bits 20:16), over temperature warning (bit 0)
  mock.registers[0x6F] = (1u << 31) | (1u << 30) | (17u << 16) | 1u;
  tmc2209_status_t status = tmc2209_get_status(&tmc);
  assert(status.standstill == 1);
  assert(status.stealth_chop_mode == 1);
  assert(status.current_scaling == 17);
  assert(status.over_temperature_warning == 1);
  assert(status.over_temperature_shutdown == 0);
  TEST_PASS();
}

static void test_global_status(void)
{
  TEST("GSTAT decodes and clear functions write the right bits");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  mock.registers[0x01] = 0x5u; // reset and uv_cp
  tmc2209_global_status_t global_status = tmc2209_get_global_status(&tmc);
  assert(global_status.reset == 1);
  assert(global_status.drv_err == 0);
  assert(global_status.uv_cp == 1);

  tmc2209_clear_reset(&tmc);
  assert(mock.registers[0x01] == 0x1u);
  tmc2209_clear_drive_error(&tmc);
  assert(mock.registers[0x01] == 0x2u);
  TEST_PASS();
}

static void test_pwm_scale_auto_sign_extension(void)
{
  TEST("pwm scale auto sign extends the 9 bit register value");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  mock.registers[0x71] = (24u << 16); // +24
  assert(tmc2209_get_pwm_scale_auto(&tmc) == 24);

  mock.registers[0x71] = (0x1E8u << 16); // -24 in 9 bit two's complement
  assert(tmc2209_get_pwm_scale_auto(&tmc) == -24);
  TEST_PASS();
}

static void test_reply_delay(void)
{
  TEST("reply delay is clamped and written to bits 11:8");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_set_reply_delay(&tmc, 5);
  assert(mock.registers[0x03] == (5u << 8));
  tmc2209_set_reply_delay(&tmc, 200);
  assert(mock.registers[0x03] == (15u << 8));
  TEST_PASS();
}

static void test_move_at_velocity(void)
{
  TEST("negative velocities are written as two's complement VACTUAL");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_move_at_velocity(&tmc, -2000);
  assert(mock.registers[0x22] == (uint32_t)(int32_t)-2000);
  tmc2209_move_using_step_dir_interface(&tmc);
  assert(mock.registers[0x22] == 0u);
  TEST_PASS();
}

static void test_cool_step(void)
{
  TEST("cool step thresholds are constrained and written to COOLCONF");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_enable_cool_step(&tmc, 0, 20);
  uint32_t coolconf = mock.registers[0x42];
  assert((coolconf & 0x0Fu) == 1u); // semin constrained up to 1
  assert(((coolconf >> 8) & 0x0Fu) == 15u); // semax constrained down to 15

  tmc2209_disable_cool_step(&tmc);
  assert((mock.registers[0x42] & 0x0Fu) == 0u); // semin off
  TEST_PASS();
}

static void test_corrupt_reply_is_rejected(void)
{
  TEST("replies with a bad crc are rejected and read returns 0");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  mock.registers[0x41] = 1234u;
  assert(tmc2209_get_stall_guard_result(&tmc) == 1234);

  mock.corrupt_replies = true;
  assert(tmc2209_get_stall_guard_result(&tmc) == 0);

  mock.corrupt_replies = false;
  assert(tmc2209_get_stall_guard_result(&tmc) == 1234);
  TEST_PASS();
}

static void test_rms_current(void)
{
  TEST("rms current selects vsense range and writes irun/ihold");
  mock_reset();
  tmc2209_t tmc;
  tmc2209_setup(&tmc, &mock_hal, TMC2209_SERIAL_ADDRESS_0);

  // 800 mA with a 110 mOhm sense resistor lands in the vsense = 1 range
  tmc2209_set_rms_current(&tmc, 800, 0.11f, 0.5f);
  assert((mock.registers[0x6C] & (1u << 17)) != 0u); // vsense set
  uint32_t ihold_irun = mock.registers[0x10];
  uint32_t irun = (ihold_irun >> 8) & 0x1Fu;
  uint32_t ihold = ihold_irun & 0x1Fu;
  assert(irun > 0u && irun <= 31u);
  assert(ihold == irun / 2u);
  TEST_PASS();
}

int main(void)
{
  printf("tmc2209 host tests\n");

  test_crc_against_known_value();
  test_write_datagram_format();
  test_read_datagram_format_and_reply();
  test_register_write_roundtrip();
  test_microsteps_rounding();
  test_version_and_communication();
  test_setup_configures_gconf();
  test_enable_disable();
  test_current_settings();
  test_status_decoding();
  test_global_status();
  test_pwm_scale_auto_sign_extension();
  test_reply_delay();
  test_move_at_velocity();
  test_cool_step();
  test_corrupt_reply_is_rejected();
  test_rms_current();

  printf("%zu tests passed\n", test_count);
  return 0;
}
