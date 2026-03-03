#include <unity.h>

#include <TMC2209.h>

#include "FakeSerial.hpp"

// Unity hooks (optional on most platforms, but harmless)
void setUp(void) {}
void tearDown(void) {}

void test_microsteps_powers_of_two_map_exactly()
{
  TMC2209 tmc;

  const uint16_t cases[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
  for (uint16_t v : cases)
  {
    tmc.setMicrostepsPerStep(v);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(v, tmc.getMicrostepsPerStep(), "microsteps power-of-two mapping");
  }
}

void test_microsteps_clamps_out_of_range_inputs()
{
  TMC2209 tmc;

  tmc.setMicrostepsPerStep(0);
  TEST_ASSERT_EQUAL_UINT16(1, tmc.getMicrostepsPerStep());

  tmc.setMicrostepsPerStep(999);
  TEST_ASSERT_EQUAL_UINT16(256, tmc.getMicrostepsPerStep());
}

void test_read_retries_after_reply_timeout()
{
  FakeSerial serial;

  // Configure the virtual device: IOIN (0x06) contains VERSION in the top byte.
  // VERSION constant in the library is 0x21.
  const uint32_t ioin_value = 0x21000000u;
  serial.set_register_value(0x06, ioin_value);

  // First read request gets no reply; second read request gets a reply.
  serial.reply_after_attempt(2);

  TMC2209 tmc;
  tmc.setup(serial, TMC2209::SERIAL_ADDRESS_0);

  // Ignore any traffic during setup.
  serial.reset();

  const uint8_t version = tmc.getVersion();
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(0x21, version, "expected VERSION after retry");

  // The retry bug fix should cause exactly 2 read requests in this scenario.
  TEST_ASSERT_EQUAL_UINT_MESSAGE(2, serial.read_request_count(), "expected one retry after timeout");
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  UNITY_BEGIN();

  RUN_TEST(test_microsteps_powers_of_two_map_exactly);
  RUN_TEST(test_microsteps_clamps_out_of_range_inputs);
  RUN_TEST(test_read_retries_after_reply_timeout);

  return UNITY_END();
}
