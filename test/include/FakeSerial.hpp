#pragma once

#include <cstdint>
#include <cstddef>
#include <deque>
#include <unordered_map>
#include <vector>

#include "Arduino.h"

// A very small host-side serial simulator for exercising the TMC2209 UART
// protocol logic in unit tests.
//
// Capabilities:
// - Echoes all written bytes back into the RX queue (coupled one-wire style).
// - Detects 4-byte TMC2209 read-request datagrams and can enqueue a valid
//   8-byte reply datagram with correct CRC.
// - Can be configured to only start replying after N read requests, enabling
//   retry behavior tests.
class FakeSerial : public HardwareSerial
{
public:
  FakeSerial () = default;

  // Reset RX/TX buffers and request counters.
  void
  reset ()
  {
    rx_.clear ();
    tx_.clear ();
    current_frame_.clear ();
    read_request_count_ = 0;
    corrupt_crc_remaining_ = corrupt_crc_for_first_replies_;
  }

  // Configure: only respond starting with this read request attempt.
  // Example: reply_after_attempt(2) -> first read request gets no reply, second
  // and later read requests get replies.
  void
  reply_after_attempt (unsigned int attempt)
  {
    reply_after_attempt_ = attempt;
  }

  // Configure: corrupt the CRC byte for the first N replies.
  // Useful for exercising CRC mismatch + retry behavior.
  void
  corrupt_crc_for_first_replies (unsigned int replies)
  {
    corrupt_crc_for_first_replies_ = replies;
    corrupt_crc_remaining_ = replies;
  }

  unsigned int
  read_request_count () const
  {
    return read_request_count_;
  }

  // Set a register value to return for reads of that register.
  void
  set_register_value (uint8_t register_address, uint32_t value)
  {
    register_map_[register_address] = value;
  }

  // HardwareSerial interface
  int
  available () override
  {
    return static_cast<int> (rx_.size ());
  }

  int
  read () override
  {
    if (rx_.empty ())
      {
        return -1;
      }
    uint8_t b = rx_.front ();
    rx_.pop_front ();
    return b;
  }

  size_t
  write (uint8_t c) override
  {
    tx_.push_back (c);

    // Coupled one-wire behavior: everything written is echoed back.
    rx_.push_back (c);

    // Track outgoing frames in 4-byte chunks to detect read requests.
    current_frame_.push_back (c);
    if (current_frame_.size () == 4)
      {
        maybe_handle_read_request_ (current_frame_);
        current_frame_.clear ();
      }

    return 1;
  }

  void
  flush () override
  {
    // no-op for host simulation
  }

private:
  static constexpr uint8_t SYNC_NIBBLE = 0b101; // library uses 0b101

  // CRC algorithm matches the implementation in TMC2209.cpp.
  static uint8_t
  crc8_tmc_ (const uint8_t *bytes, size_t count_without_crc)
  {
    uint8_t crc = 0;
    for (size_t i = 0; i < count_without_crc; ++i)
      {
        uint8_t byte = bytes[i];
        for (uint8_t j = 0; j < 8; ++j)
          {
            if ((crc >> 7) ^ (byte & 0x01))
              {
                crc = static_cast<uint8_t> ((crc << 1) ^ 0x07);
              }
            else
              {
                crc = static_cast<uint8_t> (crc << 1);
              }
            byte >>= 1;
          }
      }
    return crc;
  }

  static uint32_t
  reverse_u32_ (uint32_t data)
  {
    // Mirror of TMC2209::reverseData()
    return ((data & 0x000000FFu) << 24) | ((data & 0x0000FF00u) << 8) | ((data & 0x00FF0000u) >> 8) | ((data & 0xFF000000u) >> 24);
  }

  void
  maybe_handle_read_request_ (const std::vector<uint8_t> &frame)
  {
    // Read request datagram is 4 bytes:
    //  byte0: [reserved:4][sync:4]
    //  byte1: serial_address
    //  byte2: [rw:1][register_address:7]  (rw=0 for read)
    //  byte3: crc

    if (frame.size () != 4)
      {
        return;
      }

    const uint8_t byte0 = frame[0];
    const uint8_t sync = (byte0 & 0x0F);
    if (sync != SYNC_NIBBLE)
      {
        return;
      }

    const uint8_t byte2 = frame[2];
    const uint8_t rw = (byte2 >> 7) & 0x01;
    if (rw != 0)
      {
        // Not a read request.
        return;
      }

    const uint8_t register_address = (byte2 & 0x7F);

    // We detected a read request.
    ++read_request_count_;

    if (read_request_count_ < reply_after_attempt_)
      {
        // Simulate a dropped/no-reply transaction.
        return;
      }

    auto it = register_map_.find (register_address);
    uint32_t value = (it != register_map_.end ()) ? it->second : 0u;

    // Build a write/read reply datagram (8 bytes) with a valid CRC.
    // We don't strictly need to match every header field because the library
    // currently only validates CRC, but we keep it realistic.
    uint8_t reply[8] = {};
    reply[0] = SYNC_NIBBLE;      // reserved nibble is 0
    reply[1] = 0xFF;             // READ_REPLY_SERIAL_ADDRESS
    reply[2] = register_address; // rw bit 0 for read

    const uint32_t data_field = reverse_u32_ (value);
    reply[3] = static_cast<uint8_t> (data_field & 0xFF);
    reply[4] = static_cast<uint8_t> ((data_field >> 8) & 0xFF);
    reply[5] = static_cast<uint8_t> ((data_field >> 16) & 0xFF);
    reply[6] = static_cast<uint8_t> ((data_field >> 24) & 0xFF);

    reply[7] = crc8_tmc_ (reply, 7);

    if (corrupt_crc_remaining_ > 0)
      {
        // Flip a bit to force a CRC mismatch.
        reply[7] ^= 0x01;
        --corrupt_crc_remaining_;
      }

    // Enqueue reply after the echo bytes already placed in RX.
    for (uint8_t b : reply)
      {
        rx_.push_back (b);
      }
  }

  std::deque<uint8_t> rx_;
  std::vector<uint8_t> tx_;
  std::vector<uint8_t> current_frame_;

  std::unordered_map<uint8_t, uint32_t> register_map_;

  unsigned int reply_after_attempt_{ 1 };
  unsigned int read_request_count_{ 0 };

  unsigned int corrupt_crc_for_first_replies_{ 0 };
  unsigned int corrupt_crc_remaining_{ 0 };
};
