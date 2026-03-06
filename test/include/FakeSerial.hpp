#pragma once

#include <cstdint>
#include <cstddef>
#include <deque>
#include <unordered_map>
#include <vector>

#include "Arduino.h"
#include "TMC2209/Protocol.hpp"

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
  void
  maybe_handle_read_request_ (const std::vector<uint8_t> &frame)
  {
    // Read request datagram is 4 bytes:
    //  byte0: [reserved:4][sync:4]
    //  byte1: serial_address
    //  byte2: [rw:1][register_address:7]  (rw=0 for read)
    //  byte3: crc

    if (frame.size () != tmc2209::protocol::ReadRequestDatagram::kSize)
      {
        return;
      }

    tmc2209::protocol::ReadRequestDatagram request{};
    for (size_t i = 0; i < tmc2209::protocol::ReadRequestDatagram::kSize; ++i)
      {
        request.bytes[i] = frame[i];
      }

    if ((request.sync () != tmc2209::protocol::SYNC)
        || (request.rw () != tmc2209::protocol::RW_READ)
        || !request.hasValidCrc ())
      {
        return;
      }

    const uint8_t register_address = request.registerAddress ();

    // We detected a read request.
    ++read_request_count_;

    if (read_request_count_ < reply_after_attempt_)
      {
        // Simulate a dropped/no-reply transaction.
        return;
      }

    auto it = register_map_.find (register_address);
    uint32_t value = (it != register_map_.end ()) ? it->second : 0u;

    auto reply = tmc2209::protocol::WriteReadReplyDatagram::makeReadReply (
        register_address, value);

    if (corrupt_crc_remaining_ > 0)
      {
        // Flip a bit to force a CRC mismatch.
        reply.bytes[tmc2209::protocol::WriteReadReplyDatagram::kSize - 1]
            ^= 0x01;
        --corrupt_crc_remaining_;
      }

    // Enqueue reply after the echo bytes already placed in RX.
    for (uint8_t b : reply.bytes)
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
