#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <initializer_list>
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
// - Detects 8-byte write datagrams and updates an internal per-device register
//   map so later reads can observe the writes.
// - Can be configured to only start replying after N read requests, enabling
//   retry behavior tests.
// - Can inject stale RX garbage and corrupt echo bytes for engine tests.
class FakeSerial : public HardwareSerial
{
public:
  FakeSerial () = default;

  // Reset RX/TX buffers and request counters. Register contents persist across
  // reset() so tests can ignore setup traffic without losing virtual state.
  void
  reset ()
  {
    rx_.clear ();
    tx_.clear ();
    current_frame_.clear ();
    current_frame_echo_corrupted_ = false;
    read_request_count_ = 0;
    corrupt_crc_remaining_ = corrupt_crc_for_first_replies_;
    corrupt_echo_remaining_ = corrupt_echo_for_first_read_requests_;
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

  // Configure: corrupt the echoed first byte for the first N valid read
  // requests. The request still reaches the virtual device, but the RX echo no
  // longer matches the transmitted bytes.
  void
  corrupt_echo_for_first_read_requests (unsigned int requests)
  {
    corrupt_echo_for_first_read_requests_ = requests;
    corrupt_echo_remaining_ = requests;
  }

  // Configure: suppress IFCNT increments for the first N valid writes.
  // Useful for exercising opt-in write verification failure behavior.
  void
  suppress_ifcnt_increment_for_first_writes (unsigned int writes)
  {
    suppress_ifcnt_increment_for_first_writes_ = writes;
    suppress_ifcnt_increment_remaining_ = writes;
  }

  // Inject stale bytes into the RX queue before the next transaction.
  void
  queue_rx_byte (uint8_t value)
  {
    rx_.push_back (value);
  }

  void
  queue_rx_bytes (std::initializer_list<uint8_t> bytes)
  {
    for (uint8_t b : bytes)
      {
        rx_.push_back (b);
      }
  }

  unsigned int
  read_request_count () const
  {
    return read_request_count_;
  }

  const std::vector<uint8_t> &
  tx_bytes () const
  {
    return tx_;
  }

  // Set a default register value returned for reads of that register on any
  // serial address unless overridden by the address-specific overload below.
  void
  set_register_value (uint8_t register_address, uint32_t value)
  {
    default_register_map_[register_address] = value;
  }

  // Set a register value for a specific UART serial address.
  void
  set_register_value (uint8_t serial_address,
                      uint8_t register_address,
                      uint32_t value)
  {
    addressed_register_map_[make_key_ (serial_address, register_address)]
        = value;
  }

  uint32_t
  register_value (uint8_t serial_address,
                  uint8_t register_address,
                  uint32_t fallback = 0u) const
  {
    const auto specific
        = addressed_register_map_.find (make_key_ (serial_address,
                                                   register_address));
    if (specific != addressed_register_map_.end ())
      {
        return specific->second;
      }

    const auto generic = default_register_map_.find (register_address);
    if (generic != default_register_map_.end ())
      {
        return generic->second;
      }

    return fallback;
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
    const uint8_t b = rx_.front ();
    rx_.pop_front ();
    return b;
  }

  size_t
  write (uint8_t c) override
  {
    tx_.push_back (c);

    // Coupled one-wire behavior: everything written is echoed back.
    uint8_t echoed = c;
    if (current_frame_.empty ())
      {
        current_frame_echo_corrupted_ = false;
        if (corrupt_echo_remaining_ > 0)
          {
            echoed ^= 0x01u;
            current_frame_echo_corrupted_ = true;
          }
      }
    rx_.push_back (echoed);

    current_frame_.push_back (c);
    maybe_complete_frame_ ();

    return 1;
  }

  void
  flush () override
  {
    // no-op for host simulation
  }

private:
  static uint16_t
  make_key_ (uint8_t serial_address, uint8_t register_address)
  {
    return static_cast<uint16_t> ((uint16_t (serial_address) << 8)
                                  | uint16_t (register_address));
  }

  void
  clear_current_frame_ ()
  {
    current_frame_.clear ();
    current_frame_echo_corrupted_ = false;
  }

  bool
  is_valid_read_request_frame_ (const std::vector<uint8_t> &frame) const
  {
    if (frame.size () != tmc2209::protocol::ReadRequestDatagram::kSize)
      {
        return false;
      }

    tmc2209::protocol::ReadRequestDatagram request{};
    for (size_t i = 0; i < tmc2209::protocol::ReadRequestDatagram::kSize; ++i)
      {
        request.bytes[i] = frame[i];
      }

    return (request.sync () == tmc2209::protocol::SYNC)
           && (request.rw () == tmc2209::protocol::RW_READ)
           && request.hasValidCrc ();
  }

  void
  maybe_complete_frame_ ()
  {
    if ((current_frame_.size () == tmc2209::protocol::ReadRequestDatagram::kSize)
        && is_valid_read_request_frame_ (current_frame_))
      {
        maybe_handle_read_request_ (current_frame_);
        clear_current_frame_ ();
        return;
      }

    if (current_frame_.size ()
        == tmc2209::protocol::WriteReadReplyDatagram::kSize)
      {
        maybe_handle_write_datagram_ (current_frame_);
        clear_current_frame_ ();
        return;
      }

    if (current_frame_.size () > tmc2209::protocol::WriteReadReplyDatagram::kSize)
      {
        clear_current_frame_ ();
      }
  }

  void
  maybe_handle_read_request_ (const std::vector<uint8_t> &frame)
  {
    tmc2209::protocol::ReadRequestDatagram request{};
    for (size_t i = 0; i < tmc2209::protocol::ReadRequestDatagram::kSize; ++i)
      {
        request.bytes[i] = frame[i];
      }

    if (current_frame_echo_corrupted_ && (corrupt_echo_remaining_ > 0))
      {
        --corrupt_echo_remaining_;
      }

    ++read_request_count_;

    if (read_request_count_ < reply_after_attempt_)
      {
        return;
      }

    const uint8_t serial_address = request.serialAddress ();
    const uint8_t register_address = request.registerAddress ();
    const uint32_t value = register_value (serial_address, register_address);

    auto reply = tmc2209::protocol::WriteReadReplyDatagram::makeReadReply (
        register_address, value);

    if (corrupt_crc_remaining_ > 0)
      {
        reply.bytes[tmc2209::protocol::WriteReadReplyDatagram::kSize - 1]
            ^= 0x01u;
        --corrupt_crc_remaining_;
      }

    for (uint8_t b : reply.bytes)
      {
        rx_.push_back (b);
      }
  }

  void
  maybe_handle_write_datagram_ (const std::vector<uint8_t> &frame)
  {
    tmc2209::protocol::WriteReadReplyDatagram datagram{};
    for (size_t i = 0; i < tmc2209::protocol::WriteReadReplyDatagram::kSize;
         ++i)
      {
        datagram.bytes[i] = frame[i];
      }

    if ((datagram.sync () != tmc2209::protocol::SYNC)
        || (datagram.rw () != tmc2209::protocol::RW_WRITE)
        || !datagram.hasValidCrc ())
      {
        return;
      }

    const uint8_t serial_address = datagram.serialAddress ();
    addressed_register_map_[make_key_ (serial_address,
                                       datagram.registerAddress ())]
        = datagram.data ();

    if (suppress_ifcnt_increment_remaining_ > 0)
      {
        --suppress_ifcnt_increment_remaining_;
        return;
      }

    const uint16_t ifcnt_key = make_key_ (serial_address, 0x02u);
    const uint8_t previous_ifcnt = static_cast<uint8_t> (
        register_value (serial_address, 0x02u));
    addressed_register_map_[ifcnt_key]
        = static_cast<uint8_t> (previous_ifcnt + 1u);
  }

  std::deque<uint8_t> rx_;
  std::vector<uint8_t> tx_;
  std::vector<uint8_t> current_frame_;
  bool current_frame_echo_corrupted_{ false };

  std::unordered_map<uint8_t, uint32_t> default_register_map_;
  std::unordered_map<uint16_t, uint32_t> addressed_register_map_;

  unsigned int reply_after_attempt_{ 1 };
  unsigned int read_request_count_{ 0 };

  unsigned int corrupt_crc_for_first_replies_{ 0 };
  unsigned int corrupt_crc_remaining_{ 0 };

  unsigned int corrupt_echo_for_first_read_requests_{ 0 };
  unsigned int corrupt_echo_remaining_{ 0 };

  unsigned int suppress_ifcnt_increment_for_first_writes_{ 0 };
  unsigned int suppress_ifcnt_increment_remaining_{ 0 };
};
