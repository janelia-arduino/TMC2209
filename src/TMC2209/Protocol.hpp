#pragma once

#include <stddef.h>
#include <stdint.h>

namespace tmc2209
{
namespace protocol
{

constexpr uint8_t SYNC = 0b101;
constexpr uint8_t RW_READ = 0;
constexpr uint8_t RW_WRITE = 1;
constexpr uint8_t READ_REPLY_SERIAL_ADDRESS = 0xFF;

constexpr uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
constexpr uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
inline uint8_t
calculateCrc (const uint8_t *bytes, size_t datagram_size)
{
  uint8_t crc = 0;

  if ((bytes == nullptr) || (datagram_size == 0))
    {
      return crc;
    }

  for (size_t i = 0; i < (datagram_size - 1); ++i)
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
          byte = static_cast<uint8_t> (byte >> 1);
        }
    }

  return crc;
}

inline void
packDataBytes (uint32_t data, uint8_t *bytes)
{
  if (bytes == nullptr)
    {
      return;
    }

  bytes[0] = static_cast<uint8_t> ((data >> 24) & 0xFFu);
  bytes[1] = static_cast<uint8_t> ((data >> 16) & 0xFFu);
  bytes[2] = static_cast<uint8_t> ((data >> 8) & 0xFFu);
  bytes[3] = static_cast<uint8_t> (data & 0xFFu);
}

inline uint32_t
unpackDataBytes (const uint8_t *bytes)
{
  if (bytes == nullptr)
    {
      return 0;
    }

  return (uint32_t (bytes[0]) << 24) | (uint32_t (bytes[1]) << 16)
         | (uint32_t (bytes[2]) << 8) | uint32_t (bytes[3]);
}

struct ReadRequestDatagram
{
  static constexpr uint8_t kSize = READ_REQUEST_DATAGRAM_SIZE;

  uint8_t bytes[kSize]{};

  static ReadRequestDatagram
  make (uint8_t serial_address, uint8_t register_address)
  {
    ReadRequestDatagram datagram{};
    datagram.bytes[0] = SYNC;
    datagram.bytes[1] = serial_address;
    datagram.bytes[2] = static_cast<uint8_t> (register_address & 0x7Fu);
    datagram.bytes[3] = calculateCrc (datagram.bytes, kSize);
    return datagram;
  }

  uint8_t
  sync () const
  {
    return static_cast<uint8_t> (bytes[0] & 0x0Fu);
  }

  uint8_t
  serialAddress () const
  {
    return bytes[1];
  }

  uint8_t
  registerAddress () const
  {
    return static_cast<uint8_t> (bytes[2] & 0x7Fu);
  }

  uint8_t
  rw () const
  {
    return static_cast<uint8_t> ((bytes[2] >> 7) & 0x01u);
  }

  uint8_t
  crc () const
  {
    return bytes[3];
  }

  bool
  hasValidCrc () const
  {
    return crc () == calculateCrc (bytes, kSize);
  }
};

struct WriteReadReplyDatagram
{
  static constexpr uint8_t kSize = WRITE_READ_REPLY_DATAGRAM_SIZE;

  uint8_t bytes[kSize]{};

  static WriteReadReplyDatagram
  makeWrite (uint8_t serial_address, uint8_t register_address, uint32_t data)
  {
    WriteReadReplyDatagram datagram{};
    datagram.bytes[0] = SYNC;
    datagram.bytes[1] = serial_address;
    datagram.bytes[2]
        = static_cast<uint8_t> ((register_address & 0x7Fu) | (RW_WRITE << 7));
    packDataBytes (data, &datagram.bytes[3]);
    datagram.bytes[7] = calculateCrc (datagram.bytes, kSize);
    return datagram;
  }

  static WriteReadReplyDatagram
  makeReadReply (uint8_t register_address, uint32_t data)
  {
    WriteReadReplyDatagram datagram{};
    datagram.bytes[0] = SYNC;
    datagram.bytes[1] = READ_REPLY_SERIAL_ADDRESS;
    datagram.bytes[2] = static_cast<uint8_t> (register_address & 0x7Fu);
    packDataBytes (data, &datagram.bytes[3]);
    datagram.bytes[7] = calculateCrc (datagram.bytes, kSize);
    return datagram;
  }

  uint8_t
  sync () const
  {
    return static_cast<uint8_t> (bytes[0] & 0x0Fu);
  }

  uint8_t
  serialAddress () const
  {
    return bytes[1];
  }

  uint8_t
  registerAddress () const
  {
    return static_cast<uint8_t> (bytes[2] & 0x7Fu);
  }

  uint8_t
  rw () const
  {
    return static_cast<uint8_t> ((bytes[2] >> 7) & 0x01u);
  }

  uint32_t
  data () const
  {
    return unpackDataBytes (&bytes[3]);
  }

  uint8_t
  crc () const
  {
    return bytes[7];
  }

  bool
  hasValidCrc () const
  {
    return crc () == calculateCrc (bytes, kSize);
  }

  bool
  matchesReadReplyHeader (uint8_t register_address) const
  {
    return (sync () == SYNC)
           && (serialAddress () == READ_REPLY_SERIAL_ADDRESS)
           && (registerAddress () == static_cast<uint8_t> (register_address & 0x7Fu))
           && (rw () == RW_READ);
  }
};

} // namespace protocol
} // namespace tmc2209
