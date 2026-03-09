#pragma once

// Minimal Arduino compatibility layer for PlatformIO native unit tests.
//
// This is *not* a full Arduino implementation. It only provides the small
// subset of types, macros, and functions that the library needs to compile
// and link on the host.

#include <cstddef>
#include <cstdint>

// Common Arduino integer typedefs
using uint8_t = std::uint8_t;
using uint16_t = std::uint16_t;
using uint32_t = std::uint32_t;
using uint64_t = std::uint64_t;
using int8_t = std::int8_t;
using int16_t = std::int16_t;
using int32_t = std::int32_t;
using int64_t = std::int64_t;

// Arduino-style constants
#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef LOW
#define LOW 0x0
#endif
#ifndef OUTPUT
#define OUTPUT 0x1
#endif
#ifndef INPUT
#define INPUT 0x0
#endif

namespace arduino_test
{
inline uint64_t &
fake_time_us ()
{
  static uint64_t value = 0;
  return value;
}

inline void
reset_time ()
{
  fake_time_us () = 0;
}

inline void
advance_time_us (uint64_t us)
{
  fake_time_us () += us;
}
} // namespace arduino_test

// Stubs (no-op) for GPIO and timing.
inline void
pinMode (uint8_t /*pin*/, uint8_t /*mode*/)
{
}
inline void
digitalWrite (uint8_t /*pin*/, uint8_t /*val*/)
{
}

inline unsigned long
micros ()
{
  return static_cast<unsigned long> (arduino_test::fake_time_us ());
}

inline unsigned long
millis ()
{
  return static_cast<unsigned long> (arduino_test::fake_time_us () / 1000ULL);
}

inline void
delay (unsigned long ms)
{
  arduino_test::advance_time_us (static_cast<uint64_t> (ms) * 1000ULL);
}
inline void
delayMicroseconds (unsigned int us)
{
  arduino_test::advance_time_us (us);
}

// Arduino's map() helper.
inline long
map (long x, long in_min, long in_max, long out_min, long out_max)
{
  // Match Arduino behavior (integer math).
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino's constrain() helper.
template <typename T>
constexpr T
constrain (T x, T a, T b)
{
  return (x < a) ? a : ((x > b) ? b : x);
}

// A minimal HardwareSerial surface compatible with what this library uses.
class HardwareSerial
{
public:
  virtual ~HardwareSerial () = default;

  virtual int
  available ()
  {
    return 0;
  }
  virtual int
  read ()
  {
    return -1;
  }
  virtual size_t
  write (uint8_t /*c*/)
  {
    return 0;
  }
  virtual void
  flush ()
  {
  }
};
