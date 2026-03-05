#pragma once

#include <stdint.h>

namespace tmc
{
namespace bits
{

template <unsigned Pos>
struct Bit
{
  static_assert (Pos < 32, "Bit position out of range");
  static constexpr uint32_t mask = (uint32_t (1) << Pos);

  static bool
  get (uint32_t reg)
  {
    return (reg & mask) != 0;
  }

  static void
  set (uint32_t &reg, bool on)
  {
    reg = on ? (reg | mask) : (reg & ~mask);
  }
};

template <unsigned Pos, unsigned Width>
struct Field
{
  static_assert (Width > 0 && Width < 32, "Field width invalid");
  static_assert (Pos + Width <= 32, "Field exceeds 32-bit range");

  static constexpr uint32_t mask = uint32_t (((uint64_t (1) << Width) - 1u) << Pos);

  static uint32_t
  get (uint32_t reg)
  {
    return (reg & mask) >> Pos;
  }

  static void
  set (uint32_t &reg, uint32_t value)
  {
    reg = (reg & ~mask) | ((value << Pos) & mask);
  }
};

} // namespace bits
} // namespace tmc
