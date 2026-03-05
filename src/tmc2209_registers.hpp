#pragma once

#include <stdint.h>

#include "tmc_bits.hpp"

namespace tmc2209
{
namespace reg
{

// --------------------------------------------------------------------------
// GCONF (0x00)
// --------------------------------------------------------------------------
struct GCONF
{
  uint32_t raw{0};

  using I_SCALE_ANALOG      = tmc::bits::Bit<0>;
  using INTERNAL_RSENSE     = tmc::bits::Bit<1>;
  using EN_SPREADCYCLE      = tmc::bits::Bit<2>;
  using SHAFT               = tmc::bits::Bit<3>;
  using INDEX_OTPW          = tmc::bits::Bit<4>;
  using INDEX_STEP          = tmc::bits::Bit<5>;
  using PDN_DISABLE         = tmc::bits::Bit<6>;
  using MSTEP_REG_SELECT    = tmc::bits::Bit<7>;
  using MULTISTEP_FILT      = tmc::bits::Bit<8>;
  using TEST_MODE           = tmc::bits::Bit<9>;

  GCONF &i_scale_analog(bool v)        { I_SCALE_ANALOG::set(raw, v);   return *this; }
  bool   i_scale_analog() const        { return I_SCALE_ANALOG::get(raw); }

  GCONF &internal_rsense(bool v)       { INTERNAL_RSENSE::set(raw, v);  return *this; }
  bool   internal_rsense() const       { return INTERNAL_RSENSE::get(raw); }

  GCONF &enable_spread_cycle(bool v)   { EN_SPREADCYCLE::set(raw, v);   return *this; }
  bool   enable_spread_cycle() const   { return EN_SPREADCYCLE::get(raw); }

  GCONF &shaft(bool v)                 { SHAFT::set(raw, v);            return *this; }
  bool   shaft() const                 { return SHAFT::get(raw); }

  GCONF &index_otpw(bool v)            { INDEX_OTPW::set(raw, v);       return *this; }
  bool   index_otpw() const            { return INDEX_OTPW::get(raw); }

  GCONF &index_step(bool v)            { INDEX_STEP::set(raw, v);       return *this; }
  bool   index_step() const            { return INDEX_STEP::get(raw); }

  GCONF &pdn_disable(bool v)           { PDN_DISABLE::set(raw, v);      return *this; }
  bool   pdn_disable() const           { return PDN_DISABLE::get(raw); }

  GCONF &mstep_reg_select(bool v)      { MSTEP_REG_SELECT::set(raw, v); return *this; }
  bool   mstep_reg_select() const      { return MSTEP_REG_SELECT::get(raw); }

  GCONF &multistep_filt(bool v)        { MULTISTEP_FILT::set(raw, v);   return *this; }
  bool   multistep_filt() const        { return MULTISTEP_FILT::get(raw); }

  GCONF &test_mode(bool v)             { TEST_MODE::set(raw, v);        return *this; }
  bool   test_mode() const             { return TEST_MODE::get(raw); }
};


// --------------------------------------------------------------------------
// CHOPCONF (0x6C)
// --------------------------------------------------------------------------
// Microstep resolution encoding for CHOPCONF.MRES (bits 24..27).
// This follows the datasheet convention used by the library.
enum class Mres : uint8_t
{
  M256 = 0,
  M128 = 1,
  M64  = 2,
  M32  = 3,
  M16  = 4,
  M8   = 5,
  M4   = 6,
  M2   = 7,
  M1   = 8,
};

struct CHOPCONF
{
  uint32_t raw{0};

  using TOFF          = tmc::bits::Field<0,4>;
  using HSTART        = tmc::bits::Field<4,3>;
  using HEND          = tmc::bits::Field<7,4>;
  using TBL           = tmc::bits::Field<15,2>;
  using VSENSE        = tmc::bits::Bit<17>;
  using MRES          = tmc::bits::Field<24,4>;
  using INTERPOLATION = tmc::bits::Bit<28>;
  using DOUBLE_EDGE   = tmc::bits::Bit<29>;
  using DISS2G        = tmc::bits::Bit<30>;
  using DISS2VS       = tmc::bits::Bit<31>;

  CHOPCONF &toff(uint32_t v)           { TOFF::set(raw, v);          return *this; }
  uint32_t  toff() const               { return TOFF::get(raw); }

  CHOPCONF &hstart(uint32_t v)         { HSTART::set(raw, v);        return *this; }
  uint32_t  hstart() const             { return HSTART::get(raw); }

  CHOPCONF &hend(uint32_t v)           { HEND::set(raw, v);          return *this; }
  uint32_t  hend() const               { return HEND::get(raw); }

  CHOPCONF &tbl(uint32_t v)            { TBL::set(raw, v);           return *this; }
  uint32_t  tbl() const                { return TBL::get(raw); }

  CHOPCONF &vsense(bool v)             { VSENSE::set(raw, v);        return *this; }
  bool      vsense() const             { return VSENSE::get(raw); }

  CHOPCONF &mres(Mres v)               { MRES::set(raw, static_cast<uint32_t>(v)); return *this; }
  Mres      mres() const               { return static_cast<Mres>(MRES::get(raw)); }

  CHOPCONF &mres_raw(uint32_t v)       { MRES::set(raw, v);          return *this; }
  uint32_t  mres_raw() const           { return MRES::get(raw); }

  CHOPCONF &interpolation(bool v)      { INTERPOLATION::set(raw, v); return *this; }
  bool      interpolation() const      { return INTERPOLATION::get(raw); }

  CHOPCONF &double_edge(bool v)        { DOUBLE_EDGE::set(raw, v);   return *this; }
  bool      double_edge() const        { return DOUBLE_EDGE::get(raw); }

  CHOPCONF &diss2g(bool v)             { DISS2G::set(raw, v);        return *this; }
  bool      diss2g() const             { return DISS2G::get(raw); }

  CHOPCONF &diss2vs(bool v)            { DISS2VS::set(raw, v);       return *this; }
  bool      diss2vs() const            { return DISS2VS::get(raw); }
};


// --------------------------------------------------------------------------
// PWMCONF (0x70)
// --------------------------------------------------------------------------
struct PWMCONF
{
  uint32_t raw{0};

  using PWM_OFS      = tmc::bits::Field<0,8>;
  using PWM_GRAD     = tmc::bits::Field<8,8>;
  using PWM_FREQ     = tmc::bits::Field<16,2>;
  using PWM_AUTOSCALE= tmc::bits::Bit<18>;
  using PWM_AUTOGRAD = tmc::bits::Bit<19>;
  using FREEWHEEL    = tmc::bits::Field<20,2>;
  using PWM_REG      = tmc::bits::Field<24,4>;
  using PWM_LIM      = tmc::bits::Field<28,4>;

  PWMCONF &pwm_offset(uint32_t v)      { PWM_OFS::set(raw, v);        return *this; }
  uint32_t pwm_offset() const          { return PWM_OFS::get(raw); }

  PWMCONF &pwm_grad(uint32_t v)        { PWM_GRAD::set(raw, v);       return *this; }
  uint32_t pwm_grad() const            { return PWM_GRAD::get(raw); }

  PWMCONF &pwm_freq(uint32_t v)        { PWM_FREQ::set(raw, v);       return *this; }
  uint32_t pwm_freq() const            { return PWM_FREQ::get(raw); }

  PWMCONF &pwm_autoscale(bool v)       { PWM_AUTOSCALE::set(raw, v);  return *this; }
  bool     pwm_autoscale() const       { return PWM_AUTOSCALE::get(raw); }

  PWMCONF &pwm_autograd(bool v)        { PWM_AUTOGRAD::set(raw, v);   return *this; }
  bool     pwm_autograd() const        { return PWM_AUTOGRAD::get(raw); }

  PWMCONF &freewheel(uint32_t v)       { FREEWHEEL::set(raw, v);      return *this; }
  uint32_t freewheel() const           { return FREEWHEEL::get(raw); }

  PWMCONF &pwm_reg(uint32_t v)         { PWM_REG::set(raw, v);        return *this; }
  uint32_t pwm_reg() const             { return PWM_REG::get(raw); }

  PWMCONF &pwm_lim(uint32_t v)         { PWM_LIM::set(raw, v);        return *this; }
  uint32_t pwm_lim() const             { return PWM_LIM::get(raw); }
};

} // namespace reg
} // namespace tmc2209
