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
  uint32_t raw{ 0 };

  using I_SCALE_ANALOG = tmc::bits::Bit<0>;
  using INTERNAL_RSENSE = tmc::bits::Bit<1>;
  using EN_SPREADCYCLE = tmc::bits::Bit<2>;
  using SHAFT = tmc::bits::Bit<3>;
  using INDEX_OTPW = tmc::bits::Bit<4>;
  using INDEX_STEP = tmc::bits::Bit<5>;
  using PDN_DISABLE = tmc::bits::Bit<6>;
  using MSTEP_REG_SELECT = tmc::bits::Bit<7>;
  using MULTISTEP_FILT = tmc::bits::Bit<8>;
  using TEST_MODE = tmc::bits::Bit<9>;

  GCONF &
  i_scale_analog (bool v)
  {
    I_SCALE_ANALOG::set (raw, v);
    return *this;
  }
  bool
  i_scale_analog () const
  {
    return I_SCALE_ANALOG::get (raw);
  }

  GCONF &
  internal_rsense (bool v)
  {
    INTERNAL_RSENSE::set (raw, v);
    return *this;
  }
  bool
  internal_rsense () const
  {
    return INTERNAL_RSENSE::get (raw);
  }

  GCONF &
  enable_spread_cycle (bool v)
  {
    EN_SPREADCYCLE::set (raw, v);
    return *this;
  }
  bool
  enable_spread_cycle () const
  {
    return EN_SPREADCYCLE::get (raw);
  }

  GCONF &
  shaft (bool v)
  {
    SHAFT::set (raw, v);
    return *this;
  }
  bool
  shaft () const
  {
    return SHAFT::get (raw);
  }

  GCONF &
  index_otpw (bool v)
  {
    INDEX_OTPW::set (raw, v);
    return *this;
  }
  bool
  index_otpw () const
  {
    return INDEX_OTPW::get (raw);
  }

  GCONF &
  index_step (bool v)
  {
    INDEX_STEP::set (raw, v);
    return *this;
  }
  bool
  index_step () const
  {
    return INDEX_STEP::get (raw);
  }

  GCONF &
  pdn_disable (bool v)
  {
    PDN_DISABLE::set (raw, v);
    return *this;
  }
  bool
  pdn_disable () const
  {
    return PDN_DISABLE::get (raw);
  }

  GCONF &
  mstep_reg_select (bool v)
  {
    MSTEP_REG_SELECT::set (raw, v);
    return *this;
  }
  bool
  mstep_reg_select () const
  {
    return MSTEP_REG_SELECT::get (raw);
  }

  GCONF &
  multistep_filt (bool v)
  {
    MULTISTEP_FILT::set (raw, v);
    return *this;
  }
  bool
  multistep_filt () const
  {
    return MULTISTEP_FILT::get (raw);
  }

  GCONF &
  test_mode (bool v)
  {
    TEST_MODE::set (raw, v);
    return *this;
  }
  bool
  test_mode () const
  {
    return TEST_MODE::get (raw);
  }
};




// --------------------------------------------------------------------------
// GSTAT (0x01)
// --------------------------------------------------------------------------
struct GSTAT
{
  uint32_t raw{ 0 };

  using RESET = tmc::bits::Bit<0>;
  using DRV_ERR = tmc::bits::Bit<1>;
  using UV_CP = tmc::bits::Bit<2>;

  GSTAT &
  reset (bool v)
  {
    RESET::set (raw, v);
    return *this;
  }
  bool
  reset () const
  {
    return RESET::get (raw);
  }

  GSTAT &
  drv_err (bool v)
  {
    DRV_ERR::set (raw, v);
    return *this;
  }
  bool
  drv_err () const
  {
    return DRV_ERR::get (raw);
  }

  GSTAT &
  uv_cp (bool v)
  {
    UV_CP::set (raw, v);
    return *this;
  }
  bool
  uv_cp () const
  {
    return UV_CP::get (raw);
  }
};

// --------------------------------------------------------------------------
// REPLYDELAY (0x03)
// --------------------------------------------------------------------------
struct REPLYDELAY
{
  uint32_t raw{ 0 };

  using REPLYDELAY_FIELD = tmc::bits::Field<8, 4>;

  REPLYDELAY &
  replydelay (uint32_t v)
  {
    REPLYDELAY_FIELD::set (raw, v);
    return *this;
  }
  uint32_t
  replydelay () const
  {
    return REPLYDELAY_FIELD::get (raw);
  }
};

// --------------------------------------------------------------------------
// IOIN (0x06)
// --------------------------------------------------------------------------
struct IOIN
{
  uint32_t raw{ 0 };

  using ENN = tmc::bits::Bit<0>;
  using MS1 = tmc::bits::Bit<2>;
  using MS2 = tmc::bits::Bit<3>;
  using DIAG = tmc::bits::Bit<4>;
  using PDN_SERIAL = tmc::bits::Bit<6>;
  using STEP = tmc::bits::Bit<7>;
  using SPREAD_EN = tmc::bits::Bit<8>;
  using DIR = tmc::bits::Bit<9>;
  using VERSION = tmc::bits::Field<24, 8>;

  IOIN &
  enn (bool v)
  {
    ENN::set (raw, v);
    return *this;
  }
  bool
  enn () const
  {
    return ENN::get (raw);
  }

  IOIN &
  ms1 (bool v)
  {
    MS1::set (raw, v);
    return *this;
  }
  bool
  ms1 () const
  {
    return MS1::get (raw);
  }

  IOIN &
  ms2 (bool v)
  {
    MS2::set (raw, v);
    return *this;
  }
  bool
  ms2 () const
  {
    return MS2::get (raw);
  }

  IOIN &
  diag (bool v)
  {
    DIAG::set (raw, v);
    return *this;
  }
  bool
  diag () const
  {
    return DIAG::get (raw);
  }

  IOIN &
  pdn_serial (bool v)
  {
    PDN_SERIAL::set (raw, v);
    return *this;
  }
  bool
  pdn_serial () const
  {
    return PDN_SERIAL::get (raw);
  }

  IOIN &
  step (bool v)
  {
    STEP::set (raw, v);
    return *this;
  }
  bool
  step () const
  {
    return STEP::get (raw);
  }

  IOIN &
  spread_en (bool v)
  {
    SPREAD_EN::set (raw, v);
    return *this;
  }
  bool
  spread_en () const
  {
    return SPREAD_EN::get (raw);
  }

  IOIN &
  dir (bool v)
  {
    DIR::set (raw, v);
    return *this;
  }
  bool
  dir () const
  {
    return DIR::get (raw);
  }

  IOIN &
  version (uint32_t v)
  {
    VERSION::set (raw, v);
    return *this;
  }
  uint32_t
  version () const
  {
    return VERSION::get (raw);
  }
};

// --------------------------------------------------------------------------
// IHOLD_IRUN (0x10)
// --------------------------------------------------------------------------
struct IHOLD_IRUN
{
  uint32_t raw{ 0 };

  using IHOLD = tmc::bits::Field<0, 5>;
  using IRUN = tmc::bits::Field<8, 5>;
  using IHOLDDELAY = tmc::bits::Field<16, 4>;

  IHOLD_IRUN &
  ihold (uint32_t v)
  {
    IHOLD::set (raw, v);
    return *this;
  }
  uint32_t
  ihold () const
  {
    return IHOLD::get (raw);
  }

  IHOLD_IRUN &
  irun (uint32_t v)
  {
    IRUN::set (raw, v);
    return *this;
  }
  uint32_t
  irun () const
  {
    return IRUN::get (raw);
  }

  IHOLD_IRUN &
  iholddelay (uint32_t v)
  {
    IHOLDDELAY::set (raw, v);
    return *this;
  }
  uint32_t
  iholddelay () const
  {
    return IHOLDDELAY::get (raw);
  }
};

// --------------------------------------------------------------------------
// COOLCONF (0x42)
// --------------------------------------------------------------------------
struct COOLCONF
{
  uint32_t raw{ 0 };

  using SEMIN = tmc::bits::Field<0, 4>;
  using SEUP = tmc::bits::Field<5, 2>;
  using SEMAX = tmc::bits::Field<8, 4>;
  using SEDN = tmc::bits::Field<13, 2>;
  using SEIMIN = tmc::bits::Bit<15>;

  COOLCONF &
  semin (uint32_t v)
  {
    SEMIN::set (raw, v);
    return *this;
  }
  uint32_t
  semin () const
  {
    return SEMIN::get (raw);
  }

  COOLCONF &
  seup (uint32_t v)
  {
    SEUP::set (raw, v);
    return *this;
  }
  uint32_t
  seup () const
  {
    return SEUP::get (raw);
  }

  COOLCONF &
  semax (uint32_t v)
  {
    SEMAX::set (raw, v);
    return *this;
  }
  uint32_t
  semax () const
  {
    return SEMAX::get (raw);
  }

  COOLCONF &
  sedn (uint32_t v)
  {
    SEDN::set (raw, v);
    return *this;
  }
  uint32_t
  sedn () const
  {
    return SEDN::get (raw);
  }

  COOLCONF &
  seimin (bool v)
  {
    SEIMIN::set (raw, v);
    return *this;
  }
  bool
  seimin () const
  {
    return SEIMIN::get (raw);
  }
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
  M64 = 2,
  M32 = 3,
  M16 = 4,
  M8 = 5,
  M4 = 6,
  M2 = 7,
  M1 = 8,
};

struct CHOPCONF
{
  uint32_t raw{ 0 };

  using TOFF = tmc::bits::Field<0, 4>;
  using HSTART = tmc::bits::Field<4, 3>;
  using HEND = tmc::bits::Field<7, 4>;
  using TBL = tmc::bits::Field<15, 2>;
  using VSENSE = tmc::bits::Bit<17>;
  using MRES = tmc::bits::Field<24, 4>;
  using INTERPOLATION = tmc::bits::Bit<28>;
  using DOUBLE_EDGE = tmc::bits::Bit<29>;
  using DISS2G = tmc::bits::Bit<30>;
  using DISS2VS = tmc::bits::Bit<31>;

  CHOPCONF &
  toff (uint32_t v)
  {
    TOFF::set (raw, v);
    return *this;
  }
  uint32_t
  toff () const
  {
    return TOFF::get (raw);
  }

  CHOPCONF &
  hstart (uint32_t v)
  {
    HSTART::set (raw, v);
    return *this;
  }
  uint32_t
  hstart () const
  {
    return HSTART::get (raw);
  }

  CHOPCONF &
  hend (uint32_t v)
  {
    HEND::set (raw, v);
    return *this;
  }
  uint32_t
  hend () const
  {
    return HEND::get (raw);
  }

  CHOPCONF &
  tbl (uint32_t v)
  {
    TBL::set (raw, v);
    return *this;
  }
  uint32_t
  tbl () const
  {
    return TBL::get (raw);
  }

  CHOPCONF &
  vsense (bool v)
  {
    VSENSE::set (raw, v);
    return *this;
  }
  bool
  vsense () const
  {
    return VSENSE::get (raw);
  }

  CHOPCONF &
  mres (Mres v)
  {
    MRES::set (raw, static_cast<uint32_t> (v));
    return *this;
  }
  Mres
  mres () const
  {
    return static_cast<Mres> (MRES::get (raw));
  }

  CHOPCONF &
  mres_raw (uint32_t v)
  {
    MRES::set (raw, v);
    return *this;
  }
  uint32_t
  mres_raw () const
  {
    return MRES::get (raw);
  }

  CHOPCONF &
  interpolation (bool v)
  {
    INTERPOLATION::set (raw, v);
    return *this;
  }
  bool
  interpolation () const
  {
    return INTERPOLATION::get (raw);
  }

  CHOPCONF &
  double_edge (bool v)
  {
    DOUBLE_EDGE::set (raw, v);
    return *this;
  }
  bool
  double_edge () const
  {
    return DOUBLE_EDGE::get (raw);
  }

  CHOPCONF &
  diss2g (bool v)
  {
    DISS2G::set (raw, v);
    return *this;
  }
  bool
  diss2g () const
  {
    return DISS2G::get (raw);
  }

  CHOPCONF &
  diss2vs (bool v)
  {
    DISS2VS::set (raw, v);
    return *this;
  }
  bool
  diss2vs () const
  {
    return DISS2VS::get (raw);
  }
};



// --------------------------------------------------------------------------
// DRV_STATUS (0x6F)
// --------------------------------------------------------------------------
struct DRV_STATUS
{
  uint32_t raw{ 0 };

  using OTW = tmc::bits::Bit<0>;
  using OTS = tmc::bits::Bit<1>;
  using S2GA = tmc::bits::Bit<2>;
  using S2GB = tmc::bits::Bit<3>;
  using S2VSA = tmc::bits::Bit<4>;
  using S2VSB = tmc::bits::Bit<5>;
  using OLA = tmc::bits::Bit<6>;
  using OLB = tmc::bits::Bit<7>;
  using T120 = tmc::bits::Bit<8>;
  using T143 = tmc::bits::Bit<9>;
  using T150 = tmc::bits::Bit<10>;
  using T157 = tmc::bits::Bit<11>;
  using CS_ACTUAL = tmc::bits::Field<16, 5>;
  using STEALTH = tmc::bits::Bit<30>;
  using STST = tmc::bits::Bit<31>;

  bool
  over_temperature_warning () const
  {
    return OTW::get (raw);
  }
  bool
  over_temperature_shutdown () const
  {
    return OTS::get (raw);
  }
  bool
  short_to_ground_a () const
  {
    return S2GA::get (raw);
  }
  bool
  short_to_ground_b () const
  {
    return S2GB::get (raw);
  }
  bool
  low_side_short_a () const
  {
    return S2VSA::get (raw);
  }
  bool
  low_side_short_b () const
  {
    return S2VSB::get (raw);
  }
  bool
  open_load_a () const
  {
    return OLA::get (raw);
  }
  bool
  open_load_b () const
  {
    return OLB::get (raw);
  }
  bool
  over_temperature_120c () const
  {
    return T120::get (raw);
  }
  bool
  over_temperature_143c () const
  {
    return T143::get (raw);
  }
  bool
  over_temperature_150c () const
  {
    return T150::get (raw);
  }
  bool
  over_temperature_157c () const
  {
    return T157::get (raw);
  }

  DRV_STATUS &
  current_scaling (uint32_t v)
  {
    CS_ACTUAL::set (raw, v);
    return *this;
  }
  uint32_t
  current_scaling () const
  {
    return CS_ACTUAL::get (raw);
  }

  DRV_STATUS &
  stealth_chop_mode (bool v)
  {
    STEALTH::set (raw, v);
    return *this;
  }
  bool
  stealth_chop_mode () const
  {
    return STEALTH::get (raw);
  }

  DRV_STATUS &
  standstill (bool v)
  {
    STST::set (raw, v);
    return *this;
  }
  bool
  standstill () const
  {
    return STST::get (raw);
  }

  // For unit tests: allow setting individual status bits.
  DRV_STATUS &
  over_temperature_warning (bool v)
  {
    OTW::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  over_temperature_shutdown (bool v)
  {
    OTS::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  short_to_ground_a (bool v)
  {
    S2GA::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  short_to_ground_b (bool v)
  {
    S2GB::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  low_side_short_a (bool v)
  {
    S2VSA::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  low_side_short_b (bool v)
  {
    S2VSB::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  open_load_a (bool v)
  {
    OLA::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  open_load_b (bool v)
  {
    OLB::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  over_temperature_120c (bool v)
  {
    T120::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  over_temperature_143c (bool v)
  {
    T143::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  over_temperature_150c (bool v)
  {
    T150::set (raw, v);
    return *this;
  }
  DRV_STATUS &
  over_temperature_157c (bool v)
  {
    T157::set (raw, v);
    return *this;
  }
};

// --------------------------------------------------------------------------
// PWMCONF (0x70)
// --------------------------------------------------------------------------
struct PWMCONF
{
  uint32_t raw{ 0 };

  using PWM_OFS = tmc::bits::Field<0, 8>;
  using PWM_GRAD = tmc::bits::Field<8, 8>;
  using PWM_FREQ = tmc::bits::Field<16, 2>;
  using PWM_AUTOSCALE = tmc::bits::Bit<18>;
  using PWM_AUTOGRAD = tmc::bits::Bit<19>;
  using FREEWHEEL = tmc::bits::Field<20, 2>;
  using PWM_REG = tmc::bits::Field<24, 4>;
  using PWM_LIM = tmc::bits::Field<28, 4>;

  PWMCONF &
  pwm_offset (uint32_t v)
  {
    PWM_OFS::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_offset () const
  {
    return PWM_OFS::get (raw);
  }

  PWMCONF &
  pwm_grad (uint32_t v)
  {
    PWM_GRAD::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_grad () const
  {
    return PWM_GRAD::get (raw);
  }

  PWMCONF &
  pwm_freq (uint32_t v)
  {
    PWM_FREQ::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_freq () const
  {
    return PWM_FREQ::get (raw);
  }

  PWMCONF &
  pwm_autoscale (bool v)
  {
    PWM_AUTOSCALE::set (raw, v);
    return *this;
  }
  bool
  pwm_autoscale () const
  {
    return PWM_AUTOSCALE::get (raw);
  }

  PWMCONF &
  pwm_autograd (bool v)
  {
    PWM_AUTOGRAD::set (raw, v);
    return *this;
  }
  bool
  pwm_autograd () const
  {
    return PWM_AUTOGRAD::get (raw);
  }

  PWMCONF &
  freewheel (uint32_t v)
  {
    FREEWHEEL::set (raw, v);
    return *this;
  }
  uint32_t
  freewheel () const
  {
    return FREEWHEEL::get (raw);
  }

  PWMCONF &
  pwm_reg (uint32_t v)
  {
    PWM_REG::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_reg () const
  {
    return PWM_REG::get (raw);
  }

  PWMCONF &
  pwm_lim (uint32_t v)
  {
    PWM_LIM::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_lim () const
  {
    return PWM_LIM::get (raw);
  }
};



// --------------------------------------------------------------------------
// PWM_SCALE (0x71)
// --------------------------------------------------------------------------
struct PWM_SCALE
{
  uint32_t raw{ 0 };

  using PWM_SCALE_SUM = tmc::bits::Field<0, 8>;
  using PWM_SCALE_AUTO = tmc::bits::Field<16, 9>;

  PWM_SCALE &
  pwm_scale_sum (uint32_t v)
  {
    PWM_SCALE_SUM::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_scale_sum () const
  {
    return PWM_SCALE_SUM::get (raw);
  }

  PWM_SCALE &
  pwm_scale_auto (uint32_t v)
  {
    PWM_SCALE_AUTO::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_scale_auto () const
  {
    return PWM_SCALE_AUTO::get (raw);
  }
};

// --------------------------------------------------------------------------
// PWM_AUTO (0x72)
// --------------------------------------------------------------------------
struct PWM_AUTO
{
  uint32_t raw{ 0 };

  using PWM_OFS_AUTO = tmc::bits::Field<0, 8>;
  using PWM_GRAD_AUTO = tmc::bits::Field<16, 8>;

  PWM_AUTO &
  pwm_offset_auto (uint32_t v)
  {
    PWM_OFS_AUTO::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_offset_auto () const
  {
    return PWM_OFS_AUTO::get (raw);
  }

  PWM_AUTO &
  pwm_gradient_auto (uint32_t v)
  {
    PWM_GRAD_AUTO::set (raw, v);
    return *this;
  }
  uint32_t
  pwm_gradient_auto () const
  {
    return PWM_GRAD_AUTO::get (raw);
  }
};

} // namespace reg
} // namespace tmc2209
