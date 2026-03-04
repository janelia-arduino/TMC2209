// ----------------------------------------------------------------------------
// Result.hpp
//
// Transport result/error helpers for explicit UART error reporting.
//
// This header is intentionally lightweight so it can be used on Arduino and
// in PlatformIO native unit tests.
// ----------------------------------------------------------------------------

#ifndef TMC2209_RESULT_HPP
#define TMC2209_RESULT_HPP

namespace tmc2209
{

enum class UartError : unsigned char
{
  None = 0,
  Busy,
  NotInitialized,
  EchoTimeout,
  ReplyTimeout,
  CrcMismatch,
  UnexpectedFrame,
  RxGarbage,
  WriteVerifyFailed,
};

template<typename T>
struct Result
{
  T value{};
  UartError error{UartError::None};

  constexpr bool ok() const { return error == UartError::None; }
};

// Specialization for void results.
template<>
struct Result<void>
{
  UartError error{UartError::None};

  constexpr bool ok() const { return error == UartError::None; }
};

}  // namespace tmc2209

#endif  // TMC2209_RESULT_HPP
