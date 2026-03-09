#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Result.hpp"
#include "TMC2209/Protocol.hpp"

namespace tmc2209
{

class UartEngineIo
{
public:
  virtual ~UartEngineIo () {}

  virtual int uartAvailable () = 0;
  virtual int uartRead () = 0;
  virtual size_t uartWrite (uint8_t c) = 0;
  virtual void uartFlush () = 0;
};

class UartEngine
{
public:
  UartEngine ();

  void attach (UartEngineIo *io);
  void reset ();

  Result<void> startRead (uint8_t serial_address, uint8_t register_address);
  Result<void> startWrite (uint8_t serial_address,
                           uint8_t register_address,
                           uint32_t data);

  void poll ();

  bool busy () const;
  bool resultReady () const;

  Result<uint32_t> takeReadResult ();
  Result<void> takeWriteResult ();

  UartError lastError () const;

private:
  enum class Operation : uint8_t
  {
    None,
    Read,
    Write,
  };

  enum class State : uint8_t
  {
    Idle,
    DrainBefore,
    SendRequest,
    FlushRequest,
    ConsumeEcho,
    AccumulateReply,
    ValidateReply,
    RetryDelay,
  };

  bool startOperation_ (Operation operation,
                        uint8_t serial_address,
                        uint8_t register_address,
                        uint32_t data);

  void resetWorkingState_ ();
  void drainRx_ ();
  void scheduleReadRetry_ (UartError error);
  void finishRead_ (UartError error, uint32_t value);
  void finishWrite_ (UartError error);
  bool deadlineReached_ () const;
  bool canRetryRead_ () const;

  UartEngineIo *io_;
  Operation active_operation_;
  Operation completed_operation_;
  State state_;

  UartError last_error_;
  bool result_ready_;

  uint8_t serial_address_;
  uint8_t register_address_;

  uint8_t request_bytes_[protocol::WriteReadReplyDatagram::kSize];
  uint8_t request_size_;

  uint8_t echo_index_;

  uint8_t reply_bytes_[protocol::WriteReadReplyDatagram::kSize];
  uint8_t reply_index_;

  uint8_t read_attempt_;
  uint32_t state_deadline_us_;

  Result<uint32_t> read_result_;
  Result<void> write_result_;

  static constexpr uint32_t ECHO_TIMEOUT_US = 4000;
  static constexpr uint32_t REPLY_TIMEOUT_US = 10000;
  static constexpr uint32_t RETRY_DELAY_US = 20UL * 1000UL;
  static constexpr uint8_t MAX_READ_ATTEMPTS = 5;
};

} // namespace tmc2209
