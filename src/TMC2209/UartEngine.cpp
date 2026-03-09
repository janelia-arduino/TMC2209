#include "TMC2209/UartEngine.hpp"

#include <Arduino.h>

namespace tmc2209
{

namespace
{
inline bool
deadlineReached (uint32_t now, uint32_t deadline)
{
  return static_cast<int32_t> (now - deadline) >= 0;
}
} // namespace

UartEngine::UartEngine ()
  : io_ (nullptr)
{
  reset ();
}

void
UartEngine::attach (UartEngineIo *io)
{
  io_ = io;
  reset ();
}

void
UartEngine::reset ()
{
  active_operation_ = Operation::None;
  completed_operation_ = Operation::None;
  state_ = State::Idle;
  last_error_ = UartError::None;
  result_ready_ = false;
  serial_address_ = 0;
  register_address_ = 0;
  request_size_ = 0;
  echo_index_ = 0;
  reply_index_ = 0;
  read_attempt_ = 0;
  state_deadline_us_ = 0;

  for (size_t i = 0; i < sizeof (request_bytes_); ++i)
    {
      request_bytes_[i] = 0;
      reply_bytes_[i] = 0;
    }

  read_result_ = Result<uint32_t>{};
  write_result_ = Result<void>{};
}

Result<void>
UartEngine::startRead (uint8_t serial_address, uint8_t register_address)
{
  Result<void> result;

  if (io_ == nullptr)
    {
      result.error = UartError::NotInitialized;
      last_error_ = result.error;
      return result;
    }

  if (!startOperation_ (
          Operation::Read, serial_address, register_address, 0))
    {
      result.error = UartError::Busy;
      last_error_ = result.error;
      return result;
    }

  result.error = UartError::None;
  last_error_ = result.error;
  return result;
}

Result<void>
UartEngine::startWrite (uint8_t serial_address,
                        uint8_t register_address,
                        uint32_t data)
{
  Result<void> result;

  if (io_ == nullptr)
    {
      result.error = UartError::NotInitialized;
      last_error_ = result.error;
      return result;
    }

  if (!startOperation_ (
          Operation::Write, serial_address, register_address, data))
    {
      result.error = UartError::Busy;
      last_error_ = result.error;
      return result;
    }

  result.error = UartError::None;
  last_error_ = result.error;
  return result;
}

void
UartEngine::poll ()
{
  if (io_ == nullptr)
    {
      return;
    }

  switch (state_)
    {
    case State::Idle:
      {
        return;
      }
    case State::DrainBefore:
      {
        drainRx_ ();
        resetWorkingState_ ();
        state_ = State::SendRequest;
        return;
      }
    case State::SendRequest:
      {
        for (uint8_t i = 0; i < request_size_; ++i)
          {
            (void)io_->uartWrite (request_bytes_[i]);
          }
        state_ = State::FlushRequest;
        return;
      }
    case State::FlushRequest:
      {
        io_->uartFlush ();

        if (active_operation_ == Operation::Read)
          {
            state_deadline_us_ = micros () + ECHO_TIMEOUT_US;
            state_ = State::ConsumeEcho;
          }
        else if (active_operation_ == Operation::Write)
          {
            finishWrite_ (UartError::None);
          }
        return;
      }
    case State::ConsumeEcho:
      {
        while ((echo_index_ < request_size_)
               && (io_->uartAvailable () > 0))
          {
            const int byte = io_->uartRead ();
            if ((byte < 0)
                || (static_cast<uint8_t> (byte)
                    != request_bytes_[echo_index_]))
              {
                drainRx_ ();
                scheduleReadRetry_ (UartError::RxGarbage);
                return;
              }
            ++echo_index_;
          }

        if (echo_index_ >= request_size_)
          {
            state_deadline_us_ = micros () + REPLY_TIMEOUT_US;
            state_ = State::AccumulateReply;
            return;
          }

        if (deadlineReached_ ())
          {
            scheduleReadRetry_ (UartError::EchoTimeout);
          }
        return;
      }
    case State::AccumulateReply:
      {
        while ((reply_index_ < protocol::WriteReadReplyDatagram::kSize)
               && (io_->uartAvailable () > 0))
          {
            const int byte = io_->uartRead ();
            if (byte < 0)
              {
                drainRx_ ();
                scheduleReadRetry_ (UartError::RxGarbage);
                return;
              }
            reply_bytes_[reply_index_] = static_cast<uint8_t> (byte);
            ++reply_index_;
          }

        if (reply_index_ >= protocol::WriteReadReplyDatagram::kSize)
          {
            state_ = State::ValidateReply;
            return;
          }

        if (deadlineReached_ ())
          {
            drainRx_ ();
            scheduleReadRetry_ (UartError::ReplyTimeout);
          }
        return;
      }
    case State::ValidateReply:
      {
        protocol::WriteReadReplyDatagram reply{};
        for (uint8_t i = 0; i < protocol::WriteReadReplyDatagram::kSize; ++i)
          {
            reply.bytes[i] = reply_bytes_[i];
          }

        if (!reply.hasValidCrc ())
          {
            scheduleReadRetry_ (UartError::CrcMismatch);
            return;
          }

        if (!reply.matchesReadReplyHeader (register_address_))
          {
            drainRx_ ();
            scheduleReadRetry_ (UartError::UnexpectedFrame);
            return;
          }

        finishRead_ (UartError::None, reply.data ());
        return;
      }
    case State::RetryDelay:
      {
        if (deadlineReached_ ())
          {
            state_ = State::DrainBefore;
          }
        return;
      }
    }
}

bool
UartEngine::busy () const
{
  return state_ != State::Idle;
}

bool
UartEngine::resultReady () const
{
  return result_ready_;
}

Result<uint32_t>
UartEngine::takeReadResult ()
{
  if ((!result_ready_) || (completed_operation_ != Operation::Read))
    {
      Result<uint32_t> result;
      result.value = 0;
      result.error = UartError::Busy;
      return result;
    }

  Result<uint32_t> result = read_result_;
  result_ready_ = false;
  completed_operation_ = Operation::None;
  read_result_ = Result<uint32_t>{};
  return result;
}

Result<void>
UartEngine::takeWriteResult ()
{
  if ((!result_ready_) || (completed_operation_ != Operation::Write))
    {
      Result<void> result;
      result.error = UartError::Busy;
      return result;
    }

  Result<void> result = write_result_;
  result_ready_ = false;
  completed_operation_ = Operation::None;
  write_result_ = Result<void>{};
  return result;
}

UartError
UartEngine::lastError () const
{
  return last_error_;
}

bool
UartEngine::startOperation_ (Operation operation,
                             uint8_t serial_address,
                             uint8_t register_address,
                             uint32_t data)
{
  if ((state_ != State::Idle) || result_ready_)
    {
      return false;
    }

  resetWorkingState_ ();
  active_operation_ = operation;
  completed_operation_ = Operation::None;
  last_error_ = UartError::None;
  result_ready_ = false;
  serial_address_ = serial_address;
  register_address_ = register_address;
  read_result_ = Result<uint32_t>{};
  write_result_ = Result<void>{};

  if (operation == Operation::Read)
    {
      const auto request = protocol::ReadRequestDatagram::make (
          serial_address, register_address);
      request_size_ = protocol::ReadRequestDatagram::kSize;
      for (uint8_t i = 0; i < request_size_; ++i)
        {
          request_bytes_[i] = request.bytes[i];
        }
      read_attempt_ = 0;
    }
  else if (operation == Operation::Write)
    {
      const auto request = protocol::WriteReadReplyDatagram::makeWrite (
          serial_address, register_address, data);
      request_size_ = protocol::WriteReadReplyDatagram::kSize;
      for (uint8_t i = 0; i < request_size_; ++i)
        {
          request_bytes_[i] = request.bytes[i];
        }
    }

  state_ = State::DrainBefore;
  return true;
}

void
UartEngine::resetWorkingState_ ()
{
  echo_index_ = 0;
  reply_index_ = 0;

  for (size_t i = 0; i < sizeof (reply_bytes_); ++i)
    {
      reply_bytes_[i] = 0;
    }
}

void
UartEngine::drainRx_ ()
{
  while (io_->uartAvailable () > 0)
    {
      (void)io_->uartRead ();
    }
}

void
UartEngine::scheduleReadRetry_ (UartError error)
{
  last_error_ = error;

  if (canRetryRead_ ())
    {
      ++read_attempt_;
      resetWorkingState_ ();
      state_deadline_us_ = micros () + RETRY_DELAY_US;
      state_ = State::RetryDelay;
      return;
    }

  finishRead_ (error, 0);
}

void
UartEngine::finishRead_ (UartError error, uint32_t value)
{
  active_operation_ = Operation::None;
  completed_operation_ = Operation::Read;
  state_ = State::Idle;
  last_error_ = error;
  result_ready_ = true;
  read_result_.value = value;
  read_result_.error = error;
}

void
UartEngine::finishWrite_ (UartError error)
{
  active_operation_ = Operation::None;
  completed_operation_ = Operation::Write;
  state_ = State::Idle;
  last_error_ = error;
  result_ready_ = true;
  write_result_.error = error;
}

bool
UartEngine::deadlineReached_ () const
{
  return deadlineReached (micros (), state_deadline_us_);
}

bool
UartEngine::canRetryRead_ () const
{
  return (active_operation_ == Operation::Read)
         && ((read_attempt_ + 1) < MAX_READ_ATTEMPTS);
}

} // namespace tmc2209
