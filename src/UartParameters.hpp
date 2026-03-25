#ifndef TMC2209_UART_PARAMETERS_HPP
#define TMC2209_UART_PARAMETERS_HPP

#include <stdint.h>

namespace tmc2209
{

struct UartParameters
{
  uint8_t serial_address{ 0 };
  bool verify_writes{ false };
};

} // namespace tmc2209

#endif // TMC2209_UART_PARAMETERS_HPP
