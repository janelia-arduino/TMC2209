#ifndef TMC2209_UART_BUS_PARAMETERS_HPP
#define TMC2209_UART_BUS_PARAMETERS_HPP

namespace tmc2209
{

struct UartBusParameters
{
  bool drain_before_transaction{ true };
};

} // namespace tmc2209

#endif // TMC2209_UART_BUS_PARAMETERS_HPP
