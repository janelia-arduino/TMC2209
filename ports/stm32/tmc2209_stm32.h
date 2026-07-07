// ----------------------------------------------------------------------------
// tmc2209_stm32.h
//
// TMC2209 HAL implementation for STM32 microcontrollers using only CMSIS
// (no STM32 HAL/LL drivers required).
//
// The port drives a USART/UART peripheral by direct register access and is
// compatible with both STM32 USART peripheral generations:
// - v1 peripherals with SR/DR registers (e.g. F1, F2, F4, L1)
// - v2 peripherals with ISR/TDR/RDR registers (e.g. F0, F3, F7, G0, G4,
//   L0, L4, H7, U5, WB, WL)
//
// Usage:
// 1. Tell this port which CMSIS device header to use, either by defining
//    TMC2209_STM32_DEVICE_HEADER on the compiler command line, e.g.
//      -DTMC2209_STM32_DEVICE_HEADER='"stm32f4xx.h"'
//    or by including your device header before this file.
// 2. Configure the UART pins and peripheral yourself (clocks, GPIO alternate
//    function, baud rate, 8N1, receiver and transmitter enabled) before
//    calling tmc2209_setup().
// 3. Initialize a tmc2209_stm32_t with tmc2209_stm32_hal_init() and pass the
//    resulting tmc2209_hal_t to tmc2209_setup().
//
// See examples/STM32CMSIS for a complete example.
// ----------------------------------------------------------------------------

#ifndef TMC2209_STM32_H
#define TMC2209_STM32_H

#if defined(TMC2209_STM32_DEVICE_HEADER)
#include TMC2209_STM32_DEVICE_HEADER
#endif

#include "tmc2209.h"

#ifdef __cplusplus
extern "C" {
#endif

// Software receive buffer, must be a power of two and at least 16 bytes so a
// full echoed request plus reply datagram (12 bytes) fits.
#define TMC2209_STM32_RX_BUFFER_SIZE 16u

// Optional hardware enable (ENN) pin descriptor. Set port to NULL when the
// ENN pin is hardwired.
typedef struct tmc2209_stm32_enable_pin
{
  GPIO_TypeDef * port;
  uint8_t pin; // pin number 0-15
} tmc2209_stm32_enable_pin_t;

typedef struct tmc2209_stm32
{
  USART_TypeDef * usart;
  tmc2209_stm32_enable_pin_t enable_pin;
  uint8_t rx_buffer[TMC2209_STM32_RX_BUFFER_SIZE];
  volatile uint8_t rx_head;
  volatile uint8_t rx_tail;
} tmc2209_stm32_t;

// Fill hal with callbacks bound to the given port instance.
//
// usart: UART peripheral connected to the TMC2209, already configured for
// the desired baud rate, 8 data bits, no parity, 1 stop bit, with the
// transmitter and receiver enabled.
//
// enable_port/enable_pin: GPIO output wired to the TMC2209 ENN input,
// already configured as a push-pull output. Pass NULL for enable_port if
// the ENN pin is not controlled by the microcontroller.
//
// The DWT cycle counter is used for microsecond delays on Cortex-M3 and
// above; it is enabled by this function. SystemCoreClock must be correct
// (call SystemCoreClockUpdate() after clock configuration).
void tmc2209_stm32_hal_init(tmc2209_stm32_t * port,
  tmc2209_hal_t * hal,
  USART_TypeDef * usart,
  GPIO_TypeDef * enable_port,
  uint8_t enable_pin);

#ifdef __cplusplus
}
#endif

#endif // TMC2209_STM32_H
