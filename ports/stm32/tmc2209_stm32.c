// ----------------------------------------------------------------------------
// tmc2209_stm32.c
//
// TMC2209 HAL implementation for STM32 microcontrollers using only CMSIS.
// ----------------------------------------------------------------------------

#include "tmc2209_stm32.h"

// ----------------------------------------------------------------------------
// USART register compatibility layer
//
// STM32 devices ship one of two USART peripheral generations. Detect which
// one the CMSIS device header describes from the flag macros it defines.
// ----------------------------------------------------------------------------

#if defined(USART_ISR_RXNE_RXFNE) // v2 peripheral with FIFO (G0, G4, H7, ...)
#define TMC2209_STM32_FLAG_RXNE USART_ISR_RXNE_RXFNE
#define TMC2209_STM32_FLAG_TXE USART_ISR_TXE_TXFNF
#define TMC2209_STM32_FLAG_TC USART_ISR_TC
#define TMC2209_STM32_STATUS(usart) ((usart)->ISR)
#define TMC2209_STM32_RX_DATA(usart) ((usart)->RDR)
#define TMC2209_STM32_TX_DATA(usart) ((usart)->TDR)
#elif defined(USART_ISR_RXNE) // v2 peripheral (F0, F3, F7, L0, L4, ...)
#define TMC2209_STM32_FLAG_RXNE USART_ISR_RXNE
#define TMC2209_STM32_FLAG_TXE USART_ISR_TXE
#define TMC2209_STM32_FLAG_TC USART_ISR_TC
#define TMC2209_STM32_STATUS(usart) ((usart)->ISR)
#define TMC2209_STM32_RX_DATA(usart) ((usart)->RDR)
#define TMC2209_STM32_TX_DATA(usart) ((usart)->TDR)
#elif defined(USART_SR_RXNE) // v1 peripheral (F1, F2, F4, L1, ...)
#define TMC2209_STM32_FLAG_RXNE USART_SR_RXNE
#define TMC2209_STM32_FLAG_TXE USART_SR_TXE
#define TMC2209_STM32_FLAG_TC USART_SR_TC
#define TMC2209_STM32_STATUS(usart) ((usart)->SR)
#define TMC2209_STM32_RX_DATA(usart) ((usart)->DR)
#define TMC2209_STM32_TX_DATA(usart) ((usart)->DR)
#else
#error "Unsupported or missing STM32 CMSIS device header. Define TMC2209_STM32_DEVICE_HEADER, e.g. -DTMC2209_STM32_DEVICE_HEADER='\"stm32f4xx.h\"'."
#endif

#define TMC2209_STM32_RX_BUFFER_MASK (TMC2209_STM32_RX_BUFFER_SIZE - 1u)

// ----------------------------------------------------------------------------
// Receive buffering
//
// The TMC2209 protocol is polled, so instead of using interrupts the port
// drains the USART receive data register into a small ring buffer every time
// the library asks how many bytes are available. The library polls
// serial_available at least once per byte period while waiting for a reply,
// which keeps the hardware receiver from overrunning.
// ----------------------------------------------------------------------------

static void tmc2209_stm32_poll_rx(tmc2209_stm32_t * port)
{
  while ((TMC2209_STM32_STATUS(port->usart) & TMC2209_STM32_FLAG_RXNE) != 0u)
  {
    uint8_t byte = (uint8_t)TMC2209_STM32_RX_DATA(port->usart);
    uint8_t next_head = (port->rx_head + 1u) & TMC2209_STM32_RX_BUFFER_MASK;
    if (next_head == port->rx_tail)
    {
      // buffer full, drop the oldest byte
      port->rx_tail = (port->rx_tail + 1u) & TMC2209_STM32_RX_BUFFER_MASK;
    }
    port->rx_buffer[port->rx_head] = byte;
    port->rx_head = next_head;
  }
}

static size_t tmc2209_stm32_serial_available(void * context)
{
  tmc2209_stm32_t * port = (tmc2209_stm32_t *)context;
  tmc2209_stm32_poll_rx(port);
  return (size_t)((uint8_t)(port->rx_head - port->rx_tail) &
    TMC2209_STM32_RX_BUFFER_MASK);
}

static int16_t tmc2209_stm32_serial_read(void * context)
{
  tmc2209_stm32_t * port = (tmc2209_stm32_t *)context;
  tmc2209_stm32_poll_rx(port);
  if (port->rx_head == port->rx_tail)
  {
    return -1;
  }
  uint8_t byte = port->rx_buffer[port->rx_tail];
  port->rx_tail = (port->rx_tail + 1u) & TMC2209_STM32_RX_BUFFER_MASK;
  return (int16_t)byte;
}

static void tmc2209_stm32_serial_write(void * context,
  uint8_t const * data,
  size_t size)
{
  tmc2209_stm32_t * port = (tmc2209_stm32_t *)context;
  for (size_t i = 0; i < size; ++i)
  {
    while ((TMC2209_STM32_STATUS(port->usart) & TMC2209_STM32_FLAG_TXE) == 0u)
    {
      // keep draining the receiver while waiting; the TMC2209 single wire
      // interface echoes every transmitted byte back on RX
      tmc2209_stm32_poll_rx(port);
    }
    TMC2209_STM32_TX_DATA(port->usart) = data[i];
  }
}

static void tmc2209_stm32_serial_flush(void * context)
{
  tmc2209_stm32_t * port = (tmc2209_stm32_t *)context;
  while ((TMC2209_STM32_STATUS(port->usart) & TMC2209_STM32_FLAG_TC) == 0u)
  {
    tmc2209_stm32_poll_rx(port);
  }
}

// ----------------------------------------------------------------------------
// Delays
// ----------------------------------------------------------------------------

#if (defined(__CORTEX_M) && (__CORTEX_M >= 3u))

static void tmc2209_stm32_dwt_enable(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR_KEY) || (__CORTEX_M == 7u)
  DWT->LAR = 0xC5ACCE55u; // unlock DWT on Cortex-M7
#endif
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void tmc2209_stm32_delay_microseconds(void * context,
  uint32_t microseconds)
{
  (void)context;
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = microseconds * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

#else // Cortex-M0/M0+ have no DWT, fall back to a calibrated busy loop

static void tmc2209_stm32_dwt_enable(void)
{
}

static void tmc2209_stm32_delay_microseconds(void * context,
  uint32_t microseconds)
{
  (void)context;
  // approximately 4 cycles per loop iteration
  volatile uint32_t count = microseconds * (SystemCoreClock / 4000000u + 1u);
  while (count > 0u)
  {
    --count;
  }
}

#endif

static void tmc2209_stm32_delay_milliseconds(void * context,
  uint32_t milliseconds)
{
  for (uint32_t i = 0; i < milliseconds; ++i)
  {
    tmc2209_stm32_delay_microseconds(context, 1000u);
  }
}

// ----------------------------------------------------------------------------
// Hardware enable pin
// ----------------------------------------------------------------------------

static void tmc2209_stm32_set_hardware_enable_pin(void * context, bool enable)
{
  tmc2209_stm32_t * port = (tmc2209_stm32_t *)context;
  if (port->enable_pin.port == NULL)
  {
    return;
  }
  // ENN is active low: reset the pin to enable the driver, set it to disable
  if (enable)
  {
    port->enable_pin.port->BSRR = (uint32_t)(1u << (port->enable_pin.pin + 16u));
  }
  else
  {
    port->enable_pin.port->BSRR = (uint32_t)(1u << port->enable_pin.pin);
  }
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void tmc2209_stm32_hal_init(tmc2209_stm32_t * port,
  tmc2209_hal_t * hal,
  USART_TypeDef * usart,
  GPIO_TypeDef * enable_port,
  uint8_t enable_pin)
{
  port->usart = usart;
  port->enable_pin.port = enable_port;
  port->enable_pin.pin = enable_pin;
  port->rx_head = 0;
  port->rx_tail = 0;

  tmc2209_stm32_dwt_enable();

  hal->serial_write = tmc2209_stm32_serial_write;
  hal->serial_available = tmc2209_stm32_serial_available;
  hal->serial_read = tmc2209_stm32_serial_read;
  hal->serial_flush = tmc2209_stm32_serial_flush;
  hal->delay_microseconds = tmc2209_stm32_delay_microseconds;
  hal->delay_milliseconds = tmc2209_stm32_delay_milliseconds;
  hal->set_hardware_enable_pin =
    (enable_port != NULL) ? tmc2209_stm32_set_hardware_enable_pin : NULL;
  hal->context = port;
}
