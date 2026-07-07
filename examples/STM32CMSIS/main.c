// ----------------------------------------------------------------------------
// main.c
//
// TMC2209 example for STM32 using only CMSIS (no STM32 HAL/LL drivers).
//
// Target used in this example: STM32F401/F411 (e.g. Black Pill board)
// - USART1 TX on PA9, RX on PA10, both wired to the TMC2209 UART pin through
//   a single 1k resistor on TX (see the TMC2209 datasheet and the wiring
//   diagrams in the images directory)
// - TMC2209 ENN input on PA8
//
// For other STM32 families only usart_init()/gpio_init() below need to be
// adapted; the library and the port in ports/stm32 stay unchanged.
//
// Build with, for example:
//   arm-none-eabi-gcc -std=c17 -mcpu=cortex-m4 -mthumb \
//     -DSTM32F401xC -DTMC2209_STM32_DEVICE_HEADER='"stm32f4xx.h"' \
//     -I<cmsis_device_f4>/Include -I<cmsis_core>/Include \
//     -I<lib>/src -I<lib>/ports/stm32 \
//     main.c <lib>/src/tmc2209.c <lib>/ports/stm32/tmc2209_stm32.c ...
// ----------------------------------------------------------------------------

#include "stm32f4xx.h"

#include "tmc2209.h"
#include "tmc2209_stm32.h"

#define SERIAL_BAUD_RATE 115200u
#define RUN_CURRENT_PERCENT 100u
#define RUN_VELOCITY 20000
#define STOP_VELOCITY 0
#define RUN_DURATION_MS 2000u
#define STOP_DURATION_MS 1000u

static tmc2209_t stepper_driver;
static tmc2209_stm32_t stepper_port;
static tmc2209_hal_t stepper_hal;

static void gpio_init(void)
{
  // enable GPIOA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // PA9 (TX) and PA10 (RX) alternate function 7 (USART1)
  GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
  GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
  GPIOA->AFR[1] |= (7u << GPIO_AFRH_AFSEL9_Pos) | (7u << GPIO_AFRH_AFSEL10_Pos);

  // PA8 push-pull output for the TMC2209 ENN input, start disabled (high)
  GPIOA->MODER &= ~GPIO_MODER_MODER8;
  GPIOA->MODER |= GPIO_MODER_MODER8_0;
  GPIOA->BSRR = GPIO_BSRR_BS8;
}

static void usart_init(void)
{
  // enable USART1 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // 8N1, oversampling by 16
  USART1->CR1 = 0;
  USART1->CR2 = 0;
  USART1->CR3 = 0;
  // SystemCoreClock is used here for simplicity; use the actual APB2 clock
  // frequency if a bus prescaler other than 1 is configured
  USART1->BRR = (uint16_t)((SystemCoreClock + (SERIAL_BAUD_RATE / 2u)) /
    SERIAL_BAUD_RATE);
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void delay_milliseconds(uint32_t milliseconds)
{
  stepper_hal.delay_milliseconds(stepper_hal.context, milliseconds);
}

int main(void)
{
  SystemCoreClockUpdate();

  gpio_init();
  usart_init();

  tmc2209_stm32_hal_init(&stepper_port, &stepper_hal, USART1, GPIOA, 8u);
  tmc2209_setup(&stepper_driver, &stepper_hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_set_run_current(&stepper_driver, RUN_CURRENT_PERCENT);
  tmc2209_enable_cool_step(&stepper_driver, 1u, 0u);
  tmc2209_enable(&stepper_driver);

  for (;;)
  {
    tmc2209_move_at_velocity(&stepper_driver, STOP_VELOCITY);
    delay_milliseconds(STOP_DURATION_MS);

    tmc2209_move_at_velocity(&stepper_driver, RUN_VELOCITY);
    delay_milliseconds(RUN_DURATION_MS);
  }
}
