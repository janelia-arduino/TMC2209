# TMC2209 C Library

A portable **C17** driver library for the Trinamic **TMC2209** ultra-silent
stepper motor driver IC, with UART serial control. The library has **no
platform dependencies** — all hardware access goes through a small,
user-supplied hardware abstraction layer (HAL) — which makes it directly
usable on **STM32 microcontrollers with plain CMSIS** (no STM32 HAL/LL
required), as well as on any other MCU or RTOS.

Ported to C from the excellent Arduino C++ library
[janelia-arduino/TMC2209](https://github.com/janelia-arduino/TMC2209) by
Peter Polidoro.

- **Language:** C17 (compiles cleanly with `-std=c17 -Wall -Wextra -Wpedantic -Werror`)
- **Dependencies:** none (only `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`)
- **License:** BSD

## Repository layout

| Path | Content |
|---|---|
| `src/tmc2209.h`, `src/tmc2209.c` | Portable, platform-independent core library |
| `ports/stm32/` | Ready-made HAL implementation for STM32 using only CMSIS |
| `examples/STM32CMSIS/` | Complete bare-metal STM32 example (`main.c`) |
| `tests/` | Host unit tests with a mocked TMC2209 (`make test`) |
| `datasheet/` | TMC2209 datasheets |
| `images/` | Wiring diagrams |

## Communication

The TMC2209 is controlled over a half-duplex, single-wire UART (8N1). TX and
RX of the microcontroller UART are both connected to the TMC2209 UART pin,
with a ~1 kΩ resistor between TX and the pin (see the wiring diagrams in
`images/`). Because of the single-wire bus, every transmitted byte is echoed
back on RX; the library handles discarding the echo.

- **Unidirectional (TX only):** write settings, no feedback.
- **Bidirectional (TX + RX):** write settings and read back status,
  diagnostics, and configuration (recommended).

Up to four TMC2209s can share one UART when their MS1/MS2 pins select
distinct serial addresses (`TMC2209_SERIAL_ADDRESS_0` … `_3`).

## Using the library

### 1. Provide a HAL

Fill a `tmc2209_hal_t` with function pointers for your platform:

```c
typedef struct tmc2209_hal
{
  void (*serial_write)(void * context, uint8_t const * data, size_t size);
  size_t (*serial_available)(void * context);
  int16_t (*serial_read)(void * context);          // -1 when empty
  void (*serial_flush)(void * context);            // wait until TX done
  void (*delay_microseconds)(void * context, uint32_t microseconds);
  void (*delay_milliseconds)(void * context, uint32_t milliseconds);
  void (*set_hardware_enable_pin)(void * context, bool enable); // optional
  void * context;
} tmc2209_hal_t;
```

Configure the UART peripheral (baud rate, 8N1) yourself before calling
`tmc2209_setup()`; the library never touches peripheral configuration.
115200 baud is a good default and requires no configuration on the TMC2209
side.

### 2. Set up and drive the motor

```c
#include "tmc2209.h"

tmc2209_t stepper;
tmc2209_hal_t hal = { /* your callbacks */ };

tmc2209_setup(&stepper, &hal, TMC2209_SERIAL_ADDRESS_0);

tmc2209_set_run_current(&stepper, 100);       // percent
tmc2209_enable(&stepper);
tmc2209_move_at_velocity(&stepper, 20000);    // microsteps per period

if (tmc2209_is_setup_and_communicating(&stepper))
{
  tmc2209_status_t status = tmc2209_get_status(&stepper);
  uint16_t sg = tmc2209_get_stall_guard_result(&stepper);
}
```

The full API mirrors the original Arduino library — microstepping, run/hold
current, StealthChop, SpreadCycle, CoolStep, StallGuard, standstill modes,
automatic current scaling/gradient adaptation, status and diagnostics — as
snake_case functions taking the `tmc2209_t *` instance first. See
`src/tmc2209.h` for the complete, documented list.

## STM32 with CMSIS

The port in `ports/stm32/` implements the HAL with direct USART register
access and supports both STM32 USART generations (`SR`/`DR` devices such as
F1/F2/F4/L1 and `ISR`/`TDR`/`RDR` devices such as F0/F3/F7/G0/G4/L0/L4/H7/U5,
including FIFO variants). Microsecond delays use the DWT cycle counter on
Cortex-M3+, with a busy-loop fallback on Cortex-M0/M0+.

```c
#include "tmc2209.h"
#include "tmc2209_stm32.h"

static tmc2209_t stepper;
static tmc2209_stm32_t port;
static tmc2209_hal_t hal;

int main(void)
{
  SystemCoreClockUpdate();
  // ...configure GPIO alternate functions and the USART (8N1, 115200)...

  // USART1 wired to the TMC2209, PA8 wired to ENN (pass NULL, 0 if unused)
  tmc2209_stm32_hal_init(&port, &hal, USART1, GPIOA, 8);
  tmc2209_setup(&stepper, &hal, TMC2209_SERIAL_ADDRESS_0);

  tmc2209_set_run_current(&stepper, 100);
  tmc2209_enable(&stepper);
  tmc2209_move_at_velocity(&stepper, 20000);
  for (;;) {}
}
```

Compile the port with your CMSIS device header, e.g.:

```sh
arm-none-eabi-gcc -std=c17 -mcpu=cortex-m4 -mthumb -DSTM32F401xC \
  -DTMC2209_STM32_DEVICE_HEADER='"stm32f4xx.h"' \
  -I<cmsis_device>/Include -I<cmsis_core>/Include \
  -Isrc -Iports/stm32 \
  src/tmc2209.c ports/stm32/tmc2209_stm32.c examples/STM32CMSIS/main.c ...
```

A complete example is in `examples/STM32CMSIS/main.c`. The port also works
unchanged in STM32CubeIDE/CubeMX projects — just add the sources, define
`TMC2209_STM32_DEVICE_HEADER`, and configure the UART with CubeMX (the port
only requires that the peripheral is enabled and configured 8N1).

## Building

### CMake

```sh
cmake -B build && cmake --build build && ctest --test-dir build
```

Add `-DTMC2209_BUILD_STM32_PORT=ON` when cross-compiling for STM32. The
project can also be consumed with `add_subdirectory()` (target `tmc2209`) or
as an ESP-IDF component.

### Make

```sh
make        # builds build/libtmc2209.a
make test   # builds and runs the host unit tests
```

Or simply drop `src/tmc2209.c` and `src/tmc2209.h` into your project — the
core library is two files with no dependencies.

## Tests

`tests/test_tmc2209.c` runs on the host against a mock HAL that simulates
the TMC2209 register file, the single-wire echo, and reply datagrams. It
verifies datagram framing and byte order, the datasheet CRC8 algorithm,
register encoding/decoding, CRC-failure retries, and the current/microstep
conversion logic.

## Hardware documentation

- [TMC2209 datasheets](./datasheet) (also covers the SilentStepStick breakout)
- [Trinamic TMC2209-LA product page](https://www.trinamic.com/products/integrated-circuits/details/tmc2209-la)
- [Watterott SilentStepStick documentation](https://learn.watterott.com/silentstepstick/)
- Wiring diagrams in [images/](./images)

<img src="./images/trinamic_wiring-TMC2209-description.svg" width="1920px">
