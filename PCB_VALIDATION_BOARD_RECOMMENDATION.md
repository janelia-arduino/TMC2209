# TMC2209 + TMC429 Validation PCB Recommendation

This document captures the issue-driven PCB recommendation for a future
hardware validation board. It is intended as design input for a later KiCad and
BOM planning session, not as a final schematic or parts list.

## Background

The `TMC2209`, `TMC429`, and `TMC51X0` Arduino libraries are being reorganized
around a newer grouped API style. Before more code changes are made, the
`TMC2209` and `TMC429` libraries need a repeatable hardware bench that an agent
can use to develop and verify UART, SPI, STEP/DIR, StallGuard, current, and
motion behavior.

The local `.github-issues` dump for `TMC2209` was reviewed. It contains 96
issues, with 49 open at review time. Many reported library failures are really
hardware topology, power sequencing, UART line, pin mapping, or measurement
problems. The PCB should therefore be a failure reproduction and diagnostic
bench, not only a happy-path demo board.

## Issue Themes To Cover

Relevant issue clusters:

- Multi-driver single-UART failures: #32, #38, #45, #57, #94, #99
- UART wiring and module confusion: #49, #60, #61, #64, #87, #91, #101
- Power-cycle and recovery problems: #76, #85
- StallGuard and DIAG behavior: #5, #58, #103, #106
- INDEX and microstep behavior: #14, #63, #67, #95
- Current, VREF, Rsense, torque, and motor heating: #28, #37, #42, #47, #69,
  #93
- STEP/DIR plus UART integration: #52, #55, #73
- Platform UART differences: ESP32, RP2040, STM32, Uno R4, SAMD, and Linux CM4
  related issues

Main lesson: the board must support several UART topologies and enough
measurement points to distinguish library bugs from electrical behavior.

## Recommended Chip Count

Use footprints for:

- 2 TMC429 controllers
- 8 TMC2209 drivers

Suggested population options:

- Minimum useful population: 1 TMC429 + 4 TMC2209
- Standard validation population: 2 TMC429 + 6 TMC2209
- Extended failure-reproduction population: 2 TMC429 + 8 TMC2209

Rationale:

- One TMC429 controls exactly three STEP/DIR axes.
- Four TMC2209 drivers are needed to test the complete addressed UART space
  using `MS1/MS2` addresses 0 through 3.
- Several user issues involve more than four drivers, including six- and
  seven-driver systems.
- Two TMC429s cover six real controller axes.
- The extra TMC2209 footprints allow >4-driver unidirectional or muxed UART
  tests, and allow one or two drivers to be driven directly by the target MCU
  instead of through the TMC429.

## Recommended Physical Layout

Prefer a hybrid driver layout:

- Bank A: 4 direct onboard TMC2209 IC circuits with known Rsense, known passives,
  and controlled layout.
- Bank B: 4 optional TMC2209 circuits or StepStick-style module sockets.

The direct IC bank gives a clean reference design. The module/socket bank helps
reproduce common BIGTREETECH, FYSETC, MKS, Geeetech, and similar module wiring
issues from the GitHub reports.

If board area forces a choice, keep all 8 TMC2209 logical channels but allow
some channels to be unpopulated. The PCB should still route test headers and
bus switching for all 8 channels.

## Core UART Topologies

Each TMC2209 `PDN_UART` connection should be switchable into several modes:

1. Shared addressed bidirectional bus
   - Four drivers on one single-wire UART bus.
   - Addresses 0 through 3 via `MS1/MS2`.
   - Used to reproduce #38, #45, #57, #94, and #99.

2. Muxed one-at-a-time bus
   - One MCU UART connects to exactly one driver at a time through analog
     switches or a UART mux.
   - Used to compare against the shared-bus case and reproduce #32 style
     timing and mux-settle behavior.

3. Dedicated UART per driver or per small group
   - Used as the control case when shared bus behavior fails.

4. TX-only unidirectional programming
   - Used to match boards where only unidirectional UART is available.
   - Important for #72, #92, and software-serial style setups.

5. Disconnected / isolated
   - Each driver should be removable from the bus electrically without
     desoldering.

## UART Line Conditioning Options

Add selectable components around the UART line so failures can be reproduced
and isolated:

- Series resistor between target MCU TX and the single-wire UART node:
  - 0 ohm
  - 220 ohm
  - 650 ohm
  - 1k ohm
  - 2.2k ohm
- Pull options on shared bus and per-driver nodes:
  - no pull
  - 20k pulldown
  - 56k pullup
  - 10k pulldown
- Optional bus capacitance footprints for signal-integrity experiments.
- Optional stub-length jumpers or series damping resistors.
- Per-driver isolation switches or jumpers.
- Separate test points for:
  - target TX before coupling resistor
  - target RX
  - shared `PDN_UART` bus node
  - each selected driver `PDN_UART` pin
  - mux select signals

This directly supports debugging shallow logic-low levels, wrong resistor
placement, module RX/TX silkscreen confusion, and address-bus timing problems.

## TMC2209 Address Control

For each of the first four TMC2209 channels:

- Route `MS1` and `MS2` to selectable pulls or agent-MCU controlled outputs.
- Make address state observable at test points.
- Provide a fixed-address jumper option so hardware can be locked to known
  address states.

Address table:

- Address 0: `MS1=LOW`, `MS2=LOW`
- Address 1: `MS1=HIGH`, `MS2=LOW`
- Address 2: `MS1=LOW`, `MS2=HIGH`
- Address 3: `MS1=HIGH`, `MS2=HIGH`

The board should be able to test both static strap addressing and controlled
runtime address experiments.

## TMC429 Connections

Use two TMC429 chips with independent chip selects on a shared SPI bus.

Expose and instrument:

- `SCK`
- `MOSI`
- `MISO`
- `CS_TMC429_0`
- `CS_TMC429_1`
- TMC429 clock input
- `INT` from each TMC429
- `POSCOMP` from each TMC429
- `STEP0/DIR0`, `STEP1/DIR1`, `STEP2/DIR2` per TMC429
- left and right reference switch inputs per axis

Suggested routing:

- TMC429 0 axes 0-2 drive TMC2209 channels 0-2.
- TMC429 1 axes 0-2 drive TMC2209 channels 3-5.
- TMC2209 channels 6-7 can be driven by target-MCU STEP/DIR or left as
  UART-only/mux test channels.

## STEP/DIR Test Matrix

Each TMC2209 should be able to receive STEP/DIR from at least one source:

- TMC429 STEP/DIR
- target MCU GPIO STEP/DIR
- optional external header

Use jumpers or analog switches to choose source. This supports:

- UART setup followed by external STEP/DIR motion
- direct MCU-generated step timing
- TMC429-generated coordinated motion
- comparing AccelStepper/FastAccelStepper style firmware output against TMC429
  output

Expose selected `STEP`, `DIR`, and `ENN` signals to both the agent MCU and
ADALM2000 debug headers.

## StallGuard, DIAG, INDEX, And Limit Switches

For each TMC2209:

- Route `DIAG` to:
  - target MCU input option
  - agent MCU input option
  - ADALM2000 test point
  - optional pull-up/pull-down footprints
- Route `INDEX` to:
  - target MCU input option
  - agent MCU input option
  - ADALM2000 test point
  - optional signal-conditioning footprint
- Route `ENN` to:
  - target MCU output
  - agent MCU override path
  - manual jumper
  - ADALM2000 test point

For each TMC429 axis:

- Provide left and right reference switch inputs.
- Each reference input should support:
  - real switch connector
  - agent-MCU driven simulated switch
  - pull-up/pull-down selection
  - ADALM2000 test point

Add at least one connector for a real mechanical fixture:

- small linear stage with limit switches
- hard-stop fixture for StallGuard tests
- brake/load fixture
- externally mounted stepper with safe guarded travel

StallGuard and DIAG behavior cannot be fully validated with logic-only tests.

## Power, Reset, And Fault-Recovery Features

Add agent-controlled power switching for:

- TMC2209 `VM`, per driver or per bank
- TMC2209 `VIO`, per bank and ideally per selected driver
- TMC429 logic power or reset
- target MCU reset and boot control

Add measurement and state outputs:

- `VM_PGOOD`
- `VIO_PGOOD`
- TMC429 logic power-good if available
- switched-current monitor per bank
- selected driver current monitor

Add safe power handling:

- fused or current-limited motor input
- reverse polarity protection or keyed connector
- motor supply cutoff controlled by agent MCU
- emergency stop input
- discharge path or bleeder for motor supply rail after cutoff
- temperature sensors near driver banks

Several issues only reproduce after motor power removal, motor power return, or
driver reset while the MCU remains alive.

## Current, VREF, And Rsense Validation

Use known current-sense resistor values and make them easy to inspect:

- Preferred default: 0.11 ohm external Rsense, accurate part, Kelvin routing.
- Consider one or two configurable channels with alternate Rsense footprints:
  - 0.075 ohm
  - 0.11 ohm
  - 0.15 ohm

Provide:

- VREF generation option, if analog current scaling is to be tested.
- Potentiometer or DAC-controlled VREF option.
- VREF test point and analog mux input.
- Phase-current sense amplifier outputs for at least one selected driver.
- Motor supply current measurement per bank.
- VM and VIO voltage dividers into agent MCU ADC and ADALM analog header.

The ADALM2000 analog inputs should not connect directly to motor phase outputs.
Use current-sense amplifier outputs or properly protected attenuated nodes.

## ADALM2000 Debug Headers

The ADALM2000 has limited simultaneous digital channels, so expose more signals
than can be captured at once. Use grouped headers or a digital mux.

Recommended UART group:

- target TX before coupling resistor
- target RX
- shared `PDN_UART`
- selected driver `PDN_UART`
- UART mux select lines
- `MS1`
- `MS2`
- `ENN`
- `DIAG`
- `INDEX`
- `VM_PGOOD`
- `VIO_PGOOD`
- agent marker GPIO

Recommended TMC429 group:

- `SCK`
- `MOSI`
- `MISO`
- `CS_TMC429_0`
- `CS_TMC429_1`
- `INT`
- `POSCOMP`
- selected axis `STEP`
- selected axis `DIR`
- selected left reference switch
- selected right reference switch
- agent marker GPIO

Recommended motion/stall group:

- selected driver `STEP`
- selected driver `DIR`
- selected driver `ENN`
- selected driver `DIAG`
- selected driver `INDEX`
- encoder A if available
- encoder B if available
- left limit switch
- right limit switch
- agent marker GPIO

Recommended analog group through mux or jumpers:

- selected phase A current-sense amplifier output
- selected phase B current-sense amplifier output
- VM supply current
- VREF
- VM divided voltage
- VIO divided voltage
- UART bus analog voltage
- optional temperature sensor output

Add ground pins adjacent to every debug signal group.

## Agent MCU Requirements

The agent-accessible MCU should provide:

- debug/program connection to target MCU
- reset and boot control for target MCU
- at least two UART links to target MCU:
  - command/control
  - logs/traces
- control of power switches
- control of UART mux/isolation hardware
- digital observation of selected fault/status pins
- ADC inputs for rail/current/temperature monitoring
- marker GPIOs routed to ADALM2000 headers

The target MCU should run the Arduino libraries under test. The agent MCU should
be able to reset, reflash, power-cycle, and observe the target system.

## Target MCU Strategy

Start with an RP2040-family target because many multi-UART issues involve
RP2040/Pico-style hardware.

The board should also make room for later target-MCU adapter options:

- ESP32 or ESP32-S3
- STM32
- Arduino Uno R4 / Renesas
- SAMD
- Raspberry Pi CM4 or Linux UART adapter, if Linux-port work is in scope

Prefer a target-MCU mezzanine or adapter connector if practical, rather than
hardwiring the design to one MCU family.

## KiCad Library And Purchasing Preparation

Future sessions should choose exact parts and add KiCad symbols/footprints for:

- TMC2209 IC
- TMC429 IC
- target MCU module or target MCU adapter connector
- agent MCU module or agent MCU circuit
- analog switches / muxes for UART bus selection
- optional digital mux or crosspoint for debug signal selection
- load switches for VM and VIO
- high-side or low-side current-sense amplifiers
- precision current-sense resistors
- VREF DAC or trimmer/potentiometer circuit
- rail voltage dividers and protection parts
- temperature sensors
- stepper motor connectors
- limit switch connectors
- encoder connector, optional
- ADALM2000-compatible debug headers
- SWD/JTAG/programming headers
- USB/UART debug connectors
- jumpers or DIP switches for address and topology selection
- protection parts for analyzer-facing and MCU-facing lines
- fuses/polyfuses and power-input protection
- motor supply connector and emergency-stop connector

Exact part selection is intentionally left open.

## First Validation Tests The PCB Should Enable

The board should make these tests straightforward:

1. Single TMC2209 UART smoke test on a direct IC channel.
2. Single TMC2209 UART smoke test on a module/socket channel.
3. Four addressed TMC2209 drivers on one shared UART bus.
4. Same four drivers through muxed one-at-a-time UART.
5. Reply-delay sweep at several baud rates.
6. Power-cycle VM while target MCU remains alive.
7. Power-cycle VIO while target MCU remains alive.
8. UART setup followed by STEP/DIR motion from target MCU.
9. UART setup followed by STEP/DIR motion from TMC429.
10. TMC429 SPI communication with one chip, then two chips.
11. TMC429 three-axis motion on one controller.
12. Six-axis motion across two TMC429 controllers.
13. StallGuard SG_RESULT polling under controlled load.
14. DIAG pulse capture with ADALM2000 and agent MCU.
15. INDEX behavior across microstep settings and internal pulse generator modes.
16. Current setting validation against measured phase current.
17. VREF analog scaling validation.
18. Limit/reference switch stop and latch behavior on TMC429.
19. Step pulse timing and max-rate experiments for low microstep counts.
20. Recovery behavior after driver error or stall.

## Main Design Principle

The most important PCB feature is not the exact number of motors. It is the
ability to switch each driver among shared bus, muxed bus, dedicated UART, and
TX-only UART modes while measuring the actual electrical signals.

That single capability addresses the largest fraction of the issue history and
will help separate library bugs from board-level behavior.
