# TMC2209 Session Handoff (2026-03-25)

## Current Status

This repo now has a partial `TMC51X0`-style compatibility layer landed in
`TMC2209`.

Implemented:

- family-style async access on `TMC2209`
  - `done()`
  - `uartBus()`
  - `device()`
- recovery / health API on `TMC2209`
  - `HealthStatus`
  - `notePossibleMirrorDrift()`
  - `mirrorResyncRequired()`
  - `reinitialize()`
  - `recoverFromDeviceReset()`
  - `recoverIfNeeded()`
  - `recoverIfUnhealthy()`
  - `resyncReadableConfiguration()`
- opt-in UART write verification
  - `UartParameters::verify_writes`
  - `TMC2209::enableWriteVerification()`
  - `TMC2209::disableWriteVerification()`
  - `TMC2209::writeVerificationEnabled()`

Primary files:

- `src/TMC2209.h`
- `src/TMC2209/TMC2209.cpp`
- `src/UartParameters.hpp`
- `src/UartBus.hpp`
- `src/TMC2209/UartBus.cpp`
- `src/Device.hpp`
- `src/TMC2209/Device.cpp`

## Tests

Host-side regression coverage was added for:

- family-style async aliases
- recovery helpers
- cached configuration replay
- `IFCNT` write verification success/failure

Main test files:

- `test/test_native/test_main.cpp`
- `test/include/FakeSerial.hpp`

Direct native build against live repo sources passes:

```sh
g++ -std=gnu++17 -Isrc -Itest/include -I.pio/libdeps/native/Unity/src test/test_native/test_main.cpp src/TMC2209/*.cpp .pio/libdeps/native/Unity/src/unity.c -o /tmp/tmc2209_native_tests && /tmp/tmc2209_native_tests
```

Status at handoff:

- `44 Tests 0 Failures 0 Ignored`

## PlatformIO Quirk

PlatformIO can link stale copied local libraries from `.pio/libdeps/...`
instead of the live repo sources.

Symptoms seen:

- newly added methods present in headers but missing at link time
- examples building against older cached `TMC2209.cpp`

Workarounds used:

- use the direct `g++` command above for native validation
- if example upload links stale code, remove cached local package first:

```sh
pio pkg uninstall -e teensy40 --library TMC2209
```

## Hardware Setup Used

- Teensy 4.0
- TMC2209 connected on `Serial3`
- USB serial port on host re-enumerates after upload as `/dev/ttyACM*`

## Hardware Findings

### Read path works

Confirmed on real hardware:

- `getVersion()` returns `0x21`
- blocking reads work
- facade async reads work
- `uartBus()` async reads work

Bench sketch used:

- `examples/BidirectionalCommunication/UartCompatibilitySmoke/UartCompatibilitySmoke.ino`

Representative output:

```text
[version] 21
[blocking read] ok value=0x21000040
[facade async start] ok value=0x21000040
[bus async start] ok value=0x21000040
```

### `GSTAT.reset` behavior does not match assumptions

Observed repeatedly on real hardware:

```text
[gstat before clear] 0x1
[gstat after clear] 0x1
```

I briefly tried a `TMC51X0`-style read/clear behavior for `GSTAT` and then
backed it out after hardware showed that `reset` was not clearing as assumed.

Current library behavior intentionally does **not** force special `GSTAT`
clear-on-read semantics.

### Runtime writes are not taking effect in current hardware setup

This is the current blocker.

Latest smoke sketch tests:

- plain blocking `writeRegister()` to `CHOPCONF`
- opt-in verified write to `CHOPCONF`
- `IFCNT` before/after
- `CHOPCONF` readback before/after

Observed repeatedly on real hardware:

```text
[ifcnt before verified write] 183
[chopconf before write] 0x10010050
[manual write] ok
[ifcnt after manual write] 183
[chopconf after manual write] 0x10010050
[verified write] error=8
[ifcnt after verified write] 183
[chopconf after write] 0x10010050
```

Interpretation:

- read path is real and working
- plain write path reports transport success but hardware state does not change
- `IFCNT` does not increment
- `WriteVerifyFailed` is correctly catching that when verification is enabled

This suggests the current issue is not library shape anymore. It is likely
hardware / electrical / protocol acceptance of write datagrams in the current
setup.

## Datasheet Conclusions

Used local datasheet:

- `datasheet/TMC2209_datasheet_rev1.09.pdf`

Important conclusions:

- `GSTAT` is documented as write-1-to-clear and described as status since last
  read access, but real hardware did not match the expected clear behavior.
- `IFCNT` is documented to increment on each accepted UART write.
- `IHOLD_IRUN` is write-only, so it was a bad readback target.
- `CHOPCONF` is readable and was used as the better read-write validation
  target.

## Most Relevant Files At Handoff

- `src/TMC2209.h`
- `src/TMC2209/TMC2209.cpp`
- `src/UartParameters.hpp`
- `src/UartBus.hpp`
- `src/TMC2209/UartBus.cpp`
- `src/Device.hpp`
- `src/TMC2209/Device.cpp`
- `test/test_native/test_main.cpp`
- `test/include/FakeSerial.hpp`
- `examples/BidirectionalCommunication/UartCompatibilitySmoke/UartCompatibilitySmoke.ino`

## What Still Needs To Be Done

The next task should be bench debugging with the scope, not more API expansion.

Recommended next steps:

1. Capture the UART line during a successful read request/reply.
2. Capture the UART line during a plain `CHOPCONF` write.
3. Capture the UART line during a verified `CHOPCONF` write.
4. Confirm exact Teensy `Serial3` TX/RX wiring and coupling topology.
5. Confirm any resistor between TX and RX and any PDN_UART coupling details.
6. Confirm common ground and module variant / board straps.
7. Compare on-wire write datagrams to expected TMC2209 UART frames.
8. Once write acceptance is understood, decide whether `IFCNT` verification
   should stay opt-in for all writes or become a whitelist-based feature.

## Suggested Prompt For A New Codex Session

```text
Continue debugging the TMC2209 write path on real hardware.

Context:
- Repo: /home/peter/Repositories/arduino/TMC2209
- Read SESSION_HANDOFF_2026-03-25.md first
- Teensy 4.0 connected to TMC2209 on Serial3
- USB serial port changes after upload: /dev/ttyACM*
- Reads work reliably
- Writes appear not to take effect on hardware
- IFCNT does not increment for tested CHOPCONF writes
- CHOPCONF readback does not change after write
- Bench sketch is examples/BidirectionalCommunication/UartCompatibilitySmoke/UartCompatibilitySmoke.ino
- Direct native tests pass with:
  g++ -std=gnu++17 -Isrc -Itest/include -I.pio/libdeps/native/Unity/src test/test_native/test_main.cpp src/TMC2209/*.cpp .pio/libdeps/native/Unity/src/unity.c -o /tmp/tmc2209_native_tests && /tmp/tmc2209_native_tests
- PlatformIO may use stale copied local libs; may need:
  pio pkg uninstall -e teensy40 --library TMC2209

Please inspect the smoke sketch, summarize the expected UART write datagrams,
and help interpret scope captures for read vs write traffic.
```
