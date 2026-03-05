# Development (Pixi + PlatformIO)

This repository includes a `pixi.toml` so you can install the PlatformIO CLI in a
reproducible way and run builds/tests without managing a separate Python/venv.

## Setup

1. Install Pixi (see https://pixi.sh for installers).
2. From the repo root:

```bash
pixi install
```

## Common tasks

### Run host/native unit tests

```bash
pixi run test
```

### Build an example (compile only)

```bash
pixi run build --example examples/UnidirectionalCommunication/MoveAtVelocity --env teensy40
```

### Clean an example build

```bash
pixi run clean --example examples/UnidirectionalCommunication/MoveAtVelocity --env teensy40
```

### Upload an example

```bash
pixi run upload --example examples/UnidirectionalCommunication/MoveAtVelocity --env teensy40 --port /dev/ttyACM0
```

### List serial ports

```bash
pixi run ports
```

### Serial monitor

```bash
pixi run monitor --port /dev/ttyACM0 --baud 115200
```

## Notes

- The `tools/pio_task.py` wrapper sets `PLATFORMIO_SRC_DIR` and `PLATFORMIO_BUILD_DIR`
  so you can select which example to build without editing `platformio.ini`.
- Builds are isolated per example under `.pio/build/<example>/...` to reduce cross-example
  cache collisions.
