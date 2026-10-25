# airio firmware

Zephyr firmware for airio, the small BLE vario on a Seeed XIAO nRF52840.
Parts and pin map: [docs/airio-parts-and-pinmap.md](../../docs/airio-parts-and-pinmap.md).

This is separate from the ESP-IDF firmware for Flario Rev A in `main/`.

## What v0 does

- Reads the BMP581 at 50 Hz over I2C.
- Runs a two-state Kalman filter (altitude, climb rate) on baro altitude
  ([vario.c](app/src/vario.c)).
- Maps climb rate to the beeper ([tone.c](app/src/tone.c)): beeps from
  +0.1 m/s, rising in pitch and rate with climb; a continuous low tone from
  -2.0 m/s.
- Drives the piezo differentially from two PWM channels on D2/D3.
- Brings up the LSM6DSV16X and logs acceleration once a second.

Not yet: IMU fusion into the filter, BLE, battery monitoring, button
handling, power management.

## Layout

| Path | What |
|---|---|
| `west.yml` | West manifest: Zephyr v4.4.2 plus the cmsis_6, hal_nordic and hal_st modules. |
| `app/` | The application. `boards/xiao_ble.overlay` wires the carrier board. |
| `app/src/vario.c`, `tone.c` | Pure C, no Zephyr calls, so they can be tested and replayed on a host. |
| `app/src/baro_sim.c` | Synthetic flight used on `native_sim` in place of the BMP581. |
| `tests/vario/` | ztest suite for the filter and tone mapping, run on `native_sim`. |

## Setup

Needs the [Zephyr SDK 1.0.1](https://github.com/zephyrproject-rtos/sdk-ng/releases/tag/v1.0.1)
with at least the `arm-zephyr-eabi` toolchain, and Python 3.10+.

```sh
python3 -m venv ~/.venv/zephyr && . ~/.venv/zephyr/bin/activate
pip install west

# The workspace is the folder that contains your flario checkout.
cd <folder containing flario>
west init -l --mf airio/firmware/west.yml flario
west update --narrow -o=--depth=1
pip install -r zephyr/scripts/requirements-base.txt
```

## Build

From the workspace folder:

```sh
# Hardware: produces build-xiao/zephyr/zephyr.uf2
west build -b xiao_ble -d build-xiao flario/airio/firmware/app

# Simulation on Linux: runs the full app against a synthetic flight
west build -b native_sim/native/64 -d build-sim flario/airio/firmware/app
build-sim/zephyr/zephyr.exe -stop_at=80

# Unit tests
west build -b native_sim/native/64 -d build-test flario/airio/firmware/tests/vario
build-test/zephyr/zephyr.exe
```

## Flash

Double-tap the XIAO's reset button; a `XIAO BLE` drive appears. Copy
`build-xiao/zephyr/zephyr.uf2` onto it. Logs come out on the USB serial port
at 115200 baud.

For debugging, wire a J-Link to the SWD pads and use `west flash` /
`west debug`.

## Notes

- `CONFIG_SENSOR_ASYNC_API=y` in `app/boards/xiao_ble.conf` works around a
  build failure in Zephyr v4.4.2's LSM6DSV16X driver; see the comment there.
- The `bmp581` driver accepts chip IDs 0x50 and 0x51, so a BMP580 should run
  unchanged.
