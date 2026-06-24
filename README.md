# TestQuad AHRS Library

## Purpose

Provides reusable attitude-estimation primitives: shared AHRS data structures, a Mahony quaternion estimator, a Madgwick estimator, and a small roll/pitch EKF used for bench comparison and estimator experiments.

## Files

- `AHRSCommon.h`: Shared units, constants, input/output structures, and helper functions.
- `MahonyAHRS.h/.cpp`: Quaternion Mahony complementary filter using gyro, accelerometer, and optional magnetometer.
- `MadgwickAHRS.h/.cpp`: Madgwick-style gradient-descent AHRS implementation.
- `RollPitchEKF.h/.cpp`: Small four-state roll/pitch estimator with gyro-bias tracking.
- `library.properties`: Arduino library metadata.

## Quick Start

```cpp
#include "MahonyAHRS.h"

MahonyAHRS ahrs;

void setup() {
    ahrs.setGains(1.0f, 0.005f);
}

void loop() {
    AHRSInput in{};
    in.ax_g = 0.0f; in.ay_g = 0.0f; in.az_g = 1.0f;
    in.gx_dps = 0.0f; in.gy_dps = 0.0f; in.gz_dps = 0.0f;
    in.magValid = false;

    AttitudeEstimate out{};
    ahrs.update(in, 0.0025f, out);
}
```

## How It Fits Into The Flight Controller

This library lives under `Submodules/AHRS` in the main `Test_Quad` firmware
and is built as an Arduino library by adding `Submodules/` to the Arduino
library search path. The main firmware includes it directly from
`RC_FlightController.ino` or from another support module.

The flight controller runs a 400 Hz control loop on ESP32, so this library
should avoid heap allocation, long blocking calls, and unbounded Serial output
inside flight-critical paths. Debug output should use `DebugConfig.h` macros
where available so `VERBOSE_ON=0` builds can compile prints out.

## Data Type Choices

- `float`: ESP32 hardware and Arduino math functions are efficient with 32-bit floats; attitude math does not need double precision at 400 Hz.
- `AHRSInput`: Groups sensor values with explicit units, preventing accidental mixing of g, degrees/second, and microtesla.
- `AttitudeEstimate`: Carries both Euler angles for telemetry/control and quaternion terms for filters that need continuous orientation state.
- `bool magValid`: Separates 6-DOF and 9-DOF operation because this quad often runs without a trustworthy AK8963 magnetometer.

## Usage Guidance

1. Initialize hardware-facing classes once during `setup()`.
2. Keep update/read calls deterministic when used from a FreeRTOS task.
3. Prefer explicit validity flags over sentinel numeric values.
4. Keep units visible in field names, such as `_dps`, `_g`, `_uT`, `_m`, or `_us`.
5. When adding telemetry fields, update both the packet struct and JSON serializer.

## Example Build Integration

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32:UploadSpeed=921600,CPUFreq=240,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=min_spiffs,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default \
  --libraries ./Submodules \
  .
```

For quiet flight builds:

```bash
arduino-cli compile ... --build-property compiler.cpp.extra_flags=-DVERBOSE_ON=0
```


## Integration Notes

In the main flight-controller sketch, this library is included through Arduino's
library search path. When this folder is converted to a git submodule, keep the
folder name stable under `Submodules/` so includes such as `#include "..."`
continue to resolve.

Most examples below are intentionally small. On the real flight controller,
objects are usually constructed globally, initialized once from `setup()`, and
then called from FreeRTOS tasks at deterministic rates.

