# NUC communication protocol V3

UART4 runs at 460800 baud, 8-N-1. All integers and IEEE754 floats are little-endian. Every frame ends with RoboMaster CRC16 (initial value `0xffff`, reflected polynomial `0x8408`), stored low byte first.

V3 is additive: the current auto-aim keeps using the existing `SP` command/feedback pair while navigation and richer telemetry use separate headers. A receiver must scan the byte stream for a known header and must not assume one UART read equals one frame.

## Safety behavior

- An `SP` aim command expires 200 ms after the last valid frame. On expiry, target-present and fire request are cleared.
- An `SN` navigation command expires after its `timeout_ms` (default 250 ms, accepted range 50–1000 ms). On expiry, all navigation speeds and goal state are cleared.
- Bad CRC, NaN/Inf, invalid flags/ranges, and unknown versions never update actuator commands.
- `SN.flags bit1` is an emergency stop and takes precedence over enable.
- The lower board supplies ammunition only when a fresh `SP` command has both `control=1` and `shoot=1`. There is no artificial one-shot-per-second gate; a held valid request permits continuous fire subject to the existing heat/motor controls.

## Frames

The packed C definitions in `STM32F405/nuc.h` are authoritative and have compile-time size checks.

| Direction | Header | Type | Bytes | Rate/timeout | Purpose |
|---|---:|---:|---:|---:|---|
| NUC → MCU | `SP` | legacy | 14 | <=200 ms | aim target and explicit fire request |
| MCU → NUC | `SP` | legacy | 20 | 100 Hz | mode, gate bits, shooter RPM, IMU yaw/pitch |
| NUC → MCU | `SN` | 1 | 34 | 50–1000 ms | navigation velocity, goal state, acceleration limit |
| NUC → MCU | `SC` | 1 | 30 | event driven | tactical/referee commands reserved for decision layer |
| MCU → NUC | `SV` | 1 | 50 | 50 Hz | timestamped IMU angle/rate and chassis odometry estimate |
| MCU → NUC | `SV` | 2 | 66 | 10 Hz | referee, shooter, power, heat and RFID status |
| NUC → MCU | `NAVI` | legacy | 19 | 250 ms | old navigation compatibility; CRC is now mandatory |

### Legacy SP feedback gate bits

| Bit | Meaning |
|---:|---|
| 0 | operator auto-aim mode selected |
| 1 | lower-board control path available |
| 2 | auto-aim control context available |
| 3 | IMU values are finite |
| 4 | aim command is fresh |
| 5 | `control=1` requested |
| 6 | yaw control armed |
| 7 | pitch control armed |

The 20-byte legacy feedback keeps the diagnostic encoding expected by the current vision release: `feeder_rpm` is measured feeder RPM, and `friction_rpm_packed` contains left/right wheel magnitudes at 40 RPM per LSB.

## Navigation conventions

- `linear_x_mps`, `linear_y_mps`: chassis-frame m/s.
- `angular_z_radps`: rad/s in the existing lower-board sign convention.
- `max_accel_mps2`: optional linear acceleration slew limit; zero means no slew limit.
- `goal_status`: `-1` failed, `0` moving, `1` reached.
- `flags bit0`: enable; `flags bit1`: emergency stop.

The transmitted chassis `wz` currently preserves the former wheel-kinematic estimate. Measure and calibrate the chassis lever arm before using it as high-accuracy odometry.

## Building

The repository now has a reproducible GCC/CMake build in addition to the original VisualGDB project:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
python tools/protocol_v3_selftest.py
```

Build products are `build/sentry_uc.elf`, `build/sentry_uc.hex`, and `build/sentry_uc.bin`.
