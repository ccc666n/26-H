# 2026 Electronic Design Competition H: Vehicle-Mounted Ball Balancing Motion Control System

[中文](README.md) | [English](README_EN.md)

> 2026 National Undergraduate Electronic Design Competition, Guangdong Division
> Problem: Vehicle-Mounted Ball Balancing Motion Control System (Problem H)
> Result: Provincial First Prize

This repository preserves the currently shareable firmware and competition documents for the project. It is intended for technical exchange and reproduction reference. It is not a resume or a recruiting profile.

## Overview

This project combines a line-following mobile chassis, vision-based ball localization, and closed-loop ball-track tilt control:

- The chassis is built around an `MSPM0G3507`, an 8-channel grayscale sensor, dual GMR encoders, MG513 DC gear motors, and a TB6612 motor driver.
- A MaixCAM2 vision unit detects the ball and sends its position and velocity to the MCU over UART.
- The MCU runs a fixed-period predictive-error PID controller and commands an X42S closed-loop stepper driver to adjust the ball-track tilt.
- An OLED and keys provide mode selection, status display, and timing.

The design report records line following, fixed-point stopping, static ball positioning, short-distance dynamic balancing, full-lap balancing, and specified-position balancing tests.

## Repository Layout

```text
.
|-- docs/
|   |-- design-report.pdf
|   `-- problem-statement.pdf
|-- firmware/
|   `-- mspm0g3507-ccs/
|       |-- AnalogGray/       # Grayscale acquisition and calibration
|       |-- Camera/           # UART vision-frame receiver and validator
|       |-- Encoder/          # Dual-wheel encoder counting and speed sampling
|       |-- GetYaw/           # IMU yaw update and zeroing
|       |-- ICM42688/         # ICM42688 driver
|       |-- IMU/              # Attitude estimation
|       |-- Motor/            # TB6612 motor control
|       |-- Stepper/          # X42S closed-loop stepper commands
|       |-- Track/            # Line following and task state machines
|       |-- empty.c           # Firmware entry point
|       |-- empty.syscfg      # SysConfig source
|       `-- targetConfigs/    # CCS target configuration
|-- .gitattributes
|-- .gitignore
`-- THIRD_PARTY_NOTICES.md
```

## System Architecture

```text
8-channel grayscale sensor ─┐
Dual GMR encoders ──────────┼─> MSPM0G3507 ─> TB6612 ─> MG513 chassis motors
ICM42688 IMU ───────────────┤
MaixCAM2 UART ──────────────┤
                            └──────────────> X42S ─> stepper motor ─> ball-track tilt
```

The firmware combines a main-loop scheduler with periodic control tasks:

- `TIMER_0` updates the IMU attitude path.
- `TIMER_ENCODER` samples dual-wheel encoder counts every 50 ms.
- `Track` implements task state machines, line following, speed feedback, segment transitions, and stopping logic.
- `Camera` validates and parses UART vision frames.
- `Stepper` converts target tilt angles into X42S position commands.

## Control Design

### Line following and speed feedback

The 8 grayscale channels estimate lateral deviation from the black track. That deviation generates differential wheel-speed targets. Dual GMR encoders provide speed and accumulated-distance feedback for speed control, staged deceleration, and stopping decisions.

### Predictive-error PID for ball control

The design report describes a 20 ms ball-control period and a predictive error composed of position error plus velocity feedforward:

```text
predictive_error = position_error + kv * ball_speed
```

The controller maps predictive error to a ball-track tilt command. The report records a `+/-50 degrees` command limit and a zero-angle fallback when vision loses the ball.

### Vision UART protocol

The current MSPM0 firmware in `Camera/camera.c` validates an XOR checksum and expects:

```text
$BALL,<valid>,<position_error_cm>,<absolute_position_cm>,<speed_cm_s>,<confidence>,<timestamp_ms>*<checksum>\n
```

`valid` is nonzero when the frame contains a ball detection. The receiver stores position error, absolute position, speed, confidence, and timestamp for the control task.

## Results Reported for the Competition

| Test item | Reported result |
| --- | --- |
| One-lap line following and stop | 18.35 s average; 1.7 cm maximum stop deviation |
| Static ball positioning | 3.83 s maximum completion time; 0.9 cm maximum error |
| AB-segment dynamic balancing | 6.91 s maximum completion time |
| Full-lap center-position balancing | 24.90 s maximum completion time |
| Full-lap specified-position balancing | 25.12 s maximum completion time; 0.9 cm maximum error |

The full problem statement, test records, system diagrams, and source excerpts are in [docs/problem-statement.pdf](docs/problem-statement.pdf) and [docs/design-report.pdf](docs/design-report.pdf).

## Build and Flash

This is a Code Composer Studio (CCS) project for the `MSPM0G3507`.

1. Install CCS and the TI MSPM0 SDK.
2. `empty.syscfg` records `mspm0_sdk@2.11.00.07` and SysConfig `1.26.2+4477`. Use this combination first when reproducing the competition build.
3. Import `firmware/mspm0g3507-ccs` as a project in CCS.
4. If CCS requests a target configuration, select `targetConfigs/MSPM0G3507.ccxml`.
5. Open `empty.syscfg`, confirm the LQFP-64 MSPM0G3507 target, then build the `Debug` configuration.
6. Connect compatible MSPM0G3507 debug hardware through SWD and flash from CCS.

`Debug/` and other build products are intentionally excluded from version control; CCS regenerates them after importing the project.

## Current Public Scope

Included:

- MSPM0G3507 CCS firmware snapshot
- SysConfig and CCS target configuration
- Design report
- Original H-problem statement

Not included yet:

- MaixCAM2-side Python source
- YOLO model and training assets
- WebRTC configuration
- Hardware schematics, PCB files, and mechanical drawings
- Physical-device photos, test curves, and demonstration videos

The vision-side example in the report appendix and the current `Camera/camera.c` parser do not use exactly the same number of UART fields. The protocol will be reconciled when the MaixCAM2 source is added, before claiming a complete end-to-end reproduction path.

## Licensing and Third-Party Code

No repository-wide license is declared yet. The current snapshot includes files with third-party copyright notices, including TI copyright and redistribution terms. Read [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before reuse, redistribution, or relicensing.

After auditing each module's provenance, original code and third-party components will be documented and licensed separately.
