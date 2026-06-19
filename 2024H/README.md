# 2024H Auto Driving Car for MSPM0G3507

Firmware scaffold for the 2024 National Undergraduate Electronics Design Contest H problem: an automatic car controlled by a TI MSPM0G3507 MCU.

The project separates contest logic from board-specific code:

- `include/`: public module interfaces and tuning constants.
- `src/app/`: top-level state machine.
- `src/navigation/`: route definitions for requirements 1 to 4.
- `src/control/`: PID, line tracking, segment tracking, and motor mixing.
- `src/platform/`: MSPM0 hardware adaptation points.
- `src/system/`: minimal Cortex-M0+ startup/runtime files.
- `docs/`: problem notes and tuning guide.
- `linker/`: MSPM0G3507 128 KB Flash / 32 KB SRAM linker script.

The current platform layer is intentionally conservative: it compiles without a TI SDK and exposes the functions that must be connected to the real board peripherals.

## Expected Hardware

- TI MSPM0G3507 MCU, 80 MHz Cortex-M0+.
- Differential two-wheel drive, no crawler track and no mecanum wheel.
- LF04 four-channel digital reflective line sensor array. Black line outputs low level; white ground outputs high level.
- Wheel encoders for distance estimation.
- MPU6050 yaw integration for straight segment correction.
- LED and buzzer for waypoint prompts.
- Button or DIP switch for route selection.

## Route Modes

- Requirement 1: `A -> B`, stop at B.
- Requirement 2: `A -> B -> C -> D -> A`, one lap.
- Requirement 3: `A -> C -> B -> D -> A`, one lap.
- Requirement 4: Requirement 3 route, four laps.

## Porting Checklist

1. Update `include/app_config.h` for wheelbase, encoder scale, speed limits, and line threshold.
2. Implement the weak functions in `src/platform/platform_mspm0.c` using the actual MSPM0 pins and drivers.
3. Implement the `platform_hw_*` functions for GPIO, PWM, encoder interrupt capture, I2C, LED, and buzzer.
4. Select the real route mode with `platform_hw_route_switch_value()`.
5. Tune straight yaw PID first, then arc line PID, motor PI, and route speed.
