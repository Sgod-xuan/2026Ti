# Hardware Mapping

Target board: TI MSPM0G3507.

The old Arduino program uses Arduino-style pin names. This firmware keeps those as logical wiring names and routes actual MSPM0G3507 GPIO/PWM/I2C implementation through `platform_hw_*` functions in `src/platform/platform_mspm0.c`.

## Logical Wiring From Reference Program

Line sensor:

- LF04 S1 -> A0, left outer, black line reads LOW.
- LF04 S2 -> A1, left inner, black line reads LOW.
- LF04 S3 -> A2, right inner, black line reads LOW.
- LF04 S4 -> A3, right outer, black line reads LOW.

Motor driver:

- Left motor AIN1 -> D9 PWM.
- Left motor AIN2 -> D6 PWM or GPIO.
- Right motor AIN1 -> D10 PWM.
- Right motor AIN2 -> D5 PWM or GPIO.

Encoder:

- Left encoder A -> D7.
- Left encoder B -> D2 interrupt.
- Right encoder A -> D8.
- Right encoder B -> D3 interrupt.

Control and IMU:

- Reference start button -> D4, `INPUT_PULLUP`, pressed is LOW.
- C07A current start button -> onboard SW1 / PB8, pressed is LOW.
- MPU6050 SDA -> A4.
- MPU6050 SCL -> A5.
- MPU6050 address -> `0x68`.

Optional hardware kept for debugging only:

- HC-SR04 Trig -> D12.
- HC-SR04 Echo -> D13.
- HC-05 TX/RX -> D0/D1.

## Required MSPM0G3507 Adaptation Points

Implement these weak functions for the real board:

- `platform_hw_init()`: configure clock, GPIO, PWM timers, encoder interrupts, I2C, LED, and buzzer.
- `platform_hw_start_button_pressed()`: return true when the configured start button is pressed; the current C07A mapping uses onboard SW1 / PB8.
- `platform_hw_line_pin_is_high(index)`: return raw LF04 digital level; the platform layer maps LOW to black.
- `platform_hw_take_left_encoder_ticks()` and `platform_hw_take_right_encoder_ticks()`: atomically return and clear encoder counts accumulated during the last 10 ms control period.
- `platform_hw_set_left_pwm()` and `platform_hw_set_right_pwm()`: accept signed PWM after PI and dead-zone compensation.
- `platform_hw_i2c_write_reg()` and `platform_hw_i2c_read_regs()`: provide MPU6050 register access.
- `platform_hw_set_waypoint_signal()` and `platform_hw_set_done_signal()`: drive LED and buzzer.
