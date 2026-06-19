#ifndef PLATFORM_PLATFORM_HW_H
#define PLATFORM_PLATFORM_HW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void platform_hw_init(void);
bool platform_hw_start_button_pressed(void);
uint8_t platform_hw_route_switch_value(void);
bool platform_hw_line_pin_is_high(uint8_t index);
int platform_hw_take_left_encoder_ticks(void);
int platform_hw_take_right_encoder_ticks(void);
void platform_hw_set_left_pwm(int signed_pwm);
void platform_hw_set_right_pwm(int signed_pwm);
void platform_hw_set_waypoint_signal(bool enable);
void platform_hw_set_done_signal(bool enable);
bool platform_hw_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
bool platform_hw_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

#endif
