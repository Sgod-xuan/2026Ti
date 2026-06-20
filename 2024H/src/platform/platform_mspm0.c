#include "platform/platform.h"

#include <stddef.h>

#include "control/motor_pi.h"
#include "platform/c07a_pinmap.h"
#include "platform/platform_hw.h"

#define SYST_CSR (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018u)
#define SYST_ENABLE 1u
#define SYST_TICKINT 2u
#define SYST_CLKSOURCE 4u
#define WEAK __attribute__((weak))

static volatile uint32_t g_ms_ticks;
static MotorPi g_left_motor;
static MotorPi g_right_motor;
static float g_left_target_cm_s;
static float g_right_target_cm_s;
static float g_left_encoder_cm;
static float g_right_encoder_cm;
static float g_gyro_z_offset;
static float g_yaw_deg;
static uint32_t g_last_gyro_ms;

static float clamp_target(float value)
{
#if APP_ALLOW_REVERSE
    if (value > APP_MAX_SPEED_CM_S) return APP_MAX_SPEED_CM_S;
    if (value < -APP_MAX_SPEED_CM_S) return -APP_MAX_SPEED_CM_S;
    return value;
#else
    if (value < APP_MIN_FORWARD_CM_S) return APP_MIN_FORWARD_CM_S;
    if (value > APP_MAX_SPEED_CM_S) return APP_MAX_SPEED_CM_S;
    return value;
#endif
}

static void delay_ms(uint32_t ms)
{
    const uint32_t start = platform_millis();
    while ((uint32_t)(platform_millis() - start) < ms)
    {
    }
}

static bool mpu6050_read_gyro_z_raw(int16_t *raw)
{
    uint8_t data[2] = {0u, 0u};
    if (!platform_hw_i2c_read_regs(APP_MPU6050_ADDR, APP_MPU6050_GYRO_ZOUT_H, data, sizeof(data))) return false;

    *raw = (int16_t)(((uint16_t)data[0] << 8u) | data[1]);
    return true;
}

static void mpu6050_init(void)
{
    (void)platform_hw_i2c_write_reg(APP_MPU6050_ADDR, 0x6Bu, 0x00u);
    (void)platform_hw_i2c_write_reg(APP_MPU6050_ADDR, 0x1Bu, 0x00u);
}

static void mpu6050_calibrate(void)
{
    int32_t sum = 0;
    uint32_t valid = 0u;

    for (uint32_t i = 0u; i < APP_GYRO_CALIBRATION_SAMPLES; ++i)
    {
        int16_t raw = 0;
        if (mpu6050_read_gyro_z_raw(&raw))
        {
            sum += raw;
            ++valid;
        }
        delay_ms(10u);
    }

    g_gyro_z_offset = (valid == 0u) ? 0.0f : (float)sum / (float)valid;
}

static void update_yaw(void)
{
    const uint32_t now = platform_millis();
    const float dt_s = (float)(uint32_t)(now - g_last_gyro_ms) / 1000.0f;
    g_last_gyro_ms = now;

    int16_t raw = 0;
    if (!mpu6050_read_gyro_z_raw(&raw) || dt_s <= 0.0f) return;

    const float gyro_z_dps = ((float)raw - g_gyro_z_offset) / APP_MPU6050_GYRO_LSB_PER_DPS;
    if (gyro_z_dps > APP_GYRO_STATIC_FILTER_DPS || gyro_z_dps < -APP_GYRO_STATIC_FILTER_DPS)
    {
        g_yaw_deg += gyro_z_dps * dt_s;
    }
}

static void update_encoder_and_motor_pi(void)
{
    const int left_ticks = platform_hw_take_left_encoder_ticks();
    const int right_ticks = platform_hw_take_right_encoder_ticks();

    g_left_encoder_cm += (float)left_ticks / APP_ENCODER_TICKS_PER_CM;
    g_right_encoder_cm += (float)right_ticks / APP_ENCODER_TICKS_PER_CM;

    const float ticks_per_period = APP_ENCODER_TICKS_PER_CM * ((float)APP_CONTROL_PERIOD_MS / 1000.0f);
    const float left_target_ticks = g_left_target_cm_s * ticks_per_period;
    const float right_target_ticks = g_right_target_cm_s * ticks_per_period;

    int left_pwm = motor_pi_update(&g_left_motor, left_ticks, left_target_ticks);
    int right_pwm = motor_pi_update(&g_right_motor, right_ticks, right_target_ticks);

    if (g_left_target_cm_s == 0.0f)
    {
        motor_pi_reset(&g_left_motor);
        left_pwm = 0;
    }
    if (g_right_target_cm_s == 0.0f)
    {
        motor_pi_reset(&g_right_motor);
        right_pwm = 0;
    }

    platform_hw_set_left_pwm(motor_pi_apply_deadzone(&g_left_motor, left_pwm));
    platform_hw_set_right_pwm(motor_pi_apply_deadzone(&g_right_motor, right_pwm));
}

void SysTick_Handler(void)
{
    ++g_ms_ticks;
}

void platform_init(void)
{
    g_ms_ticks = 0u;
    SYST_RVR = (APP_SYSTEM_CORE_CLOCK_HZ / 1000u) - 1u;
    SYST_CVR = 0u;
    SYST_CSR = SYST_CLKSOURCE | SYST_TICKINT | SYST_ENABLE;

    platform_hw_init();

    motor_pi_init(&g_left_motor, APP_LEFT_MOTOR_KP, APP_LEFT_MOTOR_KI, APP_PWM_LIMIT, APP_LEFT_FWD_DEADZONE, APP_LEFT_REV_DEADZONE);
    motor_pi_init(&g_right_motor, APP_RIGHT_MOTOR_KP, APP_RIGHT_MOTOR_KI, APP_PWM_LIMIT, APP_RIGHT_FWD_DEADZONE, APP_RIGHT_REV_DEADZONE);

    g_left_target_cm_s = 0.0f;
    g_right_target_cm_s = 0.0f;
    g_left_encoder_cm = 0.0f;
    g_right_encoder_cm = 0.0f;
    g_yaw_deg = 0.0f;

    mpu6050_init();
    mpu6050_calibrate();
    g_last_gyro_ms = platform_millis();
}

uint32_t platform_millis(void)
{
    return g_ms_ticks;
}

bool platform_start_requested(void)
{
    static bool was_pressed = false;
    const bool pressed = platform_hw_start_button_pressed();
    const bool rising_edge = pressed && !was_pressed;
    was_pressed = pressed;
    return rising_edge;
}

PlatformRouteSelect platform_read_route_select(void)
{
    switch (platform_hw_route_switch_value())
    {
        case 1u:
            return PLATFORM_ROUTE_REQ1;
        case 2u:
            return PLATFORM_ROUTE_REQ2;
        case 4u:
            return PLATFORM_ROUTE_REQ4;
        case 3u:
        default:
            return PLATFORM_ROUTE_REQ3;
    }
}

void platform_read_sensors(PlatformSensors *sensors)
{
    if (sensors == NULL) return;

    update_encoder_and_motor_pi();
    update_yaw();

    for (size_t i = 0u; i < APP_LINE_SENSOR_COUNT; ++i)
    {
        const bool white_high = platform_hw_line_pin_is_high((uint8_t)i);
        sensors->line[i] = white_high ? 0u : APP_LINE_BLACK_READING;
    }
    sensors->left_encoder_cm = g_left_encoder_cm;
    sensors->right_encoder_cm = g_right_encoder_cm;
    sensors->yaw_deg = g_yaw_deg;
}

void platform_set_motors(float left_cm_s, float right_cm_s)
{
    g_left_target_cm_s = clamp_target(left_cm_s);
    g_right_target_cm_s = clamp_target(right_cm_s);
}

void platform_signal_waypoint(bool enable)
{
    platform_hw_set_waypoint_signal(enable);
}

void platform_signal_done(bool enable)
{
    platform_hw_set_done_signal(enable);
}

WEAK void platform_hw_init(void)
{
}

WEAK bool platform_hw_c07a_sw1_pin_is_high(void)
{
    return true;
}

WEAK bool platform_hw_start_button_pressed(void)
{
    const bool sw1_is_high = platform_hw_c07a_sw1_pin_is_high();
#if C07A_START_BUTTON_ACTIVE_LOW
    return !sw1_is_high;
#else
    return sw1_is_high;
#endif
}

WEAK uint8_t platform_hw_route_switch_value(void)
{
    return 3u;
}

WEAK bool platform_hw_line_pin_is_high(uint8_t index)
{
    (void)index;
    return true;
}

WEAK int platform_hw_take_left_encoder_ticks(void)
{
    return 0;
}

WEAK int platform_hw_take_right_encoder_ticks(void)
{
    return 0;
}

WEAK void platform_hw_set_left_pwm(int signed_pwm)
{
    (void)signed_pwm;
}

WEAK void platform_hw_set_right_pwm(int signed_pwm)
{
    (void)signed_pwm;
}

WEAK void platform_hw_set_waypoint_signal(bool enable)
{
    (void)enable;
}

WEAK void platform_hw_set_done_signal(bool enable)
{
    (void)enable;
}

WEAK bool platform_hw_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    (void)addr;
    (void)reg;
    (void)value;
    return false;
}

WEAK bool platform_hw_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    (void)addr;
    (void)reg;
    (void)data;
    (void)len;
    return false;
}
