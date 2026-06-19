#ifndef PLATFORM_PLATFORM_H
#define PLATFORM_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

typedef enum
{
    PLATFORM_ROUTE_REQ1 = 1,
    PLATFORM_ROUTE_REQ2 = 2,
    PLATFORM_ROUTE_REQ3 = 3,
    PLATFORM_ROUTE_REQ4 = 4,
} PlatformRouteSelect;

typedef struct
{
    uint16_t line[APP_LINE_SENSOR_COUNT];
    float left_encoder_cm;
    float right_encoder_cm;
    float yaw_deg;
} PlatformSensors;

void platform_init(void);
uint32_t platform_millis(void);
bool platform_start_requested(void);
PlatformRouteSelect platform_read_route_select(void);
void platform_read_sensors(PlatformSensors *sensors);
void platform_set_motors(float left_cm_s, float right_cm_s);
void platform_signal_waypoint(bool enable);
void platform_signal_done(bool enable);

#endif
