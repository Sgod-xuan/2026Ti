#include "app/car_app.h"

#include <stdbool.h>
#include <stddef.h>

#include "app_config.h"
#include "control/controller.h"
#include "navigation/route.h"
#include "platform/platform.h"

typedef enum
{
    APP_WAIT_START,
    APP_RUNNING,
    APP_DONE,
} AppState;

static AppState g_state;
static CarController g_controller;
static uint32_t g_alert_until_ms;

void car_app_init(void)
{
    g_state = APP_WAIT_START;
    g_alert_until_ms = 0u;
    platform_signal_waypoint(false);
    platform_signal_done(false);
}

void car_app_update(uint32_t now_ms)
{
    PlatformSensors sensors;
    platform_read_sensors(&sensors);

    if (g_alert_until_ms != 0u && (int32_t)(now_ms - g_alert_until_ms) >= 0)
    {
        g_alert_until_ms = 0u;
        platform_signal_waypoint(false);
    }

    if (g_state == APP_WAIT_START)
    {
        if (!platform_start_requested()) return;

        const RoutePlan *plan = route_get_plan(platform_read_route_select());
        controller_init(&g_controller, plan, &sensors);
        platform_signal_waypoint(true);
        g_alert_until_ms = now_ms + APP_WAYPOINT_ALERT_MS;
        g_state = APP_RUNNING;
        return;
    }

    if (g_state == APP_RUNNING)
    {
        const ControlEvent event = controller_update(&g_controller, &sensors, (float)APP_CONTROL_PERIOD_MS / 1000.0f);

        if (event.waypoint_reached)
        {
            platform_signal_waypoint(true);
            g_alert_until_ms = now_ms + APP_WAYPOINT_ALERT_MS;
        }

        if (event.route_finished)
        {
            controller_stop(&g_controller);
            platform_signal_waypoint(false);
            platform_signal_done(true);
            g_state = APP_DONE;
        }
        return;
    }

    platform_set_motors(0.0f, 0.0f);
}
