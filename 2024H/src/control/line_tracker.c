#include "control/line_tracker.h"

#include <stddef.h>

static const float k_sensor_weights[APP_LINE_SENSOR_COUNT] = {-2.0f, -1.0f, 1.0f, 2.0f};

void line_tracker_init(LineTracker *tracker)
{
    pid_init(&tracker->pid, APP_LINE_KP, APP_LINE_KI, APP_LINE_KD, APP_MAX_SPEED_CM_S * 0.45f);
    tracker->last_position = 0.0f;
}

LineTrackerOutput line_tracker_update(LineTracker *tracker, const uint16_t line[APP_LINE_SENSOR_COUNT], float dt_s)
{
    uint32_t sum = 0u;
    float weighted_sum = 0.0f;

    for (size_t i = 0u; i < APP_LINE_SENSOR_COUNT; ++i)
    {
        const uint16_t value = line[i];
        if (value > APP_LINE_ACTIVE_THRESHOLD)
        {
            sum += value;
            weighted_sum += (float)value * k_sensor_weights[i];
        }
    }

    LineTrackerOutput output;
    output.active = sum > 0u;
    output.position = tracker->last_position;
    output.correction_cm_s = 0.0f;

    if (!output.active)
    {
        pid_reset(&tracker->pid);
        return output;
    }

    output.position = weighted_sum / (float)sum;
    tracker->last_position = output.position;
    output.correction_cm_s = pid_update(&tracker->pid, output.position, dt_s);
    return output;
}
