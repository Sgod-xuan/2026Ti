#ifndef CONTROL_LINE_TRACKER_H
#define CONTROL_LINE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "control/pid.h"

typedef struct
{
    PidController pid;
    float last_position;
} LineTracker;

typedef struct
{
    bool active;
    float position;
    float correction_cm_s;
} LineTrackerOutput;

void line_tracker_init(LineTracker *tracker);
LineTrackerOutput line_tracker_update(LineTracker *tracker, const uint16_t line[APP_LINE_SENSOR_COUNT], float dt_s);

#endif
