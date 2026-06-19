#ifndef CONTROL_CONTROLLER_H
#define CONTROL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "control/line_tracker.h"
#include "control/pid.h"
#include "navigation/route.h"
#include "platform/platform.h"

typedef struct
{
    bool waypoint_reached;
    bool route_finished;
    char waypoint;
} ControlEvent;

typedef struct
{
    const RoutePlan *plan;
    uint8_t segment_index;
    uint8_t lap_index;
    float segment_start_left_cm;
    float segment_start_right_cm;
    float segment_start_yaw_deg;
    PidController straight_pid;
    LineTracker line_tracker;
    bool active;
} CarController;

void controller_init(CarController *controller, const RoutePlan *plan, const PlatformSensors *sensors);
ControlEvent controller_update(CarController *controller, const PlatformSensors *sensors, float dt_s);
void controller_stop(CarController *controller);

#endif
