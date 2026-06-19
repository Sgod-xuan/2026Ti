#include "control/controller.h"

#include <stddef.h>

#include "app_config.h"

static float clamp_forward(float value)
{
    if (value < APP_MIN_FORWARD_CM_S) return APP_MIN_FORWARD_CM_S;
    if (value > APP_MAX_SPEED_CM_S) return APP_MAX_SPEED_CM_S;
    return value;
}

static float average_distance_since_start(const CarController *controller, const PlatformSensors *sensors)
{
    const float left = sensors->left_encoder_cm - controller->segment_start_left_cm;
    const float right = sensors->right_encoder_cm - controller->segment_start_right_cm;
    return (left + right) * 0.5f;
}

static void begin_current_segment(CarController *controller, const PlatformSensors *sensors)
{
    controller->segment_start_left_cm = sensors->left_encoder_cm;
    controller->segment_start_right_cm = sensors->right_encoder_cm;
    controller->segment_start_yaw_deg = sensors->yaw_deg;
    pid_reset(&controller->straight_pid);
    line_tracker_init(&controller->line_tracker);
}

static void set_arc_open_loop(const RouteSegment *segment, float *left_cm_s, float *right_cm_s)
{
    const float inner_radius = APP_ARC_RADIUS_CM - APP_WHEEL_BASE_CM * 0.5f;
    const float outer_radius = APP_ARC_RADIUS_CM + APP_WHEEL_BASE_CM * 0.5f;
    const float inner_speed = segment->speed_cm_s * inner_radius / APP_ARC_RADIUS_CM;
    const float outer_speed = segment->speed_cm_s * outer_radius / APP_ARC_RADIUS_CM;

    if (segment->kind == SEGMENT_ARC_RIGHT)
    {
        *left_cm_s = outer_speed;
        *right_cm_s = inner_speed;
    }
    else
    {
        *left_cm_s = inner_speed;
        *right_cm_s = outer_speed;
    }
}

static void drive_segment(CarController *controller, const RouteSegment *segment, const PlatformSensors *sensors, float dt_s)
{
    float left_cm_s = segment->speed_cm_s;
    float right_cm_s = segment->speed_cm_s;

    if (segment->kind == SEGMENT_STRAIGHT)
    {
        const float yaw_error = sensors->yaw_deg - controller->segment_start_yaw_deg;
        const float correction = pid_update(&controller->straight_pid, yaw_error, dt_s);
        left_cm_s = segment->speed_cm_s + correction;
        right_cm_s = segment->speed_cm_s - correction;
    }
    else
    {
        set_arc_open_loop(segment, &left_cm_s, &right_cm_s);

        const LineTrackerOutput line = line_tracker_update(&controller->line_tracker, sensors->line, dt_s);
        if (line.active)
        {
            const float signed_correction = (segment->kind == SEGMENT_ARC_RIGHT) ? line.correction_cm_s : -line.correction_cm_s;
            left_cm_s += signed_correction;
            right_cm_s -= signed_correction;
        }
    }

    platform_set_motors(clamp_forward(left_cm_s), clamp_forward(right_cm_s));
}

void controller_init(CarController *controller, const RoutePlan *plan, const PlatformSensors *sensors)
{
    controller->plan = plan;
    controller->segment_index = 0u;
    controller->lap_index = 0u;
    controller->active = true;
    pid_init(&controller->straight_pid, APP_STRAIGHT_KP, APP_STRAIGHT_KI, APP_STRAIGHT_KD, APP_MAX_SPEED_CM_S * 0.4f);
    line_tracker_init(&controller->line_tracker);
    begin_current_segment(controller, sensors);
}

ControlEvent controller_update(CarController *controller, const PlatformSensors *sensors, float dt_s)
{
    ControlEvent event = {false, false, 0};

    if (!controller->active || controller->plan == NULL || controller->plan->segment_count == 0u)
    {
        platform_set_motors(0.0f, 0.0f);
        event.route_finished = true;
        return event;
    }

    const RouteSegment *segment = &controller->plan->segments[controller->segment_index];
    const float progress_cm = average_distance_since_start(controller, sensors);

    if (progress_cm + APP_DISTANCE_DONE_TOLERANCE_CM >= segment->length_cm)
    {
        event.waypoint_reached = true;
        event.waypoint = segment->endpoint;

        ++controller->segment_index;
        if (controller->segment_index >= controller->plan->segment_count)
        {
            controller->segment_index = 0u;
            ++controller->lap_index;
        }

        if (controller->lap_index >= controller->plan->laps)
        {
            event.route_finished = true;
            controller_stop(controller);
            return event;
        }

        begin_current_segment(controller, sensors);
        segment = &controller->plan->segments[controller->segment_index];
    }

    drive_segment(controller, segment, sensors, dt_s);
    return event;
}

void controller_stop(CarController *controller)
{
    controller->active = false;
    platform_set_motors(0.0f, 0.0f);
}
