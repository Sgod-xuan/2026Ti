#ifndef NAVIGATION_ROUTE_H
#define NAVIGATION_ROUTE_H

#include <stdint.h>

#include "app_config.h"
#include "platform/platform.h"

typedef enum
{
    SEGMENT_STRAIGHT,
    SEGMENT_ARC_RIGHT,
    SEGMENT_ARC_LEFT,
} SegmentKind;

typedef struct
{
    SegmentKind kind;
    float length_cm;
    float speed_cm_s;
    char endpoint;
} RouteSegment;

typedef struct
{
    const RouteSegment *segments;
    uint8_t segment_count;
    uint8_t laps;
} RoutePlan;

const RoutePlan *route_get_plan(PlatformRouteSelect select);

#endif
