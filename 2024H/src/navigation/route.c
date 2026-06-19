#include "navigation/route.h"

static const RouteSegment k_req1_segments[] = {
    {SEGMENT_STRAIGHT, APP_STRAIGHT_AB_CM, APP_CRUISE_SPEED_CM_S, 'B'},
};

static const RouteSegment k_req2_segments[] = {
    {SEGMENT_STRAIGHT, APP_STRAIGHT_AB_CM, APP_CRUISE_SPEED_CM_S, 'B'},
    {SEGMENT_ARC_RIGHT, APP_ARC_HALF_CM, APP_ARC_SPEED_CM_S, 'C'},
    {SEGMENT_STRAIGHT, APP_STRAIGHT_CD_CM, APP_CRUISE_SPEED_CM_S, 'D'},
    {SEGMENT_ARC_LEFT, APP_ARC_HALF_CM, APP_ARC_SPEED_CM_S, 'A'},
};

static const RouteSegment k_req3_segments[] = {
    {SEGMENT_STRAIGHT, APP_DIAGONAL_AC_CM, APP_CRUISE_SPEED_CM_S, 'C'},
    {SEGMENT_ARC_RIGHT, APP_ARC_HALF_CM, APP_ARC_SPEED_CM_S, 'B'},
    {SEGMENT_STRAIGHT, APP_DIAGONAL_BD_CM, APP_CRUISE_SPEED_CM_S, 'D'},
    {SEGMENT_ARC_LEFT, APP_ARC_HALF_CM, APP_ARC_SPEED_CM_S, 'A'},
};

static const RoutePlan k_req1_plan = {k_req1_segments, 1u, 1u};
static const RoutePlan k_req2_plan = {k_req2_segments, 4u, 1u};
static const RoutePlan k_req3_plan = {k_req3_segments, 4u, 1u};
static const RoutePlan k_req4_plan = {k_req3_segments, 4u, 4u};

const RoutePlan *route_get_plan(PlatformRouteSelect select)
{
    switch (select)
    {
        case PLATFORM_ROUTE_REQ1:
            return &k_req1_plan;
        case PLATFORM_ROUTE_REQ2:
            return &k_req2_plan;
        case PLATFORM_ROUTE_REQ4:
            return &k_req4_plan;
        case PLATFORM_ROUTE_REQ3:
        default:
            return &k_req3_plan;
    }
}
