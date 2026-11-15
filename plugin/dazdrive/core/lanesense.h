// The road curvature the game's lane-keeping law reads, for the readers that want the road's shape (Cutin's path frame). The law's inputs (the lane offsets o1, o2 and
// the curvature term o3) exist only inside the hook, on the game's thread, so the hook (Lane's stub_law) publishes o3 here and every reader asks the sense layer instead of
// calling the Lane module (WP-LA; it was lane_curvature() in modules/lane.cpp, a module-to-module call).
#pragma once
#include "suite.h"

struct lane_reading { volatile float o3; volatile double at; };   // the lane term of the last law call the lane was found in (1/m, + in a right bend), when
inline lane_reading g_lane_reading = { 0.0f, -1e9 };

inline void sense_lane_publish(float o3) { g_lane_reading.o3 = o3; g_lane_reading.at = wall_s(); }
// The road curvature (1/m, + = left) and 1 while the lane was read in the last 0.2 s; 0 otherwise (no lane assist running, the lane lost).
inline int sense_lane_curvature(float *k_left)
{
    if (wall_s() - g_lane_reading.at > 0.2) return 0;
    *k_left = -g_lane_reading.o3;
    return 1;
}
