// Route look-ahead: the height profile of the GPS route ahead of the truck.
// Without an active route (or off it) everything reports "unknown" and the
// modules drive carefully on the live grade alone.
#pragma once
#include <stddef.h>

void route_reset();                     // forget the cached route (other truck, teleport, load)
void route_update();                    // call every frame; refreshes itself 4x per second
int route_ok();                         // 1 = on the route, profile valid
float route_ahead_m();                  // how far the profile reaches (m)
float route_grade(float d0, float d1);  // mean grade (%) between d0 and d1 meters ahead; NAN if unknown
float route_max_grade(float d0, float d1);  // steepest 50 m stretch (%) in [d0, d1]; NAN if unknown
float route_min_grade(float d0, float d1);  // steepest downhill 50 m stretch (%) in [d0, d1]; NAN if unknown
float route_crest_m(float max_d);       // distance to the top of the climb we are on; -1 if none within max_d
float route_valley_m(float max_d);      // a dip: distance to the lowest point within max_d when the road climbs >= 3 m within 300 m after it; -1 if none
float route_junction_m();                        // distance to the EXIT node of the next prefab (junction/roundabout) on the route; -1 if none
int route_junction_known(float dist);            // is the prefab whose exit is dist m ahead one of the junctions read? 1 yes, 0 no, -1 we are inside it
void route_junction_dump(char *buf, size_t n);   // what the junction reading holds, for a log line
float route_straight_m(float max_turn);             // how far ahead the route's heading stays within max_turn rad of the heading now (m); -1 if there is no route
float route_bend_speed(float a_lat, float a_dec);   // m/s allowed now for the road's own bends ahead (no junctions); NAN if unknown
float route_curve_speed(float a_lat, float a_dec, float a_dec_jn = -1);  // a_dec_jn: junctions planned with this (< 0 = a_dec)  // m/s allowed now for the bends ahead; NAN if unknown

// The road's own bends as POINTS (WP-SP2: the layers' requesters ask "v at distance d", not "the speed allowed now"): the apex speed of route_bend_speed's law at
// every sample, reduced to the points that can bind: nearest first, each strictly slower than every nearer one (a nearer, slower-or-equal point always binds
// before a farther, faster one, so the others never matter for any deceleration). route_bend_speed is the lowest envelope over these. Returns the count (<= max).
struct route_bend_t { float v_ms, d_m; };
int route_bend_points(float a_lat, route_bend_t *out, int max);

// Junctions on the route (turns and roundabouts): route_curve_speed includes them once
// route_junction_cfg(on, ...) has been called (cruise.cpp does, from its ini).
struct route_junction_t { float dist_m, turn_deg, limit_ms, ratio; int roundabout, curves; };
void route_junction_cfg(int on, float lat, float roundabout_kmh, float min_deg, float roundabout_ratio, int roundabout_curves = 2);
void route_hermite(int on);   // 1 (default) = the route between two nodes is a curve through their directions, not a straight line
float route_junction_speed(float a_dec, route_junction_t *info);  // m/s allowed now for the junctions ahead; NAN if none

// Speed limit ahead (speedlimit-ahead-re notes): the next LOWER limit (km/h) within max_m and the
// distance (m) to where it starts; NAN if none or unknown (currently always unknown: no source found).
float route_limit_ahead(float max_m, float *dist_m);
void route_probe_limit(float limit_kmh);  // test-ride probe: dumps the route nodes at a limit change

// The road ahead as uniform arrays over distance, for the longitudinal planner (core/planner.h on
// branch planner-core: its PlProfile holds grade and vmax on this same grid). route_profile() fills
// `n` samples every `step_m` metres from the truck's front (s = 0), up to the route's end (the
// geometry reaches ~6.4 km, the planner's horizon). The caps are the RAW speed allowed AT the point
// (the planner applies its own braking envelope): a bend's apex speed and a junction/roundabout's
// entry speed, both from the same laws as route_bend_speed / route_junction_speed (the bends use the
// stored junction lateral route_junction_cfg() set, JC.lat, as their sideways allowance). NAN where
// the route is unknown: off it, or beyond its end. Junctions are read by the same look-ahead as the
// point queries, so their caps appear only where those are known (~1.7 km ahead, the reader's horizon).
#define ROUTE_PROF_MAX 640   // 10 m -> 6.4 km (the planner's PL_PROF_MAX)

struct route_prof {
    float step_m;                     // the spacing used (echo of step_m)
    int   n;                          // valid samples filled (<= the n requested and ROUTE_PROF_MAX)
    float h[ROUTE_PROF_MAX];          // height (m) at i*step_m ahead; NAN where unknown
    float grade[ROUTE_PROF_MAX];      // rise/run at i*step_m (+ uphill, the planner's unit); NAN where unknown
    float vmax[ROUTE_PROF_MAX];       // m/s hard cap at the point (min of bend and junction); NAN = none
    float bend[ROUTE_PROF_MAX];       // m/s cap from the road's own bends; NAN = none
    float jn[ROUTE_PROF_MAX];         // m/s cap from a junction/roundabout; NAN = none
    signed char jn_kind[ROUTE_PROF_MAX];  // 0 none, 1 junction, 2 roundabout (the cap that applies here)
    float jn_m[ROUTE_PROF_MAX];       // metres to the junction entry (signed, + ahead); NAN = none
};
int route_profile(float step_m, int n, route_prof *out);   // 1 = profile filled, 0 = off the route / bad args
