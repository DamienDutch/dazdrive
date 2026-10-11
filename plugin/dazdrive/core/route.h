// Route look-ahead: the height profile of the GPS route ahead of the truck.
// Without an active route (or off it) everything reports "unknown" and the
// modules drive carefully on the live grade alone.
#pragma once

void route_reset();                     // forget the cached route (other truck, teleport, load)
void route_update();                    // call every frame; refreshes itself 4x per second
int route_ok();                         // 1 = on the route, profile valid
float route_ahead_m();                  // how far the profile reaches (m)
float route_grade(float d0, float d1);  // mean grade (%) between d0 and d1 meters ahead; NAN if unknown
float route_max_grade(float d0, float d1);  // steepest 50 m stretch (%) in [d0, d1]; NAN if unknown
float route_min_grade(float d0, float d1);  // steepest downhill 50 m stretch (%) in [d0, d1]; NAN if unknown
float route_crest_m(float max_d);       // distance to the top of the climb we are on; -1 if none within max_d
float route_valley_m(float max_d);      // a dip: distance to the lowest point within max_d when the road climbs >= 3 m within 300 m after it; -1 if none
float route_junction_m();               // distance to the next junction/roundabout (prefab) on the route; -1 if none
float route_curve_speed(float a_lat, float a_dec, float a_dec_jn = -1);  // a_dec_jn: junctions planned with this (< 0 = a_dec)  // m/s allowed now for the bends ahead; NAN if unknown

// Junctions on the route (turns and roundabouts): route_curve_speed includes them once
// route_junction_cfg(on, ...) has been called (cruise.cpp does, from its ini).
struct route_junction_t { float dist_m, turn_deg, limit_ms, ratio; int roundabout, curves; };
void route_junction_cfg(int on, float lat, float roundabout_kmh, float min_deg, float roundabout_ratio);
float route_junction_speed(float a_dec, route_junction_t *info);  // m/s allowed now for the junctions ahead; NAN if none

// Speed limit ahead (docs/speedlimit-ahead-re.md): the next LOWER limit (km/h) within max_m and the
// distance (m) to where it starts; NAN if none or unknown (currently always unknown: no source found).
float route_limit_ahead(float max_m, float *dist_m);
void route_probe_limit(float limit_kmh);  // test-ride probe: dumps the route nodes at a limit change
