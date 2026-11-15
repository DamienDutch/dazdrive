// Traffic probe (modules/traffic.cpp): what the game's list of AI vehicles holds, in OUR frame. Read-only, nothing acts on it.
// The numbers are for the log and for the developer's phone page (core/dev.cpp). The AEB reads the wider list (`ahead`) through core/aeblat.h to tell a radar lead that is in its path from one beside it
// (needs [traffic] probe = 1; without it the AEB works as before); the overtake assist reads the blind-spot flags.
#pragma once

struct traffic_obj {
    float dx, dy;      // m ahead of us / m to our left, from the middle of the vehicle's box to our placement origin
    float dist;        // m
    float v;           // its speed along its own heading, m/s (NAN = unknown, e.g. a trailer)
    float hdg;         // its heading relative to ours, degrees, + = turned left (NAN unknown)
    float len, wid;    // box size, m
    int kind;          // 0 vehicle, 1 trailer of a vehicle
    unsigned id;       // stable while the game keeps the vehicle (a hash of its address), the same as traffic.csv's id
    float rv_long, rv_lat;   // its speed minus ours along / across our heading, m/s (+ lat = to our left); NAN = unknown
};

struct traffic_around {
    int ok;                       // 1 = the list was read in the last second
    int n;                        // objects in obj[]: the nearest within 60 m
    traffic_obj obj[12];
    // cut-in reading (modules/cutin.cpp): the vehicles up to 130 m away within 25 m sideways, 15 m behind to 130 m ahead, the nearest first
    double read_t;                // wall_s() of the walk these come from (changes every 200 ms)
    int n_ahead;
    traffic_obj ahead[16];
    int has_trailer;              // our own trailer from the SDK placement, relative to our origin
    float t_dx, t_dy, t_hdg;      // m ahead (negative = behind), m left, degrees articulation (+ = trailer turned left of the truck)
    // the vehicle outlines the SDK configuration gives (vehicle space, z backward), NAN = not known
    float truck_front_z, truck_rear_z, truck_hook_z, trailer_hook_z, trailer_rear_z;
    // our own blind spot ([traffic] blind_spot, core/blindspot.h): a side zone along the truck and the trailer holds a vehicle that moves our way.
    // LOG ONLY: nothing acts on it. bs_warn = our indicator points to an occupied side. bs_on = the detector runs (probe on, driving fast enough).
    int bs_on, bs_left, bs_right, bs_warn;
    int bs_warn_l, bs_warn_r;     // which side the warning is for (bs_warn = either)
    // the body the zones follow (m; the truck's and the trailer's own front/rear extents from their origins, positive numbers) and the band
    float bs_tf, bs_tb, bs_hw, bs_lf, bs_lb, bs_lhw, bs_inner, bs_outer, bs_fm, bs_rm;
};

// The vehicle the Cutin reader (modules/cutin.cpp) predicts to merge into our lane: the LAST prediction, n counts them (0 = none yet), t is wall_s() of it, s / d its place on our expected path
// (m ahead, m to the left of it), closing our speed minus its own (m/s). The lead model (core/leadmodel.h) passes it on and the Cruise module logs it; nothing acts on it (WP-SP3: log only, until
// a ride shows fewer than one false alarm per hour). Cutin's own follow-up line says whether the radar took the vehicle as a lead (the same id): that is the false-alarm count.
struct cutin_pred { unsigned n, id; double t; float s, d, closing; };
inline cutin_pred g_cutin_pred;

int traffic_probe_on();                         // [traffic] probe = 1 (the list is read at all)
void traffic_around_get(traffic_around *out);   // zeroed (ok = 0) while the probe is off
