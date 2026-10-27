// Traffic probe (modules/traffic.cpp): what the game's list of AI vehicles holds, in OUR frame. Read-only, nothing acts on it.
// The numbers are for the log and for the developer's phone page (core/dev.cpp); other modules do not read them.
#pragma once

struct traffic_obj {
    float dx, dy;      // m ahead of us / m to our left, from the middle of the vehicle's box to our placement origin
    float dist;        // m
    float v;           // its speed along its own heading, m/s (NAN = unknown, e.g. a trailer)
    float hdg;         // its heading relative to ours, degrees, + = turned left (NAN unknown)
    float len, wid;    // box size, m
    int kind;          // 0 vehicle, 1 trailer of a vehicle
};

struct traffic_around {
    int ok;                       // 1 = the list was read in the last second
    int n;                        // objects in obj[]: the nearest within 60 m
    traffic_obj obj[12];
    int has_trailer;              // our own trailer from the SDK placement, relative to our origin
    float t_dx, t_dy, t_hdg;      // m ahead (negative = behind), m left, degrees articulation (+ = trailer turned left of the truck)
    // the vehicle outlines the SDK configuration gives (vehicle space, z backward), NAN = not known
    float truck_front_z, truck_rear_z, truck_hook_z, trailer_hook_z, trailer_rear_z;
};

void traffic_around_get(traffic_around *out);   // zeroed (ok = 0) while the probe is off
