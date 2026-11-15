#pragma once
// The speed chooser's limit knowledge in the model layer (docs/architecture.md, keuzes 12). The cruise module (the speed chooser)
// publishes its two ini knobs (g_speed_limiter_kmh, g_car_trailer_lim_kmh) and the overspeed cap (g_overspeed_cap, g_overspeed_pct);
// everything a reader needs of the truck's top speed and of the overspeed line over the road's limit lives here, so the wish
// (core/wish.cpp) and the gearbox read the model, not the cruise module.
#include "suite.h"
#include "vehicle.h"

// Published by the cruise module (cr_start / cr_applied), read by the model and the gearbox:
extern volatile float g_speed_limiter_kmh;      // [cruise] speed_limiter_kmh (0 = off): our own speed limiter
extern volatile float g_car_trailer_lim_kmh;    // [cruise] car_trailer_limit_kmh: a light vehicle's cap with a trailer
extern volatile float g_overspeed_cap;          // [cruise] overspeed_cap_kmh (0 = no cap)
extern volatile float g_overspeed_pct;          // [cruise] overspeed_cap_pct in ATS (a share of the limit), 0 elsewhere

// The road's speed limit that applies to this vehicle (m/s, 0 = unknown): the one the game reports; a light vehicle gets the car
// limits from the game itself in Road Trip mode, or when the game is set to car limits (Damien 01-11). (core/vehicle.cpp)
float road_limit_ms();

// The game's speed limiter (the option g_use_speed_limiter) on? Cached, read once a second (was Cruise's limiter_on).
int speed_limiter_on();

// The highest speed the truck is held to (km/h, 0 = none): our own speed limiter, else the game's while it is on. TrueShift aims
// its gear choice at it where the road's limit is not known.
float speed_top_kmh();

// The cap over the road limit (km/h) where a fine starts: a share of the limit in ATS (US practice, ~10 %: 60 -> 66, 80 -> 88),
// a fixed overspeed_cap_kmh in ETS2. 0 = no cap (overspeed_cap_kmh 0 switches it off in both games). Moved from suite.h: the
// readers are the model's now.
static inline float overspeed_cap_now()
{
    if (!(g_overspeed_cap > 0)) return 0;
    float lim = road_limit_ms();
    return g_overspeed_pct > 0 && lim > 1.0f ? lim * 3.6f * g_overspeed_pct / 100.0f : g_overspeed_cap;
}
// How far the speed (m/s) is below the fine line (km/h): the road's limit plus the cap above, minus the speed; negative when over it. NAN when the limit is not
// known or the cap is off. The one place for this distance: the dashboard number (live.json "fine_room"), overspeed_room below, and anything that wants "how
// much speed is left before a fine" reads it. (It used to be a number against the cruise's set speed, which says nothing about a fine.)
static inline float fine_room_kmh(float speed_ms)
{
    float lim = road_limit_ms();
    if (!(g_overspeed_cap > 0) || !(lim > 1.0f)) return NAN;
    return lim * 3.6f + overspeed_cap_now() - speed_ms * 3.6f;
}
// What the set speed may still be allowed over itself (km/h) so that it stays under the fine line (1e9 = no cap, 0 = at or past it)
static inline float overspeed_room(float set_ms)
{
    float r = fine_room_kmh(set_ms);
    if (!(r == r)) return 1e9f;
    return r > 0 ? r : 0;
}
