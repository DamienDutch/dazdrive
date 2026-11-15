// brake_hooks_sim.h: the game side of the brake hooks (core/game.h: pedal provider, brake light, retarder feed) for the sims that
// include core/brake.cpp, which have no core/game.cpp. The pedal decision is the game's own inline code (brake_pedal_decide), the
// light and the feed keep their 6-tick life (BL_TTL) like the stubs. Every hook is "not in" until the sim sets SBH.*_ok.
// Per physics tick the sim's plant calls sbh_feed_tick() / sbh_light_tick() (what the game would see this tick) and, for the
// driver's pedal, sbh_pedal(raw, req) (the service brake the game applies: max(req, the hook's value)).
#pragma once
#include "core/game.h"

static struct {
    int pedal_ok, light_ok, feed_ok;   // the hooks are "in"
    brake_pedal_state st;              // the pedal hook's state (provider, heartbeat)
    float feed; int feed_ttl;          // the fed retarder demand and its life (ticks)
    int light_ttl;                     // the brake-light request's life (ticks)
    int light_now, feed_calls;         // what the last tick saw (the sims read it)
} SBH;

void game_brake_pedal_set_provider(float (*fn)(float raw, float req, void *ga)) { brake_pedal_set(&SBH.st, fn); }
void game_brake_pedal_heartbeat() { brake_pedal_beat(&SBH.st); }
int game_brake_pedal_ok() { return SBH.pedal_ok; }
const brake_pedal_counts *game_brake_pedal_counts() { return &SBH.st.n; }
int game_brake_light_request(int on)
{
    if (!SBH.light_ok) return 0;
    SBH.light_ttl = on ? BL_TTL : 0;
    return 1;
}
int game_brake_light_ok() { return SBH.light_ok; }
int game_retarder_feed(float d)
{
    SBH.feed_calls++;
    if (!SBH.feed_ok) return 0;
    if (!(d > 0.0f)) { SBH.feed = 0; SBH.feed_ttl = 0; return d <= 0.0f; }
    if (d > 1.0f) { SBH.feed = 0; SBH.feed_ttl = 0; return 0; }
    SBH.feed = d; SBH.feed_ttl = BL_TTL;
    return 1;
}
int game_retarder_feed_ok() { return SBH.feed_ok; }
// The game's tick: the fed demand it maxes with its own, and whether our light request holds (both count their life down).
static inline float sbh_feed_tick() { if (SBH.feed_ttl > 0) { SBH.feed_ttl--; return SBH.feed; } return 0.0f; }
static inline int sbh_light_tick() { SBH.light_now = SBH.light_ttl > 0; if (SBH.light_ttl > 0) SBH.light_ttl--; return SBH.light_now; }
// The service brake the game applies this tick for the driver's raw pedal and the slewed request: max(req, hook value).
static inline float sbh_pedal(float raw, float req)
{
    float h = SBH.pedal_ok ? brake_pedal_decide(&SBH.st, raw, req, (void *)&SBH, 1, 1, 0) : raw;
    return req > h ? req : h;
}
