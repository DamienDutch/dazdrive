#pragma once
// The powertrain layers (design: Damien 01-11/12, "Aandrijflijn in lagen"): one decision per layer.
//   layer 1  core/bands.h       the engine's rpm bands from its torque curve and the gearbox (no-go / lug / eco / power / temporary / no-go)
//   ahead    core/ahead.h       the road ahead, read-only for the layers below: the resistance over the next seconds and how sure we are
//                               (route known, planner driving); the only place the layers learn about the road
//   layer 2  core/intent.cpp    the driver's intent: the wished force, how much of it only holds the speed, how far a gear may fall
//                               short of it (lugging), and how much each band is wished for (a wish, never a wall)
//   layer 3  core/gearchoice.cpp  the gear: hard limits first (the caller's mask), then ONE score over the allowed gears, in seconds,
//                               with the expected time a gear loses to the demands still to come (gc_expect)
//   layer 4  TrueShift's hooks shift; the torque pedal delivers the wish in the chosen gear
// TrueShift owns the hard limits (rev limit, stall, braking, engine brake, hints, dwell / anti-hunting) and calls the layers.

#include "bands.h"
#include "ahead.h"

// THE MODE'S ONE FACT (K2 step 2, owner 10-11: the time:diesel rate is a mix, not a fixed truth). A mode's exchange rate is fuel_weight / response_weight (seconds of delay one litre is worth).
// q is the worth of a second against Normal's: Normal's rate over the mode's, so Normal = 1, Eco less (its litre is dear), Power more. tau = q / (1 + q) is the mode's share of time (Normal 0.5): the same
// share the roll's entry time (trueshift.cpp roll_entry_s_for) always took from the same two weights. Nothing else in the gear choice knows the mode by name for time and diesel; every
// reader takes q or tau from here. A weight of 0 (the fuel is not priced) leaves no rate: every mode is Normal's, q = 1.
static inline float pt_mode_q(float fuel_w, float resp_w, float fuel_n, float resp_n)
{
    const float rate_n = resp_n > 0.0f ? fuel_n / resp_n : 0.0f, rate_m = resp_w > 0.0f ? fuel_w / resp_w : 0.0f;
    return rate_n > 0.0f && rate_m > 0.0f ? rate_n / rate_m : 1.0f;
}
static inline float pt_mode_tau(float q) { return q / (1.0f + q); }

// Layer 2's settings (TrueShift's params), so intent.cpp needs no module.
struct pt_intent_cfg {
    float lug_max_pct;                   // the mode's lugging while pulling away (Eco 125, Normal 112.5, Power 107.5 by default)
    float band_weight;                   // seconds one unit of band cost is worth in layer 3's score (a sweep knob)
    float expect_weight = 1.0f;          // the mode's response weight (response_weight_eco / _normal / _power: Eco 0.6, Normal 1.0, Power 1.6): how many seconds
                                         // of expected delay a lower rpm is worth to this mode, the weight of the expected-time term (1 = a second is a second). The
                                         // one knob of a mode's character besides the band taste; it also sets where the band wish flips to the power band (1 / this)
    float fuel_weight = 0.0f;            // seconds one litre is worth to the mode (fuel_weight_eco / _normal / _power); 0 = the fuel is not priced
    float q = 1.0f;                      // the mode's worth of a second against Normal's (pt_mode_q): the one fact the lateness and the roll's entry read
    int controller = 0;                  // the wish is a controller's (the cruise, the planner, the game's cruise with the foot off), not a foot's (gc_in::controller): its throttle of the moment
                                         // is no operating point asked for, so a net descent does not weigh the taste (intent.cpp hold_share)
    // The pedal as an rpm target (WP-0911-K1, intent.cpp): the foot (the raw pedal the share stands for, core/wish.h wish_unmap; < 0 = the `pedal` argument is the foot), the end of this driver's
    // pedal (the learned one, core/driver.h driver_press_end, mapped to the foot's unit: the prior until he has shown it), how far up the way from P2 to P3 the mode's full-pedal point lies
    // (Eco 0.5, Normal 0.75, Power 1: K5), and whether a kickdown is out of the question (a controller's throttle is no press).
    float foot = -1.0f;
    float foot_end = 0.95f;
    float full_frac = 0.75f;
    float road_pw = 0.0f;                // the power the road takes over the engine's peak power at the wheels, 0 .. 1 (0 = the road asks nothing, a descent); the load enters the target through it (K2 v3)
    int no_kick = 0;
    int target = 1;                      // 0 = no foot: the wish has no pedal behind it (the game's own cruise, its throttle swings with every hill), so there is no operating point asked for
};

// Layer 2's output, recomputed every frame the gear choice runs.
struct pt_intent {
    int ok;                    // 0 = no intent this frame (no wish: coasting, braking, the pedal off): layer 3 keeps the gear
    int mode;                  // 0 Eco, 1 Normal, 2 Power (TrueShift's MODE_*)
    float wish_n;              // the force wished at the wheels (N): the torque pedal's power wish at this speed, or the cruise's / planner's; pedal off:
                               // the demand EXPECTED over the shift window (the caller decays the pre-lift demand with the time since the lift)
    float hold_n;              // of it, the force that only holds the speed (now, from ahead.h) (N)
    float hold_ahead_n;        // the worst holding force over the road ahead (ahead.h) (N)
    float hold_share;          // |hold_n| / wish_n, 0..1: the share of the wish the road decides (the resistance, or the push of a net descent; WP-G6b)
    float urgency;             // 0..1: how fast the pedal went down (a stomp = 1); a kickdown comes sooner with it
    // Per band, the most of a gear's full-throttle force the wish may need (wish_n <= share x the force at the rpm the gear runs at):
    // > 1 lugging allowed (while pulling away), 1 otherwise, 0 = the band is not allowed now. _land for a shift target, _run for the current gear
    // (hysteresis). What a gear in the lug band costs is gc_expect's price for the demands it cannot deliver, not a share.
    float max_share_land[PT_BANDS], max_share_run[PT_BANDS];
    float band_cost_land[PT_BANDS];  // soft cost per band for the rpm a gear would LAND at (0 = wished for; higher = less wished for)
    float band_cost_run[PT_BANDS];   // the same for the rpm the CURRENT gear runs at; the lug band has none
                                     // (PT_NOGO_LO / PT_NOGO_HI: share 0, layer 3 treats them as hard)
    float eco_top_cost_land, eco_top_cost_run;   // the eco band is one band: band_cost[PT_ECO] is the cost at P1, these at P2; pt_eco_cost() ramps between
    float need;                // the driver's demand (its 0.7-quantile) / the force the eco band can deliver at this speed (0 = unknown): 1 = the eco band is just enough
    float tgt_rpm;             // the operating point the foot asks of the engine (rpm, WP-0911-K1): P1 at a pedal of nothing, up to the mode's full-pedal point at the end of the pedal (less a pedal step),
                               // P3 in every mode from there on (the kickdown); 0 = none (no bands). Layer 3 prices every gear's distance from it.
    float tgt_kd;              // 0..1: how far into the kickdown the foot is (0 below the end of his pedal, 1 at it)
    float tgt_foot, tgt_end, tgt_full;   // what the point is made of (pt_target_at): the foot it was read at (the raw pedal), the end of this driver's pedal, the mode's full-pedal point (rpm). Layer 3 reads the point again at the foot a candidate gear would have for the same wish
    float tgt_load;            // the load's point (rpm): the lowest rpm whose full-throttle power reaches what the road takes (cfg road_pw, the power table of the bands); P1 where the plateau start already gives it. Layer 3 reads the point again at the foot a candidate gear would have for the same wish
    float tgt_lo;              // 0..1: how much the foot minds revs UNDER its point (a light foot minds none: a calm engine is what it asked for; a firm one minds as much as revs over it)
    float ahead_conf;          // 0..1 how sure the road ahead is (ahead.h), for the log
    float expect_weight;       // cfg's: the weight of gc_expect in the score
    float fuel_weight;         // cfg's: seconds per litre of the fuel term
    float q, tau;              // cfg's: the mode's worth of a second against Normal's, and its share of time q / (1 + q) (pt_mode_q)
};

// Layer 2. mode: 0/1/2. v: m/s. wish_n < 0 = no wish. road: ahead_get()'s view for this frame. pedal 0..1, pedal_rate per second.
// eco_force_n: the largest full-throttle force (N) any gear gives with its rpm in the eco bands at this speed (the caller reads it off
// the vehicle model; < 0 = unknown, the need is 0). demand_n: the force the driver's demand history reaches in 3 of 10 windows (core/demand.h: its 0.7-quantile
// level times a full wish at this speed; < 0 = none known, the wish is used): the need that switches the band wish reads this, filtered by the minutes it comes from,
// so a stab at the pedal does not flip the band (the pedal itself still does, at ~full).
void pt_intent_compute(pt_intent *out, const pt_intent_cfg *cfg, int mode, float v, float wish_n, const ahead_view *road,
                       float pedal, float pedal_rate, const eng_bands *b, float eco_force_n = -1.0f, float demand_n = -1.0f);

// The operating point of the foot `foot` (the raw pedal 0..1) for an intent computed with a target: the one formula (pt_intent_compute reads it at the foot of now, layer 3 at the foot a candidate gear would have for the same wish).
// rpm = the point, lo = how much the foot minds revs under it (the share of the way up). The kickdown (it->tgt_kd) is the driver's press and is the same whatever the gear.
void pt_target_at(const pt_intent *it, const eng_bands *b, float foot, float *rpm, float *lo);

// The band an rpm falls in (b->ok must be 1). cross = the crossover of the gear the rpm belongs to (bands_cross; < 0 = unknown: the
// power band then runs to the cap and there is no temporary band).
int pt_band_of(const eng_bands *b, float rpm, float cross = -1.0f);

// The band cost of an rpm inside the eco band: the ramp from the cost at P1 to the cost at P2 by the depth in the band (run = the gear
// we are in, else a landing).
float pt_eco_cost(const pt_intent *it, const eng_bands *b, float rpm, int run);
