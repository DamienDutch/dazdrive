// The driver's intent (layer 2 of the powertrain, see powertrain.h): the wished force, how much of it only holds
// the speed (hold_share), how far a gear may fall short of the wish (the lugging share), and how much each band is wished for
// (band_cost, a wish, never a wall). pt_intent_compute is pure; pt_band_of maps an rpm to its band.
// (How much reserve a gear keeps for what is still to come is no longer an intent: layer 3 prices it, core/gearchoice.cpp gc_expect.)
#include "powertrain.h"
#include "driver.h"
#include <math.h>
#include <string.h>

// The band wish rows, seconds-free (layer 3 uses them as seconds): the soft cost of running in each band, land and run separately
// (land = the rpm a gear would LAND at, run = the rpm the current gear runs at). 0 = wished for, 1 = less, 2 = far, 3 = farthest. Only
// ECO..TMP carry a cost: the lug band has none (what a gear in lug costs is what it cannot deliver of the demands to come, gc_expect), and the
// no-go bands are hard (layer 3). The eco band is ONE band: ECO is the cost at its start (P1) and the cost ramps up (or down) across it to the
// power band's cost at its end (P2), so there is no step between the two and the box finds the bottom of the eco band by cost (pt_eco_cost; the
// depth cost above the eco band is layer 3's).
// A mode has two rows and its wish is the blend of them by the NEED (the driver's demand, the 0.7-quantile of his last minutes, against what the eco band can
// deliver at this speed, see pt_intent_compute): LOW is what it wants while the eco band does the job (calm, light, flat), HIGH (the same for every mode) is the
// power band, for when the eco band would not. The mode's response weight sets where the blend flips (1 / cfg->expect_weight).
enum { R_ECO = 0, R_POW, R_TMP, R_N };
static const float ROW_LOW_LAND[3][R_N] = {
    //  ECO  POW  TMP
    {   0, 3.0f, 3.5f },   // 0 Eco: the lowest revs
    {   0, 1.5f, 2.5f },   // 1 Normal
    {   0, 1.5f, 2.5f },   // 2 Power: calm and light is the eco band too
};
static const float ROW_LOW_RUN[3][R_N] = {
    //  ECO  POW  TMP
    {   0, 2.5f, 3.0f },   // 0 Eco
    {   0, 1.5f, 2.5f },   // 1 Normal
    {   0, 1.5f, 2.5f },   // 2 Power
};
static const float ROW_HIGH_LAND[R_N] = { 2, 0, 1 };      // the power band is wished for: the eco band costs more than a shift does
static const float ROW_HIGH_RUN[R_N] = { 2, 0, 1 };

// The gear already running may fall this much further short of the wish than a gear we would land in (hysteresis: two gears that each meet the
// wish only at their own wish took turns as the pedal wobbled, ~one shift per second of pedal).
static const float RUN_SLACK = 1.1f;

static float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static float smoothstep(float e0, float e1, float x)
{
    if (e1 <= e0) return x >= e1 ? 1.0f : 0.0f;
    float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3 - 2 * t);
}

// The one formula of the target (K2 v3, docs/architecture.md "Round K2 v3"): the target follows the foot AND the load. The foot's point is K1's line from P1 to the mode's full-pedal point F (it->tgt_full, the mode's
// own: Eco the middle of P2 and P3, Normal three quarters, Power the peak) at the foot x (over the end of his pedal less a step); the load's point is the lowest rpm whose full-throttle power reaches the power the road
// takes (it->tgt_load, the engine's power table, made with the bands): below it the truck cannot hold the speed at all. The target is the larger of the two, smoothly: the load lifts the foot's point by the part of the
// miss over one pedal step, w = DRV_END_STEP (P3 - P1), in a ramp, one pedal step of the engine's whole working range wide (P1 .. P3: the same for every mode, so the modes keep their order where the road decides), that starts flat (C1: the lift is d^2 / 2w for a miss d under w and d - w/2 over it), so it is nothing on a road the foot already asks more
// than (the plateau start gives 0.65 of the peak power: a light truck is never lifted, a firm foot is the foot's), never under the larger point by more than w/2, and rises with the load and with the foot.
// The mode is not in the load's point (it is the engine's and the road's): where the road alone decides the modes may tie. Cost: one multiply-add and a compare per read.
void pt_target_at(const pt_intent *it, const eng_bands *b, float foot, float *rpm, float *lo)
{
    const float x = clamp01(foot / fmaxf(it->tgt_end - DRV_END_STEP, 0.5f));
    const float span = fmaxf(it->tgt_full - b->p1, 1.0f);
    const float line = b->p1 + span * x, w = DRV_END_STEP * fmaxf(b->p3 - b->p1, 1.0f), d = it->tgt_load - line;
    const float t = line + (d <= 0.0f ? 0.0f : (d < w ? d * d / (2.0f * w) : d - 0.5f * w));
    *lo = clamp01((t - b->p1) / span);   // (the share of the way up: the same number that moves the point)
    *rpm = t + it->tgt_kd * (fmaxf(b->p3, t) - t);
}

void pt_intent_compute(pt_intent *out, const pt_intent_cfg *cfg, int mode, float v, float wish_n, const ahead_view *road,
                       float pedal, float pedal_rate, const eng_bands *b, float eco_force_n, float demand_n)
{
    (void)v;
    memset(out, 0, sizeof *out);
    out->mode = mode;
    out->wish_n = wish_n;
    out->hold_n = road->resist_now_n;
    out->hold_ahead_n = road->resist_worst_n;
    out->ahead_conf = road->conf;
    out->expect_weight = cfg->expect_weight;
    out->fuel_weight = cfg->fuel_weight;
    out->q = cfg->q > 0.0f ? cfg->q : 1.0f; out->tau = pt_mode_tau(out->q);

    out->ok = wish_n > 1.0f;
    if (!out->ok)                          // no wish (coasting, braking, the pedal off): layer 3 keeps the gear
        return;                            // and everything else stays neutral (all zeros)

    // The share of the wish the road decides: the force of the road against the wish, whichever way the road pushes (WP-G6b). The weight of the band taste, the fuel and the history's
    // reserve in layer 3 is this share: a wish that is mostly a pull (the road asks little of it) is timed by the plan, a wish that is small against what the road does is a correction,
    // and the gear is chosen by the engine's operating point and the reserve it keeps. It was resist_now / wish with the resistance below 0 (a net descent: the grade pushes the truck)
    // clamped to 0, so on a descent all three vanished for every gear, whatever the engine was doing (the owner's ride of 03-11: Eco kept 10 at ~1750 rpm at a pedal of 0.14 on -2 %
    // until the limiter, and the gear's revs and the diesel are real whoever holds the speed). On a descent the push is the road's force just as the resistance is on a climb: the part of
    // the wish that the road's force decides is |resist| / wish ... but the taste is about where the ENGINE works (round 2 of G6b, the manager's review): on a descent the engine's part in the
    // force balance is the wish against the push, and an engine that is asked for (almost) nothing against a push (a cruise easing off on a crest, the throttle of a fuel cut) has no operating
    // point worth a taste: the gear that drags most is the cheapest brake the truck has (the brake channel's stage of cost 0), and a taste that pushed the box up there gave it up and the
    // retarder and the service brake paid (run_pcc crest and roll routes: brake energy +30 %). So on a descent the share is the smaller of the two forces over the larger, wish / push where
    // the engine asks less than the push (0 as the wish goes to 0, continuously), push / wish where it asks more (the pull of the climb's formula): 1 where the engine's part is as large as the
    // road's. On the flat and on a climb it is the resistance over the wish, as it was (the pedal that holds or pulls); continuous in the pedal and in the grade, also where the resistance is 0
    // (push / wish -> 0 from both sides: a pedal on a road that asks nothing is a pull, as it was).
    const float push = -out->hold_n;   // the grade's force on the truck on a net descent (> 0)
    // A controller's wish (the cruise, the planner) is not a foot's: on a descent it is the loop easing off as the grade takes over (the crest of run_pcc: the cruise's throttle 0.35 -> 0 in
    // 4 s while the push is 8 kN, a weight of ~1 sent the box from 11 to 14 in that second, the engine drag was given up and the brakes paid: brake energy +30 %). For it a net descent keeps
    // the weight 0, as it was: the choice of the gear under the cruise on a descent is the drag gear's (the brake channel's cheapest stage), not the taste's.
    out->hold_share = push > 0.0f ? (cfg->controller ? 0.0f : clamp01(fminf(wish_n, push) / fmaxf(wish_n, push))) : clamp01(out->hold_n / wish_n);
    out->urgency = driver_urgency(pedal_rate);   // clamp01(rate / the scale of this driver's presses): a press from 0 to full in 0.5 s = 1 until his presses say otherwise (core/driver.h (b))
    // The pedal as an rpm target (WP-0911-K1). ONE regime, no flag at 0.95: the foot moves an operating point continuously from the calm revs at the bottom of the engine's working range (P1: full
    // torque at the lowest revs) up to the mode's full-pedal point (K5: Eco the middle of P2 and P3, Normal three quarters of the way, Power P3), which it reaches a pedal step before the end of
    // the pedal; from there the target goes on to P3 in every mode, the kickdown (K1: only at the END of THIS driver's pedal, which the profile learns: core/driver.h driver_press_end).
    // The steps are the engine's own points (core/bands) and the pedal's: nothing in rpm is written here. A controller's throttle (the cruise, the planner) is no press: no kickdown.
    out->tgt_rpm = 0.0f; out->tgt_kd = 0.0f; out->tgt_lo = 0.0f; out->tgt_foot = 0.0f; out->tgt_end = 0.0f; out->tgt_full = 0.0f; out->tgt_load = 0.0f;
    if (b && b->ok && cfg->target) {
        const float foot = clamp01(cfg->foot >= 0.0f ? cfg->foot : pedal);
        const float end = fminf(fmaxf(cfg->foot_end, 0.5f), 1.0f);
        out->tgt_foot = foot; out->tgt_end = end;
        out->tgt_full = b->p2 + clamp01(cfg->full_frac) * fmaxf(b->p3 - b->p2, 0.0f);
        out->tgt_load = bands_rpm_for_power(b, clamp01(cfg->road_pw));   // (what the road takes of the engine's peak power: the revs under which the speed cannot be held)
        out->tgt_kd = cfg->no_kick ? 0.0f : smoothstep(end - DRV_END_STEP, end, foot);
        pt_target_at(out, b, foot, &out->tgt_rpm, &out->tgt_lo);
    }

    // The force share a gear's full-throttle pull may be asked for: above 1 while pulling away (lugging), 1 where the wish is mostly holding the speed.
    //   hold_share <= u:  1 + (lug_max/100 - 1) * (1 - t^2),  t = hold_share / u
    //   hold_share >  u:  1
    // u = 1 - 1/lug_max (the plan's derivation): a gear lugging at the full allowance gives 1/lug_max of the wish, so it lacks 1 - 1/lug_max of it.
    // Lugging is allowed while the wish is mostly acceleration; once the holding force alone takes more of the wish than the gear lacks (hold_share
    // over u) the allowance is spent and the gear must deliver the wish (Eco 125 %: u = 0.20, Normal 112.5 %: 0.11, Power 107.5 %: 0.07).
    // The reserve for what comes next is no share any more: gc_expect prices it.
    float lmax = cfg->lug_max_pct / 100.0f;
    float u = lmax > 1.0f ? 1.0f - 1.0f / lmax : 0.0f;
    float curve = 1.0f;
    if (u > 0.0f && out->hold_share < u) {
        float t = clamp01(out->hold_share / u);
        curve = 1.0f + (lmax - 1.0f) * (1.0f - t * t);
    }

    // every band except the no-go pair: the curve value, the same for land and run.
    for (int i = PT_LUG; i <= PT_TMP; i++) {
        out->max_share_land[i] = curve;
        out->max_share_run[i] = curve * RUN_SLACK;
    }
    // (PT_NOGO_LO / PT_NOGO_HI: share 0, layer 3 treats them as hard)

    // the band wish: the mode's LOW row blended toward the power band's HIGH row by the NEED, the driver's demand against what the eco band can deliver at this
    // speed (eco_force_n: layer 3's caller reads it off the vehicle model, so mass, power and the grade are in it; demand_n: the force the demand history reaches in
    // 3 of 10 windows, ten minutes of him, so a stab at the pedal changes nothing and no hysteresis is needed). Calm and light stays in the eco band in every mode, a
    // demand the eco band cannot carry goes to the power band. Where the blend flips is the mode's response weight L (how many seconds of expected delay a lower
    // rpm is worth to it): the band wish is worth moving to the power band where L x the demand reaches what the eco band delivers, so the switch is at need 1 / L
    // (Eco 0.6: 1.67, the eco band is never short for it; Normal 1.0: 1; Power 1.6: 0.63). A (nearly) floored pedal is the power band in every mode. No eco force known
    // (< 0): the need is 0.
    int m = mode < 0 ? 0 : (mode > 2 ? 2 : mode);
    float need = eco_force_n > 1.0f ? (demand_n >= 0 ? demand_n : wish_n) / eco_force_n : 0.0f;
    out->need = need;
    float sw = 1.0f / fmaxf(cfg->expect_weight, 0.1f);
    float half = 0.2f * sw;
    float k = fmaxf(smoothstep(sw - half, sw + half, need), smoothstep(0.6f, 0.95f, pedal));
    float land[R_N], run[R_N];
    for (int i = 0; i < R_N; i++) {
        land[i] = ROW_LOW_LAND[m][i] + (ROW_HIGH_LAND[i] - ROW_LOW_LAND[m][i]) * k;
        run[i] = ROW_LOW_RUN[m][i] + (ROW_HIGH_RUN[i] - ROW_LOW_RUN[m][i]) * k;
    }
    out->band_cost_land[PT_LUG] = 0; out->band_cost_run[PT_LUG] = 0;   // the lug band has a price, not a taste (gc_expect)
    out->band_cost_land[PT_ECO] = land[R_ECO]; out->band_cost_run[PT_ECO] = run[R_ECO];
    out->eco_top_cost_land = land[R_POW]; out->eco_top_cost_run = run[R_POW];   // (the ramp ends where the power band begins)
    out->band_cost_land[PT_POW] = land[R_POW]; out->band_cost_run[PT_POW] = run[R_POW];
    out->band_cost_land[PT_TMP] = land[R_TMP]; out->band_cost_run[PT_TMP] = run[R_TMP];
}

// The cost of an rpm inside the eco band: the ramp from the cost at P1 (band_cost[PT_ECO]) to the cost at P2 (the power band's, eco_top_cost), by depth.
float pt_eco_cost(const pt_intent *it, const eng_bands *b, float rpm, int run)
{
    float lo = run ? it->band_cost_run[PT_ECO] : it->band_cost_land[PT_ECO], hi = run ? it->eco_top_cost_run : it->eco_top_cost_land;
    float d = b->p2 > b->p1 ? clamp01((rpm - b->p1) / (b->p2 - b->p1)) : 0.0f;
    return lo + (hi - lo) * d;
}

int pt_band_of(const eng_bands *b, float rpm, float cross)
{
    return bands_of(b, cross, rpm);
}
