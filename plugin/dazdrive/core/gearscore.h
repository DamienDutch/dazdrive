// Score-based gear choice (TrueShift, ini [trueshift] choice = 1): a pure function of the vehicle model and a few numbers.
// It knows no TrueShift state and no hard rule (limiter, stall, braking, engine brake, kickdown, hints, timers): the caller runs
// those first and asks this only "which of the gears I may take is best now". Header only, so a simulator can call it directly.
//
//   score(g) = - gas_cost(g) - rpm_cost(g) - shift_cost(g) + stay_bonus(g)
//
//   candidates   every forward gear within 1 + skips of the current one whose rpm (at the speed after the shift) lies in the
//                usable window: from idle + 60 % (coasting 30 %) of the way up to the torque plateau (V.plateau), to the top of the power range
//                (V.eng.range[R_POWER]) or 93 % of the limit, whichever is lower. An upshift must land hyst above the lower edge,
//                a downshift hyst below the upper one (the hysteresis); the current gear may sit anywhere inside the window.
//   feasibility  share(g) = needed force / vehicle_force_at(g, v, 1): the throttle that gear needs for the demanded
//                acceleration (m*a + resistance). Over 1 the gear cannot make it: out. "Does it make it with a bit more
//                throttle" is the test, not "with reserve at the throttle of now". The gear we are in may be 10 % short (no flip back
//                at full throttle). A pull asked that no gear in the window can give counts as what the strongest one gives (never
//                less than holding the speed), or nothing would be left to choose from after a forced shift.
//   gas_cost     0 up to the comfort limit, then ((share - comfort) / (1 - comfort))^2: steep, 1 at share 1.
//   rpm_cost     w_rpm * ((rpm - plateau)+ / (window top - plateau))^2   far above the plateau: expensive revs
//               + w_rpm * u * ((plateau - rpm)+ / (plateau - idle))^2   under it, near idle with load: lugging
//                (u = the share of the power available at this speed that is asked). A guess: no efficiency map.
//                (times 1 while the truck accelerates, up to 4 at constant speed: the gear stays and the revs are paid for longer)
//   shift_cost   w_shift * shifts * shift_time * u: the drive time a shift loses (the engine's power asked, for the shift time).
//                shifts = the shift to g (one, however many gears it jumps) + the shifts g will need later because the speed
//                keeps rising at the demanded acceleration (a gear that is at the top of the window in a few seconds needs
//                another one). That is what makes one skip cheaper than two single shifts, and a slow box pay more per shift.
//   stay_bonus   the current gear gets a small bonus: no flip-flopping on near-equal scores.
#pragma once
#include "vehicle.h"
#include <math.h>

struct gs_cfg {
    float comfort;     // share of the full-throttle force above which the gas cost climbs (per mode)
    float w_rpm;       // weight of rpm_cost
    float w_shift;     // weight of shift_cost
    float stay;        // stay_bonus of the current gear
    float idle;        // idle rpm
    int skips;         // most gears an upshift may skip (a shift spans 1 + skips gears, up or down)
    float hyst;        // rpm: an upshift must land this far above the window's lower edge, a downshift this far under its upper edge
    float horizon;     // s: how far ahead the demanded acceleration is followed for the shifts a gear will need later
    float learn_res;   // extra share an upshift must have in hand (what TrueShift learned on this grade)
};

enum { GS_MAX = 32 };

struct gs_term {
    int g;             // the gear
    int ok;            // 1 = a candidate (in the window and feasible)
    float rpm, share;  // rpm at the speed after the shift (now for the current gear); share of the full-throttle force needed
    float gas, rpmc, shiftc, stay, score;
    int shifts;        // shifts counted: now + later
};

struct gs_out {
    int n;             // terms filled (gears lo..hi)
    gs_term t[GS_MAX + 1];  // indexed by gear
    int best, second;  // gears with the highest and the second highest score among the candidates (0 = none)
    int cur_ok;        // the current gear is a candidate
    float lo, hi;      // the usable window
    float hi_g[GS_MAX + 1];  // its top per gear (the same for every gear today)
};

static inline float gs_smooth(float e0, float e1, float x) { float t = x <= e0 ? 0 : x >= e1 ? 1 : (x - e0) / (e1 - e0); return t * t * (3 - 2 * t); }

static inline float gs_peak_force(float v)
{
    float f = 0;
    for (int g = 1; g <= T.fwd_gears && g <= GS_MAX; g++) f = fmaxf(f, vehicle_force_at(g, v, 1.0f));
    return f;
}

// cur = the gear now, n = the number of forward gears, v = speed (m/s), need = the force asked (N, >= f_res), f_res = the
// resistance of now (N), coast = the pedal is off (nothing is asked: need counts 0).
static inline void gs_eval(gs_out *o, int cur, int n, float v, float need, float f_res, int coast, const gs_cfg &c)
{
    o->n = 0; o->best = o->second = 0; o->cur_ok = 0;
    for (int g = 0; g <= GS_MAX; g++) o->t[g] = gs_term();
    const float m = V.mass, ts = V.eng.shift_time, idle = c.idle;
    const float plateau = V.plateau, lim = V.eng.rpm_limit;
    o->lo = idle + (coast ? 0.3f : 0.6f) * fmaxf(0, plateau - idle);   // (coasting: no load, so no lugging: the revs may sag further before a gear is wrong)
    if (o->lo < idle + 150) o->lo = idle + 150;                           // (TrueShift's own stall rule takes over at idle + 120)
    o->hi = 0;
    if (n > GS_MAX) n = GS_MAX;
    if (cur < 1 || cur > n || !(m > 0) || !(v > 0.1f)) return;
    if (coast) need = 0;
    // the window's top: the top of the power range, never above 93 % of the limit (the rpm cost, not the window, keeps the revs down)
    float fpeak = gs_peak_force(v);
    for (int g = 1; g <= n; g++) o->hi_g[g] = fminf(V.eng.range[R_POWER][1], 0.93f * lim);
    o->hi = o->hi_g[cur];
    int span = 1 + (c.skips > 0 ? c.skips : 0);
    int g0 = cur - span < 1 ? 1 : cur - span, g1 = cur + span > n ? n : cur + span;
    float v_after = fmaxf(0.5f, v - f_res / m * ts);          // speed once the shift is done
    // The pull asked can be more than any gear in the window gives (a pedal held in a low gear, then the limiter forces a shift up):
    // it counts as what the strongest of them can do, or nothing would be left to choose from.
    float fwin = 0;
    for (int g = g0; g <= g1; g++) {
        float vg = g == cur ? v : v_after, r = vehicle_rpm_at(g, vg);
        if (r >= o->lo + (g > cur ? c.hyst : 0) && r <= o->hi_g[g] - (g < cur ? c.hyst : 0)) fwin = fmaxf(fwin, vehicle_force_at(g, vg, 1.0f) / (g > cur ? 1 + c.learn_res : 1));
    }
    if (fwin > 0 && need > fwin) need = fmaxf(fwin, f_res);   // (holding the speed is never capped away: a gear that cannot hold it is out)
    float a_d = fmaxf(0, (need - f_res) / m);                 // the acceleration demanded
    float v_f = v + a_d * c.horizon;                          // where the demand takes the speed
    float u = fpeak > 1 ? fminf(1, need / fpeak) : 1;         // demanded power over the power available at this speed
    // a gear that is held for a long time (the speed hardly changes) pays its revs for longer than one the speed runs away from: up to 4 times
    float hold = 1 + 3 * (1 - gs_smooth(0.02f, 0.3f, a_d));
    float best = -1e30f, second = -1e30f;
    for (int g = g0; g <= g1; g++) {
        gs_term *t = &o->t[g];
        *t = gs_term();
        t->g = g;
        o->n++;
        float vg = g == cur ? v : v_after;
        t->rpm = vehicle_rpm_at(g, vg);
        float f1 = vehicle_force_at(g, vg, 1.0f);
        float fn = need * (g > cur ? 1 + c.learn_res : 1);
        t->share = f1 > 1 ? fn / f1 : 9;
        bool in = t->rpm >= o->lo + (g > cur ? c.hyst : 0) && t->rpm <= o->hi_g[g] - (g < cur ? c.hyst : 0);
        if (!in || t->share > (g == cur ? 1.1f : 1.0f)) continue;   // the gear we are in may be a little short (10 %), a gear to shift to may not: no flip back at full throttle
        t->ok = 1;
        float x = fmaxf(0, t->share - c.comfort) / fmaxf(0.05f, 1 - c.comfort);
        t->gas = x * x;
        float xh = fmaxf(0, t->rpm - plateau) / fmaxf(1, o->hi_g[g] - plateau), xl = fmaxf(0, plateau - t->rpm) / fmaxf(1, plateau - idle);
        t->rpmc = c.w_rpm * hold * (xh * xh + u * xl * xl);
        int later = 0;                                         // shifts this gear needs while the speed rises to v_f
        if (v_f > v * 1.02f) {
            int k = 0;
            while (g + k < n && vehicle_rpm_at(g + k, v_f) > o->hi_g[g + k]) k++;
            later = (k + span - 1) / span;
        }
        t->shifts = (g != cur) + later;
        t->shiftc = c.w_shift * t->shifts * ts * u;
        t->stay = g == cur ? c.stay : 0;
        t->score = -t->gas - t->rpmc - t->shiftc + t->stay;
        if (g == cur) o->cur_ok = 1;
        if (t->score > best) { second = best; o->second = o->best; best = t->score; o->best = g; }
        else if (t->score > second) { second = t->score; o->second = g; }
    }
}
