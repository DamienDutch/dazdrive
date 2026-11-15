// Score-based gear choice (TrueShift, ini [trueshift] choice = 1): a pure function of the vehicle model and a few numbers.
// It knows no TrueShift state and no hard rule (limiter, stall, braking, engine brake, kickdown, hints, timers): the caller runs
// those first and asks this only "which of the gears I may take is best now". Header only, so a simulator can call it directly.
//
//   score(g) = - gas_cost(g) - rpm_cost(g) - shift_cost(g) + stay_bonus(g)
//
//   candidates   every forward gear within 1 + skips (up: skips_up) of the current one whose rpm (at the speed after the shift) lies in the
//                usable window: from idle + 60 % (coasting 30 %) of the way up to the torque plateau (V.plateau), to the top of the power range
//                (V.eng.range[R_POWER]) or 93 % of the limit, whichever is lower. A gear must also
//                be floor over the start of the plateau where the demanded acceleration takes the speed in 3 horizons: a steady
//                load does not sit under it, a truck that accelerates may land lower. With floor_pw the floor is instead the lowest rpm
//                where the engine still gives that share of its peak power (torque x rpm, from its curve), and a gear up must also stand on it
//                at the speed right after the shift (and at the speed of now), not only 3 horizons on. v_cap caps the speeds the demand is
//                followed to (the route announces a slowdown: no gear up that only fits a speed the truck will not reach). An upshift must land hyst above the lower edge,
//                a downshift hyst below the upper one (the hysteresis); the current gear may sit anywhere inside the window.
//   feasibility  share(g) = needed force / vehicle_force_at(g, v, 1): the throttle that gear needs for the demanded
//                acceleration (m*a + resistance). Over 1 the gear cannot make it: out. "Does it make it with a bit more
//                throttle" is the test, not "with reserve at the throttle of now". The gear we are in may be 10 % short (no flip back
//                at full throttle). A pull asked that no gear in the window can give counts as what the strongest one gives (never
//                less than holding the speed), or nothing would be left to choose from after a forced shift. A pedal near the floor
//                (pull > 0) asks that share of the strongest gear in the window, not of the gear we are in: a gear that gives less is out.
//   gas_cost     0 up to the comfort limit, then ((share - comfort) / (1 - comfort))^2: steep, 1 at share 1.
//   rpm_cost     w_rpm * ((rpm - plateau)+ / (window top - plateau))^2   far above the plateau: expensive revs
//               + w_rpm * u * ((plateau - rpm)+ / (plateau - idle))^2   under it, near idle with load: lugging
//                (u = the share of the power available at this speed that is asked). A guess: no efficiency map.
//                (times 1 while the truck accelerates, up to 4 at constant speed: the gear stays and the revs are paid for longer)
//   shift_cost   w_shift * shifts * shift_time * u: the drive time a shift loses (the engine's power asked, for the shift time).
//                shifts = the shift to g (one, however many gears it jumps) + the shifts g will need later because the speed
//                keeps rising at the demanded acceleration (a gear that is at the top of the window in a few seconds needs
//                another one). That is what makes one skip cheaper than two single shifts, and a slow box pay more per shift.
//                At a steady speed a gear that stands on the floor already pays it divided by up to 4 (it is held for long).
//   stay_bonus   the current gear gets a small bonus: no flip-flopping on near-equal scores.
//
// time_obj (choice_objective 1): when the score says "up", the gear to shift up to is the one that gains a speed step soonest instead:
//   time(g)      the shift now, then the speed from v_after up to where the truck is going (v_target: the cruise's set speed, the road's
//                limit, the truck's limiter; unknown: v + dv, dv = 5 km/h or what the current gear still has up to the top of the window) at
//                a(g) = the demanded acceleration, capped at what the gear gives at full throttle; at each gear's upshift point (up_g) one gear
//                up (a shift: shift_time without pull, and the speed lost in it). The next shift of the gear it ends in counts for the part of
//                that gear's range the step used up (no jump where a shift falls just outside the step). The shift seconds are the engine's own
//                (29-10: in the game a shift loses its shift_time and nothing more, Renault 1.1-1.25 s, Scania 0.81-0.88 s; the per-mode weights
//                that counted them 1.3 to 5 times made up for a model that saw too few shifts ahead, and went). Staying in the gear of now
//                is a choice too. A skip may win by saving a shift although it
//                pulls less, as long as it gives a_min at full throttle (when at least that is asked; else the whole demand, as before). The
//                gear we are in may be short the same way (no flip back after the skip). The window, the floor and the rpm cost stand.
//                Staying to wait for a skip that lands a little later (this gear still under the window's top by then) is one of the choices.
//                Not while less than a_min is asked (holding a speed): there the score alone decides.
#pragma once
#include "vehicle.h"
#include <math.h>
#include <string.h>

struct gs_cfg {
    float comfort;     // share of the full-throttle force above which the gas cost climbs (per mode)
    float w_rpm;       // weight of rpm_cost
    float w_shift;     // weight of shift_cost
    float stay;        // stay_bonus of the current gear
    float idle;        // idle rpm
    int skips;         // most gears a downshift may skip (a shift spans 1 + skips gears)
    int skips_up;      // the same for an upshift
    int skips_later;   // the same for the shifts a gear will need later (a skip that is not allowed this second, at the start of a pull-away, will be)
    float floor;       // rpm over the start of the torque plateau: the lowest the window reaches under load (0 = no such floor)
    int floor_cur;     // 1 = the current gear must stand on it too, 0 = only the gears a shift would land in (the pedal just came off)
    float hyst;        // rpm: an upshift must land this far above the window's lower edge, a downshift this far under its upper edge
    float horizon;     // s: how far ahead the demanded acceleration is followed for the shifts a gear will need later
    float learn_res;   // extra share an upshift must have in hand (what TrueShift learned on this grade)
    float pull;        // 0, or the pedal when it is near the floor: the pull asked is at least this share of the strongest gear in the window
    int time_obj;      // 1 = an upshift goes to the gear that gains speed soonest (see above), 0 = to the best score
    float a_min;       // time_obj: m/s2 a gear that is short of the demand must still give at full throttle (only when at least this much is asked)
    float floor_idle_frac;   // with floor_pw: the floor is at least idle + this share of (plateau - idle) (0 = the power term alone)
    float floor_pw;    // > 0: the floor is the lowest rpm with this share of the peak power (gs_power_rpm) instead of plateau + floor, also checked right after an upshift (0 where floor is 0)
    int cross;         // 1 = each gear's window top is the power crossover to the next gear where that lies above the top of the power range (a firm pedal, Power / Normal)
    float v_cap;       // > 0: m/s the route allows ahead; the speeds the demand is followed to (later shifts, the floor's look ahead for a gear up) stop there
    float up_pt;       // > 0: the mode's upshift point (rpm): revs over it get expensive fast (w_over); 0 = off
    float w_over;      // weight of that cost (0 = off)
    float w_land;      // weight of the landing cost: a gear that sits just above the floor under load pays (0 = off)
    float pw_t;        // > 0: the lowest rpm that still gives power_target_frac of the peak power (gs_power_rpm). At a firm pedal (pw_k) the window's top per gear is the rpm
                       // that lands in the next gear on it (never the engine's power range top and never above it), so a flat power curve is not chased up to its peak
    float pw_k;        // 0..1: how far the demand has moved the shift points to the power range (TrueShift's pw_k); the window top follows it
    float floor_plat;  // > 0: the floor of floor_pw is at most this share of the start of the torque plateau (a motor with full torque from 1000 rpm gets a floor at the plateau, not 1020 from a power share); the idle share stays its lower bound
    float cr_tq;       // > 0: cruise reserve: at a light load the floor and the window's lower edge move down to the lowest rpm where the torque is still this share of the peak torque
                       // (never under idle + 250); fades out as the power asked rises from cr_u0 to cr_u1 of the power available. 0 = off
    float cr_w;        // weight of the cruise cost: at a light, steady load revs above that cruise rpm cost this much per span from idle to the plateau (the top gear that fits wins)
    float cr_u0, cr_u1;
    float cr_band;     // > 0: the cruise rpm (and the floor of the cruise reserve) is at least idle + this share of (plateau - idle): a band per mode, the same feel in rpm on every truck
    float cr_fres;     // > 0: the cruise reserve also checks this force (N): holding the speed on the steepest grade of the hill memory (a rolling road keeps the gear under the top)
    float cr_res;      // > 0: the cruise cost's reserve: a gear needing over this share of its full-throttle pull (to + 0.25) loses its cruise advantage
    float v_target;    // > 0: m/s the truck is heading for (the cruise's set speed, else the road's limit): time_obj follows the speed up to it, every shift
                       // on the way counted; 0 = not known (then a step of the current gear's range, the shifts after it shared out by the part used)
    float wish_s0, wish_s1;   // > 0: the torque pedal's wish drives the throttle (modules/pedal.cpp: it compensates at low revs, a gear needing little of its full-throttle
                       // force still delivers the wish). A gear whose share of that force is under wish_s0 needs no floor (it stands on the window's lower edge, the
                       // smooth-running limit: idle + 0.6 x (plateau - idle)); over wish_s1 the floor is as without the wish; smooth between. The landing cost (a
                       // price on the gears just above the floor) is not touched: it prices a skip the same with and without the wish
    float up_g[33];    // > 0: the rpm where TrueShift shifts gear g up at this demand (its upshift point); time_obj follows the speed with these, as the
                       // truck will shift (the window's top where 0). A model that held every gear to the window's top saw too few shifts ahead and
                       // took single steps at a part pedal (sim 29-10, the V8 at 18 t, pedal 0.5 in Power: 7-8-9-10-11-12, five shifts)
};

enum { GS_MAX = 32 };
static_assert(sizeof(((gs_cfg *)0)->up_g) / sizeof(float) == GS_MAX + 1, "up_g holds one rpm per gear");

struct gs_term {
    int g;             // the gear
    int ok;            // 1 = a candidate (in the window and feasible)
    float rpm, share;  // rpm at the speed after the shift (now for the current gear); share of the full-throttle force needed
    float gas, rpmc, shiftc, stay, score;
    int shifts;        // shifts counted: now + later
    int in;            // in the window and on the floor (feasible or not)
    float t_gain;      // time_obj: the time to gain the speed step (s, shift seconds weighted), 0 = not estimated
};

struct gs_out {
    int n;             // terms filled (gears lo..hi)
    gs_term t[GS_MAX + 1];  // indexed by gear
    int best, second;  // gears with the highest and the second highest score among the candidates (0 = none)
    int best_score;    // the best by score alone (time_obj may take another gear up)
    int wait;          // time_obj: the best is to stay and wait for this skip (0 = no)
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

// The lowest rpm (from idle up) where the engine gives at least share of its peak power (torque x rpm: the curve's factor times the rpm;
// the peak is V.peak_pw, found once per engine read); a share over 1 counts as the peak itself
static inline float gs_power_rpm(float share, float idle)
{
    const float lim = V.eng.rpm_limit;
    float pk = V.peak_pw;
    if (!(pk > 0)) for (float r = idle; r <= lim; r += 5) pk = fmaxf(pk, curve_factor(&V.eng, r) * r);
    float want = fminf(share, 1.0f) * pk - 1e-3f;
    float best_r = idle, best_p = -1;
    for (float r = idle; r <= lim; r += 5) {
        float p = curve_factor(&V.eng, r) * r;
        if (p >= want) return r;
        if (p > best_p) { best_p = p; best_r = r; }
    }
    // (none reached it: the peak sits between two samples of this grid, which starts at the idle - a Crown Victoria at idle 823 missed its
    // peak at 4500 by 1 of 4320 and the share 1 came back as the idle itself: the whole power range collapsed to its floor; the best sample is the answer)
    return best_r;
}

// The floor of choice_floor_power*: the higher of the lowest rpm with that share of the peak power and a distance above idle that is a share of
// the engine's own span (idle + frac x (plateau - idle)), so an engine with a high plateau does not get a floor that sits close to idle.
// frac 0 = the power term alone. (floor_idle_frac)
static inline float gs_floor_rpm(float share, float idle, float frac, float plat = 0)
{
    float r = gs_power_rpm(share, idle);
    if (plat > 0) r = fminf(r, plat * V.plateau);   // (the power share asks for more than the torque plateau does: the plateau's start is enough)
    return frac > 0 ? fmaxf(r, idle + frac * fmaxf(0.0f, V.plateau - idle)) : r;
}

// The lowest rpm (from idle up) where the torque is at least this share of its peak: the engine's reserve for a steady drive
static inline float gs_torque_rpm(float share, float idle)
{
    for (float r = idle; r <= V.eng.rpm_limit; r += 5) if (curve_factor(&V.eng, r) >= share - 1e-3f) return r;
    return idle;
}

// The power crossover of gear g to g + 1: the rpm in g where the engine's power equals the power in g + 1 at the rpm it lands at after the shift
// (rpm x ratio(g+1) / ratio(g)), from the torque curve and the ratios. Never below base (the top of the power range), never above 95 % of the limit.
// Below it staying in g gives more power than the next gear, above it the next gear does.
static inline float gs_cross_rpm(int g, float v, float base, float lim)
{
    float r0 = vehicle_rpm_at(g, v), r1 = vehicle_rpm_at(g + 1, v);
    float cap = 0.95f * lim;
    if (!(r0 > 1) || !(r1 > 1) || base >= cap) return base;
    float rr = r1 / r0;
    for (float r = base; r <= cap; r += 5)
        if (curve_factor(&V.eng, r) * r <= curve_factor(&V.eng, r * rr) * r * rr) return r;
    return cap;
}

// time_obj: seconds from v0 in gear g (just shifted into) to v_t at the demanded acceleration a_d, capped per gear at full throttle; at the
// window's top hi one gear up, which costs k shift times and the speed f_res loses in it. The next shift of the last gear counts for the
// share of that gear's range used up. A gear that gives under 0.02 m/s2 counts as 0.02 (it is slow, not impossible).
static inline float gs_gain_time(int g, int n, float v0, float v_t, float a_d, float f_res, const float *hi, float ts)
{
    const float m = V.mass;
    float tt = ts, v = v0, v_in = v0;
    for (int i = 0; i < 2000 && v < v_t; i++) {
        if (g < n && vehicle_rpm_at(g, v) >= hi[g]) { g++; tt += ts; v = fmaxf(0.5f, v - f_res / m * ts); v_in = v; continue; }
        float a = fmaxf(0.02f, fminf(a_d, (vehicle_force_at(g, v, 1.0f) - f_res) / m)), step = fminf(0.25f, v_t - v);
        tt += step / a;
        v += step;
    }
    float vh = g < n ? vehicle_kmh_at(g, hi[g]) / 3.6f : 0;
    if (vh > v_in + 0.1f) tt += ts * fminf(1, fmaxf(0, (v - v_in) / (vh - v_in)));
    return tt;
}

// cur = the gear now, n = the number of forward gears, v = speed (m/s), need = the force asked (N, >= f_res), f_res = the
// resistance of now (N), coast = the pedal is off (nothing is asked: need counts 0).
static inline void gs_eval(gs_out *o, int cur, int n, float v, float need, float f_res, int coast, const gs_cfg &c)
{
    o->n = 0; o->best = o->second = o->best_score = o->wait = 0; o->cur_ok = 0;
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
    // cruise reserve: at a light load the window's lower edge and the floor may sit as low as the lowest rpm that still has the torque in hand
    float w_light = 0, r_cr = 0;
    if (c.cr_tq > 0 && fpeak > 1) {
        w_light = 1 - gs_smooth(c.cr_u0, c.cr_u1, fminf(1, need / fpeak));
        r_cr = fmaxf(gs_torque_rpm(c.cr_tq, idle), idle + 250);
        if (c.cr_band > 0) r_cr = fmaxf(r_cr, idle + c.cr_band * fmaxf(0, plateau - idle));   // cruise_band: the mode's own cruise rpm, a share of the way idle -> plateau
        float lo_cr = fmaxf(r_cr - c.hyst, idle + 150);
        if (lo_cr < o->lo) o->lo += (lo_cr - o->lo) * w_light;
    }
    for (int g = 1; g <= n; g++) o->hi_g[g] = fminf(V.eng.range[R_POWER][1], 0.93f * lim);
    if (c.cross) for (int g = 1; g < n; g++) o->hi_g[g] = fmaxf(o->hi_g[g], gs_cross_rpm(g, v, o->hi_g[g], lim));
    // power_target_frac: the top of a gear is where the next gear lands on the lowest rpm that still gives the share of the peak power. Only as far
    // as the truck is free to accelerate in that next gear (what it gives at full throttle over the resistance, from 0.15 to 0.40 m/s2): a truck that
    // struggles up a climb keeps its gear to the power crossover, an early shift only costs it speed there
    if (c.pw_t > 0 && c.pw_k > 0)
        for (int g = 1; g < n; g++) {
            float r0 = vehicle_rpm_at(g, v), r1 = vehicle_rpm_at(g + 1, v);
            if (!(r0 > 1 && r1 > 1)) continue;
            float w_cap = c.pw_k * gs_smooth(0.15f, 0.40f, (vehicle_force_at(g + 1, v, 1.0f) - f_res) / fmaxf(1.0f, m));
            float cap = fmaxf(c.pw_t * r0 / r1, o->lo + 200);
            o->hi_g[g] = o->hi_g[g] + (fminf(o->hi_g[g], cap) - o->hi_g[g]) * w_cap;
        }
    o->hi = o->hi_g[cur];
    int span = 1 + (c.skips_up > 0 ? c.skips_up : 0), span_dn = 1 + (c.skips > 0 ? c.skips : 0);
    int span_later = 1 + (c.skips_later > 0 ? c.skips_later : 0);
    int g0 = cur - span_dn < 1 ? 1 : cur - span_dn, g1 = cur + span > n ? n : cur + span;
    float v_after = fmaxf(0.5f, v - f_res / m * ts);          // speed once the shift is done
    // The pull asked can be more than any gear in the window gives (a pedal held in a low gear, then the limiter forces a shift up):
    // it counts as what the strongest of them can do, or nothing would be left to choose from.
    float fwin = 0;
    for (int g = g0; g <= g1; g++) {
        float vg = g == cur ? v : v_after, r = vehicle_rpm_at(g, vg);
        if (r >= o->lo + (g > cur ? c.hyst : 0) && r <= o->hi_g[g] - (g < cur ? c.hyst : 0)) fwin = fmaxf(fwin, vehicle_force_at(g, vg, 1.0f) / (g > cur ? 1 + c.learn_res : 1));
    }
    if (fwin > 0 && need > fwin) need = fmaxf(fwin, f_res);   // (holding the speed is never capped away: a gear that cannot hold it is out)
    // the pedal near the floor asks for the truck's pull, not for what the gear of now happens to give (it would give a gear under the
    // plateau share 1 and keep it: ride, 34.6 t in 10 at 809 rpm, full pedal, 0.3-0.5 m/s2 for 5 s)
    if (c.pull > 0 && !coast) need = fmaxf(need, c.pull * fwin);
    float a_d = fmaxf(0, (need - f_res) / m);                 // the acceleration demanded
    float v_f = v + a_d * c.horizon;                          // where the demand takes the speed
    if (c.v_cap > 0) v_f = fminf(v_f, fmaxf(c.v_cap, 0.5f));  // (not past what the route allows ahead)
    // The floor: a gear must be at plateau + floor (an upshift: + hyst) where the speed has taken it in three horizons. A steady load (the
    // speed hardly rises) must stand there now; a truck that accelerates may land lower, the speed carries it up. A shorter look
    // ahead takes the skips away from a pull-away, a gear that lands right on the floor is knocked off it by the demand falling after the
    // shift (hence the hysteresis too). The caller sets floor 0 where the window must stay low (coasting for a while, engine brake).
    // (floor_pw: the lowest rpm with that share of the peak power instead; a gear up must stand on it right after the shift as well, and
    // the route's cap holds the look ahead of a gear up, so a skip does not rest on a speed the route says the truck will not reach)
    float r_min = c.floor_pw > 0 ? gs_floor_rpm(c.floor_pw, idle, c.floor_idle_frac, c.floor_plat) : c.floor > 0 ? plateau + c.floor : 0, v_floor = v + a_d * 3 * c.horizon;
    if (r_min > 0 && w_light > 0 && r_cr < r_min) r_min += (r_cr - r_min) * w_light;
    float v_floor_up = c.v_cap > 0 ? fminf(v_floor, fmaxf(c.v_cap, 0.5f)) : v_floor;
    // (the hysteresis a gear up must stand above the floor: the plateau floor has it; the power floor is the driver's own number, a gear lands on it as it is)
    const float f_hyst = c.floor_pw > 0 ? 0.0f : c.hyst;
    const bool now_up = r_min > 0 && c.floor_pw > 0;          // a gear up is checked at the speed of now and right after the shift too
    // with the torque pedal's wish: the floor of a gear fades out as its share of the full-throttle force falls (the reserve it keeps for the
    // next push is its own share, not the revs); never under the window's lower edge. wish_k = 1 without the wish (or a gear that needs most of its pull)
    auto wish_k = [&](float share) { return c.wish_s1 > 0 ? gs_smooth(c.wish_s0, c.wish_s1, share) : 1.0f; };
    auto rmin_g = [&](float share) { float k = wish_k(share); return r_min > 0 && k < 1.0f ? fminf(r_min, o->lo) + (r_min - fminf(r_min, o->lo)) * k : r_min; };
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
        // the hill memory (cr_fres): at a light, steady load a gear UP must also hold the speed on the steepest climb of the last stretch, within
        // what its share allows, so a rolling road does not take it up at every crest only to bring it down on the next rise (sim 29-10, the V8
        // at 18 t on +-1 %: 14 -> 13 -> 14 every 25 s). Holding a gear and shifting down are not touched
        // (with a margin of 15 % of what the remembered climb asks over the load of now: the gear up must hold it with room to spare, or the
        // next rise takes it out again at the same line it went up on; on a road that is flat now and was flat before there is nothing to add)
        if (g > cur && c.cr_fres > 0 && w_light > 0 && f1 > 1) {
            float fm = c.cr_fres + 0.15f * fmaxf(0.0f, c.cr_fres - need);
            t->share = fmaxf(t->share, w_light * fm * (1 + c.learn_res) / f1);
        }
        bool in = t->rpm >= o->lo + (g > cur ? c.hyst : 0) && t->rpm <= o->hi_g[g] - (g < cur ? c.hyst : 0) &&
                  (g == cur && !c.floor_cur ? true : t->rpm * ((g > cur ? v_floor_up : v_floor) / vg) >= rmin_g(t->share) + (g > cur ? f_hyst : 0)) &&
                  (g <= cur || !now_up || (t->rpm >= rmin_g(t->share) && vehicle_rpm_at(g, v) >= rmin_g(t->share)));
        t->in = in;
        // time_obj: when at least a_min is asked, the gear we are in and a gear up may also be short if at full throttle they still give a_min:
        // the truck still gains speed usefully; the gas cost stays at that of share 1 (flat out), the time to gain speed weighs the pull it lacks
        bool short_ok = c.time_obj && g >= cur && !coast && a_d >= c.a_min && (f1 / (g > cur ? 1 + c.learn_res : 1) - f_res) / m >= c.a_min;
        if (!in || (t->share > (g == cur ? 1.1f : 1.0f) && !short_ok)) continue;   // the gear we are in may be a little short (10 %), a gear to shift to may not: no flip back at full throttle
        t->ok = 1;
        float x = fmaxf(0, t->share - c.comfort) / fmaxf(0.05f, 1 - c.comfort);
        if (short_ok) x = fminf(x, 1);
        t->gas = x * x;
        float xh = fmaxf(0, t->rpm - plateau) / fmaxf(1, o->hi_g[g] - plateau), xl = fmaxf(0, plateau - t->rpm) / fmaxf(1, plateau - idle);
        t->rpmc = c.w_rpm * hold * (xh * xh + u * xl * xl);
        // cruise cost: a light load at a steady speed (a cruise, a level road) is best in the highest gear that has the torque in hand, so revs over that
        // cruise rpm cost, in proportion to how light and how steady the load is (not while accelerating or coasting)
        // A gear that holds the load only with little reserve (it needs over 60-85 % of its own full-throttle pull) gets no such advantage: it
        // pays as if its revs were far over the cruise rpm. The reserve is torque against weight: a strong engine with a light load keeps its
        // overdrive, a weak one at 44 t does not lug in it (sim 29-10: Eco at 44 t, DC13 370 / DTI11 380, throttle over 0.8 under the plateau)
        if (c.cr_w > 0 && w_light > 0 && !coast)
            t->rpmc += c.cr_w * w_light * (1 - gs_smooth(0.02f, 0.08f, a_d)) *
                       fmaxf(fminf(1.5f, fmaxf(0, t->rpm - r_cr) / fmaxf(1, plateau - idle)), c.cr_res > 0 ? 1.5f * gs_smooth(c.cr_res, c.cr_res + 0.25f,
                             fmaxf(t->share, c.cr_fres > 0 && f1 > 1 ? c.cr_fres * (g > cur ? 1 + c.learn_res : 1) / f1 : 0.0f)) : 0.0f);   // (the reserve against the hill memory too)
        // landing cost: while the truck is asked to accelerate, a gear that sits just above the floor (the lowest rpm the window gives) pays, in
        // proportion to the power asked (u) and to how much speed is asked (a_d, from 0.05 to 0.4 m/s2): cruising, holding a speed and coasting
        // make it nothing (ride 7: the cruise at 83 km/h is meant to sit in 12 at 990 rpm; a climb is the lugging term's and the floor's). The zone is the whole way from the floor up to the upshift point (the
        // window's top where the caller gives none), so it follows the engine and the mode, not a fixed rpm
        if (c.w_land > 0 && u > 0) {
            float fl = r_min > 0 ? r_min : o->lo, top = c.up_pt > 0 ? c.up_pt : o->hi_g[g], zone = fmaxf(1, top - fl);
            float xf = fminf(1.0f, fmaxf(0, 1 - (t->rpm - fl) / zone));
            // (a pedal near the floor asks for pull and lets the revs go where they must: a hard launch still skips)
            t->rpmc += c.w_land * u * gs_smooth(0.05f, 0.4f, a_d) * xf * xf * (c.pull > 0 ? 1 - gs_smooth(0.8f, 0.95f, c.pull) : 1.0f);
        }
        // over the upshift point of the mode the revs get dear quickly: holding the gear is fine up to it, not far over it (a quarter of the way
        // from there to the window's top is the scale: at 1 the cost is w_over, as dear as a shift)
        if (c.w_over > 0 && c.up_pt > 0 && t->rpm > c.up_pt) {
            float xo = (t->rpm - c.up_pt) / fmaxf(1, 0.25f * (o->hi_g[g] - c.up_pt));
            t->rpmc += c.w_over * xo * xo;
        }
        int later = 0;                                         // shifts this gear needs while the speed rises to v_f
        if (v_f > v * 1.02f) {
            int k = 0;
            while (g + k < n && vehicle_rpm_at(g + k, v_f) > o->hi_g[g + k]) k++;
            later = (k + span_later - 1) / span_later;
        }
        t->shifts = (g != cur) + later;
        // a gear that stands on the floor at the speed of now, without the speed carrying it there, is held as long as the speed stays steady
        // (the demand accelerates less than 0.02-0.08 m/s2): its shift is paid over that time, like its revs (ride: 83 km/h on the cruise
        // control, 11 at 1262 rpm held 38 s against 12 at 990 rpm: the one shift weighed more than the revs). A gear that only reaches the
        // floor because the speed rises pays it whole (sim: 17 t at 45 km/h, 9 > 10 > 9 every 5 s when it did not)
        float amort = g != cur && r_min > 0 && t->rpm >= r_min ? 1 + 3 * (1 - gs_smooth(0.02f, 0.08f, a_d)) : 1;
        t->shiftc = c.w_shift * t->shifts * ts * u / amort;
        t->stay = g == cur ? c.stay : 0;
        t->score = -t->gas - t->rpmc - t->shiftc + t->stay;
        if (g == cur) o->cur_ok = 1;
        if (t->score > best) { second = best; o->second = o->best; best = t->score; o->best = g; }
        else if (t->score > second) { second = t->score; o->second = g; }
    }
    o->best_score = o->best;
    // time_obj: the score says up; which gear up gains the speed step soonest (the rpm cost stands, the shift seconds weigh as in shift_cost).
    // Also against waiting in this gear for a skip that does not land yet (under the window's edge or the floor): the speed v_s where it
    // would, if this gear gets there under the top of the window; then the shift. Waiting wins where the shift it saves outweighs the pull
    // the step up would give meanwhile (Eco, with its dearer shifts, waits most). Only while at least a_min is asked: holding a speed
    // or creeping up to it, the score alone decides (the time to gain speed means little there)
    if (c.time_obj && o->best > cur && !coast && a_d >= c.a_min) {
        float hu[GS_MAX + 1];   // where each gear will be shifted up: the upshift point the caller gives, never over the window's top
        for (int g = 0; g <= GS_MAX; g++) hu[g] = g <= n && c.up_g[g] > 0 ? fminf(c.up_g[g], o->hi_g[g]) : o->hi_g[g];
        float v_s[GS_MAX + 1] = {}, v_t = v + fmaxf(5 / 3.6f, vehicle_kmh_at(cur, o->hi_g[cur]) / 3.6f - v), loss = f_res / m * ts;
        if (c.v_target > v_t) v_t = c.v_cap > 0 ? fmaxf(v_t, fminf(c.v_target, c.v_cap)) : c.v_target;   // (to where the truck is going: a skip that saves a shift on the way wins it)
        for (int g = cur + 2; g <= g1; g++) {
            if (o->t[g].ok || o->t[g].rpm > o->hi_g[g]) continue;
            for (float vs = v + 0.25f; vehicle_rpm_at(cur, vs) < o->hi_g[cur] - 50; vs += 0.25f) {
                float va = fmaxf(0.5f, vs - loss), r = vehicle_rpm_at(g, va), f1 = vehicle_force_at(g, va, 1.0f) / (1 + c.learn_res);
                float vp = vs + a_d * 3 * c.horizon;
                if (c.v_cap > 0) vp = fminf(vp, fmaxf(c.v_cap, 0.5f));
                float rm = rmin_g(f1 > 1 ? need / f1 : 9.0f);
                if (r >= o->lo + c.hyst && r * (vp / va) >= rm + f_hyst && (!now_up || r >= rm) && (fminf(need, f1) - f_res) / m >= fminf(c.a_min, a_d)) {
                    v_s[g] = vs; v_t = fmaxf(v_t, vs + 5 / 3.6f);
                    break;
                }
            }
        }
        float jb = 1e30f;
        int pick = 0;
        for (int g = cur + 1; g <= g1; g++) {
            gs_term *t = &o->t[g];
            if (!t->ok) continue;
            t->t_gain = gs_gain_time(g, n, v_after, v_t, a_d, f_res, hu, ts);
            float j = c.w_shift * u * t->t_gain + t->rpmc;
            if (j < jb) { jb = j; pick = g; }
        }
        // and staying: with the target known, the gear of now may get there without the shift (or with the same shifts later), the shift
        // now then only costs its seconds (sim 29-10, Volvo D16 750 at 18 t, pedal 0.5 in Power: 12 -> 13 at 76 km/h, 80 the goal)
        // (the gear of now may run to the window's top: whether to shift before it is what is being asked)
        if (c.v_target > 0 && o->cur_ok) {
            float hs[GS_MAX + 1];
            memcpy(hs, hu, sizeof hs); hs[cur] = o->hi_g[cur];
            float j = c.w_shift * u * (gs_gain_time(cur, n, v, v_t, a_d, f_res, hs, ts) - ts) + o->t[cur].rpmc;
            if (j < jb) { jb = j; pick = cur; }
        }
        for (int g = cur + 2; g <= g1 && pick && o->cur_ok; g++) {
            if (!(v_s[g] > 0)) continue;
            float tw = 0;   // in this gear up to v_s
            for (float x = v; x < v_s[g]; x += 0.25f) tw += fminf(0.25f, v_s[g] - x) / fmaxf(0.02f, fminf(a_d, (vehicle_force_at(cur, x, 1.0f) - f_res) / m));
            float j = c.w_shift * u * (tw + gs_gain_time(g, n, fmaxf(0.5f, v_s[g] - loss), v_t, a_d, f_res, hu, ts)) + o->t[cur].rpmc;
            if (j < jb) { jb = j; pick = cur; o->wait = g; }
        }
        if (pick && pick != o->best) { o->second = o->best; o->best = pick; }
    }
}
