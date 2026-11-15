// The driver's demand, one source (core/wish.h). Two parts:
//  - wish_get: the composition TrueShift's gear_layers had: the torque pedal's demand, else the cruise's force wish, else the caller's fallback, 0 while
//    the pedal is off; with the filtered wish and the pedal's rate, same constants, moved here.
//  - the torque pedal's demand (package P, the pedal's map P6, P6b, P6c): the shaping (the mode's map s = (1 + c) p^k / (p^k + c) and the rise limit), the force envelope, the engine-state
//    factor phi and the effective demand. The pedal module is the delivery: it decides when the throttle passes through and turns the demand into a throttle.
#include "wish.h"
#include "suite.h"
#include "vehicle.h"
#include "speedtarget.h"
#include "bands.h"
#include "driver.h"   // DRV_OFF: the pedal below which the gearbox reads no wish (a constant only: nothing of the driver profile is linked)
#include <math.h>

static float w_clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void wish_get(const wish_args *in, float *ws, double *ws_t, wish_t *out)
{
    const float v = in->v;
    float f = -1.0f;
    int src = WISH_NONE;
    if (!in->coast && in->use_pedal) {
        f = wish_level_at(v);                    // the pedal's level at this speed, gear-independent (the horizon, the need), held under what a cap
        if (f >= 0) { f *= wish_limit_k(); src = WISH_PEDAL; }   // or a speed limiter lets through
    }
    if (!(f >= 0) && !in->coast) {
        f = cruise_wish_ahead(v);
        if (f >= 0) src = WISH_CRUISE;
    }
    if (!(f >= 0)) f = in->coast ? 0.0f : in->fallback_n;
    out->force_n = f;
    out->source = src;
    out->share = src == WISH_PEDAL ? wish_share_eff() : src == WISH_CRUISE ? cruise_wish_share() : NAN;

    // the wish filtered over a few shift times (rising) and three times that (falling), for the need: a stab at the pedal after a shift is no reason
    // to change the band, and the power band is kept for a while once the pedal lets go (the relief after the downshift is not 'easy again')
    {
        const double dt = in->now - *ws_t;
        const float tau = w_clamp(4.0f * V.eng.shift_time, 1.5f, 5.0f);
        if (dt > 10.0 || dt < 0) *ws = fmaxf(f, 0.0f);
        else if (dt > 0) *ws += (fmaxf(f, 0.0f) - *ws) * (float)(1.0 - exp(-dt / (f > *ws ? tau : 3.0f * tau)));   // up in ~4 shift times, down in ~12: the power band is kept a while
        *ws_t = in->now;
    }
    out->slow_n = *ws;
}

// ---------------------------------------------------------------- the torque pedal's demand
// The shaping's configuration: the pedal module hands it over from its [pedal] values (core has no module dependency, the ini keys keep their names).
// The initial values are the module's defaults (pedal.cpp), for a sim that links the core without the module.
static wish_cfg C = { 0.365f, 0.295f, 0.245f, 1.47f, 1.5f, 3.0f, 8.0f, 0.8f, 0.75f };
void wish_configure(const wish_cfg *c)
{
    if (!c) return;
    C = *c;
    C.rev_gain = w_clamp(C.rev_gain == C.rev_gain ? C.rev_gain : 0.0f, 0.0f, 1.0f);
    // c > 0 is the map; at c -> 0 it is a step (s = 1 at any pedal), at c -> infinity the diagonal: 0.05 (the slope at the pedal's start is 21) and 1000 (the diagonal to 0.1 %) are its ends
    C.curve_eco = w_clamp(C.curve_eco == C.curve_eco ? C.curve_eco : 1000.0f, 0.05f, 1000.0f);
    C.curve_normal = w_clamp(C.curve_normal == C.curve_normal ? C.curve_normal : 1000.0f, 0.05f, 1000.0f);
    C.curve_power = w_clamp(C.curve_power == C.curve_power ? C.curve_power : 1000.0f, 0.05f, 1000.0f);
    // k >= 1: 1 is the rational map of P6b (a finite slope at the start), above 1 the slope starts at 0 (the toe); 6 is far past anything that keeps the top of the pedal adding
    C.curve_toe = w_clamp(C.curve_toe == C.curve_toe ? C.curve_toe : 1.0f, 1.0f, 6.0f);
    C.grade_assist = w_clamp(C.grade_assist == C.grade_assist ? C.grade_assist : 0.0f, 0.0f, 2.0f);
}

static struct {
    float s;          // the shaped share this tick: the pedal to the mode's exponent, after the rise limit
    double on_at;     // sim_s() of the last tick that gave a demand, < 0 = none (the pedal goes through)
    float v;          // the speed of that tick (m/s)
    int gear;         // and the gear
    int shaped;       // the shaping ran this tick (s is the shaped share, not the raw pedal): the launch reads it (wish_launch_share)
    float fade;       // the shaped share of the pedal DRV_OFF in this mode: the grade assist fades in over the share up to it (level_of)
    float base;       // the grade assist's base after its rate limit (N): at most the pedal's rise share x F_env per second, up and down
} S = { 0, -1, 0, 0, 0, 0, 0 };

// Hand over to the pass-through: the state follows the pedal, so the demand starts from where the pedal is, not from zero. The filtered base starts at the base of the grade now
// (not at zero): the demand comes back onto the grade's own level, with no hole under the pedal (P1b's filter is about changes while the demand is live).
static float grade_force();
static void follow(float p)
{
    S.s = p == p ? p : 0;
    S.on_at = -1;
    S.shaped = 0;
    S.base = grade_force();
}
void wish_follow(float p) { follow(p); }
// Another truck, a trailer or a load, a hard break: the shaping starts over (no per-truck state: the pedal is the same curve in every truck).
void wish_reset(int) { follow(0); }

// ---- the envelope F_env(v)
// The usable window: the engine bands' [lug floor, cap] (core/bands.h), every rpm a gear may sit at, the same range the gear layers choose within, so the
// pedal never asks for a force the choice cannot place. Without sane bands (a missing or flat torque curve) the old window: from idle + 60 % of the way
// up to the torque plateau (at least idle + 150) to the top of the power range or 93 % of the limit.
static void env_window(float *lo, float *hi)
{
    const eng_bands *b = bands_now();
    if (b->ok) { *lo = b->floor; *hi = b->cap; return; }
    float idle = ts_idle_rpm();
    *lo = fmaxf(idle + 0.6f * fmaxf(0.0f, V.plateau - idle), idle + 150);
    *hi = fminf(V.eng.range[R_POWER][1], 0.93f * V.eng.rpm_limit);
}

// The strongest in-window gear at v, either as the force it gives (per_phi 0) or in the units of the demand (per_phi 1): F_g / phi(rpm_g).
static float env_at(float v, int per_phi)
{
    if (!V.eng_ok || V.electric || !(v == v) || !(V.eta > 0)) return 0;
    int sg = ts_start_gear(0.3f);   // the start gear for a light pedal: the gear the truck pulls away in
    if (!(sg > 0)) return 0;
    // (no usable bands and no plateau either: the engine data does not look sane, the old window has nothing to stand on)
    if (!bands_now()->ok && !(V.plateau > 0)) return 0;
    float lo, hi;
    env_window(&lo, &hi);
    // Under the speed where the start gear reaches the floor no gear sits in the window as a locked driveline would have it (the clutch slips in the start
    // gear): the envelope is held at its value there, what the truck can do at walking pace. It is continuous across that speed by construction.
    float vlo = vehicle_kmh_at(sg, lo) / 3.6f;
    if (vlo > 0 && v < vlo) v = vlo;
    float best = 0;
    for (int pass = 0; pass < 2 && !(best > 0); pass++) {
        // pass 0: the gears whose revs lie in the window (1 rpm of slack: the held speed is the start gear's floor speed, whose rpm comes back from the
        // inverse a rounding under the floor). pass 1, only when none does (over the top gear's cap): any gear under the rev limit (its force is 0 over it),
        // so the demand does not vanish
        for (int g = 1; g <= T.fwd_gears; g++) {
            float f = vehicle_force_at(g, v, 1.0f);
            if (pass == 0) { float r = vehicle_rpm_at(g, v); if (!(r >= lo - 1.0f && r <= hi)) continue; }
            if (per_phi) {
                float ph = wish_phi(vehicle_engine_rpm_at(g, v));
                if (!(ph > 0)) continue;
                f /= ph;
            }
            best = fmaxf(best, f);
        }
    }
    return best;
}
// F_env(v) in the units of the demand: max over the in-window gears of F_g / phi(rpm_g). Each gear is asked F_d(g) = s x F_env x phi(rpm_g), so at s = 1 every
// in-window gear is asked at least its own full-throttle force: gas 1 in EVERY gear at any rev_gain (driver promise P1), launch under P1 included, while the
// level s x F_env stays one number for all gears (promise P4). rev_gain 0: phi = 1, the strongest force there is (today's). rev_gain 1: F_g / phi is
// c x P(P1) / v for every gear, the envelope is the constant-power hyperbola at P1's power and the throttle is the shaped share, gas == s.
float wish_env(float v) { return env_at(v, 1); }
// The strongest force the in-window gears can put on the road at v, in newtons (phi = 1): what a controller's full wish asks (the cruise, the planner: they want
// a force and have no foot), and what the envelope was before it was put in the units of the demand.
float wish_force_max(float v) { return env_at(v, 0); }
float wish_full(float v) { return wish_force_max(v); }

// ---- phi(rpm)
// The engine's power at an rpm in any constant unit (only ratios are used): torque curve x rpm
static float eng_power(float rpm) { return rpm > 0 ? curve_factor(&V.eng, rpm) * rpm : 0.0f; }
float wish_phi(float rpm)
{
    const eng_bands *b = bands_now();
    if (!b->ok || !(C.rev_gain > 0) || !V.eng_ok) return 1.0f;
    float p1 = eng_power(b->p1);
    if (!(p1 > 0) || !(rpm == rpm)) return 1.0f;
    return powf(eng_power(rpm) / p1, C.rev_gain);
}

// ---- the pedal's map (owner 03-11, P6b; the toe P6c; one family P6d)
// s = (1 + c) p^k / (p^k + c) per mode, k the same in every mode: 0 -> 0 and 1 -> 1 exactly (the same product over the same sum at p = 1), monotone (the slope k c (1 + c) p^(k-1) / (p^k + c)^2 is
// positive) and, for one k, a smaller c is livelier at every pedal, so Power >= Normal >= Eco wherever c falls from Eco to Power. k = 1 is P6b's rational map p (1 + c) / (p + c) to the digit: a
// finite slope (1 + c) / c at the start (a lively light pedal; the pedal that holds 80 km/h on the flat with 17 t sits at 0.08 in Normal). k > 1 puts a toe in front: the slope starts at 0, a foot
// resting on the pedal asks next to nothing, the lower third of the travel, where a cruising speed is held, can be dosed, and the pull comes after it, so that the pedal 0.6 stays as lively as
// before. The end slope is k c / (1 + c) (the top of the travel still adds: the suite asks for 0.25). Three ready-made sets, measured in docs/powerband-change.md "P6c" and "P6d": P6b (k 1, c 0.8 / 0.6 / 0.4),
// between (k 1.47, c 0.365 / 0.295 / 0.245: the average of the two curves within 0.010 from pedal 0.15 up) and P6c (k 2.15, c 0.165 / 0.14 / 0.135: the hold pedal at 0.19 to 0.21, the owner's own ride 0.19 to 0.28).
// The shipped defaults are the module's ([pedal] curve_*, pedal.cpp): 'between' (owner 03-11, P6d).
static float map_of(float p, int md)
{
    const float c = md == MODE_ECO ? C.curve_eco : md == MODE_POWER ? C.curve_power : C.curve_normal;
    const float pk = C.curve_toe == 1.0f ? p : powf(p, C.curve_toe);   // (k = 1: P6b to the digit; p^k at p = 1 is exactly 1)
    return (1.0f + c) * pk / (pk + c);
}

float wish_map(float p, int md) { return map_of(p < 0 ? 0.0f : p > 1 ? 1.0f : p == p ? p : 0.0f, md); }

// The inverse of the map: s = (1 + c) p^k / (p^k + c)  =>  p^k = c s / (1 + c - s). 0 -> 0, 1 -> 1; monotone like the map.
float wish_unmap(float s, int md)
{
    if (!(s == s) || s <= 0.0f) return 0.0f;
    if (s >= 1.0f) return 1.0f;
    const float c = md == MODE_ECO ? C.curve_eco : md == MODE_POWER ? C.curve_power : C.curve_normal;
    const float pk = c * s / (1.0f + c - s);
    const float p = C.curve_toe == 1.0f ? pk : powf(pk, 1.0f / C.curve_toe);
    return p > 1.0f ? 1.0f : p;
}

// ---- one tick of the shaping
static void base_filter(float dt, int md, float env);
int wish_tick(float p, float dt, int md, float v, int gear, int why)
{
    p = fminf(1.0f, fmaxf(0.0f, p));
    if (why == WT_OFF) { follow(p); return 0; }

    // the map (owner 03-11): per mode (map_of), the same in every truck (the envelope already scales per truck), then the rise limit (the 'delay'; falls at
    // once, a flat pedal is kickdown and never held back). One pedal definition everywhere: no kick, no per-truck factor, no second curve in the cruise.
    float rise = md == MODE_ECO ? C.rise_eco : md == MODE_POWER ? C.rise_power : C.rise_normal;
    float q = map_of(p, md);
    S.s = p >= 1.0f || q <= S.s ? q : fminf(q, S.s + fmaxf(rise, 0.0f) * dt);
    S.fade = map_of(DRV_OFF, md);
    S.shaped = 1;

    // standing in a forward gear or the launch: the share is shaped for the pull-away to read (wish_launch_share), but the throttle passes through
    // (the clutch does the pulling, the game's throttle is the raw pedal)
    if (why == WT_SHAPE) { S.on_at = -1; return 0; }

    // the demand at this speed, and whether the vehicle model can put it into a throttle in this gear
    const float env = wish_env(v);
    int ok = vehicle_force_at(gear, v, 1.0f) > 0 && env > 0;
    if (why == WT_SHIFT) {   // a shift: the delivered throttle goes through (only it may be held); the demand stays
        if (ok) { S.v = v; S.gear = gear; base_filter(dt, md, env); }   // the gear is still known: refresh what the demand reads, else keep the last
        S.on_at = sim_s();
        return 0;
    }
    if (!ok) { follow(p); return 0; }   // e.g. over the rev limit: the old path, as a pass-through
    S.v = v; S.gear = gear; S.on_at = sim_s();
    base_filter(dt, md, env);
    return 1;
}

// ---- the grade assist (P6.3; owner 02-11, keuzes 16 + 22)
// On a climb the same pedal gives less acceleration than on the flat; the assist lifts the BOTTOM of the pedal's range by what the hill takes, never the top. It is a remap, not an addition:
//   base  = min(grade_assist x m g sin(grade), F_env(v))     0 on the flat and downhill, and 0 when the mass is the fallback and not the game's (V.mass_ok)
//   level = base + s x (F_env - base)                         s = 1 is F_env: a full pedal is the full truck on any grade; with base 0 it is s x F_env to the digit
// Written as s x F_env + (1 - s) x base, which is exact at s = 1 and at base = 0 in float. The grade is the model's (V.grade_deg, the one vehicle_resist reads), not the route's: the gearbox acts
// on demands and the road ahead is the expect term's. One deviation from the sentence above, for the pedal at rest: at s = 0 it would give level = base, a foot OFF the pedal that still pulls on
// a hill (and a step from nothing to the hill's force at the first touch). The base is therefore faded in over the lift zone: x min(1, s / S.fade), S.fade = the share of the pedal DRV_OFF (0.05,
// core/driver.h: the pedal below which TrueShift's coast_p reads no wish) in this mode. A lift stays a lift (level 0 at s = 0), the level is continuous, and from the end of the lift zone
// up the remap is the one above to the digit. No new number: the lift zone is the one the gearbox already has.
static float grade_force()
{
    if (!(C.grade_assist > 0) || !V.mass_ok || !(V.mass > 0)) return 0;
    const float f = C.grade_assist * V.mass * 9.81f * sinf(V.grade_deg * 0.0174533f);
    return f > 0 ? f : 0;
}
// The base's filter (promise P1b). The base is read into the level as (1 - s) x base, so for a steady share a change of the base appears as (1 - s) times the change of the level:
// at s = 0, where the base is the whole level, bounding the base by rise(mode) x F_env(v) x dt bounds the level by exactly what the pedal's own rise limit allows in that tick, and at
// s > 0 the level can only change less. The bounded quantity is therefore the base itself, moving no faster than that allowance toward min(grade_force, F_env), up and down. It is a clamp
// and not a decay: on a steady grade it reaches the target exactly and the result is today's, while s = 1 still gives exactly F_env (the base has no weight there).
static void base_filter(float dt, int md, float env)
{
    const float target = fminf(grade_force(), env);
    if (!(env > 0) || !(dt > 0)) { S.base = target; return; }
    const float rise = md == MODE_ECO ? C.rise_eco : md == MODE_POWER ? C.rise_power : C.rise_normal;
    const float lim = fmaxf(rise, 0.0f) * env * dt;
    if (!(lim > 0)) { S.base = target; return; }
    S.base = S.base < target ? fminf(target, S.base + lim) : fmaxf(target, S.base - lim);
}
static float level_of(float s, float env)
{
    // A mass the game did not tell, or the assist off, is no basis for a base: the unassisted level to the digit (as before the filter).
    float base = C.grade_assist > 0 && V.mass_ok && V.mass > 0 ? fminf(S.base, env) : 0.0f;
    if (S.fade > 0 && s < S.fade) base *= s / S.fade;
    return s * env + (1.0f - s) * base;
}

// ---- reading it
static int live()
{
    double now = sim_s();
    return S.on_at >= 0 && now >= S.on_at && now - S.on_at <= 0.3 && V.eta > 0;
}
float wish_share() { return live() ? S.s : NAN; }
float wish_level_at(float v) { return live() ? level_of(S.s, wish_env(v)) : -1.0f; }
float wish_level_shaped(float v) { return level_of(S.s, wish_env(v)); }
float wish_level() { return wish_level_at(S.v); }
float wish_in(int g, float v)
{
    if (!live() || g < 1) return -1.0f;
    return level_of(S.s, wish_env(v)) * wish_phi(vehicle_engine_rpm_at(g, v));
}
float wish_launch_share(float raw) { return S.shaped ? S.s : raw; }

// The effective demand: what the gear of the tick is asked, held under what can reach the road while something holds the truck back:
//  - an actuator cap or scale on the throttle (the own speed limiter's cap, the rollover guard's): act_thr_limit() x F_cur;
//  - a speed limiter, ours or the game's while it is on (speed_top_kmh): the truck cannot gain speed at the limit, so the demand that is left is the force
//    that closes the gap to it within one shift, f_lim = max(f_res, 0) + m x max(0, top - v) / dur (farther from the limit the full pull is still all of
//    it, at the limit only the force that holds the speed is left). The game's own limiter acts after our merge and is not an actuator cap, which is why
//    it is read here by the speed. dur is the shift's own time (a converter's 0.3 s); gear_layers (TrueShift) also takes its learned estimate when longer.
//    [the limiter top lives in the model layer now (core/speedtarget.h), read through speed_top_kmh.]
// With nothing holding it the effective demand is F_d itself: a flat pedal in a gear too weak for it is a big demand (kickdown), not the little the gear has;
// the clip to the gear's full throttle is the delivery's, not the demand's. Read at call time: the owners' caps are held until they change.
float wish_limit_force(float v, float dur)
{
    const float top = speed_top_kmh() / 3.6f;
    if (!(top > 1.0f) || !(V.mass > 0) || !(dur > 0)) return 1e30f;
    return fmaxf(vehicle_resist(v), 0.0f) + V.mass * fmaxf(0.0f, top - v) / dur;
}
static int effective_parts(float *fd, float *eff)
{
    if (!live()) return 0;
    float fcur = vehicle_force_at(S.gear, S.v, 1.0f), lim = act_thr_limit();
    *fd = level_of(S.s, wish_env(S.v)) * wish_phi(vehicle_engine_rpm_at(S.gear, S.v));
    float e = *fd;
    if (fcur > 0 && lim < 1.0f) e = fminf(e, lim * fcur);
    e = fminf(e, wish_limit_force(S.v, V.tc ? 0.3f : V.eng.shift_time));
    *eff = e;
    return 1;
}
float wish_effective() { float fd, e; return effective_parts(&fd, &e) ? e : -1.0f; }
float wish_demand() { float fd, e; return effective_parts(&fd, &e) ? fd : -1.0f; }   // the pedal's own demand F_d, before any cap or limiter holds it down
float wish_limit_k() { float fd, e; return effective_parts(&fd, &e) && fd > 0 ? fminf(1.0f, e / fd) : 1.0f; }
// The effective SHAPED share: the pedal's share held under what the caps and the limiter let through (s x wish_limit_k). Gear-independent on purpose: the same pedal is
// the same demand in every gear (the history, the flip lock and the hunt metric compare it across shifts), phi is what a gear is asked on top of it.
float wish_share_eff() { float s = wish_share(); return s == s ? s * wish_limit_k() : NAN; }
