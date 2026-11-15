#pragma once
// The driver's demand, ONE source (model layer, core/model.h). The wish is a force at the wheels at the speed now: the torque pedal's demand
// (below; modules/pedal.cpp delivers it), else the cruise's force wish (modules/cruise.cpp, [cruise] force_mode 1), else what the caller passes as the
// fallback (the demand of the pedal in the gear we are in), 0 while the pedal is off. The clients read the wish_* functions below: the gearbox
// (wish_get, wish_in, wish_share_eff), the cruise's own controller (wish_full), the launch (Creep) and the low-speed owner (wish_launch_share).
//
// Also kept here, because they belong to the wish and not to the gearbox: the wish filtered over a few shift times (rising fast, falling slowly:
// a stab at the pedal after a shift is no reason to change the band, and the power band is kept a while once the pedal lets go). (The speed of the pedal, how fast
// it went down, is the driver profile's: core/driver.h driver_press_rate_now, in the unit his presses are learned in.)

enum { WISH_NONE = 0,   // neither the pedal nor the cruise asks: the caller's fallback (the demand in this gear), or 0 while coasting
       WISH_PEDAL,      // the torque pedal's power wish
       WISH_CRUISE,     // the cruise's force wish
       WISH_PLANNER };  // (reserved: the planner drives through the cruise's force wish today; nothing sets this yet)

struct wish_t {
    float force_n;      // N at the wheels at the speed asked for; 0 = nothing asked (coasting)
    float share;        // 0..1: the same wish as a share of a full torque-pedal wish (the pedal's / the cruise's own); NAN when the source has none
    int source;         // WISH_*
    float slow_n;       // force_n filtered over a few shift times (rising ~4 shift times, falling ~12)
};

struct wish_args {
    float v;            // m/s
    int coast;          // the pedal is off (lift-off by the pedal itself): the wish is 0 and neither source is read
    int use_pedal;      // the torque pedal's wish may be read (TrueShift's objective is on); else the cruise's, else the fallback
    float fallback_n;   // asked by neither source: the caller's own demand in this gear (N)
    double now;         // sim_s() of this decision
};

// The wish now. ws / ws_t: the caller's filter state (the filtered force and when it was last updated; the caller resets them with its session).
// The pedal's rate is sampled only while something is asked (the pedal off does not move it).
void wish_get(const wish_args *in, float *ws, double *ws_t, wish_t *out);

// ---------------------------------------------------------------- the torque pedal's demand (package P, notes/fable-demand-design-0211.md)
// ONE definition of what the pedal asks, delivered by the pedal module and planned with by the gearbox, a pure function of the vehicle model (V) and the
// engine bands (bands_now) plus the shaping's state (the share, the rise limit):
//   s        the shaped share 0..1: the pedal through the mode's map (s = (1 + c) p^k / (p^k + c), k and c from [pedal] curve_toe / curve_*, P6b is k 1 with c Eco 0.8 / Normal 0.6 / Power 0.4; the same in every truck), rate limited when rising
//   F_env(v) the envelope in the units of the demand: max over the gears whose revs lie inside the bands' [floor, cap] of F_g(v) / phi(rpm_g(v)); held at
//            its value at the speed where the start gear reaches the floor under that speed (what the truck can do at walking pace). At rev_gain 0 that is
//            the strongest force any gear gives. Because F_d(g) = s x F_env x phi(rpm_g), at s = 1 every in-window gear is asked at least its own full
//            throttle (a full pedal is everything the truck can give, in any gear, at any rev_gain); at rev_gain 1 the throttle is the share itself
//   level    = s x F_env(v) on the flat: the driver's demand, the same in every gear (the plan's horizon, the history's and the cruise's unit). On a climb the grade assist (P6.3) remaps
//            the pedal's range: base = min(grade_assist x m g sin(grade), F_env), level = base + s x (F_env - base); the bottom of the range goes up, the top never (s = 1 is F_env)
//   phi(rpm) = (P_eng(rpm) / P_eng(P1)) ^ rev_gain: how much the demand follows the engine's state in a gear; 1 at P1 for every rev_gain, 1 everywhere at
//            rev_gain 0 (the same push at the same speed in any gear, constant power), 1 = a real truck's torque request (the pull holds in a gear)
//   F_d(g,v) = level x phi(rpm_g(v)): what gear g is asked; the throttle is F_d / F_g (the pedal module clips it to 0..1)
//   effective = F_d(cur), held under act_thr_limit() x F_cur while an actuator cap or scale (the own limiter's cap, the rollover guard's) holds the throttle,
//            and under f_lim = max(f_res, 0) + m x (top - v) / dur near a speed limiter (speed_top_kmh: ours, else the game's): what the gearbox and the
//            history read (a flat pedal against the limiter is a cruise, not a launch)
// Without usable bands phi is 1 and the old window (idle + 60 % of the way to V.plateau .. the power range's top or 93 % of the limit) is the envelope's.
struct wish_cfg {
    float curve_eco, curve_normal, curve_power;   // the pedal's map per mode, s = (1 + c) p^k / (p^k + c) ([pedal] curve_*): c large = the diagonal, c small = front-loaded
    float curve_toe;                           // k, the same in every mode ([pedal] curve_toe): 1 = the rational map of P6b, above 1 a soft start (a toe) at the bottom of the pedal
    float rise_eco, rise_normal, rise_power;   // the shaped share may rise at most this much per second ([pedal] rise_rate_*)
    float rev_gain;                            // 0..1, [pedal] rev_gain
    float grade_assist;                        // 0..2, [pedal] grade_assist: how much of the climb the pedal takes out (0 = none)
};
void wish_configure(const wish_cfg *c);        // the pedal module, at start and when a [pedal] value changes
void wish_reset(int level);                    // RESET_*: another truck, a load, a hard break (the shaping starts over)
void wish_follow(float p);                     // the pedal goes through (wish off, a NaN pedal): the share follows the raw pedal, no live demand

// Why the demand does not drive the throttle this tick (the pedal module's pass-through cases, modules/pedal.cpp passes()):
enum { WT_ON = 0,     // it does (the clutch is locked, the truck is driving)
       WT_OFF,        // a hard pass-through (reverse, neutral, electric, manual, crawl, launch hold, cruise on, another owner): the share follows the pedal
       WT_SHIFT,      // the clutch is not locked (a shift, a slip): the throttle goes through, the demand stays so the gear choice sees no drop to zero
       WT_SHAPE };    // standing in a forward gear or the smooth start: the share is shaped for the launch to read (wish_launch_share), the throttle goes through
// One tick of the shaping (once per actuator tick, from the pedal module): p the raw pedal, dt the tick, mode MODE_*, v m/s, gear the gear of now.
// Returns 1 when the demand drives the throttle this tick (WT_ON and the vehicle model can tell); the throttle is then
// clip(wish_in(gear, v) / vehicle_force_at(gear, v, 1)), 0..1.
int wish_tick(float p, float dt, int mode, float v, int gear, int why);

float wish_map(float p, int mode);   // the pedal's map of a mode (MODE_*), s = (1 + c) p^k / (p^k + c) with the configured c and k: 0 -> 0, 1 -> 1, strictly increasing. The ONE definition of what a pedal share is:
                                      // the torque pedal's demand (wish_tick) and the crawl (modules/creep.cpp, the crawl speed the pedal asks) both read it
float wish_unmap(float s, int mode);   // the foot (the raw pedal 0..1) a shaped share stands for: the inverse of wish_map for a mode (MODE_*). The gearbox reads the pedal as an rpm target in the foot's unit
                                      // (core/intent.cpp, WP-0911-K1): the sound follows the foot, whatever the mode's map does to the force it asks
float wish_env(float v);              // F_env(v), N in the units of the demand (see above); 0 = the vehicle model cannot tell (no engine, electric, no start gear)
float wish_force_max(float v);        // the strongest full-throttle force of the in-window gears at v (phi = 1), N
float wish_full(float v);             // what a controller's full wish asks at v (the cruise's, the planner's: a force, no foot): wish_force_max(v)
float wish_phi(float rpm);            // phi at an engine rpm (1 without usable bands or at rev_gain 0)
float wish_level();                   // the live demand now (N at the speed of the last tick), -1 when the pedal gives none (stale after 0.3 s, off, standing)
float wish_level_at(float v);         // the same share on the envelope at another speed
float wish_level_shaped(float v);     // the demand level (N) during the launch/standing, from the shaped share with the grade assist, no live gate: the force the clutch law delivers
float wish_in(int g, float v);        // F_d(g, v), N, unclipped; -1 when there is no live demand
float wish_share();                   // the shaped share 0..1 (the foot's map, without the grade assist), NAN when there is no live demand
float wish_launch_share(float raw);   // the shaped share while the shaping ran this tick (standing in a forward gear or in the launch), else the raw pedal
float wish_effective();               // N: F_d(gear of the tick) held under an actuator cap / scale and a speed limiter (see above), else F_d; -1 when there is no live demand
float wish_limit_force(float v, float dur);   // the force that closes the gap to the speed limiter (speed_top_kmh) within dur seconds, N: f_lim = max(f_res, 0) + m x max(0, top - v) / dur; 1e30 = no limiter
float wish_demand();                  // N: F_d(gear of the tick) before an actuator cap and a speed limiter hold it down (wish_effective's input); -1 when there is no live demand
float wish_limit_k();                 // effective / F_d, 0..1 (1 = nothing holds the demand down, or no live demand)
float wish_share_eff();               // the shaped share held under what an actuator cap and a speed limiter let through (s x wish_limit_k), 0..1: the demand history's unit
                                      // and what a reader that compares the ask across shifts and gears uses (the flip lock, the hunt metric); NAN when there is no live demand

