// What the two gear brains do the same way (see shiftcommon.h). Moved verbatim out of modules/trueshift.cpp when the car automatic got its own
// module (modules/carshift.cpp).
#include "shiftcommon.h"
#include "game.h"
#include "rpmspan.h"
#include "shiftmodel.h"
#include "wish.h"
#include "bands.h"
#include <math.h>
#include <string.h>

const char *const why_text[W_COUNT] = {
#define X(id, text) text,
    WHY_LIST(X)
#undef X
};

// The demand's noise: a foot (or a cruise controller) wavers a fraction of what it asks, so "about the same" across a shift is a demand that
// moved by less than a quarter of the demand at the shift, never under DM_FLOOR (~3 % of full demand, under what a press or release produces).
static const float DM_FLOOR = 0.03f;
static float dm_thresh_at(float dm) { return fmaxf(DM_FLOOR, 0.25f * fabsf(dm)); }

// A hunt is only a hunt when the demand stayed about the same across the shifts: three shifts that each reverse the direction while the driver's
// foot (or the cruise/planner) clearly changed its mind between them is the driver driving, not the box hunting.
void sc_track_shifts(int ours, int prev, int g, int hint, double now, float dm_last, const char *mode_label)
{
    static struct { double t; int from, to; float dm; } sh[3];
    if (!ours) { memset(sh, 0, sizeof sh); return; }  // below the takeover speed the game's own automatic shifts
    if (prev <= 0 || g <= 0 || fabsf(T.speed) * 3.6f < 5) return;  // neutral (eco-roll) is not a shift, nor is picking the launch gear at a stop
    if (hint) {
        if (sh[2].t && now - sh[2].t < 5 && (g - prev) * (sh[2].to - sh[2].from) < 0)
            incident("override", "hint %d->%d against our shift %d->%d, %.1f s later, %.0f km/h, %.0f rpm, mode %s", prev, g, sh[2].from, sh[2].to,
                     now - sh[2].t, fabsf(T.speed) * 3.6f, T.rpm, mode_label);
        return;
    }
    sh[0] = sh[1]; sh[1] = sh[2]; sh[2].t = now; sh[2].from = prev; sh[2].to = g; sh[2].dm = dm_last;
    if (sh[0].t && now - sh[0].t < 10 && (sh[0].to - sh[0].from) * (sh[1].to - sh[1].from) < 0 && (sh[1].to - sh[1].from) * (g - prev) < 0 &&
        fabsf(sh[1].dm - sh[0].dm) <= dm_thresh_at(sh[1].dm) && fabsf(sh[2].dm - sh[1].dm) <= dm_thresh_at(sh[2].dm))
        incident("hunt", "gears %d->%d->%d->%d in %.1f s at %.0f km/h, %.0f rpm, grade %.1f%%, mode %s", sh[0].from, sh[0].to, sh[1].to, g,
                 now - sh[0].t, fabsf(T.speed) * 3.6f, T.rpm, V.grade_pct, mode_label);
}

// How much of the price of reversing the last shift is still due (WP-G8, the owner's decisions 63 and 64: "a price that fades when the situation changes").
// The anti-hunting price (a shift back to the gear just left costs its shift time twice, gc_in::prev) and the dwell after a braking downshift guard against one thing: undoing a shift while
// the situation it was made in is still there, the hunt. Where the situation is gone, going back is a new decision and not a hunt. Two things say that, both in the quantities the hunt
// metric above already counts a hunt by:
//   - the demand moved: more than the metric's noise (a quarter of the demand at the shift, never under DM_FLOOR; within it the pedal is "about the same" and the shift would be a hunt) is a
//     changed demand, fully changed at twice the noise. A lift (the pedal at 0.5, off now) is fully changed; a pedal that wavers inside the noise is not changed at all, so a hunt keeps
//     the whole price.
//   - the braking is over: a shift made for braking (the engine brake, the braking rules, the deceleration assist) was made in the braking. When it has stopped the shift's reason has gone.
// The change counts for as much as the pedal is calm. A full pedal is the one demand an upshift can fail to hold, the limit cycle the dwell is for (upshift, short of force, downshift
// back), so the protection stays whole at a full pedal however much changed; a calm pedal (the share of a full pull the pedal asks is small) is held by the higher gear with room to
// spare and has nothing to hunt. So the price left is 1 - change x (1 - share): when everything has changed it is the pedal's own share (a pull of 0.2 owes 0.2 of the price), and
// at a full pedal it is whole. No new number: the noise and DM_FLOOR are the hunt metric's own, and what counts as braking (a service brake over 0.05, a retarder lever, an engine brake
// stage) is the callers', who hand it over as braking_now 0..1.
// Continuous in the demand now, the demand then and the braking now: the ramp of the demand is one noise wide and the noise is never under DM_FLOOR, so a 0.001 step of any input moves the
// weight by at most about 0.001 / DM_FLOOR = 0.033 (the rows sweep it).
float sc_reversal_weight(float dm_then, float dm_now, int braked_then, float braking_now)
{
    const float then = fminf(1.0f, fmaxf(0.0f, dm_then)), now = fminf(1.0f, fmaxf(0.0f, dm_now));
    const float noise = dm_thresh_at(then);
    const float demand_changed = fminf(1.0f, fmaxf(0.0f, (fabsf(now - then) - noise) / noise));
    const float braking_over = braked_then ? 1.0f - fminf(1.0f, fmaxf(0.0f, braking_now)) : 0.0f;
    return 1.0f - fmaxf(demand_changed, braking_over) * (1.0f - now);
}

// Dashboard: the mode letter, also stopped and in N (a steady letter is preferred over "A" flashing in at every stop); A in reverse. Runs outside
// the gearbox update, so the gearbox comes from game_actor, not a cached pointer.
// The mode letter is the lowest wish of the dashboard letter (core/display.cpp); the writer runs here, at this rate, and shows
// the modules' status (B hill-start brake, C crawl) and flashes (e.g. the new ACC distance for 1.5 s) over it.
void sc_dash_letter(int enabled, int on, void *self, int *letter_state, void (*mode_update)(), char (*letter)())
{
    static double at;
    double now = wall_s();
    if (!enabled || !on) { disp_post(DISP_TS, DISP_MODE, 0, 0); return; }
    if (now - at < 0.25) return;
    at = now;
    // Only the gearbox the hooks have seen (never a stale or menu object).
    void *gb = game_gearbox();
    if (!gb || gb != self) return;
    mode_update();  // 4x/s, also while stopped
    disp_post(DISP_TS, DISP_MODE, T.gear < 0 ? 'A' : letter(), 0);  // reverse: plain A
    char c;
    int r = disp_write(gb, &c);
    if (r != *letter_state) {
        if (r > 0) logline("dashboard letter works (%c)", c);
        else logline("dashboard letter: auto_mode_name string not found");
        *letter_state = r;
    }
}

// Incident watch (core/incident.cpp): the rpm limiter hit while the brain drives, or the engine dragged below idle while moving.
void sc_watch_engine(int driving_ok, const char *mode_label, float idle)
{
    static double hi_at, lo_at;
    static int armed = 1;
    double now = sim_s();
    int driving = driving_ok && T.engine_on && !T.paused && T.gear > 0;
    float lim = V.eng.rpm_limit > 500 ? V.eng.rpm_limit : T.rpm_limit;
    if (driving && lim > 500 && T.rpm > 0.985f * lim && T.retarder == 0 && !T.motor_brake) {
        if (!hi_at) hi_at = now;
        if (armed && now - hi_at > 0.6) {
            armed = 0;
            incident("limiter", "rpm %.0f of %.0f for %.1f s in gear %d at %.0f km/h, throttle %.0f%%, grade %.1f%%, mode %s", T.rpm, lim, now - hi_at, T.gear,
                     fabsf(T.speed) * 3.6f, T.in_thr * 100, V.grade_pct, mode_label);
        }
    } else {
        hi_at = 0;
        if (T.rpm < 0.9f * lim) armed = 1;
    }
    if (driving && fabsf(T.speed) * 3.6f > 8 && T.rpm > 100 && T.rpm < 0.9f * idle && T.in_thr > 0.2f) {
        if (!lo_at) lo_at = now;
        if (now - lo_at > 0.8) {
            incident("lug", "rpm %.0f under idle %.0f for %.1f s in gear %d at %.0f km/h, throttle %.0f%%, grade %.1f%%", T.rpm, idle, now - lo_at, T.gear,
                     fabsf(T.speed) * 3.6f, T.in_thr * 100, V.grade_pct);
            lo_at = now + 30;  // not again for a while
        }
    } else if (!lo_at || now > lo_at) lo_at = 0;
}

// M: the engine over its limit (the wheels push it: downhill, or a downshift that was too low). Like the Mercedes and
// Volvo manuals: a warning tone, no shift. Under power the governor keeps the rpm at the limit, so this stays quiet.
void sc_overspeed_tone(int m_on)
{
    static double over_at, tone_at;
    static int said;
    double now = sim_s();
    if (now < tone_at) tone_at = 0;  // the sim clock started again (new session)
    float lim = V.eng.rpm_limit > 500 ? V.eng.rpm_limit : T.rpm_limit;
    int on = m_on && T.engine_on && !T.paused && T.gear > 0 && lim > 500;
    if (!on || T.rpm < 0.98f * lim) { over_at = 0; said = 0; return; }
    if (T.rpm < 1.03f * lim) return;
    if (!over_at) over_at = now;
    if (now - over_at < 0.5 || now - tone_at < 1.5) return;
    tone_at = now;
    play_cue(CUE_OVERSPEED);  // a warning, not feedback: also with tones 0
    if (!said) {
        said = 1;
        logline("M: engine over its limit, %.0f of %.0f rpm in gear %d at %.0f km/h, grade %.1f%%: warning tone", T.rpm, lim, T.gear,
                fabsf(T.speed) * 3.6f, V.grade_pct);
    }
}

// Idle rpm: standing, engine on for 5 s, pedal and throttle released, rpm steady for 1 s. Starts from the ini's idle, never from one sample:
// the start-up flare (1294 rpm) taken as idle made every low-speed shift after it a "stall" downshift.
void sc_idle_learn(float *idle, float p_idle, int store)
{
    static double on_at = -1, win_at;
    static float lo = 1e9f, hi = 0;
    double now = wall_s();
    if (!T.engine_on) on_at = -1;
    else if (on_at < 0) on_at = now;
    // A car starts with the idle of its last drive (ratio_cal.txt) when it has none yet
    if (store && *idle < 300) { float si = vehicle_idle_stored(); if (si > 300) *idle = si; }
    // On a converter the engine does not stand still at the lights for the 5 s above (ride 31-10: 550 all session, the lug floor 900 with the
    // real idle at 840): with the pedal up and the car rolling in gear the engine falls to its idle in gear and stays there, whatever the speed;
    // a flat rpm under 1000 over the second is that floor
    // ... but only while the converter is open: the driveline overruns the engine. A gear that is coupled (locked) drags the engine to the driveline's rpm, and a flat
    // coast in it is the road speed, not the idle (ride 05-11, converter truck at 550: coasting in a locked 4th at 30 km/h taught 789, the floor moved up with it).
    bool floor_roll = V.tc && T.gear > 0 && fabsf(T.speed) > 1.5f && clutch_seen() < 0.02f
                      && vehicle_rpm_at(T.gear, fabsf(T.speed)) > T.rpm + 100;
    int ok = T.engine_on && now - on_at > 5 && (fabsf(T.speed) < 0.3f || floor_roll) && T.in_thr < 0.02f && T.eff_thr < 0.02f
             && T.rpm > 300 && T.rpm < 1000;
    if (!ok) { lo = 1e9f; hi = 0; win_at = now; }
    else {
        lo = fminf(lo, T.rpm);
        hi = fmaxf(hi, T.rpm);
        if (now - win_at > 1) {
            if (hi - lo < 40) {
                float r = 0.5f * (lo + hi);
                if (*idle < 300) *idle = p_idle;
                *idle += 0.3f * (r - *idle);
                if (store) vehicle_idle_note(*idle);
            }
            lo = 1e9f; hi = 0; win_at = now;
        }
    }
}

// The start gear on the game's clutch of 1.62 (K1 round 16). The clutch holds the engine at the slip rpm (vehicle_slip_rpm) and the truck pulls away on the engine's torque there at the foot's
// throttle until the driveline reaches that rpm (the lock speed); from there the clutch is closed and the pull is the pedal's wish. A taller gear locks later and slips longer (the slip lasts
// v_lock / a, v_lock ~ 1 / ratio, a ~ ratio: ~ 1 / ratio^2) and saves the shift out of the gear below. The layers price a shift in seconds (gc_shift_price: the gap and the speed it loses won back at
// the pull), so a gear is worth starting in while its slip is no longer than that: the TALLEST gear whose slip fits the price of a shift. The heavier the truck, the steeper the road, the lighter the
// foot, the longer every slip and the lower the gear; a 41.9 t at a calm foot has no gear whose slip fits, and starts in the lowest that can pull the road at all (a <= 0 is no start gear).
static int sc_launch_pick_slip(const sc_launch &c)
{
    const int n = T.fwd_gears < 32 ? T.fwd_gears : 32;
    const float r = wheel_radius(), m = V.mass;
    if (n < 1 || !(r > 0.1f) || !(m > 0.0f)) return 1;
    const float rs = c.slip_rpm, p = clampf(c.foot, 0.0f, 1.0f), share = clampf(c.share, 0.0f, 1.0f);
    const float tq = V.eng.torque * curve_factor(&V.eng, rs) * p;
    const float res = vehicle_resist(0.0f), dur = fmaxf(c.gap_s, 0.1f);
    // the price of a shift at the pull of the wish at the speed a launch has when the first gear is locked: the gap and the speed it loses (the resistance and the shift's learned loss) won back at the pull
    const float v1 = rs * r / (fmaxf(T.ratio[0] * T.diff, 0.1f) * 9.5493f);
    const float Fw = fminf(share * wish_env(v1), vehicle_force_at(1, v1, 1.0f));
    const float a_w = fmaxf(0.05f, (Fw - vehicle_resist(v1)) / m);
    const float price = dur * (1.0f + fmaxf(0.0f, vehicle_resist(v1) / m + c.decel) / a_w);
    int best = 1;
    for (int g = 1; g <= n; g++) {
        const float ratio = T.ratio[g - 1] * T.diff;
        if (!(ratio > 0.0f)) continue;
        const float a_s = (tq * ratio * V.eta * V.f_scale / r - res) / m;
        if (!(a_s > 0.0f)) continue;
        const float t_slip = rs * r / (ratio * 9.5493f) / a_s;
        if (g == 1 || t_slip <= price) best = g;
    }
    return best;
}

int sc_launch_pick(const sc_launch &c, int stock, float thr, bool force)
{
    if (c.slip_rpm > 0.0f && !force && c.start_fix > 0 && (c.takeover || force) && V.eng_ok && wheel_radius() > 0.1f && stock <= (T.fwd_gears < 32 ? T.fwd_gears : 32)) return sc_launch_pick_slip(c);
    int n = T.fwd_gears;
    n = n > 0 && n < 32 ? n : 0;
    int pick = stock;
    if (c.start_fix > 0 && (force || c.takeover) && V.eng_ok && wheel_radius() > 0.1f && stock <= n) {
        float r = wheel_radius(), f_res = vehicle_resist(0);
        // Above the game's pick: only by physics (lock speed and slip time at this mass and grade), never by a gear count,
        // because every gearbox numbers its gears differently ("look at the gear ratio, not the number").
        bool lcx = c.lcx != 0;  // launch control (Creep): a higher start gear on the held revs, a longer slip
        bool raise = lcx || (c.launch_max > 0 && c.mode != MODE_POWER && c.mode != MODE_M);
        float limit = c.slip_s * (lcx ? 2.5f : thr >= c.kick_pedal && !raise ? 0.6f : 1.0f);
        float lrpm = lcx ? fminf(fmaxf(T.rpm_limit, 1900.0f) * 0.82f, 1700.0f) : c.launch_rpm;
        // The torque the slipping clutch passes is the smaller of the clutch's own share (launch_torque) and the pedal: a light
        // pedal launches with little torque, so only a low gear locks up within the slip time; a tall gear would slip on until the
        // engine sags under idle. The start gear thus follows the pedal (and through a > 0 the load and the grade), not a fixed
        // share of full torque.
        float tq = V.eng.torque * (lcx ? c.launch_torque : fminf(c.launch_torque, clampf(thr, 0.06f, 1.0f)));
        pick = 1;
        for (int g = raise ? n : stock; g >= 1; g--) {
            float ratio = T.ratio[g - 1] * T.diff;
            float v_lock = lrpm / (ratio * 9.5493f) * r;
            float a = (tq * ratio * V.eta * V.f_scale / r - f_res) / V.mass;
            if (g > stock && v_lock * 3.6f > (lcx ? 30.0f : c.launch_max)) continue;
            if (a > 0 && v_lock / a <= limit) { pick = g; break; }
        }
    }
    return pick;
}

float sc_stop_threshold(float p_stop, float p_no_creep, int crawl, int off, float idle, float thr, float brake)
{
    float stock = 0.8f * idle;
    if (p_stop <= 0 || off || thr > 0.01f || !(idle > 200 && idle < 2000)) return stock;
    // Crawl off: never creep on the clutch at idle; pedals free below 8 km/h opens it like a stop.
    // not with cruise on: in a jam the ACC rolls at walking pace, and neutral would cancel cruise
    if (brake < 0.05f && !(p_no_creep > 0 && !crawl && T.cruise < 0.5f && fabsf(T.speed) * 3.6f < 8)) return stock;
    if (fabsf(T.speed) * 3.6f > 15) return stock;
    return fmaxf(stock, idle + rs_off(idle, 150));
}
