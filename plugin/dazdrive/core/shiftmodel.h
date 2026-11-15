// What every gear brain (modules/trueshift.cpp for trucks, modules/carshift.cpp for cars) needs of the vehicle model and learns into it
// while it drives, in one place so the two brains do not each carry a copy: the tire / converter detection (V.rpm_k, V.tc), the force
// correction (V.f_scale, V.f_bias) and the calibration of a converter box's effective ratios (core/vehicle.cpp). The state is the brain's
// own (it lives in the brain's file); the functions here hold none.
#pragma once
#include "suite.h"
#include "vehicle.h"
#include <math.h>

// ---- small math both brains use
static inline float lerpf(float a, float b, float k) { return a + (b - a) * k; }
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float smooth(float e0, float e1, float x)
{
    if (e1 <= e0) return x >= e1 ? 1.0f : 0.0f;
    float t = clampf((x - e0) / (e1 - e0), 0, 1);
    return t * t * (3 - 2 * t);
}

// A throttle cap or scale of ours (the own speed limiter, the rollover guard, launch control: core/actuate.cpp) holds the engine's throttle
// below what the pedal asks. Then the pedal is not a request for pull: what counts is the throttle the engine actually gets.
int sm_thr_capped();

// ---- the torque converter and the tire correction. cfg = the brain's ini ([...] torque_converter, tc_stall, tc_lock_gear)
struct sm_tc_cfg { float tc, stall, lock, idle; };   // idle = the brain's idle rpm (0 = unknown): the converter's detection needs the engine over it
// A converter the last drive of this vehicle found is known from the start (the detection needs a second of slip, and the first upshifts of a
// launch were a manual box's).
void sm_tc_known(const sm_tc_cfg &cfg);
// Learn the vehicle model's rpm correction while the clutch is closed and the speed steady (the converter is detected here too, and its lockup
// gear learned). step_s = the seconds since the brain's last update (0..0.5); *tc_slip_s and *rpm_odd_s are the brain's accumulators.
// Returns 1 when the box does not follow the road speed (an engine slower than the driveline for 20 s): the brain leaves the gears to the game.
int sm_learn_rpm(const sm_tc_cfg &cfg, int g, float v, double step_s, float *tc_slip_s, float *rpm_odd_s);

// ---- the force correction: a recursive least squares over steady moments (V.f_scale x engine force + V.f_bias)
struct force_learner {
    double F, R;                        // the engine force at scale 1 and the modeled resistance (without f_bias), lagged 0.5 s to match T.accel
    double w, sf, sff, sy, sfy;         // the forgetting sums of the normal equations (weight, F, F^2, y, F*y)
    double aprev, atprev, eff_settle;   // the steady gates: acceleration not changing fast, throttle steady for a second
    float eff_prev;
};
void fl_reset(force_learner *f);        // the accumulation and the lag filters start over (f_scale itself survives a load change)
// One step per update with the clutch closed and the engine in gear g at speed v; eff = the throttle the engine gets. last_brake_at = the last time
// a brake was on (the game's or our own), shift_at = the last shift, eb_active = an engine brake or a retarder works (a truck's).
void fl_step(force_learner *f, int g, float v, float eff, double dt, double now, double last_brake_at, double shift_at, int eb_active);

// ---- the calibration of a converter box's effective ratios (core/vehicle.cpp): the measured engine rpm per speed in the gear. Nothing happens off a
// converter. idle = the brain's idle rpm.
struct ratio_feeder { double at, ref_at, held, gsince; int pg, pcls; float pq, pthr; };
void ratio_feed(ratio_feeder *r, int g, float v, double now, double shift_at, float idle);
