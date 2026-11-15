#pragma once
// The driver profile (model layer, core/model.h): what the plugin knows about THIS driver, the way a truck that tells you how the previous driver drove
// (owner point 2, 02-11). Two parts, both kept per game profile in one small text file and both private to the model layer (a module reads queries, never writes):
//   - the demand history (core/demand.h): how hard the pedal has asked over the last ten minutes;
//   - the learners of addendum 24: lift habits, press speed, lift-then-brake, launch habit (their queries below).
// Every learner starts from a PRIOR derived from physics or from the number the code used before (no evidence = the old behaviour exactly), blends in the
// driver's own events by their count, and may move the prior by a bounded factor at most (DRV_BOUND). A learner never breaks a driver promise
// (fable-buildplan-0211-keuzes.md point 23); each query says which one bounds it.
//
// The pure part (driver.cpp) knows no game and no vehicle model: it takes one plain sample per frame (driver_step) and parses / writes the file's text, so
// tools/sim/test_driver.cpp links it alone. The thin part (driverstore.cpp) builds the sample from T / V / the wish, reads and writes the file, and is
// called from model_frame, session_reset and the shutdown.

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "demand.h"

// ---------------------------------------------------------------- the numbers every learner shares (each with its derivation)
enum { DRV_EVENTS = 64,        // the events a learner keeps (the newest): a probability read from half of them (a lift that lasted tau keeps the ones longer than
                               // tau, about half at the median) has a standard error of at most 0.5 / sqrt(32) = 0.09, under 0.1; and the 600 s memory of the demand
                               // history holds at most a few dozen lifts of a driver who lifts every ten seconds
       DRV_PRIOR_N = 2 };     // the prior counts for as many observations as the thing it predicts has outcomes to tell apart (Laplace's rule: a re-press or not,
                              // a brake or not, two); the driver's events overtake it after a handful (10 events: 83 %)
// A learned value may move its prior by a factor of two at most: of a probability, either outcome's share (p x2 or /2 and 1-p x2 or /2, whichever binds first);
// of a rate or a share, x2 or /2. The derivation is the driver promise P2 (a lift is no surprise: a short blip never shifts, a shift on a lift comes within about
// one shift time): the prior's timing was measured on the sims at one shift time, and one doubling or halving of it is the 'about'. The same factor for every
// learner means no habit can put the box more than one doubling away from the behaviour the sims signed off (P6 'what the driver does wins' acts inside it).
static const float DRV_BOUND = 2.0f;

// What counts as what, reusing the repo's own definitions (no new thresholds):
static const float DRV_OFF = 0.05f;      // the pedal is off below this (TrueShift's coast_p: the pedal the gear choice reads as no wish)
static const float DRV_REAL = DEMAND_LEVEL_LIGHT;   // a real demand: the lowest level the demand history tells apart (0.3 of a full wish); a lift is from one
static const float DRV_BRAKE = 0.02f;    // the driver's service brake is on above this (TrueShift's 'braking')
static const float DRV_REST = 0.3f;      // m/s: standing still
static const float DRV_GAP_S = 0.5f;     // a frame gap longer than this is no continuous pedal (modules/pedal.cpp passes(): dt >= 0.5 is a pass-through)
// A lift that lasts this long is no longer an event a plan can use: a window of the plan's horizon (DEMAND_HORIZON_S) starting at an age of that horizon
// is the farthest query a gear choice makes, so lifts are followed to twice the horizon and then closed as 'still lifted' (censored).
static const float DRV_LIFT_TIMEOUT_S = 2.0f * DEMAND_HORIZON_S;
static const float DRV_PRESS_RATE_PRIOR = 2.0f;   // pedal/s: intent.cpp's urgency scale (urgency = pedal_rate / 2: a press from 0 to full in 0.5 s = 1)
static const float DRV_PRESS_Q = 0.9f;            // 'fast for him' = faster than nine presses of ten of his own

// ---------------------------------------------------------------- one frame of the driver (driverstore.cpp builds it from T, V and the wish)
struct driver_sample {
    double t;            // seconds, the simulation clock (sim_s(): it stops in a pause)
    float pedal;         // the driver's raw pedal 0..1 (T.in_thr); NAN = unknown
    float brake;         // the driver's service brake 0..1 (T.in_brake)
    float v;             // m/s, >= 0
    float mass;          // kg of truck + trailers, 0 = not known
    int gear;            // > 0 a forward gear, 0 neutral, < 0 reverse
    int own;             // 1 = the driver's pedal is what drives: engine on, not paused, no cruise / ACC / crawl / launch control
    float launch_share;  // the shaped share for a pull-away (wish_launch_share(pedal)); NAN = unknown
};
enum { DRV_EV_LIFT = 1,        // a lift just ended (re-press, brake or still lifted at the timeout)
       DRV_EV_PRESS = 2,       // a press just ended
       DRV_EV_LAUNCH = 4 };    // a pull-away just ended
// One frame: updates the open episodes and the learners. Returns the DRV_EV_* of the events that ended in this frame. The demand history is fed by
// driver_frame (demand_tick), not here.
int driver_step(const driver_sample *s);
// Drops the open episodes (a lift, a press, a launch in progress) without learning from them: a pause, a teleport, a load, a cruise taking over.
void driver_drop_open();
// One line about the last event of each kind in ev, for the log ("" if none).
void driver_describe(int ev, char *buf, size_t n);

// ---------------------------------------------------------------- (a) lift habits
// A lift is the pedal going from a real demand (it reached DRV_REAL since it was last off) to off (below DRV_OFF), in a forward gear (or in the gap of a shift: the game reports gear 0 while the clutch is open and the truck moves), moving, with the driver's pedal
// driving. It ends in a RE-PRESS (the pedal is on again), in BRAKING (the service brake comes on first), or is still lifted at DRV_LIFT_TIMEOUT_S ('timeout',
// censored: neither is known). A lift that ends in a stop, a pause, a teleport or a cruise taking over is dropped.
//
// driver_lift_return_p(tau, w, shift_s): the probability that a lift that has lasted tau seconds ends in a RE-PRESS within the next w seconds. shift_s is the
// shift's own duration (TrueShift's sl_dur_est), the one time scale there is.
//   PRIOR (no events): P = 1/2 x w / (tau + w + shift_s).
//     - a lift's length has only the shift as a scale and no other: the scale-free survival with that unit is S(t) = shift / (shift + t) (the Lindy form, the
//       one a distribution with a single scale and no other knowledge leaves): a lift as long as a shift has lasted with probability 1/2, 'a lift shorter
//       than a shift cannot be used for one' (owner point 18) is the part of S that is still above 1/2;
//     - P(it ends within w | it has lasted tau) = 1 - S(tau + w) / S(tau) = w / (shift + tau + w): a re-press is still possible while the lift is short
//       (w = shift, tau = 0: 1/2 of the endings) and the longer the lift has lasted the less likely it ends soon, but never 0 (the exponential exp(-tau / shift)
//       goes to 0 in a few shifts and then a shift costs nothing: the review's finding);
//     - the 1/2 is the share of endings that are re-presses when nothing is known (two ways to end, as often as each other: re-press or brake).
//   LEARNED: among this driver's lifts that lasted past tau (the AT RISK, the ones that ended earlier are not counted, the ones still lifted at the timeout are:
//     they are the evidence that the lift did not end), the share that ended in a re-press in (tau, tau + w]; blended with the prior as
//     (k + DRV_PRIOR_N x prior) / (n + DRV_PRIOR_N) with n the at-risk count at THIS tau (a long lift has few at risk and falls back on the prior), then bounded
//     to the prior's factor of two (DRV_BOUND). Lifts past DRV_LIFT_TIMEOUT_S are closed there: a window reaching past it counts the re-presses before it only.
//   PROMISE: P2. The bounded estimate keeps B's lift rows (a blip never shifts, a held lift shifts within about 1 to 1.5 shift times) within a doubling of
//     the prior, and the reset with the profile gives the prior back.
float driver_lift_return_p(float tau_s, float w_s, float shift_s);
int driver_lift_n(float tau_s);     // the lifts at risk at tau (the evidence the learned part stands on); with tau 0 all lifts kept

// ---------------------------------------------------------------- (c) lift-then-brake
// driver_lift_brake_p(v): the probability that a lift of this driver at speed v (m/s) ends in the service brake rather than a re-press. It reads the same lift events as (a)
// (the ones that ended in a re-press or the brake; the ones still lifted at the timeout ended in neither and do not count).
//   PRIOR: 1/2 (a lift ends one of two ways and nothing says which: the same 1/2 that (a) takes for a re-press).
//   LEARNED: per speed band, two bands, the lifts under and from the MEDIAN speed of his own lifts (a town driver and a motorway driver both get a slow half and a
//     fast half, with no speed written into the code; below two lifts there is one band). (brakes + 1) / (lifts in the band + 2), then held within a factor of two of
//     the prior in either outcome, [0.25, 0.75].
//   PROMISE: P2 and P6. What a lift asks of the engine drag (the consumer, not built here) may be at most 1.5 times the drag the box asks of a driver it knows nothing
//     about, and at least half of it: a driver who always brakes after a lift does not get a lift that is all engine brake, one who never does does not lose it.
float driver_lift_brake_p(float v_ms);
int driver_lift_brake_n(float v_ms);     // the lifts in the band of v (the evidence), 0 = the prior

// ---------------------------------------------------------------- (d) launch habit per load
// A launch is a pull-away: the pedal goes on (>= DRV_OFF) with the truck standing (< DRV_REST) in a forward gear and the brake off. It is followed until the first gear
// change, the pedal going off after the truck has moved, or DEMAND_HORIZON_S (the plan's horizon), and counts if the truck moved and it lasted at least a second (the
// demand history's time step; a share needs that long to settle under the rise limit). Its value is the time-weighted mean of the SHAPED share (wish_launch_share, the
// share the pull-away reads: the same pedal is the same share in every gear, P4) over that window. A brake, the pedal off before the truck moved, a cruise or a crawl
// taking over, a gap or a reset drop it.
// Classes by load: one per octave of the mass, floor(log2(mass / 1 t)) in 0..7 (1-2 t, 2-4, ..., 128 t and over; a 9 t truck is class 3, 18 t class 4, 41 t class 5). A
// factor of two in mass is a factor of two in the acceleration a share gives, so it is the smallest step in which the same share is another launch, and it is the
// factor DRV_BOUND already is. Per class the last DRV_LAUNCH_K launches are kept as a running mean; K = 16 because the mean's standard error 0.5 / sqrt(16) = 0.125 is then
// under the clamp's half width (0.15, below), so the learner's noise cannot decide where it lands.
//
// driver_launch_share(mass_kg): how hard he usually pulls away at this load, a share of the pedal's wish (0..1).
//   PRIOR: 0.3 = the light pedal the code already starts every truck for (the start gear is chosen for ts_start_gear(0.3): wish_env, passes() in the pedal module, the
//     low-speed owner's floor). No launch in the class: exactly that.
//   LEARNED: (2 x 0.3 + n x mean) / (n + 2) with n his launches in the class (at most K), held within a factor of two of the prior, [0.15, 0.6].
//   PROMISE: P1 and P4. It is only the PRIOR for the start gear (the consumer, wired in the second half, picks the gear that suits the pull he usually asks); what a pedal
//     position asks, in the launch and after it, is the wish's and does not move; and the start gear still comes from the physics for the pedal he actually presses.
static const float DRV_LAUNCH_PRIOR = 0.3f;
enum { DRV_LAUNCH_CLASSES = 8, DRV_LAUNCH_K = 16 };
float driver_launch_share(float mass_kg);
int driver_launch_class(float mass_kg);   // 0..DRV_LAUNCH_CLASSES - 1
int driver_launch_n(float mass_kg);       // his launches in the class of this mass (the evidence)

// ---------------------------------------------------------------- (b) press speed
// A press is a rise of the raw pedal of at least DRV_REAL (a swing the demand history would call a demand rather than trimming; smaller movements and noise are
// not presses) in a forward gear with the driver's pedal driving; it ends when the pedal falls DRV_REAL from its top. Its RATE is the pedal's speed through the first
// DRV_REAL of the rise: the rise from foot + one step (0.05) to foot + DRV_REAL over the time the pedal took between the two, in pedal per second. A straight 0 -> 1 press in 0.5 s is 2.0, which is how intent.cpp has always turned the pedal's speed into an urgency. It is known the
// moment the pedal has risen a real demand, which is when a kickdown asks for it, and it does not depend on the frame rate or on how long the press then lasts.
// ONE measure for both ends: the learner keeps this rate for every press (the newest DRV_EVENTS), and the gearbox asks for the press of now with the same function
// (driver_press_rate_now), so the learned scale is in the unit it is applied to.
//
// driver_press_scale(): the rate that counts as a fast press for THIS driver (urgency 1 at and above it), pedal per second.
//   PRIOR: 2.0 = today's fixed scale (intent.cpp: urgency = clamp01(pedal_rate / 2): a press from 0 to full in 0.5 s is a stomp).
//   LEARNED: 'fast' is rare for him: the 0.9 quantile (nearest rank) of his own press rates, the rate only one press in ten reaches. A quantile is only more than
//     the maximum from 1 / (1 - 0.9) = 10 presses on, so below that the prior stands. Blended (2 prior + n x quantile) / (n + 2), then held within a factor of two of
//     the prior ([1, 4] pedal per second). 0.9 is the one judgment call here (rare = the top decile); the bound makes it at most a doubling of when a kickdown comes.
//   PROMISE: P1 and P5. The scale only times a kickdown (how soon the plan treats a press as urgent), never the force a press gets, so a full pedal is still
//     everything the truck can give; and bounded at [1, 4] a press never saturates the urgency below half of today's stomp nor stays dead above twice of it.
// driver_urgency(rate) = clamp01(rate / driver_press_scale()): what intent.cpp does with the pedal's rate (pt_intent_compute: the wired reader).
// driver_press_rate_now(): the rate of the press of now (rising, or stopped rising less than a second ago); 0 when there is none.
float driver_press_scale();
float driver_urgency(float rate_per_s);
float driver_press_rate_now();
int driver_press_n();

// ---------------------------------------------------------------- (f) the end of his pedal (WP-0911-K1)
// The kickdown comes at the END of the pedal, and where that is, is his: a driver whose pedal tops out at 0.93 must be able to kick down, one who floors it at 1.00 must not get a
// kickdown at 0.95 on the way there. Learned from how he floors it.
//   PRIOR: DRV_END_PRIOR = 0.95, the pedal intent.cpp flagged as floored before the pedal was an rpm target. No evidence = the old position of the kickdown exactly.
//   EVENT: a press (above) whose HELD TOP reached DRV_END_LO: the highest level the pedal spent DRV_END_HOLD_S or more at or above during the press (a single frame of overshoot is no held top;
//     the pedal moves through the level so the time is the sum). The held tops of the presses that came that far are kept (the newest DRV_EVENTS, in per mille); a press that stayed lower says
//     nothing about where the pedal ends.
//   LEARNED: the median of the kept tops (nearest rank; one wild press cannot move a median of several, and the blend below is slow), blended with the prior as (DRV_PRIOR_N x prior + n x median)
//     / (n + DRV_PRIOR_N), so one held floor moves the end by a sixth of the way and ten by five sixths, and held within [DRV_END_LO, 1].
//   DRV_END_LO = the prior less two of the pedal's own steps (DRV_OFF): 0.85, the lowest end a kickdown can be asked at (a pedal that can only reach 0.8 never kicks down: it asks for no more
//     than a firm pull). DRV_END_STEP: the pedal rests at its stop within this (the ride of 08-11: a held floor reads 0.97 .. 1.00), so the target moves to P3 over that last stretch below the end.
//   PROMISE: P1 and P5. Nothing but when a kickdown comes depends on it; the force a press gets is the wish's.
static const float DRV_END_PRIOR = 0.95f;
static const float DRV_END_LO = DRV_END_PRIOR - 2.0f * DRV_OFF;
static const float DRV_END_STEP = 0.03f;
static const float DRV_END_HOLD_S = 1.0f;   // the history's step (demand.h: one sample a second): a pedal held that long is held
float driver_press_end();      // his pedal's end, raw pedal 0..1 (DRV_END_LO .. 1)
int driver_press_end_n();      // the presses it stands on

// ---------------------------------------------------------------- (e) hint votes, LOG ONLY
// Every +/- shift hint the driver presses is a vote about the gearbox: "not this gear, now". This round the box only RECORDS it (owner points 19.2 and 20.2: "in theory the
// box is perfect"; no learning from hints, no automatic adjustment; decided after a ride), the way MARK records a moment. MARK's marks.jsonl and screenshot live in the
// private core/dev.cpp, which is not in the release, so the vote goes to the plugin's log instead: one line per hint with the fixed prefix DRV_HINT_PREFIX followed by a JSON object, so
// a tool reads them with `grep HINTVOTE dazdrive.log` and a JSON parser, and the second half's report can say, per vote, which term would have had to change and by how much.
// The line is formatted here (pure, tested in test_driver.cpp); TrueShift (note_gear_change, where a hint is recognised) logs it. The layers' candidate terms are local to
// TrueShift's gear_layers (package B's), so `terms` is NULL until that function exposes its last decision read-only; the snapshot is the gear, rpm, speed, pedal, the demand the
// choice works with, the grade, the load, the mode, and the reason and age of the box's last visible decision.
static const char DRV_HINT_PREFIX[] = "HINTVOTE ";
struct driver_hint {
    int from, to;            // the gear before and after the press (the log numbering of TrueShift's lines); the direction is the sign of to - from
    float kmh, rpm, pedal;   // speed, engine rpm, the driver's pedal 0..1
    float demand;            // the demand the gear choice works with (0..1, TrueShift's S.demand)
    float grade_pct, mass_t; // the road grade and the load in tonnes
    const char *mode;        // "Eco" / "Normal" / "Power" / "M"
    const char *why;         // the reason of the box's last visible decision (the ticker's), "" = none
    float why_age_s;         // seconds since that decision, < 0 = unknown
    const char *terms;       // the layers' last candidate terms as one short string, NULL = not exposed
};
// Writes the line into buf (truncated to n if longer; returns the length it needs). NAN or infinite numbers are null.
static inline void drv_hint_str(char *out, size_t n, const char *s)   // a JSON-safe copy: quotes, backslashes and control characters become spaces
{
    size_t i = 0;
    if (s) for (; s[i] && i + 1 < n; i++) out[i] = (s[i] == '"' || s[i] == '\\' || (unsigned char)s[i] < 32) ? ' ' : s[i];
    out[i] = 0;
}
static inline void drv_hint_num(char *out, size_t n, float x, const char *fmt) { if (x == x && fabsf(x) < 1e9f) snprintf(out, n, fmt, x); else snprintf(out, n, "null"); }
static inline int driver_hint_line(char *buf, size_t n, const driver_hint &h)
{
    char kmh[16], rpm[16], pedal[16], demand[16], grade[16], mass[16], age[16], mode[24], why[64], terms[280];
    drv_hint_num(kmh, sizeof kmh, h.kmh, "%.1f"); drv_hint_num(rpm, sizeof rpm, h.rpm, "%.0f"); drv_hint_num(pedal, sizeof pedal, h.pedal, "%.2f");
    drv_hint_num(demand, sizeof demand, h.demand, "%.2f"); drv_hint_num(grade, sizeof grade, h.grade_pct, "%.1f"); drv_hint_num(mass, sizeof mass, h.mass_t, "%.1f");
    drv_hint_num(age, sizeof age, h.why_age_s >= 0.0f ? h.why_age_s : NAN, "%.1f");
    drv_hint_str(mode, sizeof mode, h.mode); drv_hint_str(why, sizeof why, h.why); drv_hint_str(terms, sizeof terms, h.terms);
    return snprintf(buf, n, "%s{\"dir\":%d,\"from\":%d,\"to\":%d,\"kmh\":%s,\"rpm\":%s,\"pedal\":%s,\"demand\":%s,\"grade\":%s,\"mass_t\":%s,\"mode\":\"%s\",\"why\":\"%s\",\"age\":%s,\"terms\":%s%s%s}",
                    DRV_HINT_PREFIX, h.to > h.from ? 1 : h.to < h.from ? -1 : 0, h.from, h.to, kmh, rpm, pedal, demand, grade, mass, mode, why, age,
                    h.terms ? "\"" : "", h.terms ? terms : "null", h.terms ? "\"" : "");
}

// ---------------------------------------------------------------- persistence (the file's text; the file itself is driverstore.cpp)
// The text of the profile: a versioned plain text, "dazdrive-driver v=1" first and "end" last, one `key=value` line per part. Returns the bytes written
// (0 = it did not fit in n).
size_t driver_serialize(char *buf, size_t n);
// Replaces the profile with the text's. A corrupt or old text (another version, a missing "end", a number out of range, a count that does not match its
// values) is ignored as a whole: returns 0 and the profile is untouched. 1 = loaded. Unknown keys are skipped (a newer file read by an older plugin).
int driver_parse(const char *text);
// Forgets everything (a new profile): the demand history and every learner back to its prior.
void driver_reset();
// Changes of the profile since the start (events and the demand history's notes): the store saves when it moved.
unsigned driver_gen();
// One line of what the profile holds, for the log ("600 s of demand, ...").
void driver_summary(char *buf, size_t n);

// ---------------------------------------------------------------- the model layer's entry points (driverstore.cpp)
void driver_frame();                    // once per frame, from model_frame(): the demand history's feed, the learners' events, the save timer, the first load
void driver_break(int level);           // session_reset (RESET_*): a pause, a teleport, a load: the open episodes are dropped (a lift across a teleport is no
                                        // lift), the learned data stays; RESET_HARD also saves
void driver_stop();                     // shutdown: save
void driver_set_profile(const char *key);   // the game profile changed (key "" = unknown): save the old, forget, load the new. Nothing in the plugin can name
                                        // the game's profile yet (the SDK has no such channel and no reader exists in core), so the key is "" and there is one file
