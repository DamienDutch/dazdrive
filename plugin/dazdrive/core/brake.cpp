// The brake channel: ONE place every automated brake request goes through, and the driver's brake pedal as a deceleration
// wish (Damien 30-10: "all braking through one brake channel, our own brake pedal model"). See core/brake.h for the API.
//
// [brake] channel = 1. A requester (Cruise: descent, the limit + overspeed_cap line, bends, junctions, the own ACC; later the
// planner) asks only a deceleration of the truck, m/s2, plus an urgency. brake_ask: the total (what coasting gives counts; the
// planner, the descent, the line, bends, junctions); brake_ask_add: a brake deceleration on top of what the truck does by itself
// (the own ACC, whose law was tuned as an extra request on top of coasting and the game's overspeed braking). Per tick (an
// ACT_POST hook of the actuator, after Cruise has asked) the channel
//   1. takes the largest ask (the same "most braking wins" as the actuator's max) and shapes a reference toward it at the
//      urgency's jerk limit (comfort jerk_comfort, firm jerk_firm, urgent 15 m/s3; down at most 6 m/s3),
//   2. subtracts what the truck slows by itself (rolling, air, the grade, the engine's drag in gear with the throttle shut; on a
//      descent that is negative: the brakes have to hold gravity too) and adds a trim that closes the loop on the measured
//      deceleration (the model's error: grade, drag, mass, the brakes' real gain). The trim also runs for an added ask: its law is
//      tuned open loop, but the service brake must still escalate when the measured deceleration falls short of the reference
//      (the game's own overspeed retarder delivering less than the model, a weaker retarder than learned),
//   3. gives the rest to the RETARDER first: a demand fed to the game (game_retarder_feed: the game takes the max of it and its
//      own), sized by the retarder's force at full demand (the vehicle model's: start value 54 kN = 3.0 m/s2 at 18 t, sim_decel's measured top step;
//      learned from the measured deceleration while the retarder brakes alone on a level road), built up with a 0.4 s lag,
//      gone in a shift gap (falls in 0.2 s, back 0.3 s after the clutch closes), faded out between 6 and 4.5 km/h, and handed to
//      the service brake from 10 km/h down to 6 (the fade is quicker than the service brake's lag). The game's own cruise
//      overspeed demand (clamp(v - A, 0, 1), cruise-re § 2; A from Cruise every tick) is retarder the channel already has: it
//      feeds only above it and never fights it (it cannot lower it either: the game takes the max),
//   4. gives the SERVICE BRAKE what the retarder does not give: always what is beyond the retarder's reach now (more than its
//      full force, a shift gap -- planned as gone at once --, the fade), and the shortfall while the retarder builds up over a
//      tolerance per urgency (comfort: the build-up is no reason to apply the brakes, it counts as built; firm 0.1 m/s2; urgent
//      0), less a soft dead band of 0.05 m/s2 for calm and firm asks. The request (m/s2 / the brake's gain: learned, else the AEB
//      fit) goes to the actuator as owner ACT_CHAN, which writes it without the generic 0.25/s rise and the 0.05 hold band (the
//      channel shapes its own ramps; 20/s up is a backstop as for the deceleration assist; down the generic 0.6/s),
//   5. asks the gearbox for more engine drag (the gear arbiter's standing request, core/gearreq.h, N at the wheels) while every cheaper
//      stage is full (the retarder fed full; the engine-brake stages on a truck without one) and the service brake still has to help
//      for a second: a lower gear inside the bands up to its crossover (more engine drag, and the retarder works through the ratio);
//      the gearbox picks the gear and weighs whether the shift pays against the road ahead (modules/trueshift.cpp stand_drag_pick),
//   (WP-BR: the stages in the owner's order with a cost each in one unit, BC_COST: coast 0, a lower gear's drag 0 inside the bands, the engine brake's
//   stages 0.15 -- a truck without a retarder, held here since EBrake's descent control moved in --, the retarder 0.30, the service brake 1.00 per m/s2;
//   every decision logs the stages, the deciding term and the cost, one line per change, promise P7.)
//   6. asks the brake lights on (game_brake_light_request) while the service brake is used, the asked deceleration is over
//      light_decel (0.7 m/s2), or any request of ours holds +0x17c (the game never lights them for +0x17c). (The fed retarder
//      demand lights them by the game's own rule anyway: 0x1444 > 0.)
// The AEB, the rollover guard, Auto Hold, hill start, the stop assist, Stop&Go's hold and the ACC's stop behind a standing lead
// keep their own owners above the channel: the actuator merges by the biggest request, and they ramp themselves.
// The driver always wins: a throttle over 0.1 (the tick's pedal, as Cruise reads it) ends every automated ask; the brake pedal
// cancels the cruise in the game, and Cruise asks nothing then.
// With channel = 0 nothing of this runs: Cruise posts its own requests exactly as before, the hook is not even registered.
//
// [brake] pedal = 1 (needs [core] brake_pedal_hook = 1 and a confident learned brake gain, [core] brake_learn = 1): the pedal
// provider. The game calls it once per tick with the raw pedal; it returns the service brake the game applies. The pedal asks a
// braking deceleration that is linear in the pedal and the same at any load: pedal / pedal_linear x the top, the top being the
// truck's own maximum at its lightest (the learned force over the mass without cargo), never more than pedal_decel_max (a good
// road's grip); from pedal_linear to the end of the travel it goes on to full brakes (and from 0.98 the hook passes the raw pedal
// anyway). With pedal_blend the retarder carries the first part (fed through the same demand; the service brake fills what it has
// not built up yet, its fade and the shift gaps). It never makes a stop weaker than the raw pedal below 5 km/h (a hold on a grade).
// It returns the raw pedal whenever it is unsure: no learned gain, mass unknown, reverse, engine off, the channel's tick stale.
// The gain and the top are taken over only while the pedal is released, so the pedal never changes under the foot.
//
// Coexistence with the game's overspeed braking under cruise (g_smart / +0x1a0): it stays as it is until the planner replaces
// it. The channel counts its retarder demand as built (see 3) and feeds only above it. The drivetrain's retarder factor
// ([core] retarder_hook, the deceleration assist's soften) also scales the fed demand; it is below 1 only while the lever assist
// runs, which ends when the cruise is on, so the channel's feed is not scaled in practice.
#include "suite.h"
#include "game.h"
#include "sense.h"
#include "act.h"
#include "vehicle.h"
#include "gearreq.h"
#include "ebrakelogic.h"
#include "brakemodel.h"
#include "brake.h"
#include <math.h>
#include <string.h>

float g_brake_channel, g_brake_pedal;
static float bk_ret, bk_jc, bk_jf, bk_light, bk_ptop, bk_plin, bk_pblend, bk_trace, bk_ebs;

static param g_params[] = {
    { "channel", &g_brake_channel, 1, "1 = all automated braking of the cruise (a descent, the line at the speed limit + overspeed_cap_kmh, bends, junctions, the own ACC) asks a deceleration from one brake channel, which uses what coasting gives (the engine's drag in the gear it is in; it may ask the gearbox for a lower gear inside the bands), then the engine brake's stages, then the retarder (with [core] retarder_feed_hook = 1), then the service brake for the rest and for anything urgent. Emergency braking, Auto Hold and the stop at a standstill keep their own way. 0 = each of them brakes on its own, as before (the engine-brake descent control still runs)" },
    { "pedal", &g_brake_pedal, 0, "1 = the brake pedal asks for a deceleration: linear over the pedal and the same at any load (light braking is no longer all in the first fifth of the travel), full pedal = full brakes. Needs [core] brake_pedal_hook = 1 and [core] brake_learn = 1 (until the brake's gain is learned the pedal stays as the game has it). 0 = the game's own pedal" },
    { "pedal_decel_max", &bk_ptop, 7.0f, "deceleration (m/s2) the linear part of the pedal reaches (never more than the truck can do unladen); about the grip of a good dry road" },
    { "pedal_end_shape", &bk_plin, 6, "brake pedal: the deceleration is linear over the whole travel; towards the end it blends smoothly into the raw pedal (weight (pedal / 0.98) to this power), so at 0.98, where the hook passes the raw pedal, nothing jumps and a light truck still reaches full brakes. Higher = later (no fixed point where it turns into full brakes)" },
    { "pedal_blend", &bk_pblend, 1, "1 = the first part of the pedal brakes with the retarder (quiet, no wear), the service brake adds the rest and what the retarder has not built up yet (needs [core] retarder_feed_hook = 1). 0 = the pedal is the service brake only" },
    { "engine_brake", &bk_ebs, 1, "1 = the engine brake is the first stage of every braking event that has a stage to give (a lever step of the deceleration assist, the brake pedal, a bend, a junction, the speed limit line, the ACC), also on a truck with a retarder: the stage whose worth fits the deceleration asked comes first, the retarder and then the service brake take the rest (the cheapest stage first: the owner's order, keuzes 20.4 and 39). Needs [ebrake] descent = 1 (the stage control). 0 = the engine brake is the descent control's only (under the cruise)" },
    { "retarder", &bk_ret, 1, "1 = the channel uses the retarder first (needs [core] retarder_feed_hook = 1). 0 = the service brake only (the game's own cruise overspeed braking still uses the retarder)" },
    { "jerk_comfort", &bk_jc, 1.5f, "how fast (m/s3) a calm brake request (a descent, a junction far ahead) builds up and lets go" },
    { "jerk_firm", &bk_jf, 3.0f, "the same for a firm one (a bend, the line at the speed limit, a lead); an urgent one comes at once" },
    { "light_decel", &bk_light, 0.7f, "the brake lights come on from this deceleration (m/s2) the channel asks, and always while it uses the service brake (needs [core] brake_light_hook = 1)" },
    { "trace", &bk_trace, 1, "1 = brake.csv (log_level 2): what the channel asked and gave, 10 rows a second while it works or the pedal is pressed" },
};

// Physics of the measurements (not preferences).
static const float BC_RET_TAU = 0.4f;       // s, the retarder's build-up (first order: 63 % after 0.4 s, 92 % after 1 s)
static const float BC_RET_FALL = 0.2f, BC_RET_RISE = 0.3f;   // s, its fall when the clutch opens and its return after it closes (brake-measure § 3)
static const float BC_FADE_LO = 4.5f, BC_FADE_HI = 6.0f;     // km/h, the retarder fades to nothing between these (brake-measure § 2)
static const float BC_HAND_HI = 10.0f;      // km/h: from here down to BC_FADE_HI the retarder hands its share to the service brake (the fade comes faster than the brake's lag)
static const float BC_TOL[BRU_N] = { 0.3f, 0.1f, 0.0f };      // m/s2 of retarder shortfall the service brake leaves (comfort, firm, urgent)
static const float BC_TOL_GAP = 0.1f;       // m/s2, the same in a shift gap and in the fade: the service brake bridges
static const float BC_SRV_DB = 0.05f;       // m/s2: the service brake's soft dead band for calm and firm asks (each application costs air)
static const float BC_OWE_EPS = BC_SRV_DB * BC_RET_TAU;   // m/s: the debt that still counts: the service brake's own whiff (BC_SRV_DB, 0.05 m/s2) held for the retarder's time constant, 0.02 m/s
static const float BC_KI[BRU_N] = { 0.8f, 1.5f, 2.5f };   // per second: the trim's integral on the deceleration error (comfort, firm, urgent)
static const float BC_J_URGENT = 10.0f;     // m/s3, how fast an urgent ask comes (no comfort limit, but no step either): the old writer's slew for a firm request was 1.0/s, and at the model's
                                            // gain of 10 m/s2 per request a reference jerk of 10 m/s3 is exactly that slew, so an urgent ask builds up no faster than the old ACC request did (WP-BR's
                                            // 15 overshot it: the ACC cut-in's jerk went 6.28 -> 7.97 m/s3)
static const float BC_JR_URGENT = 6.0f;     // m/s3, how fast an urgent ask lets go (a falling one is no hurry: more braking is the safe side)
static const float BC_TRIM_FROM = 0.4f;    // s into an event before the trim learns (the measurement's window and the brake's lag)
static const float BC_TRIM_LO = -1.0f, BC_TRIM_HI = 1.5f;
static const float BC_MEAS_WIN = 0.3f;      // s, the measured deceleration: the speed's fall over this window
static const float BC_REF_LAG = 0.25f;      // s, the reference as the measurement sees it (brake lag 0.15 + half the window)
static const float BC_DOWN_S = 1.0f;        // s the stages above the service brake have been full and the service brake still needed before more engine drag is asked of the gearbox
static const float BC_DRAG_MIN = 0.1f;      // m/s2, the least service share worth asking a lower gear for (the gearbox's own floor on the gain is 0.03)
static const float BC_LIGHT_HOLD = 0.6f;    // s the lights stay on at least (no flicker)
// What a stage costs per m/s2 it carries, ONE unit for all of them: the service brake's wear, heat and air for 1 m/s2 held = 1 (the stage the owner wants spared most, keuzes 20.4).
// The order is the owner's (coast -> engine brake -> retarder -> service); the magnitudes are what a decision logs and what the sims assert the order of. Where a stage's cost
// depends on the state it is the owner's noise rule (02-11): the engine's drag in a lower gear is free inside the bands up to the gear's crossover and not offered above it (the
// gearbox's pick, modules/trueshift.cpp drag_gear), the engine brake stage is the stage's bark (the cruise's overspeed braking would put all stages on at 1 km/h over its
// start: our control puts them one at a time, so the cost is per stage held), the retarder has no wear, only the coolant's heat and its build-up, the service brake the rest.
// The one decision the costs make by themselves is which stage is the dearest one carrying a need (the one a lower gear would spare); the gearbox weighs the shift against the
// road ahead in its own terms (modules/trueshift.cpp stand_drag_pick).
static const float BC_COST[BS_N] = { 0.0f /* coast */, 0.0f /* drag gear */, 0.15f /* engine brake stage */, 0.30f /* retarder */, 1.0f /* service brake */ };
static const float BC_PED_FLOOR_LO = 2.0f, BC_PED_FLOOR_HI = 5.0f;   // km/h: under the low one the raw pedal is the floor, fading out to the high one
// The engine brake as a stage of every event (owner 03-11, keuzes 39: "all ways, in ETS2 also the engine brake"). The stages are whole (the game's stage setter: stage / stage count of
// the engine's brake), so the allocation takes the stage whose worth FITS the deceleration the event wants of the stages (0.9 of it at most) and the retarder, which is continuous,
// takes the rest; a worth is the vehicle model's (core/vehicle.h) at the gear and the revs of now.
static const float BC_EB_MIN = 2.0f * BC_SRV_DB;       // m/s2: a stage worth less is not worth its bark (twice the service brake's whiff, BC_SRV_DB: the least it is applied for)
static const float BC_EB_FIT_UP = 0.9f, BC_EB_FIT_DN = 1.1f;   // a stage goes on when its worth is within 0.9 of the ask and off when it is over 1.1 of it: the band the loop's own correction (it measures the SUM) moves the ask in once the stage is on
static const float BC_EB_UP_S = 1.0f, BC_EB_DN_S = 2.0f;       // s between two steps up / a step down of the allocation: the engine brake is back within 0.1-0.4 s (Decel's EB_REL_S) and the measured deceleration (0.3 s window) shows a step after 1 s
static const int BC_EB_FULL_EVENTS = 5;                        // samples (all stages together) after which the model's figure for the stages counts in full: the retarder's blend waits for the same (BP.blend: vehicle_ret_events >= 5); before, a quarter of it (BM_EB_ACCEPT_LO) growing with the samples
static const float BC_EB_PEDAL_TOL = 0.12f;                    // the pedal is open loop: a stage is taken for it only while the part of it that is not counted (the surplus the pedal would brake over its wish) stays within this share of the wish, the pedal's own tolerance (sim_brake_channel's rows hold the pedal to 12 % of its wish)
static const float BC_EB_MEAS_FROM = 0.7f, BC_EB_MEAS_TO = 1.0f;   // s after a step up: the stage's response is over (0.1-0.4 s, Decel's EB_REL_S) and the deceleration measured over 0.3 s shows it; the next step up is a second after (BC_EB_UP_S), so the window is the 0.3 s before it
static const float BC_EB_MEAS_TAU = 0.4f;                      // s, the running mean of the measured deceleration, of what the retarder and the service brake were commanded and of the natural deceleration before a step (the measurement's window and a little more)
static const float BC_EB_SETTLE_S = 3.0f * BC_EB_MEAS_TAU;     // s: the running mean has settled to 95 % after three time constants
static const float BC_EB_FAST_TAU = 0.15f;                     // s, the mean of the measured deceleration over about the 0.3 s before a step (the same window the measurement after it uses)
static const float BC_EB_JUMP = 0.5f;                          // m/s2: a measured deceleration this far from its running mean is a change of the event (the pedal pressed, a shift's end), not noise: the running mean starts over
static const float BC_EB_LAG_R = 0.15f, BC_EB_LAG_S = 0.3f;   // s: how far the measured deceleration trails the retarder model (the window's half) and the service request (the brake's lag and the window's half)
static const float BC_EB_STEADY = 0.1f;                        // m/s2: the deceleration is steady when it is within this of its running mean (twice the measurement's noise of ~0.05): a step up waits for it, else what it adds can not be told from the ask building up
static const float BC_EB_NOTE_MIN = 0.02f;                     // m/s2: a step that added less than this is not a stage (under the noise of a mean over 0.3 s)
static const float BC_EB_NOTE_SHARE = 0.3f;                    // a step is measured only where the engine brake gives at least this share of its torque at the revs (the correction is of the full torque: a small share multiplies the noise)                       // samples behind a stage before the pedal takes it: the retarder's blend waits for the same (BP.blend: vehicle_ret_events >= 5), the provider subtracts the model's figure open loop
static const float BC_EB_WISH_S = 0.5f;                      // s a wish of a requester that does not renew it every tick lives (Decel renews it every tick)

static struct {
    int running;               // the tick hook is in (channel or pedal on)
    float ask[BRQ_N]; int urg[BRQ_N], add[BRQ_N];
    float pat[BRQ_N];          // m/s of braking the requester lets the truck owe while the retarder is gone for a shift (brake_patience); 0 = none: the service brake bridges
    int open;                  // the winning ask is one on top of what the truck does by itself (brake_ask_add): its law is tuned open loop, but the trim still closes the loop on the measured deceleration (see bc_tick)
    float want; int who, urgency;
    float ref, refd;           // the reference, and as the measurement sees it
    int active; float on_s;
    float nat, trim, need;
    float need0;                           // the deceleration the stages (engine brake, retarder, service brake) have to give before the engine-brake stage's credit: reference - coasting + trim
    float eb_credit;                       // m/s2 of it the engine-brake stage the allocation holds is counted to give now (its worth, conservatively: below)
    float ret_force;                       // the retarder's force at full demand (N), the vehicle model's (core/vehicle.cpp): the start value times what was learned, per truck
    float ret_full_nom, ret_full;          // m/s2 at full demand now: without and with the gap envelope
    float game_from_kmh;                   // where the game's overspeed braking starts (NAN = it does not act)
    float game_d, feed, bl, env, a_ret;
    int gap; double gap_t0, gap_t1;
    float tol;
    float srv, req; int srv_on;
    float owed;                            // m/s of braking the shift gap kept back from a patient ask (the plan waits for the gear): repaid by the retarder
    float down_s; int down;
    float drag_n;                          // N at the wheels asked of the gearbox (the standing request), 0 = none
    float eb_ask;                          // m/s2 the engine-brake stage control asks of the service brake when its stages are all on and the speed is still over (0 = none)
    float a_eb;                            // m/s2 the engine-brake stage we hold is worth now in the model (0 = none held or the clutch open): the cost and the log, not the need
    int term, stages; float cost;          // the split's deciding term (BT_*), the stages that carry the event (bits of BS_*), their cost now
    int lights; double light_until;
    // the measured deceleration
    float hv[64]; double ht[64]; int hn, hi; float a_meas; int meas_ok;
    float learn_s; double learn_told;
    double tick_at; unsigned ticks;
    int was_active; int log_who, log_key; double log_at;
    float lvl_srv_max, lvl_ret_max;        // of the braking event (log)
    float gas_ret_s; int gas_ret_told;     // seconds the retarder has been on with the driver on the gas, and the line that names who holds it was written
    int lv_k; float lv_held_s, lv_win_s, lv_sum, lv_nat, lv_min, lv_max; int lv_n;   // the lever-stage learner: the step held, how long, and the window being averaged
    float lv_ff, lv_fsum, lv_fmin, lv_fmax;                                          // ... the retarder factor we posted, through the retarder's own lag, and its sum, lowest and highest over the window
} BC;

// The pedal side: latched by the tick, read by the provider (the game's physics thread runs both).
static struct {
    volatile int ready;            // the provider maps (else the raw pedal)
    volatile float G;              // m/s2 per request (latched while released)
    volatile float top;            // m/s2 of the linear part's end (latched while released)
    volatile float ret_full_nom;   // m/s2 of the retarder at full demand now (no gap envelope)
    volatile float a_ret;          // m/s2 the retarder gives now (built, with the envelope)
    volatile float kmh;
    volatile int blend;            // the retarder may carry the first part
    volatile int auto_ret;         // the game's own auto retarder follows the raw pedal (g_retarder_auto on)
    volatile unsigned calls;       // provider calls since the last tick (stale over 30)
    volatile float feed;           // the pedal's retarder demand (ours, plus the game's auto retarder's raw pedal)
    volatile float raw, out, want;
    volatile float eb;             // m/s2 the engine-brake stage the allocation holds is counted for (latched by the tick): the pedal's retarder and service share is the wish less this
    int auto_eb;                   // the game's own automatic engine brake follows the brake pedal (g_motor_brake_auto): it does that stage by itself, the allocation leaves the pedal to it
    int told; const char *why;
} BP;

// ---------------------------------------------------------------- the engine-brake stage (the descent control of a truck without a retarder; it was modules/ebrake.cpp's)
// Under the cruise on a descent a truck holds its speed with the engine brake stages (core/ebrakelogic.h has the rule and why: one stage at a time as the
// speed nears where the game's own overspeed braking would start, off again near the set speed, never with the throttle on). Since WP-BR it is the channel's second stage, between
// what the truck does by itself and the retarder, and since the owner's 03-11 overruling it runs on EVERY truck with an engine brake, also one with a retarder (the engine brake
// first, the retarder for what it cannot hold; they may run together). The channel holds the lever (the game's own state: on byte, stage through the stage setter, the handler's flag), says to the
// gearbox that it does (the standing request: keep the gear in the engine's brake range, whatever the stage), counts what the held stage is worth (the vehicle model's, learned
// per stage) as a deceleration the truck has, and logs each stage it takes or gives with the term that decided it. The rule itself is unchanged (eb_desc_target, the EBS rows of
// run_pcc.sh hold it). It stays out of the way of the driver: it only starts with the brake off and no key held, hands the brake back at once when the state is not what it
// left it (a key press), and puts the driver's previous state back when it ends.
static struct {
    float on, margin, step, hold; double cfg_at;   // [ebrake] descent keys, pushed by the module
    float view_from, view_target;                  // Cruise's strict overspeed view of this tick (NAN = the game's overspeed braking does not act)
    int own;                 // we hold the brake (on / stage / flag are ours)
    char *ga;                // the game_actor we hold it on
    eb_full orig, last;      // the driver's state before we took over; the state we left it in
    eb_desc d;               // the rule's state
    double lock_until;       // the driver touched the keys: hands off until then
    int release_req;         // a reset asked for the release: the next tick does it (not from the reset's thread)
    double hold_until;       // sim_s() until which the gearbox keeps its gear (the stages just came off)
    int stage, cap;          // the stage we hold (0 = none) and the engine's stage count
    double ep_t0; unsigned ep_steps; int ep_max;
    unsigned n_ep, n_steps, n_driver; int lines;
    // the allocation (an event of any asker is carried by the stage that fits it first): the deceleration the stages may carry now (the largest of the channel's need,
    // the lever assist's wish and the pedal's, the tick before's), the stage it asks, when it last changed, the stage the descent rule asks (it is the rule's own state)
    float alloc_ms2; volatile float wish; volatile double wish_at;   // (the wish is written by Decel on the frame thread and read by the tick on the game's)
    int alloc_stage, rule_stage; double alloc_at;
    int alloc_learned;       // the pedal's wish is the largest: it is open loop (the provider has no trim), so a stage is taken for it only within the pedal's tolerance (BC_EB_PEDAL_TOL) of what is not counted
    int up_flag;             // the allocation stepped a stage up this tick: the measurement of the step arms
    int steady;              // the measured deceleration is steady (within BC_EB_STEADY of its running mean for 0.8 s): a step up waits for it
} BE = { 0, 0, 0, 0, -1e9, NAN, NAN };

void brake_eb_config(float on, float margin_kmh, float step_s, float hold_s) { BE.on = on; BE.margin = margin_kmh; BE.step = step_s; BE.hold = hold_s; BE.cfg_at = wall_s(); }
void brake_descent_view(float from_kmh, float target_kmh) { BE.view_from = from_kmh; BE.view_target = target_kmh; }

static int be_read(char *ga, eb_full *f)
{
    int held, on, flag; unsigned stage;
    if (!sense_ebrake_state(ga, &held, &on, &stage, &flag)) return 0;
    f->on = on; f->stage = stage; f->flag = flag; f->held = held;
    return 1;
}
// A line in the log: the first ones always, then only at log level 2 (as the module had it)
#define BELOG(...) do { if (BE.lines < 120) { BE.lines++; logline(__VA_ARGS__); } else logv(2, __VA_ARGS__); } while (0)

static void be_end(char *ga, double now, const char *why)
{
    if (!BE.own) return;
    if (ga && ga == BE.ga) act_ebrake_set("brake channel", ga, BE.orig.on, BE.orig.stage, BE.orig.flag);
    BELOG("Brake: engine brake stages end (%s) after %.0f s, %u steps, highest stage %d; your setting is back: brake %s, stage %u",
          why, now - BE.ep_t0, BE.ep_steps, BE.ep_max, BE.orig.on ? "on" : "off", BE.orig.stage);
    BE.own = 0; BE.stage = 0; BE.d.stage = 0; BE.d.changed_at = now; BE.alloc_stage = 0; BE.rule_stage = 0; BE.alloc_at = now;
    if (BE.hold > 0 && (!strcmp(why, "off") || !strcmp(why, "under the target"))) BE.hold_until = now + BE.hold;   // the gear stays for a while (the stages come back on a long descent)
}

// What the stage we would hold is counted to give in an event: the vehicle model's worth at the gear and the revs of now, and while no step of this stage has told what it is worth (no
// event behind it) only BM_EB_ACCEPT_LO of it -- the lowest a figure may be and still be an engine brake's (core/brakemodel.h): the start value is one point of one engine, and a stage
// that gives MORE than counted is only the loop's correction (the trim takes the surplus off the retarder and the service brake), one that gives LESS is the retarder's and the
// service brake's to make up. The allocation picks by the full worth (a fit), the need is reduced by this.
// The share of the model's figure that counts: BM_EB_ACCEPT_LO until there are samples of this truck's engine brake (the allocation measures its own steps, below, so a truck that is braked
// with the pedal alone gets them too), then more with each, all of it at BC_EB_FULL_EVENTS. A stage without a sample of its own takes the others' mean in the model.
static float be_cm()
{
    const float n = (float)vehicle_eb_events_total() / (float)BC_EB_FULL_EVENTS;
    return n >= 1.0f ? 1.0f : n > BM_EB_ACCEPT_LO ? n : BM_EB_ACCEPT_LO;
}
static float be_credit(int stage, int cap)
{
    if (stage < 1 || cap < 1 || T.gear <= 0) return 0.0f;
    return vehicle_eb_decel_at(stage, cap, T.gear, T.rpm) * be_cm();
}

// The allocation: the stage that fits the deceleration the stages may carry now (BE.alloc_ms2), the owner's order (the engine brake before the retarder, keuzes 20.4 / 39). One stage
// at a time and a dwell between the steps (the game's stage is a whole step of the engine's brake; the retarder, which is continuous, takes what the stage does not fit). Nothing in a
// shift gap (the engine brake does nothing while the clutch is open: the stage stays as it is and its credit is 0, the service brake bridges as before), none with the driver on the gas.
static int be_alloc(act_tick *a, double now, unsigned cap)
{
    const int s0 = BE.alloc_stage;
    const int go = bk_ebs >= 0.5f && cap >= 1 && cap <= EB_CAP_MAX && T.engine_on && !T.paused && T.gear > 0 && !(a->pedal > 0.1f) && !(T.eff_thr >= 0.05f);
    int w = s0;
    if (!go) w = 0;
    else if (clutch_seen() > 0.05f) w = s0;
    else {
        const float X = BE.alloc_ms2;
        const float sel0 = s0 > 0 ? vehicle_eb_decel_at(s0, (int)cap, T.gear, T.rpm) : 0.0f;
        int best = 0;
        for (int s = 1; s <= (int)cap; s++) {
            const float c = vehicle_eb_decel_at(s, (int)cap, T.gear, T.rpm);
            if (BE.alloc_learned && (1.0f - be_cm()) * c > BC_EB_PEDAL_TOL * X) continue;
            if (c >= BC_EB_MIN && c <= BC_EB_FIT_UP * X) best = s;
        }
        const double since = now - BE.alloc_at;
        if (best > s0) { if (since >= BC_EB_UP_S && (BE.steady || (BE.alloc_learned && be_cm() >= 1.0f) || (BC.active && BC.urgency != BRU_COMFORT))) { w = s0 + 1; BE.up_flag = 1; } }   // (a calm ask or the lever's wish waits for a steady deceleration so that the step is measured, what it adds can not be told from an ask that is building up; a firm ask does not wait, nor does the pedal once the model is measured: their first stage is not measured, the next ones are; until then the pedal waits too)
        // (no request any more -- the ask ended, the pedal or the lever was let go -- and the stage is off in this tick: after a lift there is nothing for it to serve, and the old TrueShift rule
        // that follows a held stage would pull the box down for it; only a step DOWN while the request goes on keeps the dwell)
        else if (s0 > 0 && !(X > 0)) w = 0;
        else if (s0 > 0 && since >= BC_EB_DN_S && (sel0 > BC_EB_FIT_DN * X || sel0 < 0.5f * BC_EB_MIN)) w = s0 - 1;
    }
    if (w != s0) { BE.alloc_stage = w; BE.alloc_at = now; }
    return w;
}

// The allocation's own step, measured (WP-BR3): when a stage is taken, what the measured deceleration gained 0.7-1.0 s later, less what the retarder and the service brake were commanded to give
// more or less meanwhile (the trim takes the surplus of a stage off them; the pedal's service share is the wish less the credit) and what the truck does by itself, is that stage's step. It
// goes to the vehicle model as the force of the stage in this gear (the stage below's plus the step, over the share of its torque the revs give: the correction is of the full torque), the
// same shape as the descent rule's d_meas. So a truck that is braked with the lever or the pedal alone, with no cruise and no gap bridge of an engine-brake-only truck, learns its engine
// brake from the first events. Valid only where nothing else moved: in gear, no gas, the stage, the gear, the retarder lever and its factor, the ask and the pedal as they were.
static float bc_mass();
static struct {
    int armed, stage, gear; unsigned ret_lvl; double t0;
    float a_pre, cmd_pre, nat_pre, ref_pre, raw_pre, scale_pre;
    float a_sum, cmd_sum, nat_sum; int n;
    float a_ema, a_fast, ema_s; int ema_ok;
    float r_l, s_l;
    unsigned n_notes, n_tries;
} ST;
// The deceleration the retarder (as modelled) and the service brake (the larger of our request and the pedal's, through the gain) are commanded to give, as the measurement sees it: the
// measured deceleration is a speed fall over 0.3 s (it trails the truck by 0.15 s) and the service brake trails its request by 0.15 s (BC_LAG), so the service share is smoothed over 0.3 s
// and the retarder's (its model has the build-up already) over 0.15 s; a change that is still going on when a step is measured is then compared at the same time.
static float bc_cmd_lagged(float dt)
{
    float srv = act_request();
    if (BP.raw > 0 && BP.out > srv) srv = BP.out;
    ST.r_l += (BC.a_ret - ST.r_l) * dt / (BC_EB_LAG_R + dt);
    ST.s_l += (srv * brake_gain(NULL) - ST.s_l) * dt / (BC_EB_LAG_S + dt);
    return ST.r_l + ST.s_l;
}
static void be_measure(act_tick *a, double now, float dt)
{
    const int gap = BC.gap || clutch_seen() > 0.05f || T.gear <= 0;
    const float cmd = bc_cmd_lagged(dt);
    if (gap || !BC.meas_ok) { ST.ema_ok = 0; ST.ema_s = 0; }
    else {   // (the running means go on while a step is measured: the next step is a second after, and its "before" is this one's response)
        const float k = dt / (BC_EB_MEAS_TAU + dt);
        if (ST.ema_ok && fabsf(BC.a_meas - ST.a_ema) > BC_EB_JUMP) ST.ema_ok = 0;   // a jump (a pedal pressed, a gap's end): the mean starts over, and a step waits for it
        if (!ST.ema_ok) { ST.a_ema = ST.a_fast = BC.a_meas; ST.ema_ok = 1; ST.ema_s = 0; }
        else { ST.a_ema += (BC.a_meas - ST.a_ema) * k; ST.a_fast += (BC.a_meas - ST.a_fast) * dt / (BC_EB_FAST_TAU + dt); ST.ema_s += dt; }   // (the "before" of a step is the fast mean, the last 0.3 s; the slow one tells steady)
    }
    if (ST.armed) {
        const double age = now - ST.t0;
        if (age >= BC_EB_MEAS_TO) {   // the window is over (the next step up, a second after, is taken in this very tick: what was gathered before it is the step's)
            ST.armed = 0;
            const float inv = ST.n >= 5 ? 1.0f / (float)ST.n : 0.0f;
            const float share = vehicle_eb_rpm_share(T.rpm);
            const float d_eb = (ST.a_sum * inv - ST.a_pre) - (ST.cmd_sum * inv - ST.cmd_pre) - (ST.nat_sum * inv - ST.nat_pre);
            if (inv > 0 && share >= BC_EB_NOTE_SHARE && d_eb >= BC_EB_NOTE_MIN && d_eb <= 3.0f && BE.cap >= ST.stage) {
                const float f = vehicle_eb_force(ST.stage - 1, BE.cap, ST.gear) + d_eb * bc_mass() / share;
                vehicle_eb_note(ST.stage, BE.cap, ST.gear, f);
                ST.n_notes++;
                logv(2, "Brake: engine brake stage %d of %d measured: the deceleration gained %.2f m/s2 (%.2f -> %.2f; commanded %+.2f, natural %+.2f), %.2f in full torque at gear %d, %.0f rpm; %u samples now", ST.stage, BE.cap,
                     ST.a_sum * inv - ST.a_pre, ST.a_pre, ST.a_sum * inv, ST.cmd_sum * inv - ST.cmd_pre, ST.nat_sum * inv - ST.nat_pre, d_eb / share, ST.gear, T.rpm, (unsigned)vehicle_eb_events_total());
            }
        // nothing else moved: the stage as we set it, the gear, the retarder's lever and factor, the gas, the ask and the pedal
        } else if (gap || !BC.meas_ok || BE.alloc_stage != ST.stage || T.gear != ST.gear || T.retarder != ST.ret_lvl || fabsf(g_ret_scale - ST.scale_pre) > 0.02f || a->pedal > 0.1f ||
                   fabsf(BC.ref - ST.ref_pre) > 0.15f || fabsf(BP.raw - ST.raw_pre) > 0.03f) ST.armed = 0;
        else if (age >= BC_EB_MEAS_FROM) { ST.a_sum += BC.a_meas; ST.cmd_sum += cmd; ST.nat_sum += BC.nat; ST.n++; }
    }
    if (BE.up_flag) {   // a stage was taken in this tick's allocation
        BE.up_flag = 0;
        if (ST.ema_ok && ST.ema_s >= BC_EB_SETTLE_S && BE.steady && !gap && BE.alloc_stage > 0) {
            ST.armed = 1; ST.stage = BE.alloc_stage; ST.gear = T.gear; ST.ret_lvl = T.retarder; ST.t0 = now; ST.n = 0; ST.a_sum = ST.cmd_sum = ST.nat_sum = 0;
            ST.a_pre = ST.a_fast; ST.cmd_pre = cmd; ST.nat_pre = BC.nat; ST.ref_pre = BC.ref; ST.raw_pre = BP.raw; ST.scale_pre = g_ret_scale; ST.n_tries++;
        }
    }
}

// ACT_POST, after Cruise has posted (its view of the overspeed braking is of this tick).
static void be_tick(act_tick *a, double now)
{
    char *ga = a->ga;
    if (!ga) return;
    const int cfg_ok = BE.on > 0 && now - BE.cfg_at < 2.0;
    if (!cfg_ok || BE.release_req) {
        BE.release_req = 0;
        be_end(ga, now, "reset");
        return;
    }
    if (!(a->dt > 0 && a->dt < 0.5f)) return;   // a pause or a hitch: the state stays as it is
    eb_full cur;
    if (!be_read(ga, &cur)) return;
    if (BE.own && ga != BE.ga) { BE.own = 0; BE.stage = 0; BE.d.stage = 0; BE.alloc_stage = 0; BE.rule_stage = 0; }   // another vehicle: nothing of ours is left to give back
    if (BE.own) {
        if (eb_driver_touched(BE.last, cur)) {   // a key: the brake is the driver's from here on, as the driver left it
            BE.own = 0; BE.stage = 0; BE.d.stage = 0; BE.d.changed_at = now; BE.lock_until = now + 30; BE.n_driver++; BE.alloc_stage = 0; BE.rule_stage = 0;
            BELOG("Brake: engine brake stages handed back (you touched it): brake %s, stage %u%s", cur.on ? "on" : "off", cur.stage, cur.held ? ", key held" : "");
            return;
        }
    } else if (cur.held || cur.on || now < BE.lock_until) {   // the driver's own brake, hands off after a touch
        BE.d.stage = 0; BE.alloc_stage = 0; BE.rule_stage = 0;
        return;
    }
    const unsigned cap = sense_ebrake_cap(ga);
    BE.cap = (int)cap;
    const int view = BE.view_from == BE.view_from && BE.view_target == BE.view_target;
    const float from = BE.view_from, target = BE.view_target;
    // what one stage is worth in the gear of now (m/s2): the vehicle model's engine brake (the start value from the engine's brake torque through the gear, then
    // what was learned for this truck), at the stage the control is at (stage 1's while it has none)
    const int ref = BE.d.stage > 0 ? BE.d.stage : 1;
    const float d_stage = vehicle_eb_decel(ref, (int)cap, T.gear) - vehicle_eb_decel(ref - 1, (int)cap, T.gear);
    eb_desc_in in = { now, fabsf(T.speed) * 3.6f, view ? from : NAN, view ? target : NAN, T.accel, T.eff_thr, a->pedal,
                      T.cruise > 0.5f, T.engine_on, T.paused, clutch_seen() > 0.05f, T.gear, cap, d_stage };
    eb_desc_cfg cfg = eb_desc_defaults();
    cfg.margin_kmh = BE.margin; cfg.step_s = BE.step; cfg.down_s = 2.0f * BE.step;
    const char *why = "";
    const int before = BE.stage;
    BE.rule_stage = eb_desc_target(BE.d, in, cfg, &why);
    // (the descent rule owns the stage while it holds one: its rows and the trim's closed loop on it are as they always were; the allocation takes over when the rule has none)
    int alloc = 0;
    if (BE.rule_stage > 0) { BE.alloc_stage = 0; BE.alloc_at = now; }
    else alloc = be_alloc(a, now, cap);
    const int want = BE.rule_stage > alloc ? BE.rule_stage : alloc;
    if (want > BE.rule_stage) why = "allocated";
    if (BE.d.d_meas > 0) {   // the step up that just ran told what its stage is worth: the model gets it as that stage's force (the stage below's plus the step's)
        const int st = BE.d.meas_stage;
        if (st >= 1 && (int)cap >= st) vehicle_eb_note(st, (int)cap, T.gear, vehicle_eb_force(st - 1, (int)cap, T.gear) + BE.d.d_meas * (V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK));
        BE.d.d_meas = 0;
    }
    if (!BE.own) {
        if (want <= 0) return;
        BE.orig = cur; BE.own = 1; BE.ga = ga; BE.ep_t0 = now; BE.ep_steps = 0; BE.ep_max = 0; BE.n_ep++;
        if (BE.rule_stage > 0)
            BELOG("Brake: engine brake stages start at %.1f km/h (the game's overspeed braking would start at %.1f, holding %.1f), stage 1 of %u (worth %.2f m/s2 in gear %d, %.0f rpm; cost %.2f per m/s2 against the service brake's %.2f); yours: brake %s, stage %u",
                  in.kmh, from, target, cap, vehicle_eb_decel(1, (int)cap, T.gear), T.gear, T.rpm, BC_COST[BS_EB], BC_COST[BS_SRV], cur.on ? "on" : "off", cur.stage);
        else
            BELOG("Brake: engine brake stage %d of %u for %.2f m/s2 of the event at %.1f km/h (worth %.2f m/s2 in gear %d, %.0f rpm, counted %.2f; cost %.2f per m/s2 against the retarder's %.2f and the service brake's %.2f); yours: brake %s, stage %u",
                  want, cap, BE.alloc_ms2, in.kmh, vehicle_eb_decel_at(want, (int)cap, T.gear, T.rpm), T.gear, T.rpm, be_credit(want, (int)cap), BC_COST[BS_EB], BC_COST[BS_RET], BC_COST[BS_SRV], cur.on ? "on" : "off", cur.stage);
    }
    if (want > 0) {
        if (!cur.on || (int)cur.stage != want || !cur.flag) act_ebrake_set("brake channel", ga, 1, (unsigned)want, 1);
        be_read(ga, &BE.last);
        if (want != before) {
            BE.n_steps++; BE.ep_steps++;
            if (before > 0)
                BELOG("Brake: engine brake stage %d -> %d (%s) at %.1f km/h, allowed from %.1f, holding %.1f, accel %.2f m/s2, gear %d, %.0f rpm",
                      before, want, why, in.kmh, from, target, T.accel, T.gear, T.rpm);
        }
        if (want > BE.ep_max) BE.ep_max = want;
        BE.stage = want;
    } else {
        if (before > 0) { BE.n_steps++; BE.ep_steps++; }
        be_end(ga, now, why);
    }
}

static FILE *s_csv;

static inline float bc_clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float bc_mass() { return V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK; }

int brake_channel_on() { return BC.running && g_brake_channel >= 0.5f; }
void brake_ask(int who, float decel, int urgency)
{
    if (who < 0 || who >= BRQ_N) return;
    BC.ask[who] = decel > 0 ? decel : 0;   // (NAN: none)
    BC.add[who] = 0;
    BC.urg[who] = urgency < 0 ? 0 : urgency >= BRU_N ? BRU_N - 1 : urgency;
}
void brake_patience(int who, float owe_ms)
{
    if (who < 0 || who >= BRQ_N) return;
    BC.pat[who] = owe_ms > 0 ? owe_ms : 0;
}
void brake_ask_add(int who, float brake_decel, int urgency)
{
    brake_ask(who, brake_decel, urgency);
    if (who >= 0 && who < BRQ_N) BC.add[who] = 1;
}
float brake_asked(int who) { return who >= 0 && who < BRQ_N ? BC.ask[who] : 0; }
void brake_eb_wish(float decel) { BE.wish = decel > 0 ? decel : 0; BE.wish_at = sim_s(); }
float brake_eb_credit() { return BC.eb_credit; }
float brake_eb_worth()
{
    return BE.own && BE.alloc_stage > 0 && BE.stage >= BE.alloc_stage && BE.cap > 0 && T.gear > 0 && clutch_seen() <= 0.05f ? vehicle_eb_decel_at(BE.alloc_stage, BE.cap, T.gear, T.rpm) : 0.0f;
}
float brake_asks_max()
{
    float m = 0;
    for (int i = 0; i < BRQ_N; i++) if (BC.ask[i] > m) m = BC.ask[i];
    return m;
}
void brake_game_overspeed(float from_kmh) { BC.game_from_kmh = from_kmh; }

float brake_gain(int *learned)
{
    float g = vehicle_brake_gain();
    if (learned) *learned = g == g && g > 1.0f && g < 30.0f;   // the slot is confident (the pedal provider and the channel's pedal keep to this)
    return vehicle_brake_est();                                // the model's estimate: the prior until the slot weighs in (core/vehicle.h, "the brake model")
}

// What the truck slows by itself now (m/s2, - = it speeds up): the vehicle model's natural deceleration (core/vehicle.cpp: rolling, air, the grade
// the game reports, its learned bias and, in gear with the throttle shut and the clutch closed, the engine's drag, the same one the gearbox picks a
// drag gear by).
static float bc_natural(float v, float m)
{
    (void)m;
    int in_gear = T.gear > 0 && clutch_seen() <= 0.05f && T.eff_thr < 0.05f && V.eng_ok && v > 1.0f && T.rpm > 300;
    return vehicle_natural_decel(in_gear ? T.gear : 0, v);
}

// The measured deceleration: the speed's fall over the last BC_MEAS_WIN seconds (sim time).
static void bc_measure(float v, double now)
{
    if (v != v) { BC.meas_ok = 0; return; }
    BC.hi = (BC.hi + 1) % 64; BC.hv[BC.hi] = v; BC.ht[BC.hi] = now;
    if (BC.hn < 64) BC.hn++;
    BC.meas_ok = 0;
    for (int i = 1; i < BC.hn; i++) {
        int j = (BC.hi - i + 64) % 64;
        double span = now - BC.ht[j];
        if (span >= BC_MEAS_WIN) {
            if (span < 1.0) { float a = (float)((BC.hv[j] - v) / span); if (fabsf(a) < 30) { BC.a_meas = a; BC.meas_ok = 1; } }
            break;
        }
    }
}

// One line per decision change (promise P7): who asks, and which stages carry the event and why (the term that decided the split), with the cost of the allocation in the one unit
// of BC_COST. A decision changes when the asker, the deciding term or the set of stages does; not more often than every 0.5 s (a change that does not stand that long is noise
// at the frame's rate; it is logged at the next frame it still stands). The release says what the event used.
static const char *const bc_stage_name[BS_N] = { "coast", "drag gear", "engine brake", "retarder", "service" };
static const char *bc_term_text(int t)
{
    static const char *const x[BT_N] = { "no event", "coasting and the held stages give it", "the retarder carries it", "more than the retarder's full force: the service brake fills the rest",
                                         "the retarder is still building up: the service brake fills the shortfall", "a shift gap: the retarder is gone, the service brake bridges",
                                         "low speed: the retarder fades, the service brake takes over", "urgent: the service brake at once" };
    return t >= 0 && t < BT_N ? x[t] : "?";
}
static void bc_log_event(float kmh)
{
    static const char *const nm[BRQ_N] = { "descent", "speed limit line", "bend", "junction", "ACC", "planner", "speed arbiter" };
    static const char *const un[BRU_N] = { "comfort", "firm", "urgent" };
    double now = wall_s();
    int who = BC.active ? BC.who : -1;
    const int key = who < 0 ? -1 : who | BC.term << 4 | BC.stages << 8;
    if (key == BC.log_key || now - BC.log_at < 0.5) return;
    if (who >= 0) {
        char st[96]; size_t k = 0; st[0] = 0;
        for (int s = 0; s < BS_N; s++) if (BC.stages & 1 << s) { int w = snprintf(st + k, sizeof st - k, "%s%s", k ? " + " : "", bc_stage_name[s]); if (w > 0) k = k + (size_t)w < sizeof st ? k + (size_t)w : sizeof st - 1; }
        logv(1, "Brake: %s asks %.2f m/s2 (%s) at %.0f km/h: %s; %s. coasting %.2f, engine brake %.2f (stage %d/%d), retarder %.2f (game %.2f, ours %.2f), service %.2f m/s2 = %.2f%s; cost %.2f = engine brake %.2f x %.2f + retarder %.2f x %.2f + service %.2f x %.2f",
             nm[who], BC.want, un[BC.urgency], kmh, st, bc_term_text(BC.term), BC.nat, BC.a_eb, BE.own ? BE.stage : 0, BE.cap, BC.a_ret, BC.game_d, BC.feed, BC.srv, BC.req,
             BC.drag_n > 0 ? ", more engine drag asked of the gearbox" : "", BC.cost, BC.a_eb, BC_COST[BS_EB], BC.a_ret, BC_COST[BS_RET], BC.srv, BC_COST[BS_SRV]);
    }
    else logv(1, "Brake: released at %.0f km/h (most service %.2f m/s2, retarder %.2f m/s2)", kmh, BC.lvl_srv_max, BC.lvl_ret_max);
    if (who < 0) BC.lvl_srv_max = BC.lvl_ret_max = 0;
    BC.log_who = who; BC.log_key = key; BC.log_at = now;
}

// The retarder's force at full demand, learned while it brakes alone and steadily on a level road: the measured deceleration
// minus what coasting gives, over the demand it has built up. The sample goes to the vehicle model (core/vehicle.cpp), which blends it
// into the top stage (weight 1/(n+1), at least 0.05, within 0.4-2.5x the start value) and keeps it per truck. Load-independent (a force), so a new trailer keeps it.
static void bc_learn(float dt, float v, float m, float game_d)
{
    int ok = BC.meas_ok && BC.bl > 0.3f && BC.env >= 1.0f && fabsf(fmaxf(fmaxf(BC.feed, game_d), BP.feed) - BC.bl) < 0.05f &&
             BC.req <= 0 && act_request() <= 0 && T.in_brake < 0.01f && T.eff_brake < 0.01f && T.eff_thr < 0.02f && T.retarder == 0 &&
             v * 3.6f > 15 && fabsf(V.grade_pct) < 1.0f && fabsf(T.yaw) < 0.04f && !T.paused && V.mass_ok;
    BC.learn_s = ok ? BC.learn_s + dt : 0;
    if (BC.learn_s < 0.5f) return;
    BC.learn_s = 0;
    float x = m * (BC.a_meas - BC.nat) / BC.bl;
    // (the vehicle model takes the sample: within 0.4-2.5 times the start value, blended per stage, kept per truck)
    vehicle_ret_note(vehicle_ret_steps(), x);
    float f = vehicle_ret_force_full();
    if (fabsf(f - BC.ret_force) > 0.1f * BC.ret_force || (vehicle_ret_events(vehicle_ret_steps()) == 5 && wall_s() - BC.learn_told > 60)) {
        BC.learn_told = wall_s();
        logv(1, "Brake: retarder force at full demand %.0f kN (%.2f m/s2 at %.1f t; %d samples)", f / 1000, f / m, m / 1000, vehicle_ret_events(vehicle_ret_steps()));
    }
}

// The retarder's stage force from a steady lever step (the M2 note TODO(BR): the channel learned only the top stage, from its own fed demand). The driver's (or the game's
// automatic) retarder lever holds step k of n with the retarder alone: nothing of ours feeds it (no ask, no fed demand, the game's overspeed braking off), the service brake and the
// engine brake are off, the throttle is shut with the clutch closed in a gear (the engine's drag is in the natural deceleration), on a level road above 15 km/h, our torque factor at 1.
// The step must have been held 6 build-up time constants (the retarder is at its level: exp(-6) = 0.25 %), then a window of 1 s is averaged; a window whose measured deceleration
// moves by more than 0.25 m/s2 (a bump, a shift beginning, a bend) is not a sample. The sample is the force: mass x (the mean measured deceleration - what the truck does by
// itself); the vehicle model blends it into that stage (weight 1/(n+1), at least 0.05, within 0.4-2.5 x the start value, kept per truck).
static const float BC_LV_SETTLE_S = 6.0f * BC_RET_TAU;   // s the step is held before a window opens
static const float BC_LV_WIN_S = 1.0f;                   // s, the window averaged
static const float BC_LV_FMIN = 0.05f;                   // the factor under which a window is not read: the figure is the measurement over the factor, and the measurement's noise (~0.05 m/s2) over a twentieth of a stage is a whole stage
static const float BC_LV_SPREAD = 0.25f;                 // m/s2: the most the measured deceleration may move inside a window (twice the measurement's noise of ~0.1 at 0.3 s)
static void bc_learn_lever(float dt, float v, float m)
{
    const int k = (int)T.retarder, steps = (int)T.retarder_steps;
    // The factor on the game's retarder (the deceleration assist's fade, WP-FX1) is KNOWN: it is the one we post, g_ret_scale. A window teaches the step's force whatever it was (force = what the retarder gave over the
    // factor), so the model converges while the fade runs; before, only a factor of 1 taught, and a lever that always runs at the fade's share taught nothing. The factor is read through the retarder's own lag
    // (the torque follows a change of the factor with BC_RET_TAU), and a window in which it moved more than a tenth of itself teaches nothing: the lag then biases the figure by more than it is worth.
    const int ok = steps > 0 && k >= 1 && k <= steps && BC.meas_ok && !BC.active && BC.feed == 0 && BC.game_d == 0 && BP.feed == 0 && BC.bl < 0.05f && BC.req <= 0 && act_request() <= 0 &&
                   g_ret_scale > BC_LV_FMIN && T.in_brake < 0.01f && T.eff_brake < 0.01f && T.eff_thr < 0.02f && !T.motor_brake && !(BE.own && BE.stage > 0) && T.gear > 0 &&
                   clutch_seen() <= 0.05f && !BC.gap && BC.env >= 1.0f && v * 3.6f > 15 && fabsf(V.grade_pct) < 1.0f && fabsf(T.yaw) < 0.04f && !T.paused && V.mass_ok;
    if (!ok || k != BC.lv_k) { BC.lv_k = ok ? k : 0; BC.lv_held_s = 0; BC.lv_win_s = 0; BC.lv_n = 0; BC.lv_ff = g_ret_scale; return; }
    BC.lv_ff += (g_ret_scale - BC.lv_ff) * dt / (BC_RET_TAU + dt);
    BC.lv_held_s += dt;
    if (BC.lv_held_s < BC_LV_SETTLE_S) return;
    if (BC.lv_n == 0) { BC.lv_sum = BC.lv_nat = BC.lv_fsum = 0; BC.lv_min = BC.lv_fmin = 1e9f; BC.lv_max = BC.lv_fmax = -1e9f; }
    const float x = (BC.a_meas - BC.nat) / fmaxf(BC.lv_ff, BC_LV_FMIN);   // what the retarder would give at a factor of 1
    BC.lv_sum += BC.a_meas; BC.lv_nat += BC.nat; BC.lv_fsum += BC.lv_ff; BC.lv_n++; BC.lv_win_s += dt;
    BC.lv_min = fminf(BC.lv_min, x); BC.lv_max = fmaxf(BC.lv_max, x);
    BC.lv_fmin = fminf(BC.lv_fmin, BC.lv_ff); BC.lv_fmax = fmaxf(BC.lv_fmax, BC.lv_ff);
    if (BC.lv_win_s < BC_LV_WIN_S) return;
    const float n = (float)BC.lv_n, mean = BC.lv_sum / n, nat = BC.lv_nat / n, f = BC.lv_fsum / n;
    const int steady = BC.lv_max - BC.lv_min < BC_LV_SPREAD && BC.lv_fmax - BC.lv_fmin <= 0.1f * f;
    BC.lv_win_s = 0; BC.lv_n = 0;
    if (!steady) return;
    vehicle_ret_note(k, m * (mean - nat) / f);
}

// The pedal side's values for the provider (latched while the pedal is released).
static void bc_pedal_latch(float kmh, float m)
{
    const char *why = NULL;
    int learned = 0;
    float G = brake_gain(&learned);
    if (!(g_brake_pedal >= 0.5f)) why = "off (ini)";
    else if (!game_brake_pedal_ok()) why = "the pedal hook is not in ([core] brake_pedal_hook)";
    else if (!learned) why = "the brake's gain is not learned yet ([core] brake_learn)";
    else if (!V.mass_ok) why = "the mass is not known";
    else if (!T.engine_on) why = "the engine is off";
    else if (T.gear < 0) why = "reverse";
    else if (V.electric) why = "an electric truck";
    else if (T.paused) why = "paused";
    BP.kmh = kmh;
    BP.ret_full_nom = BC.ret_full_nom;
    BP.a_ret = BC.a_ret;
    // (review 30-10: the blend subtracts the MODELLED retarder from the driver's ask, open loop: only once the retarder's force is learned
    // from this truck (a few clean samples), else a too strong prior would brake the driver too weakly; the service brake does it all until then)
    BP.blend = bk_pblend >= 0.5f && game_retarder_feed_ok() && T.retarder_steps > 0 && vehicle_ret_events(vehicle_ret_steps()) >= 5;
    BP.calls = 0;
    if (!why && T.in_brake < 0.01f) {   // only while released: the pedal never changes under the foot
        float mu = V.mass - (T.cargo_mass == T.cargo_mass && T.cargo_mass > 0 && T.cargo_mass < V.mass ? T.cargo_mass : 0);
        float top = fminf(fmaxf(bk_ptop, 1.0f), G * m / fmaxf(mu, 1000.0f));
        if (!BP.ready || fabsf(top - BP.top) > 0.05f * BP.top || fabsf(G - BP.G) > 0.05f * BP.G)
            logv(1, "Brake pedal: linear to %.1f m/s2 over the travel, end blend ^%.0f (gain %.1f m/s2 per request at %.1f t, unladen %.1f t)%s",
                 top, bk_plin, G, m / 1000, mu / 1000, BP.blend ? ", retarder first" : "");
        BP.G = G; BP.top = top; BP.ready = 1;
    } else if (why) BP.ready = 0;
    if (why != BP.why && (BP.told || g_brake_pedal >= 0.5f)) { if (why) logv(1, "Brake pedal: raw (%s)", why); else logv(1, "Brake pedal: mapped"); BP.told = 1; }
    BP.why = why;
}

float brake_pedal_map(float raw, float req, void *ga)
{
    (void)req; (void)ga;
    BP.raw = raw;
    unsigned c = BP.calls; BP.calls = c + 1;
    float out = raw;
    float feed = BP.auto_ret && raw > 0 ? fminf(raw, 1.0f) : 0;   // the game's auto retarder takes the raw pedal (it is maxed with ours)
    float want = NAN;
    if (BP.ready && c <= BP_STALE_CALLS && raw == raw && raw > 0 && raw < 0.98f && BP.G > 0 && BP.top > 0) {
        float a_w = raw * BP.top;   // linear over the whole travel (Damien 30-10: no fixed point where it becomes full brakes)
        float a_rs = fmaxf(0.0f, a_w - BP.eb);   // (what the engine-brake stage the allocation holds is counted to give is not the retarder's or the service brake's: the stage comes first)
        float a_ret = 0;
        if (BP.blend && BP.ret_full_nom > 0.05f) {
            feed = fmaxf(feed, bc_clamp(a_rs / BP.ret_full_nom, 0, 1));
            a_ret = BP.a_ret;   // what the retarder gives now (built up; the service brake fills the rest)
        } else if (BP.auto_ret) a_ret = BP.a_ret;
        float r = fmaxf(0.0f, a_rs - a_ret) / BP.G;
        // towards the end of the travel the request blends into the raw pedal (only ever up): continuous where the hook takes over at 0.98
        if (r < raw) r += (raw - r) * powf(fminf(1.0f, raw / 0.98f), bc_clamp(bk_plin, 2, 20));
        // never weaker than the raw pedal at a crawl and standing (a hold on a grade, the stop's last metres)
        float kmh = BP.kmh, fl = kmh <= BC_PED_FLOOR_LO ? 1.0f : kmh >= BC_PED_FLOOR_HI ? 0.0f : (BC_PED_FLOOR_HI - kmh) / (BC_PED_FLOOR_HI - BC_PED_FLOOR_LO);
        r = fmaxf(r, raw * fl);
        out = bc_clamp(r, 0, 1);
        want = a_w;
    } else if (!(raw > 0)) feed = 0;
    BP.feed = feed;
    BP.out = out; BP.want = want;
    if (BP.blend || BC.feed > 0) game_retarder_feed(fmaxf(BC.feed, BP.blend ? feed : 0));
    return out;
}

// ---------------------------------------------------------------- the tick (ACT_POST, after Cruise has asked)
static void bc_tick(act_tick *a)
{
    float dt = a->dt;
    if (!(dt > 0 && dt < 0.5f)) return;   // a pause or a hitch: what is posted stays one tick more
    double now = sim_s();
    BC.tick_at = now; BC.ticks++;
    float v = fabsf(T.speed), kmh = v * 3.6f, m = bc_mass();
    bc_measure(v, now);
    int chan = g_brake_channel >= 0.5f;
    // the retarder: the shift gap's envelope, the fade, the demand (the game's own counts), the build-up
    int has_ret = T.retarder_steps > 0;
    int gap = (T.gear == 0 && v > 1.0f) || (T.gear > 0 && clutch_seen() > 0.05f && v > 1.0f);
    if (gap && !BC.gap) { BC.gap = 1; BC.gap_t0 = now; }
    if (!gap && BC.gap) { BC.gap = 0; BC.gap_t1 = now; }
    BC.env = BC.gap ? fmaxf(0.0f, 1.0f - (float)(now - BC.gap_t0) / BC_RET_FALL)
                    : (BC.gap_t1 > 0 && now - BC.gap_t1 < BC_RET_RISE ? (float)(now - BC.gap_t1) / BC_RET_RISE : 1.0f);
    float fade = bc_clamp((kmh - BC_FADE_LO) / (BC_FADE_HI - BC_FADE_LO), 0, 1);
    BC.ret_force = vehicle_ret_force_full();
    BC.ret_full_nom = has_ret && T.engine_on ? BC.ret_force / m * fade : 0;
    BC.ret_full = BC.ret_full_nom * BC.env;
    float A = BC.game_from_kmh;
    BC.game_d = has_ret && A == A ? bc_clamp(kmh - A, 0, 1) : 0;
    // the asks: the largest wins (its urgency; an equal ask with a higher urgency takes it). The driver's pedals end them.
    BC.nat = bc_natural(v, m);
    // the engine-brake stage (a truck without a retarder): the descent control runs whatever the channel's own switch says, and what the stage we hold is worth is a
    // deceleration the truck has (the vehicle model's figure for the stage in the gear of now; with the clutch open the engine brake does nothing)
    BE.steady = ST.ema_ok && ST.ema_s >= BC_EB_SETTLE_S && BC.meas_ok && fabsf(BC.a_meas - ST.a_ema) < BC_EB_STEADY;
    be_tick(a, now);
    BC.a_eb = BE.own && BE.stage > 0 && BE.cap > 0 && T.gear > 0 && clutch_seen() <= 0.05f ? vehicle_eb_decel_at(BE.stage, BE.cap, T.gear, T.rpm) : 0.0f;
    // the credit of the allocation's stage: what the retarder and the service brake need not give (not the descent rule's stage: the trim has always closed the loop on that one). In a gap it is 0.
    BC.eb_credit = BE.own && BE.alloc_stage > 0 && BE.stage >= BE.alloc_stage && BE.cap > 0 && T.gear > 0 && clutch_seen() <= 0.05f ? be_credit(BE.alloc_stage, BE.cap) : 0.0f;
    BP.eb = BC.eb_credit;
    float want = 0; int who = -1, urg = BRU_COMFORT;
    // (the driver's brake pedal cancels the cruise in the game, and with it every ask of Cruise; T.in_brake is not read here: a sim
    // that mirrors our own request into it would end the asks by our own braking)
    // (review 30-10: the engine / reverse gate only for a NEW braking event: one already running is not dropped when the engine stops
    // (start-stop, a stall) or the gear reads reverse for a moment)
    if (chan && !(a->pedal > 0.1f) && (BC.active || (T.engine_on && T.gear >= 0)) && !T.paused)
        for (int i = 0; i < BRQ_N; i++) {
            if (!(BC.ask[i] > 0)) continue;
            // (an ask on top of what the truck does by itself: coasting and the game's own overspeed retarder are added to it)
            float t = BC.add[i] ? BC.ask[i] + BC.nat + BC.game_d * BC.ret_full : BC.ask[i];
            if (t > want + 1e-6f || (fabsf(t - want) <= 1e-6f && BC.urg[i] > urg)) { want = t; who = i; urg = BC.urg[i]; }
        }
    // The stage after the engine brake's: a truck without a retarder whose stages are all on (the gear stays inside the bands up to the crossover, so they cannot be made stronger)
    // and whose speed is still over where the game's own overspeed braking starts (+ 0.5 km/h, the rule's own margin): the service brake takes the rest, the descent ask's law
    // (Cruise's: 0.1 m/s2 per km/h over, one km/h of reach, at most 1.5, comfort), measured against that point. Cruise's own descent ask waits for its allowance, much higher.
    BC.eb_ask = 0;
    if (chan && BE.own && BE.cap > 0 && BE.stage >= BE.cap && BE.view_from == BE.view_from && kmh > BE.view_from + 0.5f && !(a->pedal > 0.1f) && !T.paused) {
        BC.eb_ask = fminf(1.5f, 0.1f * (kmh - BE.view_from + 0.5f));
        if (BC.eb_ask > want + 1e-6f) { want = BC.eb_ask; who = BRQ_DESCENT; urg = BRU_COMFORT; }
    }
    BC.want = want; BC.who = who; if (who >= 0) { BC.urgency = urg; BC.open = BC.add[who]; }
    // The driver's gas ends the event NOW, not at the jerk limit: the let-go below is for an ask that ends (the speed is back, the junction is passed), not for a foot on the
    // pedal. Before, the tail of a comfort ask went on for ref / 1.5 m/s3 -- 1.3 s at 2 m/s2 -- with the retarder fed and the throttle zeroed by the request (the game zeroes the
    // merged throttle while a brake request is set), the built-up retarder another 0.8 s down: ride 02-11 21:24, the retarder on with 92-99 % gas (promise P7 row, sim_brake_channel).
    if (a->pedal > 0.1f && BC.active) { BC.active = 0; BC.ref = BC.refd = 0; BC.on_s = 0; BC.down_s = 0; }
    // the reference: from the measured deceleration (what the truck does now) toward the ask at the urgency's jerk limit
    // (an ask on top of what the truck does by itself starts from that: its brake part from 0)
    if (want > 0 && !BC.active) { BC.ref = BC.open ? want - BC.ask[who] : BC.meas_ok ? bc_clamp(BC.a_meas, -2.0f, want) : fminf(BC.nat, want); BC.refd = BC.ref; BC.on_s = 0; BC.tol = BC_TOL[urg]; }
    float j = BC.urgency == BRU_URGENT ? BC_J_URGENT : BC.urgency == BRU_FIRM ? fmaxf(bk_jf, 0.2f) : fmaxf(bk_jc, 0.2f);
    if (want > 0) {
        BC.active = 1;
        // (it lets go at the jerk limit, an urgent one at BC_JR_URGENT: the old 0.6/s down slew of the writer at the fit's 10 m/s2)
        float jr = fminf(j, BC_JR_URGENT);
        // (an ask on top of what the truck does keeps the old writer's hold band on the way down: the old request stayed until the wish
        // was 0.05 under it, and the own ACC's gap and TTC were tuned with that; it goes at the end, when the ask does)
        float down_to = BC.open ? fminf(BC.ref, want + 0.05f * brake_gain(NULL)) : want;
        BC.ref = want > BC.ref ? fminf(want, BC.ref + j * dt) : fmaxf(down_to, BC.ref - jr * dt);
    } else if (BC.active) {   // let go at the jerk limit until coasting alone gives the reference
        float jr = fminf(j, BC_JR_URGENT);
        BC.ref -= jr * dt;
        if (BC.ref <= fminf(BC.nat, 0.0f) || BC.ref + BC.trim - BC.nat <= 0) BC.active = 0;
    }
    if (!chan) BC.active = 0;
    BC.refd += (BC.ref - BC.refd) * dt / (BC_REF_LAG + dt);
    BC.on_s = BC.active ? BC.on_s + dt : 0;
    // (the engine-brake stage we hold is NOT subtracted: its worth in the model is the stage at the engine brake's full torque through the gear, and inside the bands -- the gear stays
    // under its crossover -- the revs are lower and the real stage gives less: the model said 1.3 m/s2 of a full stage where the plant gave 0.5 at 60 t on 5 %, and the trim, which
    // closes the loop on the measured deceleration, needed minutes to take the difference back. The measurement has the stage in it; the figure is for the cost and the log)
    BC.need0 = BC.active ? BC.ref - BC.nat + BC.trim : 0;
    BC.need = BC.active ? BC.need0 - BC.eb_credit : 0;   // (the descent rule's stage is not subtracted; the allocation's is, by its credit)
    // (WP-SP3 round 3, the jam's service-brake work 6.93 -> 4.47 MJ at 18 t, the classic's 4.38) THE PLAN WAITS FOR THE GEAR. What the traces showed: a braking event in a jam is shifted down
    // through 4-6 gears, each shift is a gap of 0.5-1.7 s with the retarder gone, and the channel bridged every gap with the service brake at the whole ask while it fed the retarder only the ask
    // (demand = need / full force, 0.3-0.4): when the clutch closed the retarder rose to that small share and the service brake did the rest of the event (0.38 MJ of 0.54 in a wave, the
    // classic ACC's own 0.14: its demand sits at 1.0 through the gap, the retarder returns whole 0.3 s after the clutch closes and does 0.30 MJ of the wave). So a requester that can wait (brake_patience:
    // the speed it can owe) is not bridged at once: in a gap the truck OWES the braking (owed, m/s: the ask x the time), the retarder's demand is held at 1.0 for it (built up, it returns whole), and
    // the debt is repaid by the retarder's decel over the ask. Only a debt beyond the requester's budget is bridged by the service brake (from half the budget, continuously). A line (urgent) never waits.
    const float owe_max = BC.active && BC.who >= 0 && BC.urgency != BRU_URGENT ? BC.pat[BC.who] : 0.0f;
    const int wait_ok = owe_max > 0 && BC.need > 0;
    if (wait_ok && BC.gap) BC.owed = fminf(owe_max, BC.owed + BC.need * dt);
    else if (wait_ok) BC.owed = fmaxf(0.0f, BC.owed - fmaxf(0.0f, BC.a_ret - BC.need) * dt);
    else BC.owed = 0;
    const int pulse = wait_ok && (BC.gap || BC.owed > BC_OWE_EPS);
    int feed_ok = game_retarder_feed_ok() && bk_ret >= 0.5f;
    // (the retarder's share: all of it above BC_HAND_HI, handed to the service brake from there to the fade, so the brake has taken over
    // before the retarder fades: the fade passes in ~0.4 s at 1 m/s2, the brake's lag and the game's slew would leave a dip)
    float share = bc_clamp((kmh - BC_FADE_HI) / (BC_HAND_HI - BC_FADE_HI), 0, 1);
    BC.feed = feed_ok && BC.active && BC.need > 0 && BC.ret_full_nom > 0.05f ? bc_clamp(share * (pulse ? BC.ret_full_nom : BC.need) / BC.ret_full_nom, 0, 1) : 0;
    float d_eff = fmaxf(fmaxf(BC.feed, BC.game_d), BP.feed);
    BC.bl += (d_eff - BC.bl) * fminf(1.0f, dt / BC_RET_TAU);
    BC.a_ret = BC.bl * BC.ret_full;
    // the service brake: what is beyond the retarder's reach now, and its shortfall over the urgency's tolerance
    float tol_to = BC.urgency == BRU_URGENT ? 0.0f : (BC.env < 1 || share < 1) ? BC_TOL_GAP : BC_TOL[BC.urgency];
    BC.tol = tol_to < BC.tol ? tol_to : fminf(tol_to, BC.tol + 0.5f * dt);   // (down at once, back up gently: no step in the brake)
    float srv = 0, shortfall = 0, beyond = 0;
    if (BC.active && BC.need > 0) {
        // (in a shift gap the retarder is planned as gone at once: it falls within 0.2 s, about the service brake's own lag)
        // (a calm ask counts the retarder as built up: its 0.4 s build-up is no reason to apply the brakes; the fade's hand-over is)
        float a_ret_plan = BC.gap ? 0.0f : BC.urgency == BRU_COMFORT && share >= 1 ? fmaxf(BC.a_ret, d_eff * BC.ret_full - BC_TOL[BRU_COMFORT]) : BC.a_ret;
        shortfall = BC.need - a_ret_plan; beyond = BC.need - (BC.gap ? 0.0f : BC.ret_full);
        if (wait_ok && BC.gap) { const float bw = bc_clamp((BC.owed / owe_max - 0.5f) / 0.5f, 0, 1); shortfall = beyond = BC.need * bw; }
        srv = fmaxf(0.0f, fmaxf(beyond, shortfall - BC.tol));
    }
    // a soft dead band, not a switch: a calm or firm ask gives the service brake only what it needs beyond BC_SRV_DB (a whiff of brake is
    // not worth an application), and continuously so (a switch here was a knife edge: the junction sweep of sim_cruise stepped 0.6 km/h)
    float db = BC.urgency == BRU_URGENT || BC.gap || share < 1 ? 0.0f : BC_SRV_DB;
    BC.srv = fmaxf(0.0f, srv - db);
    BC.srv_on = BC.srv > 0.005f;
    int learned;
    float G = brake_gain(&learned);
    BC.req = chan ? bc_clamp(BC.srv / G, 0, 1) : 0;
    act_brake(ACT_CHAN, BC.req);   // (review 30-10: every tick, also 0 with the channel off: no request can stay behind)
    act_air_exempt(chan && BC.active && BC.urgency == BRU_URGENT && BC.req > 0);   // an urgent ask is served with what air is left (the low-air pause lets it through, like the AEB)
    // the term that decided the split and the stages that carry the event: the log line of the decision (promise P7) and its cost, one unit for all stages (BC_COST)
    {
        int term = BT_NONE;
        if (BC.active) {
            if (!(BC.need > 0)) term = BT_COAST;
            else if (BC.srv_on) term = BC.urgency == BRU_URGENT ? BT_URGENT : BC.gap ? BT_GAP : share < 1 ? BT_FADE : beyond >= shortfall - BC.tol ? BT_BEYOND : BT_SHORT;
            else term = BC.feed > 0 || BC.game_d > 0 ? BT_RETARDER : BT_COAST;
        }
        BC.term = term;
        BC.stages = BC.active ? (1 << BS_COAST) | (BE.own && BE.stage > 0 ? 1 << BS_EB : 0) | (BC.a_ret > 0.05f || BC.feed > 0 ? 1 << BS_RET : 0) | (BC.srv_on ? 1 << BS_SRV : 0) | (BC.down ? 1 << BS_DRAG : 0) : 0;
        BC.cost = BC.active ? BC.a_eb * BC_COST[BS_EB] + BC.a_ret * BC_COST[BS_RET] + BC.srv * BC_COST[BS_SRV] : 0.0f;
    }
    // the trim: closes the loop on the measured deceleration while we brake (not in a gap, not in the first moments, not at a
    // crawl); it winds down only while our own braking is part of it (the game's retarder may give more than asked: not ours to undo)
    int ours = BC.feed > BC.game_d + 0.02f || BC.req > 0;
    // (the trim closes the loop on the measured deceleration for every ask, also one asked on top of what the truck does by
    // itself: an ACC ask whose law was tuned open loop still needs the service brake to escalate when the measured deceleration
    // falls short of the reference, e.g. the game's own overspeed retarder delivering less than the model)
    if (BC.active && BC.on_s > BC_TRIM_FROM && v > 3 && !BC.gap && BC.env >= 1 && BC.meas_ok && fabsf(d_eff - BC.bl) < 0.1f) {   // (not while the retarder builds up)
        float e = BC.refd - BC.a_meas;
        if (e > 0 || ours) BC.trim = bc_clamp(BC.trim + BC_KI[BC.urgency] * e * dt, BC_TRIM_LO, BC_TRIM_HI);
    } else if (!BC.active) BC.trim *= expf(-dt / 5.0f);
    // more engine drag: every cheaper stage is full and the service brake is still needed for a while (not urgent, not in a gap). A lower gear puts the engine into its brake
    // range (more engine drag, and the retarder works through the ratio): the one stage between coasting and the engine-brake lever that costs nothing inside the bands. The
    // channel says how much it wants (the service share, in N at the wheels); the gearbox picks the gear (the bands up to the crossover, the landing, the road ahead: it weighs
    // whether the shift pays) and answers with a shift or not. The stages that are full: the retarder fed full and built (a truck with one), the engine-brake stages at the top
    // (a truck without one, when the control is ours; with it not ours the service brake is the next stage at once).
    const int cheaper_full = has_ret ? BC.feed >= 0.95f && BC.bl > 0.85f : (!BE.own || BE.stage >= BE.cap);
    BC.down_s = BC.active && BC.urgency != BRU_URGENT && !BC.gap && BC.srv_on && cheaper_full ? BC.down_s + dt : 0;
    BC.down = BC.down_s > BC_DOWN_S;
    BC.drag_n = chan && BC.down ? fmaxf(BC.srv, BC_DRAG_MIN) * m : 0.0f;
    {   // the standing request (core/gearreq.h): the drag wanted, the engine-brake stage held, the gear kept for a while after the stages came off
        // (eb_held is the descent rule's stage only: a stage the allocation takes for an event works in the gear of now, and the drag-gear request above is the one way an event asks for a lower gear)
        const gear_stand gs = { BC.drag_n, GSK_SUSTAINED, BE.own && BE.rule_stage > 0, BE.hold_until > now ? BE.hold_until : 0.0, BE.own && BE.alloc_stage > 0 };
        if (gs.drag_n > 0 || gs.eb_held || gs.hold_until > 0 || gs.eb_alloc) gear_stand_post(GSO_BRAKE, &gs); else gear_stand_clear(GSO_BRAKE);
    }
    // the retarder demand to the game (the pedal's blending is maxed in; the provider renews it too)
    if (feed_ok || BP.blend) {
        float f = fmaxf(BC.feed, BP.blend ? BP.feed : 0);
        if (f > 0 || BC.was_active) game_retarder_feed(f);
    }
    // the lights: the service brake, a firm ask, or any request of ours on +0x17c (the game lights none of them)
    if (chan) {
        float lit = BC.srv_on || (BC.active && BC.ref > bk_light) || act_request() > 0.02f;
        int keep = BC.lights && (now < BC.light_until || (BC.active && BC.ref > bk_light - 0.2f));
        if (lit) BC.light_until = now + BC_LIGHT_HOLD;
        int on = lit || keep;
        if (game_brake_light_ok() && (on || BC.lights)) game_brake_light_request(on);
        BC.lights = on;
    }
    if (BC.active) { BC.lvl_srv_max = fmaxf(BC.lvl_srv_max, BC.srv); BC.lvl_ret_max = fmaxf(BC.lvl_ret_max, BC.a_ret); }
    g_brake_fed = fmaxf(BC.feed, BP.blend ? BP.feed : 0);
    g_brake_pedal_mapped = BP.ready;
    bc_learn(dt, v, m, BC.game_d);
    bc_learn_lever(dt, v, m);
    bc_pedal_latch(kmh, m);
    be_measure(a, now, dt);
    BC.was_active = BC.active;
    // What the stages may carry of the events now, for the next tick's allocation: the largest of the channel's need (before the credit), the lever assist's wish (renewed every tick)
    // and the brake pedal's wish (the game's own automatic engine brake does that one itself when it is on).
    {
        // (the reference less coasting, WITHOUT the trim: the trim is the loop's correction for a stage that gives more or less than counted, and a stage that switched off when the trim dipped
        // would bring the surplus back and switch on again a second later)
        float x = chan && BC.active ? fmaxf(BC.ref - BC.nat, 0.0f) : 0.0f;
        if (now - BE.wish_at < BC_EB_WISH_S && BE.wish > x) x = BE.wish;
        BE.alloc_learned = 0;
        if (!BP.auto_eb && BP.ready && BP.calls <= BP_STALE_CALLS && BP.raw > 0 && BP.want == BP.want && BP.want > x) { x = BP.want; BE.alloc_learned = 1; }
        BE.alloc_ms2 = x;
    }
    // Who holds the retarder while the driver is on the gas (ride 02-11 21:24: on 5 with 92-99 % gas, no source of ours was found): after half a second a line names the holder,
    // once per time on the gas. The sources of a retarder demand: our feed (the channel's, the pedal's blend), the game's own overspeed braking, the driver's lever or the game's
    // automatic retarder (the telemetry shows the level, not whose it is: when none of ours is feeding it, it is theirs).
    if (a->pedal > 0.1f && (T.retarder > 0 || BC.bl > 0.05f)) BC.gas_ret_s += dt; else { BC.gas_ret_s = 0; BC.gas_ret_told = 0; }
    if (BC.gas_ret_s > 0.5f && !BC.gas_ret_told) {
        BC.gas_ret_told = 1;
        const char *holder = BC.feed > 0.02f ? "our feed (the brake channel)" : BP.feed > 0.02f ? "our feed (the brake pedal's blend)" : BC.game_d > 0.02f ? "the game's own overspeed braking" :
                             "none of ours: the driver's retarder lever or the game's automatic retarder";
        logline("Brake: the retarder is at level %u of %u (built up %.2f) with the gas at %.0f %%: held by %s (our feed %.2f, the pedal's blend %.2f, the game's overspeed demand %.2f, our torque factor %.2f)",
                (unsigned)T.retarder, (unsigned)T.retarder_steps, BC.bl, a->pedal * 100, holder, BC.feed, BP.feed, BC.game_d, (double)g_ret_scale);
    }
}

// ---------------------------------------------------------------- frame side: heartbeat, log, trace
static void bc_trace()
{
    static double last;
    double now = sim_s();
    if (!(bk_trace > 0) || g_log_level < 2 || now - last < 0.1) return;
    if (!BC.active && !(BP.raw > 0) && !(BC.bl > 0.01f)) return;
    last = now;
    if (!s_csv) {
        s_csv = open_out("brake.csv", "a");
        if (!s_csv) return;
        fputs("t,kmh,who,urgency,want,ref,nat,trim,need,ret_full,game_demand,feed,pedal_feed,ret_built,ret_decel,service,request,gain,measured,lights,down,pedal_raw,pedal_out,pedal_want,drag_n,eb_stage,eb_decel,term,stages,cost\n", s_csv);
    }
    fprintf(s_csv, "%.2f,%.1f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.0f,%d,%.3f,%d,%d,%.3f\n",
            now, fabsf(T.speed) * 3.6f, BC.active ? BC.who : -1, BC.urgency, BC.want, BC.ref, BC.nat, BC.trim, BC.need, BC.ret_full, BC.game_d, BC.feed,
            BP.feed, BC.bl, BC.a_ret, BC.srv, BC.req, BP.G, BC.a_meas, BC.lights, BC.down, BP.raw, BP.out, BP.want == BP.want ? BP.want : 0.0f,
            BC.drag_n, BE.own ? BE.stage : 0, BC.a_eb, BC.term, BC.stages, BC.cost);
    csv_cap(&s_csv, "brake.csv");
}

static void bc_frame()
{
    if (!BC.running) return;
    if (g_brake_pedal >= 0.5f) game_brake_pedal_heartbeat();
    bc_log_event(fabsf(T.speed) * 3.6f);
    bc_trace();
}

void brake_view_get(brake_view *o)
{
    int learned = 0;
    o->on = brake_channel_on(); o->active = BC.active; o->who = BC.active ? BC.who : -1; o->urgency = BC.urgency;
    o->want = BC.want; o->ref = BC.ref; o->nat = BC.nat; o->trim = BC.trim; o->need = BC.need; o->ret_full = BC.ret_full;
    o->ret_force = vehicle_ret_force_full(); o->game_demand = BC.game_d; o->feed = BC.feed; o->pedal_feed = BP.feed;
    o->ret_built = BC.bl; o->ret_decel = BC.a_ret; o->service = BC.srv; o->request = BC.req; o->gain = brake_gain(&learned);
    o->gain_learned = learned; o->measured = BC.a_meas; o->lights = BC.lights; o->down = BC.down;
    o->pedal = BP.ready; o->pedal_raw = BP.raw; o->pedal_out = BP.out; o->pedal_want = BP.want; o->pedal_top = BP.top;
    o->drag_n = BC.drag_n; o->stages = BC.stages; o->term = BC.term; o->cost = BC.cost;
    o->eb_stage = BE.own ? BE.stage : 0; o->eb_cap = BE.cap; o->eb_decel = BC.a_eb;
    o->eb_alloc = BE.alloc_stage; o->eb_credit = BC.eb_credit; o->eb_asked = BE.alloc_ms2;
}

static void bc_clear(int all)
{
    for (int i = 0; i < BRQ_N; i++) { BC.ask[i] = 0; BC.urg[i] = 0; BC.add[i] = 0; }
    BC.open = 0;
    BC.want = 0; BC.who = -1; BC.active = BC.was_active = 0; BC.ref = BC.refd = 0; BC.need = 0; BC.on_s = 0;
    BC.owed = 0; BC.feed = 0; BC.bl = 0; BC.a_ret = 0; BC.srv = 0; BC.req = 0; BC.srv_on = 0; BC.down_s = 0; BC.down = 0; BC.gap = 0; BC.env = 1;
    BC.hn = 0; BC.meas_ok = 0; BC.learn_s = 0; BC.lights = 0; BC.light_until = 0; BC.tol = 0;
    BC.trim = 0;   // (review 30-10: also on a soft / load reset: a stale trim from another load or trailer would brake up to ~1 m/s2 weaker)
    BP.feed = 0; BP.ready = 0; BP.calls = 0;
    g_brake_fed = 0; g_brake_pedal_mapped = 0;
    BC.drag_n = 0; BC.a_eb = 0; BC.eb_credit = 0; BC.need0 = 0; BP.eb = 0; BC.term = BT_NONE; BC.stages = 0; BC.cost = 0; BC.log_key = -1;
    if (BC.running) { act_brake(ACT_CHAN, 0); act_air_exempt(0); }   // (the standing request is not cleared here: the slot has ONE writer, the tick, which clears or posts it from this zeroed state at the next tick; the reset runs on the frame thread)
}

// The tick hook in (also for the sims, which have no game to hook and call it after Cruise's start: the hooks run in order).
void brake_attach()
{
    if (BC.running) return;
    BC.running = 1; BC.log_who = BC.log_key = -1; BC.game_from_kmh = NAN; BC.env = 1;
    act_hook(ACT_POST, bc_tick);
    if (g_brake_pedal >= 0.5f) game_brake_pedal_set_provider(brake_pedal_map);
}

static char s_status[240] = "off (ini)";
static const char *bc_status() { return s_status; }

static void bc_start()
{
    bk_plin = bc_clamp(bk_plin, 2, 20);
    if (!(g_brake_channel >= 0.5f) && !(g_brake_pedal >= 0.5f)) return;   // nothing registered: the actuator runs as before
    if (!act_hooked()) { snprintf(s_status, sizeof s_status, "off: no actuator hook"); return; }
    void *ra = game_cvar("g_retarder_auto");
    float rav = ra ? game_cvar_get(ra) : NAN;
    BP.auto_ret = rav == rav && rav > 0;
    void *ea = game_cvar("g_motor_brake_auto");   // the game's own "automatic engine brake" (the pedal engages the engine brake itself): the pedal's engine-brake stage is the game's then
    float eav = ea ? game_cvar_get(ea) : NAN;
    BP.auto_eb = eav == eav && eav > 0;
    brake_attach();
    snprintf(s_status, sizeof s_status, "channel %s (retarder feed %s, lights %s), pedal %s (hook %s, linear to %.1f m/s2, end blend ^%.0f%s)%s",
             g_brake_channel >= 0.5f ? "on" : "off", game_retarder_feed_ok() ? "in" : "not in", game_brake_light_ok() ? "in" : "not in",
             g_brake_pedal >= 0.5f ? "on" : "off", game_brake_pedal_ok() ? "in" : "not in", bk_ptop, bk_plin,
             bk_pblend >= 0.5f ? ", retarder first" : "", BP.auto_ret ? "; the game's auto retarder follows the pedal (g_retarder_auto)" : "");
}

static void bc_stop()
{
    if (!BC.running) return;
    game_brake_pedal_set_provider(NULL);
    game_retarder_feed(0);
    if (game_brake_light_ok()) game_brake_light_request(0);
    bc_clear(0);
    gear_stand_clear(GSO_BRAKE);   // (the end of the session: the tick is not running any more, nothing else writes the slot)
    if (BE.n_ep || BE.own) logline("Brake: engine brake stages: %u runs, %u steps, handed back to the driver %u times%s", BE.n_ep, BE.n_steps, BE.n_driver, BE.own ? " (the game ended while it held the brake)" : "");
    if (s_csv) { fclose(s_csv); s_csv = NULL; }
}

// Continuity broke: no ask, no reference, no built-up retarder carries over. The retarder's learned force is a force: it survives
// a new load, not another truck.
static void bc_reset(int level, const char *)
{
    bc_clear(level >= RESET_HARD);
    if (BE.own) BE.release_req = 1;   // the stages we hold go back to the driver at the next tick (not from the reset's thread)
    BE.stage = 0; BE.d.stage = 0; BE.hold_until = 0; BE.alloc_stage = 0; BE.rule_stage = 0; BE.alloc_ms2 = 0; BE.wish = 0; BE.alloc_at = 0; BE.up_flag = 0; BE.steady = 0; ST.armed = 0; ST.ema_ok = 0;
    if (BC.running) { game_retarder_feed(0); if (game_brake_light_ok()) game_brake_light_request(0); }
}

module m_brake = { "brake", "Brake", g_params, sizeof g_params / sizeof g_params[0], bc_start, bc_frame, NULL, bc_stop, bc_reset, bc_status, NULL, VCM_BOTH };
