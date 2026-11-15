// Decel: a closed-loop deceleration assist for the retarder and engine-brake lever. It adds service brake; it lowers what the
// game's own retarder does only with soften (below) and the retarder hook of [core] retarder_hook.
//
// While the driver holds a lever step (T.retarder, T.motor_brake) and both pedals are free, the assist aims at the target
// deceleration of that step and measures the real one (the speed's derivative, smoothed over 0.3 s). The retarder comes first
// and is already on; whatever is missing the service brake adds through the actuator (ACT_DECEL). No brake calibration: the
// loop closes on the measured deceleration, and what it learns (request per m/s2) is in the request's own unit.
// Three things it is for (brake-measure notes): the gearbox shift (clutch open ~1.1 s: the retarder falls to a floor within
// ~0.2 s and is back ~0.1-0.4 s after the clutch closes), the last metres (the retarder fades to nothing below ~6 km/h, the
// truck then coasts), and a heavy load (the same step gives less deceleration, F/m).
//
// Structure (decel_tick): measure, gate, target of the step, jerk-limited reference, then a PI on the request with a feedforward
// at the shift: when the clutch opens, the level that held the target in the last gap (learned: request per m/s2 of the gap) is
// asked at once, and when the clutch closes the level goes back at once to what it was before the gap. At once both times: the
// retarder falls within 0.2 s and recovers within 0.3 s, the service brake follows a request with a lag of ~0.15 s, so a ramp
// would add its own length to the brake's lag (a dip at the start, an overshoot at the end); and the close is taken when it
// happens, not predicted (the gap's length varies by +-0.1 s, the retarder's recovery is 8 m/s3 at the top step). Once the gap
// has settled, one correction step for what the feedforward missed; it also measures the brake's gain (see decel_tick).
// Stop assist: the reference tapers with speed along a = sqrt(2 j v), the deceleration that a ramp of jerk j brings to 0 exactly
// at v = 0 (j from jerk_limit, so no km/h figure is needed), led by the brake's lag; at a standstill the assist stops posting,
// unless Auto Hold takes the stop over ([cruise] auto_hold_lever): then its request stays until the hold's has risen to it.
// Other brake owners (AEB, Auto Hold, hill start) keep their priority: the actuator merges by the biggest wish.
// Soften: the other side of the target. A light truck gets far more than the target from the step alone (empty, the first step
// can give several times it), and a brake cannot take that away. When the measured deceleration is over the target by more than
// a tolerance and none of our brake is on, a factor on the game's retarder torque (act_retarder_scale, ACT_RET_DECEL) comes down,
// an integral on the error over what the retarder gives at 1, rate-limited, with a dead band so the noise does not move it; it
// goes back up as soon as the deceleration is short, and while it is under 1 the brake does not add (the retarder comes first).
// In a gap (without the ramp, below) the factor goes back to 1 as the retarder falls (the gap's feedforward is made for the
// game's own floor) and down to the factor of before as the retarder recovers, both ramped so the two together do not make a
// peak. Back to 1 at once when the assist ends, rate-limited when the lever is released. Only the player's truck gets it
// (core/actuate.cpp).
// Retarder curve (fade, WP-BR3b): the game's retarder does not fade with the road speed (it follows the engine side through the ratio), so a lever step brakes as hard at 10 km/h as at
// 60 and the service brake only tops it up: a brake assist, not a retarder. A retarder as built goes with the square of its shaft speed up to its full force: the step's target is
// multiplied by s(v / knee) (core/brakemodel.h bm_ret_shape, knee = the engine's peak power over the top stage's force, vehicle_ret_knee_ms), a smooth function of the road speed alone, and
// soften's factor takes the game's retarder down to it. Nothing tops the fade up: the service brake fills the shifts' gaps to the target of that moment and what the driver or an automatic ask
// wants beyond it. The last metres, where the curve and the retarder are as good as gone, are finished at the stop assist's gentle level along the same taper. It needs the factor on the game's
// retarder (soften, the hook); an engine brake alone has no retarder to fade.
// Ramp: the game may bring its retarder back at full strength the moment the clutch closes (the lower gear's engine brake with
// it) while the brake still holds the gap's level, and a factor could follow the close only a frame later: the deceleration
// jumps. With ramp_s and the hook (and a retarder step held) the retarder's comeback is ours: in a gap the factor goes to 0 over
// the retarder's fall time, the brake's lag after the open (the feedforward counts no floor: the brake holds the whole target
// there), and from the close the level goes linearly from the gap's to the one before it while the factor climbs from 0 to its
// own (soften's, or 1) as the brake model gives up the level, so the two sum to the target on the way. The build-up is
// jerk-limited: it lasts as long as the deceleration it hands over (the level's change over the brake's gain) needs at
// jerk_limit, so a small difference is done almost at once and a large one takes longer; ramp_s is only the longest it may
// take. Without the hook all of this is off (one log line) and the close is as above.
// Step ramp (step_ramp): the game's retarder takes the torque of a new lever step at once, a jump of the deceleration that the reference
// (which rises at the jerk limit) cannot follow. With the hook, at a step up or a pull the retarder's factor is first set to what holds
// the deceleration it gives now (the step's torque at factor 1 is the old torque times the step ratio, or the nominal curve at a pull),
// then let up so that the retarder's deceleration grows at the jerk limit, as fast as the reference, up to what the factor would be
// anyway, and with soften not past what the reference leaves the retarder (the brake's share taken off). So the retarder and the
// brake sum to the reference all the way. At a step down the retarder falls at once and the brake bridges what it lost (a part, taken
// at once, while the brake is not already carrying the lever's share), then gives it up with the reference. One frame of the old factor
// on the new step is unavoidable: the lever is seen a frame after the game has used it. Without the hook only the bridge works.
// Engine brake bridge (engine_brake_bridge, 01-11): the engine brake does nothing while the clutch is open, so at a shift the deceleration of
// a stage falls away (the retarder has its own bridge above, with a target per step). A truck WITHOUT a retarder, the engine brake on (the
// lever's stage, or our descent control's under the cruise), both pedals free and nobody else braking: the deceleration the truck had just
// before the clutch opened is the reference (a smoothed measurement, whatever its sign: on a descent holding its speed it is 0 and the
// shift must not make it speed up), and the service brake adds what the measured one falls short of while the clutch is open. At the open
// the level the same stage needed in the last gap goes in at once (kept per stage, as a force, so it holds at another load; the first gap
// asks a guess from the stage's share of the engine's stages), a PI on the measured error does the rest; when the clutch closes the
// level goes down at the jerk limit (the engine brake is back within 0.1-0.4 s, a lower gear brakes more), faster while the truck brakes
// over the reference. It posts as the lever assist's own owner (ACT_DECEL, the bigger of the two) and keeps soften off while it brakes.
// Comfort (comfort_learn): the cab's pitch is watched after every lever event, logged, and the jerk limit of the truck learned from it
// (section Comfort below).
// Gear ratio: the retarder and the engine brake work through the gearbox, so after a downshift the same step brakes harder at the
// wheels. The level the brake goes back to at the close is the one that holds the target with the retarder's share of before times the
// change of the ratio (vehicle_rpm_at of the two gears), and when that share alone is over the target soften's factor starts lower.
// Pre-blend (pre_blend): TrueShift can tell when its next downshift in braking is due (ts_brake_down_eta: the speed falling to the
// point where its braking rules judge the gear, at the deceleration of now). Once that is nearer than the hand-over needs (the
// retarder's share over the jerk limit, the brake's lag and a margin), the share goes from the retarder to the service brake at the
// jerk limit, the factor following the brake model, so the clutch opens on a brake that already holds the target. It unwinds the same
// way when the shift does not come, and is given up after a while when it is ready and nothing happens. Without the hook (or with
// ramp_s 0) only the brake rises ahead of the shift, by a small amount (the retarder stays: more would overshoot).
// Downshift assist (downshift_assist): the order is the retarder, then a lower gear, then the service brake. While the brake is needed (its level outside
// the gaps over DC_NEED, 1.5 s after the lever was pulled, when the retarder has built up) a standing request to the gear arbiter (core/gearreq.h) tells TrueShift, which then
// takes a lower gear that puts the engine into its own brake range (modules/trueshift.cpp, assist_down) earlier than its own rules do.
// The lever's steps (step_force 2, WP-0911-K4, owner's decision K4 of 09-11, after the ride of 08-11 where a step as the game's own retarder times a fade was "super soft": step 1 of 5 braked 0.13 m/s2 at
// 55 km/h, 39.5 t): a step is a share of what the truck can brake NOW. The target of the loop is the BRAKES' part (the measured deceleration less the road's own, as step_force 1 has it) and is made of
//   - the retarder's force of the step (the brake model's learned stage force) at the road speed, through that stage's own curve (core/brakemodel.h bm_ret_shape_stage: a stage fades from the speed where its own
//     force is on the retarder's n^2 line, so a low stage fades later), over the mass;
//   - the engine brake's top stage in the gear and revs of now (vehicle_eb_decel_at), times the share k/n of the lever (the game adds its engine brake to the retarder lever by itself; the engine-brake lever is its
//     own stage k of its stages);
//   - the hold on top, for every step: step 1 is max(its rung, what holds the speed of the pull), so the extra the hold needs over the rung is added to every step (the steps keep their distance, a descent steeper than
//     the rung moves the whole ladder up: 'never faster, always a little slower'). The hold is the road's push (the model's natural deceleration, if it pushes) plus the speed the truck has gained over the slowest it has
//     been since the pull, removed in three response times of the measurement path (the window and the brake's lag).
// The loop is the one of step_force 0 (the retarder first and lowered by soften where the step gives more than the target, the engine brake by the brake channel's stage, the service brake for the rest, the bridge in the
// shifts), on the brakes' part. The last metres are finished as ever (the taper to a standstill, the stop assist's gentle level where the retarder is gone). The converter box (V.tc) is not bridged, as with step_force 1.
#include "../core/suite.h"
#include "../core/sense.h"
#include "../core/vehicle.h"
#include "../core/brakemodel.h"
#include "../core/rpmspan.h"
#include "../core/gearreq.h"
#include "../core/brake.h"
#include <math.h>
#include <string.h>

#define DECEL_STEP_FORCE 1   // (the sims check for it: a force per step is in this build)
static float p_force, p_fade, p_ebb, p_enabled, p_tmin, p_tmax, p_tcurve, p_jerk, p_max, p_soften, p_ramp, p_pre, p_dsa, p_sa, p_sad, p_sramp, p_cl, p_ct;

static param g_params[] = {
    { "enabled", &p_enabled, 0, "1 = while a retarder or engine-brake step is held and both pedals are free, the service brake adds what is missing to reach that step's target deceleration (bridges the gearbox shifts, helps at low speed and with a heavy load). 0 = off" },
    { "engine_brake_bridge", &p_ebb, 1, "1 = on a truck without a retarder the service brake bridges the gearbox shifts for the engine brake as well, with the lever or with the descent control under the cruise: the engine brake does nothing while the clutch is open, so the service brake adds what the deceleration falls short of compared with just before the shift, and lets go as the clutch closes. Needs enabled = 1. 0 = the engine brake falls away in a shift" },
    { "step_force", &p_force, 2, "What a lever step is. 2 (the default, WP-0911-K4) = a share of what the truck can brake now: step k of n is k/n of the retarder's force at the road speed (a low stage fades later than the top stage: the retarder's controller fills it to the force the stage asks) plus k/n of the engine brake's in the gear and revs of now, the top step is all of it. Step 1 never lets the truck get faster than at the pull (on a descent it brakes at least what holds that speed, the steps above it add their share on top) and always brakes a little (the lowest rung of that ladder). The same for 3, 4 or 5 steps and for the engine-brake lever (stage k of its stages). The retarder first, then the engine brake (the brake channel's stage), then the service brake for what is missing; the retarder is lowered where the step alone gives more. 1 = a lever step is the force of the game's own retarder at that step, nothing more and nothing less: our retarder factor is the fade of retarder_fade alone (open loop, 1 at speed, falling toward a standstill), no deceleration is aimed at and the service brake does not top the step up; it only bridges a gearbox shift. 0 = every step aims at a whole deceleration (target_min .. target_max, spread by target_curve) whatever the road does, the grade included. Where the mass is not known (for the road's own deceleration) 2 and 1 fall back to 0. A truck without a retarder and an engine brake whose stage count is not known keeps the target deceleration" },
    { "retarder_fade", &p_fade, 1, "0..1, how far a lever step follows the fade of a real retarder. A retarder's torque goes with the square of its shaft speed up to its full force (the game's retarder follows the engine side instead and brakes as hard at 10 km/h as at 60), so below a knee that comes from the truck (the engine's peak power over the retarder's top-stage force) a real retarder gives less and less. 1 = the step's target falls with the road speed along that curve, smoothly, and the game's retarder is lowered to it; 0.5 = half way between the curve and the step's whole target; 0 = the step's target is held all the way down, as the game's retarder gives it. Not used by step_force 2, where the fade comes from the stage's own curve (a low stage fades later than the top one). The service brake does not make the fade up, it only fills the shifts' gaps to the target of that moment, and the last metres of the stop are finished at the level of stop_assist_decel. Needs the retarder hook ([core] retarder_hook = 1) and soften" },
    { "target_min", &p_tmin, 0.8f,"target deceleration (m/s2) at the first retarder step. Only with step_force 0 (and where step_force 1 or 2 cannot be used): the default step_force 2 derives the steps from what the truck can brake" },
    { "target_max", &p_tmax, 3.0f, "target deceleration (m/s2) at the last retarder step. Only with step_force 0 (see target_min)" },
    { "target_curve", &p_tcurve, 1.0f, "how the steps between the first and the last are spread: 1 = evenly, below 1 = the lower steps get more. Only with step_force 0 (see target_min)" },
    { "jerk_limit", &p_jerk, 2.0f, "most the deceleration may change per second (m/s3), rising and falling. A smaller value is gentler on the cab (less pitching) and slower to reach a new step's target. With comfort_learn this is where a truck starts; its own value (1.5-4) is learned from the pitch it measures" },
    { "step_ramp", &p_sramp, 1, "1 = when the lever moves up a step or is pulled, the game's retarder builds up to the new step at the jerk limit instead of jumping to it, and when the lever moves down a step the service brake bridges the drop for a moment. The build-up needs [core] retarder_hook = 1 (the bridging does not). 0 = the retarder jumps to the new step as the game has it" },
    { "comfort_target", &p_ct, 0.10f, "pitch swing (degrees per m/s2 of a lever step, so 0.1 means 0.2 degrees for a step of 2 m/s2) over which comfort_learn brings the jerk limit down, and under half of which it takes it up. Lower = gentler, slower to reach a new step" },
    { "comfort_learn", &p_cl, 1, "1 = after each lever pull, step, release and gearbox shift the cab's pitch is measured for 3 s (the pitch swing beyond its settled value, one log line); after a pull or a step up, over a limit the jerk limit of this truck comes down a little, well under it goes up a little, always within 1.5-4 m/s3, and the value is kept per truck (comfort_learn.txt). Nothing is learned during emergency or hard braking, a pedal press or unreliable data. 0 = the jerk limit stays as set" },
    { "max_brake", &p_max, 0.6f, "most brake request (0-1) the assist may ask for" },
    { "soften", &p_soften, 1, "1 = when the step alone brakes harder than its target (a light truck), the game's retarder is made weaker until the target holds; full again when the lever is released. Needs [core] retarder_hook = 1. 0 = the retarder stays as the game has it" },
    { "ramp_s", &p_ramp, 1.0f, "most seconds the game's retarder takes to build back up after a gearbox shift. The service brake hands over at the rate the jerk limit allows (a small difference takes a fraction of a second, a large one longer, never more than this), so the deceleration does not jump when the clutch closes; in the shift the brake alone holds the target. Needs [core] retarder_hook = 1. 0 = the retarder comes back as the game has it" },
    { "pre_blend", &p_pre, 1, "1 = when the gearbox is about to shift down while braking, the retarder's share is handed to the service brake before the clutch opens, at the jerk limit, so the deceleration holds through the shift. The retarder's side needs [core] retarder_hook = 1 and ramp_s above 0; without it only the service brake rises ahead of the shift, by a small amount. 0 = the brake takes over when the clutch opens" },
    { "downshift_assist", &p_dsa, 1, "1 = while the service brake is needed to reach the target, the gearbox shifts down earlier when a lower gear puts the engine into its own engine-brake rpm range (never near the top of the range or over the rpm limit): the retarder and the engine brake work through the lower gear before the service brake does. 0 = the gear is chosen as without the assist" },
    { "stop_assist", &p_sa, 1, "What happens when you lift off at low speed and the gearbox opens the clutch by itself (no throttle, no brake, no retarder or engine-brake lever, no cruise): 0 = the truck rolls on in neutral, as in the game. 1 = the truck rolls out on its own resistance: on the flat or uphill the service brake does nothing (uphill it stops sooner), on a descent it gives only what keeps the speed from rising, and it lets go at the standstill. 2 = the same, and Auto Hold holds the truck at the standstill (needs Auto Hold and its lever setting; uphill it keeps the truck from rolling back). The throttle or the brake pedal ends it at once. Works with the deceleration assist on or off" },
    { "stop_assist_decel", &p_sad, 0.5f, "deceleration (m/s2) the last metres of a retarder or engine-brake lever stop are finished at (it eases off along the way to the standstill). The stop assist of a lift-off no longer brakes by it: it rolls out (stop_assist). Keep it under 0.7, the level below which a car's regenerative braking does not light the brake lights" },
};

// Constants of the controller (physics of the measurements, not preferences).
static const float DC_GAIN_LO = 2.0f, DC_GAIN_HI = BM_SVC_GAIN_HI;   // m/s2 per request: the range of the brake's gain the loop works with (the vehicle model's estimate is clamped into it; the upper edge is the model's own: it was 12 here, under what the brakes of ride 8 measured, 14-16)
static const float DC_FLOOR = 0.4f;      // m/s2 the truck still decelerates with while the clutch is open (measured 0.3-0.65)
static const float DC_WIN = 0.3f;        // s, the measured deceleration is the speed's fall over this window (delay: half of it)
static const float DC_TOL = 0.25f;       // m/s2 over the reference where the overshoot guard backs off at twice the rate
static const float DC_FF_S = 0.2f;       // s, the retarder's fall at the start of a shift (the measurement shows it this long)
static const float DC_REL_S = 0.3f;      // s, the retarder's recovery after the clutch closes (the measurement shows it this long)
static const float DC_LAG_S = 0.15f;     // s, the service brake's lag behind the request
static const float DC_GAIN_SHARE = 0.6f;   // the share of the model's start value the loop counts on until the brake is measured: 6 m/s2 per request was its own nominal where the fit says 10 (tuned in sim_decel, which plants x0.6 / x1.0 / x1.5 of 6); a lever assist that does not know the brake asks softly (an undershoot ends in the PI, an overshoot is the hard braking the owner felt), and the model's weight takes it to the estimate itself
static const float DC_FF_FIRST = 0.75f;  // share of the feedforward at the first shift while the gain is only the model's start value (no slot confident, no gap of this event has taught it)
static const float DC_FF_PRIOR = 0.9f;   // the same when the gain is the vehicle model's learned one (a good estimate, not a measurement of this event: a quarter off would overshoot at the full level)
static const float DC_REFM_S = 0.3f;     // s, the delay of the measurement path (window + brake lag): the reference is compared with the measurement through the same delay
static float DC_TAPER_J = 0.5f;    // the taper to a standstill uses this share of the jerk limit: the brake's lag and the measurement's delay eat the rest
static const float DC_BLIND_S = 0.35f;   // s, the measurement (window delay, brake lag) needs this long to show what a change of the level did
static const float DC_FIX_E = 0.15f;     // m/s2 of error in a settled gap worth the gap's correction step
static const float DC_KP = 0.15f;        // request per m/s2 of error
static const float DC_KI = 0.30f;        // request per m/s2 of error per second
static const float DC_STOP_V = 0.08f;    // m/s, standing (0.3 km/h, as LowSpeed): the assist stops posting
static const float DC_PEDAL = 0.02f;     // a pedal above this ends the assist
static const float DC_OPEN = 0.05f;      // clutch above this is open (a shift), under DC_CLOSED closed again
static const float DC_CLOSED = 0.02f;
static const float DC_BAD_S = 0.5f;      // s without a usable speed derivative: the assist lets go
static const float DC_HAND_S = 0.6f;     // s at most the request waits at a standstill for Auto Hold's to rise (its ramp to 0.6 takes 0.2 s)
static const float DC_SOFT_TOL = 0.15f;  // m/s2 over the target before the retarder is lowered
static const float DC_SOFT_DB = 0.08f;   // m/s2 dead band around the target where the factor holds (the measurement's noise is ~0.03)
static const float DC_SOFT_KI = 2.0f;    // per second: share of the error (over the retarder's deceleration at 1) the factor corrects
static const float DC_SOFT_DN = 0.5f;    // factor per second, falling
static const float DC_SOFT_UP = 1.0f;    // factor per second, rising
static const float DC_SOFT_MIN = 0.2f;   // the lowest factor (soften tames a light truck's first step); with the retarder's curve (WP-BR3b) it is DC_SOFT_ABS (D.kmin)
static const float DC_SOFT_ABS = 0.02f;  // the lowest factor of all: under 2 % of a stage's 3 m/s2 (0.06 m/s2) is the deceleration measurement's noise (0.03-0.05); it keeps am / D.kr finite at a standstill
static const float DC_SOFT_U = 0.01f;    // our request under this counts as none (the brake backs off before the retarder is lowered)
static const float DC_SOFT_GAP_UP = 0.4f;   // s, the factor's way back to 1 in a gap (twice the retarder's fall: the two together dip, not peak)
static const float DC_SOFT_GAP_DN = 0.2f;   // s, its way back down after the clutch closes (a little under the retarder's recovery)
static const float DC_PRE_MARGIN = 0.2f; // s, the hand-over before a shift is done this long ahead of the prediction (its error)
static const float DC_PRE_HYS = 0.3f;    // s, a hand-over under way holds while the shift is expected up to this much later than it started
static const float DC_PRE_OVS = 0.2f;    // m/s2, without the retarder hook the brake is raised ahead of a shift by this much at most (the retarder stays)
static const float DC_PRE_MIN = 0.1f;    // m/s2, a smaller share of the retarder is not worth preparing
static const float DC_PRE_HOLD_S = 1.5f; // s, a hand-over complete this long and no shift: given up (until the prediction clears)
static const float DC_NEED = 0.05f;      // request, the service brake that counts as needed for downshift_assist
static const float DC_NEED_S = 1.5f;     // s, the lever has been held this long (the retarder has built up) before the need counts
static const float DC_RAMP_TAIL = 0.6f;  // s, a build-up ends this long after its level did at the latest (four lags of the brake)
static const float DC_BRIDGE_U = 0.05f; // request, the bridge at a step down is made only while our brake is under this (the retarder carries the target)
static const float DC_SR_MARGIN = 0.05f; // m/s2, the step ramp's build-up may be this far over what the reference leaves the retarder (the estimate of the step's torque is not exact)
static const float DC_STOP_PAD = 0.4f;   // s over a shift's length the clutch must stay open before the gap is the box's own neutral at the end of a stop (the stop assist's SA_SHIFT_PAD)
static const float DC_HOLD_S = 3.0f * (DC_WIN + DC_LAG_S);   // s, the speed the truck has gained over the pull's is taken off in three response times of the measurement path (the window and the brake's lag): a loop cannot act faster than it sees
static const float DC_RAMP_MIN = 0.05f;  // s, the shortest build-up after a shift (a few frames: a hand-over of nothing is done at once)

// The term that holds Decel's request (promise P7: every decision explains itself): one line when it changes, with the figures the term works on. The request is the biggest of
// the lever assist's loop, the engine-brake bridge and the stop assist; within the loop the term is the phase it is in.
enum { DT_NONE, DT_TRACK, DT_GAP, DT_CLOSE, DT_PREBLEND, DT_STEP, DT_EB_BRIDGE, DT_STOP, DT_HAND, DT_N };
static const char *const DT_TEXT[DT_N] = { "none", "the loop tracks the step's reference", "a shift gap: the brake holds the target alone", "the clutch closed: the retarder comes back, the brake gives its level up",
                                           "a hand-over ahead of a shift: the retarder's share goes to the brake", "a step down of the lever: the brake bridges the retarder's drop",
                                           "the engine-brake bridge: the stage falls away in the shift", "the stop assist tapers the stop to a standstill", "handed to Auto Hold at the standstill" };
static const float DC_STEP_S = 0.6f;     // s the bridge of a step down is named as the term (the level it added comes off with the reference within about this)

// Comfort (see the section below): the jerk limit of this truck. The limits and steps are variables only so that the sim can sweep them.
static float CL_LO = 1.5f, CL_HI = 4.0f;       // m/s3, the range the learned jerk limit stays in
static float CL_DOWN = 0.15f, CL_UP = 0.05f;   // m/s3: the step down (scaled by how far over the target, 0.2-2 times), the step up (well under the target, a step of 0.5 m/s2 or more)
static float CL_UP_FRAC = 0.5f;                // "well under" = under this share of the target
static float DC_BRIDGE = 0.8f;                 // share of the retarder's drop the brake bridges at a step down
static struct { float j; } CL;                 // the learned jerk limit of this truck (0 = not set yet: the ini value)
static float jerk_now()
{
    if (p_cl > 0) {
        if (!(CL.j > 0)) CL.j = fminf(fmaxf(p_jerk, CL_LO), CL_HI);
        return fminf(fmaxf(CL.j, CL_LO), CL_HI);
    }
    return fmaxf(p_jerk, 0.1f);
}

// What one tick sees (the sim sets the same fields).
struct decel_in {
    float dt, speed, thr, brake, clutch;
    int gear, retarder, steps, motor_brake, engine_on, paused, cruise;
    float other;       // the brake wish of the cruise side (ACC, bends): it owns the brake then
    float held;        // the request the actuator holds now (brake air low: none of ours)
    float hold, take;  // Auto Hold: the request it holds now, and what it will hold after a stop by the lever (0 = it will not take one)
    int soften;        // the retarder factor may be posted (soften on, the hook in)
    int ramp;          // the retarder's comeback after a shift is ramped (ramp_s on, the hook in)
    int pre;           // pre_blend on
    int sramp;         // the retarder builds up at a step up (step_ramp on, the hook in)
    int bridge;        // the brake bridges a step down (step_ramp on)
    float eta;         // seconds until the gearbox shifts down in braking (NAN = none expected)
    int ebb;           // engine_brake_bridge on
    int eb_stage;      // the engine brake's stage (0 = unknown), its share of the engine's stages (stage / cap, 0.5 if unknown) and the stage count (0 = unknown)
    float eb_share;
    int eb_cap;
    int fmode;         // step_force: the lever step is the game's own retarder at that step (step_force on, a retarder step held, the mass known for the road's own deceleration)
    int law;           // step_force 2: the lever is a share of what the truck can brake now (mass known, a retarder step or an engine-brake stage with a known stage count held)
    float lw_tgt;      // m/s2, the brakes' part the held step asks (the retarder's stage force at this road speed over the mass + the share of the engine brake in this gear and revs)
    float lw_rung;     // m/s2, the same for the lowest step of the lever (the rung step 1 is at least)
    float lw_whole;    // m/s2, the held step's without the fade (the retarder's whole stage force, the engine brake's whole stage): the finish of a stop is never more than this
    float lw_ret, lw_eb;   // m/s2, the retarder's and the engine brake's part of lw_tgt (the log)
    float ret_shape;   // the retarder's share of a step's target at the road speed of now (WP-BR3b, vehicle_ret_shape blended by retarder_fade; 1 = no curve: depth 0, no factor on the game's retarder, an engine brake alone)
};

static struct {
    int on;               // working: lever on, nothing stops it
    float a;              // measured deceleration (m/s2, + = slowing)
    float hv[64], hy[64]; // the speed samples of the last moments, and the brake model at each
    double ht[64], clk;   // their (running) times (double: the clock runs for hours)
    float y;              // the brake model: the request through the service brake's lag (what of it acts now)
    float ya;             // the model's mean over the measurement's window: what of it the measured deceleration shows
    int hn, hi;
    float bad_s;          // seconds without a usable derivative
    float tgt, ref, refm; // the step's target (tapered), the jerk-limited reference toward it, and that reference as the measurement can see it (delayed like it)
    float lvl, u;         // the integrator (the level that holds the reference), the request posted
    int gap;              // clutch open or neutral
    float gap_s;          // seconds in the gap
    int fix;              // the gap's correction: 0 = not yet, 1 = stepped (waiting to see its effect), 2 = done
    float fix_y, fix_a;   // the brake model as the measurement shows it, and the measured deceleration, before the step (a two-point gain measurement)
    float fix_r;          // the request's rate while the correction ramps in (the jerk limit with the gain the step assumed)
    float pre;            // the level before the gap (what the release goes back to)
    int g0;               // the gear as the gap began (the retarder's torque goes through the ratio: a lower gear brakes more)
    float kmin = DC_SOFT_MIN;   // the lowest soften factor now (see DC_SOFT_MIN)
    float ref0g;          // the reference as the gap began, with the retarder's curve on (-1 = not): what the retarder and the brake carried before it, which the curve lets fall during the gap
    float blind;          // seconds left in which the measurement still shows the shift's transient, not an error to correct
    float k = 0.1f;      // request per m/s2 (the inverse of the brake's gain): the vehicle model's estimate until a gap has taught it (the first tick sets it)
    int k_known;
    int k_prior;          // k comes from the vehicle model's CONFIDENT slot ([core] brake_learn), not yet from a gap of this event
    int k_fix;            // k comes from a gap's correction step (measured, not inferred from the level at the close)
    int step, n;          // what the target was made for (log)
    float hand_s;         // seconds standing while Auto Hold takes over
    float kr = 1.0f;      // soften: the loop's retarder factor (held in a gap)
    float ks = 1.0f;      // the factor shaped through a gap (to 1 as the retarder falls, back as it recovers)
    float kp = 1.0f;      // the factor posted last
    int rg;               // ramp: this gap is one (the retarder taken off in it)
    float kg;             // the factor as the gap began (down to 0 from there)
    float rt = -1;        // seconds since the close while the ramp runs (-1 = none)
    float rdl, r0;        // the ramp's change of the level (gap's to before) and its factor at the close
    float rT;             // the ramp's length (s): the hand-over (m/s2) over the jerk limit, at most ramp_s
    float rs;             // the retarder's share of the ramp so far: the share of the hand-over the brake model has made
    float ry0, rye;       // the brake model at the close and the level the ramp ends at (the hand-over, in request)
    float ph;             // pre-blend: the deceleration (m/s2) of the retarder's share handed to the brake so far
    float pbs;            // pre-blend: that share as the retarder's factor follows it (through the brake's lag)
    float pb_s;           // pre-blend: seconds the complete hand-over has been held
    int pb_lock;          // the hold ran out: no new hand-over until the prediction clears
    int ps;               // the lever step of the last tick (0 = none): a change is a step up or down
    int sr;               // a step ramp runs: the retarder is held to src until it has built up to the step
    float src, sr_r1;     // the retarder's deceleration the ramp allows now (m/s2), and what the new step gives at factor 1
    float on_s;           // seconds the assist has been working (the retarder builds up over ~1 s after the lever is pulled: its shortfall then is no need)
    int term;             // DT_*: the phase of the loop (what holds the request)
    float step_s;         // seconds left of the bridge of a step down (DT_STEP)
    float wish;           // the deceleration the engine-brake stages are wished to carry (the channel's allocation, core/brake.h), held through the shifts
    float drop_f; int drop_ok;   // where the shifts of this box really interrupt the braking: the share (0..1) of the lever's deceleration a shift takes off (1 = a clutch box, 0 = a converter box, V.tc); not learned (see the header of round 3 in docs)
    float ref_open;       // the reference as the last gap began (the lever's own deceleration then): the most the finish of a stop asks
    int law;              // this event's lever is a share of what the truck can brake now (step_force 2, in.law; decided at the pull as fm is)
    float vf, vh;         // law: the speed (m/s, through the measurement's window) and the lowest it has been since the pull: the speed the hold keeps
    float hold, lw_excess;   // law: the brakes' deceleration that holds the pull's speed (m/s2) and what of it is over the lowest rung (added to every step)
    int fm;               // this event's lever is the game's own retarder (no target, no loop outside the shifts; decided at the pull; step_force, in.fmode)
    float nat, nat_f;     // m/s2, the road's own deceleration the loop takes off the measured one (0 without fm), and its value through the measurement's own delay
    int nat_ok;           // nat_f has a value
    float aa;             // the measured deceleration less the road's own (the brakes' part: what every term of the loop works on; = D.a without fm)
} D;

// The engine brake bridge's state (see the header).
static const float EB_TAU = 0.8f;        // s, the smoothing of the deceleration the stage gives (the reference of a gap)
static const float EB_MIN_KMH = 8.0f;    // km/h, under it the engine brake is too weak to be worth a bridge (and the stop is the lever assist's)
static const float EB_ENDMIN = 0.3f;     // s, a gap this long must pass before its level counts as learned
static const float EB_REL_S = 0.3f;      // s, the level comes off over this long after the close (the engine brake is back in 0.1-0.4 s)
static const int EB_STAGES = 17;         // the per-stage memory (0 = stage unknown)
static struct {
    int have;            // ah holds a measurement
    float ah;            // the deceleration with the clutch closed, smoothed (m/s2, + = slowing)
    int gap;             // the clutch is open
    float gap_s, hold;   // seconds in the gap; the reference (ah as the gap began)
    float lvl, u, y;     // the level that holds it (request), the request posted now, the brake model
    float pk, rel;       // the highest level of this gap, the rate (request per second) the level comes off after the close
    int mark, idx;       // fix: 0 = not yet, 1 = the first look at the settled gap is done; the stage of this gap
    float blind;         // seconds in which the measurement still shows the change of the level
    int glast;           // the last gear seen with the clutch closed (the gear the engine brake works through; the gap's neutral has none)
    float dfc0;          // learned: the force (N) the service brake needed in the last gap while the stage was NOT known (its state not readable, or the game's own automatic engine brake): the model has no stage to keep it under
    int posted;          // the last tick posted a request > 0
    unsigned n_gap; int lines;
} E;

// The brake's gain the loop starts from (m/s2 per request): the vehicle model's estimate once its slot is confident (the truck's brakes were measured), and
// DC_GAIN_SHARE of it while only the start value stands (the share ramps to 1 with the slot's weight), inside the range the loop works with.
static float dc_gain()
{
    float g = vehicle_brake_est() * (DC_GAIN_SHARE + (1.0f - DC_GAIN_SHARE) * vehicle_brake_weight());
    return fminf(fmaxf(g, DC_GAIN_LO), DC_GAIN_HI);
}

// The target deceleration of a lever step, 1..n of n steps.
static float decel_target(int step, int n)
{
    if (n < 1) n = 1;
    if (step < 1) step = 1;
    if (step > n) step = n;
    if (n == 1) return p_tmax;
    float f = (float)(step - 1) / (float)(n - 1), c = p_tcurve > 0.05f ? p_tcurve : 0.05f;
    return p_tmin + (p_tmax - p_tmin) * powf(f, c);
}

static void soft_post(float k) { if (k != D.kp) { D.kp = k; act_retarder_scale(ACT_RET_DECEL, k); } }
static void soft_off() { D.sr = 0; D.kr = D.ks = 1.0f; D.rg = 0; D.rt = -1; D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0; soft_post(1.0f); }

static void decel_off()
{
    D.on = 0; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.bad_s = 0; D.wish = 0;
    act_brake(ACT_DECEL, 0);
}

// The clutch closed: learn what the gap needed (the level now, plus what the error left would still have needed; not at the
// ceiling, where the error says nothing about the gain) and go back to the level before the gap: at once, or with the ramp over
// ramp_s.
static void decel_release(float need, int g1, int soften)
{
    if (!D.k_fix && D.gap_s > 0.6f && need > 0.3f && D.lvl < p_max - 1e-3f) {
        float kn = (D.lvl + (D.refm - D.aa) * D.k) / need;
        const float k0 = 1.0f / dc_gain();
        if (kn > 0.2f * k0 && kn < 5.0f * k0) { D.k = D.k_known ? 0.5f * (D.k + kn) : kn; D.k_known = 1; D.k_prior = 0; }
    }
    // A lower gear: the retarder (and the engine brake) work through the ratio, so the same step brakes harder at the wheels. The level
    // the brake goes back to is the one that holds the target with the retarder's share of before times the change of the ratio.
    float rr = 1.0f;
    if (D.g0 > 0 && g1 > 0 && g1 < D.g0) {
        float a0 = vehicle_rpm_at(D.g0, 10.0f), a1 = vehicle_rpm_at(g1, 10.0f);
        if (a0 > 0 && a1 > a0) rr = fminf(a1 / a0, 2.0f);
    }
    float share = fmaxf((D.ref0g >= 0 ? D.ref0g : D.ref) - D.pre / D.k, 0);
    if (rr > 1.0f) {   // what the retarder carried before the gap (m/s2): against the reference of then when the retarder's curve has lowered it since
        D.pre = fminf(D.pre, fmaxf(D.ref - share * rr, 0) * D.k);
        // (stronger alone than the target now: soften's factor starts from the one that holds the target, not from the old one)
        if (soften && !D.fm && share > 0.05f && share * rr > D.ref) D.kr = fminf(fmaxf(D.kr * D.ref / (share * rr), D.kmin), 1.0f);
    }
    float to = fminf(D.lvl, D.pre);
    if (D.rg) {
        D.rdl = to - D.lvl; D.rt = 0; D.rs = 0; D.r0 = D.kp; D.rg = 0; D.ry0 = D.y; D.rye = to;
        // the build-up takes as long as the change of the deceleration it hands over needs at the jerk limit: a small difference
        // is done at once, a large one takes longer; ramp_s is only the longest it may take
        // (the lever is the game's own retarder: it comes back through the lower gear's ratio, share x rr, in the 0.3 s of its own build-up, and the plant's rise multiplies our factor's, so the sum swings more than
        // the difference of the two ends says: the ramp is as long as the larger of the brake's hand-over and the retarder's comeback needs at the jerk limit, and the retarder's factor climbs with the clock, not
        // with how far the brake has got: a small bridge no longer lets it back at once)
        const float hand = -D.rdl / D.k;
        D.rT = fminf(fmaxf(fmaxf(hand, D.fm ? share * rr : 0.0f) / jerk_now(), DC_RAMP_MIN), fmaxf(p_ramp, DC_RAMP_MIN));
        D.ks = D.kr;   // soften's own way down after the close is the ramp's
        D.blind = DC_BLIND_S;
    } else {
        D.lvl = to;
        D.blind = DC_REL_S + DC_BLIND_S;
    }
}

static void decel_tick(const decel_in &in)
{
    float dt = in.dt;
    if (!(dt > 0 && dt < 0.5f)) { if (dt != 0) D.hn = 0; return; }   // paused or a hitch: no time step, and the speed history is broken
    // The brake's gain is the vehicle model's (core/vehicle.cpp: the start value derived from the mass, moved towards what was measured), until a gap has
    // taught this event its own. With the model's slot confident (the truck's brakes were measured) the first shift asks nearly the full level (DC_FF_PRIOR)
    // and a shortfall in the first gap is corrected with that gain; with only the start value it asks DC_FF_FIRST of it and corrects as for a weaker brake.
    // A load change resets k_known and brings this back at the new mass.
    if (!D.k_known) {
        const float g = dc_gain();
        D.k = 1.0f / g;
        if (vehicle_brake_gain() == vehicle_brake_gain()) { D.k_known = D.k_prior = 1; logv(1, "Decel: brake gain %.1f m/s2 per request from the vehicle model (start value %.1f)", g, vehicle_brake_prior()); }
    }
    // the measured deceleration: the speed's derivative, smoothed; a sample that is not a number or looks like noise is dropped
    float v = fabsf(in.speed);
    int ok = 0;
    D.y += (D.u - D.y) * dt / (DC_LAG_S + dt);
    if (in.speed == in.speed) {
        D.clk += dt;
        D.hi = (D.hi + 1) % 64; D.hv[D.hi] = v; D.hy[D.hi] = D.y; D.ht[D.hi] = D.clk;
        if (D.hn < 64) D.hn++;
        float ys = D.y;
        for (int i = 1; i < D.hn; i++) {   // the newest sample that is at least the window old
            int j = (D.hi - i + 64) % 64;
            float span = (float)(D.clk - D.ht[j]);
            if (span >= DC_WIN) {
                float raw = (D.hv[j] - v) / span;
                if (fabsf(raw) < 30.0f) { D.a = raw; D.ya = ys / (float)i; ok = 1; }
                break;
            }
            ys += D.hy[j];
        }
    }
    D.bad_s = ok ? 0 : D.bad_s + dt;
    // A lever step is a FORCE (step_force, WP-FX1, ride 04-11): the loop works on what the brakes give, the measured deceleration less the road's own (rolling, air, the grade of now, the engine's drag
    // in a gear with the clutch closed: core/vehicle vehicle_natural_decel), through the delay of the measurement (the speed's fall over DC_WIN is the mean of the window, half of it old). Every term
    // below that reads the measurement reads D.aa: the PI's error, soften's excess, the overshoot guard, the reference's start, the step ramp's retarder, the release's share, the engine brake's wish.
    // Decided at the pull and kept for the event: a mass that flickers must not change the target in the middle of it. Without it D.aa is D.a and the road's own is 0: the older targets, to the bit.
    if (!D.on) { D.fm = in.fmode; D.law = !in.fmode && in.law; }
    if (D.fm || D.law) {
        const float n_now = vehicle_natural_decel(in.clutch < DC_CLOSED && in.gear > 0 ? in.gear : 0, v);
        if (!D.nat_ok || !(n_now == n_now)) { D.nat_f = n_now == n_now ? n_now : 0.0f; D.nat_ok = 1; }
        else D.nat_f += (n_now - D.nat_f) * dt / (0.5f * DC_WIN + dt);
        D.nat = D.nat_f;
    } else { D.nat = 0; D.nat_ok = 0; }
    D.aa = D.a - D.nat;
    float am = D.aa;
    // the actuator does not write our request (brake air low): the level must not wind up meanwhile
    int starved = D.u > 0.02f && in.held < 0.5f * D.u;
    int lever = in.retarder > 0 || in.motor_brake;
    // Ends at once: the driver's pedals, cruise (it owns the brake), reverse, engine off, a pause, no usable measurement, standing
    // with the lever still held. Without the lever the request winds down at the jerk limit instead, also at a standstill: the
    // driver's explicit release wins over the Auto Hold hand-over (review 30-10).
    int ends = in.paused || !in.engine_on || in.gear < 0 || in.thr > DC_PEDAL || in.brake > DC_PEDAL || in.cruise || in.other > 0 ||
               D.bad_s > DC_BAD_S;
    int standing = !(v > DC_STOP_V);
    int hard = ends || (standing && lever);
    float rlim = jerk_now() * D.k;          // request per second at the jerk limit (the learned request per m/s2, the nominal one at first)
    // Standing with nothing else ending it and Auto Hold taking the stop: the request stays as it is until the hold's has risen to it
    // (or to what the hold will hold, if less), then it goes at once: the actuator merges by the biggest wish, so the brake never lets go.
    if (!standing) D.hand_s = 0;
    else if (lever && !ends && D.on && in.take > 0) {
        D.hand_s += dt;
        if (D.hand_s < DC_HAND_S && in.hold < fminf(D.u, in.take) - 1e-3f) { act_brake(ACT_DECEL, D.u); soft_off(); return; }
    }
    if (hard) { D.ps = 0; if (D.on) decel_off(); D.wish = 0; brake_eb_wish(0); soft_off(); return; }
    if (!lever) {
        D.ps = 0; D.sr = 0; D.wish = 0; brake_eb_wish(0);   // (the lever let go: no request, the engine brake's stage goes with it at once)
        if (D.rg || D.rt >= 0 || D.pbs > 0) { D.rg = 0; D.rt = -1; soft_post(D.ks); }   // in a ramp's gap or ramp, or a hand-over ahead of a shift: to soften's factor (1 without it)
        D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0;
        if (D.kr < 1) { D.kr = D.ks = fminf(D.kr + DC_SOFT_UP * dt, 1.0f); soft_post(D.ks); }   // the lever released: back up, not a jolt
        if (D.on) {
            D.u = fmaxf(D.u - rlim * dt, 0);
            D.lvl = D.u;
            act_brake(ACT_DECEL, D.u);
            if (D.u <= 0) decel_off();
        }
        return;
    }
    if (!D.on) {   // starting: the reference starts at what the truck does now and climbs at the jerk limit, as the retarder builds up
        D.on = 1; D.rg = 0; D.rt = -1; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.hand_s = 0; D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0; D.on_s = 0;
        D.ref = D.refm = fmaxf(D.aa, 0);
        D.vf = D.vh = v;
        if (D.fm && in.soften) D.kr = D.ks = fminf(fmaxf(in.ret_shape, DC_SOFT_ABS), 1.0f);   // (the fade's share from the first tick; the step ramp builds the retarder up to it at the jerk limit)
    }
    D.on_s += dt;
    if (D.law) { D.vf += (v - D.vf) * dt / (DC_WIN + dt); D.vh = fminf(D.vh, D.vf); }   // the speed through the window, and the lowest it has been since the pull (the hold keeps it)
    int step = in.retarder > 0 ? (int)in.retarder : 1;   // an engine brake alone counts as the first step (of however many)
    int n = in.retarder > 0 ? (in.steps > step ? in.steps : step) : (in.steps > 1 ? in.steps : 2);
    D.step = step; D.n = n;
    int sev = D.ps == 0 ? 1 : step > D.ps ? 1 : step < D.ps ? -1 : 0;   // the lever was pulled or moved up a step (1), or down a step (-1)
    // the target, tapered toward a standstill along a = sqrt(2 j v) (a ramp of jerk j from a ends exactly at v = 0); the brake
    // lags, so the request leads that ramp by the lag: j x lag less (else the truck arrives braking and the last step is a jolt)
    // The retarder's fade (WP-BR3b, owner 03-11: "because the retarder becomes a brake assist instead of a real retarder you lack the natural fade: it just tops it up"). The game's retarder does not
    // fade with the road speed -- its torque follows the engine side through the ratio, a lower gear at a lower speed brakes more (tools/analyze_retarder_axis.py) -- so there is nothing in it to learn and
    // BR3's relative fade, which looked for one in the deceleration at each shift, is gone. A retarder as built goes with the square of its shaft speed up to the top stage's force; the lever's target is
    // that curve: the step's own target times s(v / knee) (vehicle_ret_shape: knee = the engine's peak power over the top stage's force, core/brakemodel.h), and the game's retarder is lowered to it by
    // soften's factor. The curve is a function of the road speed alone: continuous, the same at every shift (so the bridge fills a gap to the target of that moment, as before), and absolute (a step
    // set at 30 km/h fades from there as one set at 60 does). It needs the factor on the game's retarder (soften and the hook): without it a target under what the retarder gives is a jump at every
    // close, so it stays whole there. An engine brake alone has no retarder to fade.
    float tgt; int fin = 0;   // fin: the finish of a stop (force mode, the box's own neutral)
    if (D.fm) {
        // WP-FX1 round 2 (manager 04-11, the owner: "aim at a certain brake force per step"; the game's own step IS that force): the lever has NO target. The retarder is the game's, scaled by the fade
        // (the factor below, open loop); the reference is what the brakes gave a moment ago (the measured deceleration less the road's own), followed at the jerk limit and frozen from the instant the
        // clutch opens until the retarder is back after the close, so the bridge fills a gap to what the retarder gave just before it, at the speed of that moment. Nothing aims at a model's figure,
        // the service brake outside the shifts is 0, and a heavy truck's top step is the game's, not a number.
        const int opening = in.clutch > DC_OPEN || in.gear == 0;
        const int handing = D.ph > 0 || D.pbs >= 0.01f;   // a hand-over ahead of a shift moves the retarder's share to the brake: the sum is what the reference is, but it only rises meanwhile (the brake's lag dips the sum)
        tgt = !D.gap && !opening && D.rt < 0 && D.blind <= 0 ? (handing ? fmaxf(D.ref, D.aa) : fmaxf(D.aa, 0.0f)) : D.ref;
        // The end of a stop is the one thing the lever's force does not do: at the box's own neutral (the clutch open longer than a shift) the game's retarder is gone and the truck coasts, on a slope it runs.
        // The stop is finished the way the stop assist finishes one -- its gentle level along the same taper, a whole deceleration (the slope's pull is the road's own and is added), never more than the
        // lever gave as the gap began -- by the same loop as in a shift's gap. Not a target for the lever's force: it starts where the lever has nothing left to give.
        if (D.gap && D.gap_s > fmaxf(V.eng_ok ? V.eng.shift_time : 1.2f, 0.5f) + DC_STOP_PAD) {
            const float jt = DC_TAPER_J * jerk_now(), taper = sqrtf(2.0f * jt * v) - jt * DC_LAG_S;
            tgt = fmaxf(fminf(fminf(fminf(fmaxf(p_sad, 0.1f), 0.69f), fmaxf(taper, 0.0f)) - D.nat, fmaxf(D.ref_open, 0.0f)), 0.0f);
            fin = 1;
        }
    } else if (D.law) {
        // The lever's step is a share of what the truck can brake now (header, step_force 2). Step 1 is at least what holds the speed of the pull: the road's push (what the model says the truck gains by
        // itself, nothing where it slows) plus the speed it has gained over the lowest since the pull; what that needs over the lowest rung is added to every step, so the steps keep their distance.
        const float push = fmaxf(-D.nat, 0.0f);
        D.hold = push + fmaxf(D.vf - D.vh, 0.0f) / DC_HOLD_S;
        D.lw_excess = fmaxf(D.hold - in.lw_rung, 0.0f);
        const float jt = DC_TAPER_J * jerk_now(), taper = sqrtf(2.0f * jt * v) - jt * DC_LAG_S;
        const float room = fmaxf(taper - D.nat, 0.0f);   // the taper to a standstill is of the whole deceleration: the brakes' part is what is left of it after the road's own
        tgt = fminf(in.lw_tgt + D.lw_excess, room);
        // The last metres: the retarder's force goes with the square of the speed and the box drops to neutral, so the step's own target (with the fade) leaves the truck to roll on. The stop is finished the way
        // the stop assist finishes one -- its gentle level along the same taper, never more than the step's whole force. Max of two continuous functions: no step.
        tgt = fmaxf(tgt, fminf(fminf(fminf(fmaxf(p_sad, 0.1f), 0.69f), in.lw_whole + D.lw_excess), room));
    } else {
        tgt = decel_target(step, n) * in.ret_shape;
        const float jt = DC_TAPER_J * jerk_now(), taper = sqrtf(2.0f * jt * v) - jt * DC_LAG_S;
        if (taper < tgt) tgt = fmaxf(taper, 0);
        // The last metres (the assist's second job, header): the retarder's torque goes with v^2, under 6 % of a stage at a quarter of its knee, and the box drops to neutral and takes it out altogether, so the
        // curve alone leaves the truck to roll on at the drag's 0.1 m/s2 (a slope would not stop it). Below the curve the stop is finished the way the stop assist finishes a lift-off: its gentle level
        // (stop_assist_decel, clamped as it clamps it, the level under which a car's regenerative braking lights no brake lights) along the same taper, never more than the step's own target. Max of two
        // continuous functions: no step, and the reference follows at the jerk limit as ever. The stop assist itself is not touched. (Its own commit: it makes the service brake fill the lever's target
        // below ~8 km/h, which the curve alone does not.)
        if (in.ret_shape < 1.0f) tgt = fmaxf(tgt, fminf(fminf(fminf(fmaxf(p_sad, 0.1f), 0.69f), decel_target(step, n)), fmaxf(taper, 0)));
    }
    D.tgt = tgt;
    D.kmin = in.ret_shape < 1.0f || D.law ? DC_SOFT_ABS : DC_SOFT_MIN;   // (with the curve the factor goes as low as the target needs: the game's retarder in a low gear gives several times a stage's target at 10 km/h, and the floor of 0.2 left it over the target)
    const int alone = in.retarder > 0 && in.gear > 0 && in.clutch < DC_CLOSED && !D.gap && D.step_s <= 0 && D.blind <= 0 && !starved && !sev && D.on_s > DC_NEED_S && D.k_known;
    float dr = jerk_now() * dt;
    float ref0 = D.ref;
    if (D.fm) D.ref += fminf(fmaxf((tgt - D.ref) * dt / (0.5f * DC_WIN + dt), -DC_TAPER_J * dr), DC_TAPER_J * dr);   // (the reference follows what the brakes give through the measurement's own delay: the retarder builds up and falls by itself, and the jerk limit is not the pace of a measurement; its noise is filtered)
    else D.ref += D.ref < tgt ? fminf(dr, tgt - D.ref) : -fminf(dr, D.ref - tgt);
    if (D.fm) D.tgt = D.ref;   // (the reference is what the lever gives: that is what is shown and logged as the target)
    // The bridge fills the gap of THIS shift to the target of this moment (WP-BR3, the manager): the reference follows the target at the jerk limit, which the curve can outrun where it is steep; as the clutch opens
    // the reference falls to it at once (the feedforward asks the level of the reference as it stands when the gap begins, and the loop chases it all through the gap). With a target that is smooth in the
    // speed the two differ by little. The bridge is as it was -- the level that holds the target in the gap, at once, the same jerk rows -- for the target it now has.
    if (!D.fm && p_fade > 0 && in.ret_shape < 1.0f && !D.gap && (in.clutch > DC_OPEN || in.gear == 0) && D.ref > tgt) { D.ref = tgt; D.refm = fminf(D.refm, tgt); }
    // the gap: the clutch opens at a shift, or the box is in neutral (stop assist)
    // WP-FX1 round 3 (owner, after the Allison: "the retarder braked for comfort during the shifts, but on this truck that wasn't needed at all"): the bridge and the hand-over ahead of a shift are for a
    // box whose shift INTERRUPTS the braking. A torque converter box (V.tc) keeps the driveline closed through a shift and the retarder, on its output, brakes through it: nothing opens, so nothing is
    // bridged (only the box's own neutral at the end of a stop is a gap). drop_f is set from V.tc (1 for a clutch box, 0 for a converter box); it is NOT learned from the shifts (see round 3 in docs/powerband-change.md: the drop is unobservable in a ramp's gap).
    const int shift_gap = !((D.fm || D.law) && vehicle_tc());
    int open = (shift_gap && in.clutch > DC_OPEN) || in.gear == 0, closed = in.clutch < DC_CLOSED && in.gear != 0;
    if ((D.fm || D.law) && !D.drop_ok) { D.drop_f = vehicle_tc() ? 0.0f : 1.0f; D.drop_ok = 1; }
    int rg = D.gap ? D.rg : in.ramp && in.retarder > 0;   // a ramp's gap (the retarder taken off in it: no floor of its own)
    float need = fmaxf(D.ref - (rg ? 0 : DC_FLOOR), 0);
    int step_now = 0;   // the level jumped (the shift's feedforward or its release): the request follows at once
    float lvl = D.lvl;
    if (!D.gap && open) {
        if (D.rt >= 0) { lvl = fmaxf(lvl + D.rdl * (1 - fminf(D.rt / D.rT, 1.0f)), 0); D.rt = -1; }   // a ramp cut short: where it went
        D.g0 = in.gear; D.ref0g = in.ret_shape < 1.0f ? D.ref : -1.0f; D.ref_open = D.ref;
        D.gap = 1; D.gap_s = 0; D.fix = 0; D.pre = fmaxf(lvl - D.k * D.ph, 0); D.rg = rg; D.kg = D.kp;   // (what a hand-over ahead of the shift added is not the level before the gap)
        D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0;
        // the level that held the target in the last gap, at once; before a gap has taught it, only part of what the nominal brake
        // would need (too little the loop makes up in a moment, too much is a lurch)
        lvl = fminf(fmaxf(D.k * need * (D.fm || D.law ? D.drop_f : 1.0f) * (D.k_known ? (D.k_prior ? DC_FF_PRIOR : 1.0f) : DC_FF_FIRST), lvl), p_max);
        D.blind = DC_FF_S + DC_WIN;
        step_now = 1;
    } else if (D.gap && closed) {
        D.lvl = lvl;
        const float shift_len = fmaxf(V.eng_ok ? V.eng.shift_time : 1.2f, 0.5f) + DC_STOP_PAD;
        // Two shifts close together (10 > 8 > 7 within 0.7 s): ramping the retarder back after the first and handing it over again for the second needs more time than there is, and each swing of the sum
        // is a jerk. When the next shift is due within the time a hand-over takes (the pre-blend's own test), the first close does not ramp back: the brake keeps what it carries, the retarder stays handed
        // over, and the pre-blend below holds both until the second shift has opened and closed; if it does not come, the pre-blend gives the share back at the jerk limit, which is the ramp.
        // (the deceleration after the close is the lever's through the lower gear: the shift's due time is judged at that, not at the gap's)
        float rr_c = 1.0f;
        if (D.g0 > 0 && in.gear > 0 && in.gear < D.g0) { const float a0 = vehicle_rpm_at(D.g0, 10.0f), a1 = vehicle_rpm_at(in.gear, 10.0f); if (a0 > 0 && a1 > a0) rr_c = fminf(a1 / a0, 2.0f); }
        const float hand_s = D.ref_open / jerk_now() + DC_LAG_S + DC_PRE_MARGIN;   // (the time a hand-over takes: the pre-blend's own test)
        const float eta_c = D.fm && in.pre ? ts_brake_down_eta_hard(fmaxf(D.ref_open * rr_c + D.nat, 0.0f)) : NAN;
        if (D.fm) logv(2, "Decel: a shift closed after %.2f s: the next one is due in %.2f s (a hand-over takes %.2f s)%s", D.gap_s, eta_c, hand_s, eta_c == eta_c && eta_c < hand_s ? ": the retarder stays handed over" : "");
        if (D.fm && D.rg && in.pre && D.drop_f > 0.1f && D.gap_s < shift_len && D.ref_open > DC_PRE_MIN && eta_c == eta_c && eta_c < hand_s) {
            D.gap = 0; D.rg = 0; D.rt = -1; D.pre = 0; D.pb_s = 0; D.pb_lock = 0; D.ks = D.kr;
            D.ph = fminf(lvl / D.k, D.ref_open * D.drop_f);
            D.pbs = D.ref_open > 0.01f ? fminf(D.ph / D.ref_open, 1.0f) : 0.0f;
            D.blind = DC_BLIND_S;
            lvl = D.lvl;
        } else {
            decel_release(need, in.gear, in.soften);
            lvl = D.lvl;
            D.gap = 0;
        }
        step_now = 1;
    } else if (D.gap) D.gap_s += dt;
    // Step ramp: the game's retarder takes its new step's torque at once, a jump of the deceleration. With the hook it is held at what it
    // gave and let up at the jerk limit (src, the deceleration it may give), the reference rising with it; a step down is bridged by the
    // brake, which takes the retarder's drop at once (the brake's lag is shorter than the time the loop needs to make it up at the jerk
    // limit) and then gives it up with the reference. Not in a gap or a ramp of the shift (those own the retarder and the level then).
    if (sev && in.retarder > 0 && !D.gap && D.rt < 0 && (in.sramp || in.bridge)) {
        float r0 = D.sr ? D.src : fmaxf(D.aa - D.y / D.k, 0);   // what the retarder gives now
        if (sev > 0 && in.sramp) {
            float R1 = 0;
            if (D.sr) R1 = D.sr_r1 * (float)step / (float)(D.ps > 0 ? D.ps : 1);
            else if (D.ps > 0 && r0 > 0.15f && D.kp > 0.1f) R1 = r0 / D.kp * (float)step / (float)D.ps;
            else {   // the vehicle model's force of this stage (its start value is the 18 t truck's measured curve, learned per truck), as a deceleration at the mass of now: only until a step of this event has told
                R1 = vehicle_ret_force(step) / (V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK);
            }
            D.src = D.ps == 0 ? 0 : r0;
            D.sr = 1; D.sr_r1 = fminf(fmaxf(R1, 0.5f), 8.0f);
            logv(2, "Decel: step ramp %d>%d: the retarder gives %.2f m/s2 now (factor %.2f), the new step %.2f at factor 1", D.ps, step, D.src, D.kp, D.sr_r1);
        } else if (sev < 0) {
            D.sr = 0;
            if (in.bridge && D.ps > 0 && D.u < DC_BRIDGE_U) {   // (with the brake already carrying the lever's share the loop makes it up as the reference falls: the sim says a bridge only adds then)
                float drop = r0 * (1.0f - (float)step / (float)D.ps) * DC_BRIDGE;   // (the retarder at the new step is the old one times the step ratio)
                lvl = fminf(fmaxf(lvl + D.k * drop, 0), p_max);
                step_now = 1; D.step_s = DC_STEP_S;
            }
        }
    }
    if (D.rt >= 0) {   // the ramp's share of the level for this tick
        float s0 = fminf(D.rt / D.rT, 1.0f);
        D.rt += dt;
        lvl = fmaxf(lvl + D.rdl * (fminf(D.rt / D.rT, 1.0f) - s0), 0);
    }
    D.refm += dt / (DC_REFM_S + dt) * (D.ref - D.refm);
    // Pre-blend: the gearbox is about to shift down (its rpm falling to the point of a braking downshift at the deceleration of now).
    // When the shift is nearer than the hand-over needs (the retarder's share over the jerk limit, the brake's lag and a margin), the
    // retarder's share goes to the service brake at the jerk limit, the factor following the brake model, so that the clutch opens on a
    // brake that already holds the target. The share unwinds the same way when the shift does not come.
    if (!D.gap && D.rt < 0) {
        float base = fmaxf(lvl - D.k * D.ph, 0);
        float share = fmaxf(D.ref - base / D.k, 0);   // what the retarder carries now (m/s2)
        const float fdrop = D.fm || D.law ? D.drop_f : 1.0f;   // (the share of the retarder that a shift of this box takes off; a converter box: none)
        float cap = (in.ramp ? share : fminf(share, DC_PRE_OVS)) * fdrop;
        float near_s = share * fdrop / jerk_now() + DC_LAG_S + DC_PRE_MARGIN + (D.ph > 0 ? DC_PRE_HYS : 0);
        int expect = in.pre && !((D.fm || D.law) && (vehicle_tc() || fdrop <= 0.1f)) && share > DC_PRE_MIN && in.eta == in.eta && in.eta < near_s;
        if (!expect) D.pb_lock = 0;
        float goal = expect && !D.pb_lock ? cap : 0;
        if (goal > 0 && D.ph >= cap - 1e-3f) {
            D.pb_s += dt;
            if (D.pb_s > DC_PRE_HOLD_S) { D.pb_lock = 1; goal = 0; logv(2, "Decel: no shift after the hand-over was ready, given up"); }
        } else if (goal <= 0) D.pb_s = 0;
        float st = fminf(fmaxf(goal - D.ph, -jerk_now() * dt), jerk_now() * dt);
        if (st != 0) {
            float l2 = fminf(fmaxf(lvl + D.k * st, 0), p_max);
            D.ph = fmaxf(D.ph + (l2 - lvl) / D.k, 0);
            lvl = l2;
        }
        if (in.ramp) D.pbs += ((share > 1e-3f ? fminf(D.ph / share, 1.0f) : 0) - D.pbs) * dt / (DC_LAG_S + dt);
        else D.pbs = 0;
    }
    if (in.soften) {
        // soften: lowered only while over the target with none of our brake on, raised while short (out of the transients)
        if (D.fm) {   // the lever is the game's own: the factor is the fade's share of the road speed, open loop (it follows the speed, which changes slowly; the rate is the legacy rise rate against a glitch)
            const float want_k = fminf(fmaxf(in.ret_shape, DC_SOFT_ABS), 1.0f);
            D.kr += fminf(fmaxf(want_k - D.kr, -DC_SOFT_UP * dt), DC_SOFT_UP * dt);
        } else if (!D.gap && D.blind <= 0 && D.ph <= 0 && D.pbs < 0.01f && !D.sr && E.u <= 0.01f) {
            float over = am - D.tgt, r = fmaxf(am / D.kr, 0.5f), dk = 0;   // r: the deceleration at 1 (the drag in it too: a lower gain, never a higher one)
            if (over > DC_SOFT_DB && D.u < DC_SOFT_U && (D.kr < 1 || over > DC_SOFT_TOL)) dk = -DC_SOFT_KI * (over - DC_SOFT_DB) / r;
            else if (over < -DC_SOFT_DB && D.kr < 1) dk = DC_SOFT_KI * (-over - DC_SOFT_DB) / r;
            D.kr = fminf(fmaxf(D.kr + fminf(fmaxf(dk, -DC_SOFT_DN), DC_SOFT_UP) * dt, D.kmin), 1.0f);
        }
        if (D.gap) D.ks = fminf(D.ks + (1 - D.kr) / DC_SOFT_GAP_UP * dt, 1.0f);
        else D.ks = D.kr >= D.ks ? D.kr : fmaxf(D.kr, D.ks - (1 - D.kr) / DC_SOFT_GAP_DN * dt);
    }
    float kf = in.soften ? D.ks : 1.0f;
    int ramp_on = D.rt >= 0;   // (the tick that ends the ramp still posts the request unslewed)
    if (D.rg) kf = D.kg * fminf(fmaxf(1 - (D.gap_s - DC_LAG_S) / DC_FF_S, 0), 1.0f);   // a ramp's gap: off as the brake's feedforward arrives
    else if (D.rt >= 0) {   // the ramp: the retarder takes over what the brake gives up, as the brake model has it (the request leads by the lag, below)
        float span = D.ry0 - D.rye;
        // (the hand-over only goes forward: the loop may take the brake up again meanwhile, that is not the retarder going back; and
        // it ends at the latest a few lags after the level did, whatever the brake model does)
        D.rs = D.fm ? fminf(fmaxf(D.rt / D.rT, 0.0f), 1.0f) : fmaxf(D.rs, span > 0.01f ? fminf(fmaxf((D.ry0 - D.y) / span, 0), 1.0f) : 1.0f);
        if ((D.rs >= 0.97f && D.rt >= D.rT) || D.rt >= D.rT + DC_RAMP_TAIL) D.rt = -1;
        else kf = D.r0 + (kf - D.r0) * D.rs;
    } else kf *= 1.0f - D.pbs;
    if (D.sr) {   // the step ramp: the retarder's share grows at the jerk limit up to what the factor would give anyway
        if (D.gap || D.rt >= 0 || !in.sramp) D.sr = 0;
        else {
            D.src += jerk_now() * dt;
            int capped = 0;
            if (in.soften && !D.fm) {   // (a retarder stronger than the step's target needs: the build-up stops at the target, soften takes it from there)
                float cap = fmaxf(D.ref - D.y / D.k, 0) + DC_SR_MARGIN;
                if (D.src > cap) { D.src = cap; capped = 1; }
            }
            float fs = fminf(D.src / D.sr_r1, 1.0f);
            if (fs >= kf) D.sr = 0;
            else {
                kf = fs;
                if (capped && fabsf(D.ref - D.tgt) < 0.02f) { D.sr = 0; D.kr = D.ks = fminf(D.kr, fs); }   // (arrived: the factor goes on from here as soften's own)
            }
        }
    }
    if (in.soften || in.ramp || in.sramp) soft_post(kf);
    float e = D.refm - am;
    if (in.soften && D.kr < 1 && !D.gap && e > 0) e = 0;   // short with the retarder lowered: it comes back first, the brake waits
    int guard = am > D.refm + DC_TOL;
    if (D.fm && !D.gap) { e = 0; guard = 0; }   // no loop outside a shift: the retarder is the game's, there is nothing to track
    // the level: the steps of the shift above, the PI integral on top (rate-limited, no windup)
    float up = rlim, dn = guard ? 2 * rlim : rlim;
    if (D.gap && D.fix == 1) { up = fmaxf(up, D.fix_r); dn = fmaxf(dn, D.fix_r); }
    // the feedback waits while the measurement still shows the transient of the last change (only the overshoot guard acts then)
    if (D.blind > 0) { D.blind -= dt; if (!guard) e = 0; }
    else if (D.gap && D.fix == 0 && in.gear != 0 && !starved) {
        // The gap has settled (the retarder at its floor, the feedforward in): what is still off is the feedforward's error (the
        // first gap's guess of the gain, a new load). The integral is rate-limited and slow next to a gap of 1.1 s, so one step of
        // the level for it, ramped at the jerk limit. Before a gap has taught the gain: a shortfall means a brake weaker than the
        // first share assumed (the model's gain over DC_FF_FIRST), so the step assumes that share of it; an excess means a stronger
        // brake, the model's gain. The step is also a clean measurement of the gain: the floor of the gap does not move between
        // before and after it.
        D.fix = 2;
        if (fabsf(e) > DC_FIX_E) {
            float kc = D.k_known || e < 0 ? D.k : D.k / DC_FF_FIRST;
            // (less what of the request the measurement does not show yet: the brake's lag and the window still hold part of the feedforward)
            float to = fminf(fmaxf(lvl + e * kc - fmaxf(D.u - D.ya, 0), 0), p_max);
            if (fabsf(to - lvl) > 0.02f) {
                D.fix = 1; D.fix_y = D.ya; D.fix_a = am; D.fix_r = jerk_now() * kc;
                D.blind = fabsf(to - lvl) / D.fix_r + DC_BLIND_S;
                lvl = to;
            }
        }
        e = 0;
    } else if (D.gap && D.fix == 1) {
        // the step's effect is in the measurement: the gain, request per m/s2 (the floor of the gap cancels; the brake model, seen
        // through the same window as the measurement, says how much of the step the measured deceleration holds by now)
        D.fix = 2;
        // (only a step big enough to stand out of the noise; a later one refines the first by half)
        float dy = D.ya - D.fix_y, da = am - D.fix_a, kn = da != 0 ? dy / da : 0;
        if (fabsf(dy) > 0.04f && fabsf(da) > 0.2f && kn > 1.0f / DC_GAIN_HI && kn < 1.0f / 2.0f) {
            D.k = D.k_fix ? 0.5f * (D.k + kn) : kn; D.k_known = D.k_fix = 1; D.k_prior = 0;
        }
    }
    // a falling reference (the taper to a stop, a lower step) takes its share of the level with it, and in a gap (the brake
    // alone holds the deceleration there) a rising one too: the integral alone would lag the ramp
    if ((!D.fm || fin) && (D.ref < ref0 || D.gap)) lvl = fminf(fmaxf(lvl + (D.ref - ref0) * D.k, 0), p_max);
    float di = fminf(fmaxf(DC_KI * e, -dn), starved ? 0 : up) * dt;
    if (!((lvl >= p_max && di > 0) || (lvl <= 0 && di < 0))) lvl += di;
    if (D.fm && !D.gap && D.rt < 0 && D.ph <= 0 && D.pbs < 0.01f && D.step_s <= 0) lvl = fmaxf(lvl - rlim * dt, 0);   // what is left of a bridge (a step down's) is given up at the jerk limit
    D.lvl = fminf(fmaxf(lvl, 0), p_max);
    // in the ramp the request leads the level by the brake's lag, so the deceleration the brake gives (the request through the lag)
    // falls as the level does and the retarder can follow the level's share at once
    float lead = ramp_on && D.rt >= 0 ? D.rdl / D.rT * DC_LAG_S : 0;
    float want = fminf(fmaxf(D.lvl + DC_KP * e + lead, 0), p_max);
    D.u = step_now || ramp_on ? want : fminf(fmaxf(want, D.u - dn * dt), D.u + up * dt);
    D.ps = step;
    if (D.step_s > 0) D.step_s -= dt;
    D.term = D.rt >= 0 ? DT_CLOSE : D.gap ? DT_GAP : D.step_s > 0 ? DT_STEP : D.ph > 0 ? DT_PREBLEND : DT_TRACK;
    // The engine brake first (WP-BR3, owner 03-11): what the reference still lacks of the deceleration the truck gives without our brake is the service brake's share to carry, and the engine
    // brake's stages are cheaper than the service brake: the brake channel allocates the stage that fits it (core/brake.h), counted in the wish with the worth of the stage already on, so the wish
    // does not change when the stage comes on. Measured where the retarder works alone; through a shift (the engine brake is gone with the clutch open) the last value stands.
    if (alone && !D.fm) D.wish = fmaxf(D.ref - fmaxf(am - D.ya / D.k, 0.0f), 0.0f) + brake_eb_worth();
    brake_eb_wish(D.wish);
    act_brake(ACT_DECEL, D.u);
}

// ---------------------------------------------------------------- the engine brake bridge
static void eb_bridge(const decel_in &in)
{
    float dt = in.dt;
    if (!(dt > 0 && dt < 0.5f)) { if (dt != 0) { E.have = 0; E.gap = 0; } return; }
    const float v = fabsf(in.speed), k = D.k, rlim = jerk_now() * k;
    int gate = in.ebb && in.motor_brake && in.steps == 0 && in.retarder == 0 && in.engine_on && !in.paused && in.gear >= 0 &&
               !(in.thr > DC_PEDAL) && !(in.brake > DC_PEDAL) && !(in.other > 0) && v * 3.6f > EB_MIN_KMH && D.bad_s <= DC_BAD_S;
    if (!gate) {   // let go at twice the jerk limit (a pedal or another owner has taken over)
        E.y += (E.u - E.y) * dt / (DC_LAG_S + dt);
        E.have = 0; E.gap = 0; E.hold = 0;
        if (E.lvl > 0 || E.u > 0) { E.lvl = fmaxf(E.lvl - 2 * rlim * dt, 0); E.u = E.lvl; }
        return;
    }
    int open = in.clutch > DC_OPEN || in.gear == 0, closed = in.clutch < DC_CLOSED && in.gear != 0;
    if (closed && in.gear > 0) E.glast = in.gear;
    if (!E.have) { E.ah = D.a; E.have = 1; E.gap = 0; }
    E.y += (E.u - E.y) * dt / (DC_LAG_S + dt);   // the brake model: what of our request acts now
    int step_now = 0;
    if (!E.gap) {
        if (open) {   // the gap begins: the reference is what the stage gave, the level of the last gap of this stage goes in at once
            E.gap = 1; E.gap_s = 0; E.mark = 0; E.pk = 0; E.hold = fminf(fmaxf(E.ah, -1.5f), 3.0f);
            E.idx = in.eb_stage < 0 || in.eb_stage >= EB_STAGES ? 0 : in.eb_stage;
            // m/s2 the stage's engine brake is worth: the vehicle model's force of this stage in the gear before the gap (the start value from the engine's brake
            // torque through the gear, then what the earlier gaps of this truck taught it; a force, so it holds at another load). Once this stage has been
            // learned that is all of it; before, 0.75 of the larger of it and what the truck decelerates with now. A stage that is not known counts as the middle
            // one (the old default share of 0.5) and has its own one-slot memory of the last gap (dfc0)
            const bool known = E.idx > 0 && in.eb_cap > 0;
            const int ms = known ? E.idx : in.eb_cap > 0 ? (in.eb_cap + 1) / 2 : 0;
            const float eb = ms > 0 ? vehicle_eb_decel(ms, in.eb_cap, E.glast) : 0.0f;
            float ff = known && vehicle_eb_events(E.idx) > 0 ? eb : !known && E.dfc0 > 0 && V.mass > 500 ? E.dfc0 / V.mass : 0.75f * fmaxf(eb, 0.6f * E.hold);
            E.lvl = fminf(fmaxf(E.lvl, k * ff), p_max);
            E.blind = DC_FF_S + DC_WIN;
            step_now = 1;
        } else if (closed && E.u <= 0.02f) E.ah += (D.a - E.ah) * dt / (EB_TAU + dt);   // (our own brake in it, when it brakes: not a measurement of the stage)
    } else {
        E.gap_s += dt;
        E.pk = fmaxf(E.pk, E.lvl);
        if (closed) {   // the clutch closed: what the gap needed is learned, and the level goes down from here, over the time the engine brake takes to come back
            E.gap = 0; E.n_gap++;
            if (E.gap_s > 0.7f && V.mass > 500 && E.pk < p_max - 1e-3f) {   // what the gap needed is the stage's force: the model learns it
                const float f = E.pk / k * V.mass;
                if (E.idx > 0 && in.eb_cap > 0) vehicle_eb_note(E.idx, in.eb_cap, E.glast, f);
                else E.dfc0 = E.dfc0 > 0 && f < E.dfc0 ? 0.5f * (E.dfc0 + f) : f;   // (what fell short is taken at once, what was too much by halves)
            }
            // the brake lags the request, the engine brake is back over EB_REL_S: the request leads by the lag, so what acts follows (1 - t / EB_REL_S)
            E.rel = fmaxf(E.lvl / EB_REL_S, rlim);
            E.lvl *= 1.0f - fminf(DC_LAG_S / EB_REL_S, 0.6f);
            step_now = 1;
            if (E.lines < 40) { E.lines++; logv(1, "Decel: engine brake bridge: shift of %.1f s at %.0f km/h held %.2f m/s2 with a request of up to %.2f (measured %.2f)", E.gap_s, v * 3.6f, E.hold, E.pk, D.a); }
        }
    }
    float e = 0;
    if (E.gap) e = E.hold - D.a;                                // short: the brake adds
    float up = 2.5f * rlim, dn = E.rel;
    if (E.blind > 0) { E.blind -= dt; if (e > 0) e = 0; }
    if (E.gap) {
        // once the gap has settled (the engine brake's fall is in the measurement), one correction for what the guess missed
        if (!E.mark && E.gap_s >= 0.55f && E.blind <= 0) {
            E.mark = 1;
            if (fabsf(e) > DC_FIX_E) { E.lvl = fminf(fmaxf(E.lvl + 0.8f * (k * e - fmaxf(E.u - E.y, 0)), 0), p_max); step_now = 1; E.blind = DC_BLIND_S; }
        }
        float di = fminf(fmaxf(3 * DC_KI * e, -rlim), up) * dt;
        if (!((E.lvl >= p_max && di > 0) || (E.lvl <= 0 && di < 0))) E.lvl += di;
    } else if (E.lvl > 0) {
        // the engine brake is back: over the reference the level comes off twice as fast
        E.lvl -= (D.a > E.hold + DC_TOL ? 2 * dn : dn) * dt;
    }
    E.lvl = fminf(fmaxf(E.lvl, 0), p_max);
    float want = fminf(fmaxf(E.lvl + (E.gap ? DC_KP * e : 0.0f), 0), p_max);
    E.u = step_now || !E.gap ? want : fminf(fmaxf(want, E.u - rlim * dt), E.u + up * dt);
}

// ---------------------------------------------------------------- Comfort ([decel] comfort_learn)
// The cab's pitch is the measure of how the deceleration feels: the placement's pitch (turns, + = nose up) less the road's grade
// (V.grade_deg; the two agree to a correlation of 0.99 on a ride). After every lever event (pull, release, a step up or down, a gearbox
// shift with the lever held) the pitch is watched for 3 s. The swing is how far it goes beyond the span between the level before the
// event and the level it settles to (the last 0.5 s): a dive that is held is not a swing, a dive and a spring back is. The peak-to-peak
// and the number of reversals (turning points over 0.05 deg) are logged with it. One log line per event, a summary when the session ends.
// With comfort_learn the swing of a pull or a step up steers this truck's jerk limit (those are the build-ups the jerk limit shapes; the
// others are logged only): over the target it comes down (by how far over), well under it and after a step of 0.5 m/s2 or more it goes up a little, always inside CL_LO..CL_HI (the swing is judged per m/s2 of the step: a step of 2 m/s2 may swing twice what one of 1 does, steps under 0.5 count as 0.5). Only a clean window counts: no pedal, cruise, other brake
// owner, emergency or hard braking (over 3.8 m/s2), pause, jump in the grade, bad speed or pitch data, no other lever move meanwhile,
// and the jerk limit the same throughout. The value is kept per truck (comfort_learn.txt, the truck's key as brake_gain.txt has it).
static const float CL_WIN_S = 3.0f;      // s, a window
static const float CL_DT = 0.05f;        // s between the window's samples
static const float CL_TAU = 0.1f;        // s, the pitch's smoothing (noise)
static const float CL_PRE_TAU = 0.4f;    // s, the pitch's slower average: the level before an event
static const float CL_HYST = 0.05f;      // deg, a turning point needs this much
static const float CL_MIN_KMH = 15.0f;   // km/h, no window below (the taper to a stop is not a comfort question)
static const float CL_HARD = 3.8f;       // m/s2, braking harder than the lever's top step is not ours to learn from
static const int CL_N = 72;
enum { CE_ON, CE_OFF, CE_UP, CE_DOWN, CE_SHIFT };
static const char *const CL_NAME[] = { "pull", "release", "step up", "step down", "shift" };
static const char *const CL_FILE = "comfort_learn.txt";
static struct {
    double clk;
    int pv; float cp, pre;                           // the pitch less the grade (deg), smoothed; its slower average
    int act, kind, s0, s1, nb, nsamp;                // a window is open: its event, the steps, the samples taken
    const char *bad;                                 // why the window does not count (NULL = it does)
    float t, tick, kmh0, grade0, j0, dA, c0, buf[CL_N];
    int p_lever, p_step, p_gap, p_on;                // the last tick
    char key[64]; unsigned gen;
    int dirty_save; double save_at;
    // the session
    int n_ev, n_scored, n_adj, n_rev, n_store;   // (n_adj: the adjustments of this truck this session; n_store: what the file said it had had)
    float swings[256], swing_max, j_start;
} Q;

static int cl_parse(const char *text, const char *key, float *j, int *n)
{
    char want[96]; snprintf(want, sizeof want, "truck=%s ", key);
    for (const char *ln = text; ln && *ln;) {
        if (!strncmp(ln, want, strlen(want))) {
            const char *q = strstr(ln, " jerk=");
            float jv = 0; int nv = 0;
            if (q && sscanf(q + 6, "%f", &jv) == 1 && jv == jv && jv >= 0.5f && jv <= 10.0f) {
                const char *m = strstr(ln, " n=");
                if (m) sscanf(m + 3, "%d", &nv);
                *j = jv; *n = nv;
                return 1;
            }
            return 0;
        }
        const char *nl = strchr(ln, '\n');
        ln = nl ? nl + 1 : NULL;
    }
    return 0;
}

// The file's text with this truck's line replaced (the other trucks' lines are kept, a few in all).
static int cl_compose(char *out, int cap, const char *text, const char *key, float j, int n)
{
    char want[96]; snprintf(want, sizeof want, "truck=%s ", key);
    int m = 0, lines = 0;
    for (const char *ln = text; ln && *ln && lines < 15;) {
        const char *nl = strchr(ln, '\n');
        int len = nl ? (int)(nl - ln) : (int)strlen(ln);
        if (len > 0 && !strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + len + 2 < cap - 120) {
            memcpy(out + m, ln, (size_t)len); m += len; out[m++] = '\n'; lines++;
        }
        ln = nl ? nl + 1 : NULL;
    }
    m += snprintf(out + m, (size_t)(cap - m), "truck=%s v=1 jerk=%.2f n=%d\n", key, j, n);
    return m;
}

static void cl_load(const char *key)
{
    float j = 0; int n = 0;
    FILE *f = open_out(CL_FILE, "r");
    if (f) {
        char buf[2048];
        int r = (int)fread(buf, 1, sizeof buf - 1, f);
        fclose(f);
        buf[r > 0 ? r : 0] = 0;
        if (cl_parse(buf, key, &j, &n)) {
            CL.j = fminf(fmaxf(j, CL_LO), CL_HI); Q.n_store = n;
            logv(1, "Decel: comfort: jerk limit %.2f m/s3 loaded for '%s' (%d adjustments)", CL.j, key, n);
            return;
        }
    }
    CL.j = fminf(fmaxf(p_jerk, CL_LO), CL_HI); Q.n_store = 0;
    logv(1, "Decel: comfort: nothing stored for '%s', starting at %.2f m/s3", key, CL.j);
}

static void cl_save(const char *why)
{
    if (!Q.dirty_save || !Q.key[0] || !(CL.j > 0)) return;
    char buf[2048], out[2200];
    int r = 0;
    FILE *f = open_out(CL_FILE, "r");
    if (f) { r = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[r > 0 ? r : 0] = 0;
    int m = cl_compose(out, sizeof out, buf, Q.key, CL.j, Q.n_store + Q.n_adj);
    Q.save_at = wall_s();
    f = open_out(CL_FILE, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Decel: comfort: cannot write %s (the value stays in memory)", CL_FILE); return; }
    size_t w = fwrite(out, 1, (size_t)m, f);
    fclose(f);
    if (w == (size_t)m) { Q.dirty_save = 0; logv(2, "Decel: comfort: saved (%s)", why); }
}

static void cl_summary()
{
    if (Q.n_ev <= 0) return;
    float med = 0;
    if (Q.n_scored > 0) {
        int n = Q.n_scored < 256 ? Q.n_scored : 256;
        float a[256];
        memcpy(a, Q.swings, (size_t)n * sizeof a[0]);
        for (int i = 1; i < n; i++) { float x = a[i]; int k = i - 1; while (k >= 0 && a[k] > x) { a[k + 1] = a[k]; k--; } a[k + 1] = x; }
        med = n % 2 ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
    }
    logv(1, "Decel: comfort session: %d events, %d scored, pitch swing median %.2f / worst %.2f deg, %.1f reversals per event, jerk limit %.2f -> %.2f m/s3 (%d changes)",
         Q.n_ev, Q.n_scored, med, Q.swing_max, Q.n_ev > 0 ? (float)Q.n_rev / (float)Q.n_ev : 0, Q.j_start, jerk_now(), Q.n_adj);
    Q.n_ev = Q.n_scored = Q.n_rev = 0; Q.swing_max = 0; Q.j_start = jerk_now();
}

// A window is over (3 s, or cut by the next event): its figures, the log line, and the learning.
static void cl_finish(int cut)
{
    Q.act = 0;
    if (Q.nb < 12) return;
    float s[CL_N];
    int n = Q.nb;
    memcpy(s, Q.buf, (size_t)n * sizeof s[0]);
    int tail = n >= 40 ? 10 : n / 4;
    float c1 = 0, mx = s[0], mn = s[0];
    for (int i = 0; i < n; i++) { mx = fmaxf(mx, s[i]); mn = fminf(mn, s[i]); if (i >= n - tail) c1 += s[i] / (float)tail; }
    float hi = fmaxf(Q.c0, c1), lo = fminf(Q.c0, c1);
    float swing = fmaxf(fmaxf(mx - hi, lo - mn), 0), p2p = mx - mn;
    float ext = Q.c0; int dir = 0, rev = 0;
    for (int i = 0; i < n; i++) {
        float x = s[i];
        if (dir == 0) { if (x - ext > CL_HYST) { dir = 1; ext = x; } else if (ext - x > CL_HYST) { dir = -1; ext = x; } }
        else if (dir > 0) { if (x > ext) ext = x; else if (ext - x > CL_HYST) { dir = -1; ext = x; rev++; } }
        else { if (x < ext) ext = x; else if (x - ext > CL_HYST) { dir = 1; ext = x; rev++; } }
    }
    const char *bad = Q.bad;
    if (cut) bad = "cut short by the next lever move";
    if (!bad && fabsf(V.grade_deg - Q.grade0) > 1.0f) bad = "the grade changed";
    if (!bad && fabsf(jerk_now() - Q.j0) > 1e-3f) bad = "the jerk limit changed";
    Q.n_ev++;
    char st[40] = "";
    if (Q.kind == CE_UP || Q.kind == CE_DOWN) snprintf(st, sizeof st, " %d>%d", Q.s0, Q.s1);
    logv(1, "Decel: comfort %s%s at %.0f km/h: pitch swing %.2f deg (peak-to-peak %.2f), %d reversal%s, jerk limit %.2f m/s3%s%s%s", CL_NAME[Q.kind], st, Q.kmh0, swing, p2p, rev,
         rev == 1 ? "" : "s", Q.j0, bad ? " (not scored: " : "", bad ? bad : "", bad ? ")" : "");
    if (bad) return;
    Q.n_rev += rev;
    if (Q.n_scored < 256) Q.swings[Q.n_scored] = swing;
    Q.n_scored++;
    Q.swing_max = fmaxf(Q.swing_max, swing);
    if (p_cl <= 0 || (Q.kind != CE_ON && Q.kind != CE_UP)) return;   // (a step down, a release and a shift are logged: the jerk limit has little say in them)
    float j = CL.j;
    float sc = swing / fmaxf(Q.dA, 0.5f);   // per m/s2 of the step: a big step may swing more than a small one (and a small one's swing is mostly the road)
    if (sc > p_ct) j -= CL_DOWN * fminf(fmaxf(sc / p_ct - 1.0f, 0.2f), 2.0f);
    else if (Q.kind == CE_UP && sc < CL_UP_FRAC * p_ct && Q.dA >= 0.5f) j += CL_UP;   // (a pull builds up over the game's own second whatever the jerk limit: it only ever brings it down)
    j = fminf(fmaxf(j, CL_LO), CL_HI);
    if (fabsf(j - CL.j) > 1e-4f) {
        logv(1, "Decel: comfort learn: jerk limit %.2f -> %.2f m/s3 (swing %.2f deg for a step of %.1f m/s2 = %.2f per m/s2, target %.2f)", CL.j, j, swing, fmaxf(Q.dA, 0.5f), sc, p_ct);
        CL.j = j; Q.n_adj++; Q.dirty_save = 1;
    }
}

static void cl_start(int kind, int s0, int s1, float dA, const decel_in &in)
{
    if (Q.act) { if (kind == CE_SHIFT) return; cl_finish(1); }
    float kmh = fabsf(in.speed) * 3.6f;
    if (!(kmh >= CL_MIN_KMH) || !Q.pv) return;
    Q.act = 1; Q.kind = kind; Q.s0 = s0; Q.s1 = s1; Q.dA = dA; Q.kmh0 = kmh; Q.grade0 = V.grade_deg; Q.j0 = jerk_now();
    Q.c0 = Q.pre; Q.t = Q.tick = 0; Q.nb = 0; Q.bad = NULL;
}

static void cl_frame(const decel_in &in)
{
    float dt = in.dt;
    if (!(dt > 0 && dt < 0.5f)) { if (Q.act && dt >= 0.5f) Q.bad = "a pause or a hitch"; return; }
    Q.clk += dt;
    if (p_cl > 0 && V.eng_ok && T.fwd_gears > 0 && (!Q.key[0] || Q.gen != T.cfg_gen)) {   // the truck is known (or changed): what it learned before
        char key[64];
        snprintf(key, sizeof key, "E%.0f-%.0f-G%d-W%d", V.eng.torque, V.eng.rpm_limit, T.fwd_gears, T.wheels);
        Q.gen = T.cfg_gen;
        if (strcmp(key, Q.key)) {
            if (Q.key[0]) { if (Q.act) cl_finish(1); cl_summary(); cl_save("truck changed"); Q.n_adj = 0; }
            snprintf(Q.key, sizeof Q.key, "%s", key);
            cl_load(Q.key);
            Q.j_start = CL.j;
        }
    }
    float pd = (float)T.place.orientation.pitch * 360.0f, c = pd - V.grade_deg;
    int okp = pd == pd && V.grade_deg == V.grade_deg && fabsf(c) < 8.0f;
    if (okp) {
        if (!Q.pv) { Q.cp = Q.pre = c; Q.pv = 1; }
        else { Q.cp += (c - Q.cp) * dt / (CL_TAU + dt); Q.pre += (c - Q.pre) * dt / (CL_PRE_TAU + dt); }
    } else { Q.pv = 0; if (Q.act && !Q.bad) Q.bad = "no pitch data"; }
    int lever = in.retarder > 0 || in.motor_brake, step = in.retarder > 0 ? (int)in.retarder : 0;
    if (Q.j_start <= 0) Q.j_start = jerk_now();
    // the events
    if (D.on && lever && !Q.p_lever) cl_start(CE_ON, 0, step, D.tgt, in);
    else if (!lever && Q.p_lever) cl_start(CE_OFF, Q.p_step, 0, D.tgt, in);
    else if (D.on && lever && Q.p_lever && step > 0 && Q.p_step > 0 && step != Q.p_step) {
        int n = D.n > 0 ? D.n : 4;
        cl_start(step > Q.p_step ? CE_UP : CE_DOWN, Q.p_step, step, fabsf(decel_target(step, n) - decel_target(Q.p_step, n)), in);
    }
    else if (D.on && D.gap && !Q.p_gap && lever) cl_start(CE_SHIFT, step, step, 0.5f, in);
    Q.p_lever = lever; Q.p_step = step; Q.p_gap = D.gap; Q.p_on = D.on;
    if (!Q.act) return;
    Q.t += dt; Q.tick += dt;
    if (Q.tick >= CL_DT) { Q.tick -= CL_DT; if (Q.nb < CL_N) Q.buf[Q.nb++] = Q.cp; }
    if (!Q.bad) {
        const act_wish *w = act_wishes();
        if (in.paused) Q.bad = "paused";
        else if (in.brake > DC_PEDAL || in.thr > DC_PEDAL) Q.bad = "a pedal";
        else if (in.cruise || in.other > 0) Q.bad = "cruise or another brake owner";
        else if (w[ACT_AEB].brake > 0.02f) Q.bad = "emergency braking";
        else if (!in.engine_on || in.gear < 0) Q.bad = "engine off or reverse";
        else if (D.bad_s > 0.2f) Q.bad = "no usable speed";
        else if (D.a > CL_HARD) Q.bad = "hard braking";
        else if (fabsf(V.grade_deg - Q.grade0) > 1.0f) Q.bad = "the grade changed";
    }
    if (Q.t >= CL_WIN_S) {
        if (fabsf(in.speed) * 3.6f < 8.0f && !Q.bad) Q.bad = "slow at the end";
        cl_finish(0);
    }
}

// ---------------------------------------------------------------- Stop assist ([decel] stop_assist)
// Lifting off at low speed, the box opens the clutch at the start gear's idle point and the truck coasts on in neutral (~0.07 m/s2: it
// rolls at walking pace for a long time). Owner decision 65 (04-11, ride 987 s: "when I lift under 5 km/h it stops it far too hard, I cannot
// roll out; let it roll out in neutral, Auto Hold at a real 0 km/h"): the stop assist ROLLS OUT. It no longer brakes to a taper; it keeps the truck
// from running away and holds it at the standstill:
//   - on the flat and uphill the vehicle model says the truck slows by itself (rolling, air and the grade of now, vehicle_natural_decel in neutral):
//     the request is 0 by construction, no feedback on top of it (the measurement's noise cannot post a whiff of brake). Uphill the grade slows
//     it more, so it stops sooner;
//   - on a descent the grade pushes it (the model's natural deceleration is negative): the service brake gives what keeps the speed from rising,
//     the model's push in m/s2 times the request per m/s2 (the same feedforward the lever assist uses, with its share of the model's gain), trimmed by
//     a loop on the measured deceleration against 0 (neither faster nor slower) within one push either way, so the request is 0 where the push is 0 and
//     continuous in it; the truck then goes on at the speed of the lift-off until a pedal or the lever ends it (it does not stop by itself: "only what is
//     needed to not run away"). A truck that speeds up over half a metre per second in spite of it is left to the driver ("speeding up");
//   - at the standstill (the speed within SA_STOP_V of 0, or, with stop_assist 2 and an Auto Hold that takes it, the first frame it is below 0) stop_assist 1
//     lets go and Auto Hold is kept from taking the brake over, stop_assist 2 posts SA_HAND_REQ, which is what arms Auto Hold (cruise.cpp: below
//     0.5 km/h, a request of ours standing, g_sa_nohold 0), and holds that until Auto Hold's own request has risen (it rises 3/s to its hold level, the
//     grade's: twice the slope's pull plus 0.8 m/s2). Uphill the truck is stopped by the grade, so that is the hand-over that keeps it from rolling
//     back: the roll-back before the hold bites is the speed the grade gives in the frames of the ramp (sim rows, 17 t and 40 t on +2 %, also with the real Auto Hold: 1-2 mm).
// The stop_assist_decel key is not read here any more: the lever's last metres (the retarder curve's floor, WP-BR3b) are finished at it, along the same
// taper, byte for byte as before.
// It shares the controller of the lever assist (the measured deceleration, the feedforward from the brake's gain, the PI on the request) but is a feature of
// its own: it works with [decel] enabled 0, and it leaves D, the lever assist's state, alone.
// Trigger = the event, no km/h figure: the box opened the clutch by itself with both pedals free (no throttle, no brake, no lever, no
// cruise, not reversing, not paused, not crawling, not eco-roll). With [lowspeed] owner=1 that is the owner's own state (g_lv.assist,
// which knows its open point). Without it the assist derives it: the clutch comes open from a gear whose driveline sits at the box's stop
// threshold (idle + 150 rpm; the stock 0.8 idle is lower still), and stays open for longer than a gearbox shift takes (a downshift at
// low revs opens it too, but closes it again within about a second).
// Ends at once on the throttle, the brake pedal, the lever, the cruise, reverse, a pause, another brake owner, or the clutch closing in a
// gear. The request is capped (SA_MAX): where the push is beyond it (a very steep descent) the driver is left to it, once logged.
static const float SA_MAX = 0.3f;        // most request the stop assist asks (~1.8 m/s2 at the nominal brake: a gentle stop never needs more)
static const float SA_OPEN_PAD = 100;   // rpm over idle, on top of the shared stop-open threshold (STOP_OPEN_MARGIN0, core/suite.h): the derived trigger reads the last closed-frame speed, one frame and the measurement noise late (review 30-10)
static const float SA_SHIFT_PAD = 0.4f;  // s over the shift time the clutch has to stay open before it counts as a stop (not a shift)
static const float SA_STOP_V = 0.03f;    // m/s, standing for the stop assist (0.1 km/h: under the 0.5 km/h at which Auto Hold arms, over the speed noise of a truck that stands)
static const float SA_HAND_REQ = 0.03f;  // request kept at a standstill for Auto Hold to take over (what arms it; its own request rises 3/s to its hold level)
// The loop's reach upward, in feedforwards: the weakest brake it is tuned for (sim_decel plants 0.6 x the gain the loop counts on) needs 1 / 0.6 of the nominal request, the feedforward
// carries DC_FF_FIRST of that: (1 / 0.6) / DC_FF_FIRST of the feedforward, one of which is the feedforward itself. Proportional to the feedforward, so it is 0 where the push is.
static const float SA_TRIM_UP = 1.0f / (0.6f * DC_FF_FIRST) - 1.0f;
static const float SA_CAP_S = 1.2f;      // s at the cap and speeding up in spite of it: left to the driver
enum { SA_OFF, SA_ARMED, SA_PEND, SA_RUN, SA_HAND };
static struct {
    int st;                 // SA_OFF = not armed (waits for the clutch to be closed in a gear again)
    float hv[64]; double ht[64], clk; int hn, hi;   // the speed samples (as D's)
    float a, bad_s;         // measured deceleration (m/s2, + = slowing), seconds without a usable one
    int lastg; float lastv; // the gear and speed while the clutch was last closed in a gear
    float t_pend;           // s the clutch has been open (derived trigger)
    float a0;               // the coast's deceleration at the start (grade included)
    float ref, refm, lvl, u, k, ff;   // reference (the total deceleration aimed at: 0, neither faster nor slower), its delayed copy, level (the model's push + the trim), request, request per m/s2, share of the feedforward
    float trim;             // the loop's correction of the model's push (request, within one push either way)
    float push_max;         // the most the model's push was (m/s2), for the log
    float v0, t_run, tgt0, cap_s, hand_s, lvl_max;
    float lit_s;            // s of the run with the brake lights on (the game decides them; the log says whether ours lit them)
    int mode, posted, capped;
} SA;

static void sa_log(const char *why, float v)
{
    if (SA.st != SA_RUN && SA.st != SA_HAND) return;
    logv(2, "Decel: stop assist %d from %.1f km/h, rolls out (coast %.2f m/s2, the grade pushes it by up to %.2f), request up to %.2f; mean %.2f m/s2 over %.1f s (brake lights on %.0f %% of it); ended at %.1f km/h: %s",
         SA.mode, SA.v0 * 3.6f, SA.a0, SA.push_max, SA.lvl_max, SA.t_run > 0 ? (SA.v0 - v) / SA.t_run : 0.0f, SA.t_run,
         SA.t_run > 0 ? 100 * SA.lit_s / SA.t_run : 0.0f, v * 3.6f, why);
}

// To the next state (OFF = disarmed, ARMED = may start at the next opening); our request and the Auto Hold block go with it.
static void sa_to(int st)
{
    if (SA.posted) { act_brake(ACT_DECEL, 0); SA.posted = 0; }
    g_sa_nohold = 0;
    SA.u = SA.lvl = 0; SA.cap_s = SA.hand_s = 0;
    SA.st = st;
}

static void sa_frame(float dt)
{
    int mode = p_sa >= 1.5f ? 2 : p_sa >= 0.5f ? 1 : 0;
    if (mode == 0 || !act_hooked()) { if (SA.st != SA_OFF) sa_to(SA_OFF); return; }
    if (!(dt > 0 && dt < 0.5f)) { if (dt != 0) SA.hn = 0; return; }   // paused or a hitch: the speed history is broken (a request in force stays)
    float v = fabsf(T.speed);
    // the measured deceleration, as the lever assist has it (the speed's fall over the last 0.3 s)
    int ok = 0;
    if (T.speed == T.speed) {
        SA.clk += dt;
        SA.hi = (SA.hi + 1) % 64; SA.hv[SA.hi] = v; SA.ht[SA.hi] = SA.clk;
        if (SA.hn < 64) SA.hn++;
        for (int i = 1; i < SA.hn; i++) {
            int j = (SA.hi - i + 64) % 64;
            float span = (float)(SA.clk - SA.ht[j]);
            if (span >= DC_WIN) { float raw = (SA.hv[j] - v) / span; if (fabsf(raw) < 30.0f) { SA.a = raw; ok = 1; } break; }
        }
    }
    SA.bad_s = ok ? 0 : SA.bad_s + dt;
    const act_wish *w = act_wishes();
    float other = 0;   // another owner's brake (Auto Hold's own is not one: it is the hand-over)
    for (int o = 0; o < ACT_N; o++) if (o != ACT_DECEL && o != ACT_HOLD) other = fmaxf(other, w[o].brake);
    float cl = clutch_seen();
    int closed = cl < DC_CLOSED && T.gear > 0, opened = cl > DC_OPEN || T.gear == 0;
    int manual = T.shifter[0] && strcmp(T.shifter, "automatic") && strcmp(T.shifter, "arcade");
    // the pedals and the rest that must be free
    int free = !(T.in_thr > DC_PEDAL) && !(T.in_brake > DC_PEDAL) && !T.retarder && !T.motor_brake && T.cruise <= 0.1f && !g_lv.crawl_toggle && T.gear >= 0 &&
               T.dgear >= 0 && T.engine_on && !T.paused && !T.parking_brake && !g_rolling && !(other > 0) && !manual && !V.electric && !g_truck_unsupported && !D.on;
    if (closed) {
        SA.lastg = T.gear; SA.lastv = v;
        sa_log("the clutch closed", v);
        sa_to(SA_ARMED);
        return;
    }
    if (SA.st == SA_ARMED || SA.st == SA_PEND) {
        if (!opened) return;                                          // (a shift's clutch between the two thresholds, or no gear to speak of)
        if (!free) { sa_to(SA_OFF); return; }                         // opened with a pedal, the lever, the cruise ...: not a stop of this kind
        if (!(T.speed > 0)) return;
        int go = 0;
        if (low_owner_on()) go = g_lv.assist;                        // the owner knows its open point
        else if (SA.st == SA_ARMED) {
            float idle = ts_idle_rpm(), r = SA.lastg > 0 ? vehicle_rpm_at(SA.lastg, SA.lastv) : 0;
            if (idle > 200 && idle < 2000 && r > 0 && r <= idle + rs_off(idle, STOP_OPEN_MARGIN0) + rs_off(idle, SA_OPEN_PAD)) { SA.st = SA_PEND; SA.t_pend = 0; }
        } else {
            SA.t_pend += dt;
            go = SA.t_pend >= fmaxf(V.eng_ok ? V.eng.shift_time : 1.2f, 0.5f) + SA_SHIFT_PAD;
        }
        if (!go || v < SA_STOP_V) return;
        SA.st = SA_RUN; SA.mode = mode; SA.v0 = v; SA.t_run = 0; SA.cap_s = SA.hand_s = 0; SA.capped = 0; SA.lvl_max = 0; SA.lit_s = 0;
        SA.a0 = fminf(fmaxf(SA.a, -2.0f), 0.5f);
        SA.ref = SA.refm = 0;                 // the total deceleration aimed at: the speed must not rise (the truck rolls out; only a push of the grade is braked)
        SA.k = 1.0f / dc_gain(); SA.ff = vehicle_brake_gain() == vehicle_brake_gain() ? DC_FF_PRIOR : DC_FF_FIRST;   // the model's gain; nearly the full level once its slot is confident
        SA.tgt0 = 0;
        SA.lvl = SA.trim = SA.push_max = 0;
        SA.u = 0;
    }
    if (SA.st != SA_RUN && SA.st != SA_HAND) return;
    // ---- running
    SA.t_run += dt;
    if (T.light_brake) SA.lit_s += dt;
    cruise_info cu;
    cruise_info_get(&cu);
    const char *end = NULL;
    // (a speed below 0 is a standstill too where Auto Hold takes the stop over, stop_assist 2: the truck that the grade stopped was handed over a frame before, one that crossed 0 between
    // two frames still is, and the hand-over below keeps the request standing until Auto Hold's has risen. Without Auto Hold's take it is "rolling backwards", as it always was)
    const int hand = SA.mode == 2 && cu.lever_take > 0;
    if (!free) end = "pedal, lever, cruise or another brake";
    else if (SA.bad_s > DC_BAD_S) end = "no usable speed";
    else if (T.speed < 0 && !hand) end = "rolling backwards";
    else if (v > SA.v0 + 0.5f) end = "speeding up";
    if (end) { sa_log(end, v); sa_to(SA_OFF); return; }
    float cap = fminf(p_max > 0 ? fminf(p_max, 1.0f) : 0.0f, SA_MAX);
    float jerk = jerk_now(), rlim = jerk * SA.k;
    int standing = !(v > SA_STOP_V) || (T.speed < 0 && hand);
    if (SA.st == SA_RUN && standing) {
        if (hand) { SA.st = SA_HAND; SA.hand_s = 0; g_sa_nohold = 0; }   // Auto Hold arms on the request at a standstill
        else {
            static bool told;
            if (SA.mode == 2 && !told) { told = true; logline("Decel: stop_assist 2 needs Auto Hold with [cruise] auto_hold_lever: the stop is released at the standstill"); }
            sa_log("stopped", v); sa_to(SA_OFF); return;
        }
    }
    if (SA.st == SA_HAND) {   // standing: a small request stays until Auto Hold's has risen to it (or to what it will hold), as the lever's stop does
        SA.hand_s += dt;
        if (SA.hand_s >= DC_HAND_S || cu.lever_take <= 0 || w[ACT_HOLD].brake >= fminf(SA_HAND_REQ, cu.lever_take) - 1e-3f) { sa_log("handed to Auto Hold", v); sa_to(SA_OFF); return; }
        act_brake(ACT_DECEL, SA_HAND_REQ); SA.posted = 1;
        return;
    }
    g_sa_nohold = 1;   // Auto Hold arms on the posted request below 0.5 km/h: not yet (stop_assist 2 lets it at the standstill, stop_assist 1 never)
    // What keeps the speed from rising. The model's natural deceleration in neutral is what the truck does by itself (+ = it slows: rolling, air, the grade of now, the learned bias);
    // where that is negative the grade pushes the truck and `push` is the braking that cancels it (m/s2), 0 on the flat and uphill, where the truck rolls out. The feedforward is
    // the push in request (the brake's gain with its share until the slot is confident); the loop on the measured deceleration against 0 trims it, down to nothing and up to
    // SA_TRIM_UP times the feedforward, so a truck whose brake is within the range the loop is tuned for holds its speed, one beyond it speeds up slowly ("speeding up", above)
    // or is held too hard for a moment (the loop takes the excess off at the jerk limit). With no push the request is 0 whatever the measurement says: its noise cannot post a whiff of brake, and the request is
    // continuous in the push (0 at 0, no switch).
    const float push = fmaxf(-vehicle_natural_decel(0, v), 0.0f);
    SA.push_max = fmaxf(SA.push_max, push);
    SA.ref = SA.refm = 0;
    const float ffl = fminf(SA.k * SA.ff * push, cap);             // the model's push, in request
    // (the loop is blind for as long as the measurement needs to show what a change of the level did, from the start: the feedforward alone has to cancel a push the measured deceleration
    // still shows whole -- the speed the truck gained before the assist started -- and a loop that acted on it would brake twice as hard as the push for the length of that delay)
    const float e = SA.t_run < DC_BLIND_S + DC_LAG_S ? 0.0f : -SA.a;   // + = the truck speeds up (the target is 0: neither faster nor slower)
    const int starved = SA.u > 0.02f && act_request() < 0.5f * SA.u;
    float di = fminf(fmaxf(DC_KI * e, -rlim), starved ? 0.0f : rlim) * dt;
    SA.trim = fminf(fmaxf(SA.trim + di, -ffl), SA_TRIM_UP * ffl);
    SA.lvl = fminf(fmaxf(ffl + SA.trim, 0.0f), cap);
    float want = fminf(fmaxf(ffl + fminf(fmaxf(SA.trim + DC_KP * e, -ffl), SA_TRIM_UP * ffl), 0.0f), cap);
    SA.u = fminf(fmaxf(want, SA.u - rlim * dt), SA.u + rlim * dt);
    SA.lvl_max = fmaxf(SA.lvl_max, SA.u);
    // the push beyond the request's cap (a very steep descent): at the cap and speeding up for a while = left to the driver
    if (SA.u >= cap - 1e-3f && SA.a < 0) SA.cap_s += dt; else SA.cap_s = 0;
    if (SA.cap_s > SA_CAP_S) {
        static bool told;
        if (!told) { told = true; logline("Decel: stop assist cannot hold the speed here (request capped at %.2f, the truck speeds up by %.2f m/s2): left to the driver", cap, -SA.a); }
        sa_log("the push is out of reach", v); sa_to(SA_OFF); return;
    }
    act_brake(ACT_DECEL, SA.u); SA.posted = 1;
}

void decel_view(float *target, float *measured, const char **stop_assist)
{
    static const char *const names[] = { "off", "armed", "pending", "running", "handing over" };
    *target = *measured = NAN;
    if (D.on) { *target = D.tgt + D.nat; *measured = D.a; }   // (whole decelerations, the road's own in both)
    else if (SA.st == SA_RUN || SA.st == SA_HAND) { *target = SA.ref; *measured = SA.a; }
    *stop_assist = SA.st >= 0 && SA.st <= SA_HAND ? names[SA.st] : "off";
}

// The term that holds the request now: the stop assist, the engine-brake bridge, or the loop's phase.
static int de_term_now()
{
    if (SA.st == SA_RUN && SA.u > 0) return DT_STOP;   // (the stop assist names itself only while it holds a request: rolling out on the flat it holds none)
    if (SA.st == SA_HAND) return DT_HAND;
    if (E.u > 0 && E.u >= (D.on ? D.u : 0.0f)) return DT_EB_BRIDGE;
    if (D.on && D.u > 0) return D.term;
    return DT_NONE;
}
// A line when the term that holds the request changes (not more often than every 0.3 s: a change that does not stand is noise at the frame's rate; it is logged at the next frame it stands).
static int s_dterm = DT_NONE;
static double s_dterm_at = -1e9;
static void de_note_term(int term, float req, float tgt, float meas)
{
    const double now = wall_s();
    if (term == s_dterm || (now >= s_dterm_at && now - s_dterm_at < 0.3)) return;   // (a clock that went back -- a new session in a sim -- does not hold a line back)
    if (term == DT_NONE) logv(1, "Decel: request ended (was held by: %s)", DT_TEXT[s_dterm]);
    else logv(1, "Decel: request %.2f held by: %s (target %.2f m/s2, measured %.2f, %.0f km/h)", req, DT_TEXT[term], tgt, meas, fabsf(T.speed) * 3.6f);
    s_dterm = term; s_dterm_at = now;
}

static void de_frame()
{
    static double last;
    double now = sim_s();
    decel_in in;
    in.dt = (float)(now - last);
    last = now;
    sa_frame(in.dt);   // the stop assist: its own feature, with or without the lever assist
    if (p_enabled <= 0 || !act_hooked()) {
        if (D.on) decel_off();
        soft_off(); gear_stand_clear(GSO_DECEL); E.have = 0; E.gap = 0; E.lvl = E.u = 0; E.posted = 0;
        de_note_term(de_term_now(), SA.u, SA.tgt0, SA.a);   // (the stop assist runs without the lever assist)
        return;
    }
    const act_wish *w = act_wishes();
    in.speed = T.speed; in.thr = T.in_thr; in.brake = T.in_brake; in.clutch = clutch_seen();   // our own clutch (crawl, smooth start) counts
    in.gear = T.gear; in.retarder = T.retarder; in.steps = (int)T.retarder_steps; in.motor_brake = T.motor_brake;
    in.engine_on = T.engine_on; in.paused = T.paused; in.cruise = T.cruise > 0.1f;
    in.other = fmaxf(fmaxf(fmaxf(w[ACT_CRUISE].brake, w[ACT_ACC].brake), w[ACT_BEND].brake), w[ACT_CHAN].brake);   // (the brake channel's service brake is the cruise side's too)
    in.held = act_request();
    cruise_info cu;
    cruise_info_get(&cu);
    in.hold = w[ACT_HOLD].brake; in.take = cu.lever_take;
    in.soften = p_soften > 0 && act_retarder_hooked();
    in.ramp = p_ramp > 0 && act_retarder_hooked();
    in.pre = p_pre > 0 && !(p_force > 0 && T.retarder > 0 && vehicle_tc());   // (a converter box does not interrupt the braking at a shift: nothing to hand over)
    in.sramp = p_sramp > 0 && act_retarder_hooked();
    in.bridge = p_sramp > 0;
    // (the shift is due when the speed has fallen to the point of the braking rule at the deceleration the truck WILL have: with the lever's retarder in a gap or just back from one, the measured one is the gap's)
    in.eta = in.pre && D.on ? ts_brake_down_eta(fmaxf(D.a, D.fm ? D.ref + D.nat : 0.0f)) : NAN;
    in.ebb = p_ebb > 0;
    in.eb_stage = 0; in.eb_share = 0.5f; in.eb_cap = 0;
    const float depth = fminf(fmaxf(p_fade == p_fade ? p_fade : 0.0f, 0.0f), 1.0f);   // retarder_fade: 0 = the step's whole target, 1 = the curve, blended continuously between
    in.ret_shape = depth > 0 && in.soften && T.retarder > 0 ? 1.0f - depth * (1.0f - vehicle_ret_shape(fabsf(T.speed))) : 1.0f;   // (the curve needs a retarder step held and the factor on the game's retarder)
    // step_force: the lever is the game's own retarder at its step (the factor we post is the fade alone, open loop); needs the mass for the road's own deceleration the reference is taken free of.
    in.fmode = p_force > 0 && p_force < 1.5f && T.retarder > 0 && V.mass_ok && V.mass > 0;
    in.law = 0; in.lw_tgt = in.lw_rung = in.lw_whole = in.lw_ret = in.lw_eb = 0;
    const bool law_on = p_force >= 1.5f && (T.retarder > 0 || T.motor_brake) && V.mass_ok && V.mass > 0;
    if ((in.ebb && T.motor_brake) || law_on) {
        char *ga = sense_actor();
        int held, on, flag; unsigned stage, cap = ga ? sense_ebrake_cap(ga) : 0;
        if (ga && cap >= 1 && cap <= 16) in.eb_cap = (int)cap;   // (the stage count is known even when the state is not: the bridge's guess counts the middle stage then)
        if (ga && cap >= 1 && cap <= 16 && sense_ebrake_state(ga, &held, &on, &stage, &flag) && on && stage >= 1 && stage <= cap) { in.eb_stage = (int)stage; in.eb_share = (float)stage / (float)cap; }
    }
    if (law_on) {   // the lever as a share of what the truck can brake now (header, step_force 2)
        const float v = fabsf(T.speed), m = V.mass;
        const int kr = T.retarder > 0 ? (int)T.retarder : 0, nr = (int)T.retarder_steps > kr ? (int)T.retarder_steps : kr;
        const int ce = in.eb_cap;
        // the share of the lever: the retarder's step over its step count; the engine-brake lever's stage over its stage count (the first when the stage is not readable); the stronger of the two for the engine brake,
        // which the game adds to the retarder lever by itself
        const float fr = kr > 0 && nr > 0 ? (float)kr / (float)nr : 0.0f;
        const float fe = T.motor_brake && ce > 0 ? (float)(in.eb_stage > 0 ? in.eb_stage : 1) / (float)ce : 0.0f;
        const float f = fmaxf(fr, fe), f1 = kr > 0 && nr > 0 ? 1.0f / (float)nr : ce > 0 ? 1.0f / (float)ce : 0.0f;
        if ((kr > 0 && nr > 0) || (ce > 0 && fe > 0)) {
            // the gear the engine brake works through: a shift's gap keeps the gear of before it (the target does not dip with the clutch)
            const int g = D.gap && D.g0 > 0 ? D.g0 : in.gear;
            const float rpm = g > 0 ? vehicle_engine_rpm_at(g, v) : 0.0f;
            const float eb_top = ce > 0 && g > 0 ? vehicle_eb_decel_at(ce, ce, g, rpm) : 0.0f, eb_top_whole = ce > 0 && g > 0 ? vehicle_eb_decel(ce, ce, g) : 0.0f;
            float ret = 0, ret1 = 0, ret_whole = 0;
            if (kr > 0 && nr > 0) {
                const float F = vehicle_ret_force(kr), F1 = vehicle_ret_force(1), Ft = vehicle_ret_force_full();
                const float fk = Ft > 0 ? F / Ft : fr, f1k = Ft > 0 ? F1 / Ft : f1;   // (the stage's force as a share of the top's: the knee of its curve)
                ret = F / m * vehicle_ret_shape_stage(v, fk); ret1 = F1 / m * vehicle_ret_shape_stage(v, f1k); ret_whole = F / m;
            }
            in.law = 1;
            in.lw_ret = ret; in.lw_eb = f * eb_top;
            in.lw_tgt = ret + f * eb_top;
            in.lw_rung = ret1 + f1 * eb_top;
            in.lw_whole = ret_whole + f * eb_top_whole;
        }
    }
    bool was = D.on, soft_was = D.kr < 1;
    D.term = DT_NONE;
    decel_tick(in);
    eb_bridge(in);
    if (E.u > 0 || E.posted) {   // the bridge's request: the bigger of it and the lever assist's (one owner slot), not while the stop assist has the brake
        if (D.on) act_brake(ACT_DECEL, fmaxf(D.u, E.u));
        else if (SA.st != SA_RUN && SA.st != SA_HAND) act_brake(ACT_DECEL, E.u);
    }
    E.posted = E.u > 0;
    cl_frame(in);
    if (Q.dirty_save && wall_s() - Q.save_at > 30) cl_save("periodic");
    // downshift_assist: the service brake the lever needs besides the retarder, as the gearbox sees it (the level of before a gap while
    // one runs, without what a hand-over ahead of a shift has added)
    float need = 0;
    if (p_dsa > 0 && D.on && !D.fm && D.on_s > DC_NEED_S) { float lv = D.gap || D.rt >= 0 ? D.pre : fmaxf(D.lvl - D.k * D.ph, 0); need = lv > DC_NEED ? lv : 0; }
    {   // the gearbox is asked for more engine drag, in N at the wheels (the service brake's share in m/s2 times the mass): the arbiter's standing request, a lever the driver holds, asked as it is
        if (need > 0) { const gear_stand gs = { need * dc_gain() * (V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK), GSK_LEVER, 0, 0.0 }; gear_stand_post(GSO_DECEL, &gs); }
        else gear_stand_clear(GSO_DECEL);
    }
    de_note_term(de_term_now(), D.on ? D.u : E.u > 0 ? E.u : SA.u, D.tgt, D.aa);   // (a line when the term that holds the request changes; target and measured are what the brakes give)
    if (D.on != was) logv(1, "Decel: %s (step %d of %d, target %.2f m/s2, %.0f km/h)", D.on ? "on" : "off", D.step, D.n, D.tgt, fabsf(T.speed) * 3.6f);
    if (D.on && !was && D.fm) logv(2, "Decel: lever step %d of %d is the game's own retarder: the factor is the fade, %.2f at %.0f km/h; the road's own %+.2f m/s2; the service brake bridges the shifts only", D.step, D.n, in.ret_shape, fabsf(T.speed) * 3.6f, D.nat);
    if (D.on && !was && D.law) logv(2, "Decel: lever step %d of %d is a share of what the truck can brake now: %.2f m/s2 (retarder %.2f + engine brake %.2f; the lowest rung %.2f); the hold of the pull's %.0f km/h needs %.2f; the road's own %+.2f m/s2", D.step, D.n, in.lw_tgt, in.lw_ret, in.lw_eb, in.lw_rung, fabsf(T.speed) * 3.6f, D.hold, D.nat);
    if (D.on && !was && in.ret_shape < 1.0f) logv(2, "Decel: retarder curve: knee %.0f km/h (%.0f kW over the top stage's %.1f kN), %.0f %% of the step's %.2f m/s2 at %.0f km/h", vehicle_ret_knee_ms() * 3.6f,
                                                      V.eng.torque * V.peak_pw * 0.10471976f / 1000.0f, vehicle_ret_force_full() / 1000.0f, 100.0f * in.ret_shape, D.fm ? D.ref : decel_target(D.step, D.n), fabsf(T.speed) * 3.6f);
    if ((D.kr < 1) != soft_was) logv(1, "Decel: retarder %s (step %d of %d, target %.2f m/s2, measured %.2f, %.0f km/h)", D.kr < 1 ? "softened" : "back to the game's own", D.step, D.n, D.tgt, D.aa, fabsf(T.speed) * 3.6f);
}

// Continuity broke (pause, hitch, other truck, a new load): no state carries over. The learned request per m/s2 depends on the load.
static void de_reset(int level, const char *)
{
    D.on = 0; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.bad_s = 0; D.hn = 0; D.a = D.aa = 0; D.y = D.ya = 0; D.ps = 0; D.fm = 0; D.law = 0; D.nat = 0; D.nat_ok = 0;
    Q.act = 0; Q.pv = 0; Q.p_lever = Q.p_gap = Q.p_on = 0;
    E.have = 0; E.gap = 0; E.lvl = E.u = 0; E.posted = 0; if (level >= RESET_LOAD) E.dfc0 = 0;
    sa_to(SA_OFF); SA.hn = 0; SA.a = 0;
    if (level >= RESET_LOAD) { D.k_known = D.k_fix = D.k_prior = 0; D.k = 1.0f / dc_gain(); }
    if (level >= RESET_HARD) D.drop_ok = 0;   // (another truck, another box)
    act_brake(ACT_DECEL, 0);
    soft_off();
    gear_stand_clear(GSO_DECEL);
}

static char g_status[240] = "off (ini)";
static const char *de_status() { return g_status; }

static void de_start()
{
    int sa = p_sa >= 1.5f ? 2 : p_sa >= 0.5f ? 1 : 0;
    if (p_enabled <= 0) {
        if (sa) snprintf(g_status, sizeof g_status, "lever assist off; stop assist %d: rolls out%s", sa, act_hooked() ? "" : " (no actuator hook)");
        return;
    }
    p_max = p_max > 0 ? fminf(p_max, 1.0f) : 0;   // a request is 0..1 (and a NaN from the ini is none)
    if (!(p_jerk >= 0.1f)) p_jerk = 0.1f;         // the slews divide and multiply by it
    snprintf(g_status, sizeof g_status, "on: %.1f-%.1f m/s2, jerk %.1f m/s3, max brake %.2f%s%s%s%s%s", p_tmin, p_tmax, p_jerk, p_max,
             p_soften > 0 && g_retarder_hook >= 0.5f ? ", soften" : "", p_ramp > 0 && g_retarder_hook >= 0.5f ? ", ramp" : "",
             p_pre > 0 ? ", pre-blend" : "", p_dsa > 0 ? ", downshift assist" : "", act_hooked() ? "" : " (no actuator hook)");
    if (sa) { size_t l = strlen(g_status); snprintf(g_status + l, sizeof g_status - l, "; stop assist %d: rolls out", sa); }
    static bool told;
    if (p_soften > 0 && g_retarder_hook < 0.5f && !told) { told = true; logline("Decel: soften is on but [core] retarder_hook is 0: the game's retarder stays as it is, the service brake only adds"); }
    static bool told_ramp;
    if (p_ramp > 0 && g_retarder_hook < 0.5f && !told_ramp) { told_ramp = true; logline("Decel: ramp_s is on but [core] retarder_hook is 0: after a shift the retarder comes back as the game has it, the brake goes back at once"); }
}

static void de_stop() { act_brake(ACT_DECEL, 0); E.lvl = E.u = 0; soft_off(); gear_stand_clear(GSO_DECEL); sa_to(SA_OFF); if (Q.act) cl_finish(1); cl_summary(); cl_save("stop"); }

module m_decel = { "decel", "Decel", g_params, sizeof g_params / sizeof g_params[0], de_start, de_frame, NULL, de_stop, de_reset, de_status, NULL, VCM_TRUCK };
