// What the two gear brains (modules/trueshift.cpp for trucks, modules/carshift.cpp for cars) do exactly the same way, so it lives once:
// the reasons a gear was chosen, the dashboard letter, the incident watches, the idle learner, the start gear by physics and the stop threshold.
// The state stays in the brains; the functions here take what they need and keep only their own timers.
#pragma once
#include "suite.h"
#include "vehicle.h"
#include "shiftmodel.h"

// Why a gear was chosen (decide_gear and the stubs): one value the logic compares (it used to strcmp the text), the text only for
// the log, the trace and the dev display. One list keeps the two together.
#define WHY_LIST(X) \
    X(W_NONE, "") \
    X(W_GAME, "game") \
    X(W_HINT, "hint") \
    X(W_START, "start") \
    X(W_START_LOWERED, "start, lowered") \
    X(W_START_CORRECTED, "start, corrected") \
    X(W_LAUNCH_SLIP, "launch slip (hold)") \
    X(W_ENGAGE, "engage") \
    X(W_ECO_ROLL, "eco-roll") \
    X(W_ROLL_END, "roll end") \
    X(W_NO_DATA, "no data") \
    X(W_NEUTRAL, "neutral") \
    X(W_LOW_SPEED, "low speed") \
    X(W_WRONG_GEAR, "wrong gear") \
    X(W_REV_LIMIT, "rev limit") \
    X(W_REV_LIMIT_SKIP, "rev limit skip") \
    X(W_STALL, "stall") \
    X(W_KICKDOWN, "kickdown") \
    X(W_HINT_HOLD, "hint hold") \
    X(W_ENGINE_BRAKE, "engine brake") \
    X(W_ENGINE_BRAKE_LEVEL, "engine brake (level)") \
    X(W_ENGINE_BRAKE_HOLD, "engine brake hold") \
    X(W_LEVER_UP, "lever band (up)") \
    X(W_BRAKING, "braking") \
    X(W_BRAKING_JUNCTION, "braking (junction ahead)") \
    X(W_BRAKING_HOLD, "braking hold") \
    X(W_MIN_GAP, "min gap") \
    X(W_NO_PULL, "no pull") \
    X(W_CLIMB_AHEAD, "climb ahead") \
    X(W_LOW_SPEED_HOLD, "low-speed hold") \
    X(W_ANTI_HUNT_DOWN, "anti-hunt (down)") \
    X(W_TOP_AHEAD, "top ahead") \
    X(W_TOP_REACHABLE, "top reachable") \
    X(W_OVER_TOP, "over the top") \
    X(W_CRUISE_BAND, "cruise band") \
    X(W_LUGGING, "lugging") \
    X(W_DOWN_SKIP, "down skip") \
    X(W_DOWN, "down") \
    X(W_AIR_CHARGE, "air charge") \
    X(W_JUNCTION_AHEAD, "junction ahead") \
    X(W_SKIP_WAIT, "skip wait") \
    X(W_UP_EARLY_SKIP, "up early skip") \
    X(W_UP_EARLY, "up early") \
    X(W_POWER_HOLDS, "power holds") \
    X(W_UP_VETO, "up veto (slowing)") \
    X(W_ANTI_HUNT_UP, "anti-hunt (up)") \
    X(W_UP_SKIP, "up skip") \
    X(W_UP, "up") \
    X(W_UP_BLOCKED, "up blocked") \
    X(W_HOLD, "hold") \
    X(W_M, "M") \
    X(W_M_REV_LIMIT, "M: rev limit") \
    X(W_M_STALL, "M: stall") \
    X(W_M_KICKDOWN, "M: kickdown") \
    X(W_M_STOPPING, "M: stopping") \
    X(W_M_LUGGING, "M: lugging") \
    X(W_UP_WAIT, "up wait") \
    X(W_LIFT_UP, "up on lift") \
    X(W_WISH_UP, "wish unmet above") \
    X(W_ASSIST_DOWN, "braking assist") \
    X(W_LAYERS, "layers") \
    X(W_REV_LIMIT_HOLD, "rev limit, no gear above holds") \
    /* end */
enum why_e {
#define X(id, text) id,
    WHY_LIST(X)
#undef X
    W_COUNT
};
extern const char *const why_text[W_COUNT];

// Hunting and fighting the driver: three shifts in a row that each reverse the direction within 10 s with about the same demand (none of them the
// driver's) is a hunt; a hint against our last shift within 5 s is an override. Both go to the incident recorder. ours = the brain has the gearbox
// (below its takeover speed the game's own automatic shifts); dm_last = the demand at this shift.
void sc_track_shifts(int ours, int prev, int g, int hint, double now, float dm_last, const char *mode_label);

// How much of the price of reversing the last shift is still due (WP-G8, owner decisions 63 + 64): 1 = the situation the shift was made in is still there (a hunt: the same pedal on the
// same road), the whole price; 0 = it is gone (he lifted, or the braking is over and the pedal pulls calmly), nothing. dm_then = the demand share at the shift (shift_common_state::dm_last),
// braked_then = the shift was a braking one (the engine brake, the braking rules, the deceleration assist), dm_now = the demand share now (0 with the pedal off), braking_now = 0..1 how much it
// is braking now. Pure and continuous in the three analog inputs; the derivation is in shiftcommon.cpp.
float sc_reversal_weight(float dm_then, float dm_now, int braked_then, float braking_now);

// The dashboard letter (the mode, also stopped and in N; A in reverse). enabled = the ini switch, on = the brain drives this gearbox; mode_update
// brings the brain's mode up to date, letter() gives its letter. *letter_state: -1 unknown, 0 not found, 1 written.
void sc_dash_letter(int enabled, int on, void *self, int *letter_state, void (*mode_update)(), char (*letter)());

// Incident watch: the rpm limiter hit while the brain drives (driving_ok = the brain has the gearbox and is not in M), or the engine dragged
// below idle while moving.
void sc_watch_engine(int driving_ok, const char *mode_label, float idle);
// M: the engine over its limit (the wheels push it): a warning tone, no shift. m_on = the brain has the gearbox and the mode is M.
void sc_overspeed_tone(int m_on);
// The idle rpm: standing, engine on for 5 s, pedal and throttle released, rpm steady for 1 s. Starts from p_idle, never from one sample. A car
// (store) starts with the idle of its last drive and stores what it learned.
void sc_idle_learn(float *idle, float p_idle, int store);

// The start gear by physics: lock speed and slip time at this mass and grade, never by a gear count. stock = the game's pick; the result may be
// above it (launch_max) or below it. lcx = launch control (a truck's).
// slip_rpm > 0: the game's clutch of 1.62 (vehicle_slip_rpm(): the engine held there while the clutch slips), picked by sc_launch_pick_slip: the foot the launch is picked for (the pedal, or the driver's
// usual launch pedal while it is up), the shaped share of it, the gap of a shift (s) and the speed it loses beyond the resistance (m/s2); 0 = the plugin's own clutch of the older formula (the car brain).
struct sc_launch { int takeover; float start_fix, launch_max, slip_s, launch_torque, launch_rpm, kick_pedal; int mode; int lcx; float slip_rpm, foot, share, gap_s, decel; };
int sc_launch_pick(const sc_launch &c, int stock, float thr, bool force);
// The automatic's stop threshold: below it the clutch opens / neutral. Stock 0.8 x idle (+ a brake term the game adds), which pushes until ~2 km/h.
// While braking to a stop without throttle (below 15 km/h): idle + 150 rpm; the game's own brake term comes on top. off = the brain is out of it.
float sc_stop_threshold(float p_stop, float p_no_creep, int crawl, int off, float idle, float thr, float brake);

// The state both gear brains keep the same way (each brain's S derives from it and adds its own).
struct shift_common_state {
    int eng_seen;           // the engine line is logged for the engine the vehicle model (V.eng) has now ...
    float eng_torque, eng_limit;   // ... which had these values
    float thr;              // throttle that counts: pedal, or damped cruise control
    float cc_thr;           // cruise control throttle, smoothed
    float demand;           // smoothed demand 0..1
    double demand_at;
    float f_need;           // tractive force asked for (N), up fast, down slow
    float dm_last;          // the demand share (dm_share) at the last shift
    float pedal_prev;
    double pedal_at;
    double kick_until;
    int mode;
    int ours;               // above the takeover speed (with hysteresis)
    int old_ours;           // TrueShift: the hand-over rule of before WP-TC1 (M mode's ours; where the owner's hand-off starts the dwell timers)
    double ours_at;
    int ours_launch;        // taken over while pulling away (below 25 km/h), not after a pause or a teleport at speed
    int last_gear;
    double shift_at;
    int shift_dir;
    int commanded;          // last gear we returned to the game
    int cmd_why;            // why_e: why that gear (the logic compares it)
    double hold_until;
    float grade_hist[8];    // grade (%) once a second, for "a climb starts"
    double grade_at;
    const char *why;        // reason of the last decision, the text, for the trace
    int want, target;
    double blocked_log_at;
    double brake_at;        // the last time the brake was on (service brake over 0.05)
    double own_brake_at;    // the last time the suite itself held a brake request (act_request; choice_own_brake)
    float svc;              // the driver's service brake as the deceleration it makes still sees it (TrueShift refresh, SVC_TAIL_LAG_S): follows the pedal up at once and down over the lag
    double kd_at;           // the last kickdown or lugging downshift of ours
    int letter_state;       // -1 unknown, 0 not found, 1 written
    float idle;             // learned idle rpm (0 = not yet)
    int rolling;            // eco-roll: in neutral by our choice
    double roll_at;
    int roll_exit;          // gear to engage after a roll (kept as target until it is in)
    int roll_from;          // the gear the roll started from
    double roll_exit_at;
    double pedal_on_at;     // last time the pedal was pressed
    float drv_v;            // the speed then (m/s): what the driver last held with the pedal
    double skip_log_at;     // last 'single up, no skip' log line
    double calm_since;      // Power: since when the driving is flat, light and steady
    float calm_k;           // Power: 0 = full Power shift points, 1 = like Normal (after a calm stretch)
    const char *unsupported; // why this truck is left to the game's own automatic (NULL = fine)
    float rpm_odd_s;        // seconds the rpm did not follow the speed model at all (torque converter?)
    float tc_slip_s;        // seconds of converter slip seen (the detector's accumulator)
    int manual_logged;
    float pk;               // decide_gear: the steepest grade (%) of the last seconds, fading 0.15 %/s (the climb the upshift reckons with)
    double pk_at;           // when it was last updated
    double gap_since;       // tail_pre: parked in the gearbox's gap state (gear 0, a target, clutch open) while moving since; 0 = not
    int seed_demand, seed_force;   // after a hard reset: demand / f_need start from the pedal of now, not from 0 (one update each)
    force_learner fl;       // the learned force correction (V.f_scale x engine force + V.f_bias), a 2-parameter least squares (core/shiftmodel.cpp)
    double view_at;         // wall_s() of the last decide update (the view is stale 0.5 s later: another gearbox, a manual box)
    int view_gear, view_state;   // the gearbox's gear and state in that update
};
