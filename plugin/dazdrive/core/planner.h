// Longitudinal planner core (notes/planner/plan-draft.md, phase "planner core"): ONE receding-horizon optimisation over distance that
// replaces the economy/comfort blocks (PCC momentum/crest/descent margin, eco-roll decision, bend/junction coast plans, own ACC following).
// A PURE FUNCTION: plain structs in, a plain struct out, no game, no telemetry, no globals. The caller owns the scratch (PlWork, ~1 MB,
// create it once) so a call never allocates. Not wired into the game or the cruise module yet (a later phase). Tests: tools/sim/run_planner.sh.
//
// Algorithm (plan-research.md section 3, adapted by ds-review.md):
//   stages     s_0 = 0 .. s_K over the horizon (default 2 km): 50 m steps; 12.5 m steps in a cap's braking run-up (where the backward
//              envelope is under the local cap) and while braking down from a late start; a stage edge exactly where a hard cap starts.
//              So brake onset is quantised to 12.5 m, not 50. Caps beyond the horizon come in through the backward envelope over the whole
//              profile (up to 6.4 km, planned decel brake_plan).
//   state      kinetic energy per kg e = v^2/2 on PL_NODES nodes per stage, uniform in e between lo_k and hi_k, one node exactly on the
//              throttle cap (the set speed is held exactly) and the top node exactly on hi_k (a cap is met exactly). hi_k = min(backward
//              envelope, set + over (+ schwung), limit + 5, forward reachability), relaxed by a max-braking curve when the start is late;
//              lo_k = set - under, relaxed by physics (what full traction reaches), a gentle approach envelope to caps and a lead.
//   labels     every node keeps up to PL_LABELS Pareto labels (cost, arrival time, overspeed bucket, ratchet speed, action class): a label
//              drops another only if it is cheaper (switch handicap included) AND arrives no earlier (with a lead) AND used no more overspeed
//              time AND is no more constrained. The fix for ds-review's "one label per node drops the only feasible path under a tight
//              headway" (test 13); it also makes the overspeed time cap exact along every kept path.
//   actions    in gear by inverse dynamics to every target node (implicit trapezoid, consistent with the forward step): traction / part
//              throttle (fuel at the predicted gear), braking (engine drag, retarder levels, service brake; firm only: no dribbling brake on
//              top of a decelerating coast); coast in gear (fuel cut) by a forward step; NEUTRAL: entering coasts exactly through
//              neutral_min_s and lands in neutral, a label in neutral continues stage by stage or engages (exit penalty). Every snap to a node
//              charges the energy difference at gamma.
//   no cycling NO pulse-and-glide (owner 30-10; option pulse_glide, Eco only, to test it). Rolling (coast in gear, neutral) or braking under
//              the throttle cap needs a REASON: a descent the truck can roll to from the cap without falling under the band (the crest's
//              last stretch, within coast_reason_s), the segment itself downhill, a cap whose gentle envelope (coast_relax x roll_scale) is
//              under the throttle cap in this stage, or (per label) a lead: inside the wanted gap, or faster than it with the gap within the
//              roll-in's closing (lead_roll_decel x roll_scale). Without a reason part throttle must get back up to the cap at accel_comf
//              (or pull at full load), may not fall under the highest speed since the last reason (the ratchet), and while following may
//              not pull above the lead's speed. Light throttle (< 25 % load) counts as rolling for the switch/re-pull penalties.
//   gear       predicted by pl_gear_pred(): the highest gear in the rpm window that gives the force with a reserve, else the strongest
//              (tables per truck, cached in PlWork). A STAND-IN for TrueShift (stateful); the integration must measure the disagreement.
//   cost       fuel + beta*time + eps*|dv| + w_under*time*(set-v)+^2 + switch/re-pull/shift/neutral penalties + brake/harsh/over-cap
//              penalties + lead terms - gamma*m*min(e, e_cap) at the end. beta is calibrated per call so the set speed is the steady optimum
//              on the level (numeric derivative of the fuel model at a fixed gear) times a per-mode scale; gamma = the model's fuel per
//              wheel joule. All gram weights are REFERENCE grams, scaled by u = gamma / 6.8e-5 (the ETS2 fit is in game-scaled mL, u ~ 10).
//   fuel       callback (rpm, brake power, neutral): built in the ETS2 fit (fuel_model.hpp: torque curve x g(throttle) x omega, overrun 0,
//              neutral idle) and a Willans alternative; engine drag in gear (rpm-dependent), none in neutral.
//   lead       the wanted gap from the lead's speed (d0 + tau*v_lead), sliding to our speed with tension (lead decel 0.3 -> 1.5 m/s2, TTC
//              6 -> 3 s). Checked at both ends and the middle of every transition against a prediction (speed + measured accel decaying over
//              lead_acc_tau, clipped by the route envelope; a predicted stop becomes a stop point d0 behind it). A violation costs per metre
//              (soft but dominant: the least-bad plan when the start is already too close); a cushion above the wanted gap (never coast into
//              exactly that gap); a far penalty above the per-mode band; the violation weight fades after lead_fade_s (confidence).
//   caps       throttle only up to min(set, limit + 5); overspeed only from gravity, up to min(set + over, limit + 5); above set + over
//              (Eco's schwung) only in the first schwung_s, above set for at most over_time_s (both minus over_reserve_s), counted in a
//              bucket that refills at over_recover s per s at or under set. Bends/junctions/stops are hard at every profile point.
//   baseline   per replan also a plain cruise over the same stages (hold min(set, limit + 5) in gear, brake only for the hard caps) with
//              the same cost terms: base_cost / base_fuel_g / base_time_s, for a "fuel saved" figure (the terminal term is in both costs).
//              The baseline ignores the plan's commitments (a running roll, the bucket), so a single replan can show it cheaper.
#pragma once
#include <stdint.h>

#define PL_MAX_STAGES   96     // stages per plan (the 2 km horizon in 50 m steps is 40; fine steps near caps add)
#define PL_NODES        48     // energy nodes per stage
#define PL_LABELS       4      // Pareto labels per node (1 = the plain single-label DP)
#define PL_PROF_MAX     640    // route profile points (10 m spacing -> 6.4 km)
#define PL_GEARS        16
#define PL_CURVE        16     // torque curve points
#define PL_RET_LEVELS   6      // retarder levels (1..n)
#define PL_LEAD_STEPS   480    // lead prediction samples (0.25 s -> 120 s)

enum { PL_ECO = 0, PL_NORMAL = 1, PL_POWER = 2, PL_MANUAL = 3 };                 // drive modes (M = Power, never neutral)
enum { PL_GAS = 0, PL_COAST = 1, PL_BRAKE = 2, PL_NEUTRAL = 3, PL_ANY = 4 };     // action classes (PL_ANY: just engaged after neutral)
enum { PL_WHY_CRUISE = 0, PL_WHY_CAP, PL_WHY_LEAD, PL_WHY_CREST, PL_WHY_CLIMB, PL_WHY_DESCENT, PL_WHY_LIMIT, PL_WHY_RECOVER, PL_WHY_ECO };

// fuel model: grams per second at the engine's rpm and BRAKE power p (W, crank output; negative = the wheels drive the engine), or idling
// in neutral (neutral = 1, rpm = idle). The model carries its own friction / part-load share.
typedef float (*PlFuelFn)(const void *ctx, float rpm, float p_w, int neutral);

// built-in 1, the ETS2 fit (notes/planner/fuelfit.md, fuel_model.hpp, 30-10 ride logs): fuel = a * Tpk * curve(rpm) * g(thr) * omega with
// thr = p / (omega * Tpk * curve(rpm)) (the effective throttle ~ the brake torque share) and g concave (0 below 0.02, ~0.14 + 0.86 thr above 0.25); overrun exactly 0; neutral at idle:
// 0.062 * a * Tpk * omega_idle. a in mL/(rad Nm) in the high map zone (outside cities; x zone_scale ~0.162 in cities).
struct PlEts2Fuel { float a_ml_per_rad_nm; float tpk; float idle_rpm; float zone_scale; float g_per_ml;
                    int ncurve; float curve_rpm[16], curve_share[16]; };
float pl_fuel_ets2(const void *ctx, float rpm, float p_ind_w, int neutral);
// built-in 2, Willans (the robustness alternative): fuel = a * (p + T_fric * omega) (>= 0), neutral a * T_fric * omega_idle
struct PlWillans { float a_g_per_j; float fric_nm; float idle_rpm; };
float pl_fuel_willans(const void *ctx, float rpm, float p_ind_w, int neutral);

struct PlVehicle {
    float mass;                     // kg
    float crr;                      // rolling resistance coefficient
    float cda;                      // drag area m2
    float rho;                      // air density kg/m3 (1.2)
    int   ngear;                    // forward gears, ratio[0] = lowest
    float ratio[PL_GEARS];
    float final_drive, wheel_r;     // diff ratio, wheel radius m
    float eta;                      // driveline efficiency (0.9)
    float idle_rpm, max_rpm;        // the engine's idle and the top of the usable range
    int   ncurve;                   // full-load torque curve: rpm -> Nm, ascending rpm
    float curve_rpm[PL_CURVE], curve_nm[PL_CURVE];
    float drag_lo_nm, drag_lo_rpm;  // engine drag torque at the crank with the fuel cut, linear in rpm between the two points (ETS2 fit:
    float drag_hi_nm, drag_hi_rpm;  // ~0.08 Tpk at 1000 rpm, 0.12 at 1430); none in neutral (the fit: in-gear coasting ~25-35 % more decel)
    int   ret_levels;               // retarder: force (N) at the wheels per level 1..n at or above ret_v_full, linear below
    float ret_force[PL_RET_LEVELS];
    float ret_v_full;               // m/s
    float brake_decel_max;          // service brake, m/s2 at full pedal
    float gear_rpm_lo, gear_rpm_hi; // gear prediction window (the stand-in for TrueShift)
    float gear_reserve;             // share of the full-load force a predicted gear may be asked (0.9)
    PlFuelFn fuel;                  // fuel model; 0 = pl_fuel_ets2 with &ets2
    const void *fuel_ctx;           // 0 = &ets2 (or &willans when fuel == pl_fuel_willans)
    PlEts2Fuel ets2;                // the built-in models' numbers
    PlWillans willans;
};

// per drive mode: bands, permissions, weights (pl_mode_defaults() fills the decided table; every number is a sweepable parameter)
struct PlModeParams {
    float under_kmh;        // planned slowdown below set (crests, climbs, P&G)
    float over_kmh;         // gravity-only overspeed above set
    float schwung_kmh;      // extra above set+over, only in the first schwung_s of an excursion
    float schwung_s;
    float over_time_s;      // time above set per excursion (bucket)
    int   neutral;          // neutral allowed (only with a reason: no pulse-and-glide, see the header)
    float neutral_pen_g;    // entering + leaving neutral (half each)
    float switch_pen_g;     // gas <-> coast class change (gas <-> brake costs 2x, coast <-> brake 0.5x)
    float repull_pen_g;     // extra for gas after rolling/braking (light throttle < 25 % counts as rolling): against lift/re-pull cycles
    float shift_pen_g;      // a predicted gear change
    float beta_scale;       // x the calibrated time weight
    float w_under;          // g/s per (m/s)^2 below set
    float eps_g;            // g per m/s of |dv| per stage
    float gap_band_s;       // free band above the wanted gap (s of time gap)
    float accel_comf;       // m/s2: the lowest-speed chain assumes at least this much acceleration back up (comfort)
    float roll_scale;       // x coast_relax and lead_roll_decel: < 1 starts rolls toward caps and slower leads earlier (Eco 0.7, Power 1.5)
    int   pulse_glide;      // 1 = allow rolling/neutral without a reason (pulse-and-glide). Default 0; only honoured in PL_ECO (owner 30-10: an option to test)
};
void pl_mode_defaults(int mode, PlModeParams *p);
// a mode parameter by its field name ("under_kmh", "beta_scale", ...): 1 = set, 0 = no such key
int pl_mode_param_set(PlModeParams *p, const char *key, float v);
// "plmode.<eco|normal|power|m>.<key>=v,..." (other tokens ignored): sets this mode's tokens, returns how many. Sim builds with
// -DPL_SIM_ENV apply SIM_SET through it at the end of pl_mode_defaults (sweeps); the DLL never reads the environment
int pl_mode_params_apply(int mode, PlModeParams *p, const char *spec);

struct PlSafety {
    float limit_over_kmh;   // never above the road limit + this (5)
    float brake_plan;       // m/s2: planned braking for caps (the envelope); the plan never needs more when it is on time
    float brake_max;        // m/s2: hard maximum when late (start above the envelope, lead): above brake_plan costs harsh_pen
    float harsh_pen_g;      // g per (m/s2 above brake_plan) per stage metre/50
    float brake_pen;        // g per J of service-brake energy (retarder: free but for a tiny comfort cost)
    float brake_firm;       // m/s2: braking on top of a decelerating coast below the throttle cap only from this decel (lift early, brake late
                            // and firm; descents and a lead are exempt)
    float brake_coarse;     // m/s2 beyond the in-gear coast a coarse (50 m) stage may brake without a lead / late start (descent holding)
    float overcap_pen;      // g per (m/s above the hard cap) per s: only reachable when late (a limit drop at the sign, a late bend)
    float neutral_min_s;    // neutral lasts at least this long (swept parameter)
    float over_reserve_s;   // the plan keeps the overspeed time caps this much short (the executor's lag)
    float over_recover;     // the overspeed bucket refills at this many s per s at or under set (0.25: 4 s under buys 1 s over)
    float coast_relax;      // m/s2: the gentle approach envelope to caps (a reason to roll; may go under the band there)
    float coast_reason_s;   // s of driving: a descent this far ahead is a reason to roll (crest lift, pre-descent)
    float v_floor;          // m/s: the planner's lowest speed (stops are handed to the stop logic)
    // lead
    float d0;               // standstill gap m
    float time_gap;         // s (the distance setting)
    float cushion_s;        // soft zone above the wanted gap (never coast into exactly the gap)
    float tense_dec_lo, tense_dec_hi;   // lead decel m/s2 where the gap slides from the lead's speed to ours
    float tense_ttc_hi, tense_ttc_lo;   // TTC s, same
    float lead_acc_tau;     // s: the measured lead accel decays with this
    float lead_fade_s, lead_fade_end_s; // the lead's weight fades from 1 to 0.1 between these
    float gap_pen_g_per_m;  // violation of the wanted gap, per metre per check
    float lead_roll_decel;  // m/s2: a slower lead is a reason to roll only once the gap is within the closing a roll-in at this decel takes
    float far_pen;          // g/s per s of time gap above the band
    float cushion_pen;      // g/s at the wanted gap (quadratic to 0 at the cushion's top)
};
void pl_safety_defaults(PlSafety *s);

struct PlProfile {          // the road ahead from the truck's front (s = 0), uniform spacing
    float ds;               // m (10 recommended)
    int   n;                // points
    float grade[PL_PROF_MAX];   // rise/run (+ = uphill)
    float vmax[PL_PROF_MAX];    // m/s hard cap at the point (bends, junctions, roundabouts); <= 0 = none
    float limit[PL_PROF_MAX];   // m/s road limit at the point; <= 0 = unknown (use the input's limit). No preview: the current limit everywhere
    float stop_m;               // a hard stop point (m, < 0 = none)
};
// helper: grade array from heights (m) on the same grid
void pl_profile_from_heights(PlProfile *p, const float *h, int n, float ds);

struct PlLead { int present; float gap, v, a; };    // radar gap (m), lead speed m/s, lead accel m/s2 (filtered)

struct PlInput {
    const PlVehicle *veh;
    const PlProfile *prof;
    PlModeParams mode;
    PlSafety safety;
    int   drive_mode;       // PL_ECO .. PL_MANUAL (only for the diagnostics; the behaviour is in `mode`)
    float v0;               // m/s
    int   gear0;            // current gear (1-based, 0 = neutral/unknown)
    int   neutral0;         // currently in neutral (eco-roll)
    float neutral_s0;       // seconds in neutral so far
    int   cls0;             // current action class (PL_GAS ...)
    float over_used_s;      // the overspeed bucket now (pl_over_bucket)
    float set;              // set speed m/s
    float limit;            // road limit now m/s (<= 0 unknown)
    PlLead lead;
    float horizon_m;        // 2000
    float h_coarse, h_fine; // 50, 12.5
    int   labels;           // 1..PL_LABELS
    int   nodes;            // <= PL_NODES
};
void pl_input_defaults(PlInput *in);   // horizon, steps, labels, nodes, safety defaults; vehicle/profile/mode left to the caller

struct PlStage {
    float s, v, t;          // position, planned speed, arrival time
    float force;            // wheel force over the stage leading here (N; braking negative)
    float decel;            // braking decel beyond coasting (m/s2) when act = PL_BRAKE
    float fuel_g;           // fuel of the stage leading here
    float gap;              // planned gap to the predicted lead at arrival (NAN without a lead)
    float gap_want;
    uint8_t act, gear, ret; // action class of the stage leading here, predicted gear, retarder level
    float hi, lo;           // the stage's speed bounds (diagnostics)
};

struct PlOutput {
    int   ok;               // a plan exists (0: no node reachable; commands fall back to "hold v0, brake at brake_max if above caps")
    int   n;                // stages including the start
    PlStage st[PL_MAX_STAGES + 1];
    // commands for the executors (from the head of the plan)
    float v_target;         // plan speed ~2 s ahead (cruise PI reference)
    float v_next, v_next2;  // plan speed at stage 1 and 2
    int   act;              // what to do now: PL_GAS / PL_COAST / PL_BRAKE / PL_NEUTRAL
    float force;            // wheel force now (N)
    float decel;            // decel wish now (m/s2, > 0 when braking)
    int   ret_level;        // retarder level now
    float service_decel;    // service-brake share of decel (m/s2)
    int   neutral_req;      // request neutral (or stay in it)
    float neutral_hold_s;   // committed neutral time left
    float v_overspeed_ok;   // the game's overspeed braking may act above this (max planned speed over 2 stages + 0.3 km/h)
    float v_throttle_cap;   // no throttle above this now (min(set, limit + 5))
    int   gear_pred;
    // diagnostics
    float cost, c_fuel, c_time, c_pen, c_term;          // the plan: total (incl. terminal) and its parts
    float base_cost, base_fuel_g, base_time_s;          // the plain cruise over the same stages (same terms, incl. terminal)
    float units;            // u: the fuel model's scale vs the reference grams the weights are in
    float beta, gamma;      // g/s, g/J
    float fuel_g, time_s;   // over the plan
    float min_gap_margin;   // min over the plan of (gap - wanted gap) (m), NAN without a lead
    int   why;              // PL_WHY_*
    int   late;             // the start is above what planned braking can meet (hard braking planned)
    int   labels_used, transitions, stages_fine;
    float tw;               // the tension weight used for the wanted gap
};

struct PlWork;              // scratch (caller-owned, ~1 MB): create once at init, reuse for every replan
PlWork *pl_work_create();   // heap once (tests / init); never called per replan
void pl_work_free(PlWork *w);

// fills tables in the vehicle (call once per truck change; cheap)
void pl_vehicle_defaults_scania(PlVehicle *v, float mass);   // Damien's Scania V8 3700 Nm 14-speed (the sims' "scania"), ETS2 fuel fit
void pl_vehicle_use_willans(PlVehicle *v);                   // switch the fuel model to the Willans alternative
float pl_force_max(const PlVehicle *v, int gear, float vms); // wheel force at full load (N), 0 outside the rpm range
float pl_rpm(const PlVehicle *v, int gear, float vms);
int   pl_gear_pred(const PlVehicle *v, float vms, float force);   // the TrueShift stand-in (1-based)
float pl_resist(const PlVehicle *v, float vms, float grade);      // rolling + grade + aero (N)
float pl_ret_force(const PlVehicle *v, int level, float vms);
float pl_drag_force(const PlVehicle *v, int gear, float vms);   // engine drag at the wheels with the fuel cut (N, >= 0)
float pl_fuel_at(const PlVehicle *v, int gear, float vms, float force, int neutral);   // g/s for a wheel force in a gear (or neutral idle)
float pl_beta_calibrated(const PlVehicle *v, float set);         // g/s: set is the steady optimum on the level
float pl_gamma(const PlVehicle *v);                              // g/J of kinetic energy (fuel per unit of wheel work at the set point)

// the overspeed bucket the executor keeps (same rule as inside the plan): +dt above set + 0.05 m/s, else -recover*dt
// (recover = PlSafety.over_recover: a short dip under set does not buy a new full excursion; no pumping set+band / set)
float pl_over_bucket(float bucket, float v, float set, float dt, float recover);

// THE planner. Deterministic, allocation-free.
void pl_plan(const PlInput *in, PlOutput *out, PlWork *w);
