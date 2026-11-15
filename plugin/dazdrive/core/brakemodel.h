// The brake model's pure part (WP-M2): the start values the vehicle model (core/vehicle.cpp) begins every truck with, and the maths that
// weighs what it learned against them. No SDK, no globals, no V/T: the vehicle model, the planner's fixture (core/planner.cpp) and the
// sims all include it, so every figure below is written ONCE. Every module reads the brake through the vehicle model's functions
// (core/vehicle.h, "the brake model"), not through this file.
//
// The three parts of the model, each a start value derived from what the truck is, then learned per truck and kept in brake_model.txt
// (the service gain's own file is brake_gain.txt):
//   service brake  gain in m/s2 per unit of request:  prior(mass)  ->  blend with the learned slot (weight = confidence x plausibility)
//   retarder       force (N) per stage:               the stage table x the stage's learned correction
//   engine brake   force (N) per stage and gear:      the stage's share of the engine's brake torque through the gear x the learned correction
#pragma once
#include <math.h>

// ---------------------------------------------------------------- the service brake
//
// Prior. The service brakes of a truck are dimensioned on its rated mass, so up to that mass the gain is a deceleration (the brake force
// grows with the mass) and above it the force stays (the same force, a smaller deceleration). The AEB's fit on 26-30 t logs: 10 m/s2 per
// unit of request up to 30 t, the same force above (300 kN at 30 t and over). It was the AEB's, the channel's and (as 7 and 6) the ACC's, the
// planner's and Decel's own copies.
//   Checked against the measurements: ride 8 (33.1 t Renault, 27-10) measured 465-520 kN = 14.1-15.8 m/s2 per request, the 02-11 ride 433 kN at
//   41 t with a trailer = 10.6 m/s2. The fit gives 300 kN there: 0.58-0.69 of every measurement, so it is the CONSERVATIVE end, on purpose: a
//   gain assumed too high is the dangerous error (the AEB starts late, a hold lets go), one assumed too low only brakes a little harder than
//   asked. It stays the start value; the learned slot moves it up (and down, for weak brakes).
static const float BM_SVC_REF_MS2 = 10.0f;      // m/s2 per request up to the rated mass
static const float BM_SVC_REF_KG = 30000.0f;    // the rated mass
static inline float bm_svc_prior(float mass_kg)
{
    return BM_SVC_REF_MS2 / fmaxf(1.0f, mass_kg / BM_SVC_REF_KG);
}

// The window of gains a measurement is believed in (m/s2 per request): under 1.2 a slip or a hill the model does not know, over 16 more than any
// truck measured (the largest, 520 kN at 33 t, is 15.8). The loop of the deceleration assist works inside the same upper edge.
static const float BM_SVC_GAIN_LO = 1.2f, BM_SVC_GAIN_HI = 16.0f;

// Spread of the prior, in units of ln(learned / prior), UPWARDS. 0.6 covers a learned slot up to 1.8 times the prior, which holds every
// measurement above (the largest, 520 kN / 300 kN = 1.73, is ln 0.55 = 0.92 sigma). Beyond it a slot is down-weighted, not trusted and not
// thrown away. Only upwards: a gain assumed too high is the dangerous error (the AEB starts late, a hold lets go), so brakes that measure
// WEAKER than the prior (a game slider, a physics mod, a worn truck) are believed as fast as the confidence allows, never held back.
static const float BM_SVC_SIGMA = 0.6f;

// Confidence at which the learned slot counts in full (the old rule: the gain was offered from 0.5, four clean events); below it the
// weight ramps up from 0 so that the first events already move the estimate a little.
static const float BM_CONF_FULL = 0.5f;

// Plausibility of a learned gain against the prior: 1 up to +1 sigma above it (and everywhere below it), then a Gaussian fall in the
// distance beyond. How slow the fall is grows with the events behind the slot (n/5 is the confidence's own count): a truck that really
// brakes at 3 times the fit is believed once 20 clean events say so, one event with that figure is mostly the prior's.
static inline float bm_svc_plaus(float learned, float prior, float n)
{
    if (!(learned > 0) || !(prior > 0)) return 0.0f;
    if (learned <= prior) return 1.0f;
    float z = logf(learned / prior) / BM_SVC_SIGMA;
    float k = fmaxf(1.0f, n / 5.0f);
    float d = fmaxf(0.0f, z - 1.0f) / k;
    return expf(-0.5f * d * d);
}
static inline float bm_svc_weight(float conf, float plaus) { return fminf(1.0f, fmaxf(0.0f, conf) / BM_CONF_FULL) * plaus; }

// What a plan or an ask may use of the service brake: the estimate, never over 0.6 g. A tyre on wet asphalt (mu ~ 0.6) is where a
// full request stops being deceleration (the pedal map's own top is 7.0 on a dry road); the planner's old ceiling was 6.
static const float BM_PLAN_DECEL_MS2 = 6.0f;

// ---------------------------------------------------------------- the retarder
//
// Stage table. The Scania V8 at 17-19 t (brake-measure.md, steps 1-4): 0.8 / 1.6 / 2.4 / 3.0 m/s2 at 18 t = 14.4 / 28.8 / 43.2 / 54 kN.
// A retarder is a force, so a load change does not move it (the three copies this replaces all kept the force and scaled the
// deceleration by 18000 / mass). No second truck has been measured, so no scaling by the truck's own size either: the learner corrects
// it per truck. The def files give the stage count (retarder_step_count) but not the torque (engine-brake-re.md: the retarder def at
// drive+0x4d0 is not chased). A truck with n stages puts stage i at i / n of the table (the top stage is always the top of the table).
static const float BM_RET_REF_KG = 18000.0f;
static inline float bm_ret_table_n(int knot)    // knot 0..4 = 0, 1/4 ... 4/4 of the stages
{
    static const float ms2[5] = { 0.0f, 0.8f, 1.6f, 2.4f, 3.0f };
    return ms2[knot < 0 ? 0 : knot > 4 ? 4 : knot] * BM_RET_REF_KG;
}
static inline float bm_ret_prior_frac(float frac)       // N at a stage fraction 0..1 (interpolated between the table's knots)
{
    float x = fminf(fmaxf(frac, 0.0f), 1.0f) * 4.0f;
    int a = (int)x;
    if (a >= 4) return bm_ret_table_n(4);
    return bm_ret_table_n(a) + (x - (float)a) * (bm_ret_table_n(a + 1) - bm_ret_table_n(a));
}
static inline float bm_ret_prior(int stage, int steps)  // N at stage 1..steps; 0 = no retarder (or stage 0)
{
    if (steps <= 0 || stage <= 0) return 0.0f;
    return bm_ret_prior_frac((float)(stage > steps ? steps : stage) / (float)steps);
}
// A sample is accepted within this band of the prior (a measurement of something else is not a retarder): 0.4-2.5 times, as the channel's
// learner always had it.
static const float BM_RET_ACCEPT_LO = 0.4f, BM_RET_ACCEPT_HI = 2.5f;

// ---------------------------------------------------------------- the retarder against the road speed (WP-BR3b)
//
// What the game does (tools/analyze_retarder_axis.py on the rides of notes/brake-logs*, docs/powerband-change.md BR3b): its retarder does NOT fade with the road
// speed. The torque follows the engine side through the ratio (pooled over the steady rows of a step, F = a + 20..39 kN per 1000 rpm and -7..0 kN per 10 km/h at the
// same rpm: a lower gear at a lower road speed brakes MORE), and the only run-out is the clutch opening at 3-6 km/h. So there is no knee in the game's data to learn,
// and a lever step at 20 km/h brakes as hard as at 60 -- which is the "brake assist, not a retarder" the owner felt (03-11).
//
// A retarder as built is a hydrodynamic coupling on the gearbox output: at a fixed filling its torque goes with the SQUARE of the shaft speed (T = c n^2), up to the
// torque the controller lets the stage reach. The shaft turns with the road speed (diff and wheels are constants of the truck), so for the lever the curve is
//     F(v) = F_stage x s(v / v_k),   s(x) -> x^2 for x << 1 and -> 1 for x >> 1.
// All stages share one shape: the filling of a stage is its share of the top stage's, so stage i reaches its own cap F_i where its n^2 line (F_i / F_top) c n^2 does, and
// c is the top stage's (F_top at v_k): F_i(v) = F_i (v / v_k)^2 below the cap, the same for every i.
//
// The knee v_k is where the top stage's n^2 line meets its force, and a retarder is rated by its braking POWER, about the engine's peak power (the data sheets of
// the hydrodynamic retarders pair 300-600 kW with 300-450 kW engines; the share is taken as 1): F_top x v_k = P_peak, so v_k = P_peak / F_top. It comes from two
// things the vehicle model has per truck -- the engine's peak power (the torque curve times rpm) and the top stage's force (the start value of the stage table,
// learned per truck) -- and no km/h figure: a stronger retarder on the same engine has its knee lower, a bigger engine with the same retarder higher.
// (A 500 kW engine -- the owner's 3300 Nm truck, a 1425 rpm-factor peak of the curve -- with the table's 54 kN top stage: 9.3 m/s = 33 km/h; with 77 kN, what a 34.8 t truck's
// top stage measured at ~2.2 m/s2 would be once learned, 6.5 m/s = 23 km/h. A 320 kW engine with 54 kN: 21 km/h.)
//
// The torque limiter of a real retarder is a controller, not a wall: min(x^2, 1) would be continuous in the deceleration but step its slope at the knee (a jerk of
// 2 a^2 / v_k = 1-2 m/s3, the size of the comfort limit). The soft minimum of order 2, s = (x^-4 + 1)^-1/2 = x^2 / sqrt(1 + x^4), has no corner: s = 0.71 at the knee,
// 0.91 at 1.5 knees, 0.97 at 2, and 0.24 at half a knee. It is a shape, not a measurement; the knee's derivation above is what ties it to the truck.
static inline float bm_ret_knee(float p_peak_w, float f_top_n)            // m/s; 0 = not known (the shape is 1 then)
{
    return (p_peak_w > 0 && f_top_n > 0) ? p_peak_w / f_top_n : 0.0f;
}
static inline float bm_ret_shape(float v, float knee_ms)                  // 0..1: the share of a stage's force the retarder gives at the road speed v (m/s)
{
    if (!(knee_ms > 0) || !(v == v)) return 1.0f;
    const float x = fabsf(v) / knee_ms, x2 = x * x;
    return x2 / sqrtf(1.0f + x2 * x2);
}

// A stage that is a share of the top stage reaches its own cap earlier (WP-0911-K4). The retarder's controller fills the retarder to the torque a stage asks for: the most it can give at a road speed is the
// full-filling line c n^2 = F_top (v / v_k)^2, and a stage cuts that at its own force F_i = f F_top. Stage i is on its cap from v = v_k sqrt(f) on, so its knee is the top stage's times sqrt(f): a stage that
// asks a fifth of the top's force has its knee at 45 % of the top's, and fades only below that. (The model above gave every stage the top's knee, which is right for the top stage and too pessimistic for
// the low ones -- the steps a driver uses on a long descent at the low speeds a heavy truck comes down to.)
static inline float bm_ret_shape_stage(float v, float knee_ms, float f)  // 0..1: the share of stage f's (0..1 of the top stage's) force the retarder gives at road speed v (m/s)
{
    return bm_ret_shape(v, knee_ms * sqrtf(fminf(fmaxf(f, 0.0f), 1.0f)));
}

// ---------------------------------------------------------------- the engine brake
//
// Prior. The engine brake's torque at the crank, as a share of the engine's peak torque, times the stage's share of the engine's stages,
// through the gear:  F = (stage / cap) x BM_EB_SHARE x torque x ratio x diff x eta / r.  The two figures it replaces (EBrake's d_est 0.2
// m/s2 per stage, Decel's EB_GUESS 0.8 m/s2 at the last stage of an 18 t truck) agree on one point: the Scania V8 (3700 Nm), 18 t, 80 km/h at
// 1500 rpm (total ratio 3.57, eta 0.9, r 0.5 m), last stage of four: 0.8 m/s2 = 14.4 kN at the wheels = 2241 Nm at the crank = 0.61 of the
// peak torque (352 kW at 1500 rpm, a heavy engine's compression brake). The share is that 0.61; it comes with the gear (a lower gear
// brakes more, as Decel's retarder and engine brake always did) and the engine (a bigger engine brakes more), where the old figures
// were one force for every truck and gear.
static const float BM_EB_SHARE = 0.61f;         // of the peak torque, at the last stage
static inline float bm_eb_prior(int stage, int cap, float torque_nm, float ratio_total, float eta, float wheel_r)   // N at the wheels
{
    if (cap <= 0 || stage <= 0 || !(torque_nm > 0) || !(ratio_total > 0) || !(wheel_r > 0.1f)) return 0.0f;
    float share = (float)(stage > cap ? cap : stage) / (float)cap;
    return share * BM_EB_SHARE * torque_nm * ratio_total * eta / wheel_r;
}
static const float BM_EB_ACCEPT_LO = 0.25f, BM_EB_ACCEPT_HI = 4.0f;   // the same band, wider: one figure of the prior covers every gear

// ---------------------------------------------------------------- learned corrections per stage (the retarder and the engine brake)
//
// A stage's correction is the ratio of what was measured to the prior at that stage (1 = as the prior says). A stage with no sample takes
// the events-weighted mean of the stages that have some (the retarder's stages share one design, the engine brake's too), or 1.
// A sample moves its stage by max(1 / (n + 1), 0.05) (the weight the channel's retarder learner always used), n up to 100.
enum { BM_STAGES_MAX = 8 };
struct bm_stages { float c[BM_STAGES_MAX]; float n[BM_STAGES_MAX]; };
static inline void bm_stages_reset(bm_stages *s) { for (int i = 0; i < BM_STAGES_MAX; i++) { s->c[i] = 1.0f; s->n[i] = 0.0f; } }
static inline float bm_stages_mean(const bm_stages &s, int steps)
{
    float a = 0, w = 0;
    for (int i = 0; i < steps && i < BM_STAGES_MAX; i++) { a += s.n[i] * s.c[i]; w += s.n[i]; }
    return w > 0 ? a / w : 1.0f;
}
static inline float bm_stage_corr(const bm_stages &s, int stage, int steps)   // stage 1..steps
{
    if (stage < 1 || stage > steps || stage > BM_STAGES_MAX) return 1.0f;
    return s.n[stage - 1] > 0 ? s.c[stage - 1] : bm_stages_mean(s, steps);
}
static inline void bm_stage_add(bm_stages *s, int stage, int steps, float ratio)   // ratio = measured / prior; the caller has applied the accept band
{
    if (stage < 1 || stage > steps || stage > BM_STAGES_MAX || !(ratio > 0)) return;
    float &c = s->c[stage - 1], &n = s->n[stage - 1];
    if (n <= 0) c = ratio;      // the first sample of a stage replaces the start value
    else c += fmaxf(1.0f / (n + 1.0f), 0.05f) * (ratio - c);
    n = fminf(n + 1.0f, 100.0f);
}
