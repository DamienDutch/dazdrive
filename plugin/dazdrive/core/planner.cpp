// Longitudinal planner core: see planner.h for the algorithm. Pure function, no game includes.
#include "planner.h"
#include "brakemodel.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

static const float G = 9.81f;
static inline float clampf(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static inline float smooth01(float x) { x = clampf(x, 0, 1); return x * x * (3 - 2 * x); }
static inline float kmh(float k) { return k / 3.6f; }

// ---------------------------------------------------------------------------------------------------------------------------------
// vehicle model

float pl_fuel_willans(const void *ctx, float rpm, float p_ind, int neutral)
{
    const PlWillans *w = (const PlWillans *)ctx;
    if (neutral) return w->a_g_per_j * w->fric_nm * w->idle_rpm * 0.10472f;
    float f = w->a_g_per_j * (p_ind + w->fric_nm * rpm * 0.10472f);   // fuel cut once the wheels drive the friction
    return f > 0 ? f : 0;
}

// g(thr) of the ETS2 fit (fuel_model.hpp FUEL_G_*)
static const float FG_T[15] = { 0.00f, 0.02f, 0.03f, 0.05f, 0.07f, 0.10f, 0.14f, 0.18f, 0.225f, 0.275f, 0.30f, 0.45f, 0.60f, 0.80f, 1.00f };
static const float FG_V[15] = { 0.000f, 0.000f, 0.043f, 0.069f, 0.100f, 0.139f, 0.182f, 0.255f, 0.308f, 0.365f, 0.384f, 0.544f, 0.691f, 0.861f, 1.000f };

static float interp(const float *x, const float *y, int n, float v)
{
    if (n <= 0) return 0;
    if (v <= x[0]) return y[0];
    for (int i = 1; i < n; i++) if (v <= x[i]) return y[i-1] + (y[i] - y[i-1]) * (v - x[i-1]) / fmaxf(1e-6f, x[i] - x[i-1]);
    return y[n-1];
}

float pl_fuel_ets2(const void *ctx, float rpm, float p_ind, int neutral)
{
    const PlEts2Fuel *e = (const PlEts2Fuel *)ctx;
    float k = e->a_ml_per_rad_nm * e->tpk * e->zone_scale * e->g_per_ml;
    if (neutral) return rpm > e->idle_rpm + 60 ? 0 : k * 0.062f * e->idle_rpm * 0.10472f;
    if (p_ind <= 0) return 0;
    float w = fmaxf(1.0f, rpm * 0.10472f), share = fmaxf(0.05f, interp(e->curve_rpm, e->curve_share, e->ncurve, rpm));
    float thr = p_ind / (w * e->tpk * share);
    if (thr > 1) return k * share * w * thr;          // beyond full load (only in reachability probes): linear
    return k * share * interp(FG_T, FG_V, 15, thr) * w;
}

static inline float fuel_rate(const PlVehicle *v, float rpm, float p_ind, int neutral)
{
    PlFuelFn f = v->fuel ? v->fuel : pl_fuel_ets2;
    const void *ctx = v->fuel_ctx ? v->fuel_ctx : f == pl_fuel_willans ? (const void *)&v->willans : (const void *)&v->ets2;
    return f(ctx, rpm, p_ind, neutral);
}

static float torque_at(const PlVehicle *v, float rpm)
{
    if (v->ncurve <= 0) return 0;
    if (rpm <= v->curve_rpm[0]) return v->curve_nm[0];
    for (int i = 1; i < v->ncurve; i++)
        if (rpm <= v->curve_rpm[i]) {
            float f = (rpm - v->curve_rpm[i-1]) / fmaxf(1e-3f, v->curve_rpm[i] - v->curve_rpm[i-1]);
            return v->curve_nm[i-1] + f * (v->curve_nm[i] - v->curve_nm[i-1]);
        }
    return v->curve_nm[v->ncurve - 1];
}

static inline float gear_k(const PlVehicle *v, int g) { return v->ratio[g-1] * v->final_drive / v->wheel_r; }   // wheel N per crank Nm

float pl_rpm(const PlVehicle *v, int g, float vms) { return vms / v->wheel_r * v->ratio[g-1] * v->final_drive * 9.5493f; }

float pl_force_max(const PlVehicle *v, int g, float vms)
{
    if (g < 1 || g > v->ngear) return 0;
    float r = pl_rpm(v, g, vms);
    if (r > v->max_rpm) return 0;
    return torque_at(v, fmaxf(r, v->idle_rpm)) * gear_k(v, g) * v->eta;
}

// the stand-in for TrueShift: the highest gear in the rpm window that gives the force with the reserve; else the strongest gear that
// may run at this speed (kickdown); braking/coasting: the highest gear in the window (no braking downshifts modelled).
int pl_gear_pred(const PlVehicle *v, float vms, float force)
{
    int strongest = 0; float fs = -1;
    for (int g = v->ngear; g >= 1; g--) {
        float r = pl_rpm(v, g, vms);
        if (r > v->max_rpm) continue;
        if (r >= v->gear_rpm_lo && r <= v->gear_rpm_hi) {
            if (force <= 0) return g;
            if (force <= v->gear_reserve * pl_force_max(v, g, vms)) return g;
        }
        float f = pl_force_max(v, g, vms);
        if (r >= v->idle_rpm && f > fs) { fs = f; strongest = g; }
    }
    if (force <= 0) {   // slow: the highest gear above idle
        for (int g = v->ngear; g >= 1; g--) { float r = pl_rpm(v, g, vms); if (r >= v->idle_rpm && r <= v->max_rpm) return g; }
        return 1;
    }
    return strongest ? strongest : 1;
}

float pl_resist(const PlVehicle *v, float vms, float grade)
{
    float c = 1 / sqrtf(1 + grade * grade);
    return v->mass * G * (v->crr * c + grade * c) + 0.5f * v->rho * v->cda * vms * vms;
}

float pl_ret_force(const PlVehicle *v, int level, float vms)
{
    if (level <= 0 || v->ret_levels <= 0) return 0;
    if (level > v->ret_levels) level = v->ret_levels;
    float f = v->ret_force[level-1];
    return v->ret_v_full > 0 && vms < v->ret_v_full ? f * vms / v->ret_v_full : f;
}

static inline float drag_nm(const PlVehicle *v, float rpm)
{
    float f = (rpm - v->drag_lo_rpm) / fmaxf(1.0f, v->drag_hi_rpm - v->drag_lo_rpm);
    return fmaxf(0.3f * v->drag_lo_nm, v->drag_lo_nm + f * (v->drag_hi_nm - v->drag_lo_nm));
}

// engine drag at the wheels with the fuel cut: F = T_drag * k * eta, so that the indicated power at F = -drag is exactly 0
static float drag_force(const PlVehicle *v, int g, float vms)
{
    if (g < 1) return 0;
    float r = pl_rpm(v, g, vms);
    return r < v->idle_rpm * 0.9f ? 0 : drag_nm(v, r) * gear_k(v, g) * v->eta;
}

// brake (crank output) power for a wheel force F: the fuel model adds its own friction share. In the ETS2 fit the effective throttle
// matches the brake torque share (83 km/h 33 t g12 997 rpm: thr 0.246 measured, brake share 0.255 modelled); g(thr) holds the friction.
// A wheel force between the engine drag and zero (overrun, a sliver of throttle) costs ~nothing there.
static inline float p_ind(const PlVehicle *v, int g, float vms, float F)
{
    (void)g;
    return F * vms / v->eta;
}

float pl_drag_force(const PlVehicle *v, int gear, float vms) { return drag_force(v, gear, vms); }
float pl_fuel_at(const PlVehicle *v, int gear, float vms, float force, int neutral)
{
    if (neutral || gear < 1) return fuel_rate(v, v->idle_rpm, 0, 1);
    return fuel_rate(v, pl_rpm(v, gear, vms), p_ind(v, gear, vms, force), 0);
}

static float steady_fuel_per_m(const PlVehicle *v, float vms, int g)
{
    float F = pl_resist(v, vms, 0), rpm = pl_rpm(v, g, vms);
    return fuel_rate(v, rpm, p_ind(v, g, vms, F), 0) / vms;
}

float pl_beta_calibrated(const PlVehicle *v, float set)
{
    set = fmaxf(set, 3.0f);
    int g = pl_gear_pred(v, set, pl_resist(v, set, 0));
    float d = 0.25f;
    float dc = (steady_fuel_per_m(v, set + d, g) - steady_fuel_per_m(v, set - d, g)) / (2 * d);   // g/m per m/s
    return fmaxf(0.0f, set * set * dc);
}

float pl_gamma(const PlVehicle *v)
{
    float set = kmh(80), F = pl_resist(v, set, 0);
    int g = pl_gear_pred(v, set, F);
    float rpm = pl_rpm(v, g, set), dF = 500;
    float f0 = fuel_rate(v, rpm, p_ind(v, g, set, F), 0), f1 = fuel_rate(v, rpm, p_ind(v, g, set, F + dF), 0);
    return (f1 - f0) / (dF * set);
}

float pl_over_bucket(float bucket, float v, float set, float dt, float recover)
{
    return v > set + 0.05f ? bucket + dt : fmaxf(0.0f, bucket - recover * dt);
}

void pl_vehicle_defaults_scania(PlVehicle *v, float mass)
{
    memset(v, 0, sizeof *v);
    v->mass = mass; v->crr = 0.006f; v->cda = 6; v->rho = 1.2f;
    static const float r[14] = { 20.81f, 16.16f, 12.57f, 9.76f, 7.56f, 5.87f, 4.55f, 3.53f, 2.77f, 2.15f, 1.66f, 1.29f, 1.00f, 0.78f };
    v->ngear = 14; memcpy(v->ratio, r, sizeof r);
    v->final_drive = 2.59f; v->wheel_r = 0.538f; v->eta = 0.9f;
    v->idle_rpm = 550; v->max_rpm = 2300;
    static const float cr[8] = { 300, 440, 1000, 1450, 1600, 1800, 2300, 2500 }, cf[8] = { 0.10f, 0.40f, 1.00f, 1.00f, 0.91f, 0.82f, 0.40f, 0.10f };
    v->ncurve = 8; for (int i = 0; i < 8; i++) { v->curve_rpm[i] = cr[i]; v->curve_nm[i] = 3700 * cf[i]; }
    v->drag_lo_nm = 0.08f * 3700; v->drag_lo_rpm = 1000; v->drag_hi_nm = 0.12f * 3700; v->drag_hi_rpm = 1430;
    v->ret_levels = 4; for (int i = 0; i < 4; i++) v->ret_force[i] = bm_ret_prior(i + 1, 4);   // (the model's start values, core/brakemodel.h)
    v->ret_v_full = 8;
    v->brake_decel_max = 6;
    v->gear_rpm_lo = 900; v->gear_rpm_hi = 1750; v->gear_reserve = 0.92f;
    v->willans.a_g_per_j = 220.0f / 3.6e6f; v->willans.fric_nm = 0.09f * 3700; v->willans.idle_rpm = 550;
    PlEts2Fuel &e = v->ets2;
    e.a_ml_per_rad_nm = 7.35e-4f; e.tpk = 3700; e.idle_rpm = 550; e.zone_scale = 1; e.g_per_ml = 0.84f;
    e.ncurve = 8; for (int i = 0; i < 8; i++) { e.curve_rpm[i] = cr[i]; e.curve_share[i] = cf[i]; }
    v->fuel = pl_fuel_ets2; v->fuel_ctx = 0;
}

void pl_vehicle_use_willans(PlVehicle *v) { v->fuel = pl_fuel_willans; v->fuel_ctx = 0; }

void pl_mode_defaults(int mode, PlModeParams *p)
{
    memset(p, 0, sizeof *p);
    // closed-loop sweep 30-10 (tools/sim/plmode_sweep.py over eval_planner's 90 scenarios, the fit and Willans): Eco rolls toward caps a
    // little later (roll_scale 0.7 -> 0.8), prices sags (w_under 0.01 -> 0.03) and light-throttle / overrun chatter (switch_pen 1.5 -> 3):
    // -3.4 % fuel vs Normal at +0.4 % time (was -4.4 % at +0.7 %, +1.9 % at caps), a quarter less time deep under the band. Eco's beta
    // stays under Normal's (above it Eco lifts later than Normal at a crest). Normal (every key within +-0.05 % of its fuel + beta x time)
    // and Power (its time gain is the band's: beta, w_under, roll_scale move only its fuel) unchanged.
    switch (mode) {
    case PL_ECO:
        p->under_kmh = 9; p->over_kmh = 5; p->schwung_kmh = 3; p->schwung_s = 30; p->over_time_s = 40;
        p->neutral = 1; p->neutral_pen_g = 4; p->switch_pen_g = 3; p->repull_pen_g = 8; p->shift_pen_g = 3; p->beta_scale = 1.05f;
        p->w_under = 0.03f; p->eps_g = 0.5f; p->gap_band_s = 1.5f; p->accel_comf = 0.25f; p->roll_scale = 0.8f; break;
    case PL_POWER: case PL_MANUAL:
        p->under_kmh = 2.5f; p->over_kmh = 2.5f; p->schwung_kmh = 0; p->schwung_s = 0; p->over_time_s = 40;
        p->neutral = mode == PL_POWER; p->neutral_pen_g = 12; p->switch_pen_g = 3; p->repull_pen_g = 12; p->shift_pen_g = 1; p->beta_scale = 1.3f;
        p->w_under = 0.4f; p->eps_g = 0.2f; p->gap_band_s = 0.3f; p->accel_comf = 0.3f; p->roll_scale = 1.5f; break;
    default:
        p->under_kmh = 5; p->over_kmh = 5; p->schwung_kmh = 0; p->schwung_s = 0; p->over_time_s = 40;
        p->neutral = 1; p->neutral_pen_g = 10; p->switch_pen_g = 3; p->repull_pen_g = 12; p->shift_pen_g = 2; p->beta_scale = 1.12f;
        p->w_under = 0.05f; p->eps_g = 0.35f; p->gap_band_s = 0.8f; p->accel_comf = 0.2f; p->roll_scale = 1.0f; break;
    }
#ifdef PL_SIM_ENV   // sim builds only (never the DLL): SIM_SET="plmode.eco.under_kmh=9,plmode.normal.beta_scale=1.1" overrides for sweeps
    pl_mode_params_apply(mode, p, getenv("SIM_SET"));
#endif
}

// the mode parameters by name (sweeps; ini keys later): the struct's fields in order
static const struct { const char *key; unsigned off; int isint; } PL_MODE_KEYS[] = {
#define PLK(f, i) { #f, (unsigned)offsetof(PlModeParams, f), i }
    PLK(under_kmh, 0), PLK(over_kmh, 0), PLK(schwung_kmh, 0), PLK(schwung_s, 0), PLK(over_time_s, 0), PLK(neutral, 1),
    PLK(neutral_pen_g, 0), PLK(switch_pen_g, 0), PLK(repull_pen_g, 0), PLK(shift_pen_g, 0), PLK(beta_scale, 0), PLK(w_under, 0),
    PLK(eps_g, 0), PLK(gap_band_s, 0), PLK(accel_comf, 0), PLK(roll_scale, 0), PLK(pulse_glide, 1),
#undef PLK
};

int pl_mode_param_set(PlModeParams *p, const char *key, float v)
{
    for (unsigned i = 0; i < sizeof PL_MODE_KEYS / sizeof PL_MODE_KEYS[0]; i++)
        if (!strcmp(PL_MODE_KEYS[i].key, key)) {
            char *b = (char *)p + PL_MODE_KEYS[i].off;
            if (PL_MODE_KEYS[i].isint) *(int *)b = (int)lroundf(v); else *(float *)b = v;
            return 1;
        }
    return 0;
}

int pl_mode_params_apply(int mode, PlModeParams *p, const char *spec)
{
    static const char *const MN[4] = { "eco", "normal", "power", "m" };
    if (!spec || mode < 0 || mode > 3) return 0;
    const size_t ml = strlen(MN[mode]);
    int n = 0;
    for (const char *t = spec; *t; ) {
        const char *e = strchr(t, ',');
        size_t len = e ? (size_t)(e - t) : strlen(t);
        char tok[96];
        if (len < sizeof tok) {
            memcpy(tok, t, len); tok[len] = 0;
            char *eq = strchr(tok, '='), *k = tok + 7;
            if (eq && !strncmp(tok, "plmode.", 7) && !strncmp(k, MN[mode], ml) && k[ml] == '.') { *eq = 0; n += pl_mode_param_set(p, k + ml + 1, (float)atof(eq + 1)); }
        }
        if (!e) break;
        t = e + 1;
    }
    return n;
}

void pl_safety_defaults(PlSafety *s)
{
    memset(s, 0, sizeof *s);
    s->limit_over_kmh = 5; s->brake_plan = 1.3f; s->brake_max = 3.5f; s->harsh_pen_g = 150; s->brake_pen = 2e-5f;
    s->neutral_min_s = 8; s->v_floor = 1.5f; s->coast_relax = 0.3f; s->coast_reason_s = 30;
    s->d0 = 8; s->time_gap = 2.0f; s->cushion_s = 0.3f;
    s->tense_dec_lo = 0.3f; s->tense_dec_hi = 1.5f; s->tense_ttc_hi = 6; s->tense_ttc_lo = 3;
    s->lead_acc_tau = 3; s->lead_fade_s = 15; s->lead_fade_end_s = 25;
    s->gap_pen_g_per_m = 150; s->lead_roll_decel = 0.12f; s->over_reserve_s = 3; s->over_recover = 0.5f; s->brake_coarse = 0.35f; s->far_pen = 4; s->cushion_pen = 2; s->overcap_pen = 30; s->brake_firm = 0.6f;
}

void pl_input_defaults(PlInput *in)
{
    memset(in, 0, sizeof *in);
    pl_safety_defaults(&in->safety);
    pl_mode_defaults(PL_NORMAL, &in->mode);
    in->drive_mode = PL_NORMAL; in->cls0 = PL_GAS;
    in->horizon_m = 2000; in->h_coarse = 50; in->h_fine = 12.5f; in->labels = PL_LABELS; in->nodes = PL_NODES;
}

void pl_profile_from_heights(PlProfile *p, const float *h, int n, float ds)
{
    p->ds = ds; p->n = n < PL_PROF_MAX ? n : PL_PROF_MAX;
    for (int i = 0; i < p->n; i++) {
        int a = i > 0 ? i - 1 : 0, b = i + 1 < p->n ? i + 1 : p->n - 1;
        p->grade[i] = b > a ? (h[b] - h[a]) / ((b - a) * ds) : 0;
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// scratch

struct Lab {
    float cost, t, tov;
    int32_t pred;           // (k * PL_NODES + i) * PL_LABELS + l of the predecessor, -1 at the start
    float force, fuel, eland;
    uint8_t cls, gear, act, span, ret, nwait;
    float sdec;             // service decel
    float eref;             // the ratchet: without a reason the speed may not fall under this (the highest e since the last reason)
};

#define PL_INT_MAX 8
#define PL_TB_N    480      // speed bins of 0.1 m/s (0 .. 48 m/s): gear prediction, full-load force, drag
#define PL_TB_DV   0.1f
struct PlWork {
    // vehicle tables, rebuilt only when the vehicle changes (a copy is kept to compare)
    PlVehicle tbveh; int tbok;
    uint8_t tb_gc[PL_TB_N];             // the coast/brake gear (highest in the window)
    uint16_t tb_win[PL_TB_N], tb_ok[PL_TB_N];   // gears in the rpm window / runnable (idle .. max rpm)
    float tb_fd[PL_TB_N], tb_fe[PL_TB_N];      // drag force in tb_gc, the strongest full-load force
    float tb_fm[PL_TB_N][PL_GEARS];     // full-load force per gear (0 = over the rpm limit)
    // profile-derived (per point)
    int np; float ds;
    float capH[PL_PROF_MAX + 1];    // hard cap per point (bends/junctions/stop, limit + 5), m/s
    float env[PL_PROF_MAX + 1];     // backward envelope at brake_plan
    float env2[PL_PROF_MAX + 1];    // backward envelope at coast_relax: where a gentle approach to a cap is already under set
    float lim5[PL_PROF_MAX + 1];    // limit + 5 per point
    // stages
    int K;
    uint8_t reason[PL_MAX_STAGES + 1];   // a reason to coast/roll from this stage (downhill here, cap ahead, slower lead)
    uint8_t crest[PL_MAX_STAGES + 1];    // a descent ahead reachable by coasting (gated on speed when consumed)
    float s[PL_MAX_STAGES + 1], h[PL_MAX_STAGES], c[PL_MAX_STAGES], kh[PL_MAX_STAGES], gmin[PL_MAX_STAGES];
    float hi[PL_MAX_STAGES + 1], lo[PL_MAX_STAGES + 1], vtr[PL_MAX_STAGES + 1], vcap[PL_MAX_STAGES + 1];
    float elo[PL_MAX_STAGES + 1], de[PL_MAX_STAGES + 1];
    int   nn[PL_MAX_STAGES + 1], jmin[PL_MAX_STAGES + 1]; float etop[PL_MAX_STAGES + 1];
    int   nint[PL_MAX_STAGES]; float intf[PL_MAX_STAGES][PL_INT_MAX], inte[PL_MAX_STAGES][PL_INT_MAX];
    // labels
    Lab   lab[PL_MAX_STAGES + 1][PL_NODES][PL_LABELS];
    uint8_t cnt[PL_MAX_STAGES + 1][PL_NODES];
    // lead prediction
    int   lead; float ldt; int lnst; float lx[PL_LEAD_STEPS + 1], lv[PL_LEAD_STEPS + 1]; float tw;
    // per call constants
    float beta, gamma, m, ka, set, vtrac0;
    int   L;
};

PlWork *pl_work_create() { return (PlWork *)calloc(1, sizeof(PlWork)); }

static void tables(PlWork *w, const PlVehicle *V)
{
    if (w->tbok && !memcmp(&w->tbveh, V, sizeof *V)) return;
    memcpy(&w->tbveh, V, sizeof *V); w->tbok = 1;
    for (int b = 0; b < PL_TB_N; b++) {
        float v = b * PL_TB_DV; uint16_t win = 0, ok = 0; float fe = 0;
        for (int g = 1; g <= V->ngear && g <= PL_GEARS; g++) {
            float r = pl_rpm(V, g, v), f = pl_force_max(V, g, v);
            w->tb_fm[b][g-1] = f; fe = fmaxf(fe, f);
            if (r <= V->max_rpm && r >= V->idle_rpm) ok |= (uint16_t)(1u << (g - 1));
            if (r <= V->max_rpm && r >= V->gear_rpm_lo && r <= V->gear_rpm_hi) win |= (uint16_t)(1u << (g - 1));
        }
        w->tb_win[b] = win; w->tb_ok[b] = ok; w->tb_fe[b] = fe;
        int gc = pl_gear_pred(V, v, 0); w->tb_gc[b] = (uint8_t)gc; w->tb_fd[b] = drag_force(V, gc, v);
    }
}

static inline int tbin(float v) { int b = (int)(v / PL_TB_DV + 0.5f); return b < 0 ? 0 : b >= PL_TB_N ? PL_TB_N - 1 : b; }

// pl_gear_pred() from the tables (same rule)
static inline int gear_fast(const PlWork *w, const PlVehicle *V, int b, float F)
{
    if (F <= 0) return w->tb_gc[b];
    int strongest = 0; float fs = -1;
    for (int g = V->ngear; g >= 1; g--) {
        float f = w->tb_fm[b][g-1];
        if (f <= 0) continue;
        if ((w->tb_win[b] >> (g - 1)) & 1) { if (F <= V->gear_reserve * f) return g; }
        if (((w->tb_ok[b] >> (g - 1)) & 1) && f > fs) { fs = f; strongest = g; }
    }
    return strongest ? strongest : 1;
}
void pl_work_free(PlWork *w) { free(w); }

// ---------------------------------------------------------------------------------------------------------------------------------
// profile helpers

static inline int pidx(const PlWork *w, float x) { int i = (int)floorf(x / w->ds + 1e-4f); return i < 0 ? 0 : i >= w->np ? w->np - 1 : i; }
// the hard cap at a position: the min of the intervals touching it (a stage edge on a cap start sees the cap)
static inline float capat(const PlWork *w, const float *a, float x)
{
    int i0 = (int)floorf((x - 0.01f) / w->ds), i1 = (int)floorf((x + 0.01f) / w->ds);
    i0 = i0 < 0 ? 0 : i0 >= w->np ? w->np - 1 : i0; i1 = i1 < 0 ? 0 : i1 >= w->np ? w->np - 1 : i1;
    return fminf(a[i0], a[i1]);
}

static void lead_predict(PlWork *w, const PlInput *in)
{
    const PlLead &L = in->lead;
    w->lead = L.present && L.gap > 0;
    if (!w->lead) return;
    w->ldt = 0.25f; w->lnst = PL_LEAD_STEPS;
    float x = L.gap, v = fmaxf(0, L.v), a0 = L.a, tau = fmaxf(0.5f, in->safety.lead_acc_tau);
    // a braking lead keeps braking at least at its measured decel (plus a margin for the 0.5 s filter lag): the decaying model
    // under-predicts a lead that sustains hard braking, so the plan closes too fast and can no longer stop in the gap. The same
    // threshold as lead_hard's braking term: a lead that is not actually braking keeps the (calibrated) decaying model.
    const int hard = a0 < -in->safety.tense_dec_lo;
    const float a_floor = hard ? a0 - 1.0f : 0.0f;
    w->lx[0] = x; w->lv[0] = v;
    for (int i = 1; i <= PL_LEAD_STEPS; i++) {
        float t = (i - 0.5f) * w->ldt, a = a0 * expf(-t / tau);
        if (hard && a > a_floor) a = a_floor;   // the worst of the decaying model and the measured decel (with margin)
        float vn = fmaxf(0, v + a * w->ldt);
        if (x < w->np * w->ds) vn = fminf(vn, fmaxf(capat(w, w->env, x), fminf(v, 0.5f)));   // the lead follows the same route caps
        x += 0.5f * (v + vn) * w->ldt; v = vn;
        w->lx[i] = x; w->lv[i] = v;
    }
}

static inline void lead_at(const PlWork *w, float t, float *x, float *v)
{
    float f = t / w->ldt; int i = (int)f;
    if (i >= w->lnst) { *v = w->lv[w->lnst]; *x = w->lx[w->lnst] + *v * (t - w->lnst * w->ldt); return; }
    if (i < 0) { i = 0; f = 0; }
    f -= i;
    *x = w->lx[i] + f * (w->lx[i+1] - w->lx[i]); *v = w->lv[i] + f * (w->lv[i+1] - w->lv[i]);
}

// the lead's cost for being at s with speed v at time t (soft terms only at stage points: full = 1; mid-points: violation only)
static inline float lead_cost(const PlWork *w, const PlSafety &S, const PlModeParams &M, float s, float v, float t, float dt, int full, float *gapo, float *wanto)
{
    float xl, vl; lead_at(w, t, &xl, &vl);
    float gap = xl - s, vref = vl + w->tw * fmaxf(0, v - vl);
    float want = S.d0 + S.time_gap * vref;
    if (gapo) { *gapo = gap; *wanto = want; }
    float fade = t <= S.lead_fade_s ? 1 : t >= S.lead_fade_end_s ? 0.1f : 1 - 0.9f * (t - S.lead_fade_s) / fmaxf(0.1f, S.lead_fade_end_s - S.lead_fade_s);
    if (gap < want) return S.gap_pen_g_per_m * (want - gap) * fade;
    if (!full) return 0;
    float cush = fmaxf(1.0f, S.cushion_s * vref);
    if (gap < want + cush) { float q = (want + cush - gap) / cush; return S.cushion_pen * dt * q * q * fade; }
    float hi = want + M.gap_band_s * fmaxf(vref, 3.0f);
    if (gap > hi) return S.far_pen * dt * fminf(4.0f, (gap - hi) / fmaxf(v, 3.0f));
    return 0;
}

// the lead as a reason, per label (its arrival time). Bit 1 = may ROLL/brake (lose speed): we are inside the wanted gap, or faster than
// the lead (now or 5 s on) and the gap is within the closing a roll-in at lead_roll_decel takes (lift at the last moment and roll in).
// Bit 2 = FOLLOWING (may hold a speed under the throttle cap instead of getting back up to it): the gap is within the band + cushion
// + rolling reach. Following a steady lead at the wanted gap therefore holds its speed with a steady throttle: no roll/pull cycling.
static inline int lead_reason(const PlWork *w, const PlSafety &S, const PlModeParams &M, float s, float v, float t)
{
    float xl, vl, x5, v5; lead_at(w, t, &xl, &vl); lead_at(w, t + 5, &x5, &v5);
    float gap = xl - s, vref = vl + w->tw * fmaxf(0, v - vl), want = S.d0 + S.time_gap * vref;
    float slack = gap - want, dv = v - fminf(vl, v5);
    float reach = dv > 0 ? dv * dv / (2 * fmaxf(0.03f, S.lead_roll_decel * M.roll_scale)) : 0;
    float cush = S.cushion_s * fmaxf(vref, 3.0f) + 5;
    int r = 0;
    if (slack < 0 || (dv > 0.4f && slack <= reach + cush)) r |= 1;   // 0.4 m/s: a real speed difference, not one we just pulled up
    if (slack <= M.gap_band_s * fmaxf(vref, 3.0f) + cush + reach) r |= 2;
    return r;
}

// ---------------------------------------------------------------------------------------------------------------------------------
// labels

static inline float sw_cost(int a, int b, float P, float R)
{
    if (a == b || a == PL_ANY || b == PL_ANY) return 0;
    if (b == PL_GAS) P += R;   // a re-pull after rolling/braking
    if ((a == PL_GAS && b == PL_BRAKE) || (a == PL_BRAKE && b == PL_GAS)) return 2 * P;
    if ((a == PL_COAST && b == PL_BRAKE) || (a == PL_BRAKE && b == PL_COAST)) return 2 * P;
    return P;
}

// a dominates b: no continuation of b can be cheaper than the same one from a
static inline bool dominates(const Lab &a, const Lab &b, int lead, float P)
{
    float hcap = (a.cls == b.cls || a.cls == PL_ANY) ? 0 : 2 * P;
    if (a.cost + hcap > b.cost) return false;
    if (a.tov > b.tov + 1e-3f) return false;
    if (a.eref > b.eref + 0.05f) return false;
    if (lead && a.t < b.t - 1e-3f) return false;
    return true;
}

static int insert(PlWork *w, int k, int j, const Lab &c, float P)
{
    Lab *A = w->lab[k][j]; int n = w->cnt[k][j];
    for (int i = 0; i < n; i++) if (dominates(A[i], c, w->lead, P)) return 0;
    int m = 0;
    for (int i = 0; i < n; i++) if (!dominates(c, A[i], w->lead, P)) A[m++] = A[i];
    n = m;
    if (n < w->L) { A[n++] = c; w->cnt[k][j] = (uint8_t)n; return 1; }
    // full: protect the cheapest, the latest arrival (lead) and the least overspeed time; evict the dearest of the rest (or refuse c)
    int cheap = 0, late = 0, lowt = 0;
    for (int i = 1; i < n; i++) { if (A[i].cost < A[cheap].cost) cheap = i; if (A[i].t > A[late].t) late = i; if (A[i].tov < A[lowt].tov) lowt = i; }
    int ev = -1;
    for (int i = 0; i < n; i++) {
        if (i == cheap || (w->lead && i == late) || i == lowt) continue;
        if (ev < 0 || A[i].cost > A[ev].cost) ev = i;
    }
    if (ev < 0 || A[ev].cost <= c.cost) {
        // c is dearer than every evictable one: keep c only if it is cheaper than the cheapest or later than the latest
        bool better = c.cost < A[cheap].cost || (w->lead && c.t > A[late].t) || c.tov < A[lowt].tov;
        if (!better) { w->cnt[k][j] = (uint8_t)n; return 0; }
        if (ev < 0) ev = (cheap != late) ? (c.cost < A[cheap].cost ? cheap : late) : cheap;
    }
    A[ev] = c; w->cnt[k][j] = (uint8_t)n; return 1;
}

// nodes 0 .. nn-2 uniform (one of them exactly on the throttle cap), node nn-1 exactly on hi (a cap is met exactly)
static inline float node_e(const PlWork *w, int k, int j) { return j == w->nn[k] - 1 ? w->etop[k] : w->elo[k] + j * w->de[k]; }

static inline int snap(const PlWork *w, int k, float e)
{
    if (w->nn[k] == 1) return fabsf(e - w->etop[k]) <= fmaxf(0.5f, 0.02f * w->etop[k]) ? 0 : -1;
    float f = (e - w->elo[k]) / w->de[k];
    int j = (int)lrintf(f);
    if (j < w->jmin[k]) return -1;
    if (j >= w->nn[k] - 2) {   // the last uniform node or the top one, whichever is nearer
        float eu = w->elo[k] + (w->nn[k] - 2) * w->de[k], et = w->etop[k];
        if (e > et + w->de[k]) return -1;
        return fabsf(e - et) < fabsf(e - eu) ? w->nn[k] - 1 : w->nn[k] - 2;
    }
    return j;
}

// one constant-force step over segment k (implicit trapezoid: consistent forward and inverse): a_f = force per kg
static inline float step_e(const PlWork *w, int k, float e, float a_f)
{
    float kh2 = 0.5f * w->kh[k];
    return (e * (1 - kh2) + w->h[k] * (a_f - w->c[k])) / (1 + kh2);
}

// ---------------------------------------------------------------------------------------------------------------------------------

void pl_plan(const PlInput *in, PlOutput *out, PlWork *w)
{
    const PlVehicle *V = in->veh; const PlProfile *P = in->prof;
    // every gram-valued weight is in REFERENCE grams (a real-world diesel: gamma_ref = 6.8e-5 g per J at the wheels); the fuel model may
    // be in any unit or scale (the ETS2 fit is in game-scaled mL: ~10x), so the weights are scaled by u = gamma / gamma_ref
    const float gamma0 = pl_gamma(V), u = fmaxf(1e-3f, gamma0 / 6.8e-5f);
    PlSafety S = in->safety; PlModeParams M = in->mode;
    M.neutral_pen_g *= u; M.switch_pen_g *= u; M.repull_pen_g *= u; M.shift_pen_g *= u; M.w_under *= u; M.eps_g *= u;
    S.harsh_pen_g *= u; S.overcap_pen *= u; S.brake_pen *= u; S.gap_pen_g_per_m *= u; S.far_pen *= u; S.cushion_pen *= u;
    memset(out, 0, sizeof *out);
    out->min_gap_margin = NAN;
    const float m = V->mass, set = fmaxf(in->set, 1.0f), v0 = fmaxf(0.0f, in->v0);
    tables(w, V);
    const float lim_now = in->limit > 0 ? in->limit : 1e3f, lover = kmh(S.limit_over_kmh);
    w->m = m; w->ka = V->rho * V->cda / m; w->set = set;
    w->L = in->labels < 1 ? 1 : in->labels > PL_LABELS ? PL_LABELS : in->labels;
    const int NN = in->nodes < 2 ? 2 : in->nodes > PL_NODES ? PL_NODES : in->nodes;

    // --- per point: caps and the backward envelope (over the whole profile: caps beyond the horizon)
    w->ds = P && P->n > 0 ? P->ds : 10; w->np = P && P->n > 0 ? P->n : 1;
    for (int i = 0; i < w->np; i++) {
        float lim = P && P->n > 0 && P->limit[i] > 0 ? P->limit[i] : lim_now;
        float cap = lim + lover;
        w->lim5[i] = cap;
        if (P && P->n > 0 && P->vmax[i] > 0) cap = fminf(cap, P->vmax[i]);
        if (P && P->stop_m >= 0 && i * w->ds >= P->stop_m) cap = 0;
        w->capH[i] = cap;
    }
    auto envelope = [&]() {
        w->env[w->np - 1] = w->capH[w->np - 1];
        for (int i = w->np - 2; i >= 0; i--) w->env[i] = fminf(w->capH[i], sqrtf(w->env[i+1] * w->env[i+1] + 2 * S.brake_plan * w->ds));
        w->env2[w->np - 1] = w->capH[w->np - 1];
        for (int i = w->np - 2; i >= 0; i--) w->env2[i] = fminf(w->capH[i], sqrtf(w->env2[i+1] * w->env2[i+1] + 2 * S.coast_relax * M.roll_scale * w->ds));
    };
    envelope();
    // --- lead prediction; a predicted standstill becomes a stop point d0 behind it
    w->tw = 0;
    lead_predict(w, in);
    if (w->lead) {
        const PlLead &L = in->lead;
        float wb = smooth01((-L.a - S.tense_dec_lo) / fmaxf(0.01f, S.tense_dec_hi - S.tense_dec_lo));
        float cl = v0 - L.v, ttc = cl > 0.05f ? L.gap / cl : 1e3f;
        float wt = smooth01((S.tense_ttc_hi - ttc) / fmaxf(0.01f, S.tense_ttc_hi - S.tense_ttc_lo));
        w->tw = fmaxf(wb, wt);
        if (w->lv[w->lnst] < 0.05f) {
            float xs = w->lx[w->lnst] - S.d0;
            for (int i = 0; i < w->np; i++) if (i * w->ds >= xs) w->capH[i] = 0;
            envelope();
        }
    }
    out->tw = w->tw;
    const float overT = fmaxf(0.0f, M.over_time_s - S.over_reserve_s), schT = fmaxf(0.0f, M.schwung_s - S.over_reserve_s), vover = set + kmh(M.over_kmh) + 0.1f;

    // --- stages
    const float vfree = fmaxf(v0, set + kmh(M.over_kmh + M.schwung_kmh));
    // a start above the gravity band (set + over + schwung: the set speed lowered, a resume, a cruise that ran over it) counts as late too,
    // else the coarse first stage may brake only brake_coarse beyond the coast and cannot reach the band (shadow 30-10: 92 km/h, set 85, -3 %)
    const float hardcap0 = fminf(capat(w, w->env, 0), capat(w, w->lim5, 0));
    const float vcap0 = fminf(hardcap0, set + kmh(M.over_kmh + M.schwung_kmh));
    // a hard cap (limit + 5 / bend) is a must: only float noise above it is not late; the gravity band is a soft overspeed limit
    // where a cruise that ran a little over it is handled by the coarse stage (0.05 m/s tolerance)
    const float late_tol = hardcap0 < vcap0 + 0.1f && hardcap0 < set + kmh(M.over_kmh + M.schwung_kmh) - 0.1f ? 0.01f : 0.05f;
    float late_m = v0 > vcap0 + late_tol ? (v0 * v0 - vcap0 * vcap0) / (2 * S.brake_plan) : 0;
    // an overspeed whose time cap is about to run out must brake back the same fine run-up as a late start: a coarse stage is
    // limited to brake_coarse, and on a descent the grade eats most of that, so it cannot recover from above set. A real overspeed
    // (> 1 km/h above set) or any overspeed already on a descent counts; a cruise's 0.5 km/h overshoot on the level does not.
    const bool downhill0 = P && P->n > 0 && P->grade[0] < -0.005f;
    float back_m = 0;
    if (v0 > set + 0.1f && in->over_used_s > overT - 3.0f && (v0 > set + kmh(1.0f) || downhill0)) back_m = (v0 * v0 - set * set) / (2 * S.brake_plan);
    if (v0 > vover + 0.1f && in->over_used_s > schT - 2.0f) back_m = fmaxf(back_m, (v0 * v0 - vover * vover) / (2 * S.brake_plan));
    late_m = fmaxf(late_m, back_m);
    const float H = fmaxf(100.0f, in->horizon_m);
    float hc = fmaxf(10.0f, in->h_coarse), hf = fminf(hc, fmaxf(5.0f, in->h_fine));
    for (int pass = 0; pass < 3; pass++) {
        int k = 0; float s = 0; w->s[0] = 0; int fine = 0;
        while (s < H - 0.5f && k < PL_MAX_STAGES) {
            // fine where the envelope is a braking run-up (below the local cap) within this step, or while braking down from a start above it
            int f = s < late_m;
            for (float x = s; !f && x <= s + hc + 0.01f; x += w->ds) if (capat(w, w->env, x) < fminf(capat(w, w->capH, x), vfree) - 0.2f) f = 1;
            float h = f ? hf : hc;
            // a stage edge exactly where a hard cap starts or ends (within the step): a cap end matters when the start sits inside it
            // (the interior caps would otherwise hold the truck at the cap through a whole coarse stage)
            int p0 = (int)floorf(s / w->ds) + 1, p1 = (int)floorf((s + h) / w->ds);
            for (int p = p0; p <= p1 && p < w->np; p++)
                if (p > 0 && p * w->ds - s > 1.0f &&
                    ((w->capH[p] < w->capH[p-1] - 0.1f && w->capH[p] < vfree) ||
                     (w->capH[p] > w->capH[p-1] + 0.1f && w->capH[p-1] < vfree))) { h = p * w->ds - s; break; }
            if (s + h > H) h = H - s;
            s += h; k++; w->s[k] = s; fine += f;
        }
        w->K = k; out->stages_fine = fine;
        if (s >= H - 0.5f || pass == 2) break;
        hf *= 2;    // too many fine stages for the array: coarser near caps (rare: several caps in 2 km)
    }
    const int K = w->K;
    for (int k = 0; k < K; k++) {
        w->h[k] = w->s[k+1] - w->s[k];
        // mean grade over the segment
        float gs = 0; int gn = 0; w->gmin[k] = 0;
        if (P && P->n > 0) for (float x = w->s[k] + 0.5f * w->ds; x < w->s[k+1]; x += w->ds) { float g = P->grade[pidx(w, x)]; gs += g; gn++; w->gmin[k] = fminf(w->gmin[k], g); }
        if (P && P->n > 0 && !gn) { gs = P->grade[pidx(w, 0.5f * (w->s[k] + w->s[k+1]))]; gn = 1; }
        float gr = gn ? gs / gn : 0, cs = 1 / sqrtf(1 + gr * gr);
        w->c[k] = G * (V->crr * cs + gr * cs);
        w->kh[k] = w->ka * w->h[k];
        // interior hard-cap points
        int n = 0;
        for (int p = (int)floorf(w->s[k] / w->ds) + 1; p * w->ds < w->s[k+1] - 0.5f && p < w->np && n < PL_INT_MAX; p++) {
            float x = p * w->ds; if (x <= w->s[k] + 0.5f) continue;
            float cap = fmaxf(capat(w, w->capH, x), sqrtf(fmaxf(0.0f, v0 * v0 - 1.8f * S.brake_max * x)));   // late: what max braking allows
            if (cap < vfree + 1) { w->intf[k][n] = (x - w->s[k]) / w->h[k]; w->inte[k][n] = 0.5f * cap * cap; n++; }
        }
        w->nint[k] = n;
    }

    // --- speed bounds per stage
    const float vroll = set + kmh(M.over_kmh + M.schwung_kmh);
    float vreach = v0, vlo = v0;
    float lead_lo = 1e3f;
    if (w->lead) { float mn = 1e3f; for (int i = 0; i <= w->lnst && i * w->ldt <= 60; i++) mn = fminf(mn, w->lv[i]); lead_lo = fmaxf(0, mn * 0.85f - 1.0f); }
    out->late = v0 > vcap0 + 0.05f;   // the start is above the hard envelope: braking up to brake_max (harsh_pen above brake_plan)
    for (int k = 0; k <= K; k++) {
        float x = w->s[k];
        float vt = fminf(set, capat(w, w->lim5, x));
        w->vtr[k] = vt;
        if (k == 0) { w->hi[0] = w->lo[0] = v0; w->vcap[0] = fminf(capat(w, w->env, 0), capat(w, w->lim5, 0)); continue; }
        // forward reachability (upper bound): traction up to the throttle cap, or gravity
        float e = 0.5f * vreach * vreach, fe = w->tb_fe[tbin(vreach)];
        float ecoast = step_e(w, k - 1, e, 0), etr = fminf(step_e(w, k - 1, e, fe / m), 0.5f * vt * vt);
        vreach = sqrtf(2 * fmaxf(0.5f * S.v_floor * S.v_floor, fmaxf(ecoast, etr) + 0.5f));
        float cap = fminf(capat(w, w->env, x), vroll);
        cap = fminf(cap, capat(w, w->lim5, x));
        float vlate = sqrtf(fmaxf(0, v0 * v0 - 1.8f * S.brake_max * x));   // 0.9 x brake_max: slack for the grid
        w->vcap[k] = cap;
        float hi = fminf(cap, vreach);
        if (vlate > hi && hi < vreach) { hi = fminf(vlate, vreach); out->late = 1; }
        hi = fmaxf(hi, S.v_floor);
        // lowest: set - under, relaxed by physics (full traction from the chain), caps and a lead
        float el = 0.5f * vlo * vlo, fl = w->tb_fe[tbin(vlo)];
        float a_up = fminf(M.accel_comf, fl / m - w->c[k-1] - w->ka * el);
        float vstep = sqrtf(2 * fmaxf(0, el + w->h[k-1] * a_up));
        float lo = fminf(set - kmh(M.under_kmh), vstep);
        lo = fminf(lo, 0.9f * capat(w, w->env2, x));                  // a gentle approach to a cap ahead may go under the band
        if (hi < vt - 0.15f) lo = fminf(lo, hi - ((!w->lead && capat(w, w->env, x) < vt - 0.15f) ? fmaxf(1.0f, 0.08f * hi) : fmaxf(3.0f, 0.25f * hi)));   // a bend/junction cap is reached (~8 % / 1 m/s under), not ~25 % under; a climb or a lead keeps the loose floor
        lo = fminf(lo, hi - 0.3f);
        lo = fminf(lo, lead_lo);
        lo = fmaxf(lo, fminf(S.v_floor, hi - 0.1f));
        vlo = fmaxf(lo, fminf(vstep, hi));
        w->hi[k] = hi; w->lo[k] = lo;
    }
    // node grids (uniform in e; one node exactly on the throttle cap when it is inside)
    for (int k = 0; k <= K; k++) {
        float el = 0.5f * w->lo[k] * w->lo[k], eh = 0.5f * w->hi[k] * w->hi[k];
        w->etop[k] = eh;
        w->jmin[k] = 0;
        if (k == 0 || eh - el < 0.05f) { w->nn[k] = 1; w->elo[k] = eh; w->de[k] = 1; continue; }
        const float el0 = el;
        w->nn[k] = NN; float de = (eh - el) / (NN - 1);
        float et = 0.5f * w->vtr[k] * w->vtr[k];
        // the uniform part is shifted down (at most one step) so one node lands exactly on the throttle cap; the top node stays on hi
        if (et > el + 1e-3f && et < eh - 1e-3f) el = et - ceilf((et - el) / de - 1e-4f) * de;
        w->elo[k] = el; w->de[k] = de; w->jmin[k] = el < el0 - 1e-3f ? 1 : 0;
    }

    // --- reasons to coast (no pulse-and-glide, owner 30-10): from stage k, rolling (in gear or neutral) or braking below the throttle cap
    //     needs a descent within coast_reason_s of driving (grade <= -1 %; also the far side of a crest), the segment itself downhill
    //     (<= -0.3 %), or a cap whose gentle envelope (coast_relax) is under the throttle cap within this stage. A lead is a reason per
    //     label (lead_reason below: only once the gap is within rolling reach). M.pulse_glide (Eco only) lifts the rule.
    {
        const int pg = M.pulse_glide && in->drive_mode == PL_ECO;
        for (int k = 0; k <= K; k++) {
            float reach = w->s[k] + fmaxf(200.0f, set * S.coast_reason_s);
            // a descent the truck can ROLL to from the throttle cap without falling under the band (a crest: the last stretch of the
            // climb; on the level a few hundred metres before the descent; never at the foot of a climb that is followed by a descent).
            // Gated on the label's speed when consumed (see the transition): it only lets the truck coast into a descent while it is
            // still above the under-band, so a climb it cannot hold at set speed is pulled at full load, not given up.
            int cr = 0;
            {
                float e = 0.5f * w->vtr[k] * w->vtr[k], elo = 0.5f * fmaxf(0.0f, set - kmh(M.under_kmh)) * fmaxf(0.0f, set - kmh(M.under_kmh));
                for (int m2 = k; !cr && m2 < K && w->s[m2] < reach; m2++) {
                    if (w->gmin[m2] <= -0.01f) { cr = 1; break; }
                    e = step_e(w, m2, e, 0);
                    if (e < elo) break;
                }
            }
            w->crest[k] = (uint8_t)cr;
            int r = pg;
            if (!r && k < K && w->c[k] <= G * (V->crr - 0.02f)) r = 1;   // downhill here: only a descent the coast accelerates on (-2 %), not a mild grade where coasting still loses speed
            float xe = k < K ? w->s[k+1] : w->s[k];
            for (float x = w->s[k]; !r && x <= xe + 0.01f && x < w->np * w->ds; x += w->ds) if (capat(w, w->env2, x) < w->vtr[k] - 0.15f) r = 1;
            if (!r && capat(w, w->env2, xe) < w->vtr[k] - 0.15f) r = 1;
            w->reason[k] = (uint8_t)r;
        }
    }

    // --- costs
    w->gamma = gamma0;
    w->beta = pl_beta_calibrated(V, fminf(set, w->vtr[0] > 1 ? fmaxf(w->vtr[0], set * 0.5f) : set)) * M.beta_scale;
    const float beta = w->beta, gamma = w->gamma, swP = M.switch_pen_g, rpP = M.repull_pen_g, hP = swP + 0.5f * rpP + 0.25f * M.neutral_pen_g;
    const float tol = 0.01f;   // m/s on the throttle cap
    // the START may sit a little over the cap (the cruise's overshoot, a measured speed, the state one replan later): within 0.5 km/h it
    // counts as on it, else a climb made the plan infeasible (no throttle, the coast falls under the band; shadow 30-10, 85.04 at set 85
    // in Power) and a 0.2 km/h overshoot on the level was a reason to roll (a pulse-and-glide through the back door)
    const float tol0 = kmh(0.5f);
    // a lead needs the full braking range only when it is tense (braking / short TTC), we are near or inside the wanted gap, or the
    // prediction brings it to a stop; a calm lead far ahead is planned like a cap (keeps the replan ~3x cheaper)
    bool lead_hard = false;
    if (w->lead) {
        float vl0 = fmaxf(0.0f, in->lead.v), want0 = S.d0 + S.time_gap * (vl0 + w->tw * fmaxf(0.0f, v0 - vl0));
        lead_hard = w->tw > 0.05f || in->lead.gap < want0 + S.cushion_s * fmaxf(vl0, 3.0f) + 10 || w->lv[w->lnst] < 0.05f || in->lead.a < -S.tense_dec_lo;
    }
    const float a_brk_lim = lead_hard ? V->brake_decel_max : (out->late ? S.brake_max : fminf(S.brake_max, S.brake_plan + 0.3f));
    out->beta = beta; out->gamma = gamma;

    memset(w->cnt, 0, sizeof(uint8_t) * (K + 1) * PL_NODES);
    {
        Lab &l0 = w->lab[0][0][0];
        memset(&l0, 0, sizeof l0);
        l0.cost = 0; l0.t = 0; l0.tov = in->over_used_s; l0.pred = -1; l0.eland = 0.5f * v0 * v0;
        l0.eref = 0.5f * v0 * v0;
        l0.cls = (uint8_t)(in->neutral0 ? PL_NEUTRAL : (in->cls0 >= 0 && in->cls0 <= PL_BRAKE ? in->cls0 : PL_GAS));
        l0.gear = (uint8_t)(in->gear0 > 0 ? in->gear0 : 0);
        w->cnt[0][0] = 1;
    }
    const float dwell = S.neutral_min_s;
    int ntrans = 0;

    for (int k = 0; k < K; k++) {
        const float hk = w->h[k];
        // a descent the coast accelerates on (-2 %): gravity overspeed is free (the plan may run up to set + over for the whole
        // descent), so the overspeed time buckets (over_time_s / schwung_s) do not count it. Above set + over / limit + 5 the hard
        // caps still hold (the truck must brake back).
        const bool grav_k = w->c[k] <= G * (V->crr - 0.02f);
        for (int i = 0; i < w->nn[k]; i++) {
            int nl = w->cnt[k][i]; if (!nl) continue;
            const float ei = node_e(w, k, i), vi = sqrtf(2 * fmaxf(ei, 0.01f));
            Lab *LI = w->lab[k][i];
            // --- j window
            const float fe = w->tb_fe[tbin(vi)];
            const float tk = k == 0 ? tol0 : tol;
            float e_up = step_e(w, k, ei, (vi < w->vtr[k] + tk ? fe * 1.08f : 0) / m) + 0.5f;
            e_up = fmaxf(e_up, step_e(w, k, ei, 0) + 0.5f);
            float e_dn = ei - hk * a_brk_lim - 0.5f;
            // interior caps: e_i(1-f) + e_j f <= E
            for (int q = 0; q < w->nint[k]; q++) { float f = w->intf[k][q]; e_up = fminf(e_up, (w->inte[k][q] - ei * (1 - f)) / f); }
            int j0, j1;
            if (w->nn[k+1] == 1) { j0 = 0; j1 = node_e(w, k + 1, 0) <= e_up && node_e(w, k + 1, 0) >= e_dn ? 0 : -1; }
            else {
                j0 = (int)ceilf((e_dn - w->elo[k+1]) / w->de[k+1]); j1 = (int)floorf((e_up - w->elo[k+1]) / w->de[k+1]);
                if (j0 < w->jmin[k+1]) j0 = w->jmin[k+1];
                if (j1 >= w->nn[k+1] - 2) j1 = w->etop[k+1] <= e_up ? w->nn[k+1] - 1 : w->nn[k+1] - 2;
                if (j0 > w->nn[k+1] - 1) j0 = w->nn[k+1] - 1;
            }
            bool start_neutral = (k == 0 && LI[0].cls == PL_NEUTRAL);
            // a descent ahead (crest) is a reason to roll only while the truck is still above the under-band: below that a climb must be
            // pulled at full load (no giving up the climb for a descent that is coming later). The downhill-here and cap reasons stay
            // unconditional (the cap's envelope already under the cap may hold below the band; a descent under the truck rolls by gravity).
            const bool crest_k = w->crest[k] && vi >= set - kmh(M.under_kmh);
            const bool free_k = w->reason[k] || crest_k || vi > w->vtr[k] + tk || vi > capat(w, w->env, w->s[k]) + 0.01f || start_neutral;   // may roll/brake from here (no P&G); a start over a hard cap must brake back under it
            bool lf[PL_LABELS], lfn[PL_LABELS], lfo[PL_LABELS], lnp[PL_LABELS]; float lvl[PL_LABELS];   // may roll/brake; may hold; no pull past the lead
            for (int l = 0; l < nl; l++) {
                int lr = w->lead ? lead_reason(w, S, M, w->s[k], vi, LI[l].t) : 0;
                lf[l] = free_k || (lr & 1); lfn[l] = w->reason[k] || crest_k || vi > w->vtr[k] + tk || (lr & 1); lfo[l] = free_k || lr;
                // following without another reason: no pulling above the lead's speed (it would only buy a roll-in later: a cycle)
                lnp[l] = !free_k && (lr & 2); lvl[l] = 1e3f;
                if (lnp[l]) { float xl; lead_at(w, LI[l].t, &xl, &lvl[l]); }
            }
            if (!free_k && !w->lead && w->nn[k+1] > 1) {
                float e_keep = fminf(fminf(ei, 0.5f * w->vtr[k+1] * w->vtr[k+1]), step_e(w, k, ei, fe / m)) - 0.5f;   // (never above the throttle cap: a start just over it)
                int jk = (int)floorf((e_keep - w->elo[k+1]) / w->de[k+1]);
                if (jk > j0) j0 = jk;
            }
            float dwell_left0 = start_neutral ? fmaxf(0, dwell - in->neutral_s0) : 0;
            // coarse stages are never a cap's run-up (those are fine): braking there only holds a descent; without a lead or a late
            // start it needs at most brake_coarse beyond the in-gear coast (cuts the window ~3x on descents)
            if (!out->late && !lead_hard && hk > hf + 0.5f && w->nn[k+1] > 1) {
                float ec = step_e(w, k, ei, -w->tb_fd[tbin(vi)] / m) - hk * S.brake_coarse - 0.5f;
                int jb = (int)ceilf((ec - w->elo[k+1]) / w->de[k+1]);
                if (jb > j0) j0 = jb;
            }
            // --- in gear: inverse dynamics to every target node
            for (int j = j0; j <= j1; j++) {
                const float ej = node_e(w, k + 1, j); if (ej < 0) continue;
                const float vj = sqrtf(2 * fmaxf(ej, 0.01f));
                // interior cap check (exact per point)
                bool bad = false;
                for (int q = 0; q < w->nint[k]; q++) { float f = w->intf[k][q]; if (ei * (1 - f) + ej * f > w->inte[k][q] + 0.05f) { bad = true; break; } }
                if (bad) continue;
                const float af = (ej - ei) / hk + w->c[k] + w->ka * 0.5f * (ei + ej);   // force per kg needed
                const float F = af * m, vb = 0.5f * (vi + vj), dt = 2 * hk / fmaxf(0.2f, vi + vj);
                int gear, cls, ret = 0; float fuel = 0, extra = 0, sdec = 0, akin = 0; bool need_free = false, need_follow = false, part = false, fullload = false;
                const float etr1 = 0.5f * w->vtr[k+1] * w->vtr[k+1], rtol = w->de[k+1] + 0.01f;
                const int vbb = tbin(vb);
                int gc = w->tb_gc[vbb];
                float Fd = w->tb_fd[vbb];
                if (F > -Fd) {
                    if (vj > w->vtr[k+1] + tol || vi > w->vtr[k] + tk) continue;     // no throttle above min(set, limit + 5)
                    gear = gear_fast(w, V, vbb, F);
                    float fmx = w->tb_fm[vbb][gear-1];
                    if (F > 0 && F > fmx) continue;
                    // without a reason, under the throttle cap the plan must get back up to it at accel_comf at least, or pull at full
                    // load (the truck cannot do better): no part-throttle sag and re-pull (a pulse-and-glide in gear: the fit's throttle
                    // offset would pay for it), no saving the acceleration for a climb (PreSpeed), the set speed exactly on the level
                    // (the nodes within one node of what full load reaches count as the full-load pull: where full load gives less than
                    // accel_comf, on a climb the truck cannot hold, the node just under that reach is part throttle by the grid only, and
                    // with the 3 % band and the ratchet no transition was left: shadow 30-10, 79-83 km/h on 3 % before a crest, a Renault
                    // at 40 t up a 3 % climb. e_full is the true full-load reach: the 0.9 and the -de slack it used to have let the plan
                    // drift one node under full load every stage, sagging ~10 % below the equilibrium on climbs)
                    const float e_full = step_e(w, k, ei, fe / m) - 0.01f;
                    if (F < 0.97f * fmx && ej < e_full) {
                        // the pull must get back toward the cap at accel_comf, but never faster than the truck can at full load: a weak
                        // or heavy truck's full-load acceleration can sit below accel_comf, and then full load is the pull (the fixed
                        // accel_comf target alone would misread it as a sag, prune it, and leave the plan infeasible between caps)
                        float ewant = fminf(etr1, fminf(ei + hk * M.accel_comf, e_full + 0.01f)) - w->de[k+1] - 0.01f;
                        if (ej < ewant) need_follow = true;
                        part = true;                          // the ratchet applies per label below
                    } else fullload = true;
                    fuel = fuel_rate(V, pl_rpm(V, gear, vb), p_ind(V, gear, vb, F), 0) * dt;
                    need_free = need_free || fuel <= 1e-4f;
                    // light throttle (< 25 % of full load, where the fit's g(thr) turns affine) counts as rolling for the switch penalties: with the fit's concave g(thr) a
                    // full / near-zero throttle alternation is cheaper than a steady pull, and that is the cycling we do not want
                    cls = F > 0.25f * fmx ? PL_GAS : PL_COAST;
                } else {
                    need_free = true;
                    gear = gc; cls = PL_BRAKE;
                    float Fb = -F - Fd; akin = (ei - ej) / hk;
                    if (akin > a_brk_lim + 1e-3f) continue;
                    if (akin < S.brake_firm && vi <= w->vtr[k] + tk && !w->lead && w->c[k] + w->ka * ei + Fd / m > 0.02f) continue;
                    for (ret = 1; ret <= V->ret_levels; ret++) if (pl_ret_force(V, ret, vb) >= Fb) break;
                    if (ret > V->ret_levels) { ret = V->ret_levels; float Fs = Fb - pl_ret_force(V, ret, vb); sdec = Fs / m; if (sdec > V->brake_decel_max) continue; extra += S.brake_pen * Fs * hk; }
                    extra += 0.05f * u * ret;
                    if (akin > S.brake_plan) extra += S.harsh_pen_g * (akin - S.brake_plan) * hk / 50;
                    if (V->ret_levels <= 0) { sdec = Fb / m; extra += S.brake_pen * Fb * hk; }
                }
                if (vj > w->vcap[k+1]) extra += S.overcap_pen * (vj - w->vcap[k+1]) * dt;   // late: above the hard cap (limit drop, late bend)
                float base = fuel + beta * dt + M.eps_g * fabsf(vj - vi) + M.w_under * dt * fmaxf(0, set - vb) * fmaxf(0, set - vb) + extra;
                ntrans++;
                // lead mid-point position/time (per label time)
                float vm = sqrtf(fmaxf(0.01f, ei + ej)), dtm = hk / fmaxf(0.2f, vi + vm);
                for (int l = 0; l < nl; l++) {
                    if ((need_free && !lf[l] && !lead_hard) || (need_follow && !lfo[l])) continue;
                    if (lnp[l] && cls != PL_BRAKE && vj > vi + 0.01f && vj > lvl[l] + 0.15f) continue;
                    const Lab &A = LI[l];
                    // the ratchet (no pulse-and-glide): without a reason, part throttle may not take the speed under the highest one
                    // since the last reason (capped at the throttle cap, and at the lead's speed while following) by more than a node
                    float eref_c = fminf(A.eref, etr1);
                    if (lnp[l]) eref_c = fminf(eref_c, 0.5f * lvl[l] * lvl[l]);
                    if (part && !lf[l] && ej < eref_c - rtol) continue;
                    float eref_new = (need_free || fullload || (lf[l] && ej < eref_c - rtol)) ? ej : fmaxf(eref_c, ej);
                    Lab c; c.pred = ((k * PL_NODES) + i) * PL_LABELS + l;
                    float cc = A.cost + base;
                    if (A.cls == PL_NEUTRAL) cc += 0.5f * M.neutral_pen_g + 20 * u * dwell_left0 + (cls == PL_GAS ? rpP : 0);   // engage (early: dwell)
                    else cc += sw_cost(A.cls, cls, swP, rpP);
                    if (A.gear && gear != A.gear && cls == PL_GAS) cc += M.shift_pen_g;
                    float tov = A.tov;
                    if (vb > set + 0.05f && !grav_k) { tov += dt; if ((tov > overT && vj > set + 0.05f && akin < S.brake_firm) || (tov > schT && vj > vover && akin < S.brake_firm)) continue; }
                    // refill only off a descent (grade > -0.3 %): at/below set on a descent is a hold, not an excursion end, so it
                    // must not buy another overspeed (the plan would dip below set to refill, then roll back up)
                    else if (vb <= set + 0.05f) tov = w->c[k] > G * (V->crr - 0.003f) ? fmaxf(0, tov - S.over_recover * dt) : tov;
                    float t = A.t + dt;
                    if (w->lead) {
                        cc += lead_cost(w, S, M, w->s[k] + 0.5f * hk, vm, A.t + dtm, dt, 0, 0, 0);
                        cc += lead_cost(w, S, M, w->s[k+1], vj, t, dt, 1, 0, 0);
                    }
                    c.cost = cc; c.t = t; c.tov = tov; c.force = F; c.fuel = fuel; c.eland = ej; c.eref = eref_new;
                    c.cls = (uint8_t)cls; c.gear = (uint8_t)gear; c.act = (uint8_t)(cls == PL_BRAKE ? PL_BRAKE : fuel > 1e-4f ? PL_GAS : PL_COAST); c.span = 1; c.ret = (uint8_t)ret; c.nwait = 0; c.sdec = sdec;
                    insert(w, k + 1, j, c, hP);
                }
            }
            // --- coast in gear (fuel cut): forward step, snapped (the energy difference charged at gamma)
            {
                int gc = w->tb_gc[tbin(vi)];
                float Fd = drag_force(V, gc, vi);
                float ej = step_e(w, k, ei, -Fd / m);
                float ej2 = step_e(w, k, ei, -drag_force(V, gc, sqrtf(2 * fmaxf(0.01f, 0.5f * (ei + ej)))) / m);
                int j = snap(w, k + 1, ej2);
                bool bad = j < 0;
                for (int q = 0; !bad && q < w->nint[k]; q++) { float f = w->intf[k][q]; if (ei * (1 - f) + ej2 * f > w->inte[k][q] + 0.05f) bad = true; }
                if (!bad) {
                    float en = node_e(w, k + 1, j), vj = sqrtf(2 * fmaxf(0.01f, ej2)), vb = 0.5f * (vi + vj), dt = 2 * hk / fmaxf(0.2f, vi + vj);
                    float fuel = fuel_rate(V, pl_rpm(V, gc, vb), p_ind(V, gc, vb, -drag_force(V, gc, vb)), 0) * dt;
                    float base = fuel + beta * dt + M.eps_g * fabsf(vj - vi) + M.w_under * dt * fmaxf(0, set - vb) * fmaxf(0, set - vb) + gamma * m * (en - ej2);
                    if (vj > w->vcap[k+1]) base += S.overcap_pen * (vj - w->vcap[k+1]) * dt;
                    float vjn = sqrtf(2 * fmaxf(0.01f, en));
                    for (int l = 0; l < nl; l++) {
                        if (!lf[l]) continue;
                        const Lab &A = LI[l];
                        Lab c; c.pred = ((k * PL_NODES) + i) * PL_LABELS + l;
                        float cc = A.cost + base;
                        if (A.cls == PL_NEUTRAL) cc += 0.5f * M.neutral_pen_g + 20 * u * dwell_left0; else cc += sw_cost(A.cls, PL_COAST, swP, rpP);
                        float tov = A.tov;
                        if (vb > set + 0.05f && !grav_k) { tov += dt; if ((tov > overT && vjn > set + 0.05f) || (tov > schT && vjn > vover)) continue; }
                        else if (vb <= set + 0.05f) tov = w->c[k] > G * (V->crr - 0.003f) ? fmaxf(0, tov - S.over_recover * dt) : tov;
                        float t = A.t + dt;
                        if (w->lead) cc += lead_cost(w, S, M, w->s[k+1], vjn, t, dt, 1, 0, 0);
                        c.cost = cc; c.t = t; c.tov = tov; c.force = -Fd; c.fuel = fuel; c.eland = ej2; c.eref = en;
                        c.cls = PL_COAST; c.gear = (uint8_t)gc; c.act = PL_COAST; c.span = 1; c.ret = 0; c.nwait = 0; c.sdec = 0;
                        insert(w, k + 1, j, c, hP);
                        ntrans++;
                    }
                }
            }
            // --- neutral: entering coasts exactly (closed form) through the dwell and lands at the first stage past it, in neutral
            //     (class PL_NEUTRAL); a label in neutral continues one stage at a time (while it still has a reason) or engages through
            //     the in-gear transitions above (exit penalty). The start in neutral first runs out its remaining dwell.
            if ((M.neutral && (free_k || w->lead)) || start_neutral || LI[0].cls == PL_NEUTRAL || (nl > 1 && (LI[1].cls == PL_NEUTRAL || (nl > 2 && (LI[2].cls == PL_NEUTRAL || (nl > 3 && LI[3].cls == PL_NEUTRAL)))))) {
                float idle_g = fuel_rate(V, V->idle_rpm, 0, 1);
                float cost[PL_LABELS], t[PL_LABELS], tov[PL_LABELS], tn[PL_LABELS]; bool alive[PL_LABELS], landed[PL_LABELS]; int any = 0;
                for (int l = 0; l < nl; l++) {
                    const Lab &A = LI[l];
                    bool fromN = A.cls == PL_NEUTRAL;
                    landed[l] = false;
                    alive[l] = fromN ? (lfn[l] || (k == 0 && dwell_left0 > 0)) : (M.neutral && lf[l]); if (!alive[l]) continue;
                    cost[l] = A.cost + (fromN ? 0 : 0.5f * M.neutral_pen_g); t[l] = A.t; tov[l] = A.tov;
                    tn[l] = fromN ? (k == 0 ? in->neutral_s0 : dwell) : 0; any = 1;
                }
                float e = ei, v = vi, fuel_sum = 0;
                for (int mm = k + 1; mm <= K && any; mm++) {
                    int sg = mm - 1;
                    float en = step_e(w, sg, e, 0);
                    bool bad = en < 0.5f * S.v_floor * S.v_floor;
                    for (int q = 0; !bad && q < w->nint[sg]; q++) { float f = w->intf[sg][q]; if (e * (1 - f) + en * f > w->inte[sg][q] + 0.05f) bad = true; }
                    float vn = sqrtf(2 * fmaxf(0.01f, en));
                    if (vn > w->hi[mm] + 0.3f || vn < w->lo[mm] - 0.05f) bad = true;   // (a running roll gets a little slack)
                    if (bad) break;
                    float hs = w->h[sg], dt = 2 * hs / fmaxf(0.2f, v + vn), vb = 0.5f * (v + vn);
                    float base = idle_g * dt + beta * dt + M.eps_g * fabsf(vn - v) + M.w_under * dt * fmaxf(0, set - vb) * fmaxf(0, set - vb);
                    if (vn > w->vcap[mm]) base += S.overcap_pen * (vn - w->vcap[mm]) * dt;
                    fuel_sum += idle_g * dt;
                    float vm = sqrtf(fmaxf(0.01f, e + en)), dtm = hs / fmaxf(0.2f, v + vm);
                    int jn = snap(w, mm, en);
                    any = 0;
                    for (int l = 0; l < nl; l++) {
                        if (!alive[l] || landed[l]) continue;
                        // a NEW roll keeps 0.15 m/s under hi through its dwell: the real state differs a little at the next replan, and
                        // a roll that has to be ended early is the short neutral the dwell forbids
                        if (LI[l].cls != PL_NEUTRAL && vn > w->hi[mm] - 0.15f) { alive[l] = false; continue; }
                        cost[l] += base;
                        if (vb > set + 0.05f && !(w->c[sg] <= G * (V->crr - 0.02f))) { tov[l] += dt; if (tov[l] > overT || (fmaxf(v, vn) > vover && tov[l] > schT)) { alive[l] = false; continue; } }
                        else if (vb <= set + 0.05f) tov[l] = w->c[sg] > G * (V->crr - 0.003f) ? fmaxf(0, tov[l] - S.over_recover * dt) : tov[l];
                        if (w->lead) { cost[l] += lead_cost(w, S, M, w->s[sg] + 0.5f * hs, vm, t[l] + dtm, dt, 0, 0, 0) + lead_cost(w, S, M, w->s[mm], vn, t[l] + dt, dt, 1, 0, 0); }
                        t[l] += dt; tn[l] += dt; any = 1;
                        if (tn[l] >= dwell || mm == K) {
                            landed[l] = true;
                            if (jn < 0) continue;
                            float en_node = node_e(w, mm, jn);
                            Lab c; c.pred = ((k * PL_NODES) + i) * PL_LABELS + l;
                            c.cost = cost[l] + gamma * m * (en_node - en);
                            c.t = t[l]; c.tov = tov[l]; c.force = 0; c.fuel = fuel_sum; c.eland = en; c.eref = en_node;
                            c.cls = PL_NEUTRAL; c.gear = 0; c.act = PL_NEUTRAL; c.span = (uint8_t)(mm - k); c.ret = 0; c.sdec = 0;
                            c.nwait = 0;
                            insert(w, mm, jn, c, hP);
                            ntrans++;
                        }
                    }
                    e = en; v = vn;
                    any = 0; for (int l = 0; l < nl; l++) if (alive[l] && !landed[l]) any = 1;
                }
            }
        }
    }
    out->transitions = ntrans;
    out->units = u;

    // --- the plain cruise over the same stages (for "fuel saved"): hold min(set, limit + 5) in gear, brake only for the hard caps
    {
        float e = 0.5f * v0 * v0, t = 0, fuel = 0, cost = 0;
        for (int k = 0; k < K; k++) {
            float hk = w->h[k], vi = sqrtf(2 * fmaxf(0.01f, e));
            float vt = fminf(w->vtr[k+1], capat(w, w->env, w->s[k+1])), et = 0.5f * vt * vt;
            float F = m * ((et - e) / hk + w->c[k] + w->ka * 0.5f * (e + et));
            float vb0 = 0.5f * (vi + vt);
            int gc = pl_gear_pred(V, vb0, 0); float Fd = drag_force(V, gc, vb0), extra = 0, fu = 0, en = et;
            if (F > -Fd) {
                int g = F > 0 ? pl_gear_pred(V, vb0, F) : gc;
                float fm = pl_force_max(V, g, vb0);
                if (F > fm) { F = fm; en = step_e(w, k, e, F / m); }
                float vj = sqrtf(2 * fmaxf(0.01f, en)), vb = 0.5f * (vi + vj), dt = 2 * hk / fmaxf(0.2f, vi + vj);
                fu = fuel_rate(V, pl_rpm(V, g, vb), p_ind(V, g, vb, F), 0) * dt;
            } else {
                float Fb = -F - Fd, Fr = pl_ret_force(V, V->ret_levels, vb0);
                if (Fb > Fr) extra += S.brake_pen * (Fb - Fr) * hk;
            }
            float vj = sqrtf(2 * fmaxf(0.01f, en)), vb = 0.5f * (vi + vj), dt = 2 * hk / fmaxf(0.2f, vi + vj);
            cost += fu + beta * dt + M.eps_g * fabsf(vj - vi) + M.w_under * dt * fmaxf(0, set - vb) * fmaxf(0, set - vb) + extra;
            fuel += fu; t += dt; e = en;
        }
        cost -= gamma * m * fminf(e, 0.5f * w->vtr[K] * w->vtr[K]);
        out->base_cost = cost; out->base_fuel_g = fuel; out->base_time_s = t;
    }

    // --- best terminal label: cost - gamma * m * e (energy above the throttle cap is worth nothing: it is braked away or wasted)
    int bk = K, bj = -1, bl = -1; float best = 1e30f;
    for (int kk = K; kk >= 1 && bj < 0; kk--) {      // the last stage with a label (a dead end only if nothing reaches K)
        float etr = 0.5f * w->vtr[kk] * w->vtr[kk];
        for (int j = 0; j < w->nn[kk]; j++) for (int l = 0; l < w->cnt[kk][j]; l++) {
            float e = node_e(w, kk, j), val = w->lab[kk][j][l].cost - gamma * m * fminf(e, etr);
            if (val < best) { best = val; bj = j; bl = l; bk = kk; }
        }
    }
    int lu = 0; for (int k = 0; k <= K; k++) for (int j = 0; j < w->nn[k]; j++) lu += w->cnt[k][j];
    out->labels_used = lu;
    if (bj < 0) {
        out->ok = 0; out->n = 1;
        out->st[0].s = 0; out->st[0].v = v0;
        out->v_target = out->v_next = out->v_next2 = fminf(v0, w->hi[1 < K ? 1 : 0]);
        out->act = PL_BRAKE; out->decel = S.brake_max; out->v_overspeed_ok = v0; out->v_throttle_cap = w->vtr[0];
        return;
    }
    out->ok = 1;
    // --- backtrack into stage arrays (multi-stage neutral entries re-simulated in between)
    int chain_k[PL_MAX_STAGES + 1], chain_j[PL_MAX_STAGES + 1], chain_l[PL_MAX_STAGES + 1], nc = 0;
    { int k = bk, j = bj, l = bl;
      while (true) { chain_k[nc] = k; chain_j[nc] = j; chain_l[nc] = l; nc++; const Lab &A = w->lab[k][j][l]; if (A.pred < 0) break;
          int p = A.pred; l = p % PL_LABELS; p /= PL_LABELS; j = p % PL_NODES; k = p / PL_NODES; } }
    out->n = bk + 1;
    for (int k = 0; k <= bk; k++) { out->st[k].s = w->s[k]; out->st[k].hi = w->hi[k]; out->st[k].lo = w->lo[k]; out->st[k].gap = NAN; out->st[k].gap_want = NAN; }
    out->st[0].v = v0; out->st[0].t = 0; out->st[0].act = (uint8_t)w->lab[0][0][0].cls; out->st[0].gear = (uint8_t)in->gear0;
    float cfuel = 0, ctime = 0;
    for (int c = nc - 1; c >= 0; c--) {
        int k = chain_k[c], j = chain_j[c]; const Lab &A = w->lab[k][j][chain_l[c]];
        if (A.pred < 0) continue;
        int pk = k - A.span;
        if (A.act == PL_NEUTRAL && A.span > 1) {
            float e = out->st[pk].v * out->st[pk].v * 0.5f, t = out->st[pk].t;
            for (int mm = pk + 1; mm < k; mm++) {
                float en = step_e(w, mm - 1, e, 0), v = sqrtf(2 * fmaxf(0.01f, e)), vn = sqrtf(2 * fmaxf(0.01f, en));
                t += 2 * w->h[mm-1] / fmaxf(0.2f, v + vn);
                PlStage &s = out->st[mm]; s.v = vn; s.t = t; s.act = PL_NEUTRAL; s.gear = 0; s.force = 0; s.decel = 0; s.ret = 0; s.fuel_g = 0;
                e = en;
            }
        }
        PlStage &s = out->st[k];
        s.v = sqrtf(2 * fmaxf(0.0f, node_e(w, k, j))); s.t = A.t; s.force = A.force; s.act = A.act; s.gear = A.gear; s.ret = A.ret;
        s.fuel_g = A.fuel; s.decel = A.act == PL_BRAKE ? fmaxf(0, (out->st[pk].v * out->st[pk].v - s.v * s.v) / (2 * (w->s[k] - w->s[pk]))) : 0;
        cfuel += A.fuel;
    }
    ctime = out->st[bk].t;
    out->fuel_g = cfuel; out->time_s = ctime;
    out->cost = best; out->c_fuel = cfuel; out->c_time = beta * ctime;
    out->c_term = -gamma * m * fminf(0.5f * out->st[bk].v * out->st[bk].v, 0.5f * w->vtr[bk] * w->vtr[bk]);
    out->c_pen = w->lab[bk][bj][bl].cost - cfuel - out->c_time;   // eps, under, switches, brake, lead, snaps
    if (w->lead) {
        float mg = 1e9f;
        for (int k = 1; k <= bk; k++) {
            float g, wa; lead_cost(w, S, M, w->s[k], out->st[k].v, out->st[k].t, 0, 1, &g, &wa);
            out->st[k].gap = g; out->st[k].gap_want = wa; if (out->st[k].t <= S.lead_fade_s) mg = fminf(mg, g - wa);
        }
        out->min_gap_margin = mg < 1e8f ? mg : NAN;
    }
    // --- commands
    const Lab &first = w->lab[chain_k[nc - 2 >= 0 ? nc - 2 : 0]][chain_j[nc - 2 >= 0 ? nc - 2 : 0]][chain_l[nc - 2 >= 0 ? nc - 2 : 0]];
    out->act = out->st[1 <= bk ? 1 : 0].act;
    out->force = first.force; out->ret_level = out->act == PL_BRAKE ? first.ret : 0; out->service_decel = out->act == PL_BRAKE ? first.sdec : 0;
    out->decel = out->act == PL_BRAKE ? out->st[1].decel : 0;
    out->gear_pred = out->st[1 <= bk ? 1 : 0].gear;
    out->neutral_req = out->act == PL_NEUTRAL;
    out->neutral_hold_s = out->neutral_req ? (in->neutral0 ? fmaxf(0, dwell - in->neutral_s0) : dwell) : 0;
    out->v_next = out->st[bk >= 1 ? 1 : 0].v; out->v_next2 = out->st[bk >= 2 ? 2 : bk].v;
    {
        float sx = fmaxf(w->s[bk >= 1 ? 1 : 0], v0 * 2.0f);
        float vt = out->st[bk].v;
        for (int k = 1; k <= bk; k++) if (w->s[k] >= sx) { float f = (sx - w->s[k-1]) / fmaxf(0.1f, w->s[k] - w->s[k-1]); vt = out->st[k-1].v + f * (out->st[k].v - out->st[k-1].v); break; }
        out->v_target = vt;
    }
    out->v_overspeed_ok = fmaxf(out->v_next, out->v_next2) + kmh(0.3f);
    out->v_throttle_cap = w->vtr[0];
    // --- why (head of the plan)
    {
        float capmin = 1e3f; for (int k = 1; k <= bk && w->s[k] <= 400; k++) capmin = fminf(capmin, capat(w, w->env, w->s[k]));
        float g0 = P && P->n > 0 ? P->grade[0] : 0, gmax = -1, gmin = 1;
        if (P && P->n > 0) for (int i = 0; i < P->n && i * P->ds < 600; i++) { gmax = fmaxf(gmax, P->grade[i]); gmin = fminf(gmin, P->grade[i]); }
        int why = PL_WHY_CRUISE;
        if (out->late) why = PL_WHY_RECOVER;
        else if (w->lead && out->min_gap_margin == out->min_gap_margin && out->min_gap_margin < S.time_gap * fmaxf(v0, 5) * 0.5f && out->v_target < set - 0.3f) why = PL_WHY_LEAD;
        else if (capmin < set - 0.3f) why = capmin >= capat(w, w->lim5, 0) - 0.3f || capat(w, w->lim5, 0) < set ? PL_WHY_LIMIT : PL_WHY_CAP;
        else if (out->act == PL_NEUTRAL) why = PL_WHY_ECO;
        else if (out->v_target > set + 0.3f || gmin < -0.02f) why = PL_WHY_DESCENT;
        else if (out->v_target < set - 0.3f && g0 > 0.005f && gmin < -0.005f) why = PL_WHY_CREST;
        else if (gmax > 0.02f) why = PL_WHY_CLIMB;
        out->why = why;
    }
}
