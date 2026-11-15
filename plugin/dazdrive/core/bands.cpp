// The engine bands (see bands.h): P1 / P2 / P3 from the torque curve, the lug floor from the caller (the stall line + the hysteresis).
// bands_compute is pure; bands_refresh caches the current engine's bands (vehicle.cpp calls it when the engine is read, TrueShift when
// the idle or the floor moved), and bands_now() returns them.
#include "bands.h"
#include <math.h>
#include <string.h>

static float s_idle = 550.0f;         // the truck's idle (TrueShift updates it via bands_set_idle)
static float s_floor_off = 170.0f;    // the floor's distance above idle (TrueShift: ioff(120) + hysteresis_rpm)
static float s_used_idle = -1, s_used_off = -1;   // what the cached bands were computed with
static eng_bands s_bands;

// The first rpm (above 300) where the curve reaches thr, interpolated; NAN if never
static float rise(const engine_info *e, float thr)
{
    for (int i = 0; i < e->n_curve; i++) {
        float r1 = e->curve[i][0], f1 = e->curve[i][1];
        if (f1 < thr || r1 <= 300) continue;
        if (i == 0) return r1;
        float r0 = e->curve[i - 1][0], f0 = e->curve[i - 1][1];
        if (f0 >= thr) return 300.0f;   // r0 was skipped for lying at or under 300 rpm
        float r = r0 + (thr - f0) / (f1 - f0) * (r1 - r0);
        return fmaxf(r, 300.0f);
    }
    return NAN;
}

// The first rpm above `from` where the curve drops under thr again, interpolated; the last point's rpm if it never does
static float fall(const engine_info *e, float thr, float from)
{
    for (int i = 1; i < e->n_curve; i++) {
        float r0 = e->curve[i - 1][0], f0 = e->curve[i - 1][1], r1 = e->curve[i][0], f1 = e->curve[i][1];
        if (r1 <= from || f1 >= thr) continue;
        if (f0 < thr) return fmaxf(from, r0);
        return r0 + (f0 - thr) / (f0 - f1) * (r1 - r0);
    }
    return e->n_curve > 0 ? e->curve[e->n_curve - 1][0] : from;
}

void bands_compute(const engine_info *e, float idle, float floor_off, eng_bands *out)
{
    eng_bands b;
    memset(&b, 0, sizeof b);
    memset(out, 0, sizeof *out);
    float lim = e->rpm_limit, pk = 0, lo = 1e30f;
    for (int i = 0; i < e->n_curve; i++) { pk = fmaxf(pk, e->curve[i][1]); lo = fminf(lo, e->curve[i][1]); }
    // (a curve with no torque variation has no plateau edges: not usable, the caller keeps its old rules)
    if (e->n_curve < 2 || !(pk > 0) || pk - lo < 1e-6f || !(lim > idle) || !(idle >= 0)) return;
    float thr = BANDS_PLATEAU * pk;
    b.idle = idle; b.limit = lim; b.cap = BANDS_CAP * lim;
    b.game_low_lo = e->range[R_LOW][0];
    b.p1 = rise(e, thr);
    if (!(b.p1 == b.p1)) return;
    b.p1 = fmaxf(b.p1, idle);
    b.p2 = fmaxf(b.p1, fminf(fall(e, thr, b.p1), b.cap));
    float best = 0;
    b.p3 = b.p2;
    for (float r = idle; r <= lim; r += 5) {
        float p = curve_factor(e, r) * r;
        if (p > best) { best = p; b.p3 = r; }
    }
    b.floor = fminf(idle + floor_off, b.p1);
    // the power table, P1 .. P3 (see bands.h): the power at each rpm over the peak's, a running maximum
    {
        const float peak = curve_factor(e, b.p3) * b.p3;
        float run = 0;
        for (int k = 0; k < BANDS_PW_N; k++) {
            const float r = b.p1 + (float)k / (BANDS_PW_N - 1) * (b.p3 - b.p1);
            run = fmaxf(run, peak > 0 ? curve_factor(e, r) * r / peak : 1.0f);
            b.pw[k] = fminf(run, 1.0f);
        }
        b.pw[BANDS_PW_N - 1] = 1.0f;
    }
    // the order must hold: a curve that breaks it is not a usable engine
    if (!(b.idle <= b.floor && b.floor <= b.p1 && b.p1 < b.p2 && b.p2 <= b.cap && b.cap < b.limit)) return;
    b.ok = 1;
    *out = b;
}

void bands_set_idle(float idle)
{
    if (idle > 0) s_idle = idle;
}

void bands_set_floor_off(float off)
{
    if (off > 0) s_floor_off = off;
}

int bands_stale()
{
    return fabsf(s_idle - s_used_idle) >= 1.0f || fabsf(s_floor_off - s_used_off) >= 1.0f;
}

void bands_refresh(const engine_info *e)
{
    bands_compute(e, s_idle, s_floor_off, &s_bands);
    s_used_idle = s_idle; s_used_off = s_floor_off;
}

const eng_bands *bands_now() { return &s_bands; }
