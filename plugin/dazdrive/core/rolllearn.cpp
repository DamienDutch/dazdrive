// Roll learning: watches the bends the truck drives, measures how much the truck and its trailer lean per m/s2 of sideways acceleration, keeps
// the gains per truck and combination (roll_gain.txt) and publishes the allowance the bends are planned with (the maths: core/roll.h).
//
// The sideways acceleration is speed x yaw rate (the rollover guard's measure), low-passed at 0.25 s so it lines up with the body's response.
// The lean is the SDK placement's roll of the truck and of the first trailer. A bend runs from 0.5 m/s2 to below 0.3 m/s2 (or a flip of
// sign); its samples are fitted lean = gain x acceleration + bank. The bank (the road's camber) is the offset: it does not follow the
// acceleration. The route's node quaternion could say the bank too, but only its heading is checked against a real drive, so the fit stays
// the only source.
//
// The combination is the truck (gears, wheels, rpm limit, final drive) alone or with a trailer, at a load: the mass of the vehicle model
// (V.mass). Another mass by more than 12 % is another combination (the gains start over). The file keeps one load per truck and trailer flag.
#include "suite.h"
#include "vehicle.h"
#include "vclass.h"
#include "roll.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *const RLF = "roll_gain.txt";
static const float RL_ON_A = 0.5f, RL_OFF_A = 0.3f;   // m/s2: a bend starts above / ends below
static const float RL_MIN_V = 25.0f / 3.6f;           // m/s: slower than this the lean is not a steady one
static const float RL_TAU_A = 0.25f;                  // s, the acceleration's filter (the body's response is about that slow)

static struct {
    char key[96]; int loaded;
    roll_slot s[2];
    int idx;                      // the slot of now, -1 = none
    double last_t, hold_until, save_at;
    float af;                     // the filtered sideways acceleration (m/s2, + = left)
    int on, sgn;                  // a bend is open; its side
    roll_fit f[2];
    int trailer_seen;             // the trailer's placement delivered during the bend
    double bend_t;
    int told_n; double told_at; float told_g[2];  // the last announced figures (the log says it when they move)
    int rsc_prev;
    int recheck;                  // RESET_LOAD: judge the combination again
} R;

static float roll_deg(const scs_value_dplacement_t &p)
{
    float d = (float)p.orientation.roll * 360.0f;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

// ---------------------------------------------------------------- the file
static int parse_slot(const char *q, roll_slot *s)
{
    float mass, maxa, g0, d0, n0, g1, d1, n1; int tw;
    if (sscanf(q, "%f,%d,%f,%f,%f,%f,%f,%f,%f", &mass, &tw, &maxa, &g0, &d0, &n0, &g1, &d1, &n1) != 9) return 0;
    if (!(mass > 500 && mass < 1e6f) || !(maxa >= 0 && maxa < 30) || !(g0 >= 0 && g0 <= ROLL_G_HI) || !(g1 >= 0 && g1 <= ROLL_G_HI)) return 0;
    if (!(d0 >= 0 && d0 <= 1) || !(d1 >= 0 && d1 <= 1) || !(n0 >= 0 && n0 <= 40) || !(n1 >= 0 && n1 <= 40)) return 0;
    s->mass = mass; s->tw = tw; s->max_a = maxa;
    s->b[0].g = g0; s->b[0].dev = d0; s->b[0].n = n0;
    s->b[1].g = g1; s->b[1].dev = d1; s->b[1].n = n1;
    return 1;
}

static void load_key()
{
    char k[96];
    snprintf(k, sizeof k, "G%d-W%d-R%.0f-D%.2f", T.fwd_gears, T.wheels, T.rpm_limit, T.diff);
    if (R.loaded && !strcmp(k, R.key)) return;
    snprintf(R.key, sizeof R.key, "%s", k);
    R.loaded = 1;
    R.s[0] = roll_slot(); R.s[1] = roll_slot(); R.s[1].has_trailer = 1;
    R.idx = -1; RL.cur = NULL; RL.out = NAN;
    FILE *f = open_out(RLF, "r");
    if (!f) { logv(1, "Roll: nothing stored for '%s' yet", R.key); return; }
    char buf[4096];
    int n = (int)fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", R.key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            for (int i = 0; i < 2; i++) {
                char tag[8]; snprintf(tag, sizeof tag, " s%d=", i);
                const char *q = strstr(ln, tag);
                roll_slot t;
                if (q && parse_slot(q + 4, &t)) { bool tr = R.s[i].has_trailer; R.s[i] = t; R.s[i].has_trailer = tr; }
            }
            logv(1, "Roll: loaded for '%s': alone %.2f deg per m/s2 at %.1f t (%.0f bends), with a trailer %.2f / %.2f at %.1f t", R.key,
                 R.s[0].b[0].g, R.s[0].mass / 1000, R.s[0].b[0].n, R.s[1].b[0].g, R.s[1].b[1].g, R.s[1].mass / 1000);
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Roll: nothing stored for '%s' yet", R.key);
}

// One line per truck, a few lines in all; the other trucks' lines are kept.
static void save(const char *why)
{
    if (!R.loaded || !(R.s[0].dirty || R.s[1].dirty)) return;
    char buf[4096], out[4096]; int n = 0, m = 0, lines = 0;
    FILE *f = open_out(RLF, "r");
    if (f) { n = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", R.key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + (int)strlen(ln) + 2 < 3300) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "truck=%s v=1", R.key);
    for (int i = 0; i < 2; i++) {
        const roll_slot &s = R.s[i];
        if (s.b[0].n > 0 && s.mass > 0)
            m += snprintf(out + m, sizeof out - m, " s%d=%.0f,%d,%.2f,%.4f,%.3f,%.1f,%.4f,%.3f,%.1f", i, s.mass, s.tw, s.max_a, s.b[0].g, s.b[0].dev, s.b[0].n,
                          s.b[1].g, s.b[1].dev, s.b[1].n);
    }
    m += snprintf(out + m, sizeof out - m, "\n");
    R.save_at = wall_s();
    f = open_out(RLF, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Roll: cannot write %s (the gains stay in memory)", RLF); return; }
    size_t w = fwrite(out, 1, m, f);
    fclose(f);
    if (w == (size_t)m) { R.s[0].dirty = R.s[1].dirty = 0; logv(2, "Roll: saved (%s)", why); }
}

// ---------------------------------------------------------------- the combination
static void bend_abort() { R.on = 0; R.f[0].clear(); R.f[1].clear(); R.trailer_seen = 0; }

// The slot of now: alone or with the trailer, at the load the model knows. Another load = the slot starts over.
static void pick_slot()
{
    int i = T.trailer ? 1 : 0;
    roll_slot &s = R.s[i];
    if (s.mass > 0 && fabsf(V.mass / s.mass - 1) > ROLL_MASS_TOL) {
        logv(1, "Roll: another load (%.1f t, was %.1f t %s): the lean is learned again", V.mass / 1000, s.mass / 1000, i ? "with the trailer" : "alone");
        bool tr = s.has_trailer; s = roll_slot(); s.has_trailer = tr; s.dirty = 1;
    }
    if (s.mass <= 0) { s.mass = V.mass; s.tw = T.t_wheels; }
    if (i == 1 && s.tw != T.t_wheels && s.b[0].n > 0) {   // another trailer of about the same mass
        logv(1, "Roll: another trailer (%d wheels, was %d): the lean is learned again", T.t_wheels, s.tw);
        s = roll_slot(); s.has_trailer = 1; s.mass = V.mass; s.tw = T.t_wheels; s.dirty = 1;
    }
    if (R.idx != i) { bend_abort(); R.idx = i; R.told_n = 0; R.told_at = -1e9; R.told_g[0] = R.told_g[1] = 0; }
    RL.cur = &s;
}

// ---------------------------------------------------------------- announce
static void announce(const char *what)
{
    if (!RL.cur) return;
    const roll_slot &s = *RL.cur;
    float conf = 0, lim = 0;
    float start = RL.last_start == RL.last_start ? RL.last_start : 0, cap = RL.last_cap == RL.last_cap ? RL.last_cap : 0;
    float tgt = roll_target(&s, start, cap > 0 ? cap : 99, &conf, &lim);
    int moved = fabsf(s.b[0].g - R.told_g[0]) > 0.15f * fmaxf(R.told_g[0], 0.05f) || fabsf(s.b[1].g - R.told_g[1]) > 0.15f * fmaxf(R.told_g[1], 0.05f);
    if (!R.told_n || (moved && wall_s() - R.told_at > 60)) {   // (every bend is in the level-2 lines; this one says it when the figure moved)
        R.told_at = wall_s();
        logline("Roll: %s: lean %.2f deg per m/s2 truck%s; %.1f t %s, %.0f bends, confidence %.2f; at %.1f deg the edge is %.1f m/s2, x%.2f = %.1f, "
                "cap %.1f, start %.2f: bends planned with %.2f m/s2", what, s.b[0].g,
                s.has_trailer ? (s.b[1].n > 0 ? "" : ", trailer not measured") : "", s.mass / 1000, s.has_trailer ? "with a trailer" : "alone", s.b[0].n,
                conf, g_roll_budget * s.edge, lim, g_roll_edge, g_roll_edge * lim, cap, start, tgt == tgt ? tgt : start);
        if (s.has_trailer && s.b[1].n > 0) logline("Roll: trailer lean %.2f deg per m/s2 (%.0f bends)", s.b[1].g, s.b[1].n);
        R.told_n = 1; R.told_g[0] = s.b[0].g; R.told_g[1] = s.b[1].g;
    }
}

// ---------------------------------------------------------------- a bend ends
static void bend_end()
{
    int was_on = R.on;
    roll_fit f0 = R.f[0], f1 = R.f[1];
    int tr_ok = R.trailer_seen;
    bend_abort();
    if (!was_on || R.idx < 0 || !RL.cur) return;
    roll_slot &s = R.s[R.idx];
    float sl, off, r, se;
    int ok0 = roll_fit_eval(f0, &sl, &off, &r, &se);
    if (!ok0) {
        if (f0.w >= 2.0 && f0.peak >= 1.2f)
            logv(2, "Roll: bend left out (truck): peak %.1f m/s2, %.1f s, gain %.2f, r %.2f, error %.0f %%", f0.peak, f0.w, sl, r, se * 100);
        return;
    }
    roll_body_add(s.b[0], sl);
    if (f0.peak > s.max_a) s.max_a = f0.peak;
    float bank = off;
    float sl1 = 0, off1 = 0, r1 = 0, se1 = 0;
    int ok1 = R.idx == 1 && tr_ok && f1.w >= 0.8 * f0.w && roll_fit_eval(f1, &sl1, &off1, &r1, &se1);
    if (ok1) roll_body_add(s.b[1], sl1);
    s.dirty = 1;
    logv(2, "Roll: bend counted: peak %.1f m/s2 over %.1f s, truck %.2f deg per m/s2 (r %.2f, error %.0f %%, bank %.2f deg)%s", f0.peak, f0.w, sl, r, se * 100, bank,
         ok1 ? "" : (R.idx == 1 ? ", trailer left out" : ""));
    if (ok1) logv(2, "Roll: trailer %.2f deg per m/s2 (r %.2f, error %.0f %%, bank %.2f deg)", sl1, r1, se1 * 100, off1);
    announce("learned");
    if (wall_s() - R.save_at > 10) save("bend");
}

// ---------------------------------------------------------------- the frame
void roll_frame()
{
    double t = sim_s();
    RL.t = t;
    double dt = t - R.last_t;
    R.last_t = t;
    roll_guard_k_step(RL.cur, (float)dt);   // the rollover guard's scale (core/roll.h): follows the slot as the last frame left it, down at once, up slowly
    if (!(g_roll_learn > 0) || !vprof()->rollover_guard) {   // off, or a light vehicle (the lean of a car says nothing of a truck)
        if (R.on) bend_abort();
        RL.cur = NULL; RL.out = NAN; R.idx = -1;
        return;
    }
    if (!(dt > 0 && dt < 0.2) || T.paused) { bend_abort(); R.af = 0; return; }
    if (!V.mass_ok || T.fwd_gears <= 0) { bend_abort(); RL.cur = NULL; return; }
    load_key();
    if (R.recheck) { R.recheck = 0; bend_abort(); R.idx = -1; }
    pick_slot();
    if (t < R.hold_until) { bend_abort(); R.af = 0; return; }
    // the rollover guard fired: this combination is nearer the edge than the budget says (for the rest of it)
    if (g_rsc_level > 0 && !R.rsc_prev && RL.cur && R.idx >= 0) {
        roll_slot &s = R.s[R.idx];
        s.edge = fmaxf(0.5f, s.edge * 0.85f);
        logline("Roll: the rollover guard fired: the edge is taken down to %.0f %% of %.1f deg for this load", s.edge * 100, g_roll_budget);
    }
    R.rsc_prev = g_rsc_level > 0;

    float v = fabsf(T.speed);
    if (T.gear < 0 || T.speed < 0 || v < RL_MIN_V) { bend_abort(); R.af = 0; return; }
    float a = v * T.yaw * 6.2831853f;
    R.af += (a - R.af) * fminf(1.0f, (float)dt / RL_TAU_A);
    if (!R.on) {
        if (fabsf(R.af) >= RL_ON_A) { R.on = 1; R.sgn = R.af > 0 ? 1 : -1; R.f[0].clear(); R.f[1].clear(); R.trailer_seen = 1; R.bend_t = t; }
        else return;
    }
    float x = R.af * R.sgn;
    if (x < RL_OFF_A) { bend_end(); return; }
    // a trailer that comes and goes during the bend is not a measurement
    int tr_fresh = T.trailer && T.trailer_place_t > 0 && wall_s() - T.trailer_place_t < 0.5;
    if (R.idx == 1 && !tr_fresh) R.trailer_seen = 0;
    R.f[0].add((float)dt, x, ROLL_SIGN * R.sgn * roll_deg(T.place));
    if (R.idx == 1 && tr_fresh) R.f[1].add((float)dt, x, ROLL_SIGN * R.sgn * roll_deg(T.trailer_place));
    if (t - R.bend_t > 60) bend_abort();   // a roundabout that long is no bend to measure
}

void roll_reset(int level)
{
    bend_abort();
    R.af = 0;
    R.rsc_prev = 0;
    if (level >= RESET_LOAD) { R.recheck = 1; R.hold_until = sim_s() + 2.0; }
    if (level >= RESET_HARD) {
        if (R.loaded) { R.save_at = -1e9; save("reset"); }
        R.loaded = 0; R.key[0] = 0; R.idx = -1; RL.cur = NULL; RL.out = NAN;
        R.s[0] = roll_slot(); R.s[1] = roll_slot(); R.s[1].has_trailer = 1;
    }
}

void roll_stop() { R.save_at = -1e9; save("stop"); }

void roll_prime(int has_trailer, float g_truck, float g_trailer, float n, float max_a, float mass)
{
    R.loaded = 1; snprintf(R.key, sizeof R.key, "sim");
    roll_slot &s = R.s[has_trailer ? 1 : 0];
    s = roll_slot(); s.has_trailer = has_trailer; s.mass = mass; s.max_a = max_a;
    s.b[0].g = g_truck; s.b[0].n = n; s.b[0].dev = 0.05f;
    if (has_trailer) { s.b[1].g = g_trailer; s.b[1].n = n; s.b[1].dev = 0.05f; }
    R.idx = has_trailer ? 1 : 0; RL.cur = &s;
}
