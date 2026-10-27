// The vehicle model: mass, grade, engine and the forces, one estimate for every module. See vehicle.h.
#include "vehicle.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

float g_brake_learn;   // [core] brake_learn (the learned service-brake gain at the end of this file)
static void brake_learn_reset(int level);
static void brake_learn_step();
vehicle V = { NULL, 0, 0, -1e9, 0, 0, 0, {}, 0, 0, -1e9, 0, 0.006f, 6.0f, 0.9f, 1, 0 };

// Start of the torque plateau: first curve point within 3% of the peak.
static float find_plateau(const engine_info *e)
{
    float peak = 0;
    for (int i = 0; i < e->n_curve; i++) peak = fmaxf(peak, e->curve[i][1]);
    for (int i = 0; i < e->n_curve; i++)
        if (e->curve[i][1] >= 0.97f * peak && e->curve[i][0] > 300) return e->curve[i][0];
    return e->range[R_HIGH][0];
}

// The engine's peak power as torque factor x rpm (the unit of the power curve here: share of the peak torque times rpm). Computed once per engine read.
static float find_peak_pw(const engine_info *e)
{
    float best = 0;
    for (float r = 100; r <= e->rpm_limit; r += 5) best = fmaxf(best, curve_factor(e, r) * r);
    return best;
}

void vehicle_mass_stale() { V.mass_at = -1e9; }

// Continuity broke (core/telemetry.cpp session_reset): trailer (de)coupled (LOAD: the mass, and what was learned against
// it); other truck, save/load, teleport (HARD: the engine too, read again at the next update, and the rpm correction).
void vehicle_reset(int level)
{
    brake_learn_reset(level);
    double none = -1e9;
    if (level >= RESET_LOAD) {
        V.mass_at = none;
        V.f_bias = 0;
    }
    if (level >= RESET_HARD) {
        V.self = NULL;
        V.mass = 0; V.mass_ok = 0;
        V.eng_ok = 0;
        V.eng_at = none;
        V.rpm_k = 1;
    }
}

void vehicle_update(void *gb)
{
    double now = sim_s();
    if (gb && gb != V.self) {
        if (V.self) vehicle_reset(RESET_HARD);   // another gearbox object: another truck
        V.self = gb;
    }
    if (now - V.mass_at > 0.5 || now < V.mass_at) {
        float m = game_mass();
        if (m == m) {
            if (!(V.mass > 0) || fabsf(m - V.mass) > 0.03f * V.mass)
                logv(1, "mass %.1f t (trailer %s, cargo %.1f t)", m / 1000, T.trailer ? "yes" : "no", T.cargo_mass / 1000);
            V.mass = m;
            V.mass_ok = 1;
        } else if (!(V.mass > 0)) V.mass = VEHICLE_MASS_FALLBACK;
        V.mass_at = now;
    }
    if (gb && (!V.eng_ok || now - V.eng_at > 5.0 || now < V.eng_at)) {
        engine_info e;
        memset(&e, 0, sizeof e);
        int ok = gb_engine(gb, &e);
        V.electric = e.electric;
        if (ok && (!V.eng_ok || e.torque != V.eng.torque || e.rpm_limit != V.eng.rpm_limit)) V.f_bias = 0;  // another engine: what was learned against the old one is void
        if (ok) { V.eng = e; V.plateau = find_plateau(&e); V.peak_pw = find_peak_pw(&e); }
        V.eng_ok = ok;
        V.eng_at = now;
    }
    if (gb) {
        float gd = game_grade_deg(gb);
        V.grade_deg = gd == gd ? gd : 0;
        V.grade_pct = tanf(V.grade_deg * 0.0174533f) * 100;
    }
    // Grade (%) from the SDK placement: pitch in turns, positive = nose up.
    V.pitch_pct = tanf((float)T.place.orientation.pitch * 6.2831853f) * 100;
}

void vehicle_frame()
{
    vehicle_update(game_gearbox());
    if (g_brake_learn > 0) brake_learn_step();
}

// ---------------------------------------------------------------- gear, speed, forces

static int n_gears()
{
    int n = T.fwd_gears;
    return n > 0 && n < 32 ? n : 0;
}

// Engine rpm in gear g at speed v (m/s), from the driveline, corrected by the
// measured ratio to the real rpm.
float vehicle_rpm_at(int g, float v)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0) return 0;
    return v / r * T.ratio[g - 1] * T.diff * 9.5493f * V.rpm_k;
}

float vehicle_kmh_at(int g, float rpm)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0 || !(T.ratio[g - 1] > 0) || !(V.rpm_k > 0)) return 0;
    return rpm / (T.ratio[g - 1] * T.diff * 9.5493f * V.rpm_k) * r * 3.6f;
}

float vehicle_kmh_idle(int g, float idle_rpm, float margin_rpm) { return vehicle_kmh_at(g, idle_rpm + margin_rpm); }

// Tractive force (N) in gear g at speed v with a share of the available torque.
float vehicle_force_at(int g, float v, float share)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f) return 0;
    float rpm = vehicle_rpm_at(g, v);
    float t = V.eng.torque * curve_factor(&V.eng, rpm) * share;
    if (rpm > V.eng.rpm_limit) t = 0;
    return t * T.ratio[g - 1] * T.diff * V.eta / r;
}

// Air drag area: the truck value from the ini, a car-like one for light vehicles.
static float cda() { return V.mass > 0 && V.mass < 4500 ? 1.0f : V.cda; }

float vehicle_resist(float v)
{
    float s = sinf(V.grade_deg * 0.0174533f);
    return V.mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * cda() * v * v + V.f_bias;
}

float vehicle_resist_grade(float v, float pct)
{
    float s = sinf(atanf(pct / 100));
    return V.mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * cda() * v * v + V.f_bias;
}

float vehicle_accel(int g, float v, float share, float pct)
{
    if (!(V.mass > 0)) return 0;
    return (vehicle_force_at(g, v, share) - (pct == pct ? vehicle_resist_grade(v, pct) : vehicle_resist(v))) / V.mass;
}

// ---------------------------------------------------------------- the service brake's gain (B1 of the dynamic-threshold audit)
//
// A brake request (0..1, game_actor+0x17c or the driver's pedal) gives a deceleration that depends on the truck, its brakes and the
// load; modules used to guess it. Here it is measured: in a steady stretch of braking (one request, the clutch and the gear not
// moving, the grade steady, no throttle, no retarder or engine brake) the force the brakes gave is what the measured deceleration
// needs on top of the model's resistance:  gm = (m a_measured - vehicle_resist) / request  (N per unit request).
// It is kept as a force (gain x mass), not as a deceleration, so a load change keeps it valid (the brakes' torque does not follow the
// cargo); the trailer's brakes are another set, so a truck alone and with a trailer each get a slot. The engine's drag in gear is
// not modelled: measured in the game it is ~0.03 m/s2 (a coast in gear on the flat is 0.17 m/s2 in all, nearly all of it rolling and
// air, brake-measure notes), far under the 1 m/s2 or more of a stretch, so a closed clutch and an open one count alike.
// One window = 1.0 s after 0.4 s of settling in the same state; a window that breaks (request, clutch, gear, grade, mass, yaw, low brake air, speed
// floor, a pedal, the retarder) is thrown away. Windows of one stretch count 0.3 event each after the first: n is in events.
// A sample outside 1.2-16 m/s2 per request (at the truck's mass) or over 7.5 m/s2 of deceleration is not taken (a slip, ABS, a hill the
// model does not know: the telemetry has no wheel slip to ask).
// The filter is robust: the first three events give the median; later a window that differs from the estimate by more than 35 %
// is an outlier and counts only if four in a row agree with each other (then the brakes changed: a physics mod, a new trailer
// brake) and replace the estimate. An accepted window moves the estimate by max(1/(n+1), 0.12). Confidence =
// min(1, n / 5) x (1 - spread / 0.3), spread = the running mean of |sample / estimate - 1|; the gain is offered from 0.5 (4 clean
// events). Slots persist in brake_gain.txt per truck (engine, gears, wheels), as lane keeps steer_profile.txt, so the next session
// starts confident.
struct bg_slot { float gm, dev, n; float first[3]; int nf; float rej[4]; int nr; int dirty; float told; int was_conf; };
static struct {
    char key[96]; int loaded, primed;
    bg_slot s[2];
    int cur;                     // slot of the window (0 truck alone, 1 with a trailer)
    double last, save_at;
    // the window
    double settle, wt;           // seconds in the same state before the window / in it
    float r_ref, v0, vmid, rsum, fsum, cl_ref, grade0, mass0;
    int mid_done;
    int gear0, first_in_stretch, in_stretch;
} B;

static const float BG_SETTLE_S = 0.4f, BG_WIN_S = 1.0f, BG_MIN_R = 0.1f, BG_MIN_V = 3.0f, BG_MAX_A = 7.5f, BG_MIN_NET = 0.3f;
static const float BG_GAIN_LO = 1.2f, BG_GAIN_HI = 16.0f;
static const float BG_OUTLIER = 0.35f, BG_CONF_MIN = 0.5f;
static const char *const BG_FILE = "brake_gain.txt";

static float bg_conf(const bg_slot &s)
{
    if (s.n <= 0) return 0;
    return fminf(1.0f, s.n / 5.0f) * fmaxf(0.0f, 1.0f - s.dev / 0.3f);
}

// The truck changed (or the first frame): its key, and what the file knows about it.
static void bg_key()
{
    if (B.primed || !V.eng_ok || T.fwd_gears <= 0) return;
    char k[96];
    snprintf(k, sizeof k, "E%.0f-%.0f-G%d-W%d", V.eng.torque, V.eng.rpm_limit, T.fwd_gears, T.wheels);
    if (B.loaded && !strcmp(k, B.key)) return;
    snprintf(B.key, sizeof B.key, "%s", k);
    B.loaded = 1;
    memset(B.s, 0, sizeof B.s);
    FILE *f = open_out(BG_FILE, "r");
    if (!f) { logv(1, "Brake gain: nothing stored for '%s' yet", B.key); return; }
    char buf[4096];
    int n = (int)fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", B.key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            for (int t = 0; t < 2; t++) {
                char tag[8]; snprintf(tag, sizeof tag, " t%d=", t);
                const char *q = strstr(ln, tag);
                float gm = 0, dev = 0, cnt = 0;
                if (q && sscanf(q + 4, "%f,%f,%f", &gm, &dev, &cnt) == 3 && gm == gm && dev == dev && cnt == cnt &&
                    gm > 1000 && gm < 2e6f && dev >= 0 && dev < 1 && cnt > 0) {
                    B.s[t].gm = gm; B.s[t].dev = fminf(dev, 0.3f); B.s[t].n = fminf(cnt, 8.0f); B.s[t].nf = 3;
                    B.s[t].told = gm; B.s[t].was_conf = bg_conf(B.s[t]) >= BG_CONF_MIN;
                }
            }
            logv(1, "Brake gain: loaded for '%s': %.0f N per request alone (%.1f events), %.0f with a trailer (%.1f events)", B.key, B.s[0].gm, B.s[0].n, B.s[1].gm, B.s[1].n);
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Brake gain: nothing stored for '%s' yet", B.key);
}

// One line per truck, a few lines in all (never MB); the other trucks' lines are kept. Errors are logged once and otherwise ignored.
static void bg_save(const char *why)
{
    if (!B.loaded || !(B.s[0].dirty || B.s[1].dirty)) return;
    char buf[4096], out[4096]; int n = 0, m = 0, lines = 0;
    FILE *f = open_out(BG_FILE, "r");
    if (f) { n = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", B.key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + (int)strlen(ln) + 2 < 3300) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "truck=%s v=1", B.key);
    for (int t = 0; t < 2; t++) if (B.s[t].n > 0) m += snprintf(out + m, sizeof out - m, " t%d=%.0f,%.3f,%.1f", t, B.s[t].gm, B.s[t].dev, B.s[t].n);
    m += snprintf(out + m, sizeof out - m, "\n");
    B.save_at = wall_s();
    f = open_out(BG_FILE, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Brake gain: cannot write %s (the estimate stays in memory)", BG_FILE); return; }
    size_t w = fwrite(out, 1, m, f);
    fclose(f);
    if (w == (size_t)m) { B.s[0].dirty = B.s[1].dirty = 0; logv(2, "Brake gain: saved (%s)", why); }
}

static void bg_break() { B.settle = 0; B.wt = 0; B.in_stretch = 0; }

// A window's sample x (N per request) into slot t; first = the first window of its stretch.
static void bg_add(int t, float x, int first)
{
    bg_slot &s = B.s[t];
    float add = first ? 1.0f : 0.3f;
    if (s.nf < 3) {                                  // the first events: the median of them (a later window of the same stretch refines its event)
        if (first || s.nf == 0) s.first[s.nf++] = x;
        else s.first[s.nf - 1] = 0.5f * (s.first[s.nf - 1] + x);
        float a[3] = { s.first[0], s.nf > 1 ? s.first[1] : s.first[0], s.nf > 2 ? s.first[2] : s.first[0] };
        if (a[0] > a[1]) { float w = a[0]; a[0] = a[1]; a[1] = w; }
        if (a[1] > a[2]) { float w = a[1]; a[1] = a[2]; a[2] = w; }
        if (a[0] > a[1]) { float w = a[0]; a[0] = a[1]; a[1] = w; }
        s.gm = s.nf == 2 ? 0.5f * (s.first[0] + s.first[1]) : a[1];
        float d = 0;
        for (int i = 0; i < s.nf; i++) d = fmaxf(d, fabsf(s.first[i] / s.gm - 1));
        s.dev = s.nf > 1 ? d : 0.15f;
        s.n += add;
        s.dirty = 1;
        return;
    }
    float rel = x / s.gm - 1;
    if (fabsf(rel) > BG_OUTLIER) {
        if (s.nr < 4) s.rej[s.nr++] = x;
        else { memmove(s.rej, s.rej + 1, 3 * sizeof(float)); s.rej[3] = x; }
        if (s.nr >= 4) {
            float lo = s.rej[0], hi = s.rej[0], mean = 0;
            for (int i = 0; i < 4; i++) { lo = fminf(lo, s.rej[i]); hi = fmaxf(hi, s.rej[i]); mean += 0.25f * s.rej[i]; }
            if (hi - lo <= 0.15f * mean) {
                logv(1, "Brake gain: the brakes changed (%.0f -> %.0f N per request): the estimate starts over", s.gm, mean);
                s.gm = mean; s.n = 3; s.dev = 0.15f; s.nr = 0; s.dirty = 1;
            }
        }
        return;
    }
    s.nr = 0;
    float a = fmaxf(1.0f / (s.n + 1.0f), 0.12f);
    s.gm += a * (x - s.gm);
    s.dev += 0.25f * (fabsf(rel) - s.dev);
    s.n = fminf(s.n + add, 100.0f);
    s.dirty = 1;
}

static void bg_announce(int t)
{
    bg_slot &s = B.s[t];
    int conf = bg_conf(s) >= BG_CONF_MIN;
    if (conf && (!s.was_conf || fabsf(s.gm / fmaxf(s.told, 1.0f) - 1) > 0.10f)) {
        logv(1, "Brake gain: %.0f N per request %s (%.1f m/s2 at %.1f t, %.1f events, spread %.0f %%)", s.gm, t ? "with a trailer" : "alone",
             V.mass > 0 ? s.gm / V.mass : 0, V.mass / 1000, s.n, s.dev * 100);
        s.told = s.gm;
        B.save_at = -1e9;     // saved at the next step
    }
    s.was_conf = conf;
}

static void brake_learn_reset(int level)
{
    bg_break();
    B.last = 0;
    if (level >= RESET_HARD) {
        if (g_brake_learn > 0) { B.save_at = -1e9; bg_save("reset"); }
        B.loaded = B.primed = 0; B.key[0] = 0;
        memset(B.s, 0, sizeof B.s);
    }
}

static void brake_learn_step()
{
    double now = sim_s(), dt = now - B.last;
    B.last = now;
    if (!B.loaded) bg_key();
    if (B.loaded && (B.s[0].dirty || B.s[1].dirty) && wall_s() - B.save_at > 30) bg_save("periodic");
    int tr = T.trailer ? 1 : 0;
    if (tr != B.cur) { bg_break(); B.cur = tr; }
    float v = T.speed;
    float r = fmaxf(fmaxf(T.in_brake, T.eff_brake), act_request());
    int ok = dt > 0 && dt < 0.25 && B.loaded && V.mass_ok && V.self && !T.paused && !T.parking_brake && !T.air_warning && !(T.air_psi > 1 && T.air_psi < 60) && T.gear >= 0 && T.dgear >= 0 &&
             v == v && v >= BG_MIN_V && T.retarder == 0 && !T.motor_brake && fmaxf(T.in_thr, T.eff_thr) < 0.02f && fabsf(T.yaw) < 0.04f &&
             fabsf(V.grade_deg) < 12.0f && r >= BG_MIN_R && r <= 1.0f;
    if (!ok) { bg_break(); return; }
    float cl = T.eff_clutch;
    if (B.settle < BG_SETTLE_S) {
        // settling: the request, the clutch and the gear must stay put before a window opens
        if (B.settle == 0 || fabsf(r - B.r_ref) > 0.03f + 0.05f * B.r_ref || fabsf(cl - B.cl_ref) > 0.03f || T.gear != B.gear0) {
            B.settle = 0; B.r_ref = r; B.cl_ref = cl; B.gear0 = T.gear; B.in_stretch = 0;
        }
        B.settle += dt;
        if (B.settle >= BG_SETTLE_S) {
            B.wt = 0; B.v0 = v; B.mid_done = 0; B.rsum = B.fsum = 0; B.grade0 = V.grade_deg; B.mass0 = V.mass;
            B.first_in_stretch = !B.in_stretch;
        }
        return;
    }
    // inside a window: anything that moved throws it away
    if (fabsf(r - B.r_ref) > 0.03f + 0.05f * B.r_ref || fabsf(cl - B.cl_ref) > 0.03f || T.gear != B.gear0 ||
        fabsf(V.grade_deg - B.grade0) > 0.4f || fabsf(V.mass - B.mass0) > 0.03f * B.mass0) { bg_break(); return; }
    B.wt += dt;
    B.rsum += r * (float)dt;
    B.fsum += vehicle_resist(v) * (float)dt;
    if (!B.mid_done && B.wt >= 0.5f * BG_WIN_S) { B.vmid = v; B.mid_done = 1; }
    if (B.wt < BG_WIN_S) return;
    float a = (B.v0 - v) / (float)B.wt, rm = B.rsum / (float)B.wt, fres = B.fsum / (float)B.wt;
    float net = V.mass * a - fres;                       // the force the brakes gave
    float x = net / rm, gain = x / V.mass;
    // the two halves must agree (a brake that gave way in the middle -- a slip, a patch of ice, an ABS cycle -- is not a sample)
    float a1 = (B.v0 - B.vmid) / (0.5f * BG_WIN_S), a2 = (B.vmid - v) / (0.5f * BG_WIN_S);
    if (a > 0 && fabsf(a1 - a2) <= 0.12f * a + 0.15f && a <= BG_MAX_A && net / V.mass >= BG_MIN_NET && gain >= BG_GAIN_LO && gain <= BG_GAIN_HI) {
        bg_add(B.cur, x, B.first_in_stretch);
        bg_announce(B.cur);
        B.in_stretch = 1;
    } else B.in_stretch = 0;
    B.first_in_stretch = 0;
    // the next window follows at once (the state is settled)
    B.wt = 0; B.v0 = v; B.mid_done = 0; B.rsum = B.fsum = 0; B.grade0 = V.grade_deg; B.mass0 = V.mass;
}

float vehicle_brake_conf()
{
    if (!(g_brake_learn > 0) || !B.loaded) return 0;
    return bg_conf(B.s[T.trailer ? 1 : 0]);
}

float vehicle_brake_gain()
{
    if (!(g_brake_learn > 0) || !B.loaded || !V.mass_ok || !(V.mass > 0)) return NAN;
    const bg_slot &s = B.s[T.trailer ? 1 : 0];
    if (bg_conf(s) < BG_CONF_MIN) return NAN;
    return s.gm / V.mass;
}

float vehicle_brake_req(float decel, float fallback)
{
    float g = vehicle_brake_gain();
    if (g != g) return fallback;
    float r = decel / g;
    return r < 0 ? 0 : r > 1 ? 1 : r;
}

float vehicle_brake_hold_req(float grade_pct, float fallback)
{
    float g = vehicle_brake_gain();
    if (g != g) return fallback;
    // holding with margin: twice the pull of the slope plus 0.8 m/s2 (a push, the grade sensor's error), never under 0.2 and never over 1
    float need = 2.0f * 9.81f * fabsf(sinf(atanf(grade_pct / 100))) + 0.8f;
    float r = need / g;
    return r < 0.2f ? 0.2f : r > 1 ? 1 : r;
}

void vehicle_brake_prime(int trailer, float gm, float dev, float n)
{
    bg_slot &s = B.s[trailer ? 1 : 0];
    s = bg_slot();
    s.gm = gm; s.dev = dev; s.n = n; s.nf = 3; s.was_conf = bg_conf(s) >= BG_CONF_MIN; s.told = gm;
    B.loaded = B.primed = 1;
}
