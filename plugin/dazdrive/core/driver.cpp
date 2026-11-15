// The driver profile, the pure part (core/driver.h): the learners' state and queries, and the file's text in and out. No game, no vehicle model, no file:
// tools/sim/test_driver.cpp links it with core/demand.cpp alone.
#include "driver.h"
#include "demand.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static unsigned s_gen;   // learner events (the demand history counts its own notes: demand_gen)

unsigned driver_gen() { return s_gen + demand_gen(); }

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

// A probability learned from events, held within a factor of DRV_BOUND of its prior: either outcome's probability (p and 1 - p) may double or halve, not more.
static float bound_prob(float prior, float x)
{
    const float p = clampf(prior, 0.0f, 1.0f);
    const float lo = fmaxf(p / DRV_BOUND, 1.0f - (1.0f - p) * DRV_BOUND), hi = fminf(p * DRV_BOUND, 1.0f - (1.0f - p) / DRV_BOUND);
    return clampf(x, lo, hi);
}

// ---------------------------------------------------------------- the events
// A ring of the newest DRV_EVENTS events of one kind, oldest first on read.
namespace {
template <class E> struct ring {
    E e[DRV_EVENTS]; int n = 0, head = 0;
    void clear() { n = head = 0; }
    void add(const E &x) { e[head] = x; head = (head + 1) % DRV_EVENTS; if (n < DRV_EVENTS) n++; }
    const E &at(int i) const { return e[(head - n + i + 2 * DRV_EVENTS) % DRV_EVENTS]; }   // i = 0 the oldest
};
}

// ---------------------------------------------------------------- (a) lift habits
namespace {
enum { LF_RE = 'r', LF_BRAKE = 'b', LF_TIMEOUT = 't' };
struct lift_ev { unsigned short cs; char o; unsigned short kmh; };   // how long (centiseconds), how it ended, and the speed it started at (km/h)
}
static ring<lift_ev> s_lifts;
static lift_ev s_last_lift;
static struct { int on; double t0; float v0, pk; } L;   // an open lift: since when, from what speed; pk: the pedal's peak since it was last off

static void lift_record(double dur, char how, float v0)
{
    lift_ev e;
    e.cs = (unsigned short)clampf((float)lround(dur * 100.0), 1.0f, DRV_LIFT_TIMEOUT_S * 100.0f);
    e.o = how;
    e.kmh = (unsigned short)clampf((float)lround(v0 * 3.6f), 0.0f, 400.0f);
    s_lifts.add(e);
    s_last_lift = e;
    s_gen++;
}

// the pedal is lifted from a real demand, and what ends it
static int step_lift(const driver_sample *s)
{
    const bool off = s->pedal < DRV_OFF;
    int ev = 0;
    if (!L.on) {
        if (!off) L.pk = fmaxf(L.pk, s->pedal);
        else if (L.pk >= DRV_REAL) { L.on = 1; L.t0 = s->t; L.v0 = s->v; L.pk = 0.0f; }   // (a real demand came before: this is a lift)
        return 0;
    }
    const double dur = s->t - L.t0;
    if (!off) { lift_record(dur, LF_RE, L.v0); ev = DRV_EV_LIFT; L.on = 0; L.pk = s->pedal; }
    else if (s->brake > DRV_BRAKE) { lift_record(dur, LF_BRAKE, L.v0); ev = DRV_EV_LIFT; L.on = 0; L.pk = 0.0f; }
    else if (s->v < DRV_REST) { L.on = 0; L.pk = 0.0f; }   // the truck stopped by itself: no lift to learn from
    else if (dur >= DRV_LIFT_TIMEOUT_S) { lift_record(dur, LF_TIMEOUT, L.v0); ev = DRV_EV_LIFT; L.on = 0; L.pk = 0.0f; }
    return ev;
}

float driver_lift_return_p(float tau, float w, float shift_s)
{
    tau = tau > 0.0f ? tau : 0.0f;
    w = w > 0.0f ? w : 0.0f;
    const float shift = shift_s > 0.05f ? shift_s : 0.05f;   // (a numerical guard against 0 / 0 only: a shift lasts longer)
    const float prior = 0.5f * w / (tau + w + shift);
    int n = 0, k = 0;
    for (int i = 0; i < s_lifts.n; i++) {
        const lift_ev &e = s_lifts.at(i);
        const float d = (float)e.cs / 100.0f;
        if (!(d > tau)) continue;                                  // it ended before tau: not at risk
        n++;
        if (e.o == LF_RE && d <= tau + w) k++;                     // it ended in a re-press within the window
    }
    if (n == 0) return prior;
    return bound_prob(prior, ((float)k + DRV_PRIOR_N * prior) / ((float)n + DRV_PRIOR_N));
}

int driver_lift_n(float tau)
{
    int n = 0;
    for (int i = 0; i < s_lifts.n; i++) if ((float)s_lifts.at(i).cs / 100.0f > tau) n++;
    return n;
}

// ---------------------------------------------------------------- (c) lift-then-brake
// The lifts that ended one way or the other, split at the median of their speeds: the band of v and its counts. With fewer than two lifts there is one band (split -1: all of them).
static void brake_band(float v_ms, int *n, int *k)
{
    unsigned short sp[DRV_EVENTS]; int m = 0;
    for (int i = 0; i < s_lifts.n; i++) if (s_lifts.at(i).o != LF_TIMEOUT) sp[m++] = s_lifts.at(i).kmh;
    for (int i = 1; i < m; i++) { const unsigned short x = sp[i]; int j = i - 1; while (j >= 0 && sp[j] > x) { sp[j + 1] = sp[j]; j--; } sp[j + 1] = x; }
    const float split = m >= 2 ? (m % 2 ? (float)sp[m / 2] : 0.5f * ((float)sp[m / 2 - 1] + (float)sp[m / 2])) : -1.0f;
    const float kmh = v_ms * 3.6f;
    const bool fast = kmh >= split;   // (with one band, split = -1: everything is 'fast', the whole set)
    *n = *k = 0;
    for (int i = 0; i < s_lifts.n; i++) {
        const lift_ev &e = s_lifts.at(i);
        if (e.o == LF_TIMEOUT) continue;
        if (((float)e.kmh >= split) != fast) continue;
        ++*n;
        if (e.o == LF_BRAKE) ++*k;
    }
}

float driver_lift_brake_p(float v_ms)
{
    const float prior = 0.5f;
    int n, k;
    brake_band(v_ms, &n, &k);
    if (n == 0) return prior;
    return bound_prob(prior, ((float)k + DRV_PRIOR_N * prior) / ((float)n + DRV_PRIOR_N));
}

int driver_lift_brake_n(float v_ms) { int n, k; brake_band(v_ms, &n, &k); return n; }

static void count_lifts(int *re, int *br, int *to)
{
    *re = *br = *to = 0;
    for (int i = 0; i < s_lifts.n; i++) { const char o = s_lifts.at(i).o; if (o == LF_RE) ++*re; else if (o == LF_BRAKE) ++*br; else ++*to; }
}

// ---------------------------------------------------------------- (b) press speed
// The press detector is the zigzag of the raw pedal with the reversal DRV_REAL: m the lowest since the last turn, M the highest since m. The times at which the
// pedal first passed m + k x STEP are kept (21 of them), so the time the rise took through its first DRV_REAL is read off by interpolation, as soon as the pedal is there
// and however long it then stays up (no buffer to thin).
static ring<unsigned short> s_presses;   // the rates in 0.01 pedal/s
static ring<unsigned short> s_ends;      // the held tops of the presses that came to the end of the pedal, per mille (f)
static double s_above[17];               // the seconds the press in progress spent at or above 0.85 + k x 0.01 (k 0..15) and above 1.0 - 0 (k 16 unused): the held top is read off at the press's end
static struct { float rate, from, to; } s_last_press;
static const float PR_STEP = DRV_OFF;    // the pedal's smallest movement the plugin acts on: the grid the first-passage times are taken on
enum { PR_LEVELS = 21 };                 // 1 / PR_STEP + 1
static struct { int init; float m, M; double tM; int ng; double gt[PR_LEVELS]; double pt; float pp; } PR;   // pt, pp: the frame before

static void press_reset(float p, double t)
{
    PR.init = 1; PR.m = PR.M = p; PR.tM = t; PR.ng = 1; PR.gt[0] = t;
    memset(s_above, 0, sizeof s_above);
}

// The rate of the press in progress: the pedal's speed through its first DRV_REAL after the first step: the pedal first passing foot + STEP, then foot + DRV_REAL; the
// rise between the two over the time between them. (The foot itself is no clock: the pedal may have sat at its lowest for long, and the first step is where the rise has
// begun.) A straight 0 -> 1 press in 0.5 s is 2.0, as intent.cpp has always read the pedal's speed, and it is known the moment the pedal has risen a real demand, which is
// when a kickdown would ask for it. This is the ONE measure: the learner stores it for every press and the gearbox asks for it (driver_press_rate_now), so a learned scale is in
// the unit it is applied to. (A 10 % to 90 % rise time needs the press to be over; the instantaneous difference of two frames depends on the frame rate.)
static const int PR_KR = 6;   // DRV_REAL / PR_STEP
static float press_rate_first()
{
    if (!PR.init || PR.ng <= PR_KR) return 0.0f;
    const double dt = PR.gt[PR_KR] - PR.gt[1];
    return dt > 0.0 ? clampf((float)((double)((PR_KR - 1) * PR_STEP) / dt), 0.01f, 999.0f) : 999.0f;
}
float driver_press_rate_now()
{
    // the press of now: rising, or stopped rising less than a second ago (the history's step); a pedal held up for long is no longer a press that is going on
    return PR.init && PR.pt - PR.tM <= 1.0 ? press_rate_first() : 0.0f;
}

// The held top of the press that ends now: the highest level the pedal spent DRV_END_HOLD_S at or above; 0 = none reached DRV_END_LO (f).
static float press_held_top()
{
    for (int k = 15; k >= 0; k--)
        if (s_above[k] >= DRV_END_HOLD_S) return DRV_END_LO + (float)k * 0.01f;
    return 0.0f;
}

static int press_finish()
{
    const float top = press_held_top();
    if (top > 0.0f) { s_ends.add((unsigned short)lroundf(top * 1000.0f)); s_gen++; }
    const float rate = press_rate_first();
    if (!(rate > 0.0f)) return 0;
    s_presses.add((unsigned short)lroundf(rate * 100.0f));
    s_last_press.rate = rate; s_last_press.from = PR.m; s_last_press.to = PR.M;
    s_gen++;
    return DRV_EV_PRESS;
}

static int step_press(const driver_sample *s)
{
    const float p = s->pedal;
    int ev = 0;
    if (!PR.init) press_reset(p, s->t);
    else if (PR.M - PR.m >= DRV_REAL && p <= PR.M - DRV_REAL) { ev = press_finish(); press_reset(p, s->t); }
    else if (p < PR.m) press_reset(p, s->t);
    if (PR.init && p > PR.M) {
        // the passages of the grid levels between the old top and this frame's pedal, by interpolation between the frame before and this one
        int kmax = (int)floorf((p - PR.m) / PR_STEP + 1e-4f);
        if (kmax > PR_LEVELS - 1) kmax = PR_LEVELS - 1;
        for (int k = PR.ng; k <= kmax; k++) {
            const float L = PR.m + (float)k * PR_STEP;
            PR.gt[k] = p > PR.pp ? PR.pt + (double)(L - PR.pp) / (double)(p - PR.pp) * (s->t - PR.pt) : s->t;
        }
        if (kmax + 1 > PR.ng) PR.ng = kmax + 1;
        PR.M = p; PR.tM = s->t;
    }
    // the time at each held level (f): this frame's pedal counts for every level at or under it
    if (PR.init && PR.pt < s->t && s->t - PR.pt <= DRV_GAP_S)
        for (int k = 0; k < 16; k++) if (p >= DRV_END_LO + (float)k * 0.01f) s_above[k] += s->t - PR.pt;
    PR.pt = s->t; PR.pp = p;
    return ev;
}

float driver_press_scale()
{
    const int n = s_presses.n, need = (int)lroundf(1.0f / (1.0f - DRV_PRESS_Q));   // a 0.9 quantile of fewer than 10 is just their maximum
    if (n < need) return DRV_PRESS_RATE_PRIOR;
    unsigned short r[DRV_EVENTS];
    for (int i = 0; i < n; i++) r[i] = s_presses.at(i);
    for (int i = 1; i < n; i++) { const unsigned short x = r[i]; int j = i - 1; while (j >= 0 && r[j] > x) { r[j + 1] = r[j]; j--; } r[j + 1] = x; }   // (insertion sort: 64 at most)
    const int rank = (int)ceilf(DRV_PRESS_Q * (float)n) - 1;                  // nearest rank
    const float q = (float)r[rank < 0 ? 0 : rank >= n ? n - 1 : rank] / 100.0f;
    const float est = (DRV_PRIOR_N * DRV_PRESS_RATE_PRIOR + (float)n * q) / ((float)n + DRV_PRIOR_N);
    return clampf(est, DRV_PRESS_RATE_PRIOR / DRV_BOUND, DRV_PRESS_RATE_PRIOR * DRV_BOUND);
}

float driver_urgency(float rate) { return clampf(rate / driver_press_scale(), 0.0f, 1.0f); }
int driver_press_n() { return s_presses.n; }

float driver_press_end()
{
    const int n = s_ends.n;
    if (n < 1) return DRV_END_PRIOR;
    unsigned short r[DRV_EVENTS];
    for (int i = 0; i < n; i++) r[i] = s_ends.at(i);
    for (int i = 1; i < n; i++) { const unsigned short x = r[i]; int j = i - 1; while (j >= 0 && r[j] > x) { r[j + 1] = r[j]; j--; } r[j + 1] = x; }
    const int rank = (int)ceilf(0.5f * (float)n) - 1;                         // the median, nearest rank
    const float med = (float)r[rank < 0 ? 0 : rank >= n ? n - 1 : rank] / 1000.0f;
    const float est = (DRV_PRIOR_N * DRV_END_PRIOR + (float)n * med) / ((float)n + DRV_PRIOR_N);
    return clampf(est, DRV_END_LO, 1.0f);
}
int driver_press_end_n() { return s_ends.n; }

// ---------------------------------------------------------------- (d) launch habit per load
namespace { struct launch_cls { unsigned char n; float mean; }; }
static launch_cls s_launch[DRV_LAUNCH_CLASSES];
static struct { int cls, n; float x; } s_last_launch;
static struct { int on, cls, g0, moved; double t_prev, tsum, sum; float mass; } LA;

int driver_launch_class(float mass_kg)
{
    if (!(mass_kg >= 1000.0f)) return 0;
    const int c = (int)floorf(log2f(mass_kg / 1000.0f));
    return c < 0 ? 0 : c >= DRV_LAUNCH_CLASSES ? DRV_LAUNCH_CLASSES - 1 : c;
}

static int launch_finish()
{
    LA.on = 0;
    if (!LA.moved || !(LA.tsum >= 1.0)) return 0;   // (it must have moved, and lasted a second: the history's step)
    const float x = clampf((float)(LA.sum / LA.tsum), 0.0f, 1.0f);
    launch_cls &c = s_launch[LA.cls];
    if (c.n < DRV_LAUNCH_K) c.n++;
    c.mean += (x - c.mean) / (float)c.n;   // (the running mean of the first K, then a mean over about the last K)
    s_last_launch.cls = LA.cls; s_last_launch.n = c.n; s_last_launch.x = x;
    s_gen++;
    return DRV_EV_LAUNCH;
}

static int step_launch(const driver_sample *s)
{
    const bool rest = s->v < DRV_REST;
    if (!LA.on) {
        if (rest && s->gear >= 1 && s->pedal >= DRV_OFF && s->brake <= DRV_BRAKE && s->mass > 0.0f && s->launch_share == s->launch_share) {
            LA.on = 1; LA.cls = driver_launch_class(s->mass); LA.mass = s->mass; LA.g0 = s->gear; LA.moved = 0;
            LA.t_prev = s->t; LA.tsum = LA.sum = 0.0;
        }
        return 0;
    }
    const double dt = s->t - LA.t_prev;
    LA.t_prev = s->t;
    if (s->brake > DRV_BRAKE || !(s->launch_share == s->launch_share)) { LA.on = 0; return 0; }
    if (!rest) LA.moved = 1;
    if (s->gear != LA.g0) return launch_finish();                 // the first shift ends the pull-away in the start gear
    if (s->pedal < DRV_OFF) { if (LA.moved) return launch_finish(); LA.on = 0; return 0; }
    LA.sum += (double)s->launch_share * dt;
    LA.tsum += dt;
    if (LA.tsum >= DEMAND_HORIZON_S) return launch_finish();
    return 0;
}

float driver_launch_share(float mass_kg)
{
    const launch_cls &c = s_launch[driver_launch_class(mass_kg)];
    if (c.n == 0) return DRV_LAUNCH_PRIOR;
    const float est = (DRV_PRIOR_N * DRV_LAUNCH_PRIOR + (float)c.n * c.mean) / ((float)c.n + DRV_PRIOR_N);
    return clampf(est, DRV_LAUNCH_PRIOR / DRV_BOUND, DRV_LAUNCH_PRIOR * DRV_BOUND);
}
int driver_launch_n(float mass_kg) { return s_launch[driver_launch_class(mass_kg)].n; }

// ---------------------------------------------------------------- the frame
static double s_last_t = NAN;

void driver_drop_open()
{
    L.on = 0; L.pk = 0.0f;
    PR.init = 0;
    LA.on = 0;
}

int driver_step(const driver_sample *s)
{
    if (!s) return 0;
    // a continuous pedal: the clock moves forward by less than a gap; the driver's pedal drives; in a forward gear, or in the gap of a shift: the game reports gear 0
    // while the clutch is open (core/brake.cpp and decel.cpp read T.gear == 0 and speed as 'the gap'), so a lift or a press that contains a shift goes on through it;
    // a neutral at a standstill (revving) and reverse are not driving; a pull-away starts in a forward gear at rest.
    const bool gap = !(s_last_t == s_last_t) || s->t < s_last_t || s->t - s_last_t > DRV_GAP_S;
    s_last_t = s->t;
    const bool ok = s->own && s->pedal == s->pedal && (s->gear >= 1 || (s->gear == 0 && s->v >= DRV_REST));
    if (gap || !ok) driver_drop_open();
    if (!ok) return 0;
    int ev = 0;
    ev |= step_lift(s);
    ev |= step_press(s);
    ev |= step_launch(s);
    return ev;
}

void driver_describe(int ev, char *buf, size_t n)
{
    if (n == 0) return;
    buf[0] = 0;
    size_t len = 0;
    if (ev & DRV_EV_LIFT) {
        int re, br, to;
        count_lifts(&re, &br, &to);
        len += snprintf(buf + len, n - len, "Driver: lift of %.2f s ended in %s at %d km/h; %d lifts kept (%d re-press, %d brake, %d still lifted); a lift at that speed ends in the brake %.2f (%d lifts in its band)",
                        s_last_lift.cs / 100.0, s_last_lift.o == LF_RE ? "a re-press" : s_last_lift.o == LF_BRAKE ? "braking" : "nothing (timeout)", s_last_lift.kmh, s_lifts.n, re, br, to,
                        driver_lift_brake_p(s_last_lift.kmh / 3.6f), driver_lift_brake_n(s_last_lift.kmh / 3.6f));
    }
    if ((ev & DRV_EV_LAUNCH) && len < n) {
        const int c = s_last_launch.cls;
        len += snprintf(buf + len, n - len, "%sDriver: launch at %d-%d t (class %d): mean share %.2f; his usual there now %.2f (today's %.2f, %d launches kept in the class)", len ? "; " : "",
                        1 << c, 2 << c, c, s_last_launch.x, driver_launch_share(1500.0f * (float)(1 << c)), DRV_LAUNCH_PRIOR, s_last_launch.n);
    }
    if ((ev & DRV_EV_PRESS) && len < n)
        len += snprintf(buf + len, n - len, "%sDriver: press %.2f -> %.2f at %.1f/s; fast for him now %.2f/s (today's fixed %.1f, %d presses kept%s)", len ? "; " : "",
                        s_last_press.from, s_last_press.to, s_last_press.rate, driver_press_scale(), DRV_PRESS_RATE_PRIOR, s_presses.n,
                        s_presses.n < (int)lroundf(1.0f / (1.0f - DRV_PRESS_Q)) ? ", too few to learn from" : "");
}

void driver_reset()
{
    demand_reset();
    s_lifts.clear();
    s_last_lift = lift_ev();
    s_presses.clear();
    s_ends.clear();
    memset(s_above, 0, sizeof s_above);
    s_last_press = {};
    memset(s_launch, 0, sizeof s_launch);
    s_last_launch = {};
    driver_drop_open();
    s_last_t = NAN;
    s_gen++;
}

void driver_summary(char *buf, size_t n)
{
    int re, br, to, launches = 0, classes = 0;
    count_lifts(&re, &br, &to);
    for (int i = 0; i < DRV_LAUNCH_CLASSES; i++) if (s_launch[i].n) { launches += s_launch[i].n; classes++; }
    snprintf(buf, n, "%d s of demand, %d lifts (%d re-press, %d brake, %d still lifted), %d presses (fast = %.2f/s, his pedal ends at %.2f), %d launches in %d load classes", demand_seconds(), s_lifts.n, re, br, to,
             s_presses.n, driver_press_scale(), driver_press_end(), launches, classes);
}

// ---------------------------------------------------------------- the text
namespace {

struct wr {
    char *b; size_t n, len; bool ok;
    void f(const char *fmt, ...)
    {
        if (!ok) return;
        va_list ap; va_start(ap, fmt);
        int m = vsnprintf(b + len, n - len, fmt, ap);
        va_end(ap);
        if (m < 0 || (size_t)m >= n - len) { ok = false; return; }
        len += (size_t)m;
    }
};

// one integer in lo..hi at p (an optional '-' and 1..9 digits); p moves past it
bool rd_int(const char *&p, long lo, long hi, long *v)
{
    bool neg = *p == '-';
    if (neg) p++;
    if (*p < '0' || *p > '9') return false;
    long x = 0; int digits = 0;
    while (*p >= '0' && *p <= '9') { if (++digits > 9) return false; x = x * 10 + (*p - '0'); p++; }
    if (neg) x = -x;
    if (x < lo || x > hi) return false;
    *v = x;
    return true;
}
bool rd_char(const char *&p, char c) { if (*p != c) return false; p++; return true; }

// demand=<n>:<per mille>,<per mille>,...   the history's seconds, oldest first, as 0..1000 (a share of 0.001)
void put_demand(wr &w)
{
    static float sec[DEMAND_MEM_S];
    const int n = demand_export(sec, DEMAND_MEM_S);
    w.f("demand=%d:", n);
    for (int i = 0; i < n; i++) {
        int pm = (int)lroundf(sec[i] * 1000.0f);
        w.f(i ? ",%d" : "%d", pm < 0 ? 0 : pm > 1000 ? 1000 : pm);
    }
    w.f("\n");
}
bool get_demand(const char *p, float *out, int *n)
{
    long cnt;
    if (!rd_int(p, 0, DEMAND_MEM_S, &cnt) || !rd_char(p, ':')) return false;
    for (long i = 0; i < cnt; i++) {
        long pm;
        if (i && !rd_char(p, ',')) return false;
        if (!rd_int(p, 0, 1000, &pm)) return false;
        out[i] = (float)pm / 1000.0f;
    }
    if (*p != 0) return false;
    *n = (int)cnt;
    return true;
}

// lifts=<n>:<centiseconds>/<r|b|t>/<km/h>,...   the lifts, oldest first
void put_lifts(wr &w)
{
    w.f("lifts=%d:", s_lifts.n);
    for (int i = 0; i < s_lifts.n; i++) { const lift_ev &e = s_lifts.at(i); w.f(i ? ",%u/%c/%u" : "%u/%c/%u", (unsigned)e.cs, e.o, (unsigned)e.kmh); }
    w.f("\n");
}
bool get_lifts(const char *p, lift_ev *out, int *n)
{
    long cnt;
    if (!rd_int(p, 0, DRV_EVENTS, &cnt) || !rd_char(p, ':')) return false;
    for (long i = 0; i < cnt; i++) {
        long cs, kmh;
        if (i && !rd_char(p, ',')) return false;
        if (!rd_int(p, 1, (long)(DRV_LIFT_TIMEOUT_S * 100.0f), &cs) || !rd_char(p, '/')) return false;
        const char o = *p;
        if (o != LF_RE && o != LF_BRAKE && o != LF_TIMEOUT) return false;
        p++;
        if (!rd_char(p, '/') || !rd_int(p, 0, 400, &kmh)) return false;
        out[i].cs = (unsigned short)cs; out[i].o = o; out[i].kmh = (unsigned short)kmh;
    }
    if (*p != 0) return false;
    *n = (int)cnt;
    return true;
}

// press=<n>:<0.01 pedal/s>,...   the presses' rates, oldest first
void put_press(wr &w)
{
    w.f("press=%d:", s_presses.n);
    for (int i = 0; i < s_presses.n; i++) w.f(i ? ",%u" : "%u", (unsigned)s_presses.at(i));
    w.f("\n");
}
// ends=<n>:<per mille>,...   the held tops of his presses that came to the end of the pedal, oldest first (absent in files of before WP-0911-K1)
void put_ends(wr &w)
{
    w.f("ends=%d:", s_ends.n);
    for (int i = 0; i < s_ends.n; i++) w.f(i ? ",%u" : "%u", (unsigned)s_ends.at(i));
    w.f("\n");
}
bool get_ends(const char *p, unsigned short *out, int *n)
{
    long cnt;
    if (!rd_int(p, 0, DRV_EVENTS, &cnt) || !rd_char(p, ':')) return false;
    for (long i = 0; i < cnt; i++) {
        long r;
        if (i && !rd_char(p, ',')) return false;
        if (!rd_int(p, (long)lroundf(DRV_END_LO * 1000.0f), 1000, &r)) return false;
        out[i] = (unsigned short)r;
    }
    if (*p != 0) return false;
    *n = (int)cnt;
    return true;
}
bool get_press(const char *p, unsigned short *out, int *n)
{
    long cnt;
    if (!rd_int(p, 0, DRV_EVENTS, &cnt) || !rd_char(p, ':')) return false;
    for (long i = 0; i < cnt; i++) {
        long r;
        if (i && !rd_char(p, ',')) return false;
        if (!rd_int(p, 1, 99900, &r)) return false;
        out[i] = (unsigned short)r;
    }
    if (*p != 0) return false;
    *n = (int)cnt;
    return true;
}

// launch=<n>:<class>/<launches>/<mean per mille>,...   the classes that have launches
void put_launch(wr &w)
{
    int n = 0;
    for (int i = 0; i < DRV_LAUNCH_CLASSES; i++) if (s_launch[i].n) n++;
    w.f("launch=%d:", n);
    for (int i = 0, k = 0; i < DRV_LAUNCH_CLASSES; i++) {
        if (!s_launch[i].n) continue;
        w.f(k++ ? ",%d/%u/%d" : "%d/%u/%d", i, (unsigned)s_launch[i].n, (int)lroundf(clampf(s_launch[i].mean, 0.0f, 1.0f) * 1000.0f));
    }
    w.f("\n");
}
bool get_launch(const char *p, launch_cls *out)   // out[DRV_LAUNCH_CLASSES], zeroed by the caller; a class twice is damage
{
    long cnt;
    if (!rd_int(p, 0, DRV_LAUNCH_CLASSES, &cnt) || !rd_char(p, ':')) return false;
    for (long i = 0; i < cnt; i++) {
        long c, n, pm;
        if (i && !rd_char(p, ',')) return false;
        if (!rd_int(p, 0, DRV_LAUNCH_CLASSES - 1, &c) || !rd_char(p, '/') || !rd_int(p, 1, DRV_LAUNCH_K, &n) || !rd_char(p, '/') || !rd_int(p, 0, 1000, &pm)) return false;
        if (out[c].n) return false;
        out[c].n = (unsigned char)n; out[c].mean = (float)pm / 1000.0f;
    }
    return *p == 0;
}

}  // namespace

static const char FILE_HEAD[] = "dazdrive-driver v=1";

size_t driver_serialize(char *buf, size_t n)
{
    wr w = { buf, n, 0, n > 0 };
    w.f("%s\n", FILE_HEAD);
    put_demand(w);
    put_lifts(w);
    put_press(w);
    put_ends(w);
    put_launch(w);
    w.f("end\n");
    return w.ok ? w.len : 0;
}

int driver_parse(const char *text)
{
    if (!text) return 0;
    static float dem[DEMAND_MEM_S];
    lift_ev lifts[DRV_EVENTS];
    unsigned short press[DRV_EVENTS], ends[DRV_EVENTS];
    launch_cls launch[DRV_LAUNCH_CLASSES] = {};
    int n_dem = -1, n_lifts = -1, n_press = -1, n_ends = 0, have_launch = 0, ended = 0, first = 1;
    for (const char *ln = text; *ln && !ended;) {
        const char *nl = strchr(ln, '\n');
        char line[8192];
        size_t len = nl ? (size_t)(nl - ln) : strlen(ln);
        if (len >= sizeof line) return 0;
        memcpy(line, ln, len);
        if (len && line[len - 1] == '\r') len--;
        line[len] = 0;
        ln = nl ? nl + 1 : ln + strlen(ln);
        if (first) { if (strcmp(line, FILE_HEAD) != 0) return 0; first = 0; continue; }
        if (!strcmp(line, "end")) { ended = 1; break; }
        char *eq = strchr(line, '=');
        if (!eq) return 0;   // a line that is neither a key nor the end: damaged
        *eq = 0;
        const char *key = line, *val = eq + 1;
        if (!strcmp(key, "demand")) {
            if (n_dem >= 0 || !get_demand(val, dem, &n_dem)) return 0;
        } else if (!strcmp(key, "lifts")) {
            if (n_lifts >= 0 || !get_lifts(val, lifts, &n_lifts)) return 0;
        } else if (!strcmp(key, "press")) {
            if (n_press >= 0 || !get_press(val, press, &n_press)) return 0;
        } else if (!strcmp(key, "ends")) {
            if (n_ends > 0 || !get_ends(val, ends, &n_ends)) return 0;
        } else if (!strcmp(key, "launch")) {
            if (have_launch || !get_launch(val, launch)) return 0;
            have_launch = 1;
        }
        // (unknown keys: skipped)
    }
    if (first || !ended) return 0;
    // every part was valid: now it goes in
    driver_reset();
    if (n_dem > 0) demand_import(dem, n_dem);
    for (int i = 0; i < n_lifts; i++) s_lifts.add(lifts[i]);
    for (int i = 0; i < n_press; i++) s_presses.add(press[i]);
    for (int i = 0; i < n_ends; i++) s_ends.add(ends[i]);
    memcpy(s_launch, launch, sizeof s_launch);
    return 1;
}
