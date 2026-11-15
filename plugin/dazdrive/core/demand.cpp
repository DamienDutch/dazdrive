// The driver's demand history (core/demand.h): a ring of the per-second largest wish share, and the probability that a demand of a given size
// comes within a window. Pure arithmetic: no module, no vehicle model, so test_demand.cpp links it alone.
#include "demand.h"
#include <math.h>

// Two memories of the same kind (WP-0911-K1): the DRIVER's (the pedal, kept in the profile on disk) and a CONTROLLER's (the cruise's and the planner's own throttle share, this session only,
// never written anywhere: the minutes the truck's controllers asked for, so the gear choice can price the return of a climb under a cruise as it prices the driver's). One state each.
struct dstate {
    float buf[DEMAND_MEM_S];   // the shares, oldest first from (head - n)
    int n, head;               // entries held, the next slot to write
    int nload;                 // the oldest nload entries came from a file (demand_import): they weigh less (demand_p)
    unsigned gen;              // counts the changes (demand_gen)
    int dirty;                 // the memory changed since the view was computed
    demand_view view;
    long sec; float sec_max;   // the whole second the open accumulator belongs to (-1 = none yet) and its largest share (NAN = none)
};
static dstate g_drv = { {0}, 0, 0, 0, 0, 1, {}, -1, NAN }, g_ctl = { {0}, 0, 0, 0, 0, 1, {}, -1, NAN };
static dstate *D = &g_drv;   // the memory the functions below work on (the driver's unless demand_ctl_* is on the stack)
#define s_buf D->buf
#define s_n D->n
#define s_head D->head
#define s_nload D->nload
#define s_gen D->gen
#define s_dirty D->dirty
#define s_view D->view
#define s_sec D->sec
#define s_sec_max D->sec_max
static const float LEVELS[DEMAND_LEVELS] = { DEMAND_LEVEL_LIGHT, 0.5f, 0.7f, 0.9f };

void demand_note(float share)
{
    if (!(share == share)) return;
    if (s_n == DEMAND_MEM_S && s_nload > 0) s_nload--;   // (the oldest second leaves the ring: a loaded one first)
    s_buf[s_head] = share < 0.0f ? 0.0f : share > 1.0f ? 1.0f : share;
    s_head = (s_head + 1) % DEMAND_MEM_S;
    if (s_n < DEMAND_MEM_S) s_n++;
    s_dirty = 1;
    s_gen++;
}

void demand_tick(double now_s, float share)
{
    if (!(now_s == now_s)) return;
    const long sec = (long)floor(now_s);
    if (sec != s_sec) {
        if (s_sec >= 0) demand_note(s_sec_max);   // (NAN = nothing in that second: not recorded)
        s_sec = sec;
        s_sec_max = NAN;
    }
    if (share == share && !(share <= s_sec_max)) s_sec_max = share;   // (the comparison is false for NAN: the first share starts the maximum)
}

int demand_seconds() { return s_n; }
unsigned demand_gen() { return s_gen; }

// The weight of the second at oldest-first index i (0..s_n-1): 1 for a second of this session, and for a second that came from a file the share of the
// memory it has left, 1 - age / DEMAND_MEM_S, where age is the driving seconds since it was noted (the ring position from the newest: the ring holds only
// seconds the driver's pedal was the demand, so driving seconds are its clock). Linear, because the ring drops a second after DEMAND_MEM_S of them: the
// weight reaches 0 exactly when the second leaves, so the loaded history fades out instead of ending at a step.
static float weight_at(int i) { return i < s_nload ? 1.0f - (float)(s_n - 1 - i) / (float)DEMAND_MEM_S : 1.0f; }

// The windows the memory holds for a level: the sum of their values (a hit, blended with the prior by the weight of the window's oldest second), how many are filled, how many a full memory
// holds, and the prior. Both probabilities below read it: they differ only in what the windows not filled yet are worth.
struct dwin { float evidence, prior; int filled, total; };
static dwin windows_of(float level, float horizon_s)
{
    int h = (int)ceilf(horizon_s);
    if (h < 1) h = 1;
    if (h > DEMAND_MEM_S) h = DEMAND_MEM_S;
    dwin w;
    w.total = DEMAND_MEM_S - h + 1;                          // the windows a full memory holds
    w.filled = s_n >= h ? s_n - h + 1 : 0;                   // the windows the memory holds now
    const float l = level < 0.0f ? 0.0f : level > 1.0f ? 1.0f : level;
    w.prior = 1.0f - powf(l, (float)h);
    w.evidence = 0.0f;                                       // the windows' values: a hit, blended with the prior by the weight of the window's oldest second
    int last = -1;                                           // last: the (oldest-first) index of the latest second at or over the level
    for (int i = 0; i < s_n; i++) {
        if (s_buf[(s_head - s_n + i + 2 * DEMAND_MEM_S) % DEMAND_MEM_S] >= level) last = i;
        if (i >= h - 1) {
            const int hit = last >= i - h + 1;               // the window ending at i holds one
            const float wt = weight_at(i - h + 1);           // (a window is as old as its oldest second; all 1 when nothing was loaded: hits as before)
            w.evidence += wt * (float)hit + (1.0f - wt) * w.prior;
        }
    }
    return w;
}

float demand_p(float level, float horizon_s)
{
    const dwin w = windows_of(level, horizon_s);
    return (w.evidence + (w.total - w.filled) * w.prior) / (float)w.total;
}

// The same windows, the foot as it has been SEEN: what is not seen yet is worth one window of the prior, not the whole unfilled memory. The unknown foot is one observation (the rule of succession's
// one) and every window the foot has really been seen in is one more, so a foot that holds a speed calmly has shown it within the minute and not after ten. (demand_p keeps the unfilled memory at the
// prior for the reserve of a lift: nothing is known of a foot that has just left the pedal.) A full memory is the same number, an empty one is the prior.
float demand_p_seen(float level, float horizon_s)
{
    const dwin w = windows_of(level, horizon_s);
    const float wp = fminf((float)(w.total - w.filled), 1.0f);
    return (w.evidence + wp * w.prior) / ((float)w.filled + wp);
}

float demand_level_at(const demand_view *v, float p_scale, float q)
{
    const float target = p_scale > 1e-6f ? (1.0f - q) / p_scale : 2.0f;   // the probability the quantile's level has: scaled p = 1 - q
    if (target >= 1.0f) return 0.0f;   // the driver is there in under 3 of 10 windows (or never): the quantile is a demand of nothing
    float l0 = 0.0f, p0 = 1.0f;
    for (int i = 0; i <= DEMAND_LEVELS; i++) {
        const float l1 = i < DEMAND_LEVELS ? v->level[i] : 1.0f, p1 = i < DEMAND_LEVELS ? v->p[i] : 0.0f;
        if (p1 <= target) return p0 > p1 ? fmaxf(0.0f, l0 + (l1 - l0) * (p0 - target) / (p0 - p1)) : l1;
        l0 = l1; p0 = p1;
    }
    return 1.0f;
}

void demand_frame()
{
    if (!s_dirty) return;
    s_view.horizon_s = DEMAND_HORIZON_S;
    for (int q = 0; q < DEMAND_LEVELS; q++) {
        s_view.level[q] = LEVELS[q];
        s_view.p[q] = demand_p(LEVELS[q], DEMAND_HORIZON_S);
        s_view.p_seen[q] = demand_p_seen(LEVELS[q], DEMAND_HORIZON_S);
    }
    s_dirty = 0;
}

const demand_view *demand_now()
{
    demand_frame();   // (a client before the first frame sees the prior, not zeros)
    return &s_view;
}

int demand_export(float *out, int max)
{
    const int n = s_n < max ? s_n : max;
    for (int i = 0; i < n; i++)   // the newest n, oldest first
        out[i] = s_buf[(s_head - n + i + 2 * DEMAND_MEM_S) % DEMAND_MEM_S];
    return n;
}

int demand_import(const float *shares, int n)
{
    if (s_n != 0 || n <= 0) return 0;   // live seconds win over a file; a start loads before the first second
    if (n > DEMAND_MEM_S) { shares += n - DEMAND_MEM_S; n = DEMAND_MEM_S; }
    for (int i = 0; i < n; i++) {
        const float x = shares[i];
        s_buf[i] = !(x == x) || x < 0.0f ? 0.0f : x > 1.0f ? 1.0f : x;
    }
    s_n = n;
    s_head = n % DEMAND_MEM_S;
    s_nload = n;
    s_dirty = 1;
    s_gen++;
    return n;
}

void demand_reset()
{
    s_n = s_head = s_nload = 0;
    s_dirty = 1;
    s_gen++;
    s_sec = -1;
    s_sec_max = NAN;
}

// ---- the controller's memory (this session only)
struct ctl_scope { dstate *old; ctl_scope() : old(D) { D = &g_ctl; } ~ctl_scope() { D = old; } };
void demand_ctl_tick(double now_s, float share) { ctl_scope sc; demand_tick(now_s, share); }
const demand_view *demand_ctl_now() { ctl_scope sc; demand_frame(); return &s_view; }
int demand_ctl_seconds() { return g_ctl.n; }
void demand_ctl_reset() { ctl_scope sc; demand_reset(); }
