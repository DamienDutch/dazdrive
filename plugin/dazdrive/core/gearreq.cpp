// The gear arbiter (core/gearreq.h): the best request of a decision so far, nothing more. Only the winner matters, so a post is compared with it
// at once and the rest is dropped; no list, no allocation, a decision costs a few stores. The refused posts are kept only for the one log line a
// changed outcome writes (level 2).
#include "gearreq.h"
#include "suite.h"
#include "shiftcommon.h"

static gear_req g_best;
static int g_have;

enum { GR_REFUSED_MAX = 16 };
static struct {
    gear_req refused[GR_REFUSED_MAX];
    int n_refused;
    int have_last;      // the last resolve had a winner
    gear_req last;      // ... and this was it
} g_note;

static const char *const k_prio[GP_COUNT] = { "protect", "owner", "braking", "planner", "soft" };
static const char *const k_why[W_COUNT] = {
#define X(id, text) text,
    WHY_LIST(X)
#undef X
};

static const char *prio_name(int p) { return p >= 0 && p < GP_COUNT ? k_prio[p] : "?"; }
static const char *why_name(int w) { return w > W_NONE && w < W_COUNT ? k_why[w] : NULL; }

static void req_name(const gear_req *r, char *b, size_t n)
{
    const char *w = why_name(r->why);
    char g[24];
    if (r->gear == GR_NEUTRAL) snprintf(g, sizeof g, "N");
    else snprintf(g, sizeof g, "gear %d", r->gear);
    if (w) snprintf(b, n, "%s (%s, %s)", g, prio_name(r->prio), w);
    else snprintf(b, n, "%s (%s)", g, prio_name(r->prio));
}

static void note_refused(const gear_req *r)
{
    if (g_note.n_refused < GR_REFUSED_MAX) g_note.refused[g_note.n_refused++] = *r;
}

void gear_req_begin() { g_have = 0; g_note.n_refused = 0; }

void gear_req_post(const gear_req *r)
{
    // (the first post of a priority keeps it: a rule that posts later at the same priority did not run before the first; that is the order the
    // brain's ladder had when it was a chain of returns)
    if (!g_have || r->prio < g_best.prio) {
        if (g_have) note_refused(&g_best);   // (a later, higher post takes the lead: the old one lost)
        g_best = *r;
        g_have = 1;
    } else note_refused(r);
}

int gear_req_resolve(int *why)
{
    if (!g_have) {
        if (g_note.have_last) {
            char was[160];
            req_name(&g_note.last, was, sizeof was);
            logv(2, "Gear arbiter: nobody posted (was %s)", was);
        }
        g_note.have_last = 0;
        g_note.n_refused = 0;
        return GR_NONE;
    }
    if (!g_note.have_last || g_best.gear != g_note.last.gear || g_best.prio != g_note.last.prio || g_best.why != g_note.last.why) {
        char win[160], refused[320];
        size_t off = 0;
        req_name(&g_best, win, sizeof win);
        if (!g_note.n_refused) snprintf(refused, sizeof refused, "none");
        else {
            refused[0] = 0;
            for (int i = 0; i < g_note.n_refused; i++) {
                char one[128];
                req_name(&g_note.refused[i], one, sizeof one);
                int k = snprintf(refused + off, sizeof refused - off, "%s%s", i ? ", " : "", one);
                if (k < 0 || (size_t)k >= sizeof refused - off) { off = sizeof refused; break; }
                off += (size_t)k;
            }
        }
        logv(2, "Gear arbiter: %s wins; refused: %s", win, refused);
    }
    g_note.last = g_best;
    g_note.have_last = 1;
    g_note.n_refused = 0;
    g_have = 0;
    if (why) *why = g_best.why;
    return g_best.gear;
}

// ---------------------------------------------------------------- standing requests
// One slot per owner, a seqlock (the writer bumps the sequence to odd, writes, bumps it to even; the reader retries on an odd or a changed sequence): the owners
// post from the game's pedal function, the brain reads in the gearbox's update.
static struct { volatile unsigned seq; gear_stand s; int have; } g_stand[GSO_N];

void gear_stand_post(int owner, const gear_stand *s)
{
    if (owner < 0 || owner >= GSO_N) return;
    auto &o = g_stand[owner];
    o.seq++; __sync_synchronize();
    o.s = *s; o.have = 1;
    __sync_synchronize(); o.seq++;
}
void gear_stand_clear(int owner)
{
    if (owner < 0 || owner >= GSO_N) return;
    auto &o = g_stand[owner];
    if (!o.have) return;
    o.seq++; __sync_synchronize();
    o.have = 0;
    __sync_synchronize(); o.seq++;
}
int gear_stand_get(gear_stand *out, int *owner_mask)
{
    gear_stand m = { 0.0f, GSK_SUSTAINED, 0, 0.0 };
    int mask = 0;
    for (int i = 0; i < GSO_N; i++) {
        gear_stand c = { 0.0f, GSK_SUSTAINED, 0, 0.0 }; int have = 0, got = 0;
        for (int tries = 0; tries < 4 && !got; tries++) {
            const unsigned a = g_stand[i].seq;
            if (a & 1) continue;
            __sync_synchronize();
            c = g_stand[i].s; have = g_stand[i].have;
            __sync_synchronize();
            got = g_stand[i].seq == a;
        }
        if (!got || !have) continue;   // (a writer was busy all four times: this owner's last post is not read this once, the next decision reads it)
        if (c.drag_n > m.drag_n) m.drag_n = c.drag_n;
        if (c.drag_n > 0 && c.kind == GSK_LEVER) m.kind = GSK_LEVER;   // (a lever the driver holds is asked as it is, whoever else asks)
        if (c.eb_held) m.eb_held = 1;
        if (c.eb_alloc) m.eb_alloc = 1;
        if (c.hold_until > m.hold_until) m.hold_until = c.hold_until;
        mask |= 1 << i;
    }
    if (out) *out = m;
    if (owner_mask) *owner_mask = mask;
    return mask != 0;
}
