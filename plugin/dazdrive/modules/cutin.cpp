// Cutin: a LOG-ONLY reader of the AI traffic ahead that sees a car pulling into our lane, or the lead leaving it, before the game's radar
// does. Nothing acts on it (no braking, no ACC change, no tone). It needs the traffic probe ([traffic] probe=1: modules/traffic.cpp
// reads the game's vehicle list, in our frame).
//
// The problem it solves: in our straight frame a car in the next lane seems to slide sideways in every bend (our frame turns with the
// truck). So each vehicle is first put in the frame of our EXPECTED PATH (core/pathframe.h): s along it, d to the left of it, and its
// lateral speed across the path. The path is a constant-curvature arc from the curvature now: the lane assist's road term (o3, 0.25 s
// ahead, when the lane was read in the last 0.2 s), else yaw rate / speed. Over 0-100 m that is a good model on a motorway and a poor
// one at the start and the end of a bend: see tools/analyze_cutin.py (ride 8: the arc halves the bend artifact, our real future path
// removes it). Tighter than R 83 m (roundabouts, junction arms) nothing is classified.
//
// Events (one line each, with the vehicle's s, d, lateral speed, closing speed, our speed and our own indicator; the AI vehicle's own
// indicator is not read from the game yet):
//   cut-in likely   a vehicle in the next lane moves toward our lane (heading and change of d agree, over 0.5 m/s, held 0.4 s, moved 0.35 m)
//                   and would be inside our lane within ~3 s (or before we reach it)
//   lead leaving    a vehicle inside our lane moves out the same way and would be out of it within ~3 s
// Afterwards (up to 8 s) one line says if and how many seconds later the game's radar (the gap cruise.cpp reads) took the new lead
// over, or let the old one go. cutin.csv has the candidates at 5 Hz.
//
// The log (promise P7): an event line carries the values it was decided on next to their thresholds (lateral speed, speed from the change of d,
// movement, hold, projected position); a state in which nothing is classified (our indicator, a tight bend, too slow) is one line at log level 2
// when it begins or ends, naming the reason (core/pathframe.h pf_bad_reason).
#include "../core/suite.h"
#include "../core/traffic.h"
#include "../core/pathframe.h"
#include "../core/lanesense.h"
#include <math.h>
#include <string.h>


static float p_log;

static param g_params[] = {
    { "log", &p_log, 1, "1 = write the cut-in and lead-leaving events to the log, and cutin.csv (5 per second, with the log at its highest level). Needs the traffic probe ([traffic] probe=1). Only logs: nothing is braked, steered or announced from it. 0 = off" },
};

enum { MAX_TRK = 32, MAX_PEND = 6 };   // 32 tracks: the list holds 16 vehicles ahead (core/traffic.h), twice that leaves room for the ones that left the list a moment ago
// The follow-up against the game's radar (below). Perception tunings of this reader, not derived from the vehicle: a cut-in is "taken" when the
// radar gap appears or drops by 4 m (about a car's length, 4.5 m: the new lead is that much nearer than the old one); a lead leaving is "let go"
// when the gap vanishes or grows by 6 m; the radar gets 8 s (two of the longest lane changes, 4 s) to react.
static const float RADAR_TAKEN_M = 4.0f, RADAR_LETGO_M = 6.0f;
static const double RADAR_WAIT_S = 8.0;
static const double CI_IND_HOLD_S = 2.0;    // after our own indicator goes off the lane change is still settling: the path is not where we go (not derived)
static const double CI_STALE_S = 2.0;       // a track unseen this long is dropped (twice pathframe's re-init gap, 1 s)
static const float CI_HDG_MAX_DEG = 50.0f;  // a car turned further than this off our heading is crossing or oncoming, no lane-mate (a lane change turns a car by 3 degrees; 50 is wide on purpose)
static pf_track g_trk[MAX_TRK];
static int g_n_trk;
static FILE *g_csv;
static double g_last_read = -1;
static pf_cfg g_cfg;
static double g_ind_until = -1;
static int g_last_bad = -1;          // the last reason for not classifying a line was written for (-1 = none yet)
static unsigned g_n_cut, g_n_leave;
static char g_status[96] = "off";

struct pend {
    int kind;            // PF_CUTIN / PF_LEAVING
    unsigned id;
    double t, s, d;
    float gap0;          // radar gap when the event fired (NAN = none)
    int active;
};
static pend g_pend[MAX_PEND];

static pf_track *track_of(unsigned id, double now)
{
    for (int i = 0; i < g_n_trk; i++) if (g_trk[i].id == id) return &g_trk[i];
    // drop the stale ones, then take a free slot
    for (int i = 0; i < g_n_trk; i++)
        if (now - g_trk[i].t > CI_STALE_S) { g_trk[i] = g_trk[--g_n_trk]; i--; }
    if (g_n_trk >= MAX_TRK) return NULL;
    pf_track_init(&g_trk[g_n_trk], id);
    return &g_trk[g_n_trk++];
}

static void add_pend(int kind, unsigned id, double now, double s, double d, float gap0)
{
    for (int i = 0; i < MAX_PEND; i++)
        if (!g_pend[i].active) {
            pend &p = g_pend[i];
            p.kind = kind; p.id = id; p.t = now; p.s = s; p.d = d; p.gap0 = gap0; p.active = 1;
            return;
        }
}

// The follow-up against the game's radar: a cut-in is "taken" when the radar gap appears or drops by RADAR_TAKEN_M or more; a lead leaving is "let go"
// when the gap disappears or jumps up by RADAR_LETGO_M or more.
static void radar_check(double now, float gap)
{
    int have = gap == gap;
    for (int i = 0; i < MAX_PEND; i++) {
        pend &p = g_pend[i];
        if (!p.active) continue;
        int hit;
        if (p.kind == PF_CUTIN) hit = have && (!(p.gap0 == p.gap0) || gap < p.gap0 - RADAR_TAKEN_M);
        else hit = !have || (p.gap0 == p.gap0 && gap > p.gap0 + RADAR_LETGO_M);
        if (hit) {
            logline("Cutin: %s id %06x: the radar %s %.1f s after (radar gap %s %.0f m; %s %.0f m; the vehicle was %.0f m ahead when we saw it)",
                    p.kind == PF_CUTIN ? "cut-in" : "lead leaving", p.id, p.kind == PF_CUTIN ? "took it as lead" : "let it go", now - p.t,
                    have ? "now" : "gone, was", have ? gap : (p.gap0 == p.gap0 ? p.gap0 : 0.0f),
                    p.kind == PF_CUTIN ? "taken at a drop of" : "let go at a rise of", p.kind == PF_CUTIN ? RADAR_TAKEN_M : RADAR_LETGO_M, p.s);
            p.active = 0;
        } else if (now - p.t > RADAR_WAIT_S) {
            logline("Cutin: %s id %06x: the radar did not %s within %.0f s (radar gap %s)", p.kind == PF_CUTIN ? "cut-in" : "lead leaving", p.id,
                    p.kind == PF_CUTIN ? "take it as lead" : "let it go", RADAR_WAIT_S, have ? "present" : "none");
            p.active = 0;
        }
    }
}

static void ci_frame()
{
    if (p_log <= 0 || T.paused) return;
    double now = wall_s();
    float gap = g_inc.gap_m;
    radar_check(now, gap);
    if (T.blink_l || T.blink_r) g_ind_until = now + CI_IND_HOLD_S;   // our own lane change: the expected path is not where we go
    traffic_around ta;
    traffic_around_get(&ta);
    if (!ta.ok || ta.read_t == g_last_read) return;
    g_last_read = ta.read_t;

    float v = fabsf(T.speed);
    float k = 0;
    int from_lane = sense_lane_curvature(&k);
    // yaw rate (rotations/s) x 2 pi / speed = curvature; the 3 m/s floor under the speed only keeps a standstill from dividing by zero (nothing is
    // classified under v_min, 8 m/s, anyway)
    if (!from_lane) k = (T.speed >= 0 ? 1.0f : -1.0f) * T.yaw * 6.2831853f / fmaxf(v, 3.0f);
    int ind = (T.blink_l ? 1 : 0) | (T.blink_r ? 2 : 0);
    int suppress = now < g_ind_until;
    int bad = pf_bad_reason(g_cfg, k, v, suppress);
    if (bad != g_last_bad) {   // a state in which nothing is classified: one line when it begins or ends (log level 2)
        const char *ind_s = ind ? (ind == 1 ? "left" : ind == 2 ? "right" : "both") : "off";
        if (bad) logv(2, "Cutin: not classifying: %s (curvature %+.4f 1/m, limit %.4f; speed %.1f m/s, limit %.1f; our indicator %s)", pf_bad_name(bad), k, g_cfg.k_max, v, g_cfg.v_min, ind_s);
        else if (g_last_bad > 0) logv(2, "Cutin: classifying again (it was: %s; curvature %+.4f 1/m, speed %.1f m/s, our indicator %s)", pf_bad_name(g_last_bad), k, v, ind_s);
        g_last_bad = bad;
    }

    for (int i = 0; i < ta.n_ahead; i++) {
        const traffic_obj &o = ta.ahead[i];
        if (o.kind != 0 || !(o.v == o.v) || o.v < 0 || !(o.hdg == o.hdg) || fabsf(o.hdg) > CI_HDG_MAX_DEG) continue;   // trailers, oncoming, crossing, unknown
        double s, d;
        pf_to_path(k, o.dx, o.dy, &s, &d);
        // (3 m beyond the next lane: a car a lane further out is tracked a moment before it can be one, so its filter has a history when it comes in)
        if (!(s > 0 && s < g_cfg.s_max) || fabs(d) > g_cfg.next_max + 3) continue;
        pf_track *T_ = track_of(o.id, now);
        if (!T_) continue;
        pf_out out;
        int ev = pf_step(T_, g_cfg, now, k, v, s, d, o.hdg * 0.017453292519943295, o.v, suppress, &out);
        if (g_csv && fabs(d) < g_cfg.next_max + 1)
            fprintf(g_csv, "%.2f,%06x,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.0f,%.1f,%.4f,%d,%d,%d\n", now, o.id, s, d, out.vlat, out.vd, o.v - v, o.v * 3.6, o.hdg, o.wid, k,
                    from_lane, ind, ev);
        for (int b = 1; b <= 2; b++) {
            if (!(ev & b)) continue;
            if (b == PF_CUTIN) g_n_cut++; else g_n_leave++;
            const int kd = b == PF_CUTIN ? 0 : 1;
            // the values it was decided on, each with its threshold: the lateral speed from the heading, the speed from the change of d, the movement since
            // the condition began, how long it held, and where the car would be after the look-ahead
            logline("Cutin: %s id %06x at %.0f m: d %+.2f m (lane edge %.1f, next lane to %.1f), lateral %+.2f m/s (over %.2f), from d %+.2f (over %.2f), moved %.2f m (over %.2f) in %.1f s (over %.1f), "
                    "within %.1f s |d| is %.2f m (%s %.1f); it goes %.0f km/h, we %.0f, closing %.1f m/s%s; our indicator %s, curvature %+.4f from %s, radar gap %s",
                    b == PF_CUTIN ? "cut-in likely" : "lead leaving", o.id, s, d, g_cfg.lane_half, g_cfg.next_max, out.vlat, g_cfg.v_tow, out.vd, g_cfg.v_agree,
                    out.moved[kd], g_cfg.moved, out.held[kd], g_cfg.hold, out.tau, out.proj[kd], b == PF_CUTIN ? "needs under" : "needs at least",
                    b == PF_CUTIN ? g_cfg.lane_half - g_cfg.margin : g_cfg.lane_half + g_cfg.margin, o.v * 3.6, v * 3.6, v - o.v,
                    out.ttc == out.ttc ? "" : " (not closing)", ind ? (ind == 1 ? "left" : ind == 2 ? "right" : "both") : "off",
                    k, from_lane ? "lane" : "yaw", gap == gap ? "present" : "none");
            if (b == PF_CUTIN) { g_cutin_pred.n++; g_cutin_pred.id = o.id; g_cutin_pred.t = now; g_cutin_pred.s = (float)s; g_cutin_pred.d = (float)d; g_cutin_pred.closing = v - o.v; }   // (published for the lead model, which only logs it)
            if (b == PF_CUTIN || gap == gap) add_pend(b, o.id, now, s, d, gap);   // a lead leaving is only followed up when the radar had a lead
        }
    }
    static double flushed;
    if (g_csv && now - flushed > 2) { flushed = now; fflush(g_csv); csv_cap(&g_csv, "cutin.csv"); }
}

static void ci_start()
{
    g_cfg = pf_default_cfg();
    g_last_bad = -1;
    if (p_log <= 0) return;
    snprintf(g_status, sizeof g_status, "log only, waits for the traffic probe");
    g_csv = open_out("cutin.csv", "w");
    if (g_csv) fputs("t,id,s,d,vlat,vd,rel_v,v_kmh,hdg,wid,k,k_from_lane,ind,ev\n", g_csv);
    logline("Cutin: log only (needs [traffic] probe=1), %s", g_csv ? "cutin.csv at 5 Hz" : "no csv");
}

static void ci_reset(int, const char *)
{
    g_n_trk = 0;
    g_last_read = -1;
    g_last_bad = -1;
    for (int i = 0; i < MAX_PEND; i++) g_pend[i].active = 0;
}

static const char *ci_status() { return g_status; }

static void ci_stop()
{
    if (g_csv) fflush(g_csv);
    if (p_log > 0) logline("Cutin: %u cut-in and %u lead-leaving events this session", g_n_cut, g_n_leave);
}

module m_cutin = { "cutin", "Cutin", g_params, sizeof g_params / sizeof g_params[0], ci_start, ci_frame, NULL, ci_stop, ci_reset, ci_status, NULL, VCM_BOTH };
