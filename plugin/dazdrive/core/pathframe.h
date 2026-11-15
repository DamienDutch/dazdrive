// Path frame for the cut-in reader (modules/cutin.cpp): where another vehicle sits along and across OUR expected path, instead of in our
// straight frame. In a bend our frame turns with the truck, so a car in the next lane seems to slide sideways; along the path it does not.
// Pure math, no game access: the sim (tools/sim/sim_pathframe.cpp) and tools/analyze_cutin.py run exactly this.
//
// Our frame: dx ahead, dy to the left (core/trafficmath.h). The expected path is a constant-curvature arc through the origin along dx,
// curvature k in 1/m, + = turning left. A point is described by s (distance along the arc) and d (m to the left of it, + = left).
// Angles in radians, + = counter-clockwise (left).
//
// Every event carries the values it was decided on (pf_out: the lateral speeds, the movement, the hold, the projected position) next to the
// thresholds of pf_cfg, and every state in which nothing is classified names its reason (pf_bad_reason): the module logs both (promise P7).
#pragma once
#include <math.h>

// (dx, dy) -> (s, d). A curvature under 1e-5 is a straight path.
static inline void pf_to_path(double k, double dx, double dy, double *s, double *d)
{
    if (fabs(k) < 1e-5) { *s = dx; *d = dy; return; }
    double sg = k > 0 ? 1.0 : -1.0, R = 1.0 / fabs(k), y = dy * sg;   // mirrored to a left bend: the centre is at (0, R)
    double rho = sqrt(dx * dx + (R - y) * (R - y));
    *s = R * atan2(dx, R - y);
    *d = sg * (R - rho);
}

// The direction of the path at s, relative to our heading now.
static inline double pf_tangent(double k, double s) { return k * s; }

// How fast a vehicle (speed v along its own heading hdg_rel, relative to ours) moves ACROSS the path at s: + = to the left of it.
static inline double pf_lat_speed(double k, double s, double hdg_rel, double v) { return v * sin(hdg_rel - pf_tangent(k, s)); }

// ---------------------------------------------------------------- the classifier of one tracked vehicle

// The thresholds. The traffic list is walked every 200 ms (modules/traffic.cpp TICK_MS), so a "sample" is 0.2 s; the lane is 3.5-3.75 m wide (EU
// 3.5 m, motorway 3.75 m, ATS 12 ft = 3.66 m); a deliberate lane change moves a car across 3.7 m in 3-5 s, 0.75-1.2 m/s. Where a figure is
// derived from these it says so; the others are perception tunings of this reader (no vehicle quantity sets them) and say "not derived".
struct pf_cfg {
    double lane_half;   // m: |d| under this = in our lane
    double next_max;    // m: |d| under this = our lane or the next one
    double v_tow;       // m/s: filtered lateral speed that counts as moving
    double v_agree;     // m/s: the speed from the change of d must point the same way (a car steering inside its lane fools the heading alone)
    double moved;       // m the car has to have really moved sideways since the condition began
    double hold;        // s the condition has to hold
    double tau_v;       // s lateral speed filter
    double k_max;       // 1/m: tighter than this nothing is classified (roundabouts, junction arms: an arc is no model of the road there)
    double v_min;       // m/s our speed under which nothing is classified
    double s_max;       // m
    double horizon;     // s the lateral movement is carried forward
    double again;       // s before the same event can fire for the same vehicle
    double tau_d;       // s filter of the speed from the change of d
    double margin;      // m: a cut-in must end this far inside the lane edge, a lead leaving this far outside it
    double tau_min;     // s: the shortest look-ahead (the time to reach the car along the path, floored)
    double closing_min; // m/s: slower closing than this is "not closing" (no time to reach it)
    double gap_s;       // s: a track without a sample for this long starts again
    int n_min;          // samples a track needs before it is classified
};
static inline pf_cfg pf_default_cfg()
{
    pf_cfg c;
    c.lane_half = 1.8;   // half a lane (3.5-3.75 m)
    c.next_max = 5.8;    // the far edge of the next lane: its middle 3.7 m + half a lane 1.8 m = 5.5 m, + 0.3 m of the AI cars' own wobble inside their lane (not derived)
    c.v_tow = 0.5;       // about half of the slowest deliberate lane change (3.7 m in 5 s = 0.74 m/s); not derived below that
    c.v_agree = 0.25;    // half of v_tow: the second estimate must agree in sign and at least in half the size; not derived
    c.moved = 0.35;      // a tenth of a lane change, more than the jitter of d between two walks; not derived
    c.hold = 0.4;        // two walks of the list
    c.tau_v = 0.4;       // two walks of the list
    c.k_max = 0.012;     // 1/83 m: k x s_max = 1.2 rad, a path that turns 69 degrees inside the range is a junction or a roundabout, no lane
    c.v_min = 8.0;       // 29 km/h: under it a town or a junction, where cars cross our path and an arc through the origin is no model; not derived
    c.s_max = 100.0;     // inside the list's reach (130 m, the chord; in a bend the arc is longer, so 100 m along the path stays inside it)
    c.horizon = 3.0;     // the time a lane change takes (3-5 s), the low end: "inside our lane within about 3 s"
    c.again = 5.0;       // the high end of a lane change: one lane change is one event
    c.tau_d = 0.6;       // three walks of the list
    c.margin = 0.2;      // not derived (a tenth of a car's width)
    c.tau_min = 0.8;     // about a driver's reaction time (0.75 s): a car that closes faster is not looked at further ahead in time than that
    c.closing_min = 0.5; // not derived (the noise of the speeds of two cars)
    c.gap_s = 1.0;       // five walks of the list
    c.n_min = 3;         // two filter intervals
    return c;
}

enum { PF_BAD_NONE = 0, PF_BAD_INDICATOR, PF_BAD_BEND, PF_BAD_SLOW, PF_BAD_N };   // why nothing is classified (pf_bad_reason)
static inline const char *pf_bad_name(int r)
{
    static const char *const n[PF_BAD_N] = { "classifying", "our indicator", "tight bend", "too slow" };
    return r >= 0 && r < PF_BAD_N ? n[r] : "?";
}
// Why nothing is classified now (for every vehicle at once), 0 = nothing is in the way: `suppress` = our own indicator is on or the path is not
// trustworthy; a curvature over k_max; our speed under v_min. (A track with too few samples is not classified either, which only that track knows.)
static inline int pf_bad_reason(const pf_cfg &c, double k, double v_ego, int suppress)
{
    if (suppress) return PF_BAD_INDICATOR;
    if (fabs(k) > c.k_max) return PF_BAD_BEND;
    if (v_ego < c.v_min) return PF_BAD_SLOW;
    return PF_BAD_NONE;
}

enum { PF_CUTIN = 1, PF_LEAVING = 2 };   // event bits

struct pf_track {
    unsigned id;
    double t;            // time of the last sample, < 0 = none
    double d;            // d at the last sample
    double vh, vd;       // filtered lateral speed: from the heading / from the change of d (m/s, + = left)
    int n;               // samples in this track
    double since[2], d0[2], last[2];   // per event kind (0 cut-in, 1 leaving): when the condition began (< 0 = not holding), d then, when it last fired
    double first_t, first_s, first_d, inlane_t;
};
static inline void pf_track_init(pf_track *T, unsigned id)
{
    T->id = id; T->t = -1; T->d = 0; T->vh = T->vd = 0; T->n = 0;
    for (int i = 0; i < 2; i++) { T->since[i] = -1; T->d0[i] = 0; T->last[i] = -1e9; }
    T->first_t = T->first_s = T->first_d = 0; T->inlane_t = -1;
}

// What the classifier saw. moved / held / proj are per kind (0 cut-in, 1 leaving): how far the car has moved since its condition began (m, 0 = the
// condition does not hold), for how long (s), and where it would be after tau along its heading: |d| for a cut-in (inside the lane when under
// lane_half - margin), |d| for a lead leaving (outside when at least lane_half + margin).
struct pf_out { double s, d, vlat, vd, tau, ttc; double moved[2], held[2], proj[2]; int bad; };

// One sample of one vehicle (already in the path frame: s, d; hdg_rel and v_car as the probe gives them). `suppress` = our own indicator
// is on or the path is not trustworthy. Returns the event bits that fire now (PF_CUTIN, PF_LEAVING).
static inline int pf_step(pf_track *T, const pf_cfg &c, double now, double k, double v_ego, double s, double d, double hdg_rel, double v_car,
                          int suppress, pf_out *o)
{
    if (T->t < 0 || now - T->t > c.gap_s) {
        pf_track_init(T, T->id);
        T->vh = pf_lat_speed(k, s, hdg_rel, v_car);
        T->first_t = now; T->first_s = s; T->first_d = d;
    } else {
        double dt = now - T->t;
        if (dt < 1e-3) dt = 1e-3;
        double a = dt / c.tau_v; if (a > 1) a = 1;
        T->vh += (pf_lat_speed(k, s, hdg_rel, v_car) - T->vh) * a;
        double b = dt / c.tau_d; if (b > 1) b = 1;
        T->vd += ((d - T->d) / dt - T->vd) * b;
    }
    T->t = now; T->d = d; T->n++;
    if (fabs(d) < c.lane_half && T->inlane_t < 0) T->inlane_t = now;
    double sg = d > 0 ? 1.0 : -1.0;
    double vc = v_ego - v_car * cos(hdg_rel - pf_tangent(k, s));   // closing speed along the path
    double tau = vc < c.closing_min ? c.horizon : fmin(c.horizon, fmax(c.tau_min, s / vc));
    int reason = pf_bad_reason(c, k, v_ego, suppress);
    int bad = reason || T->n < c.n_min;
    double toward = -T->vh * sg, toward_d = -T->vd * sg;   // + = toward our path
    double proj_cut = fabs(d) - toward * tau, proj_leave = fabs(d + T->vh * tau);
    int cond[2];
    cond[0] = !bad && fabs(d) >= c.lane_half && fabs(d) < c.next_max && toward > c.v_tow && toward_d > c.v_agree && proj_cut < c.lane_half - c.margin;
    cond[1] = !bad && fabs(d) < c.lane_half && -toward > c.v_tow && -toward_d > c.v_agree && proj_leave >= c.lane_half + c.margin;
    if (o) {
        o->s = s; o->d = d; o->vlat = T->vh; o->vd = T->vd; o->tau = tau; o->ttc = vc > c.closing_min ? s / vc : NAN; o->bad = reason;
        o->moved[0] = o->moved[1] = o->held[0] = o->held[1] = 0; o->proj[0] = proj_cut; o->proj[1] = proj_leave;
    }
    int ev = 0;
    for (int i = 0; i < 2; i++) {
        if (!cond[i]) { T->since[i] = -1; continue; }
        if (T->since[i] < 0) { T->since[i] = now; T->d0[i] = d; }
        double moved = i == 0 ? (T->d0[i] - d) * sg : (d - T->d0[i]) * sg;
        if (o) { o->moved[i] = moved; o->held[i] = now - T->since[i]; }
        if (now - T->since[i] >= c.hold && moved >= c.moved && now - T->last[i] > c.again) { T->last[i] = now; ev |= i == 0 ? PF_CUTIN : PF_LEAVING; }
    }
    return ev;
}
