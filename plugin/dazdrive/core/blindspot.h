// Blind-spot geometry for the traffic probe (modules/traffic.cpp): two side zones that cover the truck AND the articulated trailer, and
// whether an AI vehicle's box overlaps one. Pure math, no game access: the sim (tools/sim/sim_blindspot.cpp) runs exactly this.
//
// Our frame (core/trafficmath.h): x = m ahead of the truck's placement origin, y = m to its left. A heading is in degrees, + = turned left, so a
// vehicle's forward in our frame is (cos h, sin h) and its left (-sin h, cos h). The trailer sits at (t_dx, t_dy) turned by t_hdg (its own origin).
#pragma once
#include <math.h>

struct bs_poly { int n; double x[4], y[4]; };   // a convex polygon (a rectangle, or a triangle)

struct bs_body {
    double tf, tb, hw;          // truck: front extent (m ahead of the origin), rear extent (m behind), half width
    int trailer;                // 1 = a trailer hangs on it
    double t_dx, t_dy, t_hdg;   // the trailer's origin in our frame, and its articulation (degrees, + = turned left)
    double lf, lb, lhw;         // trailer, in ITS frame: front extent (m ahead of its origin), rear extent (m behind), half width
};

struct bs_cfg {
    double inner, outer;        // the band: from `inner` to `outer` metres out of the body's side
    double front_margin;        // the zone starts this far ahead of the cab's front
    double rear_margin;         // and ends this far behind the rear (the trailer's, or the truck's without one)
    double min_kmh;             // only while we drive faster than this
    double hold_s;              // an occupied side stays on this long after the last overlap
    double hdg_max;             // a vehicle moving our way: |relative heading| below this (degrees)
    double v_min;               // and not standing still while we move: its speed (m/s) above this
};

static inline bs_cfg bs_cfg_default()
{
    bs_cfg c = { 0.4, 4.5, 1.0, 2.0, 15.0, 0.5, 45.0, 0.8 };
    return c;
}

static inline void bs_rot(double h_deg, double x, double y, double *ox, double *oy)
{
    double h = h_deg * 0.017453292519943295, c = cos(h), s = sin(h);
    *ox = x * c - y * s;
    *oy = x * s + y * c;
}

// A rectangle x0..x1 (ahead), y0..y1 (left) in a frame at (ox, oy) turned by h_deg, as a polygon in our frame.
static inline bs_poly bs_rect(double ox, double oy, double h_deg, double x0, double x1, double y0, double y1)
{
    bs_poly p;
    const double xs[4] = { x0, x1, x1, x0 }, ys[4] = { y0, y0, y1, y1 };
    p.n = 4;
    for (int i = 0; i < 4; i++) { double a, b; bs_rot(h_deg, xs[i], ys[i], &a, &b); p.x[i] = ox + a; p.y[i] = oy + b; }
    return p;
}

// A vehicle's box: centre (dx, dy) in our frame, length along its heading, width across it.
static inline bs_poly bs_vehicle(double dx, double dy, double len, double wid, double hdg_deg)
{
    return bs_rect(dx, dy, hdg_deg, -len * 0.5, len * 0.5, -wid * 0.5, wid * 0.5);
}

// Two convex polygons overlap (separating axis test on the edge normals; touching counts as no overlap).
static inline int bs_overlap(const bs_poly &a, const bs_poly &b)
{
    for (int k = 0; k < 2; k++) {
        const bs_poly &p = k ? b : a;
        for (int i = 0; i < p.n; i++) {
            int j = (i + 1) % p.n;
            double nx = p.y[j] - p.y[i], ny = p.x[i] - p.x[j];   // an edge normal
            if (fabs(nx) + fabs(ny) < 1e-12) continue;
            double a0 = 1e30, a1 = -1e30, b0 = 1e30, b1 = -1e30;
            for (int m = 0; m < a.n; m++) { double d = a.x[m] * nx + a.y[m] * ny; if (d < a0) a0 = d; if (d > a1) a1 = d; }
            for (int m = 0; m < b.n; m++) { double d = b.x[m] * nx + b.y[m] * ny; if (d < b0) b0 = d; if (d > b1) b1 = d; }
            if (a1 <= b0 || b1 <= a0) return 0;
        }
    }
    return 1;
}

// The pieces of one side's zone (side +1 = left, -1 = right): the truck's strip, the trailer's strip (turned by the articulation) and two
// triangles that fill the wedge between them on the outside of a bend. Returns how many (1 without a trailer, else 4). Max 4.
static inline int bs_zone(const bs_body &b, const bs_cfg &c, int side, bs_poly *out)
{
    double sy0 = side * (b.hw + c.inner), sy1 = side * (b.hw + c.outer);
    double rear = b.trailer ? b.tb : b.tb + c.rear_margin;
    out[0] = bs_rect(0, 0, 0, -rear, b.tf + c.front_margin, sy0 < sy1 ? sy0 : sy1, sy0 < sy1 ? sy1 : sy0);
    if (!b.trailer) return 1;
    double ty0 = side * (b.lhw + c.inner), ty1 = side * (b.lhw + c.outer);
    out[1] = bs_rect(b.t_dx, b.t_dy, b.t_hdg, -b.lb - c.rear_margin, b.lf, ty0 < ty1 ? ty0 : ty1, ty0 < ty1 ? ty1 : ty0);
    // the truck's rear edge (A inner, B outer) and the trailer's front edge (C outer, D inner), both in our frame
    double cx, cy, dx, dy;
    bs_rot(b.t_hdg, b.lf, ty1, &cx, &cy); cx += b.t_dx; cy += b.t_dy;
    bs_rot(b.t_hdg, b.lf, ty0, &dx, &dy); dx += b.t_dx; dy += b.t_dy;
    bs_poly t1 = { 3, { -b.tb, -b.tb, cx, 0 }, { sy0, sy1, cy, 0 } };
    bs_poly t2 = { 3, { -b.tb, cx, dx, 0 }, { sy0, cy, dy, 0 } };
    out[2] = t1; out[3] = t2;
    return 4;
}

// Does a vehicle count: moving roughly our way, not standing still while we move. `v` is its speed (m/s along its heading, NAN = unknown; for
// a trailer pass the speed of the vehicle it hangs on). Unknown = does not count (a parked car must never raise it).
static inline int bs_eligible(const bs_cfg &c, double hdg_deg, double v, double ego_mps)
{
    if (!(hdg_deg == hdg_deg) || fabs(hdg_deg) >= c.hdg_max) return 0;
    if (!(v == v)) return 0;
    if (ego_mps > 1.0 && v < c.v_min) return 0;
    return 1;
}

// Hold: an occupied side stays on for hold_s after the last raw overlap. Returns 1 while on.
static inline int bs_hold(double *until, int raw, double now, double hold_s)
{
    if (raw) *until = now + hold_s;
    return now < *until;
}
