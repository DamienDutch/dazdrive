// Plane geometry for the traffic probe: world poses (x, z) with a forward unit vector, and where another pose sits in OUR frame.
// Pure math, no game access: the sim (tools/sim/sim_traffic.cpp) runs exactly this.
//
// World: x east, y up, z south (so north = -z). Our heading is the SDK's: 0 = north, counter-clockwise, in turns (0.25 = west).
// Forward in the plane is (-sin h, -cos h); left is (-cos h, sin h). The game's quaternion (w, x, y, z) rotates a local vector into the
// world; a vehicle's local forward is -z (same convention as the SDK's placement).
#pragma once
#include <math.h>

struct tr_pose {
    double x, z;      // world meters (sector * 512 + offset in the game's lists, the SDK's own for us)
    double fx, fz;    // forward unit vector in the plane
    int ok;           // 1 = the forward vector is meaningful
};

// A game placement's world coordinate: the sector index (512 m) and the offset inside it.
static inline double tr_world(short sector, float offset) { return sector * 512.0 + offset; }

static inline tr_pose tr_pose_from_heading(double x, double z, double heading_turns)
{
    double h = heading_turns * 6.283185307179586;
    tr_pose p = { x, z, -sin(h), -cos(h), 1 };
    return p;
}

// A game placement: the quaternion rotates local (0, 0, -1) to the world forward; its horizontal part is the heading.
// A quaternion that is not close to unit length (garbage memory) gives ok = 0.
static inline tr_pose tr_pose_from_quat(double x, double z, double qw, double qx, double qy, double qz)
{
    tr_pose p = { x, z, 0, 0, 0 };
    double n = qw * qw + qx * qx + qy * qy + qz * qz;
    if (!(n > 0.95 && n < 1.05)) return p;
    double x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
    double wy2 = qw * y2, xz2 = qx * z2, xx2 = qx * x2, yy2 = qy * y2;
    // float3_t::rotate of (0, 0, -1): third column, negated (ets2la_plugin common.cpp)
    double fx = -(xz2 + wy2), fz = -(1.0 - xx2 - yy2);
    double l = sqrt(fx * fx + fz * fz);
    if (l < 0.2) return p;   // pointing nearly straight up or down: no heading
    p.fx = fx / l; p.fz = fz / l; p.ok = 1;
    return p;
}

// Left of forward (fx, fz) = (-sin h, -cos h) is (-cos h, sin h) = (fz, -fx).
static inline void tr_left(const tr_pose &p, double *lx, double *lz) { *lx = p.fz; *lz = -p.fx; }

// Another point in our frame: dx forward (m), dy to the left (m).
static inline void tr_rel(const tr_pose &me, double x, double z, double *dx, double *dy)
{
    double wx = x - me.x, wz = z - me.z, lx, lz;
    tr_left(me, &lx, &lz);
    *dx = wx * me.fx + wz * me.fz;
    *dy = wx * lx + wz * lz;
}

// Heading of `o` relative to `me`, degrees, + = o points more to our left (counter-clockwise), -180..180; NAN if unknown.
static inline double tr_rel_heading_deg(const tr_pose &me, const tr_pose &o)
{
    if (!me.ok || !o.ok) return NAN;
    double lx, lz;
    tr_left(me, &lx, &lz);
    double a = atan2(o.fx * lx + o.fz * lz, o.fx * me.fx + o.fz * me.fz);
    return a * 57.29577951308232;
}

// Speed of `o` (m/s along its forward, signed) and ours (m/s along ours) as a closing speed in our frame:
// *rl = relative speed along our forward (+ = it pulls away ahead / closes from behind), *rs = across (+ = moving to our left).
static inline int tr_rel_speed(const tr_pose &me, double my_v, const tr_pose &o, double o_v, double *rl, double *rs)
{
    if (!me.ok || !o.ok) return 0;
    double lx, lz;
    tr_left(me, &lx, &lz);
    double vx = o.fx * o_v - me.fx * my_v, vz = o.fz * o_v - me.fz * my_v;
    *rl = vx * me.fx + vz * me.fz;
    *rs = vx * lx + vz * lz;
    return 1;
}

// A box centre from a placement origin and a local aabox (start, end): origin + rotation * mid. Planar version: the local (x, z) of the
// middle is turned by the pose (local +x is to the right of forward, local +z is behind).
static inline void tr_box_centre(const tr_pose &o, double mid_x, double mid_z, double *cx, double *cz)
{
    // right = -left; backward = -forward
    double lx, lz;
    tr_left(o, &lx, &lz);
    *cx = o.x + mid_x * (-lx) + mid_z * (-o.fx);
    *cz = o.z + mid_x * (-lz) + mid_z * (-o.fz);
}
