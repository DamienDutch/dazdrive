// Offline test of the traffic probe's geometry (core/trafficmath.h): where another vehicle sits in OUR frame. The memory walk itself
// (modules/traffic.cpp) needs the running game and is checked on a ride from traffic.csv.
// Asserts: forward and left from the SDK heading, the sector coordinate, a placement quaternion against the same heading, the
// relative heading, closing speeds along and across, the middle of a placement's box, and garbage quaternions giving "no heading".
// Build+run: tools/sim/run_traffic.sh [-v]
#include "core/trafficmath.h"
#include <stdio.h>
#include <string.h>

static int fails, checks, verbose;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL line %d: %s  ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static bool near_(double a, double b, double tol = 1e-6) { return fabs(a - b) <= tol; }

// the game's quaternion for a yaw of `turns` counter-clockwise seen from above (rotation about +y), plus an optional pitch about x
static void yaw_quat(double turns, double pitch_turns, double q[4])
{
    double hy = turns * 3.141592653589793, hp = pitch_turns * 3.141592653589793;   // half angles
    double wy = cos(hy), yy = sin(hy), wp = cos(hp), xp = sin(hp);
    // q = qyaw * qpitch
    q[0] = wy * wp; q[1] = wy * xp; q[2] = yy * wp; q[3] = -yy * xp;
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    double dx, dy;

    // 1. north: ahead is -z, left is -x
    tr_pose n = tr_pose_from_heading(1000, 2000, 0.0);
    tr_rel(n, 1000, 1900, &dx, &dy);  CHECK(near_(dx, 100) && near_(dy, 0), "ahead %.3f %.3f", dx, dy);
    tr_rel(n, 990, 2000, &dx, &dy);   CHECK(near_(dx, 0) && near_(dy, 10), "left %.3f %.3f", dx, dy);
    tr_rel(n, 1005, 2020, &dx, &dy);  CHECK(near_(dx, -20) && near_(dy, -5), "behind right %.3f %.3f", dx, dy);
    // 2. west (0.25): ahead is -x, left is +z (south)
    tr_pose w = tr_pose_from_heading(1000, 2000, 0.25);
    tr_rel(w, 900, 2000, &dx, &dy);   CHECK(near_(dx, 100) && near_(dy, 0), "west ahead %.3f %.3f", dx, dy);
    tr_rel(w, 1000, 2010, &dx, &dy);  CHECK(near_(dx, 0) && near_(dy, 10), "west left %.3f %.3f", dx, dy);
    // south (0.5) and east (0.75)
    tr_pose s = tr_pose_from_heading(0, 0, 0.5);
    tr_rel(s, 0, 50, &dx, &dy);       CHECK(near_(dx, 50) && near_(dy, 0), "south ahead");
    tr_rel(s, 5, 0, &dx, &dy);        CHECK(near_(dx, 0) && near_(dy, 5), "south left is east");
    tr_pose e = tr_pose_from_heading(0, 0, 0.75);
    tr_rel(e, 30, 0, &dx, &dy);       CHECK(near_(dx, 30) && near_(dy, 0), "east ahead");
    tr_rel(e, 0, -4, &dx, &dy);       CHECK(near_(dx, 0) && near_(dy, 4), "east left is north");

    // 3. sector coordinates
    CHECK(near_(tr_world(3, 100.5f), 1636.5), "sector");
    CHECK(near_(tr_world(-2, 10.0f), -1014.0), "negative sector");

    // 4. the quaternion gives the same forward as the heading, also with some pitch; the heading is in turns, counter-clockwise
    for (int i = 0; i < 40; i++) {
        double turns = i / 40.0, q[4];
        yaw_quat(turns, i % 3 ? 0.03 : 0.0, q);
        tr_pose a = tr_pose_from_heading(5, 6, turns), b = tr_pose_from_quat(5, 6, q[0], q[1], q[2], q[3]);
        CHECK(b.ok, "quat ok at %.3f", turns);
        CHECK(near_(a.fx, b.fx, 1e-9) && near_(a.fz, b.fz, 1e-9), "turns %.3f: heading (%.4f %.4f) quat (%.4f %.4f)", turns, a.fx, a.fz, b.fx, b.fz);
    }

    // 5. garbage placements give no heading
    tr_pose g = tr_pose_from_quat(0, 0, 0, 0, 0, 0);      CHECK(!g.ok, "zero quaternion");
    g = tr_pose_from_quat(0, 0, 5, 5, 5, 5);               CHECK(!g.ok, "huge quaternion");
    g = tr_pose_from_quat(0, 0, NAN, 0, 0, 0);             CHECK(!g.ok, "NaN quaternion");
    double up[4]; yaw_quat(0.0, 0.25, up);                  // 90 degrees of pitch: the forward points straight up/down
    g = tr_pose_from_quat(0, 0, up[0], up[1], up[2], up[3]); CHECK(!g.ok, "pointing up");

    // 6. relative heading: + = turned to our left
    tr_pose me = tr_pose_from_heading(0, 0, 0.1);
    CHECK(near_(tr_rel_heading_deg(me, tr_pose_from_heading(0, 0, 0.1)), 0, 1e-6), "same");
    CHECK(near_(tr_rel_heading_deg(me, tr_pose_from_heading(0, 0, 0.15)), 18, 1e-6), "turned left 18");
    CHECK(near_(tr_rel_heading_deg(me, tr_pose_from_heading(0, 0, 0.05)), -18, 1e-6), "turned right 18");
    CHECK(near_(fabs(tr_rel_heading_deg(me, tr_pose_from_heading(0, 0, 0.6))), 180, 1e-6), "oncoming");
    CHECK(tr_rel_heading_deg(me, g) != tr_rel_heading_deg(me, g), "unknown heading is NAN");

    // 7. closing speeds in our frame (north, 25 m/s)
    double rl, rs;
    tr_pose lead = tr_pose_from_heading(0, -80, 0.0);
    CHECK(tr_rel_speed(n, 25, lead, 20, &rl, &rs) && near_(rl, -5) && near_(rs, 0), "lead slower %.2f %.2f", rl, rs);
    tr_pose onc = tr_pose_from_heading(0, -80, 0.5);
    CHECK(tr_rel_speed(n, 25, onc, 20, &rl, &rs) && near_(rl, -45) && near_(rs, 0, 1e-9), "oncoming %.2f %.2f", rl, rs);
    tr_pose cross = tr_pose_from_heading(0, -80, 0.25);   // heading west = moving to our left
    CHECK(tr_rel_speed(n, 0, cross, 10, &rl, &rs) && near_(rl, 0, 1e-9) && near_(rs, 10), "crossing to the left %.2f %.2f", rl, rs);
    tr_pose crossr = tr_pose_from_heading(0, -80, 0.75);
    CHECK(tr_rel_speed(n, 0, crossr, 10, &rl, &rs) && near_(rs, -10), "crossing to the right");
    tr_pose behind = tr_pose_from_heading(0, 30, 0.0);
    CHECK(tr_rel_speed(n, 25, behind, 30, &rl, &rs) && near_(rl, 5), "faster car behind closes: +5");
    CHECK(!tr_rel_speed(n, 25, g, 20, &rl, &rs), "no heading, no speed");
    CHECK(tr_rel_speed(n, 25, lead, -3, &rl, &rs) && near_(rl, -28), "reversing lead");

    // 8. the middle of a box: origin + rotation * mid. Box x -1..2 (middle 0.5 to the right), z -4.5..0.5 (middle 2 ahead)
    double cx, cz;
    tr_pose o = tr_pose_from_heading(100, 100, 0.0);
    tr_box_centre(o, 0.5, -2.0, &cx, &cz);  CHECK(near_(cx, 100.5) && near_(cz, 98), "north box %.3f %.3f", cx, cz);
    o = tr_pose_from_heading(100, 100, 0.25);
    tr_box_centre(o, 0.5, -2.0, &cx, &cz);  CHECK(near_(cx, 98) && near_(cz, 99.5), "west box %.3f %.3f", cx, cz);
    o = tr_pose_from_heading(0, 0, 0.1);
    tr_box_centre(o, 0.5, -2.0, &cx, &cz);
    tr_rel(o, cx, cz, &dx, &dy);            CHECK(near_(dx, 2) && near_(dy, -0.5), "box middle in its own frame %.3f %.3f", dx, dy);

    // 9. the whole chain: a car 40 m ahead in the next lane, 3 m to the left, slower, with a pose from a sector placement
    double q[4]; yaw_quat(0.0, 0.0, q);
    tr_pose truck = tr_pose_from_heading(tr_world(10, 100), tr_world(-4, 200), 0.0);
    tr_pose car = tr_pose_from_quat(tr_world(10, 97), tr_world(-4, 160), q[0], q[1], q[2], q[3]);
    tr_rel(truck, car.x, car.z, &dx, &dy);
    CHECK(near_(dx, 40) && near_(dy, 3), "chain %.3f %.3f", dx, dy);
    CHECK(tr_rel_speed(truck, 22, car, 17, &rl, &rs) && near_(rl, -5), "chain speed");
    if (verbose) printf("dx %.2f dy %.2f rl %.2f\n", dx, dy, rl);

    printf("%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
