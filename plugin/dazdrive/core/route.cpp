// Route look-ahead from the navigation route in memory. Layouts from
// ets2la_plugin (gitlab.com/ETS2LA/ets2la_plugin, MIT), verified on 1.60 in
// probe run 2 (docs/research.md § Probe run 2):
//   gps_manager = game_ctrl + G.gps_off; route_task* at +0x28
//   route_task: physical items data +0x58, count +0x60 (stride 0x20: node* +0, dist_end +0x14)
//               logical items  data +0x80, count +0x88 (stride 0x30: type +0 (1 = prefab), dist_end +0x24)
//   node_item: int3 coords +0 = meters x 256 (x, y = height, z)
// GPS distances are game-scaled (~19x outside cities), so real distances come
// from the node geometry.
#include "route.h"
#include "game.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

enum { STEP = 25, N_SAMPLES = 64 };  // heights every 25 m, up to 1575 m ahead

static struct R_t {
    int ok, n;              // n = valid samples
    float h[N_SAMPLES];     // height (m) at i*STEP ahead
    float x[N_SAMPLES], z[N_SAMPLES];  // position at i*STEP ahead (for bends)
    char herm[N_SAMPLES];   // the sample lies on a Hermite curve through the node directions (else on a straight line between nodes)
    float junction_m;
    // Junctions ahead whose turn / roundabout limits the speed (see build()).
    struct Jn { float d_in, d_out, turn, radius, ratio; int curves, round; } jn[4];
    int njn;
    double at;
    char *task;
    unsigned long long k;   // last nearest node index
    int lost;
    const char *pitems;     // physical items / count / segment of the last build (for route_probe_limit)
    unsigned long long pn, pseg;
    const char *why;        // why the last build failed (for the log)
} R;

typedef unsigned long long u64;

// Junction look-ahead settings (cruise.cpp sets them from its ini).
static struct { int on; float lat, rb_ms, min_deg, ratio; int herm; } JC = { 0, 2.0f, 25 / 3.6f, 25, 1.4f, 1 };
void route_hermite(int on) { JC.herm = on; }
void route_junction_cfg(int on, float lat, float roundabout_kmh, float min_deg, float roundabout_ratio)
{
    JC.on = on; JC.lat = lat > 0.3f ? lat : 0.3f; JC.rb_ms = roundabout_kmh / 3.6f;
    JC.min_deg = min_deg; JC.ratio = roundabout_ratio;
}

static int node_pos(const char *items, u64 i, float p[3])
{
    const char *node = *(char *const *)(items + i * 0x20);
    if (!node || !mem_readable(node, 12)) return 0;
    int c[3];
    memcpy(c, node, 12);
    p[0] = c[0] / 256.0f;
    p[1] = c[1] / 256.0f;
    p[2] = c[2] / 256.0f;
    return 1;
}

// The road direction at a node (xz, unit): the node's quaternion (w, x, y, z at +0x10) rotates the forward axis (0, 0, -1); a tangent that
// points backwards is turned round (the node has no travel direction). 0 when it cannot be read. (Offline on a real run: this matches the
// driven heading at the nodes to 2 degrees, median, runs/run2.)
static int node_tan(const char *items, u64 i, float t[2])
{
    const char *node = *(char *const *)(items + i * 0x20);
    if (!node || !mem_readable(node + 0x10, 16)) return 0;
    float q[4];
    memcpy(q, node + 0x10, 16);
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float n2 = w * w + x * x + y * y + z * z;
    if (!(n2 > 0.5f && n2 < 1.5f)) return 0;
    float vx = 0, vy = 0, vz = -1;
    float tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
    float fx = vx + w * tx + (y * tz - z * ty), fz = vz + w * tz + (x * ty - y * tx);
    float m = sqrtf(fx * fx + fz * fz);
    if (m < 1e-3f) return 0;
    t[0] = fx / m; t[1] = fz / m;
    return 1;
}

// Tangents for the segment pa -> pb (chord length len) from the directions of its two nodes: turned along the chord and refused when one is
// more than 80 degrees off it (a prefab / junction node, a U-turn: then the straight chord stays, as before).
static int seg_tangents(const char *items, u64 i, const float pa[3], const float pb[3], float len, float ta[2], float tb[2])
{
    if (!node_tan(items, i, ta) || !node_tan(items, i + 1, tb)) return 0;
    float cx = (pb[0] - pa[0]) / len, cz = (pb[2] - pa[2]) / len;
    float *tt[2] = { ta, tb };
    for (int k = 0; k < 2; k++) {
        float d = tt[k][0] * cx + tt[k][1] * cz;
        if (d < 0) { tt[k][0] = -tt[k][0]; tt[k][1] = -tt[k][1]; d = -d; }
        if (d < 0.1736f) return 0;   // cos 80 degrees
    }
    return 1;
}

static float dist2_xz(const float a[3], float x, float z)
{
    float dx = a[0] - x, dz = a[2] - z;
    return dx * dx + dz * dz;
}

// Junctions on the route ahead. The logical items interleave road nodes (type 0,
// item* = the same node_item as the physical item) and prefab entries (type 1).
// A route through a prefab is one or more consecutive type-1 items with the same
// item*, one per navigation curve (node_id = curve id, [I]); the type-0 item
// before the group is the entry node, the one after it the exit node. Turn =
// heading change road-before-entry -> road-after-exit; equivalent radius from the
// entry-exit chord (circular arc: R = chord / 2 sin(turn/2)). Roundabout [I, no flag
// found]: >= 2 curves and the GPS length through the prefab >= JC.ratio x chord
// (local GPS scale taken from the plain road around us).
static void find_junctions(char *task, const char *items, u64 n, u64 seg, float len, float t)
{
    enum { NC = 96 };
    char *litems = *(char **)(task + 0x80);
    u64 ln = *(u64 *)(task + 0x88);
    if (!litems || !ln || ln > 200000 || !mem_readable(litems, ln * 0x30)) return;
    // Distance from the truck to each node ahead (cum[m] = node seg+1+m).
    float cum[NC], p0[3], p1[3];
    int nc = 0;
    cum[nc++] = len - t;
    for (u64 j = seg + 1; nc < NC && j + 1 < n && cum[nc - 1] < 1700; j++) {
        if (!node_pos(items, j, p0) || !node_pos(items, j + 1, p1)) break;
        cum[nc] = cum[nc - 1] + sqrtf(dist2_xz(p0, p1[0], p1[2]));
        nc++;
    }
    const char *np = *(char *const *)(items + (seg + 1) * 0x20);
    u64 lq = ln;
    for (u64 q = 0; q < ln; q++) {
        unsigned ty;
        const char *ip;
        memcpy(&ty, litems + q * 0x30, 4);
        memcpy(&ip, litems + q * 0x30 + 8, 8);
        if (ty == 0 && ip == np) { lq = q; break; }
    }
    if (lq >= ln) return;
    u64 pi = seg + 1;  // physical index of the last type-0 item seen
    for (u64 q = lq + 1; q < ln && R.njn < 4; q++) {
        const char *it = litems + q * 0x30;
        unsigned ty;
        const char *ip;
        memcpy(&ty, it, 4);
        memcpy(&ip, it + 8, 8);
        if (ty == 0) {
            pi++;
            if (pi >= seg + 1 + (u64)nc || pi >= n || *(char *const *)(items + pi * 0x20) != ip) return;  // out of step / far enough
            continue;
        }
        if (ty != 1) continue;
        u64 r = q;
        while (r + 1 < ln) {
            unsigned t2;
            const char *i2;
            memcpy(&t2, litems + (r + 1) * 0x30, 4);
            memcpy(&i2, litems + (r + 1) * 0x30 + 8, 8);
            if (t2 != 1 || i2 != ip) break;
            r++;
        }
        unsigned te;
        const char *ie;
        if (r + 1 >= ln) return;
        memcpy(&te, litems + (r + 1) * 0x30, 4);
        memcpy(&ie, litems + (r + 1) * 0x30 + 8, 8);
        u64 pe = pi, px = pi + 1;
        if (te != 0 || px + 1 >= n || px >= seg + 1 + (u64)nc || pe < 1 || *(char *const *)(items + px * 0x20) != ie) {
            // a prefab on the route that cannot be read as a junction (Damien 25-10, a roundabout at 46 m the plugin did not know): say why, once per prefab
            static const char *rej[40]; static int nrej; int kn = 0;
            for (int k = 0; k < nrej; k++) kn |= rej[k] == ip;
            if (!kn && nrej < 40) {
                rej[nrej++] = ip;
                logline("prefab not read as a junction: %.0f m ahead, %d items before its exit, exit type %u, %s%s%s%s", cum[pe > seg ? pe - (seg + 1) : 0], (int)(r - q + 1), te,
                        px + 1 >= n ? "route ends " : "", px >= seg + 1 + (u64)nc ? "exit beyond the horizon " : "", pe < 1 ? "no entry node " : "",
                        te == 0 && px < n && *(char *const *)(items + px * 0x20) != ie ? "exit item mismatch" : "");
            }
            return;
        }
        float a[3], b[3], c[3], d[3];
        if (!node_pos(items, pe - 1, a) || !node_pos(items, pe, b) || !node_pos(items, px, c) || !node_pos(items, px + 1, d)) return;
        float h0 = atan2f(b[0] - a[0], b[2] - a[2]), h1 = atan2f(d[0] - c[0], d[2] - c[2]);
        float turn = h1 - h0;
        while (turn > 3.14159265f) turn -= 6.2831853f;
        while (turn < -3.14159265f) turn += 6.2831853f;
        float chord = sqrtf(dist2_xz(b, c[0], c[2]));
        float ge, gx;
        memcpy(&ge, items + pe * 0x20 + 0x14, 4);
        memcpy(&gx, items + px * 0x20 + 0x14, 4);
        // Local GPS scale from the plain segments right before the entry and after the exit (the scale differs
        // between city (~3x) and country (~19x): a global median gave ratios of 0.17 in cities, ts4k). Needs
        // >= 3 agreeing samples (+-30% of the median); else the ratio is unknown (1) and only curves count.
        float ls[8], lscale = 0;
        int nl = 0;
        for (int side = 0; side < 2; side++)
            for (int k = 1; k <= 4; k++) {
                u64 ix = side ? px + k : pe - k + 1;  // segment (ix-1, ix): before the entry / after the exit
                if (ix < 1 || ix >= n || ix >= seg + 1 + (u64)nc) continue;
                float u[3], w[3], dp;
                if (!node_pos(items, ix - 1, u) || !node_pos(items, ix, w)) continue;
                float g = sqrtf(dist2_xz(u, w[0], w[2]));
                memcpy(&dp, items + ix * 0x20 + 0xc, 4);
                if (g > 6 && dp > 0 && nl < 8) ls[nl++] = dp / g;
            }
        float ratio = 1;
        if (nl >= 3) {
            for (int a2 = 1; a2 < nl; a2++)
                for (int b2 = a2; b2 > 0 && ls[b2] < ls[b2 - 1]; b2--) { float x = ls[b2]; ls[b2] = ls[b2 - 1]; ls[b2 - 1] = x; }
            float med = ls[nl / 2];
            int agree = 0;
            for (int k = 0; k < nl; k++) agree += fabsf(ls[k] - med) < 0.3f * med;
            if (agree >= 3) lscale = med;
        }
        if (lscale < 0.5f) lscale = 0;
        if (lscale > 0 && chord > 3) {
            ratio = (ge - gx) / (chord * lscale);
            if (ratio < 1) ratio = 1;  // a path is never shorter than its chord: the rest is scale noise
        }
        R_t::Jn &j = R.jn[R.njn++];
        j.d_in = cum[pe - (seg + 1)];
        j.d_out = cum[px - (seg + 1)];
        j.turn = turn;
        float st = sinf(fabsf(turn) * 0.5f);
        j.radius = st > 0.02f ? chord / (2 * st) : 1e4f;
        if (j.radius < 8) j.radius = 8;
        j.ratio = ratio;
        j.curves = (int)(r - q + 1);
        // Roundabout [I]: 3+ navigation curves through one prefab (entry, ring parts, exit), or 2 curves with a longer
        // path (>= 1.2 x chord), or any prefab whose path is >= JC.ratio x chord and not tiny.
        j.round = j.curves >= 3 || (j.curves >= 2 && ratio >= 1.2f) || (ratio >= JC.ratio && chord >= 15);
        // First sight of a prefab: log its head words so a roundabout flag can be found later.
        static const char *seen[40];
        static int nseen;
        int known = 0;
        for (int k = 0; k < nseen; k++) known |= seen[k] == ip;
        if (!known && nseen < 40 && mem_readable(ip, 0x80)) {
            seen[nseen++] = ip;
            u64 w[12];
            memcpy(w, ip, sizeof w);
            logline("junction ahead %.0f m: %d curve(s), turn %.0f deg, chord %.0f m, path x%.2f (scale %.1f, %d samples)%s; prefab item %p: "
                    "%llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx",
                    j.d_in, j.curves, turn * 57.2958f, chord, ratio, lscale, nl, j.round ? " ROUNDABOUT" : "", (const void *)ip,
                    w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], w[9], w[10], w[11]);
        }
        q = r;
        pi = px - 1;  // the exit node (type 0) is counted by the loop
    }
}

static void build(char *task)
{
    char *items = *(char **)(task + 0x58);
    u64 n = *(u64 *)(task + 0x60);
    R.why = "no route";
    if (!items || n < 2 || n > 200000 || !mem_readable(items, n * 0x20)) return;
    float tx = (float)T.place.position.x, tz = (float)T.place.position.z;

    // Nearest node: around the last one, else (new route, lost) all of them.
    u64 lo = 0, hi = n;
    if (task == R.task && !R.lost) {
        lo = R.k > 30 ? R.k - 30 : 0;
        hi = R.k + 300 < n ? R.k + 300 : n;
    }
    u64 best = lo;
    float bd = 1e30f, p[3];
    for (u64 i = lo; i < hi; i++)
        if (node_pos(items, i, p)) {
            float d = dist2_xz(p, tx, tz);
            if (d < bd) { bd = d; best = i; }
        }
    R.task = task;
    R.k = best;
    // Far from every node = off the route. Nodes can be up to ~300 m apart on a highway (probe run 2: 15 % of the
    // gaps > 160 m), so near one node is not the test: the distance to the segment is checked below.
    R.lost = bd > 400.0f * 400.0f;
    R.why = "off the route";
    if (R.lost) return;

    // Segment the truck is on: (best-1, best) or (best, best+1), by projection.
    float a[3], b[3];
    u64 seg = best + 1 < n ? best : best - 1;
    if (best > 0 && best + 1 < n) {
        float pa[3], pb[3];
        if (node_pos(items, best - 1, pa) && node_pos(items, best, pb)) {
            float dx = pb[0] - pa[0], dz = pb[2] - pa[2];
            float t = ((tx - pa[0]) * dx + (tz - pa[2]) * dz) / (dx * dx + dz * dz + 1e-6f);
            if (t < 1.0f) seg = best - 1;
        }
    }
    R.why = "route node unreadable";
    if (!node_pos(items, seg, a) || !node_pos(items, seg + 1, b)) return;
    float dx = b[0] - a[0], dz = b[2] - a[2], len = sqrtf(dx * dx + dz * dz);
    R.why = "zero-length route segment";
    if (len < 0.1f) return;

    float t = ((tx - a[0]) * dx + (tz - a[2]) * dz) / len;  // truck's position along the segment
    t = t < 0 ? 0 : (t > len ? len : t);
    // Off the route: more than 80 m from the segment (was: from the nearest node, which turned the look-ahead
    // off for 1-7 s in the middle of every long highway segment, ts5a/ts5c: 24-30 times a ride).
    float ox = a[0] + dx / len * t - tx, oz = a[2] + dz / len * t - tz;
    R.why = "off the route";
    if (ox * ox + oz * oz > 80.0f * 80.0f) { R.lost = 1; return; }

    // Driving along the route? (SDK heading: 0 = north = -z, counter-clockwise.)
    float hd = (float)T.place.orientation.heading * 6.2831853f;
    float fx = -sinf(hd), fz = -cosf(hd);
    R.why = "heading off the route";
    if ((fx * dx + fz * dz) / len < 0.3f) return;

    R.pitems = items; R.pn = n; R.pseg = seg;
    R.njn = 0;
    if (JC.on) find_junctions(task, items, n, seg, len, t);
    // Walk the polyline from the projection, sampling the height every STEP m.
    float s0 = -t;  // distance from the truck to the start of the current segment
    float seg_len = len;
    u64 j = seg;
    float pa[3] = { a[0], a[1], a[2] }, pb[3] = { b[0], b[1], b[2] };
    float ta[2] = { 0, 0 }, tb[2] = { 0, 0 };
    int herm = JC.herm && seg_tangents(items, seg, pa, pb, seg_len, ta, tb);
    int i = 0;
    float junction_m = -1;
    float end_trk = NAN, end_j = NAN;
    memcpy(&end_trk, items + seg * 0x20 + 0x14, 4);  // GPS distance till the end at the segment start
    char *litems = *(char **)(task + 0x80);
    u64 ln = *(u64 *)(task + 0x88);
    // Next prefab (junction) by GPS distance till the end, mapped to geometry below.
    float next_prefab = NAN;
    if (litems && ln && ln < 200000 && mem_readable(litems, ln * 0x30)) {
        for (u64 q = 0; q < ln; q++) {
            const char *it = litems + q * 0x30;
            unsigned type;
            float de;
            memcpy(&type, it, 4);
            memcpy(&de, it + 0x24, 4);
            if (type == 1 && de < end_trk && (next_prefab != next_prefab || de > next_prefab)) next_prefab = de;
        }
    }
    while (i < N_SAMPLES) {
        float s = (float)(i * STEP);
        if (s <= s0 + seg_len) {
            float k = (s - s0) / seg_len;
            if (herm) {   // cubic Hermite through the two nodes with their directions (the chord length as the speed): the curve of the road, not the diagonal
                float k2 = k * k, k3 = k2 * k, h00 = 2 * k3 - 3 * k2 + 1, h10 = k3 - 2 * k2 + k, h01 = -2 * k3 + 3 * k2, h11 = k3 - k2;
                R.x[i] = h00 * pa[0] + h10 * seg_len * ta[0] + h01 * pb[0] + h11 * seg_len * tb[0];
                R.z[i] = h00 * pa[2] + h10 * seg_len * ta[1] + h01 * pb[2] + h11 * seg_len * tb[1];
            } else {
                R.x[i] = pa[0] + (pb[0] - pa[0]) * k;
                R.z[i] = pa[2] + (pb[2] - pa[2]) * k;
            }
            R.herm[i] = (char)herm;
            R.h[i++] = pa[1] + (pb[1] - pa[1]) * k;
            continue;
        }
        // next segment
        s0 += seg_len;
        j++;
        if (j + 1 >= n) break;
        memcpy(pa, pb, sizeof pa);
        if (!node_pos(items, j + 1, pb)) break;
        float ex = pb[0] - pa[0], ez = pb[2] - pa[2];
        seg_len = sqrtf(ex * ex + ez * ez);
        if (seg_len < 0.1f) seg_len = 0.1f;
        herm = JC.herm && seg_tangents(items, j, pa, pb, seg_len, ta, tb);
        memcpy(&end_j, items + j * 0x20 + 0x14, 4);
        if (junction_m < 0 && next_prefab == next_prefab && end_j <= next_prefab) junction_m = s0;
    }
    R.n = i;
    R.junction_m = junction_m;
    R.ok = i >= 4;
    R.why = "route ends here";
}

void route_reset()
{
    memset(&R, 0, sizeof R);
}

void route_update()
{
    double now = wall_s();
    if (now - R.at < 0.25 && now >= R.at) return;
    R.at = now;
    R.ok = 0;
    if (!G.gps_off) return;
    char *gc = game_ctrl();
    if (!gc) return;
    char *gps = gc + G.gps_off;
    if (!mem_readable(gps, 0x30)) return;
    char *task = *(char **)(gps + 0x28);
    if (!task || !mem_readable(task, 0x90)) { R.task = NULL; return; }
    build(task);
    static int logged = -1;
    if (R.ok != logged) {
        logged = R.ok;
        if (R.ok) logline("route look-ahead on (%.0f m ahead)", (float)((R.n - 1) * STEP));
        else logline("route look-ahead off (%s)", R.why ? R.why : "no route");
    }
}

int route_ok() { return R.ok; }
float route_ahead_m() { return R.ok ? (float)((R.n - 1) * STEP) : 0; }

static float h_at(float d)
{
    float x = d / STEP;
    int i = (int)x;
    if (i >= R.n - 1) return R.h[R.n - 1];
    return R.h[i] + (R.h[i + 1] - R.h[i]) * (x - i);
}

float route_grade(float d0, float d1)
{
    if (!R.ok || d1 <= d0) return NAN;
    float top = route_ahead_m();
    if (d0 >= top) return NAN;
    if (d1 > top) d1 = top;
    if (d1 - d0 < 10) return NAN;
    return (h_at(d1) - h_at(d0)) / (d1 - d0) * 100;
}

static float extreme_grade(float d0, float d1, int max)
{
    if (!R.ok) return NAN;
    float top = route_ahead_m(), best = NAN;
    for (float d = d0; d + 50 <= d1 && d + 50 <= top; d += STEP) {
        float g = (h_at(d + 50) - h_at(d)) / 50 * 100;
        if (best != best || (max ? g > best : g < best)) best = g;
    }
    return best;
}
float route_max_grade(float d0, float d1) { return extreme_grade(d0, d1, 1); }
float route_min_grade(float d0, float d1) { return extreme_grade(d0, d1, 0); }

float route_crest_m(float max_d)
{
    if (!R.ok) return -1;
    float top = fminf(route_ahead_m(), max_d);
    if ((h_at(50) - h_at(0)) / 50 * 100 < 0.5f) return -1;  // not climbing
    for (float d = STEP; d + 50 <= top; d += STEP)
        if ((h_at(d + 50) - h_at(d)) / 50 * 100 < 0.2f) return d + 25;
    return -1;
}

// A dip (PCC, Damien 11-10): the lowest point within max_d, with a climb soon after it
// (the road rises >= 3 m within 300 m of the bottom). The profile must reach past it.
float route_valley_m(float max_d)
{
    if (!R.ok) return -1;
    float top = fminf(route_ahead_m() - 300, max_d), low = h_at(0), at = 0;
    for (float d = STEP; d <= top; d += STEP)
        if (h_at(d) < low) { low = h_at(d); at = d; }
    if (at < STEP) return -1;
    for (float d = at + STEP; d <= at + 300; d += STEP)
        if (h_at(d) - low >= 3) return at;
    return -1;
}

// Speed limit ahead (docs/speedlimit-ahead-re.md): NO reliable source found statically, so this
// reports "unknown" (NAN) for now. The probe below collects what a test ride needs to settle it.
float route_limit_ahead(float max_m, float *dist_m)
{
    (void)max_m;
    if (dist_m) *dist_m = NAN;
    return NAN;
}

// Probe: at every change of the SDK speed limit, dump the route nodes around the truck (words of the
// node item and, one level down, of the things it points to) so a sign / road-look field that changes
// at the limit change can be found offline. At most 30 dumps, 8 nodes each.
static void dump_words(const char *tag, const char *p, int nw)
{
    u64 w[16];
    if (nw > 16) nw = 16;
    if (!p || !mem_readable(p, nw * 8)) return;
    memcpy(w, p, nw * 8);
    char b[16 * 18 + 1];
    int o = 0;
    for (int i = 0; i < nw; i++) o += snprintf(b + o, sizeof b - o, "%llx ", w[i]);
    logline("limit probe %s %p: %s", tag, (const void *)p, b);
}
void route_probe_limit(float limit_kmh)
{
    static int dumps;
    if (dumps >= 30 || !R.ok || !R.pitems || R.pseg + 8 >= R.pn) return;
    dumps++;
    logline("limit probe #%d: limit now %.0f km/h, node seg %llu of %llu", dumps, limit_kmh, R.pseg, R.pn);
    for (int k = -2; k < 6; k++) {
        const char *it = R.pitems + (R.pseg + k) * 0x20;
        if (!mem_readable(it, 0x20)) return;
        const char *node = *(char *const *)it;
        char tag[32];
        snprintf(tag, sizeof tag, "node[%+d]", k);
        dump_words(tag, node, 16);
        if (!node || !mem_readable(node, 0x80)) continue;
        for (int i = 2; i < 16; i++) {  // words 0/1 are coordinates
            const char *q = *(char *const *)(node + i * 8);
            if ((u64)q > 0x10000 && (u64)q < 0x00007fffffffffffULL && !((u64)q & 7) && mem_readable(q, 0x40)) {
                snprintf(tag, sizeof tag, "  [%+d].w%d->", k, i);
                dump_words(tag, q, 8);
            }
        }
    }
}

float route_junction_m() { return R.ok ? R.junction_m : -1; }
// What the junction reading holds right now, for a log line: distance to the entry, turn, curves, roundabout flag, path ratio
void route_junction_dump(char *buf, size_t n)
{
    int k = 0;
    buf[0] = 0;
    for (int i = 0; i < R.njn && k < (int)n - 60; i++)
        k += snprintf(buf + k, n - k, "%s[%.0f m, %.0f deg, %d curves, %s, x%.2f]", i ? " " : "", R.jn[i].d_in, fabsf(R.jn[i].turn) * 57.2958f, R.jn[i].curves,
                      R.jn[i].round ? "roundabout" : "junction", R.jn[i].ratio);
    if (!k) snprintf(buf, n, "none");
}

// Bends: the radius at each sample from the heading change over 50 m (three
// points 25 m apart). The speed allowed now so that, braking gently (a_dec),
// every bend ahead is taken with at most a_lat sideways. NAN if unknown.
static float route_bend_speed(float a_lat, float a_dec);
float route_junction_speed(float a_dec, route_junction_t *info)
{
    if (!R.ok || !JC.on) return NAN;
    float best = NAN;
    for (int i = 0; i < R.njn; i++) {
        const R_t::Jn &j = R.jn[i];
        float deg = fabsf(j.turn) * 57.2958f, v;
        if (j.round) v = JC.rb_ms;
        else if (deg < JC.min_deg) continue;
        else v = fmaxf(sqrtf(JC.lat * j.radius), 15 / 3.6f);
        float d = j.d_in - (j.round ? 45 : 25);             // be down to v well before the entry (Damien: roundabouts too fast)
        float v_now = sqrtf(v * v + 2 * a_dec * (d > 0 ? d : 0));
        if (best != best || v_now < best) {
            best = v_now;
            if (info) { info->dist_m = j.d_in; info->turn_deg = deg; info->limit_ms = v; info->roundabout = j.round; info->curves = j.curves; info->ratio = j.ratio; }
        }
    }
    return best;
}

float route_curve_speed(float a_lat, float a_dec, float a_dec_jn)
{
    float vj = route_junction_speed(a_dec_jn > 0 ? a_dec_jn : a_dec, NULL);
    float vb = route_bend_speed(a_lat, a_dec);
    return vj == vj && (vb != vb || vj < vb) ? vj : vb;
}

// Sideways acceleration allowed against the speed through the bend, in the shape of the permissible lateral acceleration in
// a Mercedes-Benz patent (US12139138: 5 m/s2 at 25 km/h, 4 at 51, 3.5 at 67, 3.25 at 92, 3.2 from 111 km/h), as a factor on the
// curve_lat of the ini (= the value at about 92 km/h). Capped at 4/3.25: a laden truck rolls over from about 3 m/s2 for real.
static float lat_scale(float v_ms)
{
    static const float kmh[] = { 25, 51, 67, 92, 111 }, a[] = { 5.0f, 4.0f, 3.5f, 3.25f, 3.2f };
    float k = v_ms * 3.6f, r;
    if (k <= kmh[0]) r = a[0];
    else if (k >= kmh[4]) r = a[4];
    else {
        int i = 0;
        while (k > kmh[i + 1]) i++;
        r = a[i] + (a[i + 1] - a[i]) * (k - kmh[i]) / (kmh[i + 1] - kmh[i]);
    }
    r /= 3.25f;
    return r > 4.0f / 3.25f ? 4.0f / 3.25f : r;
}

static float route_bend_speed(float a_lat, float a_dec)
{
    if (!R.ok || R.n < 5) return NAN;
    float best = NAN;
    // Chords of K steps on each side: the route nodes are ~60 m apart and the
    // samples run straight between them, so over one step every node looks
    // like a sharp corner (ts3e: bends taken far too slowly). The heading
    // change between the two chords is the one over K steps of arc.
    // On a Hermite stretch (the curve of the road, no corners at the nodes) a window of 2 steps does: a tight bend is not smoothed away by a
    // 75 m chord (offline on a real run, 45 bends: mean log error of the radius 0.54 -> 0.37, too-fast cases 11 -> 8, too-slow 15 -> 10).
    for (int i = 2; i + 2 < R.n; i++) {
        int K = 3;
        if (R.herm[i - 2] && R.herm[i - 1] && R.herm[i] && R.herm[i + 1] && R.herm[i + 2]) K = 2;
        if (i < K || i + K >= R.n) continue;
        float ax = R.x[i] - R.x[i - K], az = R.z[i] - R.z[i - K];
        float bx = R.x[i + K] - R.x[i], bz = R.z[i + K] - R.z[i];
        float la = sqrtf(ax * ax + az * az), lb = sqrtf(bx * bx + bz * bz);
        if (la < 1 || lb < 1) continue;
        float c = (ax * bx + az * bz) / (la * lb);
        c = c > 1 ? 1 : (c < -1 ? -1 : c);
        float ang = acosf(c);                           // heading change over K steps of arc
        if (ang < 0.02f) continue;                      // straight enough
        float radius = 0.5f * (la + lb) / ang;
        float v_bend = sqrtf(a_lat * radius);
        for (int it = 0; it < 3; it++) v_bend = sqrtf(a_lat * lat_scale(v_bend) * radius);  // the allowance depends on the speed it gives
        float d = (float)(i * STEP) - 1.5f * STEP;      // distance to the bend (its window starts earlier)
        float v_now = sqrtf(v_bend * v_bend + 2 * a_dec * (d > 0 ? d : 0));
        if (best != best || v_now < best) best = v_now;
    }
    return best;
}
