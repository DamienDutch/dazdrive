// Traffic: a LOG-ONLY reader of the game's AI traffic list. Nothing in the suite acts on it (no braking, no tones, no steering).
//
// Purpose: to check on a ride that the list is read right, as the foundation for a blind-spot warning (real vehicles only, by
// distance, following the trailer), a cut-in-aware ACC and a turn assist later.
//
// Where it comes from (ets2la_plugin, MIT, gitlab.com/ETS2LA/ets2la_plugin, commit c925bd9, built for ETS2 1.60; notes/research.md § 6):
//   - game_traffic_u* global: pattern src/patterns.win32.hpp game_traffic (+6, rip-relative) = pattern k_pat below.
//   - game_traffic_u (src/prism/traffic/traffic.hpp, Windows offsets): spawned_vehicles_1 +0xF0, spawned_vehicles_2 +0x118 (entries
//     of 0x10: traffic_ai_vehicle_t* then 8 bytes), traffic_objects_1 +0x1E0 (traffic_object_t*, any type), traffic_player_vehicles_1
//     +0x208 and traffic_player_trailers_1 +0x230 (pointers; the truck and trailers we drive, convoy players, trailers parked in
//     companies). array_dyn_t (src/prism/collections/array*.hpp): a vtable pointer, then data +8 and size +0x10, capacity, allocator.
//   - traffic_actor_t (src/prism/traffic/objects/traffic_actor.hpp): placement +0x28 (float3 pos in the sector, int16 sector x, int16
//     sector z, quaternion w x y z; world = sector * 512 + pos), aabox +0x48 (start, end: float3 each), slave (the next trailer of a
//     vehicle) +0x68. traffic_ai_vehicle_t (objects/traffic_ai_vehicle.hpp): physics* +0x238 (traffic_vehicle_physics_object_t at
//     +0x218, pointer at +0x20); the physics (traffic_chassis_physics_t): speed +0x70, acceleration +0x74.
//   - traffic_player_vehicle_t: speed +0x218.
//   - processing/traffic.cpp reads exactly these three lists for the AI vehicles and uses get_type() on traffic_objects_1; we do not
//     call into the game: an entry of traffic_objects_1 counts as a vehicle when its vtable is one the two spawned lists use.
//
// Safety: the layouts are ETS2 1.60 only (layout name "1.60"); every pointer goes through mem_readable, the lists are capped, an
// implausible value (NaN, |coordinate| > 1e7, a box over 60 m, a list longer than 512) drops that entry and is counted; nothing is
// ever written to the game. Read on the game's own thread (frame_end), so the lists do not change under the walk.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/trafficmath.h"
#include "../core/traffic.h"
#include <math.h>
#include <string.h>
#include <windows.h>

static float p_probe;

static param g_params[] = {
    { "probe", &p_probe, 0, "1 = read the game's list of AI traffic and only log it: traffic.csv (needs the log at its highest level; 5 per second while driving, the 8 nearest vehicles with where they are relative to you) and one summary line a minute. Nothing is braked, steered or announced from it. For checking the reading on a ride. 0 = off" },
};

// ---------------------------------------------------------------- layout (ETS2 1.60, Windows)

static const char *const k_pat = "48 8B D9 48 8B 0D ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 83 ?? ?? ?? ?? 48 85 C0";
enum {
    GT_SPAWNED1 = 0xF0, GT_SPAWNED2 = 0x118, GT_OBJECTS1 = 0x1E0, GT_PVEH = 0x208, GT_PTRL = 0x230,   // arrays inside game_traffic_u
    ARR_DATA = 0x8, ARR_SIZE = 0x10,                                                                   // inside an array_dyn_t
    ACT_PLACEMENT = 0x28, ACT_SLAVE = 0x68, ACT_END = 0x70,                                            // traffic_actor_t
    AI_PHYSICS = 0x238, PH_SPEED = 0x70, PV_SPEED = 0x218, PV_END = 0x220,
    MAX_LIST = 512, MAX_ENT = 1400, NEAR_LOG = 8, TICK_MS = 200,
};
enum { L_SP1 = 1, L_SP2 = 2, L_OBJ = 4, L_SLAVE = 8 };   // the lists an entry was found in (bit mask in the csv)
enum { K_AI = 0, K_AIT = 1 };

struct ent {
    uintptr_t p;
    double x, z;          // world metres, middle of the box
    double ox, oz;        // world metres, the placement origin
    float len, wid, speed, rv_l, rv_s, hdg, dx, dy, dist;
    int mask, kind;
    tr_pose pose;
};

static ent g_ent[MAX_ENT];
static int g_n, g_bad;
static FILE *g_csv;
static char g_status[160] = "off (ini)";
static int g_state;                    // 0 off, 1 no pattern / wrong layout, 2 waiting for the instance, 3 reading
static uintptr_t g_slot, g_img_lo, g_img_hi;
static int g_hits;
static double g_ok_at = -1;            // wall time of the last good walk
static uintptr_t g_ai_vt[4];           // vtables of the vehicles in the spawned lists
static int g_n_ai_vt;
static struct { uintptr_t vt; int n, ai; } g_ovt[8];   // vtables seen in traffic_objects_1 this tick
static int g_n_ovt;

struct geo { float hook_z, front_z, rear_z; };
static geo g_truck = { NAN, NAN, NAN }, g_trailer = { NAN, NAN, NAN };

// what the last tick found, for the dev feed and the summary
static traffic_around g_around;
static int g_cnt[8];                   // entries per list this tick: sp1, sp2, obj, obj-as-vehicle, slaves, pv, pt, dup
static int g_pv_n, g_pt_n;
static double g_pv_dx, g_pv_dy, g_pv_d = NAN, g_pt_dx, g_pt_dy, g_pt_d = NAN;   // nearest player vehicle / trailer origin relative to ours
static double g_walk_ms_sum, g_walk_ms_max;
static int g_walk_n;
static unsigned g_ticks, g_ticks_fail;

// ---------------------------------------------------------------- safe reads

static int ptr_ok(const void *p, size_t n) { return p && !((uintptr_t)p & 7) && mem_readable(p, n); }

// An array_dyn_t at base+off: the data pointer and the size, checked. 0 if it is not plausible.
static int arr_get(const char *base, unsigned off, unsigned stride, const char **data, unsigned *n)
{
    const char *a = base + off;
    if (!mem_readable(a, 0x18)) return 0;
    const char *d = *(const char *const *)(a + ARR_DATA);
    unsigned long long sz = *(const unsigned long long *)(a + ARR_SIZE);
    if (sz == 0) { *data = d; *n = 0; return 1; }
    if (sz > MAX_LIST || !ptr_ok(d, (size_t)sz * stride)) return 0;
    *data = d;
    *n = (unsigned)sz;
    return 1;
}

static int plausible(double v, double lim) { return v == v && fabs(v) <= lim; }

// One traffic_actor_t (vehicle, trailer, player vehicle): placement + box -> e. 0 if it does not look like one.
static int read_actor(const char *p, ent *e)
{
    if (!ptr_ok(p, ACT_END)) return 0;
    const char *q = p + ACT_PLACEMENT;
    float f[6];
    short cx, cz;
    memcpy(f, q, 12);
    memcpy(&cx, q + 12, 2);
    memcpy(&cz, q + 14, 2);
    float rot[4], box[6];
    memcpy(rot, q + 16, 16);
    memcpy(box, q + 0x20, 24);
    double gx = tr_world(cx, f[0]), gz = tr_world(cz, f[2]);
    if (!plausible(f[0], 4096) || !plausible(f[1], 5000) || !plausible(f[2], 4096) || !plausible(gx, 1e7) || !plausible(gz, 1e7)) return 0;
    for (int i = 0; i < 4; i++) if (!plausible(rot[i], 2)) return 0;
    for (int i = 0; i < 6; i++) if (!plausible(box[i], 60)) return 0;
    e->p = (uintptr_t)p;
    e->pose = tr_pose_from_quat(gx, gz, rot[0], rot[1], rot[2], rot[3]);
    e->len = fabsf(box[5] - box[2]);
    e->wid = fabsf(box[3] - box[0]);
    e->x = e->ox = gx; e->z = e->oz = gz;
    if (e->pose.ok) tr_box_centre(e->pose, 0.5 * (box[0] + box[3]), 0.5 * (box[2] + box[5]), &e->x, &e->z);
    e->speed = NAN;
    e->mask = 0;
    return 1;
}

static void add_ai(const char *p, int list, int kind, float speed)
{
    for (int i = 0; i < g_n; i++)
        if (g_ent[i].p == (uintptr_t)p) { g_ent[i].mask |= list; g_cnt[7]++; return; }
    if (g_n >= MAX_ENT) return;
    ent e;
    if (!read_actor(p, &e)) { g_bad++; return; }
    e.mask = list;
    e.kind = kind;
    e.speed = speed;
    g_ent[g_n++] = e;
}

static float ai_speed(const char *p)
{
    const char *ph = *(const char *const *)(p + AI_PHYSICS);
    if (!ptr_ok(ph, PH_SPEED + 8)) return NAN;
    float v = *(const float *)(ph + PH_SPEED);
    return plausible(v, 120) ? v : NAN;
}

// A vehicle and the trailers hanging on it (slave chain, at most 3, with a cycle guard).
static void add_vehicle(const char *p, int list)
{
    if (!ptr_ok(p, AI_PHYSICS + 8)) { g_bad++; return; }
    add_ai(p, list, K_AI, ai_speed(p));
    const char *s = *(const char *const *)(p + ACT_SLAVE);
    for (int i = 0; i < 3 && s && s != p; i++) {
        if (!ptr_ok(s, ACT_END)) break;
        add_ai(s, L_SLAVE, K_AIT, NAN);
        const char *nx = *(const char *const *)(s + ACT_SLAVE);
        if (nx == s) break;
        s = nx;
    }
}

static int known_ai_vt(uintptr_t vt)
{
    for (int i = 0; i < g_n_ai_vt; i++) if (g_ai_vt[i] == vt) return 1;
    return 0;
}

static void learn_ai_vt(const char *p)
{
    if (!ptr_ok(p, 8)) return;
    uintptr_t vt = *(const uintptr_t *)p;
    if (vt < g_img_lo || vt >= g_img_hi || known_ai_vt(vt) || g_n_ai_vt >= 4) return;
    g_ai_vt[g_n_ai_vt++] = vt;
}

// ---------------------------------------------------------------- the walk

static void walk(const char *gt, const tr_pose &me)
{
    g_n = g_bad = 0;
    memset(g_cnt, 0, sizeof g_cnt);
    g_n_ovt = 0;
    const char *d;
    unsigned n;

    // 1. spawned_vehicles_1 / _2: entries of 0x10 (vehicle pointer first)
    for (int k = 0; k < 2; k++) {
        if (!arr_get(gt, k ? GT_SPAWNED2 : GT_SPAWNED1, 0x10, &d, &n)) { g_bad++; continue; }
        g_cnt[k] = (int)n;
        for (unsigned i = 0; i < n; i++) {
            const char *v = *(const char *const *)(d + i * 0x10);
            learn_ai_vt(v);
            add_vehicle(v, k ? L_SP2 : L_SP1);
        }
    }
    // 2. traffic_objects_1: every kind of object; the vehicles are the ones with a vehicle vtable (no call into the game)
    if (arr_get(gt, GT_OBJECTS1, 8, &d, &n)) {
        g_cnt[2] = (int)n;
        for (unsigned i = 0; i < n; i++) {
            const char *o = *(const char *const *)(d + i * 8);
            if (!ptr_ok(o, 8)) { g_bad++; continue; }
            uintptr_t vt = *(const uintptr_t *)o;
            int ai = known_ai_vt(vt), j;
            for (j = 0; j < g_n_ovt && g_ovt[j].vt != vt; j++) { }
            if (j == g_n_ovt && g_n_ovt < 8) { g_ovt[j].vt = vt; g_ovt[j].n = 0; g_ovt[j].ai = ai; g_n_ovt++; }
            if (j < g_n_ovt) g_ovt[j].n++;
            if (!ai) continue;
            g_cnt[3]++;
            add_vehicle(o, L_OBJ);
        }
    } else g_bad++;
    for (int i = 0; i < g_n; i++) if (g_ent[i].kind == K_AIT) g_cnt[4]++;

    // our frame
    for (int i = 0; i < g_n; i++) {
        ent &e = g_ent[i];
        double dx, dy;
        tr_rel(me, e.x, e.z, &dx, &dy);
        e.dx = (float)dx; e.dy = (float)dy; e.dist = (float)sqrt(dx * dx + dy * dy);
        e.hdg = (float)tr_rel_heading_deg(me, e.pose);
        double rl, rs;
        e.rv_l = e.rv_s = NAN;
        if (e.speed == e.speed && tr_rel_speed(me, T.speed, e.pose, e.speed, &rl, &rs)) { e.rv_l = (float)rl; e.rv_s = (float)rs; }
    }

    // 3. the players' own vehicles and trailers: the one nearest to our placement. Ours are in there (the truck we drive and its trailers),
    //    so the nearest one should sit within a metre or two of our placement: that checks the coordinate frame.
    g_pv_n = g_pt_n = 0;
    g_pv_d = g_pt_d = NAN;
    for (int k = 0; k < 2; k++) {
        if (!arr_get(gt, k ? GT_PTRL : GT_PVEH, 8, &d, &n)) { g_bad++; continue; }
        if (k) g_pt_n = (int)n; else g_pv_n = (int)n;
        for (unsigned i = 0; i < n; i++) {
            ent e;
            const char *v = *(const char *const *)(d + i * 8);
            if (!read_actor(v, &e)) { g_bad++; continue; }
            double dx, dy;
            tr_rel(me, e.ox, e.oz, &dx, &dy);   // the origin, not the box middle: ours is the SDK placement origin
            double dist = sqrt(dx * dx + dy * dy);
            double &bd = k ? g_pt_d : g_pv_d;
            if (bd != bd || dist < bd) { bd = dist; (k ? g_pt_dx : g_pv_dx) = dx; (k ? g_pt_dy : g_pv_dy) = dy; }
        }
    }
}

// ---------------------------------------------------------------- per tick

static void pick_nearest(ent **out, int *m)
{
    *m = 0;
    static unsigned char used[MAX_ENT];
    memset(used, 0, (size_t)g_n);
    for (int r = 0; r < NEAR_LOG; r++) {
        int best = -1;
        for (int i = 0; i < g_n; i++)
            if (!used[i] && g_ent[i].dist < 250 && (best < 0 || g_ent[i].dist < g_ent[best].dist)) best = i;
        if (best < 0) break;
        used[best] = 1;
        out[(*m)++] = &g_ent[best];
    }
}

static unsigned id_of(uintptr_t p) { return (unsigned)((p >> 4) & 0xFFFFFF); }

static void csv_row(double t, const char *kind, unsigned id, int mask, double dx, double dy, double dist, double len, double wid,
                    double hdg, double v, double rl, double rs, int ind)
{
    fprintf(g_csv, "%.2f,%s,%06x,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,%.2f,%.2f,%.2f,%.1f,%d\n", t, kind, id, mask, dx, dy, dist, len, wid,
            hdg, v, rl, rs, fabs(T.speed) * 3.6, ind);
}

static void summary(double now, const ent *const *near_, int m)
{
    static double last;
    if (now - last < 60) return;
    last = now;
    char vts[160] = "";
    for (int i = 0; i < g_n_ovt && i < 4; i++)
        snprintf(vts + strlen(vts), sizeof vts - strlen(vts), "%s+0x%llx x%d%s", i ? ", " : "", (unsigned long long)(g_ovt[i].vt - g_img_lo),
                 g_ovt[i].n, g_ovt[i].ai ? " (vehicle)" : "");
    char nr[64] = "none";
    if (m > 0) snprintf(nr, sizeof nr, "%.0f m (%+.0f ahead, %+.0f left)", near_[0]->dist, near_[0]->dx, near_[0]->dy);
    logline("Traffic: %d vehicles read (spawned_1 %d, spawned_2 %d, objects_1 %d of which %d vehicles, trailers %d, in several lists %d, dropped %d), "
            "nearest %s; objects_1 types: %s; player list: %d vehicles (nearest %.1f m from our placement), %d trailers (nearest %.1f m); "
            "walk %.2f ms mean %.2f max over %d ticks, %u ticks with no list",
            g_n, g_cnt[0], g_cnt[1], g_cnt[2], g_cnt[3], g_cnt[4], g_cnt[7], g_bad, nr, vts[0] ? vts : "-", g_pv_n, g_pv_d, g_pt_n, g_pt_d,
            g_walk_n ? g_walk_ms_sum / g_walk_n : 0.0, g_walk_ms_max, g_walk_n, g_ticks_fail);
    g_walk_ms_sum = g_walk_ms_max = 0;
    g_walk_n = 0;
}

static void tr_frame()
{
    if (g_state < 2 || T.paused) return;
    static double last = -1;
    double now = wall_s();
    if (last >= 0 && now - last < TICK_MS / 1000.0 && now >= last) return;
    last = now;
    g_ticks++;

    char *gt = ptr_ok((void *)g_slot, 8) ? *(char **)g_slot : NULL;
    if (!gt || !ptr_ok(gt, GT_PTRL + 0x20)) {
        g_ticks_fail++;
        g_around.ok = 0;
        return;
    }
    uintptr_t vt = *(uintptr_t *)gt;
    if (vt < g_img_lo || vt >= g_img_hi) { g_ticks_fail++; g_around.ok = 0; return; }   // not an object of the game
    if (g_state == 2) {
        g_state = 3;
        logline("Traffic: game_traffic instance found (vtable +0x%llx), reading its lists (log only)", (unsigned long long)(vt - g_img_lo));
    }

    LARGE_INTEGER f0, a, b;
    QueryPerformanceFrequency(&f0);
    QueryPerformanceCounter(&a);
    tr_pose me = tr_pose_from_heading(T.place.position.x, T.place.position.z, T.place.orientation.heading);
    walk(gt, me);
    QueryPerformanceCounter(&b);
    double ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)f0.QuadPart;
    g_walk_ms_sum += ms; g_walk_n++;
    if (ms > g_walk_ms_max) g_walk_ms_max = ms;
    g_ok_at = now;

    ent *nr[NEAR_LOG];
    int m;
    pick_nearest(nr, &m);

    // the dev feed
    g_around.ok = 1;
    g_around.n = 0;
    {
        static unsigned char used[MAX_ENT];
        memset(used, 0, (size_t)g_n);
        for (int r = 0; r < 12; r++) {
            int best = -1;
            for (int i = 0; i < g_n; i++)
                if (!used[i] && g_ent[i].dist < 60 && (best < 0 || g_ent[i].dist < g_ent[best].dist)) best = i;
            if (best < 0) break;
            used[best] = 1;
            const ent &e = g_ent[best];
            traffic_obj &o = g_around.obj[g_around.n++];
            o.dx = e.dx; o.dy = e.dy; o.dist = e.dist; o.v = e.speed; o.hdg = e.hdg; o.len = e.len; o.wid = e.wid; o.kind = e.kind;
        }
    }
    // our trailer from the SDK
    g_around.has_trailer = 0;
    tr_pose tp = { 0, 0, 0, 0, 0 };
    if (T.trailer && T.trailer_place_t > 0 && now - T.trailer_place_t < 1.0) {
        tp = tr_pose_from_heading(T.trailer_place.position.x, T.trailer_place.position.z, T.trailer_place.orientation.heading);
        double dx, dy;
        tr_rel(me, tp.x, tp.z, &dx, &dy);
        if (plausible(dx, 100) && plausible(dy, 100)) {
            g_around.has_trailer = 1;
            g_around.t_dx = (float)dx; g_around.t_dy = (float)dy; g_around.t_hdg = (float)tr_rel_heading_deg(me, tp);
        }
    }
    g_around.truck_front_z = g_truck.front_z; g_around.truck_rear_z = g_truck.rear_z; g_around.truck_hook_z = g_truck.hook_z;
    g_around.trailer_hook_z = g_trailer.hook_z; g_around.trailer_rear_z = g_trailer.rear_z;

    // the csv: only while the engine runs
    if (g_csv && T.engine_on) {
        int ind = (T.blink_l ? 1 : 0) | (T.blink_r ? 2 : 0);
        for (int i = 0; i < m; i++) {
            const ent *e = nr[i];
            csv_row(now, e->kind == K_AI ? "A" : "AT", id_of(e->p), e->mask, e->dx, e->dy, e->dist, e->len, e->wid, e->hdg, e->speed,
                    e->rv_l, e->rv_s, ind);
        }
        if (g_around.has_trailer) {
            double dx = g_around.t_dx, dy = g_around.t_dy;
            csv_row(now, "S", 0, 0, dx, dy, sqrt(dx * dx + dy * dy), 0, 0, g_around.t_hdg, NAN, NAN, NAN, ind);
        }
        static unsigned slow;
        if (++slow % 5 == 0) {   // 1 per second: where the game's list puts us and our trailer, against the SDK
            if (g_pv_d == g_pv_d) csv_row(now, "PV", g_pv_n, 0, g_pv_dx, g_pv_dy, g_pv_d, 0, 0, NAN, NAN, NAN, NAN, ind);
            if (g_pt_d == g_pt_d) csv_row(now, "PT", g_pt_n, 0, g_pt_dx, g_pt_dy, g_pt_d, 0, 0, NAN, NAN, NAN, NAN, ind);
        }
        static double flushed;
        if (now - flushed > 2) { flushed = now; fflush(g_csv); csv_cap(&g_csv, "traffic.csv"); }
    }
    summary(now, nr, m);
}

void traffic_around_get(traffic_around *out)
{
    memset(out, 0, sizeof *out);
    out->truck_front_z = out->truck_rear_z = out->truck_hook_z = out->trailer_hook_z = out->trailer_rear_z = NAN;
    if (g_state < 3 || g_ok_at < 0 || wall_s() - g_ok_at > 1.0) return;
    *out = g_around;
}

// ---------------------------------------------------------------- configuration, start, reset

// The vehicle outlines for the page: the hook and the wheel positions (vehicle space, z backward).
static void tr_config(const scs_telemetry_configuration_t *c)
{
    int truck = !strcmp(c->id, "truck"), trl = !strcmp(c->id, "trailer.0");
    if (!truck && !trl) return;
    geo g = { NAN, NAN, NAN };
    for (const scs_named_value_t *a = c->attributes; a && a->name; a++) {
        if (a->value.type != SCS_VALUE_TYPE_fvector) continue;
        float z = a->value.value_fvector.z;
        if (!strcmp(a->name, "hook.position")) g.hook_z = z;
        else if (!strcmp(a->name, "wheel.position") && z == z) {
            if (!(g.front_z <= z)) g.front_z = z;   // smallest z = frontmost wheel
            if (!(g.rear_z >= z)) g.rear_z = z;
        }
    }
    (truck ? g_truck : g_trailer) = g;
}

static void tr_start()
{
    if (p_probe <= 0) return;
    if (!L || strcmp(L->name, "1.60")) {
        snprintf(g_status, sizeof g_status, "off: the traffic layout is ETS2 1.60 only");
        logline("Traffic: probe is on, but the list layout is for the 1.60 layout only: not reading");
        return;
    }
    unsigned stamp, size;
    if (!game_exe_stamp(&stamp, &size)) { snprintf(g_status, sizeof g_status, "off: game image unreadable"); return; }
    g_img_lo = (uintptr_t)GetModuleHandleW(NULL);
    g_img_hi = g_img_lo + size;
    uintptr_t hits[4];
    g_hits = game_scan_hits(k_pat, hits, 4);
    if (g_hits != 1) {
        g_state = 1;
        snprintf(g_status, sizeof g_status, "probe on, game_traffic pattern not found (%d hits)", g_hits);
        logline("Traffic: game_traffic pattern: %d hits (need exactly 1): not reading", g_hits);
        return;
    }
    int32_t rel;
    memcpy(&rel, (const void *)(hits[0] + 6), 4);
    g_slot = hits[0] + 10 + (intptr_t)rel;
    if (g_slot < g_img_lo || g_slot >= g_img_hi) {
        g_state = 1;
        snprintf(g_status, sizeof g_status, "probe on, game_traffic slot outside the image");
        logline("Traffic: game_traffic pattern hit 1 but the slot is outside the game image (+0x%llx): not reading", (unsigned long long)(g_slot - g_img_lo));
        return;
    }
    g_state = 2;
    g_csv = open_out("traffic.csv", "w");
    if (g_csv) fputs("t,kind,id,lists,dx,dy,dist,len,wid,hdg,speed,rv_long,rv_lat,ego_kmh,ind\n", g_csv);
    snprintf(g_status, sizeof g_status, "probe on (log only), game_traffic slot at +0x%llx%s", (unsigned long long)(g_slot - g_img_lo), g_csv ? "" : " (no csv)");
    logline("Traffic: game_traffic pattern found (1 hit), global slot at image +0x%llx, probe on: log only, nothing acts on it", (unsigned long long)(g_slot - g_img_lo));
}

static void tr_reset(int level, const char *)
{
    g_n = 0;
    g_around.ok = 0;
    g_ok_at = -1;
    if (level >= RESET_HARD) g_n_ai_vt = 0;   // another save: the vtables are learned again
}

static const char *tr_status() { return g_status; }

static void tr_stop() { if (g_csv) fflush(g_csv); }

module m_traffic = { "traffic", "Traffic", g_params, sizeof g_params / sizeof g_params[0], tr_start, tr_frame, tr_config, tr_stop, tr_reset, tr_status };
