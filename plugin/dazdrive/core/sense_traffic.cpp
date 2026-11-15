// The sense layer's reading of the game's AI traffic list (WP-S step S3; it was the memory walk of modules/traffic.cpp, copied here as it was).
// Where it comes from and what is read: see the head of modules/traffic.cpp (ets2la_plugin, MIT; ETS2 1.60 layout, every pointer through mem_readable).
#include "sense_traffic.h"
#include "game.h"
#include "sense.h"
#include <math.h>
#include <string.h>
#include <windows.h>

tr_sense g_trs;
static const int s_init = (g_trs.pv_d = g_trs.pt_d = NAN, 0);   // (as the module's own statics were: NAN until the first walk)

static const char *const k_pat = "48 8B D9 48 8B 0D ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 83 ?? ?? ?? ?? 48 85 C0";
enum {
    GT_SPAWNED1 = 0xF0, GT_SPAWNED2 = 0x118, GT_OBJECTS1 = 0x1E0, GT_PVEH = 0x208, GT_PTRL = 0x230,   // arrays inside game_traffic_u
    ARR_DATA = 0x8, ARR_SIZE = 0x10,                                                                   // inside an array_dyn_t
    ACT_PLACEMENT = 0x28, ACT_SLAVE = 0x68, ACT_END = 0x70,                                            // traffic_actor_t
    AI_PHYSICS = 0x238, PH_SPEED = 0x70, PV_SPEED = 0x218, PV_END = 0x220,
    MAX_LIST = 512,
};

static uintptr_t s_gt;                 // the instance trs_instance found

// ---------------------------------------------------------------- safe reads

static int ptr_ok(const void *p, size_t n) { return sense_ptr_ok(p, n); }   // (core/sense.h: 8-aligned and readable)

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
static int read_actor(const char *p, tr_ent *e)
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
    e->bz0 = box[2]; e->bz1 = box[5];
    e->speed = e->pspeed = NAN;
    e->mask = 0;
    return 1;
}

static float g_cur_pspeed = NAN;   // the vehicle being added: its speed, for the trailers behind it

static void add_ai(const char *p, int list, int kind, float speed)
{
    for (int i = 0; i < g_trs.n; i++)
        if (g_trs.ent[i].p == (uintptr_t)p) { g_trs.ent[i].mask |= list; g_trs.cnt[7]++; return; }
    if (g_trs.n >= TR_MAX_ENT) return;
    tr_ent e;
    if (!read_actor(p, &e)) { g_trs.bad++; return; }
    e.mask = list;
    e.kind = kind;
    e.speed = speed;
    e.pspeed = kind == K_AI ? speed : g_cur_pspeed;
    g_trs.ent[g_trs.n++] = e;
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
    if (!ptr_ok(p, AI_PHYSICS + 8)) { g_trs.bad++; return; }
    g_cur_pspeed = ai_speed(p);
    add_ai(p, list, K_AI, g_cur_pspeed);
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
    for (int i = 0; i < g_trs.n_ai_vt; i++) if (g_trs.ai_vt[i] == vt) return 1;
    return 0;
}

static void learn_ai_vt(const char *p)
{
    if (!ptr_ok(p, 8)) return;
    uintptr_t vt = *(const uintptr_t *)p;
    if (vt < g_trs.img_lo || vt >= g_trs.img_hi || known_ai_vt(vt) || g_trs.n_ai_vt >= 4) return;
    g_trs.ai_vt[g_trs.n_ai_vt++] = vt;
}

// ---------------------------------------------------------------- the walk

static void walk(const char *gt, const tr_pose &me)
{
    g_trs.n = g_trs.bad = 0;
    memset(g_trs.cnt, 0, sizeof g_trs.cnt);
    g_trs.n_ovt = 0;
    const char *d;
    unsigned n;

    // 1. spawned_vehicles_1 / _2: entries of 0x10 (vehicle pointer first)
    for (int k = 0; k < 2; k++) {
        if (!arr_get(gt, k ? GT_SPAWNED2 : GT_SPAWNED1, 0x10, &d, &n)) { g_trs.bad++; continue; }
        g_trs.cnt[k] = (int)n;
        for (unsigned i = 0; i < n; i++) {
            const char *v = *(const char *const *)(d + i * 0x10);
            learn_ai_vt(v);
            add_vehicle(v, k ? L_SP2 : L_SP1);
        }
    }
    // 2. traffic_objects_1: every kind of object; the vehicles are the ones with a vehicle vtable (no call into the game)
    if (arr_get(gt, GT_OBJECTS1, 8, &d, &n)) {
        g_trs.cnt[2] = (int)n;
        for (unsigned i = 0; i < n; i++) {
            const char *o = *(const char *const *)(d + i * 8);
            if (!ptr_ok(o, 8)) { g_trs.bad++; continue; }
            uintptr_t vt = *(const uintptr_t *)o;
            int ai = known_ai_vt(vt), j;
            for (j = 0; j < g_trs.n_ovt && g_trs.ovt[j].vt != vt; j++) { }
            if (j == g_trs.n_ovt && g_trs.n_ovt < 8) { g_trs.ovt[j].vt = vt; g_trs.ovt[j].n = 0; g_trs.ovt[j].ai = ai; g_trs.n_ovt++; }
            if (j < g_trs.n_ovt) g_trs.ovt[j].n++;
            if (!ai) continue;
            g_trs.cnt[3]++;
            add_vehicle(o, L_OBJ);
        }
    } else g_trs.bad++;
    for (int i = 0; i < g_trs.n; i++) if (g_trs.ent[i].kind == K_AIT) g_trs.cnt[4]++;

    // our frame
    for (int i = 0; i < g_trs.n; i++) {
        tr_ent &e = g_trs.ent[i];
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
    g_trs.pv_n = g_trs.pt_n = 0;
    g_trs.pv_d = g_trs.pt_d = NAN;
    g_trs.own_v.ok = g_trs.own_t.ok = 0;
    g_trs.own_v.dist = g_trs.own_t.dist = 1e9f;
    const int sdk_trl = T.trailer && T.trailer_place_t > 0 && wall_s() - T.trailer_place_t < 1.0;
    for (int k = 0; k < 2; k++) {
        if (!arr_get(gt, k ? GT_PTRL : GT_PVEH, 8, &d, &n)) { g_trs.bad++; continue; }
        if (k) g_trs.pt_n = (int)n; else g_trs.pv_n = (int)n;
        for (unsigned i = 0; i < n; i++) {
            tr_ent e;
            const char *v = *(const char *const *)(d + i * 8);
            if (!read_actor(v, &e)) { g_trs.bad++; continue; }
            double dx, dy;
            tr_rel(me, e.ox, e.oz, &dx, &dy);   // the origin, not the box middle: ours is the SDK placement origin
            double dist = sqrt(dx * dx + dy * dy);
            double &bd = k ? g_trs.pt_d : g_trs.pv_d;
            if (bd != bd || dist < bd) { bd = dist; (k ? g_trs.pt_dx : g_trs.pv_dx) = dx; (k ? g_trs.pt_dy : g_trs.pv_dy) = dy; }
            // the box of the one that is ours: its origin within 2 m of the SDK placement (the truck's, or the trailer's)
            {
                double ex = k ? (sdk_trl ? T.trailer_place.position.x : NAN) : me.x, ez = k ? (sdk_trl ? T.trailer_place.position.z : NAN) : me.z;
                double od = sqrt((e.ox - ex) * (e.ox - ex) + (e.oz - ez) * (e.oz - ez));
                tr_own_box &ob = k ? g_trs.own_t : g_trs.own_v;
                if (od < 2.0 && od < ob.dist) {
                    ob.dist = (float)od;
                    ob.f = -fminf(e.bz0, e.bz1); ob.r = fmaxf(e.bz0, e.bz1); ob.w = e.wid;
                    ob.ok = 1;
                }
            }
        }
    }
}

void trs_walk(const tr_pose &me) { walk((const char *)s_gt, me); }
void trs_forget_vtables() { g_trs.n_ai_vt = 0; }
unsigned long long trs_slot_off() { return (unsigned long long)(g_trs.slot - g_trs.img_lo); }

int trs_instance(unsigned long long *vt_off)
{
    char *gt = ptr_ok((void *)g_trs.slot, 8) ? *(char **)g_trs.slot : NULL;
    if (!gt || !ptr_ok(gt, GT_PTRL + 0x20)) return 0;
    uintptr_t vt = *(uintptr_t *)gt;
    if (vt < g_trs.img_lo || vt >= g_trs.img_hi) return 0;   // not an object of the game
    s_gt = (uintptr_t)gt;
    *vt_off = (unsigned long long)(vt - g_trs.img_lo);
    return 1;
}

int trs_discover(char *status, size_t nstatus)
{
    if (!L) {
        snprintf(status, nstatus, "off: no layout");
        logline("Traffic: probe is on, but there is no layout: not reading");
        return 0;
    }
    unsigned stamp, size;
    if (!game_exe_stamp(&stamp, &size)) { snprintf(status, nstatus, "off: game image unreadable"); return 0; }
    g_trs.img_lo = (uintptr_t)GetModuleHandleW(NULL);
    g_trs.img_hi = g_trs.img_lo + size;
    uintptr_t hits[4];
    g_trs.hits = game_scan_hits(k_pat, hits, 4);
    if (g_trs.hits != 1) {
        snprintf(status, nstatus, "probe on, game_traffic pattern not found (%d hits)", g_trs.hits);
        logline("Traffic: game_traffic pattern: %d hits (need exactly 1): not reading", g_trs.hits);
        return 1;
    }
    int32_t rel;
    memcpy(&rel, (const void *)(hits[0] + 6), 4);
    g_trs.slot = hits[0] + 10 + (intptr_t)rel;
    if (g_trs.slot < g_trs.img_lo || g_trs.slot >= g_trs.img_hi) {
        snprintf(status, nstatus, "probe on, game_traffic slot outside the image");
        logline("Traffic: game_traffic pattern hit 1 but the slot is outside the game image (+0x%llx): not reading", (unsigned long long)(g_trs.slot - g_trs.img_lo));
        return 1;
    }
    return 2;
}
