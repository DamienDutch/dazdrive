// Traffic: a reader of the game's AI traffic list. It logs it; the one thing that acts on it is the AEB (core/aeblat.h: is the radar's lead in our path, or beside it?), which brakes less
// for a vehicle beside the path and nothing else (no tones, no steering).
//
// Purpose: to check on a ride that the list is read right, as the foundation for a blind-spot warning (real vehicles only, by
// distance, following the trailer), a cut-in-aware ACC and a turn assist later.
//
// Where it comes from (ets2la_plugin, MIT, gitlab.com/ETS2LA/ets2la_plugin, commit c925bd9, built for ETS2 1.60; notes/research.md § 6):
//   - game_traffic_u* global: pattern src/patterns.win32.hpp game_traffic (+6, rip-relative) = pattern k_pat in core/sense_traffic.cpp, where the whole walk lives since WP-S step S3.
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
#include "../core/sense.h"
#include "../core/sense_traffic.h"
#include "../core/trafficmath.h"
#include "../core/traffic.h"
#include "../core/blindspot.h"
#include <math.h>
#include <string.h>
#include <windows.h>


enum { NEAR_LOG = 8, TICK_MS = 200 };

static float p_probe, p_bs;

static param g_params[] = {
    { "blind_spot", &p_bs, 1, "with probe=1: our own blind-spot detector, LOG ONLY (nothing warns or steers): two side zones that follow the truck and the trailer; a vehicle that moves our way inside one (not a parked one, only above 15 km/h) is logged, per minute and as a 'would warn' when your indicator points to an occupied side. 0 = off" },
    { "probe", &p_probe, 0, "1 = read the game's list of AI traffic: the emergency braking (AEB) uses it to tell a vehicle in your path from one beside it, so it does not brake for the vehicle you are passing. Also logged: traffic.csv (needs the log at its highest level; 5 per second while driving, the 8 nearest vehicles) and one summary line a minute. 0 = off: the AEB brakes for whatever the game's radar reports" },
};

// the walk of the game's lists is core/sense_traffic.cpp (WP-S step S3); this module reads what it found (g_trs) and keeps the zones, the csv, the summary

static FILE *g_csv;
static char g_status[160] = "off (ini)";
static int g_state;                    // 0 off, 1 no pattern / wrong layout, 2 waiting for the instance, 3 reading
static double g_ok_at = -1;            // wall time of the last good walk

struct geo { float hook_z, front_z, rear_z; };
static geo g_truck = { NAN, NAN, NAN }, g_trailer = { NAN, NAN, NAN };

// what the last tick found, for the dev feed and the summary
static traffic_around g_around;
static double g_walk_ms_sum, g_walk_ms_max;
static int g_walk_n;
static unsigned g_ticks, g_ticks_fail;

// ---------------------------------------------------------------- per tick

static void pick_nearest(tr_ent **out, int *m)
{
    *m = 0;
    static unsigned char used[TR_MAX_ENT];
    memset(used, 0, (size_t)g_trs.n);
    for (int r = 0; r < NEAR_LOG; r++) {
        int best = -1;
        for (int i = 0; i < g_trs.n; i++)
            if (!used[i] && g_trs.ent[i].dist < 250 && (best < 0 || g_trs.ent[i].dist < g_trs.ent[best].dist)) best = i;
        if (best < 0) break;
        used[best] = 1;
        out[(*m)++] = &g_trs.ent[best];
    }
}

static unsigned id_of(uintptr_t p) { return (unsigned)((p >> 4) & 0xFFFFFF); }

static void csv_row(double t, const char *kind, unsigned id, int mask, double dx, double dy, double dist, double len, double wid,
                    double hdg, double v, double rl, double rs, int ind)
{
    fprintf(g_csv, "%.2f,%s,%06x,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,%.2f,%.2f,%.2f,%.1f,%d\n", t, kind, id, mask, dx, dy, dist, len, wid,
            hdg, v, rl, rs, fabs(T.speed) * 3.6, ind);
}

static void summary(double now, const tr_ent *const *near_, int m)
{
    static double last;
    if (now - last < 60) return;
    last = now;
    char vts[160] = "";
    for (int i = 0; i < g_trs.n_ovt && i < 4; i++)
        snprintf(vts + strlen(vts), sizeof vts - strlen(vts), "%s+0x%llx x%d%s", i ? ", " : "", (unsigned long long)(g_trs.ovt[i].vt - g_trs.img_lo),
                 g_trs.ovt[i].n, g_trs.ovt[i].ai ? " (vehicle)" : "");
    char nr[64] = "none";
    if (m > 0) snprintf(nr, sizeof nr, "%.0f m (%+.0f ahead, %+.0f left)", near_[0]->dist, near_[0]->dx, near_[0]->dy);
    logline("Traffic: %d vehicles read (spawned_1 %d, spawned_2 %d, objects_1 %d of which %d vehicles, trailers %d, in several lists %d, dropped %d), "
            "nearest %s; objects_1 types: %s; player list: %d vehicles (nearest %.1f m from our placement), %d trailers (nearest %.1f m); "
            "walk %.2f ms mean %.2f max over %d ticks, %u ticks with no list",
            g_trs.n, g_trs.cnt[0], g_trs.cnt[1], g_trs.cnt[2], g_trs.cnt[3], g_trs.cnt[4], g_trs.cnt[7], g_trs.bad, nr, vts[0] ? vts : "-", g_trs.pv_n, g_trs.pv_d, g_trs.pt_n, g_trs.pt_d,
            g_walk_n ? g_walk_ms_sum / g_walk_n : 0.0, g_walk_ms_max, g_walk_n, g_ticks_fail);
    g_walk_ms_sum = g_walk_ms_max = 0;
    g_walk_n = 0;
}

// ---------------------------------------------------------------- blind spot (log only; geometry in core/blindspot.h)

static const bs_cfg g_bs_cfg = bs_cfg_default();
static double g_bs_until[2], g_bs_since[2], g_bs_lastlog[2], g_bs_occ_s[2], g_bs_drive_s, g_bs_t = -1, g_bs_sum_at, g_bs_warn_at = -100;
static int g_bs_logged[2], g_bs_events[2], g_bs_warns, g_bs_warn_prev, g_bs_unknown;
static bs_body g_bs_body;

// The body the zones follow: the box from the player lists when it looks right, else the wheel positions from the configuration (the same
// guess the phone page draws), else fixed numbers.
static bs_body bs_build()
{
    bs_body b;
    memset(&b, 0, sizeof b);
    double w = 2.55;
    b.tf = g_truck.front_z == g_truck.front_z ? -g_truck.front_z + 1.2 : 3.5;
    b.tb = g_truck.rear_z == g_truck.rear_z ? g_truck.rear_z + 1.0 : 4.0;
    if (g_trs.own_v.ok && g_trs.own_v.f >= 1 && g_trs.own_v.f <= 6 && g_trs.own_v.r >= 1 && g_trs.own_v.r <= 10 && g_trs.own_v.f + g_trs.own_v.r >= 4 && g_trs.own_v.f + g_trs.own_v.r <= 12) {
        b.tf = g_trs.own_v.f; b.tb = g_trs.own_v.r;
        if (g_trs.own_v.w > 2.2f && g_trs.own_v.w < 3.2f) w = g_trs.own_v.w;
    }
    b.hw = b.lhw = w * 0.5;
    if (g_around.has_trailer) {
        b.trailer = 1;
        b.t_dx = g_around.t_dx; b.t_dy = g_around.t_dy; b.t_hdg = g_around.t_hdg;
        b.lf = g_trailer.hook_z == g_trailer.hook_z ? -g_trailer.hook_z : 5.0;
        b.lb = g_trailer.rear_z == g_trailer.rear_z ? g_trailer.rear_z + 1.0 : 5.5;
        if (g_trs.own_t.ok && g_trs.own_t.f >= 2 && g_trs.own_t.f <= 12 && g_trs.own_t.r >= 2 && g_trs.own_t.r <= 14 && g_trs.own_t.f + g_trs.own_t.r >= 6 && g_trs.own_t.f + g_trs.own_t.r <= 20) {
            b.lf = g_trs.own_t.f; b.lb = g_trs.own_t.r;
        }
    }
    return b;
}

static void bs_tick(double now)
{
    g_around.bs_on = g_around.bs_left = g_around.bs_right = g_around.bs_warn = g_around.bs_warn_l = g_around.bs_warn_r = 0;
    if (p_bs <= 0) return;
    double dt = g_bs_t >= 0 && now > g_bs_t ? now - g_bs_t : 0;
    if (dt > 0.5) dt = 0.5;
    g_bs_t = now;
    const bs_cfg &c = g_bs_cfg;
    double kmh = fabs(T.speed) * 3.6;
    int gate = T.engine_on && (kmh > c.min_kmh || ((now < g_bs_until[0] || now < g_bs_until[1]) && kmh > c.min_kmh - 3));
    g_bs_body = bs_build();
    g_around.bs_tf = (float)g_bs_body.tf; g_around.bs_tb = (float)g_bs_body.tb; g_around.bs_hw = (float)g_bs_body.hw;
    g_around.bs_lf = (float)g_bs_body.lf; g_around.bs_lb = (float)g_bs_body.lb; g_around.bs_lhw = (float)g_bs_body.lhw;
    g_around.bs_inner = (float)c.inner; g_around.bs_outer = (float)c.outer; g_around.bs_fm = (float)c.front_margin; g_around.bs_rm = (float)c.rear_margin;

    int raw[2] = { 0, 0 };
    const tr_ent *who[2] = { NULL, NULL };
    if (gate) {
        g_bs_drive_s += dt;
        bs_poly z[2][4];
        int nz[2];
        for (int s = 0; s < 2; s++) nz[s] = bs_zone(g_bs_body, c, s ? -1 : 1, z[s]);
        for (int i = 0; i < g_trs.n; i++) {
            const tr_ent &e = g_trs.ent[i];
            if (!(e.dist < 45)) continue;
            double v = e.kind == K_AI ? e.speed : e.pspeed;
            if (!(v == v)) { g_bs_unknown++; continue; }
            if (!bs_eligible(c, e.hdg, v, fabs(T.speed))) continue;
            bs_poly vp = bs_vehicle(e.dx, e.dy, e.len > 1 ? e.len : 4.5, e.wid > 0.5 ? e.wid : 2.0, e.hdg);
            for (int s = 0; s < 2; s++) {
                int hit = 0;
                for (int k = 0; k < nz[s] && !hit; k++) hit = bs_overlap(z[s][k], vp);
                if (hit) { raw[s] = 1; if (!who[s] || e.dist < who[s]->dist) who[s] = &e; }
            }
        }
    }
    int occ[2];
    for (int s = 0; s < 2; s++) {
        if (!gate) g_bs_until[s] = 0;
        occ[s] = bs_hold(&g_bs_until[s], raw[s], now, c.hold_s);
        if (occ[s]) g_bs_occ_s[s] += dt;
        const char *nm = s ? "right" : "left";
        if (occ[s] && !g_bs_since[s]) {
            g_bs_since[s] = now;
            g_bs_events[s]++;
            if (now - g_bs_lastlog[s] > 2.0 && who[s]) {
                g_bs_lastlog[s] = now; g_bs_logged[s] = 1;
                logline("Blind spot: %s occupied by a %s, %.1f m %s, %.1f m %s, %.0f km/h, heading %+.0f deg (we drive %.0f km/h)", nm,
                        who[s]->kind == K_AI ? "vehicle" : "trailer", fabs(who[s]->dx), who[s]->dx >= 0 ? "ahead" : "behind",
                        fabs(who[s]->dy), who[s]->dy >= 0 ? "left" : "right",
                        (who[s]->kind == K_AI ? who[s]->speed : who[s]->pspeed) * 3.6, who[s]->hdg, kmh);
            }
        } else if (!occ[s] && g_bs_since[s]) {
            if (g_bs_logged[s]) { logline("Blind spot: %s free again after %.1f s", nm, now - g_bs_since[s]); g_bs_lastlog[s] = now; }
            g_bs_logged[s] = 0;
            g_bs_since[s] = 0;
        }
    }
    // the indicator: the lever (SDK) or the game's own flag bits (+0x1d0: 4 = left, 8 = right, the way lane.cpp reads them)
    unsigned fl = sense_lights_aligned(sense_actor());
    int ind_l = T.blink_l || (fl & 4), ind_r = T.blink_r || (fl & 8);
    int warn = gate && ((ind_l && occ[0]) || (ind_r && occ[1]));
    if (warn && !g_bs_warn_prev) {
        g_bs_warns++;
        if (now - g_bs_warn_at > 1.0) {
            g_bs_warn_at = now;
            logline("Blind spot: WOULD WARN, indicator %s and that side is occupied (we drive %.0f km/h)",
                    ind_l && occ[0] && ind_r && occ[1] ? "both" : (ind_l && occ[0] ? "left" : "right"), kmh);
        }
    }
    g_bs_warn_prev = warn;
    g_around.bs_on = gate;
    g_around.bs_left = occ[0]; g_around.bs_right = occ[1]; g_around.bs_warn = warn;
    g_around.bs_warn_l = gate && ind_l && occ[0]; g_around.bs_warn_r = gate && ind_r && occ[1];

    // traffic.csv: one row "B" per tick while the detector runs: lists = bits (1 left, 2 right, 4 would warn), dx/dy/len/wid/hdg/speed = the
    // nearest vehicle in a zone (nan = none)
    if (g_csv && gate) {
        const tr_ent *w = who[0] && (!who[1] || who[0]->dist <= who[1]->dist) ? who[0] : who[1];
        int ind = (T.blink_l ? 1 : 0) | (T.blink_r ? 2 : 0);
        int bits = occ[0] | (occ[1] << 1) | (warn << 2);
        if (w) csv_row(now, "B", 0, bits, w->dx, w->dy, w->dist, w->len, w->wid, w->hdg, w->kind == K_AI ? w->speed : w->pspeed, NAN, NAN, ind);
        else csv_row(now, "B", 0, bits, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, ind);
    }
    // once a minute: seconds occupied, and what the zones were made of (the boxes from the lists against the wheel guess)
    if (now - g_bs_sum_at >= 60) {
        g_bs_sum_at = now;
        if (g_bs_drive_s > 0.5) {
            logline("Blind spot: last minute at >%.0f km/h for %.0f s: left occupied %.1f s (%d times), right %.1f s (%d times), would warn %d times, "
                    "%d vehicles of unknown speed skipped; zones: truck %.1f ahead %.1f behind, half width %.2f%s; trailer %.1f ahead %.1f behind of its origin%s; "
                    "boxes from the lists: truck %s (%.1f ahead %.1f behind, %.2f wide, origin %.1f m off), trailer %s (%.1f ahead %.1f behind, %.2f wide, origin %.1f m off)",
                    c.min_kmh, g_bs_drive_s, g_bs_occ_s[0], g_bs_events[0], g_bs_occ_s[1], g_bs_events[1], g_bs_warns, g_bs_unknown,
                    g_bs_body.tf, g_bs_body.tb, g_bs_body.hw, g_trs.own_v.ok ? "" : " (wheel guess)", g_bs_body.lf, g_bs_body.lb, g_bs_body.trailer ? "" : " (no trailer)",
                    g_trs.own_v.ok ? "found" : "none", g_trs.own_v.f, g_trs.own_v.r, g_trs.own_v.w, g_trs.own_v.dist < 1e8f ? g_trs.own_v.dist : -1.f,
                    g_trs.own_t.ok ? "found" : "none", g_trs.own_t.f, g_trs.own_t.r, g_trs.own_t.w, g_trs.own_t.dist < 1e8f ? g_trs.own_t.dist : -1.f);
        }
        g_bs_drive_s = g_bs_occ_s[0] = g_bs_occ_s[1] = 0;
        g_bs_events[0] = g_bs_events[1] = g_bs_warns = g_bs_unknown = 0;
    }
}

static void tr_frame()
{
    if (g_state < 2 || T.paused) return;
    static double last = -1;
    double now = wall_s();
    if (last >= 0 && now - last < TICK_MS / 1000.0 && now >= last) return;
    last = now;
    g_ticks++;

    unsigned long long vt_off;
    if (!trs_instance(&vt_off)) {   // no object at the slot, or not an object of the game
        g_ticks_fail++;
        g_around.ok = 0;
        return;
    }
    if (g_state == 2) {
        g_state = 3;
        logline("Traffic: game_traffic instance found (vtable +0x%llx), reading its lists", vt_off);
    }

    LARGE_INTEGER f0, a, b;
    QueryPerformanceFrequency(&f0);
    QueryPerformanceCounter(&a);
    tr_pose me = tr_pose_from_heading(T.place.position.x, T.place.position.z, T.place.orientation.heading);
    trs_walk(me);
    QueryPerformanceCounter(&b);
    double ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)f0.QuadPart;
    g_walk_ms_sum += ms; g_walk_n++;
    if (ms > g_walk_ms_max) g_walk_ms_max = ms;
    g_ok_at = now;

    tr_ent *nr[NEAR_LOG];
    int m;
    pick_nearest(nr, &m);

    // the dev feed
    g_around.ok = 1;
    g_around.n = 0;
    {
        static unsigned char used[TR_MAX_ENT];
        memset(used, 0, (size_t)g_trs.n);
        for (int r = 0; r < 12; r++) {
            int best = -1;
            for (int i = 0; i < g_trs.n; i++)
                if (!used[i] && g_trs.ent[i].dist < 60 && (best < 0 || g_trs.ent[i].dist < g_trs.ent[best].dist)) best = i;
            if (best < 0) break;
            used[best] = 1;
            const tr_ent &e = g_trs.ent[best];
            traffic_obj &o = g_around.obj[g_around.n++];
            o.dx = e.dx; o.dy = e.dy; o.dist = e.dist; o.v = e.speed; o.hdg = e.hdg; o.len = e.len; o.wid = e.wid; o.kind = e.kind;
            o.id = id_of(e.p); o.rv_long = e.rv_l; o.rv_lat = e.rv_s;
        }
    }
    // the wider list for the cut-in reading
    g_around.read_t = now;
    g_around.n_ahead = 0;
    {
        static unsigned char used[TR_MAX_ENT];
        memset(used, 0, (size_t)g_trs.n);
        for (int r = 0; r < 16; r++) {
            int best = -1;
            for (int i = 0; i < g_trs.n; i++)
                if (!used[i] && g_trs.ent[i].dist < 130 && g_trs.ent[i].dx > -15 && fabsf(g_trs.ent[i].dy) < 25 && (best < 0 || g_trs.ent[i].dist < g_trs.ent[best].dist)) best = i;
            if (best < 0) break;
            used[best] = 1;
            const tr_ent &e = g_trs.ent[best];
            traffic_obj &o = g_around.ahead[g_around.n_ahead++];
            o.dx = e.dx; o.dy = e.dy; o.dist = e.dist; o.v = e.speed; o.hdg = e.hdg; o.len = e.len; o.wid = e.wid; o.kind = e.kind;
            o.id = id_of(e.p); o.rv_long = e.rv_l; o.rv_lat = e.rv_s;
        }
    }
    // our trailer from the SDK
    g_around.has_trailer = 0;
    tr_pose tp = { 0, 0, 0, 0, 0 };
    if (T.trailer && T.trailer_place_t > 0 && now - T.trailer_place_t < 1.0) {
        tp = tr_pose_from_heading(T.trailer_place.position.x, T.trailer_place.position.z, T.trailer_place.orientation.heading);
        double dx, dy;
        tr_rel(me, tp.x, tp.z, &dx, &dy);
        if (trs_plausible(dx, 100) && trs_plausible(dy, 100)) {
            g_around.has_trailer = 1;
            g_around.t_dx = (float)dx; g_around.t_dy = (float)dy; g_around.t_hdg = (float)tr_rel_heading_deg(me, tp);
        }
    }
    g_around.truck_front_z = g_truck.front_z; g_around.truck_rear_z = g_truck.rear_z; g_around.truck_hook_z = g_truck.hook_z;
    g_around.trailer_hook_z = g_trailer.hook_z; g_around.trailer_rear_z = g_trailer.rear_z;
    bs_tick(now);

    // the csv: only while the engine runs
    if (g_csv && T.engine_on) {
        int ind = (T.blink_l ? 1 : 0) | (T.blink_r ? 2 : 0);
        for (int i = 0; i < m; i++) {
            const tr_ent *e = nr[i];
            csv_row(now, e->kind == K_AI ? "A" : "AT", id_of(e->p), e->mask, e->dx, e->dy, e->dist, e->len, e->wid, e->hdg, e->speed,
                    e->rv_l, e->rv_s, ind);
        }
        if (g_around.has_trailer) {
            double dx = g_around.t_dx, dy = g_around.t_dy;
            csv_row(now, "S", 0, 0, dx, dy, sqrt(dx * dx + dy * dy), 0, 0, g_around.t_hdg, NAN, NAN, NAN, ind);
        }
        static unsigned slow;
        if (++slow % 5 == 0) {   // 1 per second: where the game's list puts us and our trailer, against the SDK
            if (g_trs.pv_d == g_trs.pv_d) csv_row(now, "PV", g_trs.pv_n, 0, g_trs.pv_dx, g_trs.pv_dy, g_trs.pv_d, 0, 0, NAN, NAN, NAN, NAN, ind);
            if (g_trs.pt_d == g_trs.pt_d) csv_row(now, "PT", g_trs.pt_n, 0, g_trs.pt_dx, g_trs.pt_dy, g_trs.pt_d, 0, 0, NAN, NAN, NAN, NAN, ind);
        }
        static double flushed;
        if (now - flushed > 2) { flushed = now; fflush(g_csv); csv_cap(&g_csv, "traffic.csv"); }
    }
    summary(now, nr, m);
}

int traffic_probe_on() { return p_probe > 0; }

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
    if (!c) return;
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
    g_state = trs_discover(g_status, sizeof g_status);   // the pattern scan is core/sense_traffic.cpp; 0 = no layout / no image (state stays 0), 1 = no pattern
    if (g_state != 2) return;
    g_csv = open_out("traffic.csv", "w");
    if (g_csv) fputs("t,kind,id,lists,dx,dy,dist,len,wid,hdg,speed,rv_long,rv_lat,ego_kmh,ind\n", g_csv);
    snprintf(g_status, sizeof g_status, "probe on (the AEB reads it), game_traffic slot at +0x%llx%s", trs_slot_off(), g_csv ? "" : " (no csv)");
    logline("Traffic: game_traffic pattern found (1 hit), global slot at image +0x%llx, probe on: the AEB reads where its lead is sideways", trs_slot_off());
}

static void tr_reset(int level, const char *)
{
    g_trs.n = 0;
    g_around.ok = 0;
    g_bs_until[0] = g_bs_until[1] = 0; g_bs_since[0] = g_bs_since[1] = 0; g_bs_logged[0] = g_bs_logged[1] = 0; g_bs_warn_prev = 0; g_bs_t = -1;
    g_around.bs_on = g_around.bs_left = g_around.bs_right = g_around.bs_warn = g_around.bs_warn_l = g_around.bs_warn_r = 0;
    g_ok_at = -1;
    if (level >= RESET_HARD) trs_forget_vtables();   // another save: the vtables are learned again
}

static const char *tr_status() { return g_status; }

static void tr_stop() { if (g_csv) fflush(g_csv); }

module m_traffic = { "traffic", "Traffic", g_params, sizeof g_params / sizeof g_params[0], tr_start, tr_frame, tr_config, tr_stop, tr_reset, tr_status, NULL, VCM_BOTH };
