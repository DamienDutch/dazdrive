// Incident recorder: instead of a broad log, a file per thing that went wrong (see suite.h, "incident.cpp").
//
// Two rings run all the time in memory: the telemetry row of the truck 10 times a second for the last 20 s, and the last
// log lines (log.cpp keeps those, also the ones the log level leaves out of dazdrive.log). incident() writes both to
// incidents\<kind>_<local date_time>.csv and keeps writing rows and new log lines until 15 s after the last trigger.
#include "suite.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 40;  // dazdrive.ini [core] incident_mb; the standard preset has 4

enum { HZ = 10, PRE_S = 20, POST_S = 15, MAX_FILE_S = 90, RING = PRE_S * HZ + 8, MAX_FILES = 40, MAX_PER_KIND = 12 };

struct row {
    double t;
    float kmh, rpm, thr, brake, accel, grade, set_kmh, gap;
    short gear, target, mode, flags;
};
static row g_rows[RING];
static int g_n, g_i;
static double g_row_at, g_prev_wall;

static FILE *g_f;
static double g_t0, g_last;  // trigger of the first incident in the file; time of the latest trigger (the file ends POST_S after)
static int g_files;
static struct { char kind[24]; int n; } g_kinds[16];

static void put_row(FILE *f, const row &r, double t0)
{
    fprintf(f, "%.1f,%.1f,%.0f,%d,%d,%.2f,%.2f,%.2f,%.1f,%.0f,%.1f,%d,%d\n", r.t - t0, r.kmh, r.rpm, r.gear, r.target, r.thr, r.brake,
            r.accel, r.grade == r.grade ? r.grade : 0.0f, r.set_kmh, r.gap == r.gap ? r.gap : 0.0f, r.mode, r.flags);
}

// every new log line while a file is open (the lock is held)
static void tap(double t, const char *s)
{
    if (g_f) fprintf(g_f, "# %+.2f  %s\n", t - g_t0, s);
}

static void close_file(const char *why)
{
    if (!g_f) return;
    fprintf(g_f, "# end (%s)\n", why);
    fclose(g_f);
    g_f = NULL;
    g_log_tap = NULL;
}

void incident(const char *kind, const char *fmt, ...)
{
    if (g_incident_mb <= 0 || !g_n) return;  // off, or no row recorded yet (not driving)
    char detail[240];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);
    double now = wall_s();
    log_lock();
    if (g_f) {  // one file per episode: the next trigger is added to it and the file runs on
        g_last = now;
        log_unlock();
        logline("incident: %s: %s (same file)", kind, detail);
        return;
    }
    int k = 0;
    while (k < 16 && g_kinds[k].kind[0] && strcmp(g_kinds[k].kind, kind)) k++;
    if (k < 16 && !g_kinds[k].kind[0]) snprintf(g_kinds[k].kind, sizeof g_kinds[k].kind, "%s", kind);
    if (g_files >= MAX_FILES || (k < 16 && g_kinds[k].n >= MAX_PER_KIND)) {  // one hunting loop must not push all history out
        static int said;
        log_unlock();
        if (said++ < 3) logline("incident: %s: %s (not recorded: the limit for this session is reached)", kind, detail);
        return;
    }
    char stamp[32];
    g_f = open_incident("incidents", kind, g_incident_mb, stamp, sizeof stamp);
    if (!g_f) { log_unlock(); logline("incident: %s: %s (no file could be made)", kind, detail); return; }
    g_files++;
    if (k < 16) g_kinds[k].n++;
    g_t0 = g_last = now;
    fprintf(g_f, "# " SUITE_NAME " " SUITE_VERSION " incident \"%s\" at %s (local time); log clock %.0f s in " SUITE_NAME ".log\n", kind, stamp, now);
    fprintf(g_f, "# %s\n", detail);
    fprintf(g_f, "# truck: %d forward gears, rpm limit %.0f, cargo %.1f t, trailer %s\n", T.fwd_gears, T.rpm_limit, T.cargo_mass / 1000,
            T.trailer ? "yes" : "no");
    fprintf(g_f, "# time is seconds from the trigger (negative = before). Lines starting with # are log lines (before: the last %d s).\n", PRE_S);
    fprintf(g_f, "# mode: 0 off/game 1 Eco 2 Normal 3 Power 4 M. flags: 1 neutral roll, 2 crawl, 4 AEB, 8 speed-limit following, 16 hint hold.\n");
    log_ring_dump(g_f, now - PRE_S, now);
    fputs("t,kmh,rpm,gear,target,thr,brake,accel,grade_pct,set_kmh,gap_m,mode,flags\n", g_f);
    for (int j = 0; j < g_n; j++) put_row(g_f, g_rows[(g_i - g_n + j + RING) % RING], now);
    fflush(g_f);
    g_log_tap = tap;
    log_unlock();
    logline("incident: %s: %s -> incidents\\%s_%s.csv", kind, detail, kind, stamp);
}

void incident_frame()
{
    double now = wall_s();
    double dt = now - g_prev_wall;
    g_prev_wall = now;
    if (g_f) {
        log_lock();
        if (T.paused && g_f) g_last += dt;  // the time in the pause menu does not count as "after"
        if (g_f && (now - g_last > POST_S || now - g_t0 > MAX_FILE_S)) close_file(now - g_last > POST_S ? "quiet again" : "file limit");
        log_unlock();
    }
    if (T.paused || !T.engine_on || now - g_row_at < 1.0 / HZ) return;
    g_row_at = now;
    row &r = g_rows[g_i];
    r.t = now;
    r.kmh = T.speed * 3.6f; r.rpm = T.rpm; r.thr = T.in_thr; r.brake = T.in_brake; r.accel = T.accel;
    r.grade = g_inc.grade_pct; r.set_kmh = T.cruise * 3.6f; r.gap = g_inc.gap_m;
    r.gear = (short)T.gear; r.target = (short)g_inc.target; r.mode = (short)g_inc.mode;
    r.flags = (short)(g_inc.flags | (g_crawl_on ? INC_CRAWL : 0));
    g_i = (g_i + 1) % RING;
    if (g_n < RING) g_n++;
    if (g_f) {
        log_lock();
        if (g_f) {
            put_row(g_f, r, g_t0);
            static double flushed;
            if (now - flushed > 2) { flushed = now; fflush(g_f); }
        }
        log_unlock();
    }
}

void incident_stop()
{
    log_lock();
    close_file("game closed");
    log_unlock();
}
