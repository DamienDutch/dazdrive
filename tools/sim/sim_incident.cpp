// Test of the incident recorder (core/incident.cpp + core/log.cpp) as a Windows exe, run under wine by run_incident.sh.
// Real time (the clock is GetTickCount64): ~28 s. Checks: quiet at level 0 (only the start line in dazdrive.log), the previous
// log kept as dazdrive.prev.log, an incident file with the 20 s before (including level 1 lines that never reached the log),
// rows at 10 Hz, post-trigger lines, a second trigger joining the file, a different kind afterwards, the folder cap.
#include "core/suite.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

telemetry T;
volatile int g_crawl_on;
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static char *slurp(const char *path, long *n)
{
    FILE *f = fopen(path, "rb");
    if (!f) { *n = -1; return NULL; }
    fseek(f, 0, SEEK_END); *n = ftell(f); fseek(f, 0, SEEK_SET);
    char *b = (char *)calloc(*n + 1, 1);
    fread(b, 1, *n, f);
    fclose(f);
    return b;
}
static void tick(double secs)
{
    double t0 = wall_s();
    while (wall_s() - t0 < secs) {
        T.speed = 22; T.rpm = 1400; T.gear = 8; T.in_thr = 0.4f; T.engine_on = 1;
        incident_frame();
        Sleep(16);
    }
}
static int count(const char *b, const char *needle)
{
    int n = 0;
    for (const char *p = b; (p = strstr(p, needle)); p++) n++;
    return n;
}

int main()
{
    snprintf(g_dir, sizeof g_dir, "dztest\\");
    CreateDirectoryA("dztest", NULL);
    FILE *old = fopen("dztest\\dazdrive.log", "w");
    fputs("OLD SESSION LINE\n", old);
    fclose(old);
    g_log_level = 0; g_log_max_mb = 1; g_incident_mb = 0.05;  // a 50 KB cap, to see the trimming
    log_open();
    logline("start line");
    tick(6);
    logv(1, "level one detail BEFORE (ring only)");
    logv(2, "level two detail (not even the ring)");
    tick(3);
    incident("hunt", "gears 8->9->8->9 in %.1f s", 4.0);
    logline("after line one");
    tick(2);
    incident("limiter", "rpm %d", 1900);   // joins the file
    tick(16);                              // quiet: the file closes 15 s after the last trigger
    long n;
    char *prev = slurp("dztest\\dazdrive.prev.log", &n);
    CHECK(prev && strstr(prev, "OLD SESSION LINE"), "previous log not kept as dazdrive.prev.log");
    char *log = slurp("dztest\\dazdrive.log", &n);
    CHECK(log && !strstr(log, "level one detail"), "level 1 line leaked into the quiet log");
    CHECK(log && count(log, "incident: hunt") == 1 && count(log, "incident: limiter") == 1, "log should hold one line per trigger");
    // find the incident file
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA("dztest\\incidents\\hunt_*.csv", &fd);
    CHECK(h != INVALID_HANDLE_VALUE, "no hunt file");
    if (h != INVALID_HANDLE_VALUE) {
        FindClose(h);
        char path[300]; snprintf(path, sizeof path, "dztest\\incidents\\%s", fd.cFileName);
        char *f = slurp(path, &n);
        CHECK(f && strstr(f, "level one detail BEFORE"), "level 1 line from before the trigger is missing");
        CHECK(f && !strstr(f, "level two detail"), "level 2 line should not be in the ring");
        CHECK(f && strstr(f, "gears 8->9->8->9"), "trigger text missing");
        CHECK(f && strstr(f, "after line one"), "line after the trigger missing");
        CHECK(f && strstr(f, "rpm 1900"), "second trigger did not join");
        CHECK(f && strstr(f, "# end (quiet again)"), "file did not end");
        const char *hdr = f ? strstr(f, "t,kmh,rpm") : NULL;
        int rows = 0, neg = 0;
        if (hdr) for (const char *p = strchr(hdr, '\n') + 1; *p; p = strchr(p, '\n') ? strchr(p, '\n') + 1 : p + strlen(p)) {
            if (*p == '#') continue;
            rows++; if (*p == '-') neg++;
        }
        printf("file %s: %ld bytes, %d rows (%d before the trigger)\n", fd.cFileName, n, rows, neg);
        CHECK(neg >= 80 && neg <= 95, "expected ~90 rows before (9 s at 10 Hz), got %d", neg);
        CHECK(rows >= 200 && rows <= 270, "expected ~230-250 rows in all (8 s before, 2 s + 15 s after), got %d", rows);
    }
    // the cap: many incidents of one kind do not exceed the per-kind limit, and the folder stays near the cap
    for (int i = 0; i < 14; i++) { incident("lug", "n %d", i); tick(0.3); incident_stop(); }
    long total = 0; int files = 0;
    h = FindFirstFileA("dztest\\incidents\\*.csv", &fd);
    if (h != INVALID_HANDLE_VALUE) { do { total += fd.nFileSizeLow; files++; } while (FindNextFileA(h, &fd)); FindClose(h); }
    printf("after 14 more: %d files, %ld bytes\n", files, total);
    CHECK(total < 50 * 1024 + 20000, "folder %ld bytes is far over the 50 KB cap", total);
    h = FindFirstFileA("dztest\\incidents\\lug_*.csv", &fd);
    int lug = 0; if (h != INVALID_HANDLE_VALUE) { do lug++; while (FindNextFileA(h, &fd)); FindClose(h); }
    CHECK(lug <= 12, "per-kind limit exceeded: %d lug files", lug);
    printf(fails ? "INCIDENT TEST FAILED (%d)\n" : "incident test: ok\n", fails);
    return fails ? 1 : 0;
}
