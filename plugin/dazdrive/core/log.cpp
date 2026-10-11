// Log file and clocks.
#include "suite.h"
#include <stdarg.h>
#include <string.h>

char g_dir[MAX_PATH];
int g_log_level = 2;       // until the ini is read; the compiled default is the developer's (2)
double g_log_max_mb = 5;   // dazdrive.log rotates to dazdrive.log.1 beyond this
static size_t g_log_bytes;
static FILE *g_log;
static CRITICAL_SECTION g_log_lock;
static ULONGLONG g_t0;

double wall_s() { return (GetTickCount64() - g_t0) / 1000.0; }

double sim_s() { return T.sim_time ? T.sim_time / 1e6 : wall_s(); }

FILE *open_out(const char *name, const char *mode)
{
    // csv traces only at log_level 2 (modules treat NULL as "no trace")
    size_t n = strlen(name);
    if (g_log_level < 2 && n > 4 && !strcmp(name + n - 4, ".csv")) return NULL;
    char path[MAX_PATH * 2];
    snprintf(path, sizeof path, "%s%s", g_dir, name);
    return fopen(path, mode);
}

FILE *open_incident(const char *dir, const char *prefix, double max_mb, char *stamp, size_t n)
{
    char d[MAX_PATH * 2], f[MAX_PATH * 3];
    snprintf(d, sizeof d, "%s%s", g_dir, dir);
    CreateDirectoryA(d, NULL);
    for (int guard = 0; guard < 200; guard++) {  // oldest out until there is room for a new one
        WIN32_FIND_DATAA fd;
        snprintf(f, sizeof f, "%s\\*.csv", d);
        HANDLE h = FindFirstFileA(f, &fd);
        if (h == INVALID_HANDLE_VALUE) break;
        ULONGLONG total = 0;
        FILETIME oldest = { 0xffffffff, 0xffffffff };
        char old[MAX_PATH] = "";
        do {
            total += ((ULONGLONG)fd.nFileSizeHigh << 32) | fd.nFileSizeLow;
            if (CompareFileTime(&fd.ftLastWriteTime, &oldest) < 0) { oldest = fd.ftLastWriteTime; snprintf(old, sizeof old, "%s", fd.cFileName); }
        } while (FindNextFileA(h, &fd));
        FindClose(h);
        if (total < (ULONGLONG)(max_mb * 0.9 * 1048576.0) || !old[0]) break;
        snprintf(f, sizeof f, "%s\\%s", d, old);
        if (!DeleteFileA(f)) break;
    }
    SYSTEMTIME t;
    GetLocalTime(&t);
    snprintf(stamp, n, "%04d-%02d-%02d_%02d-%02d-%02d", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    snprintf(f, sizeof f, "%s\\%s_%04d-%02d-%02d_%02d-%02d-%02d.csv", d, prefix, t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    return fopen(f, "w");
}

double g_csv_max_mb = 5;
void csv_cap(FILE **f, const char *name)
{
    if (!f || !*f || g_csv_max_mb <= 0) return;
    long size = ftell(*f);
    if (size < 0 || size < (long)(g_csv_max_mb * 1048576.0)) return;
    char a[MAX_PATH * 2], b[MAX_PATH * 2], head[4096] = "";
    snprintf(a, sizeof a, "%s%s", g_dir, name);
    snprintf(b, sizeof b, "%s%s.1", g_dir, name);
    fclose(*f);
    FILE *r = fopen(a, "r");
    if (r) { if (!fgets(head, sizeof head, r)) head[0] = 0; fclose(r); }
    MoveFileExA(a, b, MOVEFILE_REPLACE_EXISTING);
    *f = fopen(a, "w");
    if (*f && head[0]) fputs(head, *f);
    logline("%s reached %.1f MB: the earlier part is in %s.1", name, size / 1048576.0, name);
}

void log_open()
{
    if (g_log) return;
    g_t0 = GetTickCount64();
    InitializeCriticalSection(&g_log_lock);
    CreateDirectoryA(g_dir, NULL);
    g_log = open_out(SUITE_NAME ".log", "w");
}

void logline(const char *fmt, ...)
{
    if (!g_log) return;
    EnterCriticalSection(&g_log_lock);
    // Cap: past log_max_mb the log moves to dazdrive.log.1 (one generation) and starts again. A failing disk
    // (read-only folder, full) only costs the log lines, never a module.
    if (g_log_bytes > (size_t)(g_log_max_mb * 1048576.0)) {
        fclose(g_log);
        char a[MAX_PATH * 2], b[MAX_PATH * 2];
        snprintf(a, sizeof a, "%s%s", g_dir, SUITE_NAME ".log");
        snprintf(b, sizeof b, "%s%s", g_dir, SUITE_NAME ".log.1");
        MoveFileExA(a, b, MOVEFILE_REPLACE_EXISTING);
        g_log = fopen(a, "w");
        g_log_bytes = 0;
        if (!g_log) { LeaveCriticalSection(&g_log_lock); return; }
        fprintf(g_log, "(log rotated: the earlier part is in " SUITE_NAME ".log.1)\n");
    }
    int n = fprintf(g_log, "%8.2f  ", wall_s());
    va_list ap;
    va_start(ap, fmt);
    int m = vfprintf(g_log, fmt, ap);
    va_end(ap);
    fputc('\n', g_log);
    fflush(g_log);
    g_log_bytes += (n > 0 ? n : 0) + (m > 0 ? m : 0) + 1;
    LeaveCriticalSection(&g_log_lock);
}
