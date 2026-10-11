#include <windows.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define OFF_INNER           0x10
#define OFF_GEAR            0x14b0
#define OFF_THROTTLE        0x68
#define OFF_ENGINE_HANDLE   0x1040
#define OFF_RPM_LIMIT       0x234
#define OFF_STEP_TABLE      0x48
#define OFF_STEP_COUNT      0x50
#define OFF_REV_GEARS       0x20

#define RPM_LIMIT_MIN       500.0f
#define RPM_LIMIT_MAX       20000.0f

#define PAT_WINDOW  "48 8B 46 10 80 B8 A8 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00"
#define PAT_BOUND   "48 8B 41 10 0F 57 C9 F3 0F 10 15 ?? ?? ?? ?? 48 8B 90 F8 01 00 00 " \
                    "F3 0F 10 82 B0 01 00 00 F3 0F 58 82 A4 01 00 00"
#define PAT_LOBOUND "40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8"
#define PAT_CHOOSER "4C 63 B9 B0 14 00 00"
#define PAT_GATE    "F3 0F 59 81 F4 02 00 00 41 0F 2F C0 72 ??"

#define BOUND_DISC_OFF  0x40
#define GATE_JB_OFF     12
#define MAX_WIN_SITES   16

typedef int    (*bound_fn)(void *self);
typedef float *(*window_fn)(void *self, float *out, unsigned int gear);
typedef void  *(*resolve_fn)(void *handle);

static uintptr_t  g_base;
static bound_fn   o_hi_e60, o_hi_ed0, o_lo_e70;
static window_fn  o_window;
static resolve_fn o_resolve;

static struct {
    int   enabled;
    int   up_step;
    int   down_step;
    int   kick_step;
    float thr_gain;
    float thr_peak;
    float min_rpm_hi;
    float thr_lo;
    float thr_hi;
    float thr_curve;
    int   dn_on_accel;
    float up_delay;
    float dn_delay;
    int   status_file;
} cfg = { 1, 1, 2, 0, 3.00f, 0.90f, 0.65f, 0.45f, 0.85f, 1.6f, 1, 0.80f, 0.00f, 0 };

static char g_dir[MAX_PATH];

static void status(const char *fmt, ...)
{
    char path[MAX_PATH * 2];
    va_list ap;
    FILE *f;
    if (!cfg.status_file) return;
    snprintf(path, sizeof path, "%scar_automatic_transmission.txt", g_dir);
    f = fopen(path, "w");
    if (!f) return;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fputc(10, f);
    fclose(f);
}

static const unsigned char *g_text;
static size_t               g_text_size;
static uintptr_t            g_text_va;

typedef struct { uint32_t begin, end, unwind; } RTF;
static const RTF *g_rtf;
static size_t     g_rtf_count;

static int pe_init(void)
{
    const unsigned char *b = (const unsigned char *)g_base;
    const IMAGE_DOS_HEADER *dos = (const IMAGE_DOS_HEADER *)b;
    const IMAGE_NT_HEADERS64 *nt;
    const IMAGE_SECTION_HEADER *sec;
    const IMAGE_DATA_DIRECTORY *dir;
    unsigned i;

    if (!b || dos->e_magic != IMAGE_DOS_SIGNATURE) return 0;
    nt = (const IMAGE_NT_HEADERS64 *)(b + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return 0;
    if (nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC) return 0;

    sec = (const IMAGE_SECTION_HEADER *)((const unsigned char *)&nt->OptionalHeader +
                                         nt->FileHeader.SizeOfOptionalHeader);
    for (i = 0; i < nt->FileHeader.NumberOfSections; i++) {
        if (!memcmp(sec[i].Name, ".text", 5)) {
            g_text      = b + sec[i].VirtualAddress;
            g_text_size = sec[i].Misc.VirtualSize;
            g_text_va   = g_base + sec[i].VirtualAddress;
            break;
        }
    }
    dir = &nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXCEPTION];
    if (!dir->VirtualAddress || dir->Size < sizeof(RTF)) return 0;
    g_rtf       = (const RTF *)(b + dir->VirtualAddress);
    g_rtf_count = dir->Size / sizeof(RTF);
    return g_text && g_text_size && g_rtf_count;
}

static int pat_parse(const char *p, unsigned char *bytes, unsigned char *mask, int cap)
{
    int n = 0;
    while (*p && n < cap) {
        while (*p == ' ') p++;
        if (!*p) break;
        if (p[0] == '?' && p[1] == '?') { bytes[n] = 0; mask[n] = 0; p += 2; }
        else {
            int hi = (p[0] <= '9') ? p[0] - '0' : (p[0] | 32) - 'a' + 10;
            int lo = (p[1] <= '9') ? p[1] - '0' : (p[1] | 32) - 'a' + 10;
            bytes[n] = (unsigned char)((hi << 4) | lo);
            mask[n] = 1;
            p += 2;
        }
        n++;
    }
    return n;
}

static int pat_scan(const char *pattern, uintptr_t *hits, int max_hits)
{
    unsigned char bytes[80], mask[80];
    int len = pat_parse(pattern, bytes, mask, 80);
    int found = 0;
    const unsigned char *p = g_text;
    size_t left = g_text_size;

    if (len < 4 || !mask[0]) return -1;
    for (;;) {
        const unsigned char *m = (const unsigned char *)memchr(p, bytes[0], left);
        size_t off;
        int i, ok;
        if (!m) break;
        off = (size_t)(m - g_text);
        if (off + (size_t)len > g_text_size) break;
        ok = 1;
        for (i = 1; i < len; i++)
            if (mask[i] && m[i] != bytes[i]) { ok = 0; break; }
        if (ok) {
            if (found >= max_hits) return -1;
            hits[found++] = g_text_va + off;
        }
        p    = m + 1;
        left = g_text_size - (size_t)(p - g_text);
        if (!left) break;
    }
    return found;
}

static int rtf_index_of(uintptr_t va)
{
    uint32_t rva = (uint32_t)(va - g_base);
    int lo = 0, hi = (int)g_rtf_count - 1;
    while (lo <= hi) {
        int m = lo + (hi - lo) / 2;
        if (rva < g_rtf[m].begin)     hi = m - 1;
        else if (rva >= g_rtf[m].end) lo = m + 1;
        else return m;
    }
    return -1;
}

static int rtf_primary(int i)
{
    int guard;
    for (guard = 0; guard < 16; guard++) {
        const unsigned char *u = (const unsigned char *)(g_base + g_rtf[i].unwind);
        unsigned codes, off;
        uint32_t chained;
        int j;
        if (!((u[0] >> 3) & 4)) return i;
        codes = u[2];
        off   = 4 + ((codes + 1) / 2) * 4;
        memcpy(&chained, u + off, 4);
        j = rtf_index_of(g_base + chained);
        if (j < 0 || j == i) return i;
        i = j;
    }
    return i;
}

static int func_extent(uintptr_t va, uintptr_t *lo_out, uintptr_t *hi_out)
{
    int i = rtf_index_of(va), p, lo, hi;
    if (i < 0) return 0;
    p  = rtf_primary(i);
    lo = i;
    while (lo > 0 && rtf_primary(lo - 1) == p) lo--;
    hi = i;
    while (hi + 1 < (int)g_rtf_count && rtf_primary(hi + 1) == p) hi++;
    *lo_out = g_base + g_rtf[lo].begin;
    *hi_out = g_base + g_rtf[hi].end;
    return 1;
}

static int calls_to(uintptr_t target, uintptr_t lo, uintptr_t hi,
                    uintptr_t *out, int max_out)
{
    const unsigned char *p = (const unsigned char *)lo;
    const unsigned char *end;
    int found = 0;
    if (hi <= lo + 5) return 0;
    end = (const unsigned char *)(hi - 5);
    for (; p <= end; p++) {
        int32_t rel;
        if (*p != 0xE8) continue;
        memcpy(&rel, p + 1, 4);
        if ((uintptr_t)(p + 5) + (intptr_t)rel != target) continue;
        if (found >= max_out) return -1;
        out[found++] = (uintptr_t)p;
    }
    return found;
}

static uintptr_t first_call_in(uintptr_t lo, uintptr_t hi)
{
    const unsigned char *p = (const unsigned char *)lo;
    const unsigned char *end;
    if (hi <= lo + 5) return 0;
    end = (const unsigned char *)(hi - 5);
    for (; p <= end; p++) {
        int32_t rel;
        if (*p != 0xE8) continue;
        memcpy(&rel, p + 1, 4);
        return (uintptr_t)(p + 5) + (intptr_t)rel;
    }
    return 0;
}

static uintptr_t s_hi_kick, s_lo_kick, s_hi_shared, s_lo_down, s_hi_up;
static uintptr_t s_window[MAX_WIN_SITES];
static int       s_window_count;
static uintptr_t s_gate;

static const char *discover(void)
{
    uintptr_t hits[8], lo, hi, ced0[4], ce70[4], ce60[4];
    uintptr_t fn_window, fn_resolve, fn_e60 = 0, fn_ed0 = 0, fn_e70;
    int n, i;

    if (!pe_init()) return "PE header";

    n = pat_scan(PAT_WINDOW, hits, 8);
    if (n != 1) return "window pattern";
    if (!func_extent(hits[0], &lo, &hi)) return "window extent";
    fn_window  = lo;
    fn_resolve = first_call_in(lo, hi);
    if (!fn_resolve || fn_resolve < g_text_va || fn_resolve >= g_text_va + g_text_size)
        return "resolver";

    n = pat_scan(PAT_BOUND, hits, 8);
    if (n != 2) return "bound pattern";
    for (i = 0; i < 2; i++) {
        const unsigned char *d = (const unsigned char *)(hits[i] + BOUND_DISC_OFF);
        if (d[0] == 0x8B && d[1] == 0x41 && d[2] == 0x1C) fn_e60 = hits[i];
        if (d[0] == 0x8B && d[1] == 0x41 && d[2] == 0x20) fn_ed0 = hits[i];
    }
    if (!fn_e60 || !fn_ed0) return "bound discriminator";

    n = pat_scan(PAT_LOBOUND, hits, 8);
    if (n != 1) return "lower bound pattern";
    fn_e70 = hits[0];

    n = pat_scan(PAT_CHOOSER, hits, 8);
    if (n != 1) return "chooser pattern";
    if (!func_extent(hits[0], &lo, &hi)) return "chooser extent";

    if (calls_to(fn_ed0, lo, hi, ced0, 4) != 2) return "kick/up call sites";
    if (calls_to(fn_e70, lo, hi, ce70, 4) != 2) return "kick/down call sites";
    if (calls_to(fn_e60, lo, hi, ce60, 4) != 1) return "shared call site";
    s_hi_kick   = ced0[0];
    s_hi_up     = ced0[1];
    s_lo_kick   = ce70[0];
    s_lo_down   = ce70[1];
    s_hi_shared = ce60[0];

    s_window_count = calls_to(fn_window, g_text_va, g_text_va + g_text_size,
                              s_window, MAX_WIN_SITES);
    if (s_window_count < 1) return "window call sites";

    n = pat_scan(PAT_GATE, hits, 8);
    if (n != 1) return "gate pattern";
    s_gate = hits[0] + GATE_JB_OFF;
    if (*(const unsigned char *)s_gate != 0x72) return "gate opcode";

    o_hi_e60  = (bound_fn)fn_e60;
    o_hi_ed0  = (bound_fn)fn_ed0;
    o_lo_e70  = (bound_fn)fn_e70;
    o_window  = (window_fn)fn_window;
    o_resolve = (resolve_fn)fn_resolve;
    return NULL;
}

static int cur_abs_gear(void *self)
{
    char *inner;
    int gi;
    if (!self) return -1;
    inner = *(char **)((char *)self + OFF_INNER);
    if (!inner) return -1;
    gi = *(int32_t *)(inner + OFF_GEAR);
    if (gi < -64 || gi > 64) return -1;
    return gi < 0 ? -gi : gi;
}

static int clamp_hi(void *self, int orig, int step)
{
    int cur, lim;
    if (step <= 0) return orig;
    cur = cur_abs_gear(self);
    if (cur < 0) return orig;
    lim = cur + step;
    if (lim < 1) lim = 1;
    return orig < lim ? orig : lim;
}

static int clamp_lo(void *self, int orig, int step)
{
    int cur, lim;
    if (step <= 0) return orig;
    cur = cur_abs_gear(self);
    if (cur < 0) return orig;
    lim = cur - step;
    if (lim < 1) lim = 1;
    return orig > lim ? orig : lim;
}

#define NSLOT 16
static struct { void *self; int gear; ULONGLONG t; } g_slot[NSLOT];

static ULONGLONG since_shift(void *self)
{
    ULONGLONG now = GetTickCount64();
    int cur = cur_abs_gear(self);
    int i, free_i = -1;
    if (cur < 0) return (ULONGLONG)-1;
    for (i = 0; i < NSLOT; i++) {
        if (g_slot[i].self == self) {
            if (g_slot[i].gear != cur) { g_slot[i].gear = cur; g_slot[i].t = now; return 0; }
            return now - g_slot[i].t;
        }
        if (g_slot[i].self == NULL && free_i < 0) free_i = i;
    }
    if (free_i < 0) free_i = (int)((uintptr_t)self >> 5) & (NSLOT - 1);
    g_slot[free_i].self = self;
    g_slot[free_i].gear = cur;
    g_slot[free_i].t    = now;
    return 0;
}

static int shift_locked(void *self, float delay_s)
{
    ULONGLONG dt = since_shift(self);
    if (delay_s <= 0.0f) return 0;
    if (dt == (ULONGLONG)-1) return 0;
    return dt < (ULONGLONG)(delay_s * 1000.0f);
}

static int hold_gear(void *self, int orig)
{
    int cur = cur_abs_gear(self);
    if (cur < 1) return orig;
    return cur;
}

static int stub_hi_kick(void *s)
{
    return clamp_hi(s, o_hi_ed0(s), cfg.kick_step);
}

static int stub_lo_kick(void *s)
{
    return clamp_lo(s, o_lo_e70(s), cfg.kick_step);
}

static int stub_hi_shared(void *s)
{
    int o = o_hi_e60(s);
    if (shift_locked(s, cfg.up_delay)) return hold_gear(s, o);
    return clamp_hi(s, o, cfg.up_step);
}

static int stub_lo_down(void *s)
{
    int o = o_lo_e70(s);
    if (shift_locked(s, cfg.dn_delay)) return hold_gear(s, o);
    return clamp_lo(s, o, cfg.down_step);
}

static int stub_hi_up(void *s)
{
    int o = o_hi_ed0(s);
    if (shift_locked(s, cfg.up_delay)) return hold_gear(s, o);
    return clamp_hi(s, o, cfg.up_step);
}

static float throttle_of(void *self)
{
    float t;
    if (!self) return 0.0f;
    t = *(float *)((char *)self + OFF_THROTTLE);
    if (!(t >= 0.0f)) return 0.0f;
    return t > 1.0f ? 1.0f : t;
}

static float pedal_ramp(void *self)
{
    float t = throttle_of(self);
    float f;
    if (cfg.thr_hi <= cfg.thr_lo) return 0.0f;
    f = (t - cfg.thr_lo) / (cfg.thr_hi - cfg.thr_lo);
    if (f <= 0.0f) return 0.0f;
    if (f > 1.0f) f = 1.0f;
    if (cfg.thr_curve != 1.0f) f = powf(f, cfg.thr_curve);
    return f;
}

static float gear_step(void *self, unsigned int gear)
{
    char *base = (char *)self;
    unsigned long long n;
    long long idx;
    float *arr;
    if (!self) return 0.0f;
    n   = *(unsigned long long *)(base + OFF_STEP_COUNT);
    arr = *(float **)(base + OFF_STEP_TABLE);
    idx = (long long)(*(int32_t *)(base + OFF_REV_GEARS)) + (int)gear;
    if (!arr || n > 256 || idx < 0 || (unsigned long long)idx >= n) return 0.0f;
    return arr[idx];
}

static float rpm_limit_of(void *self)
{
    char *inner, *eng;
    void *handle;
    float lim;
    if (!o_resolve || !self) return 0.0f;
    inner = *(char **)((char *)self + OFF_INNER);
    if (!inner) return 0.0f;
    handle = *(void **)(inner + OFF_ENGINE_HANDLE);
    if (!handle) return 0.0f;
    eng = (char *)o_resolve(handle);
    if (!eng) return 0.0f;
    lim = *(float *)(eng + OFF_RPM_LIMIT);
    if (!(lim > RPM_LIMIT_MIN && lim < RPM_LIMIT_MAX)) return 0.0f;
    return lim;
}

static float *stub_window(void *self, float *out, unsigned int gear)
{
    float f, k, lim, ceil_, need, mn, mx, mn0, mx0;

    o_window(self, out, gear);
    since_shift(self);
    if (!out) return out;
    if (!(out[0] > 0.0f) || !(out[1] > out[0])) return out;

    f = pedal_ramp(self);
    if (f <= 0.0f || cfg.thr_gain <= 0.0f) return out;

    mn0 = out[0];
    mx0 = out[1];

    lim   = rpm_limit_of(self);
    ceil_ = (lim > 0.0f) ? lim * 0.99f : 0.0f;

    if (lim > 0.0f) {
        float demand = lim * cfg.min_rpm_hi;
        if (demand < mn0) demand = mn0;
        mn = mn0 + f * (demand - mn0);
    } else {
        mn = mn0 * (1.0f + f * cfg.thr_gain);
    }
    k = mn / mn0;
    if (k > 1.0f + cfg.thr_gain) { k = 1.0f + cfg.thr_gain; mn = mn0 * k; }
    if (k <= 1.0f) return out;

    mx = mx0 * k;
    if (ceil_ > 0.0f && mx > lim * cfg.thr_peak) mx = lim * cfg.thr_peak;
    if (ceil_ > 0.0f && mx > ceil_) mx = ceil_;

    need = gear_step(self, gear);
    need = (need > 1.01f) ? need + 0.05f : 1.15f;
    if (mn > mx / need) mn = mx / need;
    if (mn < mn0) mn = mn0;
    if (mx <= mn) { out[0] = mn0; out[1] = mx0; return out; }

    out[0] = mn;
    out[1] = mx;
    return out;
}

static unsigned char *g_tramp;
static size_t         g_tramp_used;

static unsigned char *alloc_near(uintptr_t target)
{
    SYSTEM_INFO si;
    uintptr_t gran, d;
    GetSystemInfo(&si);
    gran = si.dwAllocationGranularity;
    for (d = gran; d < 0x60000000u; d += gran) {
        void *p;
        uintptr_t a = (target - d) & ~(uintptr_t)(gran - 1);
        p = VirtualAlloc((void *)a, 0x1000, MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE);
        if (p) return (unsigned char *)p;
        a = (target + d) & ~(uintptr_t)(gran - 1);
        p = VirtualAlloc((void *)a, 0x1000, MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE);
        if (p) return (unsigned char *)p;
    }
    return NULL;
}

static unsigned char *make_tramp(void *fn)
{
    unsigned char *p = g_tramp + g_tramp_used;
    p[0] = 0x48; p[1] = 0xB8;
    memcpy(p + 2, &fn, 8);
    p[10] = 0xFF; p[11] = 0xE0;
    g_tramp_used += 16;
    return p;
}

static int write_mem(void *dst, const void *src, size_t n)
{
    DWORD old;
    if (!VirtualProtect(dst, n, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy(dst, src, n);
    VirtualProtect(dst, n, old, &old);
    FlushInstructionCache(GetCurrentProcess(), dst, n);
    return 1;
}

static int patch_call(uintptr_t site, void *stub)
{
    unsigned char *tr = make_tramp(stub);
    int64_t rel64 = (int64_t)(uintptr_t)tr - (int64_t)(site + 5);
    int32_t rel;
    if (rel64 < (int64_t)INT32_MIN || rel64 > (int64_t)INT32_MAX) return 0;
    rel = (int32_t)rel64;
    return write_mem((void *)(site + 1), &rel, 4);
}

static void load_cfg(void)
{
    char path[MAX_PATH * 2];
    char line[256];
    FILE *f;
    snprintf(path, sizeof path, "%scar_automatic_transmission.cfg", g_dir);
    f = fopen(path, "r");
    if (!f) {
        f = fopen(path, "w");
        if (f) {
            fprintf(f, "# car_automatic_transmission\n");
            fprintf(f, "enabled=1\n");
            fprintf(f, "\n");
            fprintf(f, "# Maximum number of gears crossed per shift.\n");
            fprintf(f, "# 1 = strictly sequential, 0 = unlimited (stock behaviour).\n");
            fprintf(f, "up_step=1\n");
            fprintf(f, "down_step=2\n");
            fprintf(f, "# Kickdown may drop as far as needed.\n");
            fprintf(f, "kick_step=0\n");
            fprintf(f, "\n");
            fprintf(f, "# Load-dependent shift map. The rpm window moves up with the\n");
            fprintf(f, "# throttle, so the gearbox downshifts earlier at part throttle\n");
            fprintf(f, "# and holds gears longer.\n");
            fprintf(f, "# Main control: minimum rpm demanded at full map, as a\n");
            fprintf(f, "# fraction of the rev limit.\n");
            fprintf(f, "min_rpm_hi=0.65\n");
            fprintf(f, "# Pedal range over which the map fades in, and its curvature.\n");
            fprintf(f, "thr_lo=0.45\n");
            fprintf(f, "thr_hi=0.85\n");
            fprintf(f, "thr_curve=1.6\n");
            fprintf(f, "# Top of the window at full map, as a fraction of the rev limit.\n");
            fprintf(f, "thr_peak=0.90\n");
            fprintf(f, "# Safety ceiling on the window shift. 0 disables the shift map.\n");
            fprintf(f, "thr_gain=3.00\n");
            fprintf(f, "\n");
            fprintf(f, "# Stock blocks every downshift while the vehicle accelerates and\n");
            fprintf(f, "# is not braking. 1 = lift that block, 0 = stock behaviour.\n");
            fprintf(f, "downshift_on_accel=1\n");
            fprintf(f, "\n");
            fprintf(f, "# Lockout between two consecutive shifts, in seconds (0 = off).\n");
            fprintf(f, "upshift_delay=0.80\n");
            fprintf(f, "downshift_delay=0.00\n");
            fprintf(f, "\n");
            fprintf(f, "# Write one status line to car_automatic_transmission.txt.\n");
            fprintf(f, "status_file=0\n");
            fclose(f);
        }
        return;
    }
    while (fgets(line, sizeof line, f)) {
        char *e;
        int v;
        double d;
        if (line[0] == '#' || line[0] == 10 || line[0] == 13) continue;
        e = strchr(line, '=');
        if (!e) continue;
        *e = 0;
        v = atoi(e + 1);
        d = atof(e + 1);
        if      (!strcmp(line, "enabled"))    cfg.enabled     = v;
        else if (!strcmp(line, "up_step"))    cfg.up_step     = v;
        else if (!strcmp(line, "down_step"))  cfg.down_step   = v;
        else if (!strcmp(line, "kick_step"))  cfg.kick_step   = v;
        else if (!strcmp(line, "thr_gain"))   cfg.thr_gain    = (float)d;
        else if (!strcmp(line, "thr_peak"))   cfg.thr_peak    = (float)d;
        else if (!strcmp(line, "min_rpm_hi")) cfg.min_rpm_hi  = (float)d;
        else if (!strcmp(line, "thr_lo"))     cfg.thr_lo      = (float)d;
        else if (!strcmp(line, "thr_hi"))     cfg.thr_hi      = (float)d;
        else if (!strcmp(line, "thr_curve"))  cfg.thr_curve   = (float)d;
        else if (!strcmp(line, "downshift_on_accel")) cfg.dn_on_accel = v;
        else if (!strcmp(line, "upshift_delay"))      cfg.up_delay    = (float)d;
        else if (!strcmp(line, "downshift_delay"))    cfg.dn_delay    = (float)d;
        else if (!strcmp(line, "status_file"))        cfg.status_file = v;
    }
    fclose(f);
}

static void init_paths(HMODULE self)
{
    char *slash;
    GetModuleFileNameA(self, g_dir, sizeof g_dir);
    slash = strrchr(g_dir, '\\');
    if (slash) slash[1] = 0; else g_dir[0] = 0;
}

static void do_patch(void)
{
    const char *err;
    int i, done = 0;

    g_base = (uintptr_t)GetModuleHandleW(NULL);
    if (!cfg.enabled) { status("disabled by config"); return; }

    err = discover();
    if (err) { status("not applied: %s not found", err); return; }

    g_tramp = alloc_near(s_hi_kick);
    if (!g_tramp) { status("not applied: no memory in range"); return; }

    done += patch_call(s_hi_kick,   (void *)stub_hi_kick);
    done += patch_call(s_lo_kick,   (void *)stub_lo_kick);
    done += patch_call(s_hi_shared, (void *)stub_hi_shared);
    done += patch_call(s_lo_down,   (void *)stub_lo_down);
    done += patch_call(s_hi_up,     (void *)stub_hi_up);

    if (cfg.thr_gain > 0.0f)
        for (i = 0; i < s_window_count; i++)
            done += patch_call(s_window[i], (void *)stub_window);

    if (cfg.dn_on_accel) {
        static const unsigned char jmp_op = 0xEB;
        done += write_mem((void *)s_gate, &jmp_op, 1);
    }
    status("applied, %d patches, %d window call sites", done, s_window_count);
}

__declspec(dllexport) int __cdecl scs_telemetry_init(unsigned int version, const void *params)
{
    (void)version; (void)params;
    do_patch();
    return 0;
}

__declspec(dllexport) void __cdecl scs_telemetry_shutdown(void)
{
}

BOOL WINAPI DllMain(HINSTANCE inst, DWORD reason, LPVOID reserved)
{
    (void)reserved;
    if (reason == DLL_PROCESS_ATTACH) {
        HMODULE dummy;
        DisableThreadLibraryCalls(inst);
        init_paths(inst);
        load_cfg();
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN |
                           GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
                           (LPCWSTR)(void *)&DllMain, &dummy);
    }
    return TRUE;
}
