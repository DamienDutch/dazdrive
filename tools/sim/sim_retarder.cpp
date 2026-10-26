// Offline test of the retarder hook (core/actuate.cpp, core/game.cpp): the merge of the owners' factors into g_ret_scale, the
// stub's bytes run natively (game.h retarder_stub_code: what game_patch_retarder puts next to the drivetrain), the hook pattern,
// and that only the player's vehicle gets the factor (another vehicle's inner, or the player's not found: the demand as it is).
// The actuator is included, so its statics are reachable. Build+run: tools/sim/run_retarder.sh
#include "core/suite.h"
#include "core/game.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#if defined(__x86_64__) && defined(__linux__)
#include <sys/mman.h>
#define NATIVE_STUB 1
#endif

telemetry T;
const layout *L;
game_addrs G;
void logline(const char *, ...) {}
int g_log_level = 2;
void logring(const char *, ...) {}
void incident(const char *, const char *, ...) {}
double wall_s() { return 0; }
static int g_mem_ok;   // memory counts as readable (the player's vehicle can be found through the sim's game_actor)
int mem_readable(const void *p, size_t) { return g_mem_ok && p; }
char *game_actor_safe() { return NULL; }
void rsc_step(float) {}
// the patch helpers of the game: a stub in this sim (the real ones patch the exe's memory), so the hook stays "not in"
uintptr_t game_scan(const char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
int game_patch_clutch_floor(uintptr_t, clutch_floor *) { return 0; }
int game_patch_retarder(uintptr_t, const retarder_scale *) { return 0; }
#include "core/actuate.cpp"

static float stock_cruise(void *, float, float pedal) { return pedal; }
static int bad;
static void check(int ok, const char *what) { if (!ok) { printf("   FAIL: %s\n", what); bad = 1; } }
static unsigned bits(float f) { unsigned u; memcpy(&u, &f, 4); return u; }
static float from_bits(unsigned u) { float f; memcpy(&f, &u, 4); return f; }

// ---- the merge
static void merge_test()
{
    printf("--- retarder factor merge\n");
    char GA[0x400] = {};
    float tick;
    o_cruise = stock_cruise;
    act_stub(GA, 1 / 60.0f, 0);
    printf("no owner posting -> g_ret_scale %.2f (1.00)\n", g_ret_scale);
    check(bits(g_ret_scale) == bits(1.0f), "no post = exactly 1.0");
    act_retarder_scale(ACT_RET_NORM, 0.8f);
    act_stub(GA, 1 / 60.0f, 0);
    printf("one owner posts 0.80 -> %.2f\n", g_ret_scale);
    check(g_ret_scale == 0.8f, "one owner");
    act_retarder_scale(ACT_RET_DECEL, 0.5f);
    act_stub(GA, 1 / 60.0f, 0); tick = g_ret_scale;
    printf("second owner posts 0.50 -> %.2f (the lowest)\n", tick);
    check(tick == 0.5f, "minimum over owners");
    act_retarder_scale(ACT_RET_DECEL, 0.9f);
    act_stub(GA, 1 / 60.0f, 0);
    printf("second owner raises to 0.90 -> %.2f (0.80: held until the owner changes it, the other one is now lowest)\n", g_ret_scale);
    check(g_ret_scale == 0.8f, "held per owner");
    act_retarder_scale(ACT_RET_DECEL, 1.7f); act_retarder_scale(ACT_RET_NORM, -3);
    act_stub(GA, 1 / 60.0f, 0);
    printf("out-of-range posts (1.7, -3) -> %.2f (clamped to 0..1: lowest 0.00)\n", g_ret_scale);
    check(g_ret_scale == 0.0f, "clamp");
    act_retarder_scale(ACT_RET_NORM, NAN); act_retarder_scale(ACT_RET_DECEL, NAN);
    act_stub(GA, 1 / 60.0f, 0);
    check(bits(g_ret_scale) == bits(1.0f), "NaN post = no factor");
    act_retarder_scale(ACT_RET_NORM, 0.3f); act_retarder_scale(-1, 0.1f); act_retarder_scale(ACT_N, 0.1f);
    act_stub(GA, 1 / 60.0f, 0);
    check(g_ret_scale == 0.3f, "an unknown owner is ignored");
    act_retarder_scale(ACT_RET_NORM, 1.0f);
    act_stub(GA, 1 / 60.0f, 0);
    printf("posts withdrawn (1.00) -> %.2f (1.00)\n", g_ret_scale);
    check(bits(g_ret_scale) == bits(1.0f), "withdrawn = 1.0");
    printf("hook flag with the stubbed patch helper: %d (0)\n", (int)act_retarder_hooked());
    check(!act_retarder_hooked(), "not hooked with the sim's stub");
    g_retarder_hook = 1; act_start();
    check(!act_retarder_hooked(), "still not hooked after act_start with the stub (also without the 1.60 layout)");
    g_retarder_hook = 0;
}

// ---- the pattern in the source against the bytes of the static analysis (written out here, independently)
static int pat_find(const unsigned char *buf, size_t n, const char *pat, size_t *hit)
{
    unsigned char b[64]; bool w[64]; size_t m = 0;
    for (const char *p = pat; *p; ) {
        while (*p == ' ') p++;
        if (!*p) break;
        if (p[0] == '?') { b[m] = 0; w[m++] = true; p += 2; } else { b[m] = (unsigned char)strtoul(p, NULL, 16); w[m++] = false; p += 2; }
    }
    int hits = 0;
    for (size_t i = 0; i + m <= n; i++) {
        size_t j = 0;
        while (j < m && (w[j] || buf[i + j] == b[j])) j++;
        if (j == m) { if (!hits) *hit = i; hits++; }
    }
    return hits;
}

static void pattern_test()
{
    printf("--- retarder hook pattern\n");
    static const unsigned char want[] = { 0x48, 0x8B, 0xBF, 0xD0, 0x04, 0x00, 0x00,                  // mov rdi,[rdi+0x4d0]
                                          0xF3, 0x0F, 0x10, 0x80, 0x44, 0x14, 0x00, 0x00,            // movss xmm0,[rax+0x1444]
                                          0xE8, 0x11, 0x22, 0x33, 0x44,                              // call (any)
                                          0xF3, 0x44, 0x0F, 0x10, 0x87, 0xA8, 0x00, 0x00, 0x00 };    // movss xmm8,[rdi+0xa8]
    unsigned char img[512];
    for (size_t i = 0; i < sizeof img; i++) img[i] = (unsigned char)(i * 37 + 11);   // filler without the pattern
    memcpy(img + 200, want, sizeof want);
    size_t at = 0;
    int n = pat_find(img, sizeof img, k_ret_pat, &at);
    printf("pattern: %d hit in the synthetic image at %zu (1 at 200), demand read at +%d\n", n, at, (int)RET_READ_AT);
    check(n == 1 && at == 200, "the pattern matches the documented bytes exactly once");
    unsigned char odd[sizeof want]; memcpy(odd, want, sizeof want); odd[9] = 0x11;   // another instruction: movups
    memcpy(img + 300, odd, sizeof odd);
    check(pat_find(img, sizeof img, k_ret_pat, &at) == 1, "a near miss does not match");
    // the read instruction at the offset is what the stub starts with (and what game_patch_retarder insists on)
    unsigned char code[64]; static retarder_scale one = { NULL, 1.0f };
    size_t len = retarder_stub_code(code, &one);
    check(len < 64 && !memcmp(code, want + RET_READ_AT, 8), "the stub begins with the original 8-byte read");
    check(!memcmp(img + 200 + RET_READ_AT, code, 8), "the read sits at +7 of the pattern");
    printf("stub: %zu bytes, begins with the original read\n", len);
}

#ifdef NATIVE_STUB
#include <xmmintrin.h>
// Run the stub's bytes as the game would: the patched site is `call stub; nop; nop; nop` (then a ret back to the harness), rax = inner,
// every other register, xmm1-15 and the flags carry a pattern. run_site loads them, calls the site and records what comes back.
struct site_ctx {                 // offsets used by run_site below
    void *site;                   // 0
    void *inner;                  // 8
    unsigned long long flags_in;  // 16
    unsigned long long rsp_before, rsp_after;   // 24, 32
    unsigned long long flags_out; // 40
    unsigned long long gpr[16];   // 48: rax, rcx, rdx, rbx, rsp (unused), rbp, rsi, rdi, r8 .. r15 as numbered by the CPU
    unsigned char xin[16][16];    // 176
    unsigned char xout[16][16];   // 432
};
extern "C" void run_site(site_ctx *c);
asm(R"(
.text
.globl run_site
run_site:
    push %rbp
    push %rbx
    push %r12
    push %r13
    push %r14
    push %r15
    sub $40, %rsp
    mov %rdi, 8(%rsp)
    mov 0(%rdi), %rax
    mov %rax, 0(%rsp)
    mov %rsp, 24(%rdi)
    movups 176(%rdi), %xmm0
    movups 192(%rdi), %xmm1
    movups 208(%rdi), %xmm2
    movups 224(%rdi), %xmm3
    movups 240(%rdi), %xmm4
    movups 256(%rdi), %xmm5
    movups 272(%rdi), %xmm6
    movups 288(%rdi), %xmm7
    movups 304(%rdi), %xmm8
    movups 320(%rdi), %xmm9
    movups 336(%rdi), %xmm10
    movups 352(%rdi), %xmm11
    movups 368(%rdi), %xmm12
    movups 384(%rdi), %xmm13
    movups 400(%rdi), %xmm14
    movups 416(%rdi), %xmm15
    pushq 16(%rdi)
    popfq
    mov 8(%rdi), %rax
    movabs $0x1111111111111111, %rcx
    movabs $0x2222222222222222, %rdx
    movabs $0x3333333333333333, %rbx
    movabs $0x5555555555555555, %rbp
    movabs $0x6666666666666666, %rsi
    movabs $0x7777777777777777, %rdi
    movabs $0x8888888888888888, %r8
    movabs $0x9999999999999999, %r9
    movabs $0xAAAAAAAAAAAAAAAA, %r10
    movabs $0xBBBBBBBBBBBBBBBB, %r11
    movabs $0xCCCCCCCCCCCCCCCC, %r12
    movabs $0xDDDDDDDDDDDDDDDD, %r13
    movabs $0xEEEEEEEEEEEEEEEE, %r14
    movabs $0xFFFFFFFFFFFFFFF0, %r15
    call *0(%rsp)
    pushfq
    popq 16(%rsp)
    mov %rax, 24(%rsp)
    mov 8(%rsp), %rax
    mov %rcx, 56(%rax)
    mov %rdx, 64(%rax)
    mov %rbx, 72(%rax)
    mov %rbp, 88(%rax)
    mov %rsi, 96(%rax)
    mov %rdi, 104(%rax)
    mov %r8, 112(%rax)
    mov %r9, 120(%rax)
    mov %r10, 128(%rax)
    mov %r11, 136(%rax)
    mov %r12, 144(%rax)
    mov %r13, 152(%rax)
    mov %r14, 160(%rax)
    mov %r15, 168(%rax)
    mov 24(%rsp), %rcx
    mov %rcx, 48(%rax)
    mov 16(%rsp), %rcx
    mov %rcx, 40(%rax)
    mov %rsp, 32(%rax)
    movups %xmm0, 432(%rax)
    movups %xmm1, 448(%rax)
    movups %xmm2, 464(%rax)
    movups %xmm3, 480(%rax)
    movups %xmm4, 496(%rax)
    movups %xmm5, 512(%rax)
    movups %xmm6, 528(%rax)
    movups %xmm7, 544(%rax)
    movups %xmm8, 560(%rax)
    movups %xmm9, 576(%rax)
    movups %xmm10, 592(%rax)
    movups %xmm11, 608(%rax)
    movups %xmm12, 624(%rax)
    movups %xmm13, 640(%rax)
    movups %xmm14, 656(%rax)
    movups %xmm15, 672(%rax)
    add $40, %rsp
    pop %r15
    pop %r14
    pop %r13
    pop %r12
    pop %rbx
    pop %rbp
    ret
)");
static_assert(offsetof(site_ctx, flags_in) == 16 && offsetof(site_ctx, rsp_before) == 24 && offsetof(site_ctx, flags_out) == 40 &&
              offsetof(site_ctx, gpr) == 48 && offsetof(site_ctx, xin) == 176 && offsetof(site_ctx, xout) == 432, "run_site offsets");

static const unsigned long long k_fill[16] = { 0, 0x1111111111111111ULL, 0x2222222222222222ULL, 0x3333333333333333ULL, 0, 0x5555555555555555ULL,
    0x6666666666666666ULL, 0x7777777777777777ULL, 0x8888888888888888ULL, 0x9999999999999999ULL, 0xAAAAAAAAAAAAAAAAULL, 0xBBBBBBBBBBBBBBBBULL,
    0xCCCCCCCCCCCCCCCCULL, 0xDDDDDDDDDDDDDDDDULL, 0xEEEEEEEEEEEEEEEEULL, 0xFFFFFFFFFFFFFFF0ULL };

static char g_inner[0x1500];
static unsigned char *g_page;
// one call of the patched site; returns the xmm0 float, sets *clean when nothing but xmm0 changed
static float run_patched(float demand, unsigned flags, int *clean, char *inner = g_inner)
{
    *(float *)(inner + 0x1444) = demand;
    site_ctx c = {};
    c.site = g_page; c.inner = inner; c.flags_in = flags;
    for (int i = 0; i < 16; i++) for (int j = 0; j < 16; j++) c.xin[i][j] = (unsigned char)(0xA0 + i * 3 + j);
    run_site(&c);
    int ok = (void *)c.gpr[0] == inner && c.rsp_before == c.rsp_after;                // rax = inner, the stack as it was
    for (int i = 1; i < 16; i++) if (i != 4 && c.gpr[i] != k_fill[i]) ok = 0;            // every other register
    for (int i = 1; i < 16; i++) if (memcmp(c.xin[i], c.xout[i], 16)) ok = 0;            // xmm1-15 whole
    static const unsigned char zero12[12] = {};
    if (memcmp(c.xout[0] + 4, zero12, 12)) ok = 0;                                       // xmm0's upper lanes: movss from memory zeroes them, the original too
    const unsigned long long st = 0x8D5;                                                 // CF PF AF ZF SF OF
    if ((c.flags_out & st) != (flags & st)) ok = 0;
    *clean = ok;
    float r; memcpy(&r, c.xout[0], 4);
    return r;
}

static void stub_test()
{
    printf("--- the stub's bytes, run natively\n");
    g_page = (unsigned char *)mmap(NULL, 4096, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (g_page == MAP_FAILED) { printf("stub: no executable memory here, not run\n"); return; }
    unsigned char *stub = g_page + 64;
    size_t n = retarder_stub_code(stub, &s_rs);
    (void)n;
    int32_t rel = (int32_t)(stub - (g_page + 5));
    g_page[0] = 0xE8; memcpy(g_page + 1, &rel, 4);        // the patched site: call stub
    g_page[5] = g_page[6] = g_page[7] = 0x90;             // + 3 nops
    g_page[8] = 0xC3;                                     // back to the harness
    const float demand[] = { 0.0f, 0.25f, 0.5f, 1.0f, from_bits(0x00000001), from_bits(0x007FFFFF), 0.8f, -0.0f, from_bits(0x7FA00001) /* signaling NaN */ };
    const unsigned flags[] = { 0x202, 0x8D7, 0x282, 0x246, 0x203 };
    int ident = 0, tot = 0, clean_all = 1;
    // the player's vehicle is found through game_actor -> gearbox -> inner (offsets of a stand-in layout): the runs below are the
    // player's, through the scale compare
    char GA[0x400] = {};
    static layout lay; lay.name = "sim"; lay.a_gearbox = 0x40; lay.s_inner = 0x10;
    static char gbox[0x40];
    *(char **)(gbox + lay.s_inner) = g_inner; *(char **)(GA + lay.a_gearbox) = gbox;
    L = &lay; g_mem_ok = 1;
    act_stub(GA, 1 / 60.0f, 0);
    check(s_rs.inner == g_inner && bits(s_rs.scale) == bits(1.0f), "the player's vehicle found, factor exactly 1.0");
    for (float d : demand) for (unsigned f : flags) {
        int cl; float r = run_patched(d, f, &cl);
        tot++; ident += bits(r) == bits(d); clean_all &= cl;
    }
    printf("factor 1.0: %d of %d runs return the demand bit for bit (0, 0.25, 0.5, 1, 2 denormals, 0.8, -0, sNaN x 5 flag patterns)\n", ident, tot);
    check(ident == tot, "factor 1.0 returns the identical float");
    printf("factor 1.0: rax, 14 other registers, xmm1-15, the flags and the stack as the game expects: %s\n", clean_all ? "yes" : "NO");
    check(clean_all, "state preserved at factor 1.0");
    unsigned csr = _mm_getcsr();
    _mm_setcsr(csr | 0x8040);                              // FTZ + DAZ, the mode a plain mulss by 1.0 would change a denormal under
    int cl; float r = run_patched(from_bits(0x00000001), 0x202, &cl);
    _mm_setcsr(csr);
    printf("factor 1.0 with DAZ on: denormal 0x00000001 -> 0x%08x (0x00000001)\n", bits(r));
    check(bits(r) == 0x00000001 && cl, "factor 1.0 bit-identical also with DAZ");
    // a post through the actuator: owner -> merge -> g_ret_scale -> the stub
    act_retarder_scale(ACT_RET_NORM, 0.5f); act_stub(GA, 1 / 60.0f, 0);
    int halved = 0, tot2 = 0; clean_all = 1;
    for (float d : demand) {
        if (d != d) continue;
        r = run_patched(d, 0x246, &cl); clean_all &= cl;
        tot2++; halved += bits(r) == bits(d * 0.5f);
    }
    printf("owner posts 0.50: %d of %d demands (0, 0.25, 0.5, 1, denormals, 0.8, -0) come back halved, state preserved: %s\n", halved, tot2, clean_all ? "yes" : "NO");
    check(halved == tot2 && clean_all, "factor 0.5 halves the demand");
    r = run_patched(0.8f, 0x246, &cl);
    act_retarder_scale(ACT_RET_NORM, 1.0f); act_stub(GA, 1 / 60.0f, 0);
    float back = run_patched(0.8f, 0x246, &cl);
    printf("post withdrawn: 0.8 -> %.2f (post 0.5: %.2f)\n", back, r);
    check(bits(back) == bits(0.8f), "withdrawn post returns the demand");
    // the drivetrain runs the stub for other vehicles too (AI trucks): only the player's inner gets the factor
    static char other[0x1500];
    act_retarder_scale(ACT_RET_NORM, 0.5f); act_stub(GA, 1 / 60.0f, 0);
    int same = 0, mine = 0; tot2 = 0; clean_all = 1;
    for (float d : demand) for (unsigned f : flags) {
        if (d != d) continue;
        r = run_patched(d, f, &cl, other); clean_all &= cl;
        tot2++; same += bits(r) == bits(d);
        mine += bits(run_patched(d, f, &cl)) == bits(d * 0.5f); clean_all &= cl;
    }
    printf("owner posts 0.50, another vehicle: %d of %d demands come back as they are (the player's: %d halved), state preserved: %s\n", same, tot2, mine, clean_all ? "yes" : "NO");
    check(same == tot2 && mine == tot2 && clean_all, "a vehicle that is not the player's gets factor 1");
    // the player's vehicle not found (gearbox pointer unreadable, then no layout): factor 1 for every vehicle, the merge itself stands
    g_mem_ok = 0; act_stub(GA, 1 / 60.0f, 0);
    float lost = run_patched(0.8f, 0x246, &cl), lost_o = run_patched(0.8f, 0x246, &cl, other);
    g_mem_ok = 1; L = NULL; act_stub(GA, 1 / 60.0f, 0);
    float nolay = run_patched(0.8f, 0x246, &cl);
    printf("player's vehicle not found: 0.8 -> %.2f, another vehicle %.2f, no layout %.2f (0.80; g_ret_scale %.2f)\n", lost, lost_o, nolay, g_ret_scale);
    check(bits(lost) == bits(0.8f) && bits(lost_o) == bits(0.8f) && bits(nolay) == bits(0.8f) && g_ret_scale == 0.5f, "no player vehicle = factor 1");
    L = &lay;
    // the brake writer alone (the sims' tick) merges too; a reset clears every factor
    act_retarder_scale(ACT_RET_DECEL, 0.4f); act_brake_write(GA);
    float bw = run_patched(0.8f, 0x246, &cl);
    act_reset(RESET_SOFT, "sim");
    float rs = run_patched(0.8f, 0x246, &cl);
    act_stub(GA, 1 / 60.0f, 0);
    float rs2 = run_patched(0.8f, 0x246, &cl);
    printf("brake writer alone, post 0.40: 0.8 -> %.2f (0.32); after a reset %.2f, next tick %.2f (0.80, 0.80)\n", bw, rs, rs2);
    check(bits(bw) == bits(0.8f * 0.4f), "the brake writer merges the factors");
    check(bits(rs) == bits(0.8f) && bits(rs2) == bits(0.8f) && bits(g_ret_scale) == bits(1.0f), "a reset clears the factors");
    L = NULL; g_mem_ok = 0;
}
#endif

int main()
{
    merge_test();
    pattern_test();
#ifdef NATIVE_STUB
    stub_test();
#else
    printf("--- the stub's bytes are run on x86-64 Linux only: skipped\n");
#endif
    printf("retarder sim: %s\n", bad ? "FAIL" : "ok");
    return bad;
}
