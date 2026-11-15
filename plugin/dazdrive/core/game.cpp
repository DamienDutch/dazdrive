// Game discovery and access. Pattern scanning, .pdata function extents and the
// call-site trampolines follow car_automatic_transmission (MIT, Deppelover).
#include "game.h"
#include "vehicle.h"   // V.electric: the brake pedal hook falls back to the raw pedal on an electric truck
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------- layouts
// 1.60: ETS2 1.60.1.7 (2026-10). Offsets verified live with the probe plugin
// (runs 1 and 2) and in the disassembly of the automatic's update (0x14098c640),
// its gear chooser (0x14098d0d0) and start-gear fn (0x14098dbf0).
// ATS 1.61.3.18 (game_id "ats") is the same prism3d 1.61 build family as ETS2 1.61.1.9: every object offset and
// pattern is identical (notes/port-ats161.md). Only the game_actor's action objects (the cruise/lane/gearbox-mode
// keys) sit +0x100 higher than in 1.60, which the modules handle through game_is_161().

static const layout g_layouts[] = {
    { "1.60", "eut2",
      "48 8B 45 10 80 B8 40 14 00 00 00 74 ?? F2 0F 10 83 3C 02 00 00",
      "4C 63 B9 48 14 00 00",
      "40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
      "48 8B 05 ?? ?? ?? ?? B2 01 48 8B 88 ?? ?? ?? ?? E8",
      "89 46 68",
      "80 BE 19 01 00 00 00",
      "44 8B 80 48 14 00 00",
      "F3 41 0F 59 BE C0 01 00 00",
      "48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 41 0F 2F C0",
      0x10, 0x18, 0x68, 0x119,
      0x1448, 0x1424, 0x1428, 0xff0, 0xfe8,
      0x1c0, 0x1d0, 0x1d8, 0x22c, 0x23c,
      0x1c0,
      0x199,
      0x258, 0x150, 0x358,
      0x1440, 0x143c, 0x260, 0x264, 0x378,
      0x1d68,
      "48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54",
      "49 8B E3 41 5E 5F 5E 5B E9 ?? ?? ?? ??",
      "F3 0F 59 3D ?? ?? ?? ?? 45 85 F6 0F 84",
      0x1b8,
      0x150, 0x154, 0x17c, 0x180, 0x190, 0x198, 0x199, 0x19c, 0x1a0, 0x1a8,
      0x1d0, 0x1d1, 0x1d5c, 0x340, 0x1e0, 0x1d00, 0x1cc0, 0x1cc4, 0x142c, 0x1444,
      0x1408, 0x1420, 0x640, 0x100, 0x101, 0x110, 0x228, 8,
      "48 8B CB F3 44 0F 11 83 E4 1C 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8",
      "85 C0 74 17 F6 87 D0 01 00 00 10 75 0E 44 0F 2F 8F 7C 01 00 00 73 04 B2 01",
      "0F 2F CF 76 ?? 83 BB 50 01 00 00 02 75 ?? 0F 2F BB 7C 01 00 00 72 0A F3 0F 11 81 24 14",
      "48 8B 03 F3 41 0F 5F FE 48 8B CB 0F 28 CF FF 90 78 01 00 00",
      "E8 ?? ?? ?? ?? 48 8B 0B 44 0F 28 F0 48 8B 01 FF 90 88 00 00 00",
      "F3 0F 10 BE 7C 01 00 00 0F 2F FE",
      "48 8B 01 F3 41 0F 5F FE 0F 28 CF FF 90 78 01 00 00",
      "48 8B 43 18 83 A0 A0 02 00 00 DF 80 BB 54 01 00 00 00",
      "83 88 A0 02 00 00 20 EB 0B 48 8B 43 18 83 A0 A0 02 00 00 DF",
      "E8 ?? ?? ?? ?? 0F 2F C6 77 4C 0F 2F B3 60 01 00 00",
      "8B 00 89 81 44 14 00 00 48 8D 45 7F",
      "48 8B 4E 18 41 0F 2F C2",
      "48 8D 4C 24 38 48 0F 46 C8" },
    // 1.61 (ats): ATS 1.61.3.18 (2026-10), re-derived offline from amtrucks.exe (notes/port-ats161.md). Identical to
    // the ETS2 1.61 entry below (same build family): the gearbox self grew a float at +0x68, the vehicle inner +0x78
    // at the handles / +0x90 through the drivetrain, the engine def +8/+0x10, the game_actor +0x68/+0x78/+0xcc.
    { "1.61", "ats",
      "48 8B 46 10 80 B8 D0 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00",
      "4C 63 B9 D8 14 00 00",
      "40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
      "48 8B 05 ?? ?? ?? ?? B2 01 48 8B 88 ?? ?? ?? ?? E8",
      "89 46 70", "80 BE 21 01 00 00 00", "44 8B 80 D8 14 00 00",
      "F3 0F 59 97 C8 01 00 00",
      "48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 41 0F 2F C0",
      0x10, 0x18, 0x70, 0x121,
      0x14d8, 0x14b4, 0x14b8, 0x1068, 0x1060,
      0x1c8, 0x1d8, 0x1e0, 0x234, 0x244,
      0x1c8,
      0x204,
      0x270, 0x150, 0x358,
      0x14d0, 0x14cc, 0x270, 0x274, 0x3f0,
      0x1e30,
      "48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54",
      "49 8B E3 41 5E 5F 5E E9 ?? ?? ?? ??",
      "F3 0F 59 3D ?? ?? ?? ?? E8",
      0x1c0,
      0x1b8, 0x1bc, 0x1e4, 0x1e8, 0x1f8, 0x200, 0x204, 0x208, 0x20c, 0x214,
      0x248, 0x249, 0x1e24, 0x3b8, 0x258, 0x1dcc, 0x1d8c, 0x1d90, 0x14bc, 0x14d4,
      0x1498, 0x14b0, 0x6b8, 0x108, 0x109, 0x118, 0x230, 7,
      "48 8B CB F3 44 0F 11 83 B0 1D 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8",
      "85 C0 74 17 F6 87 48 02 00 00 10 75 0E 44 0F 2F 8F E4 01 00 00 73 04 B2 01",
      "0F 2F CF 76 ?? 83 BB B8 01 00 00 02 75 ?? 0F 2F BB E4 01 00 00 72 0A F3 0F 11 81 B4 14 00 00",
      "F3 0F 10 8E E4 01 00 00 48 8B 01 F3 0F 5F CE FF 90 78 01 00 00",
      "", "",
      "",
      "48 8B 43 18 83 A0 A0 02 00 00 DF 80 BB BC 01 00 00 00",
      "83 88 A0 02 00 00 20 EB 0B 48 8B 43 18 83 A0 A0 02 00 00 DF",
      "",
      "8B 00 89 81 D4 14 00 00 48 8D 45 7F",
      "48 8B 4E 18 41 0F 2F C2",
      "48 8D 4C 24 38 48 0F 46 C8",
      // engine brake lever (core/ebrakelogic.h): verified on both 1.61 exes (eurotrucks2.exe, amtrucks.exe)
      0x1cd, 0x1ce, 0x1d0, 0x1e1, 0x1e9b0, 0x1e9c0,
      "E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 ?? 48 8B 47 18 48 8B 88 68 10 00 00 E8 ?? ?? ?? ?? 8B 97 D0 01 00 00 3B 90 74 02 00 00 73 ?? FF C2 48 8B CF E8 ?? ?? ?? ??",
      "E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 ?? 48 8D 97 98 E9 01 00 E8 ?? ?? ?? ?? 8B 97 D0 01 00 00 84 C0 41 8B CD 88 87 E1 01 00 00 0F 95 C1 3B D1 76 ?? FF CA 48 8B CF E8 ?? ?? ?? ??" },
    // 1.61 (eut2): ETS2 1.61.1.9 (2026-10), ported offline 31-10 (notes/port-161.md). The gearbox self object grew a
    // float at +0x68 (target 0x70, force 0x121) like ATS 1.61; the vehicle inner grew +0x78 at the handles and +0x90
    // from the steering through the drivetrain; the engine def +8 (+0x10 at the engine-brake fields); the game_actor
    // grew in three blocks (+0x68 below 0x199, +0x78 at the lights, +0xcc at the pedals/hill). Every offset and pattern
    // re-derived from the 1.61 disassembly; the brake-pedal getter/hill self-checks are skipped (empty patterns: the
    // service-brake max moved registers, so only the request read anchors the site). t_shift_time is inferred (+8).
    { "1.61", "eut2",
      "48 8B 46 10 80 B8 D0 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00",
      "4C 63 B9 D8 14 00 00",
      "40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
      "48 8B 05 ?? ?? ?? ?? B2 01 48 8B 88 ?? ?? ?? ?? E8",
      "89 46 70", "80 BE 21 01 00 00 00", "44 8B 80 D8 14 00 00",
      "F3 0F 59 97 C8 01 00 00",
      "48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 41 0F 2F C0",
      0x10, 0x18, 0x70, 0x121,
      0x14d8, 0x14b4, 0x14b8, 0x1068, 0x1060,
      0x1c8, 0x1d8, 0x1e0, 0x234, 0x244,
      0x1c8,
      0x204,
      0x270, 0x150, 0x358,
      0x14d0, 0x14cc, 0x270, 0x274, 0x3f0,
      0x1e30,
      "48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54",
      "49 8B E3 41 5E 5F 5E E9 ?? ?? ?? ??",
      "F3 0F 59 3D ?? ?? ?? ?? E8",
      0x1c0,
      0x1b8, 0x1bc, 0x1e4, 0x1e8, 0x1f8, 0x200, 0x204, 0x208, 0x20c, 0x214,
      0x248, 0x249, 0x1e24, 0x3b8, 0x258, 0x1dcc, 0x1d8c, 0x1d90, 0x14bc, 0x14d4,
      0x1498, 0x14b0, 0x6b8, 0x108, 0x109, 0x118, 0x230, 7,
      "48 8B CB F3 44 0F 11 83 B0 1D 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8",
      "85 C0 74 17 F6 87 48 02 00 00 10 75 0E 44 0F 2F 8F E4 01 00 00 73 04 B2 01",
      "0F 2F CF 76 ?? 83 BB B8 01 00 00 02 75 ?? 0F 2F BB E4 01 00 00 72 0A F3 0F 11 81 B4 14 00 00",
      "F3 0F 10 8E E4 01 00 00 48 8B 01 F3 0F 5F CE FF 90 78 01 00 00",
      "", "",
      "",
      "48 8B 43 18 83 A0 A0 02 00 00 DF 80 BB BC 01 00 00 00",
      "83 88 A0 02 00 00 20 EB 0B 48 8B 43 18 83 A0 A0 02 00 00 DF",
      "",
      "8B 00 89 81 D4 14 00 00 48 8D 45 7F",
      "48 8B 4E 18 41 0F 2F C2",
      "48 8D 4C 24 38 48 0F 46 C8",
      // engine brake lever (core/ebrakelogic.h): verified on both 1.61 exes (eurotrucks2.exe, amtrucks.exe)
      0x1cd, 0x1ce, 0x1d0, 0x1e1, 0x1e9b0, 0x1e9c0,
      "E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 ?? 48 8B 47 18 48 8B 88 68 10 00 00 E8 ?? ?? ?? ?? 8B 97 D0 01 00 00 3B 90 74 02 00 00 73 ?? FF C2 48 8B CF E8 ?? ?? ?? ??",
      "E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 ?? 48 8D 97 98 E9 01 00 E8 ?? ?? ?? ?? 8B 97 D0 01 00 00 84 C0 41 8B CD 88 87 E1 01 00 00 0F 95 C1 3B D1 76 ?? FF CA 48 8B CF E8 ?? ?? ?? ??" },
    // 1.62: ETS2 1.62.1.0 / ATS 1.62.1.1 (2026-11), ported offline 06-11 (notes/port-162.md). Same build family in both games.
    // SCS rewrote the automatic: the gearbox self lost 0xc after +0x10c (drive state 0x10c with 1 = at rest after reverse /
    // 3 = at rest after forward, so no latch byte; force 0x115; a 1.5 s hold timer at 0x118), the start gear runs through a
    // wrapper (start fn, then the lower bound, max). The vehicle inner -0x28 from the handles on (gear 0x14b0), the drive +8
    // (engine 0x368, clutch copy 0x338), the engine and transmission defs unchanged. game_actor: unchanged up to 0x218, +4 at
    // the lights / blink (0x24c, 0x25c; the flash float stays 0x3b8), +8 from the pedals on (0x1d94, hill 0x1dd4, lka u 0x1e2c,
    // automatic 0x1e38: 0x1e30 is now the manual box), the action objects +8, the engine-brake fraction 0x3f8. The brake-pedal
    // hook is off (xmm14 at the 1.61 site is no longer the raw pedal), the engage step too (the generic gearbox changed).
#define LAYOUT_162(GAME) \
    { "1.62", GAME, \
      "48 8B 46 10 80 B8 A8 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00", \
      "4C 63 A9 B0 14 00 00", \
      "40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8", \
      "48 8B 05 ?? ?? ?? ?? B2 01 44 0F 29 8C 24 90 00 00 00 44 0F 29 94 24 80 00 00 00 48 8B 88 ?? ?? ?? ?? E8", \
      "89 46 70", "80 BE 15 01 00 00 00", "44 8B 80 B0 14 00 00", \
      "F3 44 0F 59 97 C8 01 00 00", \
      "48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 41 0F 2F C3", \
      0x10, 0x18, 0x70, 0x115, \
      0x14b0, 0x148c, 0x1490, 0x1040, 0x1038, \
      0x1c8, 0x1d8, 0x1e0, 0x234, 0x244, \
      0x1c8, \
      0x204, \
      0x270, 0x150, 0x358, \
      0x14a8, 0x14a4, 0x270, 0x274, 0x3f8, \
      0x1e38, \
      "48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54", \
      "49 8B E3 5F 5E 5D E9 ?? ?? ?? ??", \
      "F3 0F 59 3D ?? ?? ?? ?? E8", \
      0x1c0, \
      0x1b8, 0x1bc, 0x1e4, 0x1e8, 0x1f8, 0x200, 0x204, 0x208, 0x20c, 0x214, \
      0x24c, 0x24d, 0x1e2c, 0x3b8, 0x25c, 0x1dd4, 0x1d94, 0x1d98, 0x1494, 0x14ac, \
      0x1470, 0x1488, 0x6b8, 0x108, 0, 0x10c, 0x230, 6, \
      "48 8B CB F3 44 0F 11 83 B8 1D 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8", \
      "85 C0 74 17 F6 87 4C 02 00 00 10 75 0E 44 0F 2F 8F E4 01 00 00 73 04 B2 01", \
      "0F 2F CF 76 ?? 83 BB B8 01 00 00 02 75 ?? 0F 2F BB E4 01 00 00 72 0A F3 0F 11 81 8C 14 00 00", \
      "", "", "", "", \
      "48 8B 43 18 83 A0 A0 02 00 00 DF 80 BB BC 01 00 00 00", \
      "83 88 A0 02 00 00 20 EB 0B 48 8B 43 18 83 A0 A0 02 00 00 DF", \
      "", \
      "8B 00 89 81 AC 14 00 00 48 8D 45 7F", \
      "48 8B 4E 18 41 0F 2F C2", \
      "48 8D 4C 24 38 48 0F 46 C8", \
      0x1cd, 0x1ce, 0x1d0, 0x1e1, 0x1e9b8, 0x1e9c8, \
      "E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 ?? 48 8B 47 18 48 8B 88 40 10 00 00 E8 ?? ?? ?? ?? 8B 97 D0 01 00 00 3B 90 74 02 00 00 73 ?? FF C2 48 8B CF E8 ?? ?? ?? ??", \
      "E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 ?? 48 8D 97 A0 E9 01 00 E8 ?? ?? ?? ?? 8B 97 D0 01 00 00 84 C0 41 8B CF 88 87 E1 01 00 00 0F 95 C1 3B D1 76 ?? FF CA 48 8B CF E8 ?? ?? ?? ??" }
    LAYOUT_162("eut2"),
    LAYOUT_162("ats"),
};
#define N_LAYOUTS (int)(sizeof g_layouts / sizeof g_layouts[0])

const layout *L;
game_addrs G;

// ---------------------------------------------------------------- PE helpers

static uintptr_t g_base;
static const unsigned char *g_text;
static size_t g_text_size;
static uintptr_t g_text_va;
struct RTF { uint32_t begin, end, unwind; };
static const RTF *g_rtf;
static size_t g_rtf_count;

static int pe_init()
{
    const unsigned char *b = (const unsigned char *)g_base;
    const IMAGE_DOS_HEADER *dos = (const IMAGE_DOS_HEADER *)b;
    if (!b || dos->e_magic != IMAGE_DOS_SIGNATURE) return 0;
    const IMAGE_NT_HEADERS64 *nt = (const IMAGE_NT_HEADERS64 *)(b + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE || nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC) return 0;
    const IMAGE_SECTION_HEADER *sec = IMAGE_FIRST_SECTION(nt);
    for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; i++) {
        if (!memcmp(sec[i].Name, ".text", 5)) {
            g_text = b + sec[i].VirtualAddress;
            g_text_size = sec[i].Misc.VirtualSize;
            g_text_va = g_base + sec[i].VirtualAddress;
            break;
        }
    }
    const IMAGE_DATA_DIRECTORY *dir = &nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXCEPTION];
    if (!dir->VirtualAddress || dir->Size < sizeof(RTF)) return 0;
    g_rtf = (const RTF *)(b + dir->VirtualAddress);
    g_rtf_count = dir->Size / sizeof(RTF);
    return g_text && g_text_size && g_rtf_count;
}

static int pat_parse(const char *p, unsigned char *bytes, unsigned char *mask, int cap)
{
    int n = 0;
    while (*p && n < cap) {
        while (*p == ' ') p++;
        if (!*p) break;
        if (p[0] == '?') { bytes[n] = 0; mask[n] = 0; }
        else {
            int hi = p[0] <= '9' ? p[0] - '0' : (p[0] | 32) - 'a' + 10;
            int lo = p[1] <= '9' ? p[1] - '0' : (p[1] | 32) - 'a' + 10;
            bytes[n] = (unsigned char)(hi << 4 | lo);
            mask[n] = 1;
        }
        p += 2;
        n++;
    }
    return n;
}

// Hits of a pattern in [lo, hi); -1 if more than max_hits.
static int pat_scan(const char *pattern, uintptr_t lo, uintptr_t hi, uintptr_t *hits, int max_hits)
{
    unsigned char bytes[96], mask[96];
    int len = pat_parse(pattern, bytes, mask, 96), found = 0;
    if (len < 3 || !mask[0]) return -1;
    for (uintptr_t a = lo; a + len <= hi; a++) {
        const unsigned char *m = (const unsigned char *)a;
        if (m[0] != bytes[0]) continue;
        int ok = 1;
        for (int i = 1; i < len; i++)
            if (mask[i] && m[i] != bytes[i]) { ok = 0; break; }
        if (!ok) continue;
        if (found >= max_hits) return -1;
        hits[found++] = a;
    }
    return found;
}

static int pat_at(const char *pattern, uintptr_t a)
{
    uintptr_t h;
    unsigned char bytes[96], mask[96];
    int len = pat_parse(pattern, bytes, mask, 96);
    return pat_scan(pattern, a, a + len, &h, 1) == 1;
}

static int rtf_index_of(uintptr_t va)
{
    uint32_t rva = (uint32_t)(va - g_base);
    int lo = 0, hi = (int)g_rtf_count - 1;
    while (lo <= hi) {
        int m = lo + (hi - lo) / 2;
        if (rva < g_rtf[m].begin) hi = m - 1;
        else if (rva >= g_rtf[m].end) lo = m + 1;
        else return m;
    }
    return -1;
}

static int rtf_primary(int i)
{
    for (int guard = 0; guard < 16; guard++) {
        const unsigned char *u = (const unsigned char *)(g_base + g_rtf[i].unwind);
        if (!((u[0] >> 3) & 4)) return i;
        unsigned off = 4 + ((u[2] + 1) / 2) * 4;
        uint32_t chained;
        memcpy(&chained, u + off, 4);
        int j = rtf_index_of(g_base + chained);
        if (j < 0 || j == i) return i;
        i = j;
    }
    return i;
}

static int func_extent(uintptr_t va, uintptr_t *lo_out, uintptr_t *hi_out)
{
    int i = rtf_index_of(va);
    if (i < 0) return 0;
    int p = rtf_primary(i), lo = i, hi = i;
    while (lo > 0 && rtf_primary(lo - 1) == p) lo--;
    while (hi + 1 < (int)g_rtf_count && rtf_primary(hi + 1) == p) hi++;
    *lo_out = g_base + g_rtf[lo].begin;
    *hi_out = g_base + g_rtf[hi].end;
    return 1;
}

static uintptr_t call_target(uintptr_t site)
{
    int32_t rel;
    memcpy(&rel, (const void *)(site + 1), 4);
    return site + 5 + (intptr_t)rel;
}

static int calls_to(uintptr_t target, uintptr_t lo, uintptr_t hi, uintptr_t *out, int max_out)
{
    int found = 0;
    for (uintptr_t a = lo; a + 5 <= hi; a++) {
        if (*(const unsigned char *)a != 0xE8 || call_target(a) != target) continue;
        if (found >= max_out) return -1;
        out[found++] = a;
    }
    return found;
}

static uintptr_t first_call_in(uintptr_t lo, uintptr_t hi)
{
    for (uintptr_t a = lo; a + 5 <= hi; a++)
        if (*(const unsigned char *)a == 0xE8) return call_target(a);
    return 0;
}

static int in_text(uintptr_t a) { return a >= g_text_va && a < g_text_va + g_text_size; }

// ---------------------------------------------------------------- discovery

static const char *discover_with()
{
    uintptr_t hits[8], lo, hi, sites[8];
    uintptr_t tlo = g_text_va, thi = g_text_va + g_text_size;
    memset(&G, 0, sizeof G);

    // shift window fn and the handle resolver (its first call)
    if (pat_scan(L->pat_window, tlo, thi, hits, 8) != 1) return "window pattern";
    if (!func_extent(hits[0], &lo, &hi)) return "window extent";
    G.window_fn = lo;
    G.resolve_fn = first_call_in(lo, hi);
    if (!in_text(G.resolve_fn)) return "resolver";

    // gear chooser and its two callers, both in the automatic's update
    if (pat_scan(L->pat_chooser, tlo, thi, hits, 8) != 1) return "chooser pattern";
    uintptr_t clo, chi;
    if (!func_extent(hits[0], &clo, &chi)) return "chooser extent";
    G.chooser_fn = clo;
    {
        uintptr_t c[4];
        if (pat_scan(L->chk_torque, clo, chi, c, 4) != 1) return "torque check";
    }
    G.n_choose = calls_to(G.chooser_fn, tlo, thi, G.choose_sites, 4);
    if (G.n_choose != 2) return "chooser call sites (another plugin hooked them?)";
    if (!func_extent(G.choose_sites[0], &G.update_lo, &G.update_hi)) return "update extent";
    for (int i = 0; i < G.n_choose; i++)
        if (G.choose_sites[i] < G.update_lo || G.choose_sites[i] >= G.update_hi) return "chooser callers differ";
    if (!pat_at(L->chk_target, G.choose_sites[0] + 5) && !pat_at(L->chk_target, G.choose_sites[1] + 5))
        return "target gear check";
    if (pat_scan(L->chk_force, G.update_lo, G.update_hi, hits, 8) < 1) return "force flag check";

    // the window call in the update: where the automatic decides
    if (calls_to(G.window_fn, G.update_lo, G.update_hi, sites, 8) != 1) return "decide site";
    G.decide_site = sites[0];
    if (!pat_at(L->chk_decide, G.decide_site - 7)) return "decide site check";

    // lowest-gear fn; its first call is the grade fn. In the update it is
    // called right after the start-gear fn (mov rcx,rsi; mov ebx,eax).
    if (pat_scan(L->pat_lobound, tlo, thi, hits, 8) != 1) return "lower bound pattern";
    G.lobound_fn = hits[0];
    if (*(const unsigned char *)(hits[0] + 18) != 0xE8) return "grade fn call";
    G.grade_fn = call_target(hits[0] + 18);
    if (!in_text(G.grade_fn)) return "grade fn";
    int n;
    const char *start_chk;
    if (game_is_162()) {
        // 1.62: one wrapper (call start; mov rcx,rbx; mov edi,eax; call lower bound; max) that the update calls; the chooser
        // and the manual-mode fn call the lower bound on their own (no start call before them)
        uintptr_t all[8];
        int na = calls_to(G.lobound_fn, tlo, thi, all, 8);
        n = 0;
        for (int i = 0; i < na; i++)
            if (*(const unsigned char *)(all[i] - 10) == 0xE8 && pat_at("48 8B CB 8B F8", all[i] - 5) && n < 8) sites[n++] = all[i];
        if (n != 1) return "start-gear wrapper";
        uintptr_t wlo, whi, u[2];
        if (!func_extent(sites[0], &wlo, &whi) || calls_to(wlo, G.update_lo, G.update_hi, u, 2) != 1) return "start-gear wrapper call in update";
        start_chk = "48 8B CB 8B F8";
    } else {
        n = calls_to(G.lobound_fn, G.update_lo, G.update_hi, sites, 8);
        if (n != 2) return "lower bound calls in update";
        start_chk = "48 8B CE 8B D8";
    }
    for (int i = 0; i < n; i++) {
        uintptr_t s = sites[i] - 10;
        if (*(const unsigned char *)s != 0xE8 || !pat_at(start_chk, sites[i] - 5)) return "start-gear call";
        uintptr_t t = call_target(s);
        if (G.start_fn && t != G.start_fn) return "start-gear fns differ";
        G.start_fn = t;
    }
    G.n_start = calls_to(G.start_fn, tlo, thi, G.start_sites, 8);
    if (G.n_start < 2) return "start-gear call sites";

    // mass: mov rax,[rip+slot]; mov dl,1; mov rcx,[rax+game_actor]; call mass
    if (pat_scan(L->pat_mass, G.update_lo, G.update_hi, hits, 8) != 1) return "mass pattern";
    int32_t rel;
    memcpy(&rel, (const void *)(hits[0] + 3), 4);
    G.base_ctrl_slot = hits[0] + 7 + (intptr_t)rel;
    {   // the pattern ends in mov rcx,[rax+game_actor]; call mass (1.62: two movaps sit between mov dl,1 and the load)
        int n = 0;
        for (const char *q = L->pat_mass; *q; q++) if (*q != ' ' && (q == L->pat_mass || q[-1] == ' ')) n++;
        memcpy(&G.game_actor_off, (const void *)(hits[0] + n - 5), 4);
        G.mass_fn = call_target(hits[0] + n - 1);
    }
    if (!in_text(G.mass_fn) || G.game_actor_off > 0x10000) return "mass fn";

    // optional: the adaptive-shift cvar and its reader
    if (pat_scan(L->pat_adaptive, G.update_lo, G.update_hi, hits, 8) == 1) {
        memcpy(&rel, (const void *)(hits[0] + 3), 4);
        G.cvar_adaptive = hits[0] + 7 + (intptr_t)rel;
        G.cvar_read_fn = call_target(hits[0] + 7);
        if (!in_text(G.cvar_read_fn)) G.cvar_adaptive = G.cvar_read_fn = 0;
    }

    // optional: the update's tail jump into the generic gearbox (rcx = self),
    // the last point before the gearbox acts on self+0x68 (lka-roll-re notes B)
    if (pat_scan(L->pat_tail, G.update_lo, G.update_hi, hits, 8) == 1 && *(const unsigned char *)(hits[0] + L->tail_jmp) == 0xE9)
        G.tail_site = hits[0] + L->tail_jmp;

    // optional: the stop threshold (mulss xmm7,[0.8] = 0.8 x idle) in the update (lka-roll-re notes C)
    if (pat_scan(L->pat_stop, G.update_lo, G.update_hi, hits, 8) == 1) G.stop_site = hits[0];

    // optional: gps_manager offset in game_ctrl (route look-ahead), all hits must agree
    {
        uintptr_t g[4];
        int ng = pat_scan(L->pat_gps, tlo, thi, g, 4);
        unsigned off = 0;
        for (int i = 0; i < ng; i++) {
            unsigned o;
            memcpy(&o, (const void *)(g[i] + 3), 4);
            if (i && o != off) { off = 0; break; }
            off = o;
        }
        G.gps_off = off < 0x10000 ? off : 0;
    }
    return NULL;
}

const char *game_discover(const char *game_id)
{
    static char err[512];
    g_base = (uintptr_t)GetModuleHandleW(NULL);
    if (!pe_init()) return "PE header";
    err[0] = 0;
    int tried = 0;
    for (int i = 0; i < N_LAYOUTS; i++) {
        if (!game_id || strcmp(g_layouts[i].game, game_id)) continue;  // a layout of another game is never tried
        L = &g_layouts[i];
        const char *e = discover_with();
        if (!e) return NULL;
        snprintf(err + strlen(err), sizeof err - strlen(err), "%s%s: %s", tried++ ? "; " : "", L->name, e);
    }
    L = NULL;
    memset(&G, 0, sizeof G);  // a failed layout must not leave half-filled addresses behind
    if (!tried) snprintf(err, sizeof err, "no layout for game %s", game_id ? game_id : "?");
    return err;
}

int game_exe_stamp(unsigned *timestamp, unsigned *image_size)
{
    const unsigned char *b = (const unsigned char *)GetModuleHandleW(NULL);
    if (!b || !mem_readable(b, 0x400)) return 0;
    const IMAGE_DOS_HEADER *dos = (const IMAGE_DOS_HEADER *)b;
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 || dos->e_lfanew > 0x1000) return 0;
    const IMAGE_NT_HEADERS64 *nt = (const IMAGE_NT_HEADERS64 *)(b + dos->e_lfanew);
    if (!mem_readable(nt, sizeof *nt) || nt->Signature != IMAGE_NT_SIGNATURE) return 0;
    if (timestamp) *timestamp = nt->FileHeader.TimeDateStamp;
    if (image_size) *image_size = nt->OptionalHeader.SizeOfImage;
    return 1;
}

// ---------------------------------------------------------------- call-site hooks

static unsigned char *g_tramp;
static size_t g_tramp_used;

static unsigned char *alloc_near(uintptr_t target)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x60000000u; d += gran) {
        void *p = VirtualAlloc((void *)((target - d) & ~(gran - 1)), 0x1000, MEM_RESERVE | MEM_COMMIT,
                               PAGE_EXECUTE_READWRITE);
        if (p) return (unsigned char *)p;
        p = VirtualAlloc((void *)((target + d) & ~(gran - 1)), 0x1000, MEM_RESERVE | MEM_COMMIT,
                         PAGE_EXECUTE_READWRITE);
        if (p) return (unsigned char *)p;
    }
    return NULL;
}

// Run pre(rcx) before a tail jump (E9) goes on to its target, keeping the
// argument registers rcx, rdx, r8, r9 and xmm0-3 intact.
int game_patch_tail(uintptr_t site, void *pre)
{
    if (*(const unsigned char *)site != 0xE9) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + 128 > 0x1000) return 0;
    uintptr_t target = site + 5 + (intptr_t) * (const int32_t *)(site + 1);
    unsigned char *c = g_tramp + g_tramp_used, *p = c;
    static const unsigned char head[] = {
        0x51, 0x52, 0x41, 0x50, 0x41, 0x51,              // push rcx, rdx, r8, r9
        0x48, 0x83, 0xEC, 0x68,                          // sub rsp, 0x68 (shadow + 4 xmm, 16-aligned)
        0xF3, 0x0F, 0x7F, 0x44, 0x24, 0x20,              // movdqu [rsp+0x20], xmm0
        0xF3, 0x0F, 0x7F, 0x4C, 0x24, 0x30,              // movdqu [rsp+0x30], xmm1
        0xF3, 0x0F, 0x7F, 0x54, 0x24, 0x40,              // movdqu [rsp+0x40], xmm2
        0xF3, 0x0F, 0x7F, 0x5C, 0x24, 0x50,              // movdqu [rsp+0x50], xmm3
        0x48, 0xB8 };                                    // mov rax, imm64 (pre)
    static const unsigned char mid[] = {
        0xFF, 0xD0,                                      // call rax (rcx = self still)
        0xF3, 0x0F, 0x6F, 0x44, 0x24, 0x20,              // movdqu xmm0, [rsp+0x20]
        0xF3, 0x0F, 0x6F, 0x4C, 0x24, 0x30,
        0xF3, 0x0F, 0x6F, 0x54, 0x24, 0x40,
        0xF3, 0x0F, 0x6F, 0x5C, 0x24, 0x50,
        0x48, 0x83, 0xC4, 0x68,                          // add rsp, 0x68
        0x41, 0x59, 0x41, 0x58, 0x5A, 0x59,              // pop r9, r8, rdx, rcx
        0x48, 0xB8 };                                    // mov rax, imm64 (target)
    memcpy(p, head, sizeof head); p += sizeof head;
    memcpy(p, &pre, 8); p += 8;
    memcpy(p, mid, sizeof mid); p += sizeof mid;
    memcpy(p, &target, 8); p += 8;
    *p++ = 0xFF; *p++ = 0xE0;                            // jmp rax
    g_tramp_used += ((p - c) + 15) & ~(size_t)15;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    int32_t rel = (int32_t)rel64;
    DWORD old;
    if (!VirtualProtect((void *)(site + 1), 4, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)(site + 1), &rel, 4);
    VirtualProtect((void *)(site + 1), 4, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)(site + 1), 4);
    return 1;
}

// Replace the 8-byte "mulss xmm7,[rip+x]" at site by a call to a thunk that
// sets xmm7 = fn(xmm7 = idle rpm, xmm10 = throttle, xmm12 = brake) and keeps
// every other register (lka-roll-re notes C).
int game_patch_stop(uintptr_t site, float (*fn)(float, float, float))
{
    const unsigned char *s = (const unsigned char *)site;
    if (s[0] != 0xF3 || s[1] != 0x0F || s[2] != 0x59 || s[3] != 0x3D) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + 160 > 0x1000) return 0;
    unsigned char *c = g_tramp + g_tramp_used, *p = c;
    static const unsigned char head[] = {
        0x50, 0x51, 0x52, 0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53,  // push rax rcx rdx r8 r9 r10 r11
        0x48, 0x81, 0xEC, 0x80, 0x00, 0x00, 0x00,                          // sub rsp, 0x80
        0xF3, 0x0F, 0x7F, 0x44, 0x24, 0x20,                                // movdqu [rsp+0x20], xmm0
        0xF3, 0x0F, 0x7F, 0x4C, 0x24, 0x30,                                // .. xmm1
        0xF3, 0x0F, 0x7F, 0x54, 0x24, 0x40,                                // .. xmm2
        0xF3, 0x0F, 0x7F, 0x5C, 0x24, 0x50,                                // .. xmm3
        0xF3, 0x0F, 0x7F, 0x64, 0x24, 0x60,                                // .. xmm4
        0xF3, 0x0F, 0x7F, 0x6C, 0x24, 0x70,                                // .. xmm5
        0x0F, 0x28, 0xC7,                                                  // movaps xmm0, xmm7
        0x41, 0x0F, 0x28, 0xCA,                                            // movaps xmm1, xmm10
        0x41, 0x0F, 0x28, 0xD4,                                            // movaps xmm2, xmm12 (the brake; 1.61: xmm13)
        0x48, 0xB8 };                                                      // mov rax, fn
    static const unsigned char tail[] = {
        0xFF, 0xD0,                                                        // call rax
        0x0F, 0x28, 0xF8,                                                  // movaps xmm7, xmm0
        0xF3, 0x0F, 0x6F, 0x44, 0x24, 0x20,                                // restore xmm0-5
        0xF3, 0x0F, 0x6F, 0x4C, 0x24, 0x30,
        0xF3, 0x0F, 0x6F, 0x54, 0x24, 0x40,
        0xF3, 0x0F, 0x6F, 0x5C, 0x24, 0x50,
        0xF3, 0x0F, 0x6F, 0x64, 0x24, 0x60,
        0xF3, 0x0F, 0x6F, 0x6C, 0x24, 0x70,
        0x48, 0x81, 0xC4, 0x80, 0x00, 0x00, 0x00,                          // add rsp, 0x80
        0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58, 0x5A, 0x59, 0x58,  // pop r11 r10 r9 r8 rdx rcx rax
        0xC3 };                                                            // ret
    memcpy(p, head, sizeof head); p += sizeof head;
    if (game_is_162()) { p[-7] = 0xCD; p[-3] = 0xD7; }                   // 1.62: the update keeps the throttle in xmm13, the brake in xmm15
    else if (game_is_161()) p[-3] = 0xD5;                               // the brake moved to xmm13 on 1.61 (ModRM of the movaps)
    memcpy(p, &fn, 8); p += 8;
    memcpy(p, tail, sizeof tail); p += sizeof tail;
    g_tramp_used += ((p - c) + 15) & ~(size_t)15;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    unsigned char code[8] = { 0xE8, 0, 0, 0, 0, 0x90, 0x90, 0x90 };       // call thunk; nop x3
    int32_t rel = (int32_t)rel64;
    memcpy(code + 1, &rel, 4);
    DWORD old;
    if (!VirtualProtect((void *)site, 8, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)site, code, 8);
    VirtualProtect((void *)site, 8, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, 8);
    return 1;
}

// Replace a 9-byte "movss [reg+disp32], xmm8" store by a call to a thunk that
// sets xmm8 = fn(xmm8) (all volatile registers kept) and then does the
// original store. Used for the LKA assist state A (lka-roll-re notes A4).
int game_patch_store_xmm8(uintptr_t site, float (*fn)(float))
{
    const unsigned char *s = (const unsigned char *)site;
    if (s[0] != 0xF3 || s[1] != 0x44 || s[2] != 0x0F || s[3] != 0x11) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + 160 > 0x1000) return 0;
    unsigned char *c = g_tramp + g_tramp_used, *p = c;
    static const unsigned char head[] = {
        0x50, 0x51, 0x52, 0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53,  // push rax rcx rdx r8 r9 r10 r11
        0x48, 0x81, 0xEC, 0x80, 0x00, 0x00, 0x00,                          // sub rsp, 0x80
        0xF3, 0x0F, 0x7F, 0x44, 0x24, 0x20, 0xF3, 0x0F, 0x7F, 0x4C, 0x24, 0x30,
        0xF3, 0x0F, 0x7F, 0x54, 0x24, 0x40, 0xF3, 0x0F, 0x7F, 0x5C, 0x24, 0x50,
        0xF3, 0x0F, 0x7F, 0x64, 0x24, 0x60, 0xF3, 0x0F, 0x7F, 0x6C, 0x24, 0x70,  // save xmm0-5
        0x41, 0x0F, 0x28, 0xC0,                                            // movaps xmm0, xmm8
        0x48, 0xB8 };                                                      // mov rax, fn
    static const unsigned char tail[] = {
        0xFF, 0xD0,                                                        // call rax
        0x44, 0x0F, 0x28, 0xC0,                                            // movaps xmm8, xmm0
        0xF3, 0x0F, 0x6F, 0x44, 0x24, 0x20, 0xF3, 0x0F, 0x6F, 0x4C, 0x24, 0x30,
        0xF3, 0x0F, 0x6F, 0x54, 0x24, 0x40, 0xF3, 0x0F, 0x6F, 0x5C, 0x24, 0x50,
        0xF3, 0x0F, 0x6F, 0x64, 0x24, 0x60, 0xF3, 0x0F, 0x6F, 0x6C, 0x24, 0x70,  // restore xmm0-5
        0x48, 0x81, 0xC4, 0x80, 0x00, 0x00, 0x00,                          // add rsp, 0x80
        0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58, 0x5A, 0x59, 0x58 };// pop r11 .. rax
    memcpy(p, head, sizeof head); p += sizeof head;
    memcpy(p, &fn, 8); p += 8;
    memcpy(p, tail, sizeof tail); p += sizeof tail;
    memcpy(p, s, 9); p += 9;                                               // the original store
    *p++ = 0xC3;                                                           // ret
    g_tramp_used += ((p - c) + 15) & ~(size_t)15;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    unsigned char code[9] = { 0xE8, 0, 0, 0, 0, 0x90, 0x90, 0x90, 0x90 };
    int32_t rel = (int32_t)rel64;
    memcpy(code + 1, &rel, 4);
    DWORD old;
    if (!VirtualProtect((void *)site, 9, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)site, code, 9);
    VirtualProtect((void *)site, 9, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, 9);
    return 1;
}

// Small thunk helpers for code that has no call to hook. The thunk runs the
// original instructions itself (they must not be rip-relative) plus a tweak.
static int patch_with_thunk(uintptr_t site, int len, const unsigned char *code, size_t n)
{
    if (len < 5 || !g_tramp) return 0;
    if (g_tramp_used + n + 16 > 0x1000) return 0;
    unsigned char *c = g_tramp + g_tramp_used;
    memcpy(c, code, n);
    g_tramp_used += (n + 15) & ~(size_t)15;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    unsigned char rep[32];
    memset(rep, 0x90, sizeof rep);
    rep[0] = 0xE8;
    int32_t rel = (int32_t)rel64;
    memcpy(rep + 1, &rel, 4);
    DWORD old;
    if (!VirtualProtect((void *)site, len, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)site, rep, len);
    VirtualProtect((void *)site, len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, len);
    return 1;
}

// After the original len bytes (they load xmm1): xmm1 = max(xmm1, *feed).
int game_patch_feed_xmm1(uintptr_t site, int len, float *feed)
{
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    unsigned char code[64], *p = code;
    memcpy(p, (const void *)site, len); p += len;
    *p++ = 0x50;                                  // push rax
    *p++ = 0x48; *p++ = 0xB8; memcpy(p, &feed, 8); p += 8;  // mov rax, feed
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x5F; *p++ = 0x08;    // maxss xmm1, [rax]
    *p++ = 0x58;                                  // pop rax
    *p++ = 0xC3;                                  // ret
    return patch_with_thunk(site, len, code, p - code);
}

// The original len-byte instruction only runs while *skip == 0 (flags kept).
int game_patch_skip_if(uintptr_t site, int len, const unsigned char *skip)
{
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    unsigned char code[64], *p = code;
    *p++ = 0x9C;                                  // pushfq
    *p++ = 0x50;                                  // push rax
    *p++ = 0x48; *p++ = 0xB8; memcpy(p, &skip, 8); p += 8;  // mov rax, skip
    *p++ = 0x80; *p++ = 0x38; *p++ = 0x00;        // cmp byte [rax], 0
    *p++ = 0x58;                                  // pop rax
    *p++ = 0x75; *p++ = (unsigned char)len;       // jne over the original
    memcpy(p, (const void *)site, len); p += len;
    *p++ = 0x9D;                                  // popfq
    *p++ = 0xC3;                                  // ret
    return patch_with_thunk(site, len, code, p - code);
}

// After the original len bytes (they load xmm<reg>, 0-15): xmm<reg> = 0 while *flag. Flags are kept.
int game_patch_zero_xmm_if(uintptr_t site, int len, int reg, const unsigned char *flag)
{
    if (reg < 0 || reg > 15) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    unsigned char code[64], *p = code;
    memcpy(p, (const void *)site, len); p += len;            // original: the load
    *p++ = 0x9C;                                              // pushfq
    *p++ = 0x50;                                              // push rax
    *p++ = 0x48; *p++ = 0xB8; memcpy(p, &flag, 8); p += 8;    // mov rax, flag
    *p++ = 0x80; *p++ = 0x38; *p++ = 0x00;                    // cmp byte [rax], 0
    *p++ = 0x58;                                              // pop rax
    *p++ = 0x74; *p++ = reg >= 8 ? 0x04 : 0x03;              // je over the clear (3 bytes, 4 with REX.B)
    if (reg >= 8) *p++ = 0x41;                                // xorps xmmR, xmmR (REX.B for xmm8-15)
    *p++ = 0x0F; *p++ = 0x57; *p++ = (unsigned char)(0xC0 | (reg & 7) << 3 | (reg & 7));
    *p++ = 0x9D;                                              // popfq
    *p++ = 0xC3;                                              // ret
    return patch_with_thunk(site, len, code, p - code);
}

// `comiss xmm0, xmm7; jbe d8` (5 bytes) becomes a compare of xmm0 against our own
// constant, same branch: the call's return address is the fall-through, a taken
// jbe adds d8 to it. (accmode key: xmm7 is shared with our min_kmh patch.)
int game_patch_comiss_xmm0(uintptr_t site, float value)
{
    const unsigned char *b = (const unsigned char *)site;
    if (b[0] != 0x0F || b[1] != 0x2F || (b[2] != 0xC7 && b[2] != 0xC6) || b[3] != 0x76 || b[4] > 0x7F) return 0;   // xmm7 (1.60/1.61) or xmm6 (1.62)
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    static float consts[4];
    static int n_consts;
    if (n_consts >= 4) return 0;
    float *c = &consts[n_consts++];
    *c = value;
    unsigned char code[64], *p = code;
    *p++ = 0x50;                                              // push rax
    *p++ = 0x48; *p++ = 0xB8; memcpy(p, &c, 8); p += 8;       // mov rax, c
    *p++ = 0x0F; *p++ = 0x2F; *p++ = 0x00;                    // comiss xmm0, [rax]
    *p++ = 0x58;                                              // pop rax
    *p++ = 0x76; *p++ = 0x01;                                 // jbe taken
    *p++ = 0xC3;                                              // ret (fall-through)
    *p++ = 0x48; *p++ = 0x83; *p++ = 0x04; *p++ = 0x24; *p++ = b[4];  // taken: add qword [rsp], d8
    *p++ = 0xC3;                                              // ret
    return patch_with_thunk(site, 5, code, p - code);
}

// The original len-byte compare (rip-free) runs only while *flag == 0; while
// it is set the carry flag is cleared instead (a following jae/jnb is taken).
int game_patch_cf0_if(uintptr_t site, int len, const unsigned char *flag)
{
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    unsigned char code[64], *p = code;
    *p++ = 0x50;                                              // push rax
    *p++ = 0x48; *p++ = 0xB8; memcpy(p, &flag, 8); p += 8;    // mov rax, flag
    *p++ = 0x80; *p++ = 0x38; *p++ = 0x00;                    // cmp byte [rax], 0
    *p++ = 0x58;                                              // pop rax
    *p++ = 0x74; *p++ = 0x02;                                 // je original
    *p++ = 0xF8;                                              // clc
    *p++ = 0xC3;                                              // ret
    memcpy(p, (const void *)site, len); p += len;             // original: the compare
    *p++ = 0xC3;                                              // ret
    return patch_with_thunk(site, len, code, p - code);
}

// Redirect one E8 call to stub through a nearby trampoline (mov rax, stub; jmp rax).
int game_patch_call(uintptr_t site, void *stub)
{
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + 16 > 0x1000) return 0;
    unsigned char *tr = g_tramp + g_tramp_used;
    tr[0] = 0x48; tr[1] = 0xB8;
    memcpy(tr + 2, &stub, 8);
    tr[10] = 0xFF; tr[11] = 0xE0;
    g_tramp_used += 16;
    int64_t rel64 = (int64_t)(uintptr_t)tr - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    int32_t rel = (int32_t)rel64;
    DWORD old;
    if (!VirtualProtect((void *)(site + 1), 4, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)(site + 1), &rel, 4);
    VirtualProtect((void *)(site + 1), 4, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)(site + 1), 4);
    return 1;
}

// The drivetrain copies the vehicle's clutch (inner+0x142c) to drive+0x330, which all torque coupling and
// engine-speed code reads (creep-re notes). At the copy (12 bytes, 8B 83 2C 14 00 00 89 86 30 03 00 00):
// for the vehicle cf->inner only, the copy is max(clutch, cf->floor); every other vehicle is left alone.
// The vehicle's own clutch state (gearbox sequencer, stop rule) stays as the game has it.
int game_patch_clutch_floor(uintptr_t site, clutch_floor *cf)
{
    unsigned char orig[12] = { 0x8B, 0x83, 0, 0, 0, 0, 0x89, 0x86, 0x30, 0x03, 0x00, 0x00 };   // mov eax,[rbx+clutch]; mov [rsi+0x330],eax
    unsigned cl = L->i_clutch;
    for (int i = 0; i < 4; i++) orig[2 + i] = (unsigned char)(cl >> (8 * i));
    unsigned char dclutch = game_is_162() ? 0x38 : 0x30;   // drive+0x330, 1.62: +0x338 (the drive object grew 8 there)
    orig[8] = dclutch;
    if (memcmp((const void *)site, orig, sizeof orig)) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + 64 > 0x1000) return 0;
    unsigned char *c = g_tramp + g_tramp_used, *p = c;
    *p++ = 0x8B; *p++ = 0x83;                             // mov eax,[rbx+clutch]
    for (int i = 0; i < 4; i++) *p++ = (unsigned char)(cl >> (8 * i));
    *p++ = 0x48; *p++ = 0xBA;                             // mov rdx, imm64 (cf)
    memcpy(p, &cf, 8); p += 8;
    static const unsigned char b[] = {
        0x48, 0x3B, 0x1A,                                // cmp rbx,[rdx]
        0x75, 0x03,                                      // jne +3 (store)
        0x8B, 0x42, 0x08,                                // mov eax,[rdx+8]: our value replaces the game's (ts4i: at rest
                                                         // the automatic keeps its clutch open at idle, a max() never closed it)
        0x89, 0x86, 0x30, 0x03, 0x00, 0x00,              // store: mov [rsi+0x330],eax
        0xC3 };                                          // ret
    memcpy(p, b, sizeof b); p[-5] = dclutch; p += sizeof b;
    g_tramp_used += ((p - c) + 15) & ~(size_t)15;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    unsigned char rep[12];
    memset(rep, 0x90, sizeof rep);
    rep[0] = 0xE8;
    int32_t rel = (int32_t)rel64;
    memcpy(rep + 1, &rel, 4);
    DWORD old;
    if (!VirtualProtect((void *)site, sizeof rep, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)site, rep, sizeof rep);
    VirtualProtect((void *)site, sizeof rep, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, sizeof rep);
    return 1;
}

// The retarder demand the drivetrain reads (0x1408c87a8, ETS2 1.60.1.7): `48 8B BF D0 04 00 00` (mov rdi,[rdi+0x4d0], the retarder
// definition), then the 8 bytes at site, `F3 0F 10 80 44 14 00 00` (movss xmm0,[rax+0x1444]: the demand float of the vehicle's
// inner, rax = inner), then `E8` (a curve lookup of xmm0) and `F3 44 0F 10 87 A8 00 00 00` (movss xmm8,[rdi+0xa8]: the definition's
// torque). The 8 bytes become a call to our stub plus 3 nops. The stub (retarder_stub_code, game.h) is the original read followed
// by xmm0 *= rs->scale for the vehicle rs->inner only (rax = the inner of whichever vehicle the drivetrain updates, AI trucks too
// possibly), and the one instruction it replaces had no other effect: rax, every other register, xmm1-15 and the flags
// are as the game expects them after a movss (the stub saves rdx and the flags around its own use and never calls anything, so
// the stack alignment of the game is irrelevant). The HUD, the animation and the pedal function read inner+0x1444 at other
// places; they are not patched and keep showing the game's own step. The drivetrain runs on a game thread while we patch, so the
// 8 bytes go in with one locked 8-byte compare-exchange: the site must be 8-aligned (it is, 0x1408c87a8), else we decline.
int game_patch_retarder(uintptr_t site, const retarder_scale *rs)
{
    unsigned char orig[8] = { 0xF3, 0x0F, 0x10, 0x80, 0, 0, 0, 0 };   // movss xmm0,[rax+retarder]
    unsigned r = L->i_retarder;
    for (int i = 0; i < 4; i++) orig[4 + i] = (unsigned char)(r >> (8 * i));
    if (site & 7) return 0;
    if (memcmp((const void *)site, orig, sizeof orig)) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + 64 > 0x1000) return 0;
    unsigned char *c = g_tramp + g_tramp_used;
    size_t n = retarder_stub_code(c, rs);
    g_tramp_used += (n + 15) & ~(size_t)15;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    unsigned char rep[sizeof orig];
    memset(rep, 0x90, sizeof rep);
    rep[0] = 0xE8;
    int32_t rel = (int32_t)rel64;
    memcpy(rep + 1, &rel, 4);
    LONG64 want, was;
    memcpy(&want, rep, 8);
    memcpy(&was, orig, 8);
    DWORD old;
    if (!VirtualProtect((void *)site, sizeof rep, PAGE_EXECUTE_READWRITE, &old)) return 0;
    LONG64 got = InterlockedCompareExchange64((volatile LONG64 *)site, want, was);
    VirtualProtect((void *)site, sizeof rep, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, sizeof rep);
    return got == was;
}

// ---------------------------------------------------------------- object access

int mem_readable(const void *p, size_t n)
{
    MEMORY_BASIC_INFORMATION mi;
    if (!p || !VirtualQuery(p, &mi, sizeof mi)) return 0;
    if (mi.State != MEM_COMMIT || (mi.Protect & (PAGE_GUARD | PAGE_NOACCESS))) return 0;
    return (const char *)p + n <= (const char *)mi.BaseAddress + mi.RegionSize;
}

static int readable(const void *p, size_t n) { return mem_readable(p, n); }

typedef void *(*resolve_fn_t)(void *handle);
typedef float (*mass_fn_t)(void *game_actor, bool with_trailers);
typedef float (*grade_fn_t)(void *inner);
typedef float (*cvar_fn_t)(void *cvar);

char *gb_inner(void *self) { return self ? *(char **)((char *)self + L->s_inner) : NULL; }

int gb_gear(void *self)
{
    char *in = gb_inner(self);
    return in ? *(int32_t *)(in + L->i_gear) : 0;
}

int gb_target(void *self) { return *(int32_t *)((char *)self + L->s_target); }
int gb_state(void *self) { return *(int32_t *)((char *)self + L->s_state); }
void gb_force(void *self) { *((volatile unsigned char *)self + L->s_force) = 1; }
void gb_set_target(void *self, int g) { *(volatile int32_t *)((char *)self + L->s_target) = g; }
// The automatic's at-rest bookkeeping, as Creep writes it (1.60 offsets, creep.cpp): drive state 4 = a forward gear is wanted, 3 = at rest in
// neutral, and the latch byte "at rest, neutral". The game's at-rest rule sets 3 and 1 every update; written back at the end of the update
// they keep a target we set until the generic gearbox reads it.
void gb_rest_state(void *self, int drive, int latch)
{
    *(volatile int32_t *)((char *)self + L->s_drive) = drive;
    if (L->s_latch) *(volatile unsigned char *)((char *)self + L->s_latch) = (unsigned char)latch;   // 1.62 has no latch (drive state 3 says it)
}

// The generic gearbox's own engage step from its gap state (0x140601c7c, neutral-exit-re notes
// § 2): with gear 0 it waits there for throttle; this does
// what throttle would: vfunc +0x70(self, gear), state 3 (closing), +0x2c = 1.
// Only when the gap-state code matches the layout exactly (per-layout pattern).
static int g_engage_ok;
int game_engage_init()
{
    g_engage_ok = L && L->s_state == 0x18 && !game_is_162() && game_scan(game_is_161() ?   // 1.62: the generic gearbox's engage path changed (off)
        "48 8B 43 10 83 B8 D8 14 00 00 00 75 ?? 83 7B 70 00 74 ?? 0F 2F B8 B4 14 00 00 72 ?? 48 8B 03 48 8B CB FF 50 78" :
        "0F 2F 80 B8 01 00 00 72 ?? 85 C9 75 ?? 85 D2 74 ?? 41 0F 2F 80 24 14 00 00 72 ?? 48 8B 03 48 8B CB FF 50 78") != 0;
    return g_engage_ok;
}

int gb_engage(void *self, int g)
{
    if (!g_engage_ok || !self || gb_state(self) != 1 || gb_gear(self) != 0 || g <= 0) return 0;
    typedef void (*engage_fn)(void *, int);
    void **vt = *(void ***)self;
    if (!readable(vt, 0x78)) return 0;
    ((engage_fn)vt[0x70 / 8])(self, g);
    *(volatile int32_t *)((char *)self + L->s_state) = 3;
    *((volatile unsigned char *)self + 0x2c) = 1;
    return 1;
}

static float clamp01(float t) { return t >= 0.0f ? (t > 1.0f ? 1.0f : t) : 0.0f; }

float gb_thr_in(void *self)
{
    char *in = gb_inner(self);
    return in ? clamp01(*(float *)(in + L->i_thr_in)) : 0.0f;
}

float gb_thr_eff(void *self)
{
    char *in = gb_inner(self);
    return in ? clamp01(*(float *)(in + L->i_thr_eff)) : 0.0f;
}

static char *resolve(void *self, unsigned handle_off)
{
    char *in = gb_inner(self);
    if (!in) return NULL;
    void *h = *(void **)(in + handle_off);
    return h ? (char *)((resolve_fn_t)G.resolve_fn)(h) : NULL;
}

static int writable(const void *p, size_t n);
#ifdef DAZDRIVE_DEV
// Chiptune test (dev only, 28-10): the engine's max torque x5 in the game's own engine object, once per engine object, to learn
// whether the physics reads it live. TrueShift keeps seeing the original torque, so only the physics can tell the difference.
// The originals go back at shutdown (game_tune_restore). Remove after the test.
#define TUNE_TEST_K 1.0f   // (x5 for the test; 1 = off, the Tune button still flips x5 / x1)
static struct { char *eng; float orig; } s_tune[8];
static int s_tune_n;
static float tune_test(char *eng)
{
    float *t = (float *)(eng + L->e_torque);
    for (int i = 0; i < s_tune_n; i++) if (s_tune[i].eng == eng) return s_tune[i].orig;
    float orig = *t;
    if (s_tune_n >= 8 || !(orig > 50 && orig < 20000) || !writable(t, 4)) return orig;
    s_tune[s_tune_n].eng = eng; s_tune[s_tune_n].orig = orig; s_tune_n++;
    *t = orig * TUNE_TEST_K;
    logv(1, "chiptune test: engine torque %.0f -> %.0f N m (read back %.0f)", orig, orig * TUNE_TEST_K, *t);
    return orig;
}
void game_tune_set(float k)
{
    for (int i = 0; i < s_tune_n; i++)
        if (writable(s_tune[i].eng + L->e_torque, 4)) *(float *)(s_tune[i].eng + L->e_torque) = s_tune[i].orig * k;
}
// The engine objects the test wrote, and their original torque (for the scan in dev.cpp)
int game_tune_engines(char **eng, float *orig, int max)
{
    int n = s_tune_n < max ? s_tune_n : max;
    for (int i = 0; i < n; i++) { eng[i] = s_tune[i].eng; orig[i] = s_tune[i].orig; }
    return n;
}
void game_tune_restore()
{
    for (int i = 0; i < s_tune_n; i++)
        if (writable(s_tune[i].eng + L->e_torque, 4)) *(float *)(s_tune[i].eng + L->e_torque) = s_tune[i].orig;
    s_tune_n = 0;
}
#endif

int gb_engine(void *self, engine_info *e)
{
    char *eng = resolve(self, L->i_engine);
    if (!eng || !readable(eng, L->e_ranges + 32)) return 0;
#ifdef DAZDRIVE_DEV
    e->torque = tune_test(eng);
#else
    e->torque = *(float *)(eng + L->e_torque);
#endif
    e->rpm_limit = *(float *)(eng + L->e_rpm_limit);
    memcpy(e->range, eng + L->e_ranges, sizeof e->range);
    const float *c = *(const float **)(eng + L->e_curve);
    uint64_t n = *(uint64_t *)(eng + L->e_curve_n);
    e->electric = L->e_type && readable(eng + L->e_type, 8) && *(const unsigned long long *)(eng + L->e_type) == ELECTRIC_ENGINE_ID;
    e->n_curve = 0;
    if (c && n >= 2 && n <= 24 && readable(c, n * 8)) {
        for (uint64_t i = 0; i < n; i++) {
            float r = c[2 * i], f = c[2 * i + 1];
            if (!(r >= 0 && r < 10000 && f >= 0 && f <= 2)) break;
            if (e->n_curve && r <= e->curve[e->n_curve - 1][0]) continue;
            e->curve[e->n_curve][0] = r;
            e->curve[e->n_curve][1] = f;
            e->n_curve++;
        }
    }
    char *tr = resolve(self, L->i_trans);
    e->shift_time = tr && readable(tr, L->t_shift_time + 4) ? *(float *)(tr + L->t_shift_time) : NAN;
    if (!(e->shift_time > 0.05f && e->shift_time < 5.0f)) e->shift_time = 0.8f;
    // WP-TC2: the stall torque ratio sits 8 bytes under the shift time, the diff 4 (live RE, ETS2 1.61); trusted only where that diff equals the truck config's (core/vehicle.cpp).
    // Every layout that has the shift time gets the same read: ATS 1.61 has the ETS2 1.61 object (0x1c8), 1.60 has it at 0x1c0 (-8 = 0x1b8 not seen live: the diff check decides).
    engine_read_stall(e, tr && L->t_shift_time >= 8 && readable(tr, L->t_shift_time + 4) ? tr : NULL, L->t_shift_time, T.diff);
    // 1 = the torque is usable. What the definition did not give (a mod engine with only `torque:`: no curve, no ranges, no limit) is marked in e->missing and the model
    // stands the game's own defaults in for it (vehicle_update, core/vehicle.cpp; the ride of 04-11 read none of it and TrueShift was blind for the whole drive).
    return engine_raw_check(e);
}

float curve_factor(const engine_info *e, float rpm)
{
    if (e->n_curve < 2) return 1.0f;
    if (rpm <= e->curve[0][0]) return e->curve[0][1];
    for (int i = 1; i < e->n_curve; i++) {
        if (rpm <= e->curve[i][0]) {
            float r0 = e->curve[i - 1][0], f0 = e->curve[i - 1][1];
            float k = (rpm - r0) / (e->curve[i][0] - r0);
            return f0 + k * (e->curve[i][1] - f0);
        }
    }
    return e->curve[e->n_curve - 1][1];
}

static char *game_actor()
{
    if (!G.base_ctrl_slot) return NULL;
    char *bc = *(char **)G.base_ctrl_slot;
    if (!bc) return NULL;
    return *(char **)(bc + G.game_actor_off);
}

// Only call these from inside the game's gearbox update (our hook stubs):
// that is where the game calls them itself, so the objects are valid.
float game_mass()
{
    char *ga = game_actor();
    if (!ga || !G.mass_fn) return NAN;
    float m = ((mass_fn_t)G.mass_fn)(ga, true);
    return m > 500.0f && m < 500000.0f ? m : NAN;
}

float game_mass_own()
{
    char *ga = game_actor();
    if (!ga || !G.mass_fn) return NAN;
    float m = ((mass_fn_t)G.mass_fn)(ga, false);
    return m > 200.0f && m < 200000.0f ? m : NAN;
}

// The ATS game layer (ds-ats161) fills these in; until then the mass rule of vehicle_class() carries the detection and the
// truck limit (truck.navigation.speed.limit) is the only limit the suite reads.
int game_vehicle_is_car() { return -1; }
float game_car_speed_limit_ms() { return NAN; }

float game_grade_deg(void *self)
{
    char *in = gb_inner(self);
    if (!in || !G.grade_fn) return NAN;
    float g = ((grade_fn_t)G.grade_fn)(in);
    return g > -60.0f && g < 60.0f ? g : NAN;
}

float game_adaptive()
{
    if (!G.cvar_adaptive) return NAN;
    return ((cvar_fn_t)G.cvar_read_fn)((void *)G.cvar_adaptive);
}

int game_adaptive_blocked()
{
    char *ga = game_actor();
    return ga ? *(unsigned char *)(ga + L->a_adaptive_block) : -1;
}

// ---------------------------------------------------------------- dashboard letter

static int writable(const void *p, size_t n)
{
    MEMORY_BASIC_INFORMATION mi;
    if (!p || !VirtualQuery(p, &mi, sizeof mi) || mi.State != MEM_COMMIT) return 0;
    if (!(mi.Protect & (PAGE_READWRITE | PAGE_EXECUTE_READWRITE)) || (mi.Protect & PAGE_GUARD)) return 0;
    return (const char *)p + n <= (const char *)mi.BaseAddress + mi.RegionSize;
}

// The one-letter auto_mode_name buffer of a transmission object, or NULL when
// the object does not look like one (auto name 1 char, manual name "M", a letter of disp_letters(): core/display.cpp refuses any
// other, so a letter we show never makes the buffer look foreign).
static char *auto_name(char *tx)
{
    unsigned o = L->t_auto_name;
    if (!tx || !readable(tx + o, 0x28)) return NULL;
    char *a = *(char **)(tx + o), *m = *(char **)(tx + o + 0x20);
    uint32_t na = *(uint32_t *)(tx + o + 8), nm = *(uint32_t *)(tx + o + 0x28);
    if (na != 1 || nm != 1 || !writable(a, 2) || !readable(m, 2)) return NULL;
    if (m[0] != 'M' || m[1] || a[1] || !strchr(disp_letters(), a[0])) return NULL;
    return a;
}

void *game_gearbox()
{
    char *ga = game_actor();
    if (!ga || !readable(ga + L->a_gearbox, 8)) return NULL;
    char *gb = *(char **)(ga + L->a_gearbox);
    if (!gb || !readable(gb + L->s_inner, 8)) return NULL;
    char *in = *(char **)(gb + L->s_inner);
    return in && readable(in + L->i_gear, 4) ? gb : NULL;
}

static char *s_letter_buf;       // the buffer we wrote last
static char s_letter_game;       // the letter the game had in it before our first write
int game_letter(void *self, char c)
{
    char *in = gb_inner(self);
    if (!in) return 0;
    char *a = auto_name(resolve(self, L->i_trans));
    if (!a && readable(in + L->i_tx_alt, 8)) {
        char *o = *(char **)(in + L->i_tx_alt);
        if (o && readable(o + L->x_tx_alt, 8)) a = auto_name(*(char **)(o + L->x_tx_alt));
    }
    if (!a) return 0;
    if (a != s_letter_buf) { s_letter_buf = a; s_letter_game = *a; }
    if (!c) c = s_letter_game;
    if (*a != c) *a = c;
    return 1;
}

// ---------------------------------------------------------------- engine brake

// The game's pedal fn sets inner+0x1440 every tick when an engine brake is
// active (stage, held or automatic); the stock automatic then shifts into the
// engine-brake rpm range, but only for engines whose def has
// engine_brake_downshift (engine+0x260).
int game_ebrake_downshift(void *self)
{
    char *in = gb_inner(self), *eng = resolve(self, L->i_engine);
    if (!in || !eng || !readable(eng + L->e_eb_downshift, 8)) return -1;
    return *(unsigned char *)(in + L->i_eb_flag) && *(unsigned char *)(eng + L->e_eb_downshift);
}

float game_ebrake_level(void *self)
{
    char *in = gb_inner(self), *ga = game_actor();
    float lv = -1.0f;
    if (in) {
        float d = *(float *)(in + L->i_eb_demand);  // max of stage value and automatic engine brake
        if (d >= 0.0f && d <= 1.0f) lv = d;
    }
    if (ga && readable(ga + L->a_eb_frac, 4)) {
        float f = *(float *)(ga + L->a_eb_frac);    // held = 1, on = stage/stages
        if (f >= 0.0f && f <= 1.0f) lv = lv > f ? lv : f;
    }
    return lv;
}

// ---------------------------------------------------------------- engine brake lever (modules/ebrake.cpp, core/ebrakelogic.h)

// The key handler consumes the up and down key event with a call, then acts on the press (stage + 1 / - 1 through the stage setter).
// We wrap those two calls: the callback sees every press of the player's keys, with its direction, BEFORE the handler acts on it.
// Layout-gated (L->eb_pat_*): the patterns start at the E8 of the consume call and end with the E8 of the stage setter call.
typedef void (*eb_consume_fn)(void *act);
typedef void (*eb_setter_fn)(void *game_actor, unsigned level);
static ebrake_press_fn s_eb_cb;
static eb_consume_fn s_eb_consume;
static eb_setter_fn s_eb_setter;

static void eb_consume_stub(void *act)
{
    s_eb_consume(act);
    char *ga = game_actor_safe();
    if (!ga || !s_eb_cb || !readable(ga + L->a_eb_up, 8) || !readable(ga + L->a_eb_dwn, 8)) return;
    if (act == *(void **)(ga + L->a_eb_up)) s_eb_cb(ga, 1);
    else if (act == *(void **)(ga + L->a_eb_dwn)) s_eb_cb(ga, -1);
}

static int pat_len(const char *p)
{
    int n = 0;
    for (const char *c = p; *c; c++) if (*c == ' ') n++;
    return n + 1;
}

// The bytes of a mov with a 32-bit displacement: `<prefix> disp32`, searched in the first 0x80 bytes of the setter.
static int has_disp_store(uintptr_t fn, const unsigned char *prefix, int np, unsigned disp)
{
    unsigned char want[8];
    memcpy(want, prefix, np);
    for (int i = 0; i < 4; i++) want[np + i] = (unsigned char)(disp >> (8 * i));
    for (int i = 0; i + np + 4 <= 0x80; i++)
        if (!memcmp((const void *)(fn + i), want, np + 4)) return 1;
    return 0;
}

// Resolve the game's stage setter from the key code patterns (no patching): the same checks the lever's hook needs. 1 = s_eb_setter is set.
// u / d (optional) get the two scan hits (the up and down key calls).
static int eb_resolve(const char **why, uintptr_t *u_out, uintptr_t *d_out)
{
    *why = NULL;
    if (!L || !L->eb_pat_up || !L->eb_pat_dwn || !L->a_eb_on || !L->a_eb_stage || !L->a_eb_up || !L->a_eb_dwn || !L->e_eb_cap || !L->i_engine) {
        *why = "this game build is not verified for it";
        return 0;
    }
    uintptr_t u = game_scan(L->eb_pat_up), d = game_scan(L->eb_pat_dwn);
    if (!u || !d) { *why = !u && !d ? "neither key code found" : !u ? "up key code not found" : "down key code not found"; return 0; }
    uintptr_t us = u + pat_len(L->eb_pat_up) - 5, ds = d + pat_len(L->eb_pat_dwn) - 5;
    if (*(const unsigned char *)u != 0xE8 || *(const unsigned char *)d != 0xE8 || *(const unsigned char *)us != 0xE8 || *(const unsigned char *)ds != 0xE8) {
        *why = "key code looks different";
        return 0;
    }
    if (call_target(u) != call_target(d) || call_target(us) != call_target(ds)) { *why = "the two keys do not share their calls"; return 0; }
    uintptr_t set = call_target(us);
    if (!in_text(set) || !in_text(call_target(u))) { *why = "call targets outside the code"; return 0; }
    // the setter must read the on byte and store the stage at the offsets of the layout: movzx ebx,[rdi+on] and mov [rdi+stage],r9d
    static const unsigned char rd_on[] = { 0x0F, 0xB6, 0x9F }, wr_stage[] = { 0x44, 0x89, 0x8F };
    if (!has_disp_store(set, rd_on, 3, L->a_eb_on) || !has_disp_store(set, wr_stage, 3, L->a_eb_stage)) { *why = "stage setter does not match the layout"; return 0; }
    s_eb_setter = (eb_setter_fn)set;
    if (u_out) *u_out = u;
    if (d_out) *d_out = d;
    return 1;
}

int game_ebrake_hook(ebrake_press_fn cb, const char **why)
{
    uintptr_t u = 0, d = 0;
    if (!eb_resolve(why, &u, &d)) return 0;
    s_eb_consume = (eb_consume_fn)call_target(u);
    s_eb_cb = cb;
    if (!game_patch_call(u, (void *)eb_consume_stub) || !game_patch_call(d, (void *)eb_consume_stub)) { s_eb_cb = NULL; *why = "patching the calls failed"; return 0; }
    return 1;
}

// The stage setter alone, without wrapping the keys (the descent control): 1 = game_ebrake_set can be called.
int game_ebrake_setter(const char **why)
{
    if (s_eb_setter) { *why = NULL; return 1; }
    return eb_resolve(why, NULL, NULL);
}

// The stage count of the player's engine (engine def u32), 0 = unknown. game_actor+0x18 is the vehicle (the stage setter reads it the same way).
unsigned game_ebrake_cap(char *ga)
{
    if (!ga || !L || !G.resolve_fn || !L->e_eb_cap || !L->i_engine || !readable(ga + 0x18, 8)) return 0;
    char *in = *(char **)(ga + 0x18);
    if (!in || !readable(in + L->i_engine, 8)) return 0;
    void *h = *(void **)(in + L->i_engine);
    char *eng = h ? (char *)((resolve_fn_t)G.resolve_fn)(h) : NULL;
    return eng && readable(eng + L->e_eb_cap, 4) ? *(unsigned *)(eng + L->e_eb_cap) : 0;
}

int game_ebrake_state(char *ga, int *held, int *on, unsigned *stage, int *flag)
{
    if (!ga || !L || !L->a_eb_on || !L->a_eb_stage || !readable(ga + L->a_eb_on, 1) || !readable(ga + L->a_eb_stage, 4)) return 0;
    *on = *(unsigned char *)(ga + L->a_eb_on);
    *stage = *(unsigned *)(ga + L->a_eb_stage);
    *held = L->a_eb_held && readable(ga + L->a_eb_held, 1) ? *(unsigned char *)(ga + L->a_eb_held) : 0;
    *flag = L->a_eb_flag && readable(ga + L->a_eb_flag, 1) ? *(unsigned char *)(ga + L->a_eb_flag) : 0;
    return 1;
}

// Set the on byte and the stage the way the key handler does: the byte, then the stage setter (it also recomputes the brake fraction;
// with the brake off or the stage 0 it zeroes the stage values, so the brake does nothing).
// Threads: the key handler (which calls the setter), the pedal function (which reads these bytes and computes the engine brake
// demand) and the throttle function (whose cruise call is our actuator's tick) are all called one after the other by the same parent,
// the game_actor update, in this order (ETS2 and ATS 1.61, call graph checked): key handler, pedal function, and inside it the throttle
// function. So this is safe from the callback of game_ebrake_hook and from the actuator's ACT_PRE / ACT_POST hooks (the tick is inside
// the pedal function and before its engine brake block and cruise cancel rules), and from nowhere else (not the telemetry frame).
void game_ebrake_put(char *ga, int on, unsigned stage)
{
    if (!ga || !s_eb_setter || !L->a_eb_on || !readable(ga + L->a_eb_on, 1)) return;
    *(volatile unsigned char *)(ga + L->a_eb_on) = on ? 1 : 0;
    s_eb_setter(ga, stage);
}

// The same plus the handler's flag byte (the "stage lever was used" byte the cruise rules read): the descent control writes the flag
// too, so the game's cruise keeps running with the brake on (flag set and the cruise's overspeed setting on) and its own overspeed
// engine brake stays within the stage. flag < 0 = leave it.
void game_ebrake_set(char *ga, int on, unsigned stage, int flag)
{
    if (!ga || !L) return;
    game_ebrake_put(ga, on, stage);
    if (flag >= 0 && L->a_eb_flag && readable(ga + L->a_eb_flag, 1)) *(volatile unsigned char *)(ga + L->a_eb_flag) = flag ? 1 : 0;
}

// ---------------------------------------------------------------- game_ctrl (outside the hooks)

// base_ctrl is a game_ctrl only while driving (not in menus); check the unit
// descriptor's class name like ets2la_plugin does (vtable slot 5 on Windows).
char *game_ctrl()
{
    static char *checked;
    static void *checked_vt;
    static int is_game;
    if (!G.base_ctrl_slot || !readable((void *)G.base_ctrl_slot, 8)) return NULL;
    char *bc = *(char **)G.base_ctrl_slot;
    if (!bc || !readable(bc, 8)) return NULL;
    void **vt = *(void ***)bc;
    if (bc != checked || vt != checked_vt) {
        checked = bc;
        checked_vt = vt;
        is_game = 0;
        if (readable(vt, 6 * 8)) {
            typedef void *(*desc_fn)(void *);
            void *desc = ((desc_fn)vt[5])(bc);
            if (desc && readable(desc, 8)) {
                const char **pn = *(const char ***)desc;
                if (pn && readable(pn, 8) && *pn && readable(*pn, 16)) is_game = !strcmp(*pn, "game_ctrl");
            }
        }
    }
    return is_game ? bc : NULL;
}

// ---------------------------------------------------------------- cvars by name

// Settings cvars are static objects: vtable pointer, then the name inline at
// +8 (acc notes). Find one by scanning the image outside .text.
void *game_cvar(const char *name)
{
    size_t len = strlen(name) + 1;
    const unsigned char *b = (const unsigned char *)g_base;
    const IMAGE_NT_HEADERS64 *nt = (const IMAGE_NT_HEADERS64 *)(b + ((const IMAGE_DOS_HEADER *)b)->e_lfanew);
    const IMAGE_SECTION_HEADER *sec = IMAGE_FIRST_SECTION(nt);
    uintptr_t img_lo = g_base, img_hi = g_base + nt->OptionalHeader.SizeOfImage;
    for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; i++) {
        if (!memcmp(sec[i].Name, ".text", 5) || !(sec[i].Characteristics & IMAGE_SCN_MEM_WRITE)) continue;
        const char *p = (const char *)(b + sec[i].VirtualAddress), *e = p + sec[i].Misc.VirtualSize - len;
        for (p += 8; p < e; p += 8) {
            if (*p != name[0] || memcmp(p, name, len)) continue;
            uintptr_t vt = *(const uintptr_t *)(p - 8);
            if (vt >= img_lo && vt < img_hi) return (void *)(p - 8);
        }
    }
    return NULL;
}

float game_cvar_get(void *cvar)
{
    if (!cvar || !G.cvar_read_fn) return NAN;
    return ((cvar_fn_t)G.cvar_read_fn)(cvar);
}

// The same path the game uses when a setting changes: virtual set(text).
void game_cvar_set(void *cvar, float value)
{
    if (!cvar) return;
    char buf[32];
    snprintf(buf, sizeof buf, "%g", value);
    typedef void (*set_fn)(void *, const char *);
    void **vt = *(void ***)cvar;
    ((set_fn)vt[1])(cvar, buf);
}

// The one hit of a pattern in .text (0 if none or several).
uintptr_t game_scan(const char *pattern)
{
    uintptr_t hits[2];
    return pat_scan(pattern, g_text_va, g_text_va + g_text_size, hits, 2) == 1 ? hits[0] : 0;
}

// All hits of a pattern in .text (up to max); -1 if there are more, 0 if none. For a pattern whose hit count is itself worth logging.
int game_scan_hits(const char *pattern, uintptr_t *hits, int max)
{
    return pat_scan(pattern, g_text_va, g_text_va + g_text_size, hits, max);
}

// Point a rip-relative float operand (disp32 at instr + disp_off, instruction
// length len) at a float of our own near the code: changes one constant for
// one instruction without touching the pooled original.
float *game_patch_rip_float(uintptr_t instr, int disp_off, int len, float value)
{
    if (!g_tramp && !(g_tramp = alloc_near(instr))) return NULL;
    if (g_tramp_used + 16 > 0x1000) return NULL;
    float *f = (float *)(g_tramp + g_tramp_used);
    *f = value;
    g_tramp_used += 16;
    int64_t rel64 = (int64_t)(uintptr_t)f - (int64_t)(instr + len);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return NULL;
    int32_t rel = (int32_t)rel64;
    DWORD old;
    if (!VirtualProtect((void *)(instr + disp_off), 4, PAGE_EXECUTE_READWRITE, &old)) return NULL;
    memcpy((void *)(instr + disp_off), &rel, 4);
    VirtualProtect((void *)(instr + disp_off), 4, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)instr, len);
    return f;
}

// game_actor from outside the hooks: only while base_ctrl is a game_ctrl.
char *game_actor_safe()
{
    char *gc = game_ctrl();
    if (!gc || !G.game_actor_off || !readable(gc + G.game_actor_off, 8)) return NULL;
    char *ga = *(char **)(gc + G.game_actor_off);
    return ga && readable(ga, 0x2000) ? ga : NULL;
}

// ---------------------------------------------------------------- brake pedal, brake light, retarder feed
// The mechanism for a later brake channel (brakepedal-re notes; the stubs and their reasons are in game.h). Each hook is installed
// at start only with its [core] switch, only when the layout has the offsets the stub reads (a_req, a_thr_pedal, a_brake_pedal,
// a_engine_run, i_retarder), and only when its pattern has exactly one hit and every self-check passes; otherwise the game's
// code stays as it is and the API calls below do nothing. The patterns and site/self-check offsets are per-layout (in the layout).

static brake_pedal_state s_bp;
static brake_light_block s_bl;
static retarder_feed_block s_rf;
static int s_bp_in, s_bl_in, s_rf_in;

// The game thread, every tick, from the pedal stub. game_actor is read only while a provider is registered.
float game_brake_pedal_fn(float raw, float req, void *ga)
{
    return brake_pedal_hook_body(&s_bp, raw, req, ga, s_bp.provider ? game_actor() : NULL, V.electric);
}
void game_brake_pedal_set_provider(float (*fn)(float raw, float req, void *ga)) { brake_pedal_set(&s_bp, fn); }
void game_brake_pedal_heartbeat() { brake_pedal_beat(&s_bp); }
int game_brake_pedal_ok() { return s_bp_in; }
const brake_pedal_counts *game_brake_pedal_counts() { return &s_bp.n; }

int game_brake_light_request(int on) { return s_bl_in ? brake_light_set(&s_bl, on ? game_actor_safe() : NULL, on) : 0; }
int game_brake_light_ok() { return s_bl_in; }
int game_retarder_feed(float demand)
{
    return s_rf_in ? retarder_feed_set(&s_rf, demand > 0.0f ? game_actor_safe() : NULL, demand) : 0;
}
int game_retarder_feed_ok() { return s_rf_in; }

// The 1.61 update is detected by its tail jump (the epilogue lost its pop rbx). The brake hooks have per-layout patterns,
// so the 1.60 self-checks (pedal getter, hill-start max) are skipped on 1.61 where the service-brake max moved registers:
// only the request read anchors the brake-pedal site there.

// The pedal getter (0x140990ee0, 1.60): returns game_actor+0x1cc0 (the throttle) when the pedals are swapped, else +0x1cc4
// (the brake field game_brake_pedal_fn compares raw with). 1.60 only (the 1.61 light/pedal checks do not follow it).
static int brake_getter_ok(uintptr_t fn)
{
    return in_text(fn) && pat_at("F3 0F 10 87 C0 1C 00 00", fn + 0x44) && pat_at("F3 0F 10 87 C4 1C 00 00", fn + 0x57);
}

// Put a stub of n bytes next to the game and turn the len bytes at site (they must be orig) into `call stub` + nops. cas: the
// bytes lie within one aligned qword and go in with one locked compare-exchange (safe while the game runs that code, like the
// retarder hook); else a plain write, only at start (before the game simulates anything, like patch_with_thunk).
static int patch_call_stub(uintptr_t site, int len, const unsigned char *orig, const unsigned char *stub, size_t n, int cas)
{
    if (len < 5 || len > 8 || memcmp((const void *)site, orig, len)) return 0;
    if (cas && (site & 7) + len > 8) return 0;
    if (!g_tramp && !(g_tramp = alloc_near(site))) return 0;
    if (g_tramp_used + n + 16 > 0x1000) return 0;
    unsigned char *c = g_tramp + g_tramp_used;
    int64_t rel64 = (int64_t)(uintptr_t)c - (int64_t)(site + 5);
    if (rel64 < INT32_MIN || rel64 > INT32_MAX) return 0;
    memcpy(c, stub, n);
    g_tramp_used += (n + 15) & ~(size_t)15;
    unsigned char rep[8];
    memset(rep, 0x90, sizeof rep);
    rep[0] = 0xE8;
    int32_t rel = (int32_t)rel64;
    memcpy(rep + 1, &rel, 4);
    DWORD old;
    if (cas) {
        uintptr_t q = site & ~(uintptr_t)7;
        LONG64 was, want;
        unsigned char nb[8];
        memcpy(&was, (const void *)q, 8);
        memcpy(nb, &was, 8);
        memcpy(nb + (site - q), rep, len);
        memcpy(&want, nb, 8);
        if (!VirtualProtect((void *)q, 8, PAGE_EXECUTE_READWRITE, &old)) return 0;
        LONG64 got = InterlockedCompareExchange64((volatile LONG64 *)q, want, was);
        VirtualProtect((void *)q, 8, old, &old);
        FlushInstructionCache(GetCurrentProcess(), (void *)q, 8);
        return got == was;
    }
    if (!VirtualProtect((void *)site, len, PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy((void *)site, rep, len);
    VirtualProtect((void *)site, len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, len);
    return 1;
}

static const char *brake_pedal_check(uintptr_t *site)
{
    if (!L->bp_pat || !*L->bp_pat) return "not verified on this game version";
    uintptr_t h = game_scan(L->bp_pat);
    if (!h) return "pattern (not exactly one hit)";
    if (game_is_161()) { *site = h + 8; return NULL; }   // the request read anchors the pattern; registers differ
    uintptr_t s = h + 3;
    if (!pat_at(L->bp_getter, s - 0x2d8)) return "pedal getter call at site-0x2d8";
    if (!brake_getter_ok(call_target(s - 0x2d8))) return "pedal getter body (+0x1cc0 / +0x1cc4 reads)";
    if (!pat_at(L->bp_req, s - 0x71)) return "brake request read at site-0x71";
    if (!pat_at(L->bp_hill, s - 0x8d)) return "hill-start max at site-0x8d";
    *site = s;
    return NULL;
}

static const char *brake_light_check(uintptr_t *site)
{
    uintptr_t h = game_scan(L->bl_pat);
    if (!h) return "pattern (not exactly one hit)";
    uintptr_t s = h + 4;
    if (!pat_at(L->bl_on, s - 0xd)) return "'on' store at site-0xd";
    if (!game_is_161()) {
        if (!pat_at(L->bl_getter, s - 0x67)) return "pedal getter call at site-0x67";
        if (!brake_getter_ok(call_target(s - 0x67))) return "pedal getter body (+0x1cc0 / +0x1cc4 reads)";
    }
    *site = s;
    return NULL;
}

static const char *retarder_feed_check(uintptr_t *site)
{
    uintptr_t h = game_scan(L->rf_pat);
    if (!h) return "pattern (not exactly one hit)";
    if (!pat_at(L->rf_before, h - 8)) return "rcx / comiss at site-8";
    if (!pat_at(L->rf_after, h + 0xc)) return "cmovbe at site+0xc";
    *site = h;
    return NULL;
}

void game_brake_hooks_start(int pedal, int light, int feed)
{
    int ok = L && L->game && (!strcmp(L->game, "eut2") || !strcmp(L->game, "ats")) && L->a_req && L->a_thr_pedal && L->a_brake_pedal;
    static unsigned char code[96];
    uintptr_t site = 0;
    const char *e;
    if (pedal) {
        if (!ok) logline("Brake pedal hook: not supported (no brake-pedal offsets): not installed");
        else {
            unsigned char orig[8]; int len;
            if (game_is_161()) { memcpy(orig, "\x48\x8B\x01\xF3\x0F\x5F\xCE", 7); len = 7; }   // mov rax,[rcx]; maxss xmm1,xmm6
            else { memcpy(orig, "\xF3\x41\x0F\x5F\xFE", 5); len = 5; }                          // maxss xmm7,xmm14
            if ((e = brake_pedal_check(&site)) != NULL) logline("Brake pedal hook: self-check failed (%s): not installed", e);
            else if (!patch_call_stub(site, len, orig, code,
                     game_is_161() ? brake_pedal_stub_code_161(code, (const void *)&game_brake_pedal_fn)
                                     : brake_pedal_stub_code(code, (const void *)&game_brake_pedal_fn),
                     !game_is_161()))   // 1.60's 5 bytes sit in one qword (one CAS); 1.61's 7 cross it (plain write at init)
                logline("Brake pedal hook: self-check failed (site bytes / aligned qword / trampoline): not installed");
            else { s_bp_in = 1; logline("Brake pedal hook: installed at %p (the raw pedal until a provider registers)", (void *)site); }
        }
    }
    if (light) {
        static const unsigned char orig[] = { 0x83, 0xA0, 0xA0, 0x02, 0x00, 0x00, 0xDF };   // and dword [rax+0x2a0],0xffffffdf
        if (!ok) logline("Brake light hook: not supported (no brake-pedal offsets): not installed");
        else if ((e = brake_light_check(&site)) != NULL) logline("Brake light hook: self-check failed (%s): not installed", e);
        else if (!patch_call_stub(site, 7, orig, code, brake_light_stub_code(code, &s_bl), 0))
            logline("Brake light hook: self-check failed (site bytes / trampoline): not installed");
        else { s_bl_in = 1; logline("Brake light hook: installed at %p (the game's lights until a request)", (void *)site); }
    }
    if (feed) {
        unsigned char orig[8] = { 0x8B, 0x00, 0x89, 0x81, 0, 0, 0, 0 };   // mov eax,[rax]; mov [rcx+retarder],eax
        unsigned r = L->i_retarder;
        for (int i = 0; i < 4; i++) orig[4 + i] = (unsigned char)(r >> (8 * i));
        if (!ok) logline("Retarder feed hook: not supported (no brake-pedal offsets): not installed");
        else if ((e = retarder_feed_check(&site)) != NULL) logline("Retarder feed hook: self-check failed (%s): not installed", e);
        else if (!patch_call_stub(site, 8, orig, code, retarder_feed_stub_code(code, &s_rf), 0))
            logline("Retarder feed hook: self-check failed (site bytes / trampoline): not installed");
        else { s_rf_in = 1; logline("Retarder feed hook: installed at %p (the game's retarder demand until a feed)", (void *)site); }
    }
}
