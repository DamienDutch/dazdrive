# Offline emulation of the suite's game_discover() (plugin/dazdrive/core/game.cpp)
# on an exe file. Usage: python3 tools/check_suite.py <eurotrucks2.exe|amtrucks.exe>
# Keep the patterns in sync with g_layouts in game.cpp.
import sys, re, struct, bisect

LAYOUTS = {
    "1.60": dict(
        window="48 8B 45 10 80 B8 40 14 00 00 00 74 ?? F2 0F 10 83 3C 02 00 00",
        chooser="4C 63 B9 48 14 00 00",
        lobound="40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
        mass="48 8B 05 ?? ?? ?? ?? B2 01 48 8B 88 ?? ?? ?? ?? E8",
        target="89 46 68", force="80 BE 19 01 00 00 00", decide="44 8B 80 48 14 00 00",
        torque="F3 41 0F 59 BE C0 01 00 00",
        adaptive="48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 41 0F 2F C0",
        gps="48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54"),
    # ATS 1.61 (docs/ats-161.md): gearbox object grew a float at +0x68 (target 0x70, force 0x121), inner +0x68
    "1.61": dict(
        window="48 8B 46 10 80 B8 A8 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00",
        chooser="4C 63 B9 B0 14 00 00",
        lobound="40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
        mass="48 8B 05 ?? ?? ?? ?? B2 01 48 8B 88 ?? ?? ?? ?? E8",
        target="89 46 70", force="80 BE 21 01 00 00 00", decide="44 8B 80 B0 14 00 00",
        torque="F3 44 0F 59 A6 C8 01 00 00",
        adaptive="48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 41 0F 2F C0",
        gps="48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54"),
}

d = open(sys.argv[1], "rb").read()
pe = struct.unpack_from("<I", d, 0x3C)[0]
nsec = struct.unpack_from("<H", d, pe + 6)[0]; optsz = struct.unpack_from("<H", d, pe + 20)[0]
opt = pe + 24; base = struct.unpack_from("<Q", d, opt + 24)[0]; imgsz = struct.unpack_from("<I", d, opt + 56)[0]
img = bytearray(imgsz)
for i in range(nsec):
    n, vsz, va, rsz, rp = struct.unpack_from("<8sIIII", d, opt + optsz + 40 * i)
    img[va:va + min(vsz, rsz)] = d[rp:rp + min(vsz, rsz)]
    if n.rstrip(b"\0") == b".text": tva, tsz = va, vsz
exc_rva, exc_sz = struct.unpack_from("<II", d, opt + 112 + 8 * 3)
rtf = [struct.unpack_from("<III", img, exc_rva + 12 * j) for j in range(exc_sz // 12)]
begins = [r[0] for r in rtf]

def rx(p): return b"".join(b"." if t == "??" else re.escape(bytes([int(t, 16)])) for t in p.split())
def scan(p, lo=None, hi=None):
    lo = tva if lo is None else lo; hi = tva + tsz if hi is None else hi
    return [lo + m.start() for m in re.finditer(rx(p), bytes(img[lo:hi]), re.S)]
def at(p, a): return re.fullmatch(rx(p), bytes(img[a:a + len(p.split())]), re.S) is not None
def idx(rva):
    i = bisect.bisect_right(begins, rva) - 1
    return i if i >= 0 and rtf[i][0] <= rva < rtf[i][1] else -1
def primary(i):
    for _ in range(16):
        u = rtf[i][2]
        if not ((img[u] >> 3) & 4): return i
        off = 4 + ((img[u + 2] + 1) // 2) * 4
        j = idx(struct.unpack_from("<I", img, u + off)[0])
        if j < 0 or j == i: return i
        i = j
    return i
def extent(rva):
    i = idx(rva); p = primary(i); lo = hi = i
    while lo > 0 and primary(lo - 1) == p: lo -= 1
    while hi + 1 < len(rtf) and primary(hi + 1) == p: hi += 1
    return rtf[lo][0], rtf[hi][1]
def target(a): return a + 5 + struct.unpack_from("<i", img, a + 1)[0]
def calls(t, lo=None, hi=None):
    lo = tva if lo is None else lo; hi = tva + tsz if hi is None else hi
    return [lo + m.start() for m in re.finditer(b"\xE8", bytes(img[lo:hi])) if target(lo + m.start()) == t]
H = lambda r: hex(base + r)

def discover(P):
    h = scan(P["window"])
    if len(h) != 1: return f"window pattern ({len(h)})"
    window = extent(h[0])[0]
    h = scan(P["chooser"])
    if len(h) != 1: return f"chooser pattern ({len(h)})"
    clo, chi = extent(h[0])
    if len(scan(P["torque"], clo, chi)) != 1: return "torque check"
    cs = calls(clo)
    if len(cs) != 2: return f"chooser call sites ({len(cs)})"
    ulo, uhi = extent(cs[0])
    if not all(ulo <= c < uhi for c in cs): return "chooser callers differ"
    if not any(at(P["target"], c + 5) for c in cs): return "target gear check"
    if not scan(P["force"], ulo, uhi): return "force flag check"
    ds = calls(window, ulo, uhi)
    if len(ds) != 1: return f"decide site ({len(ds)})"
    if not at(P["decide"], ds[0] - 7): return "decide site check"
    h = scan(P["lobound"])
    if len(h) != 1: return "lower bound pattern"
    lob = h[0]
    if img[lob + 18] != 0xE8: return "grade fn call"
    grade = target(lob + 18)
    ls = calls(lob, ulo, uhi)
    if len(ls) != 2: return f"lower bound calls in update ({len(ls)})"
    start = None
    for s in ls:
        if img[s - 10] != 0xE8 or not at("48 8B CE 8B D8", s - 5): return "start-gear call"
        t = target(s - 10)
        if start and t != start: return "start-gear fns differ"
        start = t
    ss = calls(start)
    if len(ss) < 2: return "start-gear call sites"
    h = scan(P["mass"], ulo, uhi)
    if len(h) != 1: return f"mass pattern ({len(h)})"
    slot = h[0] + 7 + struct.unpack_from("<i", img, h[0] + 3)[0]
    ga = struct.unpack_from("<I", img, h[0] + 12)[0]
    mass = target(h[0] + 16)
    h = scan(P["adaptive"], ulo, uhi)
    cvar = H(h[0] + 7 + struct.unpack_from("<i", img, h[0] + 3)[0]) if len(h) == 1 else f"not found ({len(h)})"
    print(f"  chooser {H(clo)} sites {[H(c) for c in cs]}, update {H(ulo)}..{H(uhi)}")
    print(f"  decide site {H(ds[0])}, start fn {H(start)} sites {[H(s) for s in ss]}")
    g = scan(P["gps"]); offs = {struct.unpack_from("<I", img, a + 3)[0] for a in g}
    gps = f"{len(g)} hits, gps_manager +{offs.pop():#x}" if len(offs) == 1 else f"not found/ambiguous ({len(g)} hits)"
    print(f"  gps pattern: {gps}")
    print(f"  grade fn {H(grade)}, mass fn {H(mass)}, base_ctrl slot {H(slot)}, game_actor +{ga:#x}, adaptive cvar {cvar}")
    return "OK"

# A layout belongs to one game (game.cpp binds it to the SDK game_id): only the layouts of the exe's game are tried.
GAME = {"1.60": "eurotrucks2", "1.61": "amtrucks"}
exe_game = "amtrucks" if "amtrucks" in sys.argv[1].lower() else "eurotrucks2"
ok = 0
for name, P in LAYOUTS.items():
    if GAME[name] != exe_game: continue
    print(f"[{name} layout]")
    r = discover(P)
    print("  ->", r)
    ok += r == "OK"
if not ok:
    print("no layout matches this exe: the plugin would stay inactive on it")
    sys.exit(1)
