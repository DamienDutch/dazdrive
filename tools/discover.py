# Offline emulation of upstream discover() on an exe file, for a given pattern set.
import sys, re, struct, bisect
SETS = {
 "1.61": dict(window="48 8B 46 10 80 B8 A8 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00",
   bound="48 8B 41 10 0F 57 C9 F3 0F 10 15 ?? ?? ?? ?? 48 8B 90 F8 01 00 00 F3 0F 10 82 B0 01 00 00 F3 0F 58 82 A4 01 00 00",
   lobound="40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
   chooser="4C 63 B9 B0 14 00 00", gate="F3 0F 59 81 F4 02 00 00 41 0F 2F C0 72 ??"),
 "1.60": dict(window="48 8B 45 10 80 B8 40 14 00 00 00 74 ?? F2 0F 10 83 3C 02 00 00",
   bound="48 8B 41 10 0F 57 C9 F3 0F 10 15 ?? ?? ?? ?? 48 8B 90 F8 01 00 00 F3 0F 10 82 A8 01 00 00 F3 0F 58 82 9C 01 00 00",
   lobound="40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
   chooser="4C 63 B9 48 14 00 00", gate="F3 0F 59 82 F4 02 00 00 41 0F 2F C0 72 ??"),
}
d = open(sys.argv[1], "rb").read()
pe = struct.unpack_from("<I", d, 0x3C)[0]
nsec = struct.unpack_from("<H", d, pe+6)[0]; optsz = struct.unpack_from("<H", d, pe+20)[0]
opt = pe+24; base = struct.unpack_from("<Q", d, opt+24)[0]; imgsz = struct.unpack_from("<I", d, opt+56)[0]
img = bytearray(imgsz)
for i in range(nsec):
    n, vsz, va, rsz, rp = struct.unpack_from("<8sIIII", d, opt+optsz+40*i)
    img[va:va+min(vsz, rsz)] = d[rp:rp+min(vsz, rsz)]
    if n.rstrip(b"\0") == b".text": tva, tsz = va, vsz
exc_rva, exc_sz = struct.unpack_from("<II", d, opt+112+8*3)
rtf = [struct.unpack_from("<III", img, exc_rva+12*j) for j in range(exc_sz//12)]
begins = [r[0] for r in rtf]
def rx(p): return b"".join(b"." if t == "??" else re.escape(bytes([int(t, 16)])) for t in p.split())
text = bytes(img[tva:tva+tsz])
def scan(p): return [tva+m.start() for m in re.finditer(rx(p), text, re.S)]  # rvas
def idx(rva):
    i = bisect.bisect_right(begins, rva)-1
    return i if i >= 0 and rtf[i][0] <= rva < rtf[i][1] else -1
def primary(i):
    for _ in range(16):
        u = rtf[i][2]
        if not ((img[u] >> 3) & 4): return i
        codes = img[u+2]; off = 4+((codes+1)//2)*4
        ch = struct.unpack_from("<I", img, u+off)[0]; j = idx(ch)
        if j < 0 or j == i: return i
        i = j
    return i
def extent(rva):
    i = idx(rva)
    if i < 0: return None
    p = primary(i); lo = hi = i
    while lo > 0 and primary(lo-1) == p: lo -= 1
    while hi+1 < len(rtf) and primary(hi+1) == p: hi += 1
    return rtf[lo][0], rtf[hi][1]
def calls(target, lo, hi):
    out = []
    for a in range(lo, hi-5+1):
        if img[a] == 0xE8 and a+5+struct.unpack_from("<i", img, a+1)[0] == target: out.append(a)
    return out
def calls_text(target):
    out = []
    for m in re.finditer(b"\xE8", text):
        a = tva+m.start()
        if a+5+struct.unpack_from("<i", img, a+1)[0] == target: out.append(a)
    return out
def first_call(lo, hi):
    for a in range(lo, hi-5+1):
        if img[a] == 0xE8: return a+5+struct.unpack_from("<i", img, a+1)[0]
H = lambda r: hex(base+r)
def discover(P):
    h = scan(P["window"]);
    if len(h) != 1: return f"window pattern ({len(h)})"
    lo, hi = extent(h[0]); fn_window = lo; res = first_call(lo, hi)
    print("  window fn", H(fn_window), "extent", H(lo), H(hi), "resolver", H(res))
    h = scan(P["bound"])
    if len(h) != 2: return f"bound pattern ({len(h)})"
    e60 = ed0 = None
    for x in h:
        disc = bytes(img[x+0x40:x+0x43])
        if disc == b"\x8b\x41\x1c": e60 = x
        if disc == b"\x8b\x41\x20": ed0 = x
    if not (e60 and ed0): return "bound discriminator"
    h = scan(P["lobound"])
    if len(h) != 1: return f"lower bound pattern ({len(h)})"
    e70 = h[0]
    h = scan(P["chooser"])
    if len(h) != 1: return f"chooser pattern ({len(h)})"
    lo, hi = extent(h[0]); print("  chooser extent", H(lo), H(hi))
    c1, c2, c3 = calls(ed0, lo, hi), calls(e70, lo, hi), calls(e60, lo, hi)
    print("  e60", H(e60), "ed0", H(ed0), "e70", H(e70), "calls ed0/e70/e60:", len(c1), len(c2), len(c3))
    if len(c1) != 2: return "kick/up call sites"
    if len(c2) != 2: return "kick/down call sites"
    if len(c3) != 1: return "shared call site"
    w = calls_text(fn_window); print("  window call sites", len(w), [H(x) for x in w])
    if not w: return "window call sites"
    h = scan(P["gate"])
    if len(h) != 1: return f"gate pattern ({len(h)})"
    if img[h[0]+12] != 0x72: return "gate opcode"
    print("  gate jb at", H(h[0]+12))
    return "OK"
for name in sys.argv[2:]:
    print(f"[{sys.argv[1]} with {name} set]"); print("  ->", discover(SETS[name]))
