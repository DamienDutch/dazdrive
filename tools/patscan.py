import sys, re, struct
pats = {
 "window (1)": "48 8B 46 10 80 B8 A8 14 00 00 00 74 ?? F2 0F 10 83 44 02 00 00",
 "bound (2)": "48 8B 41 10 0F 57 C9 F3 0F 10 15 ?? ?? ?? ?? 48 8B 90 F8 01 00 00 F3 0F 10 82 B0 01 00 00 F3 0F 58 82 A4 01 00 00",
 "lobound (1)": "40 53 48 83 EC 40 48 8B D9 0F 29 7C 24 20 48 8B 49 10 E8",
 "chooser (1)": "4C 63 B9 B0 14 00 00",
 "gate (1)": "F3 0F 59 81 F4 02 00 00 41 0F 2F C0 72 ??",
}
data = open(sys.argv[1], "rb").read()
# .text section
pe = struct.unpack_from("<I", data, 0x3C)[0]
nsec = struct.unpack_from("<H", data, pe+6)[0]; opt = struct.unpack_from("<H", data, pe+20)[0]
sec = pe+24+opt
for i in range(nsec):
    name, vsz, va, rsz, rptr = struct.unpack_from("<8sIIII", data, sec+40*i)
    if name.rstrip(b"\0") == b".text": text = data[rptr:rptr+rsz]
for k, p in pats.items():
    rx = b"".join(b"." if t == "??" else re.escape(bytes([int(t,16)])) for t in p.split())
    print(k, "->", len(re.findall(rx, text, re.S)), "hits")
