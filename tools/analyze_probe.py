#!/usr/bin/env python3
"""Analyze the output of ets2ra_probe (bin/win_x64/plugins/ets2ra_probe/).

Usage: analyze_probe.py <probe dir> [--find FLOAT ...]

Prints:
  - status and SDK truck config
  - per object (self/inner/engine): 4-byte fields that track a telemetry value (|r| > 0.97)
  - constant floats in ranges that look like torque, power, rpm or mass
  - rare-changing words around SDK gear changes
  - with --find: every place in snaps and deep dumps where a float is within 0.5% of a value
"""
import csv, math, os, struct, sys
from collections import defaultdict

REG = ["self", "inner", "engine"]


def read_snaps(path):
    snaps = []
    with open(path, "rb") as f:
        d = f.read()
    i = 0
    while i + 12 <= len(d) and d[i:i+4] == b"SNAP":
        t, = struct.unpack_from("<d", d, i+4); i += 12
        regs = []
        for _ in range(3):
            addr, ln = struct.unpack_from("<QI", d, i); i += 12
            regs.append((addr, d[i:i+ln])); i += ln
        snaps.append((t, regs))
    return snaps


def read_deep(path):
    with open(path, "rb") as f:
        d = f.read()
    if d[:4] != b"DEEP":
        return []
    i = 12; out = []
    while i + 24 <= len(d) and d[i:i+4] == b"PTR ":
        r, o, p, ln = struct.unpack_from("<IIQI", d, i+4); i += 24
        out.append((REG[r], o, p, d[i:i+ln])); i += ln
    return out


def read_tel(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            try: r[k] = float(v)
            except ValueError: r[k] = math.nan
    return rows


def corr(a, b):
    n = len(a)
    if n < 10: return 0.0
    ma, mb = sum(a)/n, sum(b)/n
    va = sum((x-ma)**2 for x in a); vb = sum((y-mb)**2 for y in b)
    if va <= 0 or vb <= 0: return 0.0
    return sum((x-ma)*(y-mb) for x, y in zip(a, b)) / math.sqrt(va*vb)


def nearest(rows, t):
    lo, hi = 0, len(rows)-1
    while lo < hi:
        m = (lo+hi)//2
        if rows[m]["t"] < t: lo = m+1
        else: hi = m
    return rows[lo]


def floats(buf):
    return [struct.unpack_from("<f", buf, o)[0] for o in range(0, len(buf)-3, 4)]


def main():
    d = sys.argv[1]
    finds = [float(x) for x in sys.argv[sys.argv.index("--find")+1:]] if "--find" in sys.argv else []
    print(open(os.path.join(d, "status.txt")).read())
    cfg = open(os.path.join(d, "config.txt")).read()
    last_truck = cfg[cfg.rfind("] truck"):] if "] truck" in cfg else ""
    print("SDK truck config (last):")
    for line in last_truck.splitlines()[1:]:
        if line.startswith("  ") and any(k in line for k in ("brand", "name", "rpm.limit", "gears.", "differential", "forward.ratio")):
            print(line)
    for line in cfg.splitlines():
        if "cargo.mass" in line or "cargo =" in line: print(line)

    snaps = read_snaps(os.path.join(d, "snaps.bin"))
    tel = [r for r in read_tel(os.path.join(d, "telemetry.csv")) if not math.isnan(r["rpm"])]
    print(f"\n{len(snaps)} snapshots, {len(tel)} telemetry rows")
    if not snaps or not tel: return

    # fields tracking telemetry
    keys = ["rpm", "kmh", "eff_thr", "in_thr", "gear", "pitch", "acc_z", "eff_brake", "wheel0"]
    tv = {k: [nearest(tel, t)[k] for t, _ in snaps] for k in keys}
    for ri, rn in enumerate(REG):
        ln = min(len(s[1][ri][1]) for s in snaps)
        if not ln: continue
        series = [floats(s[1][ri][1][:ln]) for s in snaps]
        print(f"\n== {rn}: {ln:#x} bytes")
        for o in range(ln // 4):
            col = [s[o] for s in series]
            if any(not math.isfinite(x) or abs(x) > 1e7 for x in col): continue
            if max(col) - min(col) < 1e-6:
                v = col[0]
                tags = []
                if 900 <= v <= 6000 and abs(v - round(v)) < 1e-3: tags.append("torque/rpm?")
                if 150 <= v <= 800 and abs(v - round(v)) < 1e-3: tags.append("power?")
                if 4000 <= v <= 60000 and abs(v - round(v)) < 1: tags.append("mass?")
                if tags: print(f"  const {rn}+{o*4:#06x} = {v:g}  {' '.join(tags)}")
                continue
            for k in keys:
                r = corr(col, tv[k])
                if abs(r) > 0.97:
                    print(f"  {rn}+{o*4:#06x} tracks {k:9s} r={r:+.3f}  range {min(col):.4g}..{max(col):.4g}")

    # rare events around gear changes
    ev_path = os.path.join(d, "events.csv")
    if os.path.exists(ev_path):
        cnt = defaultdict(int)
        with open(ev_path) as f:
            for r in csv.DictReader(f):
                cnt[(r["region"], r["offset"])] += 1
        rare = sorted((n, k) for k, n in cnt.items() if n <= 40)
        print(f"\n{len(rare)} rarely changing words (<= 40 changes):")
        for n, (rg, off) in rare[:80]:
            print(f"  {rg}+{off}: {n} changes")

    if finds:
        print("\nSearch:")
        allblocks = [(f"snap0 {REG[i]}", 0, b) for i, (_, b) in enumerate(snaps[0][1])]
        for name in sorted(os.listdir(d)):
            if name.startswith("deep_"):
                allblocks += [(f"{name} {rg}+{o:#x} -> {p:#x}", 0, b) for rg, o, p, b in read_deep(os.path.join(d, name))]
        for want in finds:
            for label, _, b in allblocks:
                for o in range(0, len(b)-3, 4):
                    v = struct.unpack_from("<f", b, o)[0]
                    if math.isfinite(v) and want and abs(v - want) <= abs(want) * 0.005:
                        print(f"  {want:g}: {label} +{o:#x} = {v:g}")


if __name__ == "__main__":
    main()
