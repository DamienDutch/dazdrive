#!/usr/bin/env python3
"""One-page summary of a ride: what the gearbox, the pedal, the brakes and lane keeping did.

Usage: analyze_ride.py <folder with dazdrive.log and trueshift.csv> [--since SECONDS]

Reads only what the plugin already logs (log_level 2 gives the most). Sections:
session, shifts (reasons, upshift rpm per mode and pedal), kickdown, steady-driving rpm,
stop assist (with the brake lights), brake gain, lane (key, calibration), events.
"""
import csv, os, re, statistics, sys
from collections import Counter, defaultdict

LINE = re.compile(r"^\s*(\d+\.\d+)\s+(.*)$")
SHIFT = re.compile(r"shift (\d+)->(\d+) \(([^)]*)\) at (\d+) km/h, (\d+) rpm, pedal ([\d.]+), demand ([\d.-]+), "
                   r"grade ([\d.-]+)%, ([\d.]+) t, (\w+)")
UP_REASONS = {"up", "up skip", "up early", "up early skip", "up on lift", "wish unmet above", "rev limit", "rev limit skip"}
CSV_MODE = {"D": "Normal", "E": "Eco", "S": "Power", "M": "M"}   # trueshift.csv letters (S = Sport = Power)
PEDAL_BINS = [(0.0, 0.5, "<0.50"), (0.5, 0.8, "0.50-0.80"), (0.8, 0.95, "0.80-0.95"), (0.95, 1.01, ">=0.95")]


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * p / 100
    f = int(k)
    return xs[f] if f + 1 >= len(xs) else xs[f] + (xs[f + 1] - xs[f]) * (k - f)


def read_log(path, since):
    out = []
    with open(path, errors="replace") as f:
        for raw in f:
            m = LINE.match(raw.rstrip("\n"))
            if m and float(m.group(1)) >= since:
                out.append((float(m.group(1)), m.group(2)))
    return out


def first(lines, needle):
    return next((t for _, t in lines if needle in t), None)


def all_of(lines, needle):
    return [(s, t) for s, t in lines if needle in t]


def section(title):
    print(f"\n== {title}")


def session(lines):
    section("Session")
    for needle in ("dazdrive ", "truck config: ", "engine: "):
        hits = [t for _, t in all_of(lines, needle) if not t.startswith("truck config: 0 forward")]
        for t in dict.fromkeys(hits):   # each distinct truck/engine once
            print("  " + t[:200])
    for t in dict.fromkeys(t for _, t in all_of(lines, "Pedal: curve")):
        print("  " + t)
    for t in (t for _, t in lines if t.startswith("ini: ")):
        print("  " + t)
    modes = Counter(t for _, t in lines if re.match(r"mode (Eco|Normal|Power)", t))
    if modes:
        print("  mode changes: " + ", ".join(f"{k.split()[1]} x{v}" for k, v in modes.items()))


def shifts(lines):
    rows = []
    for s, t in lines:
        m = SHIFT.search(t)
        if m:
            rows.append(dict(t=s, frm=int(m[1]), to=int(m[2]), why=m[3], kmh=int(m[4]), rpm=int(m[5]),
                             pedal=float(m[6]), grade=float(m[8]), mass=float(m[9]), mode=m[10]))
    section(f"Shifts ({len(rows)})")
    if not rows:
        print("  none logged")
        return rows
    print("  reasons: " + ", ".join(f"{k} {v}" for k, v in Counter(r["why"] for r in rows).most_common()))
    ups = [r for r in rows if r["why"] in UP_REASONS and r["to"] > r["frm"]]
    skips = Counter(r["to"] - r["frm"] for r in ups)
    print("  upshift steps: " + ", ".join(f"+{k} x{v}" for k, v in sorted(skips.items())))
    print("  upshift rpm at the command (median / max, count) per mode and pedal:")
    by = defaultdict(list)
    for r in ups:
        b = next(lbl for lo, hi, lbl in PEDAL_BINS if lo <= r["pedal"] < hi)
        by[(r["mode"], b)].append(r["rpm"])
    for mode in ("Eco", "Normal", "Power"):
        cells = []
        for _, _, b in PEDAL_BINS:
            xs = by.get((mode, b))
            cells.append(f"{b} {statistics.median(xs):.0f}/{max(xs)} ({len(xs)})" if xs else f"{b} -")
        if any(by.get((mode, b)) for _, _, b in PEDAL_BINS):
            print(f"    {mode:6} " + " | ".join(cells))
    lifts = [r for r in rows if r["why"] == "up on lift"]
    if lifts:
        print(f"  up on lift: {len(lifts)}, at " + ", ".join(f"{r['rpm']} rpm/{r['kmh']} km/h" for r in lifts[:8]))
    return rows


def kickdown(lines, rows):
    section("Kickdown")
    kd = [r for r in rows if r["why"] in ("kickdown", "M: kickdown")]
    if not kd:
        print("  none")
    for mode in ("Eco", "Normal", "Power"):
        xs = [r for r in kd if r["mode"] == mode]
        if xs:
            print(f"  {mode:6} {len(xs)}: " + ", ".join(f"{r['frm']}->{r['to']} {r['kmh']} km/h ped {r['pedal']:.2f}" for r in xs[:6])
                  + (" ..." if len(xs) > 6 else ""))
    wish = all_of(lines, "kickdown by wish")
    if wish:
        rat = [float(m[1]) for _, t in wish if (m := re.search(r"ratio ([\d.]+)", t))]
        print(f"  by wish: {len(wish)} decisions, ratio median {statistics.median(rat):.2f} (min {min(rat):.2f})" if rat else
              f"  by wish: {len(wish)} decisions")
        for s, t in wish[:5]:
            print(f"    {s:8.1f}  {t[:190]}")


def steady_rpm(folder, since):
    path = os.path.join(folder, "trueshift.csv")
    if not os.path.exists(path):
        return
    section("Steady driving rpm (trueshift.csv: >30 km/h, |accel| < 0.15, no brake, clutch closed)")
    by, fl = defaultdict(list), defaultdict(list)
    dt, last = [], None
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                t = float(r["t"])
                if last is not None and 0 < t - last < 1:
                    dt.append(t - last)
                last = t
                if float(r["t"]) < since or float(r["kmh"]) < 30 or abs(float(r["accel"])) > 0.15:
                    continue
                if float(r["brake"] or 0) > 0.02 or float(r.get("clutch") or 0) > 0.05 or int(float(r["gear"])) <= 0:
                    continue
                src = "cruise" if float(r["cruise_kmh"] or 0) > 0 else "pedal"
                mode = CSV_MODE.get(r["mode"], r["mode"])
                by[(mode, src)].append(float(r["rpm"]))
                if float(r.get("floor") or 0) > 0:
                    fl[(mode, src)].append(float(r["floor"]))
            except (KeyError, ValueError):
                continue
    step = statistics.median(dt) if dt else 0.05
    if not by:
        print("  no samples")
    for (mode, src), xs in sorted(by.items()):
        f = fl.get((mode, src))
        floor = f"  floor p50 {pct(f, 50):4.0f}" if f else ""
        print(f"  {mode:6} {src:6} p10 {pct(xs, 10):5.0f}  p50 {pct(xs, 50):5.0f}  p90 {pct(xs, 90):5.0f}{floor}  ({len(xs) * step:.0f} s)")


def floor_lines(lines):
    fl = all_of(lines, "choice floor:")
    if fl:
        rpm = [float(m[1]) for _, t in fl if (m := re.search(r": (\d+) rpm", t))]
        print(f"  choice floor lines: {len(fl)}, floor rpm median {statistics.median(rpm):.0f} (range {min(rpm):.0f}-{max(rpm):.0f})")


def stop_assist(lines):
    section("Stop assist")
    sa = all_of(lines, "Decel: stop assist")
    if not sa:
        print("  no runs logged (needs log_level 2 and [decel] stop_assist 1 or 2)")
    for s, t in sa:
        m = re.search(r"from ([\d.]+) km/h.*?mean ([\d.-]+) m/s2 over ([\d.]+) s(?: \(brake lights on (\d+) % of it\))?; "
                      r"ended at ([\d.]+) km/h: (.*)", t)
        if m:
            lights = f"lights {m[4]:>3} %" if m[4] is not None else "lights ?"
            print(f"  {s:8.1f}  {m[1]:>5} -> {m[5]:>4} km/h, {m[2]} m/s2 over {m[3]} s, {lights}: {m[6]}")
        else:
            print(f"  {s:8.1f}  {t[:190]}")


def brakes(lines):
    section("Brake gain and decel")
    for s, t in lines:
        if t.startswith("Brake gain:") or t.startswith("Decel: brake gain") or "stop assist cannot reach" in t:
            print(f"  {s:8.1f}  {t[:190]}")


def lane(lines):
    section("Lane")
    key = first(lines, "the lane-keeping key is the action at")
    print("  " + (key if key else "lane key: not discovered"))
    for s, t in lines:
        if t.startswith("Lane calibration:") and "waiting" not in t:
            print(f"  {s:8.1f}  {t[:230]}")
        elif t.startswith("Lane learn: new profile") or "wheel loop has changed" in t:
            print(f"  {s:8.1f}  {t[:190]}")


def events(lines):
    section("Events")
    c = Counter()
    for _, t in lines:
        if t.startswith("AEB: warning") or t.startswith("AEB: partial") or t.startswith("AEB: full"):
            c["AEB " + t.split(":")[1].strip().split()[0]] += 1
        if "incident file" in t:
            c["incident files"] += 1
        if "runaway" in t.lower():
            c["runaway"] += 1
        if t.startswith("ini: ") and "unknown key" in t:
            c["unknown ini keys"] += 1
    print("  " + (", ".join(f"{k} {v}" for k, v in c.items()) if c else "nothing"))


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    since = 0.0
    if "--since" in args:
        i = args.index("--since")
        since = float(args[i + 1])
        del args[i:i + 2]
    folder = args[0]
    lines = read_log(os.path.join(folder, "dazdrive.log"), since)
    session(lines)
    rows = shifts(lines)
    kickdown(lines, rows)
    steady_rpm(folder, since)
    floor_lines(lines)
    stop_assist(lines)
    brakes(lines)
    lane(lines)
    events(lines)


if __name__ == "__main__":
    main()
