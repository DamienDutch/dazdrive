# Summary of a TrueShift test ride: python3 tools/analyze_trueshift.py <dir with trueshift.csv and dazdrive.log>
# Shifts per reason, hunting (opposite shifts within 5 s), shifts per km, time per gear, hint holds, blocked upshifts.
import csv, sys, os, re, collections

d = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(d, "trueshift.csv"))))
log = open(os.path.join(d, "dazdrive.log")).read().splitlines()
print(f"trace: {len(rows)} rows, {float(rows[-1]['t']) - float(rows[0]['t']):.0f} s" if rows else "trace empty")

shifts = []
for line in log:
    m = re.match(r"\s*([\d.]+)\s+shift (-?\d+)->(-?\d+) \(([^)]*)\)", line)
    if m: shifts.append((float(m[1]), int(m[2]), int(m[3]), m[4]))
    elif " by hint " in line:
        m = re.match(r"\s*([\d.]+)\s+shift (-?\d+)->(-?\d+)", line)
        shifts.append((float(m[1]), int(m[2]), int(m[3]), "hint"))
print(f"shifts: {len(shifts)}")
for why, n in collections.Counter(s[3] for s in shifts).most_common(): print(f"  {n:4d}  {why}")

hunt = [(a, b) for a, b in zip(shifts, shifts[1:])
        if b[0] - a[0] < 5 and (a[2] - a[1]) * (b[2] - b[1]) < 0 and a[3] != "hint" and b[3] != "hint"]
print(f"hunting pairs (opposite shift < 5 s): {len(hunt)}")
for a, b in hunt[:15]: print(f"  {a[0]:.1f} {a[1]}->{a[2]} ({a[3]})  then {b[0]:.1f} {b[1]}->{b[2]} ({b[3]})")

if rows:
    km = 0
    for p, q in zip(rows, rows[1:]): km += float(q["kmh"]) / 3600 * (float(q["t"]) - float(p["t"]))
    print(f"distance {km:.1f} km, {len(shifts) / km:.1f} shifts/km" if km > 0.1 else "distance < 0.1 km")
    tg = collections.Counter()
    for p, q in zip(rows, rows[1:]): tg[int(p["gear"])] += float(q["t"]) - float(p["t"])
    print("time per gear: " + ", ".join(f"{g}:{t:.0f}s" for g, t in sorted(tg.items())))
    why = collections.Counter(r["why"] for r in rows)
    print("decision states: " + ", ".join(f"{w or '-'} {n}" for w, n in why.most_common(10)))

for key in ("mass ", "start gear", "up blocked", "no pull after", "load changed", "mode ", "dashboard letter", "engine:", "truck config", "not active", "not recognized"):
    hits = [l for l in log if key in l]
    if hits:
        print(f"[{key.strip()}] {len(hits)}x, first: {hits[0].strip()}")
