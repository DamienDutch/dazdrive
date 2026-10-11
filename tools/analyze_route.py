#!/usr/bin/env python3
"""Check a probe route dump against the driven path.

Usage: analyze_route.py <run dir> [route index]
Node coords are int, meters*256 (x, y=height, z). For every telemetry sample on the
route: nearest route segment in x/z, linear height between its nodes, error vs SDK y.
"""
import csv, math, sys, os

d = sys.argv[1]; k = int(sys.argv[2]) if len(sys.argv) > 2 else 0
rows = [r for r in csv.DictReader(l for l in open(os.path.join(d, f"route_{k}.csv")) if not l.startswith("#"))]
P = [(int(r["x"]) / 256, int(r["y"]) / 256, int(r["z"]) / 256, float(r["dist_end"])) for r in rows]
gaps = [math.dist((a[0], a[2]), (b[0], b[2])) for a, b in zip(P, P[1:])]
print(f"route_{k}: {len(P)} nodes, straight-line gaps m: median {sorted(gaps)[len(gaps)//2]:.0f}, max {max(gaps):.0f}")
print(f"  height range {min(p[1] for p in P):.1f} .. {max(p[1] for p in P):.1f} m")
tel = list(csv.DictReader(open(os.path.join(d, "telemetry.csv"))))
errs = []
for t in tel[::10]:
    x, y, z = float(t["x"]), float(t["y"]), float(t["z"])
    best = None
    for a, b in zip(P, P[1:]):
        dx, dz = b[0] - a[0], b[2] - a[2]
        L2 = dx * dx + dz * dz
        if L2 == 0: continue
        u = max(0, min(1, ((x - a[0]) * dx + (z - a[2]) * dz) / L2))
        px, pz = a[0] + u * dx, a[2] + u * dz
        dist = math.hypot(x - px, z - pz)
        if best is None or dist < best[0]:
            best = (dist, a[1] + u * (b[1] - a[1]), a[3] + u * (b[3] - a[3]))
    if best and best[0] < 30:
        errs.append((float(t["t"]), y - best[1], best[0], best[2]))
if errs:
    e = sorted(abs(x[1]) for x in errs)
    print(f"  {len(errs)} samples within 30 m of the node polyline; |height error| median {e[len(e)//2]:.1f} m, p90 {e[int(len(e)*.9)]:.1f} m, max {e[-1]:.1f} m")
else:
    print("  no telemetry samples near this route")
