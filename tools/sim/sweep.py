#!/usr/bin/env python3
"""Sweep ini keys in the native sims (needs SIM_SET, tools/sim/sim_env.h) and print one table per key.

  tools/sim/sweep.py ts   trueshift.choice_rpm_weight 0,0.25,0.5,1     (SCORES run: tools/sim/sim, build it with run_scores.sh first)
  tools/sim/sweep.py raw  cruise.ahead_accel 0.06,0.12,0.24 sim_cruise  (the whole output of a sim binary, grep what you need with --grep)

Several keys together: key=v1,v2 can be repeated and are swept one at a time against the defaults.
Options: --grep REGEX (raw mode: print matching lines of each run), -j N jobs (default 16), --env K=V extra environment.
Mode agg (any sim binary as the last argument): --agg 'name@op@regex' (op mean|max|min|sum|count) over the output lines.
Mode ts extracts per run: P13 totals (score costs), P2 pull-away sums for choice 1, the 70 km/h 'choice 1, time' rows, and FAIL lines.
"""
import os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))


def run(binary, setting, extra_env, args=()):
    env = dict(os.environ, **extra_env)
    if setting.startswith("ENV:"):   # ENV:NAME=value sets an environment variable of the sim (SC_SHARES, LCT_OVER ...) instead of SIM_SET
        k, v = setting[4:].split("=", 1); env[k] = v
    elif setting:
        env["SIM_SET"] = setting
    p = subprocess.run([os.path.join(HERE, binary), *args], capture_output=True, text=True, env=env)
    return p.stdout + p.stderr, p.returncode


def nums(s):
    return [float(x) for x in re.findall(r"-?\d+\.?\d*", s)]


def ts_metrics(out):
    m = {}
    for l in out.splitlines():
        if l.startswith("  total score costs"):
            n = nums(l.split(":", 1)[1].replace("up_pt+150", "").replace("t30-70", "tt"))
            m["p13 shifts"], m["p13 skips"], m["p13 drop"], m["p13 land"], m["p13 over_s"], m["p13 t30-70"] = n[0], n[1], n[2], n[3], n[4], n[5]
    lines = out.splitlines()
    for i, l in enumerate(lines):
        if l.startswith("  pull-away, 14 rows"):
            n = nums(lines[i + 1])
            m["p2 shifts"], m["p2 skips"], m["p2 shift_s"], m["p2 rpm"], m["p2 t50"], m["p2 hunts"] = n[1], n[2], n[3], n[4], n[5], n[6]
    rowmap = {"33 t pedal 0.7": "N", "33 t pedal 0.7 Eco": "E", "33 t pedal 0.7 Power": "P", "17 t pedal 0.3": "N17"}
    for l in lines:
        if "choice 1, time" in l:
            head = " ".join(l.split("choice 1, time")[0].split())
            if head in rowmap:
                tag = rowmap[head]
                n = nums(l.split("choice 1, time")[1])
                m[f"{tag} sh/sk"], m[f"{tag} t50"], m[f"{tag} t70"] = f"{int(n[0])}/{int(n[1])}", n[3], n[4]
    for l in lines:
        r = re.search(r"Normal skips to a gear short of the force asked in (\d+) of (\d+) upshifts", l)
        if r: m["p3 short"] = f"{r.group(1)}/{r.group(2)}"
        r = re.search(r"full-pedal launch 33 t with the score costs: (\d+) skips in (\d+) shifts", l)
        if r: m["launch sk/sh"] = f"{r.group(1)}/{r.group(2)}"
    m["FAIL"] = sum(1 for l in lines if l.startswith("  FAIL"))
    return m


def agg_metrics(out, rc, aggs):
    """--agg name@op@regex (op: mean max min sum count): over all output lines the regex matches (first group = the number)."""
    m = {"exit": rc}
    for name, op, rx in aggs:
        v = [(float(r.group(r.lastindex)) if op != "count" else 1.0) for r in (re.search(rx, l) for l in out.splitlines()) if r]
        m[name] = len(v) if op == "count" else (round({"mean": lambda x: sum(x) / len(x), "max": max, "min": min, "sum": sum}[op](v), 2) if v else float("nan"))
    return m


def table(rows, keys):
    cols = ["value"] + keys
    w = [max(len(c), max(len(str(r[i])) for r in rows)) for i, c in enumerate(cols)]
    print("  " + "  ".join(c.rjust(w[i]) for i, c in enumerate(cols)))
    for r in rows:
        print("  " + "  ".join(str(x).rjust(w[i]) for i, x in enumerate(r)))


def main():
    a = sys.argv[1:]
    jobs, grep, extra = 16, None, {}
    while "-j" in a: i = a.index("-j"); jobs = int(a[i + 1]); del a[i:i + 2]
    cols = None
    while "--cols" in a: i = a.index("--cols"); cols = a[i + 1].split(","); del a[i:i + 2]
    while "--grep" in a: i = a.index("--grep"); grep = a[i + 1]; del a[i:i + 2]
    while "--env" in a: i = a.index("--env"); k, v = a[i + 1].split("="); extra[k] = v; del a[i:i + 2]
    aggs = []
    while "--agg" in a: i = a.index("--agg"); aggs.append(a[i + 1].split("@", 2)); del a[i:i + 2]
    mode = a[0]
    if mode == "ts":
        extra["SCORES"] = "1"
        binary = "sim"
        sweeps = a[1:]
    else:
        binary = a[-1]
        sweeps = a[1:-1]
    for sw in sweeps:
        key, vals = sw.split(" ")[0], None
        if "=" in key:
            key, vals = key.split("=", 1)
        else:
            raise SystemExit("use key=v1,v2,...")
        vals = vals.split(",")
        with ThreadPoolExecutor(jobs) as ex:
            res = list(ex.map(lambda v: run(binary, f"{key}={v}", extra), vals))
        print(f"\n{key}")
        if mode == "agg":
            ms = [agg_metrics(o, rc, aggs) for o, rc in res]
            table([[v] + [m[n] for n, _, _ in aggs] + [m["exit"]] for v, m in zip(vals, ms)], [n for n, _, _ in aggs] + ["exit"])
        elif mode == "ts":
            ms = [ts_metrics(o) for o, _ in res]
            keys = [k for k in ms[0].keys() if cols is None or k in cols]
            table([[v] + [(round(m[k], 2) if isinstance(m.get(k), float) else m.get(k, 'nan')) for k in keys] for v, m in zip(vals, ms)], keys)
        else:
            for v, (o, rc) in zip(vals, res):
                print(f"--- {key}={v} (exit {rc})")
                for l in o.splitlines():
                    if grep is None or re.search(grep, l):
                        print(l[:220])


if __name__ == "__main__":
    main()
