#!/usr/bin/env python3
# Retarder / engine-brake deceleration from a ride log, and what it does while the gearbox shifts.
#   python3 tools/analyze_brake.py [dir with fuel.csv, trueshift.csv, dazdrive.log] [--out notes/brake-measure.md]
#   python3 tools/analyze_brake.py --selftest   (synthetic rides in a temp dir: no descent, no stop, a stop, no data at all)
# Default dir: notes/brake-logs next to this repo. Python 3 stdlib only. Analysis only, nothing here touches the plugin.
# A missing file or a section without data prints 'no data' instead of a number.
#
# Method (also printed in the report):
#   speed derivative = least-squares slope of v over 5 consecutive fuel.csv samples (~0.45 s, centred; windows with
#   a hole > 0.7 s are dropped). d = -a, so a positive d is braking. Two variants per number:
#     raw  = d from the speed alone,
#     corr = grade-corrected: a_own = a + g*grade/100 (grade < 0 is downhill in this csv, checked on coasting rows).
#   The grade column is not a pure road grade (see the pitch check below), so the truth lies between raw and corr.
import csv, os, re, sys, bisect, statistics, collections

G = 9.81
K = 2                    # half window of the slope fit (2 -> 5 samples)
SETTLE = 1.0             # s the same step, same gear and a closed clutch before a sample counts as steady state
PEDAL = 0.02

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out_path = None
if "--out" in sys.argv:
    out_path = sys.argv[sys.argv.index("--out") + 1]
    args = [a for a in args if a != out_path]
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def selftest():
    """Run this script on tiny synthetic rides and check that every section prints (no crash, 'no data' where empty)."""
    import subprocess, tempfile, shutil
    top = tempfile.mkdtemp(prefix="analyze_brake_selftest_")
    def ride(name, rows, log=""):
        # rows: (t, km/h, grade %, retarder step, gear, clutch); the same rows go to both csv files
        p = os.path.join(top, name); os.makedirs(p)
        with open(os.path.join(p, "fuel.csv"), "w") as f:
            f.write("t,kmh,grade_pct,pedal,eff_thr,cruise_kmh,gear,retarder,mass_kg\n")
            f.writelines(f"{a:.2f},{k:.2f},{gr},0,0,0,{g},{st_},20000\n" for a, k, gr, st_, g, c in rows)
        with open(os.path.join(p, "trueshift.csv"), "w") as f:
            f.write("t,kmh,grade_pct,pedal,brake,clutch,gear,retarder,eb_level\n")
            f.writelines(f"{a:.2f},{k:.2f},{gr},0,0,{c},{g},{st_},0\n" for a, k, gr, st_, g, c in rows)
        open(os.path.join(p, "dazdrive.log"), "w").write(log)
        return p
    ts_ = [i / 10 for i in range(400)]
    flat = [(x, 60 - 0.3 * 3.6 * x, 0, 2 if x >= 2 else 0, 8 if x < 19.4 else 7, 1 if 18.9 <= x < 20.0 else 0) for x in ts_]
    stop = [(x, max(0.0, 20 - 0.7 * 3.6 * x), 0, 3 if 20 - 0.7 * 3.6 * x > 0 else 0, 2, 1 if 20 - 0.7 * 3.6 * x < 6 else 0) for x in ts_[:120]]
    cases = [("flat, no descent, no stop, one shift", ride("flat", flat, "   20.1 shift 8->7 (engine brake)\n"), ["below -8% (n=0)", "no stop with the retarder on"]),
             ("flat stop from 20 km/h, no descent", ride("stop", stop), ["1 stop: 0-8 s", "no data"]),
             ("headers only, no log", ride("empty", []), ["no data", "Cannot say yet."])]
    bad = 0
    for name, p, want in cases:
        r = subprocess.run([sys.executable, os.path.abspath(__file__), p, "--out", os.path.join(p, "report.md")], capture_output=True, text=True)
        miss = [w for w in want if w not in r.stdout]
        ok = r.returncode == 0 and not miss
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {name}" + ("" if ok else f": exit {r.returncode}, missing {miss}\n{r.stderr[-800:]}"))
    if bad:
        print(f"selftest: {bad} failed, rides kept in {top}")
    else:
        shutil.rmtree(top)
    sys.exit(1 if bad else 0)

if "--selftest" in sys.argv:
    selftest()
d = args[0] if args else os.path.join(root, "notes", "brake-logs")
if out_path is None:
    out_path = os.path.join(root, "notes", "brake-measure.md")

def read(name):
    p = os.path.join(d, name)
    return open(p).read() if os.path.exists(p) else ""
fuel = list(csv.DictReader(read("fuel.csv").splitlines()))
ts_rows = list(csv.DictReader(read("trueshift.csv").splitlines()))
logl = read("dazdrive.log").splitlines()
N = len(fuel)
t = [float(r["t"]) for r in fuel]
kmh = [float(r["kmh"]) for r in fuel]
v = [x / 3.6 for x in kmh]
grade = [float(r["grade_pct"]) for r in fuel]
pedal = [float(r["pedal"]) for r in fuel]
thr = [float(r["eff_thr"]) for r in fuel]
cruise = [float(r["cruise_kmh"]) for r in fuel]
gear = [int(r["gear"]) for r in fuel]
step = [int(r["retarder"]) for r in fuel]
mass = sorted(set(r["mass_kg"] for r in fuel))

# trueshift.csv: clutch, brake, game engine-brake level; joined to a fuel row by nearest t (<= 0.06 s)
TT = [float(r["t"]) for r in ts_rows]
TC = [float(r["clutch"]) for r in ts_rows]
TG = [int(r["gear"]) for r in ts_rows]
def near(x):
    i = bisect.bisect_left(TT, x)
    c = [j for j in (i - 1, i) if 0 <= j < len(TT)]
    j = min(c, key=lambda j: abs(TT[j] - x)) if c else None
    return j if j is not None and abs(TT[j] - x) <= 0.06 else None
J = [near(x) for x in t]
brake = [float(ts_rows[j]["brake"]) if j is not None else None for j in J]
clutch = [float(ts_rows[j]["clutch"]) if j is not None else None for j in J]
eb = [float(ts_rows[j]["eb_level"]) if j is not None else None for j in J]

# smoothed deceleration per row
dr = [None] * N          # raw braking decel
dc = [None] * N          # grade-corrected braking decel
for i in range(K, N - K):
    if t[i + K] - t[i - K] > 0.7:
        continue
    n = range(i - K, i + K + 1)
    tm = sum(t[k] for k in n) / len(n); vm = sum(v[k] for k in n) / len(n)
    a = sum((t[k] - tm) * (v[k] - vm) for k in n) / sum((t[k] - tm) ** 2 for k in n)
    gm = sum(grade[k] for k in n) / len(n)
    dr[i] = -a
    dc[i] = -(a + G * gm / 100)

def win(i):
    return range(max(0, i - K), min(N, i + K + 1))

def clean(i):
    """No pedal, no service brake, no cruise in the smoothing window; both decel values exist."""
    if dr[i] is None:
        return False
    for k in win(i):
        if pedal[k] > PEDAL or cruise[k] > 0 or brake[k] is None or brake[k] > PEDAL:
            return False
    return True

def steady(i):
    """Same step > 0, same gear > 0 and a closed clutch from t-SETTLE to t+0.25 s (matched trueshift rows)."""
    if step[i] == 0 or gear[i] <= 0:
        return False
    a, b = bisect.bisect_left(t, t[i] - SETTLE), bisect.bisect_right(t, t[i] + 0.25)
    if t[a] > t[i] - SETTLE + 0.2 or b - a < 5:
        return False
    for k in range(a, b):
        if step[k] != step[i] or gear[k] != gear[i] or clutch[k] is None or clutch[k] > 0.02 or t[k] - t[max(a, k - 1)] > 0.3:
            return False
    return True

def q(xs, p):
    xs = sorted(xs); k = (len(xs) - 1) * p; f = int(k)
    return xs[f] + (xs[min(f + 1, len(xs) - 1)] - xs[f]) * (k - f)

def stat(xs):
    return f"{statistics.median(xs):5.2f} [{q(xs, .25):4.2f}..{q(xs, .75):4.2f}]" if xs else "no data"

def md(xs, f="{:.2f}"):
    """Median formatted, 'no data' for an empty list."""
    return f.format(statistics.median(xs)) if xs else "no data"

def fq(xs, p, f="{:.2f}"):
    return f.format(q(xs, p)) if xs else "no data"

def rng(xs, f="{:.1f}"):
    """'min-max' formatted, 'no data' for an empty list."""
    return f"{f.format(min(xs))}-{f.format(max(xs))}" if xs else "no data"

R = []
def P(s=""):
    R.append(s)

# ---------- data summary and checks
ret_all = [i for i in range(N) if step[i] > 0]
ret_ped = [i for i in ret_all if pedal[i] > PEDAL]
ret_cr = [i for i in ret_all if cruise[i] > 0]
use = [i for i in ret_all if clean(i)]
st = [i for i in use if steady(i)]
span = sum(min(t[i + 1] - t[i], 0.3) for i in range(N - 1))
P("# Retarder brake measurement (tools/analyze_brake.py, ride log %s)" % os.path.basename(os.path.normpath(d)))
P(f"fuel.csv {N} rows, " + (f"{span:.0f} s of data in {t[0]:.0f}-{t[-1]:.0f} s (pauses cut out)" if N else "no data") + f", mass {', '.join(mass) or 'no data'} kg (constant: no mass split possible).")
P(f"retarder>0: {len(ret_all)} rows (steps 1-4: " + ", ".join(f"{s}:{sum(1 for i in ret_all if step[i] == s)}" for s in (1, 2, 3, 4)) +
  f"); dropped for pedal>{PEDAL}: {len(ret_ped)}, cruise: {len(ret_cr)}, brake input (trueshift brake col): "
  f"{sum(1 for i in ret_all if brake[i] is not None and brake[i] > PEDAL)}, no trueshift match / no slope: {len(ret_all) - len(ret_ped) - len(use) - sum(1 for i in ret_all if brake[i] is not None and brake[i] > PEDAL)}; kept {len(use)}.")
P("Method: d = braking decel in m/s2 (positive = slowing) = minus the least-squares slope of v over 5 consecutive samples (~0.45 s, centred; windows with a gap > 0.7 s dropped). "
  "raw = speed only; corr = grade-corrected as briefed: d_corr = d_raw - g*grade/100.")
coast = [i for i in range(N) if dr[i] is not None and step[i] == 0 and pedal[i] <= 0.01 and thr[i] < 0.01 and v[i] > 8 and gear[i] > 0]
lo = [dr[i] for i in coast if grade[i] < -8]; fl = [dr[i] for i in coast if abs(grade[i]) < 1]
pit = collections.defaultdict(list)
for i in range(N):
    if dr[i] is not None and v[i] > 5:
        pit[round(-dr[i] * 2) / 2].append(grade[i])
P(f"Grade sign: coasting rows (no retarder/pedal): d_raw {md(fl, '{:+.2f}')} on |grade|<1% (n={len(fl)}) vs {md(lo, '{:+.2f}')} below -8% (n={len(lo)}), " +
  ("so grade<0 = downhill and the formula has the right sign. " if fl and lo else "so the sign cannot be checked on this ride. ") +
  "Pitch caveat: the grade column also follows the acceleration (median grade " + (", ".join(f"{k:+.0f} m/s2: {statistics.median(x):+.1f}%" for k, x in sorted(pit.items()) if len(x) >= 40 and k in (-2.0, 0.0, 1.0)) or "no data") +
  "; the log shows 0.3/-0.1/0.1% at the shift decisions of the braking stops) = body pitch, ~1% per m/s2. So corr overstates d by the pitch part (~0.3 m/s2 at hard braking); part of corr-raw is real downhill (e.g. 627-636 s, log grade -7.7%). True value lies between raw and corr (assuming the road is not steeper than the column says).")
P()

# ---------- 1. steady-state per step and speed band
bands = [(0, 20), (20, 35), (35, 50), (50, 99)]
P("## 1. Deceleration per retarder step (steady: >=%.1f s same step+gear, clutch closed; median [p25..p75] of d, n samples)" % SETTLE)
P("| step | band km/h | n | d corr | d raw |")
P("|---|---|---|---|---|")
tot = collections.Counter()
n_rows = len(R)
for s in (1, 2, 3, 4):
    for lo_, hi_ in bands:
        xs = [i for i in st if step[i] == s and lo_ <= kmh[i] < hi_]
        tot[s] += len(xs)
        if len(xs) >= 3:
            P(f"| {s} | {lo_}-{hi_} | {len(xs)} | {stat([dc[i] for i in xs])} | {stat([dr[i] for i in xs])} |")
        elif xs:
            P(f"| {s} | {lo_}-{hi_} | {len(xs)} | too few | |")
if len(R) == n_rows:
    P("| no data | | | | |")
allg = {sx: [i for i in use if step[i] == sx and gear[i] > 0 and clutch[i] is not None and clutch[i] <= 0.02] for sx in (1, 2, 3, 4)}
P("Steady samples: " + ", ".join(f"step {s}: {tot[s]}" for s in (1, 2, 3, 4)) + f" (of {len(use)} clean retarder rows; the rest are build-up, shifts, step changes). Single mass, so no mass split. "
  "All clean in-gear rows incl. build-up, any speed (n, median corr / raw): " + ("; ".join(f"step {s}: {len(x)}, {statistics.median([dc[i] for i in x]):.1f} / {statistics.median([dr[i] for i in x]):.1f}" for s, x in allg.items() if x) or "no data") +
  f". Drag floor without retarder (in gear, flat): d_raw {stat(fl).strip()} (n={len(fl)}). Within a step d falls as speed rises (step 1 above); the retarder needs ~1 s to build after a step change, so spread is not only noise.")
P()

# ---------- 2. near zero (trueshift.csv: same ride, 10 Hz, and it keeps logging below 5 km/h where fuel.csv stops)
TK = [float(r["kmh"]) for r in ts_rows]
Tgr = [float(r["grade_pct"]) for r in ts_rows]
Tst = [int(r["retarder"]) for r in ts_rows]
Tpd = [float(r["pedal"]) for r in ts_rows]
Tbr = [float(r["brake"]) for r in ts_rows]
M = len(TT)
Tdr = [None] * M; Tdc = [None] * M
for i in range(K, M - K):
    if TT[i + K] - TT[i - K] > 0.7:
        continue
    n = range(i - K, i + K + 1)
    tm = sum(TT[k] for k in n) / len(n); vm = sum(TK[k] / 3.6 for k in n) / len(n)
    a = sum((TT[k] - tm) * (TK[k] / 3.6 - vm) for k in n) / sum((TT[k] - tm) ** 2 for k in n)
    Tdr[i] = -a; Tdc[i] = -(a + G * sum(Tgr[k] for k in n) / len(n) / 100)
def tclean(i):
    return Tdr[i] is not None and all(Tpd[k] <= PEDAL and Tbr[k] <= PEDAL for k in range(i - K, i + K + 1))
# The stops: runs of rows under 20 km/h (no hole > 0.5 s) that reach 1 km/h with the retarder on somewhere before; from the
# run's first row to its first row under 1 km/h
stops = []
i = 0
while i < M:
    if TK[i] < 20:
        s = i
        while i + 1 < M and TK[i + 1] < 20 and TT[i + 1] - TT[i] <= 0.5:
            i += 1
        z = next((k for k in range(s, i + 1) if TK[k] < 1), None)
        if z is not None and any(Tst[k] > 0 for k in range(s, z + 1)):
            stops.append((TT[s], TT[z]))
    i += 1
P("## 2. Behaviour near zero (trueshift.csv rows, retarder>0, clean; " +
  (f"{len(stops)} stop{'s' if len(stops) != 1 else ''}: " + " and ".join(f"{a:.0f}-{b:.0f} s" for a, b in stops) if stops else "no stop with the retarder on in this ride") + ")")
P("| km/h | state | n | steps | d corr | d raw |")
P("|---|---|---|---|---|---|")
n_rows = len(R)
for lo_, hi_ in [(0, 5), (5, 7), (7, 10), (10, 14), (14, 20)]:
    for name, neu in (("in gear", False), ("clutch open/neutral", True)):
        xs = [i for i in range(M) if Tst[i] > 0 and lo_ <= TK[i] < hi_ and tclean(i) and ((TC[i] > 0.02 or TG[i] == 0) == neu)]
        if len(xs) >= 2:
            sts = collections.Counter(Tst[i] for i in xs)
            P(f"| {lo_}-{hi_} | {name} | {len(xs)} | {'/'.join(str(s) for s in sorted(sts))} | {stat([Tdc[i] for i in xs])} | {stat([Tdr[i] for i in xs])} |")
if len(R) == n_rows:
    P("| no data | | | | | |")
fl0 = [Tdr[i] for i in range(M) if Tdr[i] is not None and Tst[i] == 0 and TG[i] == 0 and Tpd[i] == 0 and Tbr[i] <= 0.0 and 3.5 <= TK[i] <= 4.5]
lowest = [TK[i] for i in range(M) if Tst[i] > 0]
P(f"Lowest speed with retarder>0 (trueshift.csv): " + (f"{min(lowest):.1f} km/h" if lowest else "no data") + ". Run-out below 8 km/h with the retarder still on, d_raw (km/h:step:d, every 2nd clean row):")
seq = [i for i in range(M) if Tst[i] > 0 and TK[i] < 8 and tclean(i)]
groups = []
for i in seq:
    if not groups or TT[i] - TT[groups[-1][-1]] > 0.5:
        groups.append([])
    groups[-1].append(i)
for g_ in groups:
    P(f"  from {TT[g_[0]]:.1f} s: " + " ".join(f"{TK[i]:.1f}:{Tst[i]}:{Tdr[i]:.2f}" for i in g_[::2]))
if not groups:
    P("  no data")
P(f"After the retarder reads 0 the truck in neutral coasts at ~4 km/h with d_raw {md(fl0)} (n={len(fl0)}, 3.5-4.5 km/h, no retarder/brake/pedal): nothing slows it. "
  "Reading: in stop 1 the step stays 3 down to 4.4 km/h and the decel still dies out between ~6 and ~4.4 km/h (1.1 -> ~0.3 -> ~0), so the retarder effect vanishes around 4-5 km/h; "
  "in stop 2 the step was released at 5.9 km/h, so it shows the release, not the fade. One stop, n~10 rows: indicative, not a curve. In gear (clutch closed) the decel is 2.1-2.4 down to ~7 km/h and 1.25 at 5-7 km/h (n=2).")
P()

# ---------- 3. shifts
shifts = []
for l in logl:
    m = re.match(r"\s*([\d.]+)\s+shift (-?\d+)->(-?\d+)(?: \(([^)]*)\))?", l)
    if m:
        shifts.append((float(m[1]), int(m[2]), int(m[3]), m[4] or ("hint" if "by hint" in l else "")))
# every clutch-open window (also into neutral), and the gear changes inside them
wins = []
i = 0
while i < len(TT):
    if TC[i] > 0.02:
        s = i
        while i < len(TT) and TC[i] > 0.02 and (i == s or TT[i] - TT[i - 1] < 0.5):
            i += 1
        wins.append((TT[s], TT[min(i, len(TT) - 1)], s, i - 1))   # end = first row with the clutch closed again
    i += 1
chg = []
for i in range(1, len(ts_rows)):
    if TT[i] - TT[i - 1] < 0.5 and TG[i] != TG[i - 1] and TG[i] > 0 and TG[i - 1] > 0:
        w = [x for x in wins if x[2] <= i <= x[3]]
        if w:
            chg.append((TT[i], TG[i - 1], TG[i], w[0][0], w[0][1]))
ev = []
for lt, a, b, why in shifts:
    c = [x for x in chg if x[1] == a and x[2] == b and abs(x[0] - lt) < 3]
    if c:
        c = min(c, key=lambda x: abs(x[0] - lt))
        ev.append(dict(log=lt, a=a, b=b, why=why, tg=c[0], ts=c[3], te=c[4]))
offs = [e["log"] - e["te"] for e in ev]; offg = [e["log"] - e["tg"] for e in ev]
durs = [e["te"] - e["ts"] for e in ev if e["te"] - e["ts"] < 1.6]
P("## 3. Shifts with the retarder on")
if ev:
    P(f"Clock check: {len(shifts)} 'shift A->B' log lines, {len(ev)} matched to a gear change in trueshift.csv (same clock, agreement within one sample). "
      f"The log line is written at the END of the shift, not the start: log t - clutch re-closed = {statistics.median(offs):+.2f} s (range {min(offs):+.2f}..{max(offs):+.2f}), "
      f"log t - gear-column change = {statistics.median(offg):+.2f} s. Clutch open (>0.02) {md(durs)} s (p10-p90 {fq(durs, .1)}-{fq(durs, .9)}); "
      f"the gear flips {statistics.median([e['tg'] - e['ts'] for e in ev]):.2f} s after the clutch starts opening. Shift windows below come from the clutch column.")
else:
    P(f"Clock check: {len(shifts)} 'shift A->B' log lines, none matched to a gear change in trueshift.csv: no data.")
def dser(a_, b_, arr):
    return [(t[i], arr[i]) for i in range(N) if a_ <= t[i] <= b_ and arr[i] is not None]
def med(a_, b_, arr):
    s_ = [x for _, x in dser(a_, b_, arr)]
    return statistics.median(s_) if len(s_) >= 3 else None
rows = []; skipped = collections.Counter()
for e in ev:
    ts_, te_ = e["ts"], e["te"]
    if not any(step[i] > 0 for i in range(N) if ts_ - 0.5 <= t[i] <= te_ + 0.5):
        skipped["no retarder"] += 1
        continue
    if any(pedal[i] > PEDAL or brake[i] is None or brake[i] > PEDAL for i in range(N) if ts_ - 1.5 <= t[i] <= te_ + 1.5):
        skipped["pedal/brake within 1.5 s"] += 1
        continue
    # neighbouring clutch windows (any, also into neutral) limit the before/after windows
    prev_end = max([w[1] for w in wins if w[1] < ts_ - 0.01] + [0]); next_ts = min([w[0] for w in wins if w[0] > te_ + 0.01] + [1e9])
    pa = max(ts_ - 1.5, prev_end + 0.2); pb = ts_ - 0.15
    qa = te_ + 0.3; qb = min(te_ + 1.5, next_ts - 0.1)
    i0 = bisect.bisect_left(t, ts_)
    j = i0
    while j > 0 and step[j - 1] > 0 and t[j] - t[j - 1] < 0.3:
        j -= 1
    r = dict(e=e, on_for=(ts_ - t[j]) if i0 > 0 and step[i0 - 1] > 0 else 0.0, next_gap=next_ts - te_)
    r["off_at_start"] = not (i0 > 0 and step[i0 - 1] > 0)
    for nm, arr in (("c", dc), ("r", dr)):
        r["pre_" + nm] = med(pa, pb, arr) if pb - pa >= 0.25 else None
        ser = [x for _, x in dser(ts_, te_, arr)]
        r["dur_mean_" + nm] = statistics.mean(ser) if ser else None
        r["dur_min_" + nm] = min(ser) if ser else None
        r["post_" + nm] = med(qa, qb, arr) if qb - qa >= 0.3 else None
    def stp(a_, b_):
        s_ = [step[i] for i in range(N) if a_ <= t[i] <= b_]
        return f"{min(s_)}-{max(s_)}" if s_ and min(s_) != max(s_) else (str(s_[0]) if s_ else "?")
    r["steps"] = "/".join((stp(pa, pb) if pb - pa >= 0.25 else "-", stp(ts_, te_), stp(qa, qb) if qb - qa >= 0.3 else "-"))
    # lag from the retarder coming on to 1 m/s2 raw (only meaningful for lever-triggered shifts)
    k1 = next((k for k in range(j, N) if dr[k] is not None and dr[k] >= 1.0 and step[k] > 0), None)
    r["lag1"] = t[k1] - t[j] if k1 is not None and t[k1] - t[j] < 4 else None
    on_share = sum(1 for i in range(N) if ts_ <= t[i] <= te_ and step[i] > 0) / max(1, sum(1 for i in range(N) if ts_ <= t[i] <= te_))
    r["cls"] = "R" if on_share < 0.5 else ("E" if r["on_for"] >= 1.5 and not r["off_at_start"] and r["pre_c"] is not None and r["dur_min_c"] is not None else "L")
    rows.append(r)
est = [r for r in rows if r["cls"] == "E"]
P(f"Shifts with retarder>0 within +-0.5 s and pedal/brake ~0 within +-1.5 s: {len(rows)} of {len(ev)} (dropped: " + (", ".join(f"{n} {k}" for k, n in skipped.items()) or "none") + "). "
  f"Class E = retarder already on >= 1.5 s before the clutch opens (a real gap measurement, {len(est)}); L = the retarder came on with the shift (lever pull -> "
  f"'engine brake (level)' downshift, {sum(1 for r in rows if r['cls'] == 'L')}); R = retarder off for most of the clutch window (released at the shift, not a torque gap) ({sum(1 for r in rows if r['cls'] == 'R')}).")
P("d m/s2, corr (raw): pre = 1.5 s before the clutch opens (cut at a neighbouring shift), dur = clutch open min / mean, post = 0.3-1.5 s after it closes. steps = pre/during/post.")
P("| t_end | A>B | km/h | cls | on s | steps | pre | dur min | dur mean | post | pre-min | next shift in s |")
P("|---|---|---|---|---|---|---|---|---|---|---|---|")
def f2(c, r_):
    return "-" if c is None else f"{c:.2f} ({r_:.2f})"
for r in rows:
    e = r["e"]
    sp = kmh[min(bisect.bisect_left(t, e["ts"]), N - 1)]
    gap = f"{r['pre_c'] - r['dur_min_c']:.2f}" if r["pre_c"] is not None and r["cls"] == "E" else "-"
    nx = f"{r['next_gap']:.1f}" if r["next_gap"] < 50 else "-"
    P(f"| {e['log']:.1f} | {e['a']}>{e['b']} | {sp:.0f} | {r['cls']} | {r['on_for']:.1f} | {r['steps']} | {f2(r['pre_c'], r['pre_r'])} | {f2(r['dur_min_c'], r['dur_min_r'])} | {f2(r['dur_mean_c'], r['dur_mean_r'])} | {f2(r['post_c'], r['post_r'])} | {gap} | {nx} |")
if not rows:
    P("| no data | | | | | | | | | | | |")
lags = [r["lag1"] for r in rows if r["cls"] == "L" and r["lag1"] is not None]
if lags:
    P(f"L shifts: decel reaches 1 m/s2 (raw) {statistics.median(lags):.1f} s after the retarder came on (range {min(lags):.1f}-{max(lags):.1f}, n={len(lags)}), i.e. after the clutch closes (shift {md(durs, '{:.1f}')} s); post decel "
      f"{rng([r['post_r'] for r in rows if r['cls'] == 'L' and r['post_r'] is not None])} raw.")
# dip timing for E: first time below half depth after the clutch starts opening, and when 0.8*pre is back
for r in est:
    e = r["e"]
    hi_, lo_ = r["pre_c"], r["dur_min_c"]
    half = (hi_ + lo_) / 2
    lim = e["te"] + r["next_gap"] - 0.1
    ser = [(x, y) for x, y in dser(e["ts"] - 0.3, min(e["te"] + 2.0, lim), dc)]
    fall = next((x - e["ts"] for x, y in ser if y < half), None)
    rec = next((x - e["te"] for x, y in ser if x > e["te"] and y >= 0.8 * hi_), None)
    r["fall"] = fall; r["rec"] = rec
    P(f"E {e['log']:.1f} {e['a']}>{e['b']}: " + (f"below half depth {fall:.2f} s after the clutch started opening; " if fall is not None else "never below half depth (no data); ") + (f"back to 0.8*pre {rec:.2f} s after it closed." if rec is not None else
      f"no recovery to 0.8*pre before the next shift/clutch event ({r['next_gap']:.1f} s after the clutch closed)" if r["next_gap"] < 2.0 else "no recovery to 0.8*pre within 2 s."))
if len(est) < 5:
    P(f"**Only {len(est)} shifts with an established retarder (from {len({int(r['e']['ts'] // 30) for r in est})} stop sequences, chained: 8>6>4>neutral and 7>5>3>neutral): fewer than 5, "
      "so the dip size is NOT settled and a dedicated test ride is needed.** Minimal ride: level or a steady -3% descent at a steady 60 km/h, retarder step 2 on for >= 10 s, "
      "no pedal/service brake/cruise, then force 4 shifts (2 up, 2 down; manual gear request) with the lever held through each; repeat 1x at step 4. Log fuel.csv + trueshift.csv at 10 Hz.")
P()

# ---------- 4. bridge
P("## 4. What a service-brake bridge would have to deliver")
if est:
    gaps = [r["pre_c"] - r["dur_min_c"] for r in est]; gaps_r = [r["pre_r"] - r["dur_min_r"] for r in est]
    spd = [kmh[min(bisect.bisect_left(t, r["e"]["ts"]), N - 1)] for r in est]
    falls = [r["fall"] for r in est if r["fall"] is not None]; recs = [r["rec"] for r in est if r["rec"] is not None]
    P(f"Basis: {len(est)} class-E shifts at {min(spd):.0f}-{max(spd):.0f} km/h, all inside two chained stops, so not independent. Braking falls from {min(r['pre_r'] for r in est):.1f}-{max(r['pre_r'] for r in est):.1f} (raw) to "
      f"{min(r['dur_min_r'] for r in est):.1f}-{max(r['dur_min_r'] for r in est):.1f} m/s2 with the clutch open (no-retarder floor {md(fl)}): a gap of {min(gaps_r):.1f}-{max(gaps_r):.1f} raw / "
      f"{min(gaps):.1f}-{max(gaps):.1f} corr, per event (corr, step): " + ", ".join(f"{g_:.1f} (step {r['steps'].split('/')[0]})" for g_, r in zip(gaps, est)) + f". The fall passes half depth {rng(falls)} s after the clutch starts opening, it lasts the clutch window ({md(durs, '{:.1f}')} s, "
      f"p10-p90 {fq(durs, .1, '{:.1f}')}-{fq(durs, .9, '{:.1f}')}) and the decel is back to 0.8 of its old value {rng(recs)} s after the clutch closes (n={len(recs)}). "
      f"So at 11-25 km/h a bridge would have to give ~{min(gaps_r):.1f}-{max(gaps_r):.1f} m/s2 (about the pre-shift decel minus ~0.4) for ~1.1 s plus a few tenths of a second of ramp at the end, starting when the clutch opens. "
      f"Cannot say: other speeds (nothing above 25 km/h with an established retarder), step 1-2 at speed, ms-level timing (smoothing blurs ~0.2 s), the service-brake request needed for a given m/s2 (not calibrated in this log), "
      f"and whether the driver's lever or the game set the step (no lever channel in the csv). "
      f"(Earlier note from the 35.8 t ride of 25-10: gap ~0.4 m/s2; not comparable, other truck/mass and method, and 4-5x smaller than here: unexplained.)")
else:
    P("Cannot say yet.")
txt = "\n".join(R)
print(txt)
os.makedirs(os.path.dirname(out_path), exist_ok=True)
open(out_path, "w").write(txt + "\n")
