#!/usr/bin/env python3
"""Generate the DazDrive ini presets from the param tables in the C++ sources, so they never drift.

Usage:
  tools/gen_ini.py dev      [-o FILE]   the developer's compiled-in defaults, same layout as the DLL's own default ini
  tools/gen_ini.py standard [-o FILE]   preset for other players = the ~2010 year of the setup page: stable features on
                                        (not the ones that look ahead), beta and experimental off, no button numbers
  tools/gen_ini.py inc      [-o FILE]   the standard preset as a C table (the DLL's fallback when the ini or a key is missing)
  tools/gen_ini.py inc-ini  [-o FILE]   the standard ini text as C string literals (the DLL writes it when there is no ini)
  tools/gen_ini.py html     [-o FILE]   self-contained dazdrive-setup.html (browser setup page, same tables, no deps)
  tools/gen_ini.py --list               every section.key, default and help (for checking)

Parses  { "key", &p_var, default, "help" },  in `static param g_params[]` of core/config.cpp and modules/*.cpp, in the
order of g_modules[] in core/main.cpp. Run from anywhere. A key that STANDARD overrides but that no longer exists is an
error (the preset would silently stop doing what it says).

The code and this file use the internal (module, key); what the ini calls a setting ([section] key, true/false, words) is in
tools/ini_names.tsv (gen_ini_names.py). Every param needs a row there, or this script stops: a new param without a public
name cannot reach the build. The generated ini holds the user and advanced settings, sub-section by sub-section; the internal
ones (the tuning constants) are left out, but still read when a player adds them.
"""
import json, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import gen_ini_names

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "plugin", "dazdrive"))
VERSION = re.search(r'#define SUITE_VERSION "([^"]*)"', open(f"{ROOT}/core/suite.h").read()).group(1)

# --- the "standard" preset: what a player gets who just drops the DLL in. Everything not listed keeps the compiled
# default (= the developer's), so a key that is ON in the standard preset is simply not listed unless it needs a comment.
# Since 15-11 (owner): the standard preset is the owner's own ini, what he drives every day ("owner's ini = the standard"
# below), except the log sizes and his own 90 km/h limiter. It is the "DazDrive" tile of the setup page (ERAS "dd", no
# overrides, build_html checks it); the truck years keep what they meant before (_YEARS_BASE). The look-ahead items
# (lookahead, curves, curve_brake, predictive), junctions and lane centering stay off: the gearbox first on its own (10-11).
STANDARD = {
    # Players: a quiet log plus incident files (what went wrong, with the 20 s before and 15 s after). At most ~6 MB in all:
    # the log 0.6 MB x (log, .1, previous session) and 4 MB of incidents. The developer's compiled defaults are 10x that.
    "core": {"log_level": 2, "log_max_mb": 0.6, "incident_mb": 4, "brake_learn": 1, "retarder_hook": 1, "retarder_feed_hook": 1, "brake_light_hook": 1, "brake_pedal_hook": 1},   # BETA: bug hunt on by default until the first full release (make_release refuses a full release with it); brake_learn ON (owner 03-11, WP-M2: the brake model learns and starts from the mass); the two retarder hooks ON (owner 03-11: the channel feeds the retarder and Decel scales it)
    "trueshift": {
        "takeover_rpm": 1000,     # owner's ini = the standard (15-11)
        "takeover_kmh": 7,        # owner's ini = the standard (15-11)
        "handback_kmh": 4,        # owner's ini = the standard (15-11)
        "eco_roll": 1,           # ON: neutral roll where the learned fuel model says it saves (never with brake/retarder/engine brake); ridden
        "lookahead": 0,          # off (also the compiled default since 09-11: the gearbox is good on its own, the route comes back as the PCC layer on top): the ~2010 year of the setup page; the page turns it on from ~2015
        "no_creep": 1,           # ON: with crawl off the truck never creeps on the clutch at idle (the stock automatic did)
        "smooth_stop": 1,        # ON: clutch opens around idle when braking to a stop (no pushing down to 2 km/h); ridden
        "dash_letter": 1,        # ON: D/E/P/M on the dashboard instead of A
        "tones": 1,              # ON: short tone when the mode changes
        "trace": 1,              # csv traces (need log_level 2): ON in the betas for bug reports, off in the first full release
    },
    "carshift": {                # the car automatic (modules/carshift.cpp): the same proven handover and switches as TrueShift's, nothing of the truck-only ones
        "takeover_rpm": 1000,
        "takeover_kmh": 7,
        "handback_kmh": 4,
        "no_creep": 1,
        "smooth_stop": 1,
        "dash_letter": 1,
        "tones": 1,
        "trace": 1,
    },
    "cruise": {
        "descent_overspeed_kmh": 8, # owner's ini = the standard (15-11)
        "auto_hold_park_s": 60,   # owner's ini = the standard (15-11)
        "smooth": 1,             # ON: own cruise throttle, no bursts after shifts: part of what TrueShift is for
        "curves": 0,             # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2020
        "curve_brake": 0,        # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2020
        "descent": 1,            # ON: hold the set speed downhill with engine brake/retarder; ridden
        "emergency_hazards": 1,  # ON: hazards after an emergency stop
        "air_min_psi": 75,       # ON: own brake requests stop below this brake air pressure (guard)
        "limiter_kmh": 0,        # ON: set speed never above the game's speed limiter (its own value, truck 90; 100 for a bus)
        "acc_calm_cut_in": 1,    # ON: someone cutting in or pulling away does not make the ACC speed up/slow down hard; ridden
        "acc_cycle_up": 1,       # ON: distance key order C -> 1 -> 2 -> 3 (the game: far, middle, close, off); ridden
        "step_tap_kmh": 5,       # ON: tap on + / - = 5 km/h step
        "step_hold_kmh": 1,      # ON: hold on + / - = 1 km/h steps
        "retarder_off_on_set": 1,# ON: retarder lever on is switched off when you press set/resume (the game refuses/cancels cruise)
        "hill_start": 1,         # ON: brake holds briefly after stopping on a slope (only effective while auto_hold is 0)
        "min_kmh": 5,             # owner's ini = the standard (15-11) (a jam use case)
        "min_set_kmh": 20,        # owner's ini = the standard (15-11)
        "predictive": 0,         # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2015
        "junctions": 0,          # OFF: junctions/roundabouts: roundabout detection is a heuristic, not ridden enough
        "own_acc": 1,             # ON: owner's ini = the standard (15-11)
        "overtake_assist": 1,    # ON: with the own ACC on, the left signal prepares a lane change (closes in, pulls toward the set speed)
        "auto_hold": 1,           # ON: owner's ini = the standard (15-11)
        "stop_go": 1,             # ON: owner's ini = the standard (15-11)
        "speed_limit": 1,         # ON: ISA follows the limit; owner's ini = the standard (15-11)
        "speed_limit_ahead": 1,   # ON: owner's ini = the standard (15-11)
        "resume_to_limit": 1,     # ON: owner's ini = the standard (15-11)
        "speed_limit_probe": 0,  # OFF: test-ride probe
        "trace": 1,
        "acc_short_s": 0.9,       # owner's ini = the standard (15-11)
        "acc_middle_s": 1.15,     # owner's ini = the standard (15-11)
        "acc_long_s": 1.4,        # owner's ini = the standard (15-11): the "Dutch" preset of the setup page
        "curve_lat": 2.325,       # the "Dutch" preset (with the gaps above)
        "aeb_mode": 1,            # required deceleration; owner's ini = the standard (15-11)
        "bend_now": 0,            # OFF: owner's ini = the standard (15-11)
        "engine": 1,              # layers: owner's ini = the standard (15-11)
        "lim_show_set": 1,        # owner's ini = the standard (15-11)
        "rollover_guard": 0,      # OFF: owner's ini = the standard (15-11)
        "stop_go_auto_s": 2,      # owner's ini = the standard (15-11)
    },
    "traffic": {
        "probe": 1,              # ON (owner 14-11): the AEB reads where the radar's lead is sideways (core/aeblat.h): no braking for a vehicle beside the path when passing
    },
    "lane": {
        "mode": 0,               # OFF: the game's own lane keeping (mode 2 is tuned on one truck/wheel)
        "tone": 1,                # ON: owner's ini = the standard (15-11)
        "min_kmh": 55,           # the game's own minimum speed (below 55 the plugin would patch it)
        "keep_on": 0,            # the game's own: off below its minimum speed
        "on_kmh": 55,
        "indicator_pause": 0,    # OFF with the game's law: the game only blocks steering against the indicator
        "needs_cruise": 1,        # ON: lane keeping only with the cruise, the lane key switches the cruise on; owner's ini = the standard (15-11)
        "lane_bias_m": 0,        # OFF until proven safe (24-10: the keep-right aim ran into a runaway)
        "lane_bend_out_m": 0,
        "control": 0,             # OFF: no calibration test on the key with the game's law; owner's ini = the standard (15-11)
    },
    "creep": {
        "smooth_start": 1,        # ON: owner's ini = the standard (15-11)
        "trace": 1,
    },
    "lowspeed": {
        "owner": 0,              # OFF: the low-speed owner does not drive anything yet
        "trace": 0,              # OFF: the shadow owner and its csv are for test builds
    },
    "decel": {
        "enabled": 1,             # ON: owner's ini = the standard (15-11)
        "retarder_fade": 0.75,    # owner's ini = the standard (15-11)
        "stop_assist": 2,         # brake and hold; owner's ini = the standard (15-11)
        "target_curve": 1.6,      # owner's ini = the standard (15-11)
        "target_min": 0.4,        # owner's ini = the standard (15-11)
    },
    "planner": {
        "shadow": 0,             # OFF: the planner shadow only logs, for the developer's test rides
    },
    "brake": {
        "pedal": 1,               # ON: owner's ini = the standard (15-11)
    },
    "cab": {
        "act": 1,                 # ON: auto parking brake and start-stop press the controls; owner's ini = the standard (15-11)
    },
}

# Sections that are not on the setup page (no tile, no knob, no Custom list): developer switches of something not wired in yet
# or not ridden yet.
NO_PAGE = ("carshift", "brake", "pedal", "lowspeed", "decel", "traffic", "cutin", "axle", "lights", "cab", "ebrake", "planner")
# Single keys of a shown section that stay off the page too (still in the ini, with their help text).
NO_PAGE_KEYS = (("core", "input_device"), ("core", "retarder_hook"), ("core", "brake_pedal_hook"), ("core", "brake_light_hook"), ("core", "retarder_feed_hook"), ("core", "brake_learn"), ("core", "dash_messages"), ("core", "air_from_truck"), ("cruise", "force_mode"))

STANDARD_HEADER = ("standard preset: the developer's own setup (own ACC with Stop & Go and Auto Hold, speed limit following, smooth start and the deceleration assist on; the look-ahead, bends, junctions and lane centering off; lane keeping only with the cruise on).")

SECTION_INTRO = {
    "core": "Logging. The level short keeps a short log and writes an incident file when something goes wrong; detailed and traces log more (traces adds csv files, large).",
    "trueshift": "The gearbox: TrueShift picks the gears of the game's automatic in a truck.",
    "carshift": "The gearbox of a car or pickup: CarShift picks the gears of the game's automatic there. Trucks use the [trueshift] sections above.",
    "cruise": "Cruise control and ACC extras. The standard preset has ACC, Stop & Go, Auto Hold and speed limit following (ISA) on; the ones that look ahead (bends, climbs) and JunctionSense are off.",
    "lane": "Lane centering. The standard preset lets the game steer, and only while the cruise is on: the lane key switches the cruise on, the cruise going off switches lane keeping off.",
    "creep": "Crawl and pull-away (slipping clutch in 1, 2 and R). With both pedals free the truck is held still (hold) or creeps (creep); a pressed pedal is the launch. The smooth start is on in the standard preset.",
    "pedal": "Torque pedal: the accelerator pedal asks for a share of what the truck can pull at this speed, and the throttle delivers it in whatever gear the truck is in. Off: the pedal asks a share of the engine's torque, as always.",
    "lowspeed": "Low-speed owner for stopping, standing and pulling away. Not wired in yet: both settings stay off.",
    "traffic": "Reads the game's AI traffic list. The emergency braking uses it to tell a vehicle in your path from one beside it (no braking for the truck you are passing). On.",
    "cutin": "Needs the traffic probe above. Only logs when a car pulls into your lane or the lead leaves it, and how much earlier that was than the game's radar. Nothing is braked, steered or announced from it.",
    "axle": "Raises a lift axle when the load allows it. Only logs what it would do unless press_controls_enabled is true; the game lowers the axle itself and never raises it.",
    "lights": "Own high-beam assist. Only logs what it would do unless press_controls_enabled is true. Needs the traffic probe above.",
    "cab": "Auto parking brake: on at engine off, off when you drive off; and start-stop: the engine off while Auto Hold holds, on again on gas / the lead moving off / low air. Only logs what it would do unless press_controls_enabled is true.",
    "ebrake": "Engine brake keys as a retarder lever: stage up switches the brake on at stage 1, up goes on to the next stage, down goes back, down at stage 1 switches it off. Works with whatever buttons are bound to the engine brake stage keys in the game. Also the descent control: under cruise, a truck with an engine brake holds its speed on a descent with the engine brake stages, one at a time (with a retarder too: the engine brake first, the retarder for the rest). On by default.",
    "planner": "The longitudinal planner in shadow: it plans beside the cruise and only writes what it would do to planner.csv. Nothing is driven from it. Off.",
    "brake": "Brake channel: one place for the cruise's automated braking (what coasting gives, a lower gear inside the bands, the engine brake's stages, the retarder, then the service brake) and the brake pedal as a deceleration wish. The channel is on in the standard preset, the retarder feed hook is on ([core] retarder_feed_hook), and the pedal is on (it needs [core] brake_pedal_hook, also on).",
    "decel": "Deceleration assist for the retarder and engine-brake lever: a lever step is a share of what the truck can brake now (step k of n is k/n of the retarder and the engine brake; the top step is all of it; step 1 never lets the truck get faster than at the pull and always brakes a little), and the service brake adds what the retarder and the engine brake lack (on in the standard preset). Also the stop assist: when you lift off at low speed and the gearbox opens the clutch the truck rolls out on its own resistance (on a descent the brake only keeps the speed from rising) and, in the last mode, Auto Hold holds it at the standstill ([deceleration_assist.stop_assist] mode, on by default).",
}


def cnum(tok):
    tok = tok.strip().rstrip("fF")
    return float(tok)


def fmt(v):
    """C's %g for a float value."""
    return "%g" % float(v)


def unescape(s):
    return s.replace('\\"', '"').replace("\\\\", "\\")


STR = r'"((?:[^"\\]|\\.)*)"'
ROW = re.compile(r'\{\s*"([a-z0-9_]+)"\s*,\s*&[\w.]+\s*,\s*(-?[0-9.]+[fF]?)\s*,\s*' + STR + r'\s*\}', re.S)   # &p_var or &V.field (core/vehicle.h)


def parse_table(path):
    src = open(path).read()
    m = re.search(r"static param g_params\[\]\s*=\s*\{(.*?)\n\};", src, re.S)
    if not m:
        raise SystemExit(f"{path}: no param table")
    rows = ROW.findall(m.group(1))
    # every row of the table must have been understood
    n_expected = len(re.findall(r'^\s*\{\s*"', m.group(1), re.M))
    if len(rows) != n_expected:
        raise SystemExit(f"{path}: parsed {len(rows)} of {n_expected} rows")
    name = re.search(r'module\s+m_\w+\s*=\s*\{\s*"([a-z]+)"', src).group(1)
    return name, [(k, cnum(d), unescape(h)) for k, d, h in rows]


def load():
    main = open(f"{ROOT}/core/main.cpp").read()
    order = re.search(r"g_modules\[\]\s*=\s*\{([^}]*)\}", main).group(1)
    modvars = re.findall(r"&m_(\w+)", order)
    files = {"core": f"{ROOT}/core/config.cpp", "brake": f"{ROOT}/core/brake.cpp"}
    tables = {}
    for var in modvars:
        path = files.get(var, f"{ROOT}/modules/{var}.cpp")
        name, rows = parse_table(path)
        tables[name] = rows
    check_names([m for m in modvars], tables)
    return [m for m in modvars], tables


# --- the public names (tools/ini_names.tsv)
_NAMES = None


def names():
    """(rows in file order, {(module, old key): row}, {(section, key): row})."""
    global _NAMES
    if _NAMES is None:
        rows = gen_ini_names.load_tsv()
        _NAMES = (rows, {(r["module"], r["old_key"]): r for r in rows}, {(r["section"], r["key"]): r for r in rows})
    return _NAMES


def check_names(order, tables):
    """Every param has a public name and every public name a param (a new param without a row fails the build); a bare section
    name that is also a module's legacy header must not reuse another setting's old key."""
    rows, by_old, by_new = names()
    code = {(m, k) for m in order for k, _, _ in tables[m]}
    missing = sorted(code - set(by_old))
    extra = sorted(set(by_old) - code)
    if missing or extra:
        raise SystemExit("tools/ini_names.tsv does not match the param tables.\n  params without a row: %s\n  rows without a param: %s\n"
                         "Add the row (module, old_key, section, key, audience, type) and run tools/gen_ini_names.py." % (missing, extra))
    for r in rows:
        mod = r["module"]
        other = by_old.get((r["section"], r["key"]))   # [planner] shadow_enabled when "planner" is a legacy header too
        if other is not None and other is not r:
            raise SystemExit(f"ini_names.tsv: [{r['section']}] {r['key']} is also the legacy name of {other['module']} {other['old_key']}")
        if r["type"] == "switch" and (mod, r["old_key"]) in by_old:
            d = next(d for k, d, _ in tables[mod] if k == r["old_key"])
            if d not in (0, 1) and r["old_key"] not in ("lever",):
                print(f"note: switch [{r['section']}] {r['key']} has the default {d}", file=sys.stderr)


def fmt_value(row, v):
    """A value as the generated ini writes it: true/false for a switch at 0 or 1, the word of a choice, else the number."""
    v = float(v)
    if row["type"] == "switch" and v in (0.0, 1.0):
        return "true" if v else "false"
    if row["type"] == "choice":
        for val, word in row["choices"]:
            if float(val) == v:
                return word
    return fmt(v)


def display(row, home=None):
    """How a help text names a setting: the key alone inside its own section, else [section] key."""
    if home is not None and row["section"] == home["section"] and row["key"] != "enabled":
        return row["key"]
    return f"[{row['section']}] {row['key']}"


def with_value(row, home, val):
    """A reference to a setting plus the value that the help text puts after it (0 / 1 / a choice number)."""
    d = display(row, home)
    if val is None:
        return d
    v = float(val)
    if row["type"] == "number":
        return f"{d} {val}"
    if row["type"] == "switch" and v in (0.0, 1.0):
        return f"{d}={'true' if v else 'false'}"
    for cv, word in row["choices"]:
        if float(cv) == v:
            return f"{d}={word}"
    return f"{d}={val}"


# Help texts that name a setting by its old key: ".. a key (optional digit)", "[module] key (optional digit)". Rewritten to the
# new names by the rules in rewrite_help, then by the few hand cases in HELP_FIX.
_REF = re.compile(r"(?:\[([a-z]+)\]\s*)?(?<![\w.\[])([a-z][a-z0-9]*(?:_[a-z0-9]+)*)(?![\w*]|\.(?:txt|csv|log|ini)\b)(?:(\s*=\s*|\s+)(-?\d+)(?!\d|\.\d))?")

HELP_FIX = {   # (module, key): [(old text, new text)] applied after the automatic rewrite
    ("core", "retarder_hook"): [("soften, ramp_s and pre_blend", "soften_enabled, ramp_seconds and pre_shift_blend_enabled")],
    ("core", "dash_messages"): [("its dash_letter 0", "its [trueshift.general] dashboard_letter_enabled=false")],
    ("core", "input_device"): [("when a feature of it is set to act", "when a feature of it is set to press them (press_controls_enabled)")],
    ("brake", "trace"): [("(log_level 2)", "([core.logging] level=traces)")],
    ("lane", "non_ffb_mode"): [("(mode=model_law is fitted", "(the model law is fitted"), ("2 = mode=model_law anyway", "2 = the model law anyway")],
    ("cruise", "overtake_assist"): [("probe_enabled and blind_spot", "probe_enabled and blind_spot_log_enabled")],
}


def rewrite_help(mod, key, text):
    rows, by_old, _ = names()
    home = by_old[(mod, key)]

    def one(m):
        bm, tok, sep, val = m.group(1), m.group(2), m.group(3), m.group(4)
        target = by_old.get((bm, tok)) if bm else None
        if bm and target is None:
            return m.group(0)
        if target is None:
            cand = by_old.get((mod, tok))
            if cand is None:
                return m.group(0)
            if "_" not in tok and val is None:
                return m.group(0)   # a plain word ("mode", "tone") only counts with a value after it: "mode 2", "learn 1"
            target = cand
        if target is home and not bm and val is None and tok == key:
            return display(target, home)
        text_ = with_value(target, home, val)
        return text_
    out = _REF.sub(one, text)
    for a, b in HELP_FIX.get((mod, key), []):
        if a not in out:
            raise SystemExit(f"HELP_FIX [{mod}] {key}: '{a}' is not in the help any more")
        out = out.replace(a, b)
    if mod in ("axle", "lights", "cab"):   # their "act" switch is press_controls_enabled
        out = re.sub(r"\bact\b", "press_controls_enabled", out)
    if home["type"] == "switch":   # a switch is true/false in the ini: "1 = on, 0 = off" reads "true = on, false = off"
        out = re.sub(r"(?<![\w.=/-])1 = ", "true = ", out)
        out = re.sub(r"(?<![\w.=/-])0 = ", "false = ", out)
    if home["type"] == "choice":
        out += ("" if out.rstrip().endswith(".") else ".") + " Values: " + ", ".join(f"{w} ({v})" for v, w in home["choices"]) + "."
    return out


INI_HOWTO_TEXT = ("One setting per line as name=value, with what it does in the lines above it. Settings are grouped in [module] and [module.section] blocks. Lines that start with # are ignored. Restart the game after a change. A setting you delete gets its standard value. The defaults were tuned on real rides: change one thing at a time. Switches are true or false (on, off, yes, no, 1 and 0 work too).")

def wrap(text, width=96):
    """A comment, wrapped at `width` columns, every line starting with '# '."""
    import textwrap
    return ["# " + l for l in textwrap.wrap(text, width - 2, break_long_words=False, break_on_hyphens=False)] or ["#"]


def layout(order, tables, internal=False):
    """The ini in order: [{module, section, rows: [[module, old key], ...]}]. A section holds its module's rows in the order of
    tools/ini_names.tsv. The internal (tuning) rows only with internal=True; a section with no row left is left out."""
    rows, by_old, _ = names()
    out = []
    for mod in order:
        secs = []
        for r in rows:
            if r["module"] != mod or (r["audience"] == "internal" and not internal):
                continue
            sec = next((x for x in secs if x["section"] == r["section"]), None)
            if sec is None:
                sec = {"module": mod, "section": r["section"], "rows": []}
                secs.append(sec)
            sec["rows"].append([mod, r["old_key"]])
        out += secs
    return out


def help_of(tables):
    """{(module, key): help text with the old key names rewritten to the new ones}."""
    return {(m, k): rewrite_help(m, k, h) for m, rows in tables.items() for k, _, h in rows}


def build(preset):
    order, tables = load()
    over = STANDARD if preset == "standard" else {}  # preset "dev": nothing overridden
    for sec, kv in over.items():
        keys = {k for k, _, _ in tables.get(sec, [])}
        for k in kv:
            if k not in keys:
                raise SystemExit(f"STANDARD overrides [{sec}] {k}, which does not exist in the source any more")
    out = []
    if preset == "dev":
        out.append(f"# dazdrive {VERSION} settings: the developer's own (every default as compiled in, traces on, the internal tuning settings too).")
        out.append("# Without this file the game writes the standard preset instead.")
    else:
        out += wrap(f"dazdrive {VERSION} settings: {STANDARD_HEADER}")
        out += wrap("dazdrive-setup.html (in this folder) makes a new one for you. Without this file the game writes this preset again.")
    out.append("#")
    out += wrap(INI_HOWTO_TEXT)
    _, by_old, _ = names()
    defaults = {(m, k): d for m, rows in tables.items() for k, d, _ in rows}
    helps = help_of(tables)
    seen = set()
    for sec in layout(order, tables, internal=(preset == "dev")):
        mod = sec["module"]
        out.append("")
        if preset == "standard" and mod in SECTION_INTRO and mod not in seen:
            out.append("# " + "=" * 94)
            out += wrap(SECTION_INTRO[mod])
            out.append("# " + "=" * 94)
        seen.add(mod)
        out.append(f"[{sec['section']}]")
        for m, k in sec["rows"]:
            r = by_old[(m, k)]
            v = over.get(m, {}).get(k, defaults[(m, k)])
            out.append("")
            out += wrap(helps[(m, k)])
            out.append(f"{r['key']}={fmt_value(r, v)}")
    return "\n".join(out) + "\n"


def c_literal(text):
    """Text as adjacent C string literals, one per line (core/config.cpp includes it as the standard ini)."""
    lines = []
    for l in text.split("\n")[:-1]:
        lines.append('"' + l.replace("\\", "\\\\").replace('"', '\\"') + '\\n"')
    return "\n".join(lines) + "\n"


# --- the browser setup page (tools/dazdrive-setup.tmpl.html + this data -> dazdrive-setup.html).
# Fine-tuning has three layers per module (Damien 17-10):
#   Presets  = the chips in "tuning" (a preset sets only keys that a knob also sets, so every preset is a knob position);
#   Advanced = "knobs": a few easy controls, one control may set several keys (see below);
#   Custom   = every raw ini key of the module with the plugin's own help line (no curation, no safety net).
# A module owns the keys of its "section"; with "prefix" only the keys that start with it (smooth start inside [creep]).
# The trace keys of every section belong to Logging. Keys a feature tile sets (step 2) are not tuning: they never decide the layer.
# Knob kinds:
#   ("range", label, help, unit, min, max, step, (sec, key), at, follow): a slider on (sec, key). "follow" = [((sec, key), [values], round)]:
#       keys that move along, piecewise linear through the points at[i] -> values[i] (beyond the ends with the slope of the
#       last stretch), rounded to a multiple of "round". With "presets" the points are the module's presets in order.
#   ("toggle", label, help, [(sec, key, on, off), ...])
#   ("choice", label, help, (sec, key), [[value, text], ...])
SETUP_GROUPS = [
    {"id": "ts", "off_when": ["trueshift", "enabled", "0"], "title": "Gearbox (TrueShift)", "section": "trueshift",
     "tuning": [{"name": "Standard", "desc": "Tuned in test drives", "over": {}}],
     "about": ["How TrueShift shifts."],
     "knobs": [
        {"kind": "choice", "label": "M mode", "help": "M is manual mode: you shift with + and −. This sets what the gearbox still does by itself there.",
         "key": ["trueshift", "m_style"], "options": [["0", "European: you shift everything, also at the rev limiter. A tone warns when the engine over-revs (Volvo, Mercedes)"],
                                                      ["1", "Protected: like European, but it shifts up at the rev limiter and down before a stall (Eaton)"],
                                                      ["2", "Assisted: like Protected, plus kickdown, a downshift when you brake to a stop, and one gear down when the engine lugs (like a VW DSG)"]]},
        {"kind": "range", "label": "Gear stays in after + or −", "help": "How long a gear you picked stays in, in seconds. 0 = off.",
         "unit": "s", "min": 0, "max": 30, "step": 1, "key": ["trueshift", "hint_hold_s"]},
        {"kind": "toggle", "label": "Show the mode on the dashboard", "help": "D, E, S or M instead of the game's A.",
         "keys": [["trueshift", "dash_letter", 1, 0]]},
        {"kind": "toggle", "label": "Tone when the mode changes", "help": "Eco low, Normal middle, Power rising, M two short.",
         "keys": [["trueshift", "tones", 1, 0]]}]},
    {"id": "cc", "title": "Cruise control and ACC", "section": "cruise",
     "tuning": [{"name": "Close", "desc": "Gaps 1.0 / 1.25 / 1.5 s, bends 3.125 m/s². Normal in game traffic.",
                 "over": {"cruise": {"acc_short_s": 1, "acc_middle_s": 1.25, "acc_long_s": 1.5, "curve_lat": 3.125}}},
                {"name": "Relaxed", "desc": "Gaps 1.5 / 2.0 / 2.5 s, bends 2.75 m/s². More room.",
                 "over": {"cruise": {"acc_short_s": 1.5, "acc_middle_s": 2, "acc_long_s": 2.5, "curve_lat": 2.75}}},
                {"name": "Real truck", "desc": "Gaps 2.4 / 3.0 / 3.6 s like Detroit and Bendix, bends 2.4 m/s². Stop & Go drives off by itself only after a stop under 2 s.",
                 "over": {"cruise": {"acc_short_s": 2.4, "acc_middle_s": 3, "acc_long_s": 3.6, "curve_lat": 2.4, "stop_go_auto_s": 2}}},
                {"name": "Dutch", "desc": "Gaps 0.9 / 1.15 / 1.4 s, bends 2.325 m/s². Right at the edge: 0.8 s is the lowest gap the standard allows.",
                 "over": {"cruise": {"acc_short_s": 0.9, "acc_middle_s": 1.15, "acc_long_s": 1.4, "curve_lat": 2.325}}},
                {"name": "US", "desc": "The gap follows the rule of thumb for US trucks: 1 second per 10 feet of truck below 40 mph, 1 second more above (7 and 8 seconds for a tractor and trailer). The game's radar sees a lead only up to about 100 m, so at highway speed the gap tops out near 3.5 seconds at 100 km/h.",
                 "over": {"cruise": {"acc_us_rule": 1, "acc_short_s": 1, "acc_middle_s": 1.25, "acc_long_s": 1.5, "curve_lat": 3.125}}}],
     "about": ["How the cruise, ACC and emergency braking behave."],
     "knobs": [
        {"kind": "range", "presets": True, "label": "ACC gap", "help": "Gap at the middle setting; short and long follow. Real trucks keep about 3 s; in the game 1.25 to 2 s feels normal.",
         "unit": "s", "min": 0.7, "max": 3.5, "step": 0.05, "key": ["cruise", "acc_middle_s"], "at": [1.25, 2, 3, 1.15, 1.25],
         "follow": [[["cruise", "acc_short_s"], [1, 1.5, 2.4, 0.9, 1], 0.05], [["cruise", "acc_long_s"], [1.5, 2.5, 3.6, 1.4, 1.5], 0.05]]},
        {"kind": "toggle", "label": "Gap by the US truck rule", "help": "Instead of the gap above: 1 second per 10 feet of vehicle length below 40 mph, 1 second more above. A tractor and trailer is taken as 70 feet, a truck alone as 25 feet. The three distance steps are 0.8, 1 and 1.2 times that. The game's radar sees only about 100 m ahead, so the gap cannot grow past that at highway speed.",
         "keys": [["cruise", "acc_us_rule", 1, 0]]},
        {"kind": "range", "presets": True, "label": "Speed through bends", "help": "Sideways force allowed in a bend (m/s²). Lower is slower and calmer.",
         "unit": "m/s²", "min": 2, "max": 3.5, "step": 0.025, "key": ["cruise", "curve_lat"], "at": [3.125, 2.75, 2.4, 2.325, 3.125], "follow": []},
        {"kind": "toggle", "label": "Stop & Go drives off by itself only after a short stop", "help": "Like real trucks. After a stop over 2 s, press Resume or the throttle.",
         "keys": [["cruise", "stop_go_auto_s", 2, 0]]},
        {"kind": "choice", "label": "Emergency braking warns", "help": "When the warning starts. With ACC on it warns at 2 s at the latest. Braking stays at 2 s (partial) and 1.2 s (full).",
         "key": ["cruise", "aeb_warn_s"], "options": [["3.5", "Early (3.5 s before a crash)"], ["3", "Normal (3 s)"], ["2.5", "Late (2.5 s)"]]},
        {"kind": "range", "label": "Set speed above the limit by", "help": "With Follow speed limits (ISA) on.",
         "unit": "km/h", "min": 0, "max": 10, "step": 1, "key": ["cruise", "speed_limit_offset_kmh"]},
        {"kind": "range", "label": "Never more than this over the limit", "help": "The cruise's own extra speed (descents, hills, coasting boost) stops here. 0 = no cap. Needs the limit to be known.",
         "unit": "km/h", "min": 0, "max": 10, "step": 1, "key": ["cruise", "overspeed_cap_kmh"]},
        {"kind": "choice", "label": "A tap on cruise + / −", "help": "Changes the set speed by this much.",
         "key": ["cruise", "step_tap_kmh"], "options": [["0", "The game's own step"], ["1", "1 km/h"], ["2", "2 km/h"], ["5", "5 km/h"], ["10", "10 km/h"]]},
        {"kind": "range", "label": "Speed limiter caps the set speed at", "help": "With the game's speed limiter on. 0 = the game's own value (truck 90), 100 for a bus.",
         "unit": "km/h", "min": 0, "max": 100, "step": 1, "key": ["cruise", "limiter_kmh"]},
        {"kind": "toggle", "label": "Emergency braking also without cruise", "help": "AEB also works when you drive yourself. Needs the game's radar: an ACC distance, blind spot or emergency brake switched on in the game's options.",
         "keys": [["cruise", "aeb_without_cruise", 1, 0]]},
        {"kind": "toggle", "label": "Distance key goes off, close, middle, far", "help": "The game: far, middle, close.",
         "keys": [["cruise", "acc_cycle_up", 1, 0]]},
        {"kind": "toggle", "label": "Coast in neutral with cruise on", "help": "On gentle descents, with SteadyCruise, Cruise looks ahead, Gearbox looks ahead and a route. The cruise gives a little gas first, and only for a roll of 15 s or more. Works without Eco-roll.",
         "keys": [["trueshift", "eco_roll_cruise", 1, 0]]},
        {"kind": "toggle", "label": "Pressing Set turns the retarder off", "help": "The game refuses cruise while the retarder lever is on.",
         "keys": [["cruise", "retarder_off_on_set", 1, 0]]}]},
    {"id": "lka", "off_when": ["lane", "mode", "0"], "title": "Lane centering", "section": "lane",
     "tuning": [{"name": "Standard", "desc": "Tones when it switches on, off or pauses", "over": {"lane": {"tone": 1}}},
                {"name": "Quiet", "desc": "No tones", "over": {"lane": {"tone": 0}}}],
     "about": ["How lane centering behaves."],
     "knobs": [
        {"kind": "toggle", "label": "Yields in roundabouts and fast bends", "help": "When you hold the wheel and the truck leans hard sideways (a roundabout, a bend taken too fast), lane centering fades out, and is back within a second when it eases. Hands off, nothing changes. Needs no route.", "keys": [["lane", "roundabout_off", 1, 0]]},
        {"kind": "toggle", "label": "Cruise switches it on", "help": "Switching the cruise on (also Resume) switches lane centering on. Off: the cruise leaves it as it is.", "keys": [["lane", "cruise_switches_on", 1, 0]]},
        {"kind": "toggle", "label": "Tones when it switches on, off or pauses", "help": "Rising for on, falling for off.", "keys": [["lane", "tone", 1, 0]]},
        {"kind": "toggle", "label": "Hold the lane key to calibrate", "help": "Hold the game's lane-keeping key for a moment: on the next straight stretch with lane centering steering, a few small steering pulses measure how your wheel responds, and the result is stored in your wheel's profile. A tap still switches lane centering on or off.", "keys": [["lane", "control", 1, 0]]}]},
    {"id": "crawl", "off_when": ["creep", "pedals_free", "0"], "title": "Creep at a standstill", "section": "creep",
     "tuning": [{"name": "Standard", "desc": "5 km/h", "over": {}},
                {"name": "Slow", "desc": "3 km/h, for tight docks", "over": {"creep": {"creep_kmh": 3}}}],
     "about": ["With both pedals free the truck either holds still or creeps; a pressed pedal is always the pull-away."],
     "knobs": [
        {"kind": "range", "presets": True, "label": "Creep speed", "help": "Speed with both pedals released. The brake slows it.",
         "unit": "km/h", "min": 2, "max": 7, "step": 0.5, "key": ["creep", "creep_kmh"], "at": [5, 3], "follow": []},
        {"kind": "toggle", "label": "Mode key switches hold / creep", "help": "Hold the Adaptive Automatic Transmission Mode key at a standstill to switch between hold and creep.", "keys": [["creep", "toggle_mode_key", 1, 0]]}]},
    {"id": "smoothstart", "off_when": ["creep", "smooth_start", "0"], "title": "Smooth start", "section": "creep", "prefix": "smooth_start",
     "tuning": [{"name": "Standard", "desc": "Engine speed follows the pedal", "over": {}}],
     "about": ["How the clutch closes when you pull away."],
     "knobs": [
        {"kind": "range", "label": "How firm the start is", "help": "Engine rpm with the pedal fully down while the clutch slips. Lower is gentler.",
         "unit": "rpm", "min": 900, "max": 1600, "step": 50, "key": ["creep", "smooth_start_rpm_full"], "at": [1000, 1200, 1500],
         "follow": [[["creep", "smooth_start_rpm_light"], [150, 200, 300], 10]]}]},
    {"id": "log", "title": "Logging", "section": "core",
     "tuning": [{"name": "Bug hunt", "desc": "Every shift in the log, plus csv files.", "over": {}},
                {"name": "Quiet", "desc": "A short log. Problems still get an incident file.", "over": {"core": {"log_level": 0}, "trueshift": {"trace": 0}, "carshift": {"trace": 0}, "cruise": {"trace": 0}, "creep": {"trace": 0}}}],
     "about": ["Problems are saved in the incidents folder: the 20 s before and 15 s after. Attach it to a bug report. Bug hunt also writes csv files (a few MB each). It is the default in the betas, not in the first full release."],
     "knobs": [
        {"kind": "choice", "label": "How much goes into dazdrive.log", "help": "Incidents are saved separately at every level.",
         "key": ["core", "log_level"], "options": [["0", "Quiet"], ["1", "Also every shift"], ["2", "Also csv traces (large)"]]},
        {"kind": "toggle", "label": "Write csv traces", "help": "trueshift.csv, carshift.csv, cruise.csv, creep.csv; needs the log at its highest level.",
         "keys": [["trueshift", "trace", 1, 0], ["carshift", "trace", 1, 0], ["cruise", "trace", 1, 0], ["creep", "trace", 1, 0]]},
        {"kind": "range", "label": "Incidents folder size", "help": "0 = no incident files.", "unit": "MB", "min": 0, "max": 40, "step": 1, "key": ["core", "incident_mb"]}]},
]

# --- wheel buttons: none. Everything is switched with the game's own keys (see the feature texts), so the page has no button step.
BUTTONS = []

# --- truck eras: how smart the truck is, as presets on the setup page. Each is "base" (standard) plus overrides,
# after when the real thing appeared in European series trucks: AMT 1995, ACC ~2000,
# eco-roll and emergency stop signal ~2010, predictive cruise and Stop & Go ~2012-2015, bends/roundabouts and lane
# centering 2019, ISA mandatory 2024. "recommended" = preselected on the page (no label): the newest era DazDrive has
# ridden through and through; it moves up after test rides. Realistic years, experimental parts included where they belong.
# "shipped" = the era that is the zip's STANDARD preset: it has no overrides (build_html checks that), and the page
# calls it "Standard (as shipped)" while nothing is changed.
# The _OFF lists name the look-ahead keys too although STANDARD has them off already, so the old years do not depend on it.
_OFF_1995 = {
    "trueshift": {"eco_roll": 0, "lookahead": 0, "smooth_stop": 0, "mode_character": 0, "eco_rev_ceiling_rpm": 0},
    "cruise": {"curves": 0, "curve_brake": 0, "descent": 0, "emergency_hazards": 0, "acc_calm_cut_in": 0,
               "hill_start": 0, "predictive": 0, "aeb": 0, "rollover_guard": 0, "eco_cruise_throttle_cap": 1},
}
_OFF_2000 = {
    "trueshift": {"eco_roll": 0, "lookahead": 0, "mode_character": 0, "eco_rev_ceiling_rpm": 0},
    "cruise": {"curves": 0, "curve_brake": 0, "emergency_hazards": 0, "predictive": 0, "aeb": 0, "rollover_guard": 0, "eco_cruise_throttle_cap": 1},
}
# The truck years keep what they meant before the owner's ini became the standard (15-11): the standard of before for the
# keys that changed, under each year's own overrides.
_YEARS_BASE = {
    "core": {"brake_light_hook": 0, "brake_pedal_hook": 0},
    "trueshift": {"takeover_rpm": 0, "takeover_kmh": 10, "handback_kmh": 6},
    "carshift": {"takeover_rpm": 0, "takeover_kmh": 10, "handback_kmh": 6},
    "cruise": {"descent_overspeed_kmh": 0, "auto_hold_park_s": 0, "min_kmh": 30, "min_set_kmh": 0, "own_acc": 0, "auto_hold": 0, "stop_go": 0, "speed_limit": 0, "speed_limit_ahead": 0, "resume_to_limit": 0, "acc_short_s": 1, "acc_middle_s": 1.25, "acc_long_s": 1.5, "curve_lat": 3.125, "aeb_mode": 0, "bend_now": 1, "engine": 0, "lim_show_set": 0, "rollover_guard": 3, "stop_go_auto_s": 0},
    "lane": {"tone": 0, "needs_cruise": 0, "control": 1},
    "creep": {"smooth_start": 0},
    "decel": {"enabled": 0, "retarder_fade": 1, "stop_assist": 1, "target_curve": 1, "target_min": 0.8},
    "brake": {"pedal": 0},
    "cab": {"act": 0},
}
def _years(over):
    return {s: dict(_YEARS_BASE.get(s, {}), **over.get(s, {})) for s in set(_YEARS_BASE) | set(over)}


_ON_2015 = {"trueshift": {"lookahead": 1},
            "cruise": {"predictive": 1, "curves": 0, "curve_brake": 0, "own_acc": 1, "auto_hold": 1, "stop_go": 1, "min_kmh": 5}}
_ON_2020 = {"trueshift": _ON_2015["trueshift"],
            "cruise": dict(_ON_2015["cruise"], curves=1, curve_brake=1, junctions=1), "creep": {"pedals_free": 1},
            "lane": {"mode": 2, "tone": 1, "min_kmh": 20, "keep_on": 1, "on_kmh": 39, "indicator_pause": 1}}
_ON_2024 = {"trueshift": _ON_2020["trueshift"], "cruise": dict(_ON_2020["cruise"], speed_limit=1, resume_to_limit=1),
            "creep": _ON_2020["creep"], "lane": _ON_2020["lane"]}
ERAS = [
    {"id": "1995", "name": "Classic", "year": "~1995", "base": "standard", "over": _years(_OFF_1995),
     "story": "First automated gearboxes (Scania 1995, ZF AS-Tronic about 1997) and a plain cruise control. Pick it if you only want the gearbox."},
    {"id": "2000", "name": "Radar cruise", "year": "~2000", "base": "standard", "over": _years(_OFF_2000),
     "story": "Radar cruise (Mercedes Actros, about 2000). Retarders hold speed downhill. Trucks get hill hold and smoother stops. The game's own ACC works in every year."},
    {"id": "2010", "name": "Modern", "year": "~2010", "base": "standard", "over": _years({}),
     "story": "Eco-roll (Volvo 2006, Mercedes 2008), emergency braking (Mercedes 2006, required in the EU from 2015) and hazard lights after hard braking."},
    {"id": "2015", "name": "Looks ahead", "year": "~2015", "base": "standard", "over": _years(_ON_2015),
     "story": "Gearbox and cruise read the road ahead (Scania, Mercedes, Volvo, 2012 to 2015), ACC keeps going in a jam, and Auto Hold. Here the road comes from your navigation route. Without a route, it acts like ~2010."},
    {"id": "2020", "name": "Assisted", "year": "~2020", "base": "standard", "over": _years(_ON_2020),
     "story": "Trucks slow for bends and roundabouts (Mercedes 2019) and steer in the lane (Mercedes Active Drive Assist, 2019). Crawl mode is older (Scania 2009) but sits here because it is experimental. Lane centering was only tested with one steering wheel."},
    {"id": "2024", "name": "Current", "year": "2024", "base": "standard", "over": _years(_ON_2024),
     "story": "Speed limit following (ISA), required on all new EU trucks since July 2024. Your cruise speed follows the signs, a little above them."},
    {"id": "dd", "name": "DazDrive", "year": "2026", "base": "standard", "over": {}, "recommended": True, "shipped": True, "standalone": True,
     "story": "What the developer drives every day, and the ini in the zip: own ACC with Stop & Go and Auto Hold, speed limit following, smooth start. The look-ahead, bends and lane centering stay off; lane keeping (the game's) only with the cruise on."},
]

# --- quick switches: one tile per feature, each sets one or more keys. [section, key, value when on, value when off]
# stage: stable = tested a lot, beta = light testing, experimental = not proven yet. An era
# on the page shows the weakest stage of the features it switches on. A tile's first key is the feature itself, the others
# refine it (the tile is "mixed" while only some of its keys have the "on" value).
FEATURES = [
    # group, stage, since (first in a real European truck), info (the popup), needs (feature ids that must be on)
    {"id": "ts", "group": "Gearbox", "stage": "beta", "since": "1995", "label": "TrueShift gearbox", "desc": "Shifts like a real truck automatic",
     "info": "Replaces the game's gear choice. Picks gears from engine torque, weight and slope. Keeps revs low and skips gears when it pays off. Eco puts diesel first, Power time, Normal is in between. Change modes (Eco, Normal, Power, M) with the game's Adaptive Automatic Transmission Mode key.",
     "keys": [["trueshift", "enabled", 1, 0]]},
    {"id": "roll", "group": "Gearbox", "stage": "stable", "since": "2006", "label": "Eco-roll", "desc": "Coasts in neutral on gentle descents to save fuel", "needs": ["ts"],
     "info": "On a gentle descent with the pedal released, the gearbox goes to neutral and the truck coasts. Never with the brake, engine brake or retarder. It shifts back in when you press the pedal or the speed drops too far.",
     "keys": [["trueshift", "eco_roll", 1, 0]]},
    {"id": "look", "group": "Gearbox", "stage": "stable", "since": "2012", "label": "Gearbox looks ahead", "desc": "Plans gears for climbs and hilltops on your route", "needs": ["ts"],
     "info": "Reads climbs, hilltops and junctions on your navigation route. Holds a gear before a climb instead of shifting halfway up, and rolls over hilltops. Without a route it shifts as normal. Real trucks call this predictive shifting.",
     "keys": [["trueshift", "lookahead", 1, 0]]},
    {"id": "stop", "group": "Gearbox", "stage": "stable", "since": "2000", "label": "Smooth stop", "desc": "No shaking when you stop", "needs": ["ts"],
     "info": "The clutch opens near idle as you brake to a stop, so the engine does not shake and the truck does not push against the brake. (The game keeps the clutch closed down to about 2 km/h.)",
     "keys": [["trueshift", "smooth_stop", 1, 0]]},
    {"id": "crawl", "group": "Gearbox", "stage": "experimental", "since": "2009", "label": "Creep at a standstill", "desc": "With both pedals free the truck creeps instead of being held",
     "info": "In gears 1, 2 and R the clutch slips and the truck creeps by itself, like a real automatic, instead of the light brake that holds it still. The brake slows it, the throttle pulls away. Hold the Adaptive Automatic Transmission Mode key at a standstill to switch between hold and creep.",
     "keys": [["creep", "pedals_free", 1, 0]]},
    {"id": "start", "group": "Gearbox", "stage": "experimental", "since": "2009", "label": "Smooth start", "desc": "Closes the clutch gently when you pull away",
     "info": "Replaces the game's quick, fixed clutch closing when you pull away. Engine speed follows the pedal, the clutch slips until the wheels catch up, then closes gently. A shift, lifting off, braking or about 14 km/h ends it. Lightly tested: turn it off if it feels wrong.",
     "keys": [["creep", "smooth_start", 1, 0]]},
    {"id": "cc", "group": "Cruise control", "stage": "stable", "since": "1990", "label": "SteadyCruise", "desc": "Calm cruise, no throttle bursts after shifts",
     "info": "Replaces the game's cruise throttle: no bursts after shifts, no speed swings around the set speed. Most cruise features build on it.",
     "keys": [["cruise", "smooth", 1, 0]]},
    {"id": "bend", "group": "Cruise control", "stage": "beta", "since": "2019", "label": "Slow for bends", "desc": "Eases off before sharp bends", "needs": ["cc"],
     "info": "Reads the route and eases off before a sharp bend. If easing off is not enough, it brakes gently (engine brake and retarder first). Needs a navigation route.",
     "keys": [["cruise", "curves", 1, 0], ["cruise", "curve_brake", 1, 0]]},
    {"id": "junc", "group": "Cruise control", "stage": "experimental", "since": "2019", "label": "JunctionSense", "desc": "Slows for junctions and roundabouts", "needs": ["bend"],
     "info": "Slows to the turn speed plus 10 km/h for junctions and roundabouts on your route. About 5 seconds before, it cancels the cruise with a tone, and you do the rest. Press Resume to get your speed back. It reads the road layout and can be wrong, for example braking for a bend that needed nothing.",
     "keys": [["cruise", "junctions", 1, 0]]},
    {"id": "pcc", "group": "Cruise control", "stage": "stable", "since": "2012", "label": "Cruise looks ahead", "desc": "Carries speed into climbs, eases off before crests", "needs": ["cc"],
     "info": "Instead of one fixed speed, the cruise carries speed into a climb, may run a little over the set speed in a dip, and eases off before a hilltop. Needs a navigation route. Real trucks call this predictive cruise (Scania, Mercedes, Volvo).",
     "keys": [["cruise", "predictive", 1, 0]]},
    {"id": "desc", "group": "Cruise control", "stage": "stable", "since": "1996", "label": "Downhill speed control", "desc": "Holds the speed downhill",
     "info": "Holds the set speed on descents with engine brake and retarder, like Mercedes Bremsomat. The truck does not run away and the brakes stay cool.",
     "keys": [["cruise", "descent", 1, 0]]},
    {"id": "haz", "group": "Cruise control", "stage": "stable", "since": "2010", "label": "Emergency stop signal", "desc": "Hazard lights flash after hard braking",
     "info": "Hazard lights come on after hard braking and stay on at a full stop (an EU rule for real trucks).",
     "keys": [["cruise", "emergency_hazards", 1, 0]]},
    {"id": "aeb", "group": "Cruise control", "stage": "stable", "since": "2006", "label": "Emergency braking (AEB)", "desc": "Warns, then brakes when a crash is near", "needs": ["cc"],
     "info": "Warning tones, then partial braking, then full braking when a crash is near. It replaces the game's own emergency braking, which drained the brake air. Works without cruise too. Needs SteadyCruise; without it the game's own braking stays.",
     "keys": [["cruise", "aeb", 1, 0]]},
    {"id": "rsc", "group": "Cruise control", "stage": "beta", "since": "2005", "label": "Rollover protection", "desc": "Tone, then throttle off, then a brake when the truck starts to sway",
     "info": "Watches for a growing sideways swing, also when you drive by hand. A tone first, then no throttle, then a soft or firm brake when the swing keeps growing. It does not judge a single bend by a fixed limit: that limit depends on the physics mod (bend_enabled in [cruise.rollover_guard], under Custom, turns it on anyway). Real trucks have had roll stability control since about 2005 (required in the EU from 2014).",
     "needs": ["cc"], "keys": [["cruise", "rollover_guard", 3, 0]]},
    {"id": "hill", "group": "Cruise control", "stage": "stable", "since": "2000", "label": "Hill start assist", "desc": "Holds the brake 3 s on a slope",
     "info": "After stopping on a slope the brake holds for 3 s, so you can move your foot to the throttle without rolling back. Not used while Auto Hold is on.",
     "needs": ["cc"], "keys": [["cruise", "hill_start", 1, 0]]},
    {"id": "acc", "group": "Cruise control", "stage": "beta", "since": "2000", "label": "Adaptive cruise (ACC)", "desc": "DazDrive's own: follows the vehicle ahead, brakes early", "needs": ["cc"],
     "info": "DazDrive's own adaptive cruise, built on the game's radar. Follows smoothly, brakes early and gently, and stays calm when someone cuts in. The gap is in seconds, set with the game's distance key. With Slow for bends on, it also slows for junctions.",
     "keys": [["cruise", "own_acc", 1, 0]]},
    {"id": "sg", "group": "Cruise control", "stage": "beta", "since": "2011", "label": "Stop & Go", "desc": "ACC keeps going in a jam", "needs": ["cc", "acc"],
     "info": "ACC stops behind the vehicle ahead and drives off with it. With the Real truck preset (Fine-tuning) it drives off by itself only after a stop under 2 s; after a longer stop, press Resume or the throttle. It also lowers the lowest cruise speed to 5 km/h (game: 30).",
     "keys": [["cruise", "stop_go", 1, 0], ["cruise", "min_kmh", 5, 30]]},
    {"id": "hold", "group": "Cruise control", "stage": "beta", "since": "2011", "label": "Auto Hold", "desc": "Keeps the brake on at a stop until you press the throttle", "needs": ["cc"],
     "info": "After a stop, the brake stays on until you press the throttle, like Auto Hold in a modern truck. No creeping at a light.",
     "keys": [["cruise", "auto_hold", 1, 0]]},
    {"id": "isa", "group": "Cruise control", "stage": "beta", "since": "2022", "label": "Follow speed limits (ISA)", "desc": "Cruise speed follows the signs (+3 km/h)",
     "info": "The set speed follows the signs plus an offset you choose (3 km/h by default). With this switched on it starts on; switched off it starts off but stays available: hold the distance key (Adaptive Cruise Control Mode) for about a second to turn it on or off. A short tone tells which.",
     "keys": [["cruise", "speed_limit", 1, 0], ["cruise", "resume_to_limit", 1, 0]]},
    {"id": "lka", "group": "Steering", "stage": "experimental", "since": "2019", "label": "Lane centering", "desc": "Keeps the truck in its lane while the cruise control is on",
     "info": "Steers gently to keep the truck in its lane while the cruise control is on: switching the cruise on switches it on, like a travel assist. It learns how your wheel responds and damps sway, and pauses while you use the turn signal. Keep your hands near the wheel: it assists, it does not drive for you.",
     "keys": [["lane", "mode", 2, 0], ["lane", "min_kmh", 20, 55], ["lane", "keep_on", 1, 0], ["lane", "on_kmh", 39, 55],
              ["lane", "indicator_pause", 1, 0]]},
]


def era_values(tables, era):
    """What the page's presetValues() computes for an era: compiled defaults, then STANDARD (base), then the overrides."""
    v = {s: {k: d for k, d, _ in rows} for s, rows in tables.items()}
    for src in ((STANDARD if era["base"] == "standard" else {}), era["over"]):
        for sec, kv in src.items():
            v[sec].update({k: float(x) for k, x in kv.items()})
    return v


def check_eras(tables):
    """The shipped era must be exactly the standard preset; no tile may be on in a year before its 'since', and a stable
    tile must be on from that year."""
    shipped = [e for e in ERAS if e.get("shipped")]
    if len(shipped) != 1 or shipped[0]["base"] != "standard" or any(shipped[0]["over"].values()):
        raise SystemExit("exactly one era must be 'shipped' (base standard, no overrides): it is the preset in the zip")
    for e in ERAS:
        v = era_values(tables, e)
        for f in FEATURES:
            s, k, on, _ = f["keys"][0]
            if not f.get("since"):
                continue
            is_on, early = v[s][k] == float(on), int(e["year"].lstrip("~")) < int(f["since"])
            if is_on and early:
                raise SystemExit(f"era {e['id']} switches on '{f['label']}', which real trucks had only from ~{f['since']}")
            if not is_on and not early and f.get("stage", "stable") == "stable" and not e.get("standalone"):  # stable ones are on from their year (beta and experimental come later)
                raise SystemExit(f"era {e['id']} has '{f['label']}' off, but it is stable and real trucks had it from ~{f['since']}")


def setup_scopes(order, tables):
    """Which module owns which ini key on the setup page: [[sec, key, kind b|n]] per group id. Every key exactly once."""
    scopes = {g["id"]: [] for g in SETUP_GROUPS}
    _, by_old, _ = names()
    for sec in order:
        if sec in NO_PAGE and sec != "carshift":   # carshift: only its csv trace switch, which the Logging module's "Write csv traces" sets
            continue
        for k, d, h in tables[sec]:
            if (sec, k) in NO_PAGE_KEYS or (sec == "carshift" and k != "trace"):
                continue
            if by_old[(sec, k)]["audience"] == "internal":   # not in the ini, so not on the page; the page keeps a value it reads from an ini
                continue
            gid = None
            if k == "trace":
                gid = "log"
            else:
                for g in SETUP_GROUPS:
                    if g["section"] == sec and g.get("prefix") and k.startswith(g["prefix"]):
                        gid = g["id"]
                if gid is None:
                    gid = next((g["id"] for g in SETUP_GROUPS if g["section"] == sec and not g.get("prefix")), None)
            if gid is None:
                raise SystemExit(f"setup page: no module owns [{sec}] {k}")
            # a switch: the map says so and the default is 0 or 1
            scopes[gid].append([sec, k, "b" if by_old[(sec, k)]["type"] == "switch" and d in (0, 1) else "n"])
    return scopes


def knob_keys(kb):
    if kb["kind"] == "toggle":
        return [(s, k) for s, k, _, _ in kb["keys"]]
    return [tuple(kb["key"])] + [tuple(f[0]) for f in kb.get("follow", [])]


def knob_at(kb, x, i):
    """Value of follow key i at slider position x: piecewise linear through the points, the end stretches extended."""
    pts = sorted(zip(kb["at"], kb["follow"][i][1]))
    j = 0
    while j < len(pts) - 2 and x > pts[j + 1][0]:
        j += 1
    (x0, y0), (x1, y1) = pts[j], pts[j + 1]
    r = kb["follow"][i][2]
    return round((y0 + (y1 - y0) * (x - x0) / (x1 - x0)) / r) * r


def check_knobs(tables, known):
    """Knob keys exist and are not set by a feature tile; a preset only sets knob keys (so it is a knob position); a range
    with "presets" goes through the presets' values; follow keys at a point give exactly that point's value."""
    feat = {(s, k) for f in FEATURES for s, k, _, _ in f["keys"]}
    shipped = era_values(tables, [e for e in ERAS if e.get("shipped")][0])
    for g in SETUP_GROUPS:
        kk = set()
        for kb in g["knobs"]:
            for s, k in knob_keys(kb):
                if k not in known[s]:
                    raise SystemExit(f"knob {g['id']}/{kb['label']}: [{s}] {k} does not exist in the source any more")
                if (s, k) in feat:
                    raise SystemExit(f"knob {g['id']}/{kb['label']}: [{s}] {k} is set by a feature tile")
                kk.add((s, k))
            if kb["kind"] == "range":
                pts = kb.get("at", [])
                if any(not kb["min"] <= x <= kb["max"] for x in pts) or any(len(f[1]) != len(pts) for f in kb.get("follow", [])):
                    raise SystemExit(f"knob {g['id']}/{kb['label']}: points outside min..max or of unequal length")
                for i, f in enumerate(kb.get("follow", [])):
                    for x, y in zip(pts, f[1]):
                        if abs(knob_at(kb, x, i) - y) > 1e-9:
                            raise SystemExit(f"knob {g['id']}/{kb['label']}: {f[0][1]} at {x} rounds to {knob_at(kb, x, i)}, not {y}")
                if kb.get("presets"):
                    if len(pts) != len(g["tuning"]):
                        raise SystemExit(f"knob {g['id']}/{kb['label']}: one point per preset")
                    for t, x, i in zip(g["tuning"], pts, range(len(pts))):
                        for (s, k), want in [(tuple(kb["key"]), x)] + [(tuple(f[0]), f[1][i]) for f in kb["follow"]]:
                            got = t["over"].get(s, {}).get(k, shipped[s][k])
                            if abs(float(got) - want) > 1e-9:
                                raise SystemExit(f"knob {g['id']}/{kb['label']}: preset {t['name']} has [{s}] {k} = {got}, the knob {want}")
        for t in g["tuning"]:
            for s, kv in t["over"].items():
                for k in kv:
                    if (s, k) not in kk:
                        raise SystemExit(f"preset {g['id']}/{t['name']}: [{s}] {k} has no knob, so Advanced could not show it")


def knob_data(kb):
    d = dict(kb)
    if kb["kind"] == "toggle":
        d["keys"] = [[s, k, fmt(on), fmt(off)] for s, k, on, off in kb["keys"]]
    return d


def build_html():
    order, tables = load()
    known = {s: {k for k, _, _ in tables[s]} for s in order}
    check_eras(tables)
    scopes = setup_scopes(order, tables)
    check_knobs(tables, known)
    for e in ERAS:
        for s, kv in e["over"].items():
            for k in kv:
                if k not in known[s]:
                    raise SystemExit(f"era {e['id']}: [{s}] {k} does not exist in the source any more")
    for g in SETUP_GROUPS:
        for t in g["tuning"]:
            for sec, kv in t["over"].items():
                for k in kv:
                    if k not in known[sec]:
                        raise SystemExit(f"tuning {g['id']}/{t['name']}: [{sec}] {k} does not exist in the source any more")
    for sec, k, _, _, _ in BUTTONS:
        if k not in known[sec]:
            raise SystemExit(f"button helper: [{sec}] {k} does not exist in the source any more")
    ids = {f["id"] for f in FEATURES}
    for f in FEATURES:
        for s, k, _, _ in f["keys"]:
            if k not in known[s]:
                raise SystemExit(f"feature {f['id']}: [{s}] {k} does not exist in the source any more")
        for n in f.get("needs", []):
            if n not in ids:
                raise SystemExit(f"feature {f['id']} needs unknown feature {n}")
    helps = help_of(tables)
    lay = layout(order, tables, internal=True)
    data = {
        "version": VERSION,
        "order": order,
        "tables": {s: [[k, fmt(d), helps[(s, k)]] for k, d, h in tables[s]] for s in order},
        # the public names: "module.old key" -> [section, key, audience u|a|i, type n|s|c, "value=word ..."]; the page works with the old
        # names inside and writes and reads the new ones
        "names": {f"{r['module']}.{r['old_key']}": [r["section"], r["key"], r["audience"][0], r["type"][0], " ".join(f"{v}={w}" for v, w in r["choices"])]
                  for r in names()[0]},
        # the ini in order (internal rows included: the page writes one only when a player changed it)
        "layout": [dict(x, intro=(SECTION_INTRO.get(x["module"]) if not any(y["module"] == x["module"] for y in lay[:i]) else None))
                   for i, x in enumerate(lay)],
        "standard": {s: {k: fmt(v) for k, v in kv.items()} for s, kv in STANDARD.items()},
        "intro": SECTION_INTRO,
        "standard_header": STANDARD_HEADER,
        "howto": INI_HOWTO_TEXT,
        "buttons": BUTTONS,
        "groups": [{"id": g["id"], "off_when": g.get("off_when"), "title": g["title"], "about": g["about"],
                    "tuning": [dict(t, over={sec: {k: fmt(v) for k, v in kv.items()} for sec, kv in t["over"].items()}) for t in g["tuning"]],
                    "knobs": [knob_data(kb) for kb in g["knobs"]], "scope": scopes[g["id"]]} for g in SETUP_GROUPS],
        "eras": [dict(e, over={s: {k: fmt(v) for k, v in kv.items()} for s, kv in e["over"].items()}) for e in ERAS],
        "features": [dict(f, keys=[[s, k, fmt(on), fmt(off)] for s, k, on, off in f["keys"]]) for f in FEATURES],
    }
    tmpl = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "dazdrive-setup.tmpl.html")).read()
    blob = json.dumps(data, ensure_ascii=False).replace("</", "<\\/")
    return tmpl.replace("/*DATA*/null", blob)


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        return
    if a[0] == "--list":
        order, tables = load()
        _, by_old, _ = names()
        for sec in order:
            for k, d, h in tables[sec]:
                r = by_old[(sec, k)]
                print(f"{sec}.{k}\t{fmt(d)}\t[{r['section']}] {r['key']} ({r['audience']})\t{h}")
        return
    if a[0] == "inc":
        # the standard preset as a C table for core/config.cpp: a missing ini or key gets these, not the developer's
        order, tables = load()
        lines = ["// generated by tools/gen_ini.py inc from STANDARD: do not edit", ]
        for sec in order:
            keys = {k for k, _, _ in tables[sec]}
            for k, v in STANDARD.get(sec, {}).items():
                if k not in keys:
                    raise SystemExit(f"STANDARD overrides [{sec}] {k}, which does not exist in the source any more")
                lines.append(f'{{ "{sec}", "{k}", {float(v)!r}f }},')
        text = "\n".join(lines) + "\n"
        if "-o" in a:
            open(a[a.index("-o") + 1], "w").write(text)
        else:
            sys.stdout.write(text)
        return
    if a[0] == "inc-ini":
        # the standard ini as C string literals for core/config.cpp: what the DLL writes when there is no ini
        text = "// generated by tools/gen_ini.py inc-ini: do not edit\n" + c_literal(build("standard"))
        if "-o" in a:
            open(a[a.index("-o") + 1], "w").write(text)
        else:
            sys.stdout.write(text)
        return
    if a[0] == "html":
        text = build_html()
        if "-o" in a:
            open(a[a.index("-o") + 1], "w").write(text)
        else:
            sys.stdout.write(text)
        return
    if a[0] == "damien":  # old name of "dev", still accepted
        a[0] = "dev"
    if a[0] not in ("dev", "standard"):
        raise SystemExit("preset must be dev or standard")
    text = build(a[0])
    if "-o" in a:
        open(a[a.index("-o") + 1], "w").write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
