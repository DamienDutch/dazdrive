#!/usr/bin/env python3
"""Generate the DazDrive ini presets from the param tables in the C++ sources, so they never drift.

Usage:
  tools/gen_ini.py dev      [-o FILE]   the developer's compiled-in defaults, same layout as the DLL's own default ini
  tools/gen_ini.py standard [-o FILE]   preset for other players = the ~2010 year of the setup page: stable features on
                                        (not the ones that look ahead), beta and experimental off, no button numbers
  tools/gen_ini.py inc      [-o FILE]   the standard preset as a C table (the DLL's fallback when the ini or a key is missing)
  tools/gen_ini.py html     [-o FILE]   self-contained dazdrive-setup.html (browser setup page, same tables, no deps)
  tools/gen_ini.py --list               every section.key, default and help (for checking)

Parses  { "key", &p_var, default, "help" },  in `static param g_params[]` of core/config.cpp and modules/*.cpp, in the
order of g_modules[] in core/main.cpp. Run from anywhere. A key that STANDARD overrides but that no longer exists is an
error (the preset would silently stop doing what it says).
"""
import json, os, re, sys

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "plugin", "dazdrive"))
VERSION = re.search(r'#define SUITE_VERSION "([^"]*)"', open(f"{ROOT}/core/suite.h").read()).group(1)

# --- the "standard" preset: what a player gets who just drops the DLL in. Everything not listed keeps the compiled
# default (= the developer's), so a key that is ON in the standard preset is simply not listed unless it needs a comment.
# Rule: ON = stable (tested a lot, found safe); OFF = beta or experimental, not tested enough, or specific to one setup.
# The shipped preset IS the "~2010 Modern" year of the setup page: ERAS "2010" has no overrides (build_html checks it),
# so the page shows it as "Standard (as shipped)". The look-ahead items (lookahead, curves, curve_brake, predictive) are
# stable but come on only from ~2015 / ~2020 on the page, so they are off here.
STANDARD = {
    # Players: a quiet log plus incident files (what went wrong, with the 20 s before and 15 s after). At most ~6 MB in all:
    # the log 0.6 MB x (log, .1, previous session) and 4 MB of incidents. The developer's compiled defaults are 10x that.
    "core": {"log_level": 2, "log_max_mb": 0.6, "incident_mb": 4},   # BETA: bug hunt on by default until the first full release (make_release refuses a full release with it)
    "trueshift": {
        "takeover_rpm": 0,       # players: by speed only (the developer's default takes over at 1250 rpm in any gear)
        "takeover_kmh": 10,      # players: the proven handover (the developer's compiled default takes over earlier, from 7 km/h, still being tested)
        "handback_kmh": 6,
        "eco_roll": 1,           # ON: neutral roll on gentle descents, with its strict limits (eco_roll_max_decel, never with brake/retarder/engine brake); ridden
        "lookahead": 0,          # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2015
        "wet_wipers": 0,         # OFF: automatic wipers keep the lever on, so it was always "wet"; traction control does grip
        "no_creep": 1,           # ON: with crawl off the truck never creeps on the clutch at idle (the stock automatic did)
        "smooth_stop": 1,        # ON: clutch opens around idle when braking to a stop (no pushing down to 2 km/h); ridden
        "dash_letter": 1,        # ON: D/E/P/M on the dashboard instead of A
        "tones": 1,              # ON: short tone when the mode changes
        "trace": 1,              # csv traces (need log_level 2): ON in the betas for bug reports, off in the first full release
    },
    "cruise": {
        "smart_overspeed_kmh": -1,  # players: the game's own overspeed setting
        "descent_overspeed_kmh": 0, # players: descents held at the set speed, as before
        "smooth": 1,             # ON: own cruise throttle, no bursts after shifts: part of what TrueShift is for
        "curves": 0,             # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2020
        "curve_brake": 0,        # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2020
        "descent": 1,            # ON: hold the set speed downhill with engine brake/retarder; ridden
        "emergency_hazards": 1,  # ON: hazards after an emergency stop
        "air_min_psi": 75,       # ON: own brake requests stop below this brake air pressure (guard)
        "limiter_kmh": 90,       # ON: set speed never above the game's speed limiter (cap 90)
        "acc_calm_cut_in": 1,    # ON: someone cutting in or pulling away does not make the ACC speed up/slow down hard; ridden
        "acc_cycle_up": 1,       # ON: distance key order C -> 1 -> 2 -> 3 (the game: far, middle, close, off); ridden
        "set_ease_kmh_s": 2,     # ON: a lower set speed is reached by easing down at 2 km/h/s
        "step_tap_kmh": 5,       # ON: tap on + / - = 5 km/h step
        "step_hold_kmh": 1,      # ON: hold on + / - = 1 km/h steps
        "retarder_off_on_set": 1,# ON: retarder lever on is switched off when you press set/resume (the game refuses/cancels cruise)
        "hill_start": 1,         # ON: brake holds briefly after stopping on a slope (only effective while auto_hold is 0)
        "min_kmh": 30,           # like the game: lowest cruise speed 30 (the Stop & Go switch of the setup page sets 5, a jam use case)
        "min_set_kmh": 0,        # like the game
        "predictive": 0,         # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2015
        "junctions": 0,          # OFF: junctions/roundabouts: roundabout detection is a heuristic, not ridden enough
        "own_acc": 0,            # OFF: own ACC target, until a ride confirms the lead speed from the radar
        "auto_hold": 0,          # OFF: Auto Hold, until ridden
        "stop_go": 0,            # OFF: ACC at a standstill, untested
        "speed_limit": 0,        # OFF as start state: ISA (limit following) stays available but starts off
        "speed_limit_ahead": 0,  # OFF: only matters with ISA on
        "resume_to_limit": 0,    # OFF: only matters with ISA, changes what "resume" does
        "speed_limit_probe": 0,  # OFF: test-ride probe
        "trace": 1,
    },
    "lane": {
        "mode": 0,               # OFF: the game's own lane keeping (mode 2 is tuned on one truck/wheel)
        "tone": 0,
        "min_kmh": 55,           # the game's own minimum speed (below 55 the plugin would patch it)
        "keep_on": 0,            # the game's own: off below its minimum speed
        "on_kmh": 55,
        "indicator_pause": 0,    # OFF with the game's law: the game only blocks steering against the indicator
    },
    "creep": {
        "enabled": 0,            # OFF: crawl / maneuver mode is experimental
        "smooth_start": 0,       # OFF: the plugin's own clutch when pulling away is new (0.6.0-beta.2-dev), not ridden enough
        "start_on": 0,
        "trace": 1,
    },
}

STANDARD_HEADER = ("standard preset (stable on, except the look-ahead ones; beta and experimental "
                   "(ACC, Stop & Go, Auto Hold, ISA, JunctionSense, lane centering, crawl mode, smooth start) off).")

SECTION_INTRO = {
    "core": "Logging. Level 0 keeps a short log and writes an incident file when something goes wrong; level 1 and 2 log more (2 adds csv traces, large).",
    "trueshift": "The gearbox: TrueShift picks the gears of the game's automatic.",
    "cruise": "Cruise control and ACC extras. The standard preset has the stable ones on, except the ones that look ahead (bends, climbs); beta and experimental ones (ACC, Stop & Go, Auto Hold, ISA, JunctionSense) are off.",
    "lane": "Lane centering. The standard preset leaves the game's own assist alone.",
    "creep": "Crawl / maneuver mode (slipping clutch in 1, 2 and R). Off in the standard preset.",
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
ROW = re.compile(r'\{\s*"([a-z0-9_]+)"\s*,\s*&\w+\s*,\s*(-?[0-9.]+[fF]?)\s*,\s*' + STR + r'\s*\}', re.S)


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
    files = {"core": f"{ROOT}/core/config.cpp"}
    tables = {}
    for var in modvars:
        path = files.get(var, f"{ROOT}/modules/{var}.cpp")
        name, rows = parse_table(path)
        tables[name] = rows
    return [m for m in modvars], tables


INI_HOWTO_TEXT = ("One setting per line as name=value, with what it does in the lines above it. Lines that start with # are "
                  "ignored. Restart the game after a change. A setting you delete gets its standard value. The defaults were "
                  "tuned on real rides: change one thing at a time. Switches are 1 (on) or 0 (off).")

def wrap(text, width=96):
    """A comment, wrapped at `width` columns, every line starting with '# '."""
    import textwrap
    return ["# " + l for l in textwrap.wrap(text, width - 2, break_long_words=False, break_on_hyphens=False)] or ["#"]

def switch_keys():
    """The feature switches of the setup page (first key of each tile) per section: they come first in the ini."""
    out = {}
    for f in FEATURES:
        sec, key = f["keys"][0][0], f["keys"][0][1]
        if key not in out.setdefault(sec, []):
            out[sec].append(key)
    return out


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
        out.append(f"# dazdrive {VERSION} settings: the developer's own (every default as compiled in, traces on).")
        out.append("# Without this file the game writes the standard preset instead.")
    else:
        out += wrap(f"dazdrive {VERSION} settings: {STANDARD_HEADER}")
        out += wrap("dazdrive-setup.html (in this folder) makes a new one for you. Without this file the game writes this preset again.")
    out.append("#")
    out += wrap(INI_HOWTO_TEXT)
    sw = switch_keys()
    for sec in order:
        out.append("")
        if preset == "standard" and sec in SECTION_INTRO:
            out.append("# " + "=" * 94)
            out += wrap(SECTION_INTRO[sec])
            out.append("# " + "=" * 94)
        out.append(f"[{sec}]")
        rows = tables[sec]
        first = [r for r in rows if r[0] in sw.get(sec, [])]
        rest = [r for r in rows if r not in first]
        for title, group in (("Switches", first), ("Settings", rest)):
            if not group:
                continue
            if first:
                out += ["", f"# --- {title} ---"]
            for k, d, h in group:
                v = over.get(sec, {}).get(k, d)
                out.append("")
                out += wrap(h)
                out.append(f"{k}={fmt(v)}")
    return "\n".join(out) + "\n"


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
     "tuning": [{"name": "Standard", "desc": "As tuned on real rides", "over": {}}],
     "about": ["Shifts like a real truck automatic.", "Low rpm, skips gears at full throttle", "Reads your navigation route: climbs, tops, junctions", "Learns when an upshift did not hold"],
     "knobs": [
        {"kind": "choice", "label": "M (manual) mode", "help": "What the gearbox still does by itself in M, picked with the gearbox-mode key.",
         "key": ["trueshift", "m_style"], "options": [["0", "European: every shift is yours, a tone when the engine over-revs (like Volvo, Mercedes)"],
                                                      ["1", "Protected: also one gear up at the rev limiter and one down before a stall (like Eaton)"],
                                                      ["2", "Assisted: also kickdown and downshifts when you brake to a stop"]]},
        {"kind": "range", "label": "Gear held after + / −", "help": "How long a gear you picked with the shift-hint buttons stays in (0 = off).",
         "unit": "s", "min": 0, "max": 30, "step": 1, "key": ["trueshift", "hint_hold_s"]},
        {"kind": "toggle", "label": "Wipers on means a wet road", "help": "Softer start, earlier upshifts in low gears, softer kickdown.",
         "keys": [["trueshift", "wet_wipers", 1, 0]]},
        {"kind": "toggle", "label": "Show the mode on the dashboard", "help": "D, E, P or M instead of the game's A.",
         "keys": [["trueshift", "dash_letter", 1, 0]]},
        {"kind": "toggle", "label": "Tone when the mode changes", "help": "Eco low, Normal middle, Power rising, M two short.",
         "keys": [["trueshift", "tones", 1, 0]]}]},
    {"id": "cc", "title": "Cruise control and ACC", "section": "cruise",
     "tuning": [{"name": "Close", "desc": "ACC gaps 1.0 / 1.25 / 1.5 s: busy traffic, feels normal at the game's scale; bends 3.125 m/s²", "over": {}},
                {"name": "Relaxed", "desc": "ACC gaps 1.5 / 2.0 / 2.5 s: more room; bends 2.75 m/s²",
                 "over": {"cruise": {"acc_short_s": 1.5, "acc_middle_s": 2, "acc_long_s": 2.5, "curve_lat": 2.75}}},
                {"name": "Real truck", "desc": "ACC gaps 2.4 / 3.0 / 3.6 s like Detroit and Bendix; Stop & Go drives off by itself only after < 2 s; bends 2.4 m/s²",
                 "over": {"cruise": {"acc_short_s": 2.4, "acc_middle_s": 3, "acc_long_s": 3.6, "curve_lat": 2.4, "stop_go_auto_s": 2}}}],
     "about": ["SteadyCruise, ACC and AEB on top of the game's own cruise control.", "No throttle bursts after shifts", "Bends and descents need a navigation route"],
     "knobs": [
        {"kind": "range", "presets": True, "label": "Following distance", "help": "The ACC gap at the middle distance setting; short and long move along. Real trucks keep about 3 s, at the game's scale 1.25-2 s feels normal.",
         "unit": "s", "min": 1, "max": 3.5, "step": 0.05, "key": ["cruise", "acc_middle_s"], "at": [1.25, 2, 3],
         "follow": [[["cruise", "acc_short_s"], [1, 1.5, 2.4], 0.05], [["cruise", "acc_long_s"], [1.5, 2.5, 3.6], 0.05]]},
        {"kind": "range", "presets": True, "label": "Speed through bends", "help": "Sideways push allowed in a bend (m/s²): lower = slower and calmer, higher = brisker.",
         "unit": "m/s²", "min": 2, "max": 3.5, "step": 0.025, "key": ["cruise", "curve_lat"], "at": [3.125, 2.75, 2.4], "follow": []},
        {"kind": "toggle", "label": "Stop & Go drives off by itself only after a short stop", "help": "Like real trucks: after a stop longer than 2 s you press resume or the throttle.",
         "keys": [["cruise", "stop_go_auto_s", 2, 0]]},
        {"kind": "choice", "label": "Emergency braking warns", "help": "When the warning tones start; braking itself stays at 2 s (partial) and 1.2 s (full).",
         "key": ["cruise", "aeb_warn_s"], "options": [["3.5", "Early (3.5 s before a crash)"], ["3", "Normal (3 s)"], ["2.5", "Late (2.5 s)"]]},
        {"kind": "range", "label": "Speed limits: set speed = limit plus", "help": "With Follow speed limits (ISA) on.",
         "unit": "km/h", "min": 0, "max": 10, "step": 1, "key": ["cruise", "speed_limit_offset_kmh"]},
        {"kind": "choice", "label": "A tap on cruise + / −", "help": "Changes the set speed by this much.",
         "key": ["cruise", "step_tap_kmh"], "options": [["0", "The game's own step"], ["1", "1 km/h"], ["2", "2 km/h"], ["5", "5 km/h"], ["10", "10 km/h"]]},
        {"kind": "range", "label": "Speed limiter caps the set speed at", "help": "With the game's speed limiter on (0 = no cap).",
         "unit": "km/h", "min": 0, "max": 100, "step": 1, "key": ["cruise", "limiter_kmh"]},
        {"kind": "toggle", "label": "Emergency braking also without cruise", "help": "AEB watches when you drive yourself too (needs an ACC distance set).",
         "keys": [["cruise", "aeb_without_cruise", 1, 0]]},
        {"kind": "toggle", "label": "Distance key steps further away", "help": "Off, close, middle, far (the game: far, middle, close).",
         "keys": [["cruise", "acc_cycle_up", 1, 0]]},
        {"kind": "toggle", "label": "Cruise set switches the retarder off", "help": "The game refuses cruise while the retarder lever is on.",
         "keys": [["cruise", "retarder_off_on_set", 1, 0]]}]},
    {"id": "lka", "off_when": ["lane", "mode", "0"], "title": "Lane centering", "section": "lane",
     "tuning": [{"name": "Standard", "desc": "The law and the self-learning as tuned on real rides", "over": {}}],
     "about": ["Experimental. Off = the game's own lane assist.", "Calm, from 20 km/h, off while you indicate, learns how your wheel responds", "Keep your hands near the wheel"],
     "knobs": [
        {"kind": "toggle", "label": "Tone when it switches on or off", "help": "Rising for on, falling for off.", "keys": [["lane", "tone", 1, 0]]},
        {"kind": "toggle", "label": "Back on by itself after a lane change", "help": "When lane keeping went off while you indicated.", "keys": [["lane", "indicator_resume", 1, 0]]}]},
    {"id": "crawl", "off_when": ["creep", "enabled", "0"], "title": "Crawl / maneuver mode", "section": "creep",
     "tuning": [{"name": "Standard", "desc": "Crawls at 5 km/h", "over": {}},
                {"name": "Slow", "desc": "Crawls at 3 km/h, for tight docks", "over": {"creep": {"creep_kmh": 3}}}],
     "about": ["Experimental. Walking pace in gears 1, 2 and R.", "Hold Adaptive Automatic Transmission Mode at a standstill: on/off", "Brake = slower, throttle = a bit more"],
     "knobs": [
        {"kind": "range", "presets": True, "label": "Crawl speed", "help": "With both pedals released; the brake makes it slower.",
         "unit": "km/h", "min": 2, "max": 7, "step": 0.5, "key": ["creep", "creep_kmh"], "at": [5, 3], "follow": []},
        {"kind": "range", "label": "Maneuvering top speed", "help": "What a little throttle can ask for while maneuvering.",
         "unit": "km/h", "min": 6, "max": 15, "step": 1, "key": ["creep", "manoeuvre_kmh"]},
        {"kind": "toggle", "label": "Crawl on when the game starts", "help": "Otherwise hold the gearbox-mode key at a standstill.", "keys": [["creep", "start_on", 1, 0]]},
        {"kind": "toggle", "label": "Gearbox-mode key switches crawl", "help": "Hold it at a standstill to switch crawl on or off.", "keys": [["creep", "toggle_mode_key", 1, 0]]}]},
    {"id": "smoothstart", "off_when": ["creep", "smooth_start", "0"], "title": "Smooth start", "section": "creep", "prefix": "smooth_start",
     "tuning": [{"name": "Standard", "desc": "The engine rises with the pedal, the clutch slips, then closes gently", "over": {}}],
     "about": ["Experimental. Pulling away in a forward gear from a standstill.", "The pedal sets how high the engine revs while the clutch slips", "A shift, lifting off, the brake or about 14 km/h ends it"],
     "knobs": [
        {"kind": "range", "label": "Pulling away: gentle or firm", "help": "Engine rpm with the pedal flat while the clutch slips; the light-pedal rpm moves along. Lower = gentler, higher = firmer.",
         "unit": "rpm", "min": 900, "max": 1600, "step": 50, "key": ["creep", "smooth_start_rpm_full"], "at": [1000, 1200, 1500],
         "follow": [[["creep", "smooth_start_rpm_light"], [150, 200, 300], 10]]}]},
    {"id": "log", "title": "Logging", "section": "core",
     "tuning": [{"name": "Bug hunt", "desc": "Every shift in the log and csv traces: what the betas ship with, so a bug report has the data", "over": {}},
                {"name": "Quiet", "desc": "A short log, plus an incident file when something goes wrong", "over": {"core": {"log_level": 0}, "trueshift": {"trace": 0}, "cruise": {"trace": 0}, "creep": {"trace": 0}}}],
     "about": ["When something goes wrong (the gearbox hunts, you fight a shift, the rpm hits the limiter, ...) the plugin saves the 20 s before and 15 s after in the incidents folder. Attach those files to a bug report.", "Bug hunt writes csv files (a few MB each, the oldest part is dropped): it is on in the betas so a bug report has the data, and goes quiet in the first full release."],
     "knobs": [
        {"kind": "choice", "label": "How much goes into dazdrive.log", "help": "Incidents are saved separately at every level.",
         "key": ["core", "log_level"], "options": [["0", "Quiet"], ["1", "Also every shift"], ["2", "Also csv traces (large)"]]},
        {"kind": "toggle", "label": "Write csv traces", "help": "trueshift.csv, cruise.csv, creep.csv; needs the log at its highest level.",
         "keys": [["trueshift", "trace", 1, 0], ["cruise", "trace", 1, 0], ["creep", "trace", 1, 0]]},
        {"kind": "range", "label": "Incidents folder size", "help": "0 = no incident files.", "unit": "MB", "min": 0, "max": 40, "step": 1, "key": ["core", "incident_mb"]}]},
]

# --- wheel buttons: none. Everything is switched with the game's own keys (see the feature texts), so the page has no button step.
BUTTONS = []

# --- truck eras: how smart the truck is, as presets on the setup page. Each is "base" (standard) plus overrides,
# after when the real thing appeared in European series trucks (docs/eras.md): AMT 1995, ACC ~2000,
# eco-roll and emergency stop signal ~2010, predictive cruise and Stop & Go ~2012-2015, bends/roundabouts and lane
# centering 2019, ISA mandatory 2024. "recommended" = preselected on the page (no label): the newest era DazDrive has
# ridden through and through; it moves up after test rides. Realistic years, experimental parts included where they belong.
# "shipped" = the era that is the zip's STANDARD preset: it has no overrides (build_html checks that), and the page
# calls it "Standard (as shipped)" while nothing is changed.
# The _OFF lists name the look-ahead keys too although STANDARD has them off already, so the old years do not depend on it.
_OFF_1995 = {
    "trueshift": {"eco_roll": 0, "lookahead": 0, "smooth_stop": 0, "wet_wipers": 0},
    "cruise": {"curves": 0, "curve_brake": 0, "descent": 0, "emergency_hazards": 0, "acc_calm_cut_in": 0,
               "hill_start": 0, "predictive": 0, "aeb": 0},
}
_OFF_2000 = {
    "trueshift": {"eco_roll": 0, "lookahead": 0, "wet_wipers": 0},
    "cruise": {"curves": 0, "curve_brake": 0, "emergency_hazards": 0, "predictive": 0, "aeb": 0},
}
# ~2010 is the shipped preset, so it overrides nothing
_ON_2015 = {"trueshift": {"lookahead": 1},
            "cruise": {"predictive": 1, "curves": 0, "curve_brake": 0, "own_acc": 1, "auto_hold": 1, "stop_go": 1, "min_kmh": 5}}
_ON_2020 = {"trueshift": _ON_2015["trueshift"],
            "cruise": dict(_ON_2015["cruise"], curves=1, curve_brake=1, junctions=1), "creep": {"enabled": 1},
            "lane": {"mode": 2, "min_kmh": 20, "keep_on": 1, "on_kmh": 39, "indicator_pause": 1}}
_ON_2024 = {"trueshift": _ON_2020["trueshift"], "cruise": dict(_ON_2020["cruise"], speed_limit=1, resume_to_limit=1),
            "creep": _ON_2020["creep"], "lane": _ON_2020["lane"]}
ERAS = [
    {"id": "1995", "name": "Classic", "year": "~1995", "base": "standard", "over": _OFF_1995,
     "story": "The first automated manual gearboxes on series trucks (Scania Opticruise 1995, ZF AS-Tronic around 1997) and a plain cruise control: no radar, no help beyond the gearbox. Pick it if you want the gearbox only and drive the rest yourself.",
     "blurb": "Smart gearbox, plain cruise control. The rest is up to you."},
    {"id": "2000", "name": "Radar cruise", "year": "~2000", "base": "standard", "over": _OFF_2000,
     "story": "Radar cruise control arrives (Mercedes Actros ART, around 2000), the retarder can hold a set speed downhill, and trucks get a hill holder and smoother stops. The game's own ACC works in every year; this year adds calm following, downhill speed control, hill start assist and smooth stop.",
     "blurb": "+ smooth stop, calm following of the game's own ACC, downhill speed control, hill start assist"},
    {"id": "2010", "name": "Modern", "year": "~2010", "base": "standard", "over": {}, "recommended": True, "shipped": True,
     "story": "Eco-roll coasting (Volvo I-Roll 2006, Mercedes EcoRoll 2008), emergency braking (Mercedes 2006, mandatory on new trucks in the EU from 2015) and hazard lights on hard braking. This is the year the ini in the zip is set to, because everything in it has been driven a lot.",
     "blurb": "+ eco-roll, emergency braking, emergency stop signal"},
    {"id": "2015", "name": "Looks ahead", "year": "~2015", "base": "standard", "over": _ON_2015,
     "story": "Predictive gearbox and cruise that read the road ahead (Scania, Mercedes and Volvo, 2012 to 2015), radar cruise that keeps going in a jam, and auto hold at a standstill. Here the road ahead is your navigation route; without an active route it shifts and cruises like ~2010. Parts of this year are still beta.",
     "blurb": "+ gearbox and cruise look ahead, ACC with Stop & Go, Auto Hold"},
    {"id": "2020", "name": "Assisted", "year": "~2020", "base": "standard", "over": _ON_2020,
     "story": "Trucks slow for bends and roundabouts by themselves (Mercedes 2019) and steer along in the lane with the radar cruise (Mercedes Active Drive Assist, 2019). Crawl mode follows Scania's maneuvering mode (2009). These parts are experimental here; lane centering in particular has only been tried with one wheel.",
     "blurb": "+ slows for bends and roundabouts, lane centering, crawl mode"},
    {"id": "2024", "name": "Current", "year": "2024", "base": "standard", "over": _ON_2024,
     "story": "Adds speed limit following (ISA), mandatory on all new trucks in the EU since July 2024: the cruise set speed follows the signs, a little above them. Everything from ~2020 stays, so the experimental parts do too.",
     "blurb": "+ follows speed limits (ISA)"},
]

# --- quick switches: one tile per feature, each sets one or more keys. [section, key, value when on, value when off]
# stage: stable = tested a lot, beta = light testing, experimental = not proven yet. An era
# on the page shows the weakest stage of the features it switches on. A tile's first key is the feature itself, the others
# refine it (the tile is "mixed" while only some of its keys have the "on" value).
FEATURES = [
    # group, stage, since (first in a real European truck, docs/eras.md), info (the popup), needs (feature ids that must be on)
    {"id": "ts", "group": "Gearbox", "stage": "stable", "since": "1995", "label": "TrueShift gearbox", "desc": "Shifts like a real truck automatic",
     "info": "Takes over the gear choice of the game's automatic. It picks the gear from engine torque, weight, grade and the route ahead instead of fixed rpm, keeps the revs low when it can, skips gears when the box is slow and learns when an upshift did not pull. The driving modes Eco, Normal, Power and M (manual) are cycled with the game's own \"Adaptive Automatic Transmission Mode\" key.",
     "keys": [["trueshift", "enabled", 1, 0]]},
    {"id": "roll", "group": "Gearbox", "stage": "stable", "since": "2006", "label": "Eco-roll", "desc": "Rolls in neutral on gentle descents to save fuel",
     "info": "On a gentle descent with the cruise on, the gearbox goes to neutral and the truck rolls without burning fuel. Never with the brake, engine brake or retarder, and it re-engages as soon as the speed drifts too far from the set speed.",
     "keys": [["trueshift", "eco_roll", 1, 0]]},
    {"id": "look", "group": "Gearbox", "stage": "stable", "since": "2012", "label": "Gearbox looks ahead", "desc": "Picks gears for the climbs and tops on your route (predictive shifting)",
     "info": "Reads your navigation route ahead: climbs, crests and junctions. It keeps a gear before a climb instead of shifting halfway up, and lets the truck roll over a crest. Without an active route it shifts like normal.",
     "keys": [["trueshift", "lookahead", 1, 0]]},
    {"id": "stop", "group": "Gearbox", "stage": "stable", "since": "2000", "label": "Smooth stop", "desc": "No judder or pushing when you brake to a halt",
     "info": "When you brake to a stop the clutch opens around idle, so the engine does not judder and the truck does not push on against the brake. The game's own automatic keeps the clutch closed down to about 2 km/h.",
     "keys": [["trueshift", "smooth_stop", 1, 0]]},
    {"id": "crawl", "group": "Gearbox", "stage": "experimental", "since": "2009", "label": "Crawl mode", "desc": "Creeps at walking pace for parking and docking",
     "info": "At walking pace in gears 1, 2 and R the clutch slips like in a real truck automatic: the truck creeps by itself. The brake makes it slower, the throttle a bit faster. Switch it on or off by holding the Adaptive Automatic Transmission Mode key at a standstill.",
     "keys": [["creep", "enabled", 1, 0]]},
    {"id": "start", "group": "Gearbox", "stage": "experimental", "since": "2009", "label": "Smooth start", "desc": "Closes the clutch itself when you pull away, like a real truck automatic",
     "info": "Pulling away in a forward gear from a standstill, DazDrive closes the clutch itself instead of the game's quick, fixed way: the engine rises to a speed that follows the pedal (a little for a light pedal, high for a flat one, a bit more with a heavy load), the clutch slips to hold it until the wheels catch up, then closes gently. It works with or without crawl mode. A shift, lifting off the pedal, the brake or about 14 km/h ends it and the game takes over. New and lightly tested: if it feels wrong, switch it off.",
     "keys": [["creep", "smooth_start", 1, 0]]},
    {"id": "cc", "group": "Cruise control", "stage": "stable", "since": "1990", "label": "SteadyCruise", "desc": "Calm cruise control, no throttle bursts after shifts",
     "info": "Replaces the game's cruise throttle with a calm one: no bursts of full throttle after every shift and no hunting around the set speed. Most other cruise features build on it.",
     "keys": [["cruise", "smooth", 1, 0]]},
    {"id": "bend", "group": "Cruise control", "stage": "stable", "since": "2019", "label": "Slow for bends", "desc": "Eases off before sharp bends", "needs": ["cc"],
     "info": "Looks at the route ahead and eases off before a sharp bend. If lifting is not enough it brakes gently, engine brake and retarder first. Needs a navigation route.",
     "keys": [["cruise", "curves", 1, 0], ["cruise", "curve_brake", 1, 0]]},
    {"id": "junc", "group": "Cruise control", "stage": "experimental", "since": "2019", "label": "JunctionSense", "desc": "Slows for junctions and roundabouts", "needs": ["bend"],
     "info": "Slows for the turn at a junction and for roundabouts on your route, but only down to its speed plus 10 km/h. About 5 seconds before the junction it cancels the cruise with a sound, so the last part is yours; press Resume afterwards to get your set speed back. The detector reads the road layout, which is not always right yet: it can brake for a bend that did not need it.",
     "keys": [["cruise", "junctions", 1, 0]]},
    {"id": "pcc", "group": "Cruise control", "stage": "stable", "since": "2012", "label": "Cruise looks ahead", "desc": "Carries speed into climbs, eases off before tops (predictive cruise)", "needs": ["cc"],
     "info": "Instead of holding one speed, the cruise lets the speed dip a little before a climb and carries speed into it, and eases off before a crest. Needs a navigation route. Real trucks: Scania Active Prediction, Mercedes PPC, Volvo I-See.",
     "keys": [["cruise", "predictive", 1, 0]]},
    {"id": "desc", "group": "Cruise control", "stage": "stable", "since": "1996", "label": "Downhill speed control", "desc": "Holds the speed downhill",
     "info": "Holds the set speed on a descent with the engine brake and retarder, like Bremsomat in a Mercedes: no running away, no brake fade.",
     "keys": [["cruise", "descent", 1, 0]]},
    {"id": "haz", "group": "Cruise control", "stage": "stable", "since": "2010", "label": "Emergency stop signal", "desc": "Hazards flash after a hard stop",
     "info": "After hard braking the hazard lights come on by themselves and stay on at a full stop, as the EU requires of real trucks.",
     "keys": [["cruise", "emergency_hazards", 1, 0]]},
    {"id": "aeb", "group": "Cruise control", "stage": "stable", "since": "2006", "label": "Emergency braking (AEB)", "desc": "Warns, then brakes by itself when a crash is near", "needs": ["cc"],
     "info": "Automatic emergency braking: warning tones when a collision is near, then partial braking, then full braking. It replaces the game's own emergency braking, which dumped the brake air. It also watches when you drive without cruise control. It runs on SteadyCruise, our own cruise throttle: without that the game's own emergency braking stays active.",
     "keys": [["cruise", "aeb", 1, 0]]},
    {"id": "hill", "group": "Cruise control", "stage": "stable", "since": "2000", "label": "Hill start assist", "desc": "Brake holds 3 s on a slope, no rolling back",
     "info": "Hill start assist: after stopping on a slope the brake holds for a moment, so you can move your foot to the throttle without rolling back.",
     "keys": [["cruise", "hill_start", 1, 0]]},
    {"id": "acc", "group": "Cruise control", "stage": "beta", "since": "2000", "label": "Adaptive cruise (ACC)", "desc": "Our own: follows the vehicle ahead smoothly, brakes early", "needs": ["cc"],
     "info": "Our own adaptive cruise control on top of the game's radar: follows the vehicle in front smoothly, brakes early and gently, and does not react hard when someone cuts in. The gap is in seconds (short, middle, long) with the game's distance key. With Slow for bends on, it also slows for junctions.",
     "keys": [["cruise", "own_acc", 1, 0]]},
    {"id": "sg", "group": "Cruise control", "stage": "beta", "since": "2011", "label": "Stop & Go", "desc": "ACC keeps going in a jam", "needs": ["cc", "acc"],
     "info": "The ACC keeps working in a jam: it brakes to a stop behind the vehicle in front and drives off with it by itself. With the \"Real truck\" preset (step 3, Fine-tuning) it does that only after a stop shorter than 2 s; after a longer stop you press resume or the throttle. Switching it on also sets the lowest cruise speed to 5 km/h (the game: 30), so the cruise can be switched on in a jam.",
     "keys": [["cruise", "stop_go", 1, 0], ["cruise", "min_kmh", 5, 30]]},
    {"id": "hold", "group": "Cruise control", "stage": "beta", "since": "2011", "label": "Auto Hold", "desc": "Stays braked at a stop until you give gas", "needs": ["cc"],
     "info": "After you stop, the brake stays on by itself until you press the throttle, like Auto Hold in a modern truck: no creeping at a traffic light.",
     "keys": [["cruise", "auto_hold", 1, 0]]},
    {"id": "isa", "group": "Cruise control", "stage": "beta", "since": "2022", "label": "Follow speed limits (ISA)", "desc": "The cruise set speed follows the signs (+3 km/h)",
     "info": "Intelligent speed assistance (ISA): the cruise set speed follows the signs, plus the offset you choose (3 km/h by default). With this on, it starts active. While driving, hold the Adaptive Cruise Control Mode key (the distance key) for a second to switch it off and on; the distance stays as it was, and a short tone says which way it went.",
     "keys": [["cruise", "speed_limit", 1, 0], ["cruise", "resume_to_limit", 1, 0]]},
    {"id": "lka", "group": "Steering", "stage": "experimental", "since": "2019", "label": "Lane centering", "desc": "Keeps the truck in its lane, calm and self-learning, from 20 km/h",
     "info": "Keeps the truck in the middle of the lane with gentle steering, learns how your wheel responds and damps any sway. Works from 20 km/h and pauses while you indicate. Keep your hands near the wheel: it is an assistant, not an autopilot.",
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
            if not is_on and not early and f.get("stage", "stable") == "stable":  # stable ones are on from their year (beta and experimental come later)
                raise SystemExit(f"era {e['id']} has '{f['label']}' off, but it is stable and real trucks had it from ~{f['since']}")


def setup_scopes(order, tables):
    """Which module owns which ini key on the setup page: [[sec, key, kind b|n]] per group id. Every key exactly once."""
    scopes = {g["id"]: [] for g in SETUP_GROUPS}
    for sec in order:
        for k, d, h in tables[sec]:
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
            # a switch when the help starts with "1 = " (the plugin's convention for on/off keys)
            scopes[gid].append([sec, k, "b" if re.match(r"1 = ", h) and d in (0, 1) else "n"])
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
    data = {
        "version": VERSION,
        "order": order,
        "tables": {s: [[k, fmt(d), h] for k, d, h in tables[s]] for s in order},
        "standard": {s: {k: fmt(v) for k, v in kv.items()} for s, kv in STANDARD.items()},
        "intro": SECTION_INTRO,
        "standard_header": STANDARD_HEADER,
        "switches": switch_keys(),
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
        for sec in order:
            for k, d, h in tables[sec]:
                print(f"{sec}.{k}\t{fmt(d)}\t{h}")
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
