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
# Rule: ON = stable (ridden a lot, found safe); OFF = beta or experimental, not ridden enough, or specific to one setup.
# The shipped preset IS the "~2010 Modern" year of the setup page: ERAS "2010" has no overrides (build_html checks it),
# so the page shows it as "Standard (as shipped)". The look-ahead items (lookahead, curves, curve_brake, predictive) are
# stable but come on only from ~2015 / ~2020 on the page, so they are off here.
STANDARD = {
    # Players: a quiet log plus incident files (what went wrong, with the 20 s before and 15 s after). At most ~6 MB in all:
    # the log 0.6 MB x (log, .1, previous session) and 4 MB of incidents. The developer's compiled defaults are 10x that.
    "core": {"log_level": 0, "log_max_mb": 0.6, "incident_mb": 4},
    "trueshift": {
        "takeover_kmh": 10,      # players: the proven handover (the developer's compiled default takes over earlier, from 7 km/h, still being tested)
        "handback_kmh": 6,
        "eco_roll": 1,           # ON: neutral roll on gentle descents, with its strict limits (eco_roll_max_decel, never with brake/retarder/engine brake); ridden
        "lookahead": 0,          # off in the shipped preset: the ~2010 year of the setup page; the page turns it on from ~2015
        "wet_wipers": 0,         # OFF: automatic wipers keep the lever on, so it was always "wet"; traction control does grip
        "no_creep": 1,           # ON: with crawl off the truck never creeps on the clutch at idle (the stock automatic did)
        "smooth_stop": 1,        # ON: clutch opens around idle when braking to a stop (no pushing down to 2 km/h); ridden
        "dash_letter": 1,        # ON: D/E/P/M on the dashboard instead of A
        "tones": 1,              # ON: short tone when the mode changes
        "trace": 0,              # csv traces only for bug hunting (needs log_level 2 anyway)
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
        "trace": 0,
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
        "start_on": 0,
        "trace": 0,
    },
}

STANDARD_HEADER = ("standard preset (stable on, except the look-ahead ones; beta and experimental "
                   "(ACC, Stop & Go, Auto Hold, ISA, JunctionSense, lane centering, crawl mode) off).")

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
        out.append(f"# dazdrive {VERSION} settings: {STANDARD_HEADER}")
        out.append("# dazdrive-setup.html (in this folder) makes a new one for you. Without this file the game writes this preset again.")
    out.append("# Changes take effect after restarting the game. Each setting has a line above it saying what it does.")
    for sec in order:
        out.append("")
        if preset == "standard" and sec in SECTION_INTRO:
            out.append(f"# --- {SECTION_INTRO[sec]}")
        out.append(f"[{sec}]")
        for k, d, h in tables[sec]:
            v = over.get(sec, {}).get(k, d)
            out.append(f"# {h}")
            out.append(f"{k}={fmt(v)}")
    return "\n".join(out) + "\n"


# --- the browser setup page (tools/dazdrive-setup.tmpl.html + this data -> dazdrive-setup.html). Items:
# (section, key, kind b|n|c, label, tag, options for c). tag: False = none, True or "experimental" = the red experimental
# tag, "beta" = the beta tag; it follows the stage of the tile that switches the item on. Every key not listed lands under "Advanced".
SETUP_GROUPS = [
    {"id": "ts", "off_when": ["trueshift", "enabled", "0"], "title": "Gearbox (TrueShift)", "advanced": "trueshift",
     "tuning": [{"name": "Standard", "desc": "As tuned on real rides", "over": {}}],
     "about": ["Shifts like a real truck automatic.", "⚙️ Low rpm, skips gears at full throttle", "🗺️ Reads your navigation route: climbs, tops, junctions", "🧠 Learns when an upshift did not hold"],
     "items": [
        ("trueshift", "enabled", "b", "TrueShift: the gearbox logic", False),
        ("trueshift", "eco_roll", "b", "Eco-roll: neutral on gentle descents (Eco 2, Normal 1.5, Power 1 km/h under the set speed)", False),
        ("trueshift", "smooth_stop", "b", "Smooth stop: open the clutch around idle when braking to a stop", False),
        ("trueshift", "wet_wipers", "b", "Wet road (wipers on): softer start, earlier upshifts", False),
        ("trueshift", "no_creep", "b", "Never creep on the clutch at idle when crawl mode is off", False),
        ("trueshift", "lookahead", "b", "Use the navigation route ahead (climbs, tops, junctions)", False),
        ("trueshift", "dash_letter", "b", "Show the gearbox mode (D, E, P, M) on the dashboard", False),
        ("trueshift", "tones", "b", "Short tone when the gearbox mode changes", False),
        ("trueshift", "hint_hold_s", "n", "Seconds a gear chosen with the shift-hint buttons is held", False)]},
    {"id": "cc", "title": "Cruise control and ACC", "advanced": "cruise",
     "tuning": [{"name": "Close", "desc": "ACC gaps 1.0 / 1.25 / 1.5 s: busy traffic, feels normal at the game's scale; bends 3.125 m/s2", "over": {}},
                {"name": "Relaxed", "desc": "ACC gaps 1.5 / 2.0 / 2.5 s: more room; bends 2.75 m/s2",
                 "over": {"cruise": {"acc_short_s": 1.5, "acc_middle_s": 2, "acc_long_s": 2.5, "curve_lat": 2.75}}},
                {"name": "Real truck", "desc": "ACC gaps 2.4 / 3.0 / 3.6 s like Detroit and Bendix; Stop&Go drives off by itself only after < 2 s; bends 2.4 m/s2",
                 "over": {"cruise": {"acc_short_s": 2.4, "acc_middle_s": 3, "acc_long_s": 3.6, "curve_lat": 2.4, "stop_go_auto_s": 2}}}],
     "about": ["SteadyCruise, ACC and AEB on top of the game's own cruise control.", "🚫 No throttle bursts after shifts", "🗺️ Bends and descents need a navigation route", "🧪 Experimental items only come on in the ~2020 and 2024 years"],
     "items": [
        ("cruise", "enabled", "b", "Cruise module", False),
        ("cruise", "smooth", "b", "SteadyCruise: our own calm cruise throttle (most cruise features need it)", False),
        ("cruise", "curves", "b", "Slow down gently before sharp bends", False),
        ("cruise", "curve_brake", "b", "...and brake gently when lifting is not enough", False),
        ("cruise", "descent", "b", "Hold the set speed downhill (engine brake / retarder)", False),
        ("cruise", "emergency_hazards", "b", "Hazard lights by themselves after an emergency stop", False),
        ("cruise", "acc_calm_cut_in", "b", "ACC stays calm when someone cuts in or pulls away", False),
        ("cruise", "acc_cycle_up", "b", "ACC distance key steps C, 1, 2, 3 (the game: far, middle, close)", False),
        ("cruise", "retarder_off_on_set", "b", "Switch the retarder off when you press cruise set/resume", False),
        ("cruise", "hill_start", "b", "Hill start: brake holds briefly after stopping on a slope", False),
        ("cruise", "aeb", "b", "AEB: automatic emergency braking (warning tones, partial, then full braking)", False),
        ("cruise", "aeb_without_cruise", "b", "AEB also watches when you drive without cruise control", False),
        ("cruise", "aeb_warn_s", "n", "AEB warns at this time to collision (s); partial braking at 2, full at 1.2", False),
        ("cruise", "curve_lat", "n", "Comfort in bends: sideways acceleration in m/s2 (lower = slower through bends)", False),
        ("cruise", "limiter_kmh", "n", "Cap the set speed at this value with the speed limiter on (0 = no cap)", False),
        ("cruise", "step_tap_kmh", "n", "A tap on + / - changes the set speed by this (0 = the game's step)", False),
        ("cruise", "set_ease_kmh_s", "n", "A lower set speed is eased down at this km/h per second (0 = at once)", False),
        ("cruise", "min_kmh", "n", "Lowest speed at which cruise can be switched on (the game: 30)", False),
        ("cruise", "junctions", "b", "JunctionSense: slow down before junction turns and roundabouts (always on with the own ACC)", True),
        ("cruise", "predictive", "b", "Carry a little speed into climbs, ease off before tops", False),
        ("cruise", "own_acc", "b", "Own, smoother ACC speed target from the lead vehicle", "beta"),
        ("cruise", "auto_hold", "b", "Auto Hold: stay braked after stopping until you give gas", "beta"),
        ("cruise", "stop_go", "b", "ACC keeps going in a jam (stop and go)", "beta"),
        ("cruise", "stop_go_auto_s", "n", "Stop&Go drives off by itself only after a stop shorter than this (s; 0 = always; real trucks: 2)", False),
        ("cruise", "acc_short_s", "n", "ACC gap at the short setting (s; real trucks 2.4-3.6, but the game is scaled)", False),
        ("cruise", "acc_middle_s", "n", "ACC gap at the middle setting (s)", False),
        ("cruise", "acc_long_s", "n", "ACC gap at the long setting (s)", False),
        ("cruise", "speed_limit", "b", "Follow speed limits with the set speed (ISA) from the start", "beta"),
        ("cruise", "speed_limit_offset_kmh", "n", "Set speed = limit plus this (km/h), with ISA", False)]},
    {"id": "lka", "off_when": ["lane", "mode", "0"], "title": "Lane centering", "advanced": "lane",
     "tuning": [{"name": "Standard", "desc": "The law and the self-learning as tuned on real rides", "over": {}}],
     "about": ["🧪 Experimental. Off = the game's own lane assist.", "🛣️ Calm, from 20 km/h, off while you indicate, learns how your wheel responds", "✋ Keep your hands near the wheel"],
     "items": [
        ("lane", "mode", "c", "Lane centering law", True, [["0", "0: the game's own"], ["2", "2: model-based (experimental)"]]),
        ("lane", "min_kmh", "n", "Lane centering steers from this speed (the game: 55)", False),
        ("lane", "keep_on", "b", "Stays on at any speed, can be switched on standing still", False),
        ("lane", "indicator_pause", "b", "Indicator pauses lane centering, back on after the lane change", False),
        ("lane", "tone", "b", "Tone when lane centering switches on/off", False)]},
    {"id": "crawl", "off_when": ["creep", "enabled", "0"], "title": "Crawl / maneuver mode", "advanced": "creep",
     "tuning": [{"name": "Standard", "desc": "Crawls at 5 km/h", "over": {}},
                {"name": "Slow", "desc": "Crawls at 3 km/h, for tight docks", "over": {"creep": {"creep_kmh": 3}}}],
     "about": ["🧪 Experimental. Walking pace in gears 1, 2 and R.", "🎮 Hold Adaptive Automatic Transmission Mode at a standstill: on/off", "🦶 Brake = slower, throttle = a bit more"],
     "items": [
        ("creep", "enabled", "b", "Crawl mode available", True),
        ("creep", "start_on", "b", "Crawl switched on when the game starts", False),
        ("creep", "toggle_mode_key", "b", "Hold Adaptive Automatic Transmission Mode at a standstill to switch crawl on or off", False),
        ("creep", "creep_kmh", "n", "Crawl speed (km/h)", False)]},
    {"id": "log", "title": "Logging", "advanced": "core",
     "tuning": [{"name": "Standard", "desc": "A quiet log, plus an incident file when something goes wrong", "over": {}},
                {"name": "Bug hunt", "desc": "Also every shift in the log and large csv traces, only when asked", "over": {"core": {"log_level": 2}, "trueshift": {"trace": 1}, "cruise": {"trace": 1}, "creep": {"trace": 1}}}],
     "about": ["🐞 When something goes wrong (the gearbox hunts, you fight a shift, the rpm hits the limiter, ...) the plugin saves the 20 s before and 15 s after in the incidents folder. Attach those files to a bug report.", "Bug hunt writes big csv files: only when asked."],
     "items": [("core", "log_level", "c", "How much goes into dazdrive.log", False, [["0", "0: quiet (incidents are saved separately)"], ["1", "1: + every shift"], ["2", "2: + csv traces (large)"]]),
               ("core", "incident_mb", "n", "Size of the incidents folder (MB, 0 = no incident files)", False)]},
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
     "blurb": "Smart gearbox, plain cruise control. The rest is up to you."},
    {"id": "2000", "name": "Radar cruise", "year": "~2000", "base": "standard", "over": _OFF_2000,
     "blurb": "+ smooth stop, calm following of the game's own ACC, downhill speed control, hill start assist"},
    {"id": "2010", "name": "Modern", "year": "~2010", "base": "standard", "over": {}, "recommended": True, "shipped": True,
     "blurb": "+ eco-roll, emergency braking, emergency stop signal"},
    {"id": "2015", "name": "Looks ahead", "year": "~2015", "base": "standard", "over": _ON_2015,
     "blurb": "+ gearbox and cruise look ahead, ACC with Stop & Go, Auto Hold"},
    {"id": "2020", "name": "Assisted", "year": "~2020", "base": "standard", "over": _ON_2020,
     "blurb": "+ slows for bends and roundabouts, lane centering, crawl mode"},
    {"id": "2024", "name": "Current", "year": "2024", "base": "standard", "over": _ON_2024,
     "blurb": "+ follows speed limits (ISA)"},
]

# --- quick switches: one tile per feature, each sets one or more keys. [section, key, value when on, value when off]
# stage: stable = ridden a lot, beta = built and ridden a little, experimental = not proven yet. An era
# on the page shows the weakest stage of the features it switches on. A tile's first key is the feature itself, the others
# refine it (the tile is "mixed" while only some of its keys have the "on" value).
FEATURES = [
    # group, stage, since (first in a real European truck, docs/eras.md), info (the popup), needs (feature ids that must be on)
    {"id": "ts", "group": "Gearbox", "stage": "stable", "since": "1995", "label": "TrueShift gearbox", "desc": "Shifts like a real truck automatic",
     "info": "Takes over the gear choice of the game's automatic. It picks the gear from engine torque, weight, grade and the route ahead instead of fixed rpm, keeps the revs low when it can, skips gears when the box is slow and learns when an upshift did not pull. The driving modes Eco, Normal, Power and M (manual, with help) are cycled with the game's own \"Adaptive Automatic Transmission Mode\" key.",
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
    {"id": "cc", "group": "Cruise control", "stage": "stable", "since": "1990", "label": "SteadyCruise", "desc": "Calm cruise control, no throttle bursts after shifts",
     "info": "Replaces the game's cruise throttle with a calm one: no bursts of full throttle after every shift and no hunting around the set speed. Most other cruise features build on it.",
     "keys": [["cruise", "smooth", 1, 0]]},
    {"id": "bend", "group": "Cruise control", "stage": "stable", "since": "2019", "label": "Slow for bends", "desc": "Eases off before sharp bends", "needs": ["cc"],
     "info": "Looks at the route ahead and eases off before a sharp bend. If lifting is not enough it brakes gently, engine brake and retarder first. Needs a navigation route.",
     "keys": [["cruise", "curves", 1, 0], ["cruise", "curve_brake", 1, 0]]},
    {"id": "junc", "group": "Cruise control", "stage": "experimental", "since": "2019", "label": "JunctionSense", "desc": "Slows for junctions and roundabouts", "needs": ["bend"],
     "info": "Slows for the turn at a junction and for roundabouts on your route, then picks the speed up again. The detector reads the road layout, which is not always right yet: it can brake for a bend that did not need it.",
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
     "info": "The ACC keeps working in a jam: it brakes to a stop behind the vehicle in front and drives off with it by itself. With the \"Real truck\" tuning (step 4) it does that only after a stop shorter than 2 s; after a longer stop you press resume or the throttle. Switching it on also sets the lowest cruise speed to 5 km/h (the game: 30), so the cruise can be switched on in a jam.",
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


def build_html():
    order, tables = load()
    known = {s: {k for k, _, _ in tables[s]} for s in order}
    check_eras(tables)
    for g in SETUP_GROUPS:
        for it in g["items"]:
            if it[1] not in known[it[0]]:
                raise SystemExit(f"setup page item [{it[0]}] {it[1]} does not exist in the source any more")
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
        "buttons": BUTTONS,
        "groups": [{"id": g["id"], "off_when": g.get("off_when"), "title": g["title"], "about": g["about"], "advanced": g["advanced"],
                    "tuning": [dict(t, over={sec: {k: fmt(v) for k, v in kv.items()} for sec, kv in t["over"].items()}) for t in g["tuning"]],
                    "items": [list(i) for i in g["items"]]} for g in SETUP_GROUPS],
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
