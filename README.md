# DazDrive

A plugin for Euro Truck Simulator 2 that makes the game's automatic gearbox, cruise control and lane
assist behave like the real thing. Lekker rijden.

**[Download the latest release](https://github.com/DamienDutch/dazdrive/releases)** ·
[Problems? Open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose) ·
**Status: beta.** Built and tested by one driver with one wheel.

## Read this first

You **must** know these before you install it:

- **ETS2 1.60 only.** It works on the Steam beta branch `temporary_1_60`, not on the regular version
  (1.61) yet. Steam > right-click Euro Truck Simulator 2 > Properties > Betas > `temporary_1_60`. No
  access code is needed; Steam then downloads the older build (a few GB) and the library shows
  `[temporary_1_60]` behind the game's name. A save made with a newer game version may not load in an
  older one, so copy your profile folder (`Documents\Euro Truck Simulator 2`) somewhere safe first. If
  your profile is missing or will not load on 1.60, make a new profile for the test: your normal one is
  untouched and is back when you set the beta to "None" again. DazDrive writes nothing into profiles or
  saves. If Steam ever drops the beta branch, the game goes to 1.61 and DazDrive goes quiet (the dash
  shows A again) until the 1.61 version is out. American Truck Simulator is not supported.
- **Mods:** map and truck mods are fine as long as they work on 1.60 (ProMods: the version made for
  1.60). Only another automatic-gearbox plugin conflicts.
- **Do not use it on TruckersMP.** Its rules forbid tools that change gameplay and its client scans
  memory, so you risk a ban. Remove the DLL before you join. It is made for singleplayer. SCS Convoy is
  untested: drive singleplayer for now; Convoy comes later.
- **Use at your own risk.** DazDrive patches the game's code in memory. It can crash the game and there
  is no guarantee it works on your setup. On a game version it does not know it should find nothing to
  hook and do nothing; it says so in `plugins\dazdrive\dazdrive.log`. That is not a guarantee either.
- **The DLL is not signed**, so Windows Defender or SmartScreen may warn about it. `BUILD-INFO.txt` in
  the zip has the SHA-256 of the DLL, so you can check you have the real file. SmartScreen on the
  download: "Keep anyway". If Windows removes `dazdrive.dll` after you copied it: Windows Security >
  Protection history > Allow (or Restore), then copy it again.
- Not affiliated with SCS Software.

## Why I made this

My biggest pet peeve with the game's automatic is that it doesn't think. It has adaptive modes, and
they are what makes kickdown and the like possible, but they only work without cruise control. I drive
most of the time on cruise control, and then the game just follows the rpm. Climbing or cruising
gently, same shift point, no idea what is coming. After enough hours of that I wanted a gearbox with some brain.

So I looked at what real truck gearboxes do and started building that. Once it worked, the lane
assist was next. It is fine as it is, but I wanted something closer to hands-free: calm enough on a
straight stretch to grab a drink without stopping. (That part is experimental and off by default, and
you still keep your hands near the wheel.) One thing led to another, and a gearbox turned into a
plugin.

If you ever caught yourself thinking "why is it shifting *now*?", this is for you.

## Why you would use it

The game already has an automatic, cruise control and lane assist. This changes how they decide:

- **A gearbox that thinks.** It picks the gear from engine torque, weight and grade instead of fixed rpm:
  lower rpm when it can, skipping gears, learning when an upshift did not pull, rolling in neutral on
  gentle descents, opening the clutch near idle when you brake to a stop.
- **Calmer cruise control.** No throttle bursts after a shift, it holds the set speed downhill with
  the engine brake and retarder, and an emergency braking that does not empty the brake air like the
  game's does.
- **Nothing to set up.** Drop it in and drive; the standard setup has the proven features on and the
  rest off. A setup page is there if you want more. Remove the DLL and everything is stock again.
- **When something goes wrong it keeps the evidence**: the moment is saved as a file you can attach to a
  bug report (see "Reporting a problem").

## What is in it

| Feature | Stage | Default | What it does |
|---|---|---|---|
| **TrueShift** gearbox | stable | on | The game's automatic shifts like a real truck AMT: gear from engine torque, load and grade instead of fixed rpm. Low rpm when it can, learns when an upshift does not pull, skips gears, kickdown, engine brake and retarder aware. Modes Eco / Normal / Power / M, shown as a letter on the dash. |
| Eco-roll | stable | on | Neutral on gentle descents. |
| Smooth stop | stable | on | The clutch opens around idle when you brake to a halt. |
| **SteadyCruise** | stable | on | A calm cruise throttle: no bursts after shifts, no hunting. Most cruise features build on it. |
| Downhill speed control | stable | on | Holds the set speed downhill with engine brake and retarder. |
| Hill start assist | stable | on | The brake holds briefly on a slope. |
| Emergency stop signal | stable | on | Hazards flash after a hard stop. |
| AEB (automatic emergency braking) | stable | on | Warning tones, then partial, then full braking. Replaces the game's own emergency braking. Needs the game's radar: leave an ACC distance, blind spot or emergency braking on in the game's options. |
| Rollover protection | beta | on | A tone, then no throttle, then a brake when the truck starts to sway in a bend (a growing sideways swing). Also when you drive by hand. |
| Gearbox looks ahead | stable | off (~2015) | Predictive shifting: gears chosen for the climbs and tops on your navigation route. |
| Cruise looks ahead (PCC, predictive cruise control) | stable | off (~2015) | Carries speed into climbs and eases off before tops. Needs a route. With it, the truck also coasts in neutral on gentle descents with the cruise on (Fine-tuning, Cruise control). |
| Slow for bends | beta | off (~2020) | Eases off, and brakes gently if needed, before sharp bends. Needs a route. |
| ACC (adaptive cruise control) | beta | off (~2015) | Follows the vehicle ahead smoothly and brakes early. |
| Stop & Go | beta | off (~2015) | The ACC keeps going in a jam: stops behind the vehicle ahead and drives off with it. |
| Auto Hold | beta | off (~2015) | Stays braked at a stop until you give gas. |
| Follow speed limits (ISA, intelligent speed assistance) | beta | off (2024) | The cruise set speed follows the signs. Starts off: hold the game's ACC distance key for a second to switch it on. |
| Crawl mode | experimental | off (~2020) | Creeps at walking pace in 1, 2 and R, for parking and docking. |
| Smooth start | experimental | off (setup page only) | Closes the clutch itself when you pull away: the engine follows the pedal, the clutch slips, then closes gently. |
| **JunctionSense** | experimental | off (~2020) | Slows for junctions and roundabouts on your route. |
| Lane centering | experimental | off (~2020) | Keeps the truck in its lane while the cruise control is on (self-learning). Keep your hands near the wheel: it is an assistant, not an autopilot. |

Stable = tested a lot. Beta = light testing, not confirmed yet. Experimental = not
proven yet. Where a feature has an established industry name (ACC, AEB, Auto Hold, ISA, PCC) it uses
that name; where it has none, it has its own (TrueShift, SteadyCruise, JunctionSense). The year in
brackets is the truck year on the setup page that switches the feature on.

## Install

0. Put ETS2 on the Steam beta `temporary_1_60` (see above).
1. Close the game, then unzip the download from the
   [Releases page](https://github.com/DamienDutch/dazdrive/releases).
2. Copy what is inside the zip's `plugins` folder into `<ETS2>\bin\win_x64\plugins\` (create the
   folder if it is missing): `dazdrive.dll` and the `dazdrive` folder. Steam tip: right-click the
   game > Manage > Browse local files, then open `bin` > `win_x64`.
   On Linux/Proton the folder is
   `~/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/win_x64/plugins/`.
   Check: you now have `...\win_x64\plugins\dazdrive.dll` and `...\win_x64\plugins\dazdrive\dazdrive.ini`,
   not `plugins\plugins\...`.
3. In the game: Options > Gameplay > Transmission type = **Automatic**. On Sequential or Manual the
   gearbox part does nothing; the cruise parts still work. Leave the game's own adaptive cruise control
   and emergency braking switched on there too: DazDrive uses their radar. Start the game. Every start
   shows a warning about plugins ("advanced SDK features"): click OK, that is normal.
4. Check it works: the gear display on the dashboard shows D (or E / S) instead of A, and
   `plugins\dazdrive\dazdrive.log` has a line with "applied". Nothing else appears on screen. If it
   still shows A, read "Which game" above.

Updating: close the game first. Copying the new files over the old ones replaces your
`dazdrive.ini`, so keep a copy of yours and put it back, or make a new one with the setup page. Never
used the setup page? Just overwrite everything.
To switch it off for a while (a stock evening, TruckersMP), move `dazdrive.dll` out of `plugins`, for
example into a folder `plugins_off` next to it, and put it back later. To remove it, delete
`plugins\dazdrive.dll` and the `plugins\dazdrive` folder.

The included `dazdrive.ini` matches the year "~2010 (Modern)" on the setup page: the features marked
"on" above. To change that, open `plugins\dazdrive\dazdrive-setup.html` in a browser (works offline,
sends nothing): pick how smart your truck is (~1995, ~2000, ~2010, ~2015, ~2020 or 2024), switch
single features, save the ini next to the page and restart the game (no button setup: DazDrive uses the keys you bound in the game).
The page walks you through it one step at a time and starts by opening your current `dazdrive.ini`, so you
only change what you want; per module you can fine-tune with Presets, Advanced (a few easy controls) or Custom
(every raw setting). Full details, controls, dash letters and tones:
`README.txt` in the zip ([source](release/README-player.txt)).

**Keys.** Keep your bindings: there is no button setup, DazDrive uses the keys you bind in the game
(Options > Controls; keyboard, wheel or controller, all fine). Only check that cruise control (on/off, + and -,
resume) has a key or button: every cruise feature needs it. Two optional extras, search the Controls list for
"Adaptive": *Adaptive Cruise Control Mode* (ACC distance; hold it for a second to switch speed-limit following on or
off; if your cruise speed suddenly follows the signs, you held it, hold it again) and *Adaptive Automatic
Transmission Mode* (steps the gearbox mode Eco / Normal / Power / M; hold it at a standstill to switch crawl mode).
Unbound, only the part that needs the key does nothing.

## Reporting a problem

[Open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose), or send the same things to
whoever gave you the zip, with:

- the folder `plugins\dazdrive\incidents\` (zipped: right-click the folder > Send to > Compressed
  (zipped) folder) and `plugins\dazdrive\dazdrive.log`. Too big for your chat? Only `incidents` and the
  log, leave the `.csv` files out. When something
  goes wrong (the gearbox hunts, you fight a shift, the rpm hits the limiter, stuck in neutral,
  emergency braking, a ringing lane assist) DazDrive saves that moment as a file there: the 20 s
  before and 15 s after, with the log lines of that time. The name has the local date and time. The
  folder stays under 4 MB and the previous session's log is kept as `dazdrive.prev.log`.
- the game version, truck and engine/gearbox (mods?), trailer or not, whether a route was active,
  and what you were doing.
- nothing in the incidents folder but it felt wrong? Say so. The betas ship with "Bug hunt" on, so the
  `.csv` files in the same folder already have the data: zip them and send them along. The first full
  release will ship quiet ("Quiet" under Logging on the setup page does that now).

## For developers

### Build from source

```
plugin/dazdrive/build.sh            # MinGW g++ in a throwaway Fedora container (podman), no host packages
plugin/dazdrive/build.sh install    # also copy the DLL into ETS2's plugins folder (Steam on Linux)
tools/sim/run.sh                    # gearbox simulator: shift behavior on a set of hills and loads
tools/sim/run_cruise.sh             # cruise / ACC / AEB simulator
tools/check_setup.py                # setup page self-test in headless Chromium
tools/make_release.sh <version>     # build, run the sims, generate presets + setup page, assemble release/dazdrive-<version>.zip
```

The DLL finds the game's functions by byte patterns and hooks them at runtime. On a game build it
does not know, it should find nothing, log that and do nothing, but that is no guarantee (see
"Please read first").

### Repository layout

- `plugin/dazdrive/` – the plugin: `core/` (hooks, telemetry, route, input, config, log) and
  `modules/` (`trueshift.cpp`, `cruise.cpp`, `lane.cpp`, `creep.cpp`).
- `plugin/sdk/` – SCS telemetry SDK headers. `plugin/upstream/` – the original
  `car_automatic_transmission` for reference.
- `tools/` – build, release, ini/setup-page generator (`gen_ini.py` + `dazdrive-setup.tmpl.html`),
  ride-log analysis, simulators (`tools/sim/`).
- `docs/ASSISTANT.md` – a short map for an AI assistant that is helping someone with DazDrive (files, ini, incidents, what to ask for).
- `release/` – the player README template (`README-player.txt`).

## Credits and license

MIT, see [LICENSE](LICENSE).

- The hook layer for the game's automatic gearbox is derived from
  [car_automatic_transmission](https://github.com/deppelover/car_automatic_transmission) by
  Deppelover (MIT): thanks.
- [ets2la_plugin](https://gitlab.com/ETS2LA/ets2la_plugin) (MIT) was used as a reference for the
  route and memory layouts.
- The SCS telemetry SDK is by SCS Software (MIT).
