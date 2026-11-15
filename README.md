# DazDrive

A plugin for Euro Truck Simulator 2 that makes the game's automatic gearbox, cruise control and lane
assist behave like the real thing. Lekker rijden.

**[Download the latest release](https://github.com/DamienDutch/dazdrive/releases)** ·
[Problems? Open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose) ·
**Status: beta.** Built and tested by one driver with one wheel.

## Read this first

You **must** know these before you install it:

- **ETS2 1.62**, the regular Steam version. No beta branch needed. After a game update DazDrive may go
  quiet (the dash shows A again) until a new version is out. DazDrive writes nothing into profiles or
  saves. American Truck Simulator 1.62 loads it too, but it is barely tested there.
- **Mods:** map and truck mods are fine. Only another automatic-gearbox plugin conflicts.
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
  lower rpm when it can, skipping gears only when it pays off, rolling in neutral on gentle descents,
  opening the clutch near idle when you brake to a stop. Eco saves diesel, Power saves time, Normal is in
  between.
- **A pedal that means the same in every gear.** Half throttle asks for half of what the truck can pull,
  loaded or empty, in any gear.
- **Calmer cruise control.** No throttle bursts after a shift, it holds the speed downhill with
  the engine brake and retarder, and an emergency braking that does not empty the brake air like the
  game's does.
- **Nothing to set up.** Drop it in and drive. The standard setup is the developer's own, so most
  features are on; the look-ahead ones, bends, junctions and own lane centering are off. A setup page
  is there if you want something else. Remove the DLL and everything is stock again.
- **When something goes wrong it keeps the evidence**: the moment is saved as a file you can attach to a
  bug report (see "Reporting a problem").

## What is in it

| Feature | Stage | Default | What it does |
|---|---|---|---|
| **TrueShift** gearbox | beta | on | The game's automatic shifts like a real truck AMT: gear from engine torque, load and grade instead of fixed rpm. Low rpm when it can, skips gears when it pays off, kickdown, engine brake and retarder aware. Modes Eco (diesel first) / Normal / Power (time first) / M, shown as a letter on the dash. Rewritten in 0.7. |
| Torque pedal | beta | on | The throttle asks for a share of what the truck can pull at this speed, so the same pedal feels the same loaded or empty and in every gear. Not with the cruise on. New in 0.7. |
| Eco-roll | stable | on | Neutral on gentle descents with the pedal released. With the cruise on it rolls only with "Cruise looks ahead" (~2015), which knows how long the roll lasts. |
| Smooth stop | stable | on | The clutch opens around idle when you brake to a halt. |
| **SteadyCruise** | stable | on | A calm cruise throttle: no bursts after shifts, no hunting. Most cruise features build on it. |
| Downhill speed control | stable | on | Holds the speed downhill with engine brake and retarder: the set speed plus a small margin. |
| Hill start assist | stable | on | The brake holds briefly on a slope. Not used while Auto Hold is on. |
| Emergency stop signal | stable | on | Hazards flash after a hard stop. |
| AEB (automatic emergency braking) | stable | on | Warning tones, then firm braking to a stop behind the vehicle ahead. It steps in by the deceleration it would take to stop short of it, so you stay in charge until late. Replaces the game's own emergency braking. Does not brake for a vehicle beside your path, such as the one you are passing. Needs the game's radar: leave an ACC distance, blind spot or emergency braking on in the game's options. |
| ACC (adaptive cruise control) | beta | on | DazDrive's own, built on the game's radar. Follows the vehicle ahead smoothly and brakes early. Gaps 0.9 / 1.15 / 1.4 s at the three distance settings. |
| Stop & Go | beta | on | The ACC keeps going in a jam: stops behind the vehicle ahead and drives off with it, by itself after a stop under 2 s (Resume or the throttle after a longer one). The cruise can be switched on from 5 km/h (the game: 30), and the set speed never goes below 20. |
| Auto Hold | beta | on | Stays braked at a stop until you give gas. After 60 s the parking brake goes on. |
| Follow speed limits (ISA, intelligent speed assistance) | beta | on | The cruise set speed follows the signs (limit plus 3 km/h), and Resume goes to the limit. Hold the game's ACC distance key for a second to switch it off. |
| Smooth start | experimental | on | Closes the clutch itself when you pull away: the engine follows the pedal, the clutch slips, then closes gently. |
| Deceleration assist | beta | on | A retarder or engine-brake lever step brakes with a share of what the truck can brake now (the top step is all of it). The service brake adds what they lack, also across gearbox shifts. Stop assist: lift off at low speed and the truck rolls to a stop, held by Auto Hold. |
| Brake pedal | experimental | on | The pedal asks for a deceleration, linear over the travel and the same at any load. The first part brakes with the retarder. It learns the truck's brakes first: until then the pedal is the game's. |
| Start-stop and parking brake | experimental | on | Under Auto Hold the engine switches off after 3 s at a standstill and starts again with the throttle, when the vehicle ahead drives off, or when the air runs low. Switch the engine off yourself and the parking brake goes on; it comes off when you drive off. |
| Lane keeping with the cruise | experimental | on | The game steers, as before, but only while the cruise is on. The lane key switches the cruise on at the speed limit, and the cruise going off switches lane keeping off. |
| Adjustable speed limiter | experimental | on | Three quick presses of the cruise on/off key switch the cruise into a limiter: the pedal drives, the throttle is held at the limit, and it never brakes. Three more switch back. Comes with the "layers" cruise engine. |
| Rollover protection | beta | off (~2010) | A tone, then no throttle, then a brake when the truck starts to sway in a bend (a growing sideways swing). Also when you drive by hand. |
| Gearbox looks ahead | stable | off (~2015) | Predictive shifting: gears chosen for the climbs and tops on your navigation route. |
| Cruise looks ahead (PCC, predictive cruise control) | stable | off (~2015) | Carries speed into climbs and eases off before tops. Needs a route. With it, the truck also coasts in neutral on gentle descents with the cruise on (Fine-tuning, Cruise control). |
| Slow for bends | beta | off (~2020) | Eases off, and brakes gently if needed, before sharp bends. Needs a route. |
| Crawl mode | experimental | off (~2020) | Creeps at walking pace in 1, 2 and R, for parking and docking. |
| **JunctionSense** | experimental | off (~2020) | Slows for junctions and roundabouts on your route. |
| Lane centering | experimental | off (~2020) | Keeps the truck in its lane while the cruise control is on (self-learning). Keep your hands near the wheel: it is an assistant, not an autopilot. |

Stable = tested a lot. Beta = light testing, not confirmed yet. Experimental = not
proven yet. The standard setup is the developer's own, so it has some beta and experimental features
on. Where a feature has an established industry name (ACC, AEB, Auto Hold, ISA, PCC) it uses
that name; where it has none, it has its own (TrueShift, SteadyCruise, JunctionSense). The year in
brackets is the truck year on the setup page that switches the feature on.

## Install

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
4. Check it works: the gear display on the dashboard shows D (or E / P) instead of A, and
   `plugins\dazdrive\dazdrive.log` has a line with "applied". Nothing else appears on screen. If it
   still shows A, read "Read this first" above.

Updating: close the game first. Copying the new files over the old ones replaces your
`dazdrive.ini`, so keep a copy of yours and put it back, or make a new one with the setup page. Never
used the setup page? Just overwrite everything.
To switch it off for a while (a stock evening, TruckersMP), move `dazdrive.dll` out of `plugins`, for
example into a folder `plugins_off` next to it, and put it back later. To remove it, delete
`plugins\dazdrive.dll` and the `plugins\dazdrive` folder.

The included `dazdrive.ini` matches the "DazDrive" tile on the setup page: the features marked
"on" above. To change that, open `plugins\dazdrive\dazdrive-setup.html` in a browser (works offline,
sends nothing): keep DazDrive or pick how smart your truck is (~1995, ~2000, ~2010, ~2015, ~2020 or 2024;
~2010 is the earlier, more careful standard), switch
single features, save the ini next to the page and restart the game (no button setup: DazDrive uses the keys you bound in the game).
The page walks you through it one step at a time and starts by opening your current `dazdrive.ini`, so you
only change what you want; per module you can fine-tune with Presets, Advanced (a few easy controls) or Custom
(every raw setting). Full details, controls, dash letters and tones:
`README.txt` in the zip ([source](release/README-player.txt)).

**Keys.** Keep your bindings: there is no button setup, DazDrive uses the keys you bind in the game
(Options > Controls; keyboard, wheel or controller, all fine). Only check that cruise control (on/off, + and -,
resume) has a key or button: every cruise feature needs it. Two optional extras, search the Controls list for
"Adaptive": *Adaptive Cruise Control Mode* (ACC distance; hold it for a second to switch speed-limit following off or
on; if your cruise speed suddenly stops following the signs, you held it, hold it again) and *Adaptive Automatic
Transmission Mode* (steps the gearbox mode Eco / Normal / Power / M; hold it at a standstill to switch crawl mode).
Unbound, only the part that needs the key does nothing. Good to know: three quick presses of the cruise on/off
key switch between the cruise and the speed limiter.

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
"Read this first").

### Repository layout

- `plugin/dazdrive/` – the plugin: `core/` (hooks, telemetry, route, input, config, log) and
  `modules/` (`trueshift.cpp`, `pedal.cpp`, `cruise.cpp`, `lane.cpp`, `creep.cpp` and more).
- `plugin/sdk/` – SCS telemetry SDK headers. `plugin/upstream/` – the original
  `car_automatic_transmission` for reference.
- `tools/` – build, release, ini/setup-page generator (`gen_ini.py` + `dazdrive-setup.tmpl.html`),
  the simulators the release runs (`tools/sim/`).
- `release/` – the player README template (`README-player.txt`).

## Credits and license

MIT, see [LICENSE](LICENSE).

- The hook layer for the game's automatic gearbox is derived from
  [car_automatic_transmission](https://github.com/deppelover/car_automatic_transmission) by
  Deppelover (MIT): thanks.
- [ets2la_plugin](https://gitlab.com/ETS2LA/ets2la_plugin) (MIT) was used as a reference for the
  route and memory layouts.
- The SCS telemetry SDK is by SCS Software (MIT).
