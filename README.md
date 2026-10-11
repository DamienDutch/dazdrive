# DazDrive

One in-game plugin for Euro Truck Simulator 2 that makes the game's own driver-assistance systems
behave like the real thing. It draws nothing on screen and changes nothing in your save: it only
changes *how* the automatic gearbox, the cruise control and the lane assist decide. Remove the DLL
and everything is stock again. Lekker rijden.

**Status: beta.** Built and tested by one driver with one wheel.

Project: <https://github.com/DamienDutch/dazdrive> · Downloads: [Releases](https://github.com/DamienDutch/dazdrive/releases)
· Problems: [open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose)

## Please read first

- DazDrive patches the game's code in memory. It can crash the game, and there is no guarantee that
  it works on your setup. You use it at your own risk.
- The DLL is not signed, so Windows Defender or SmartScreen may warn about it. `BUILD-INFO.txt` in
  the zip has the version and the SHA-256 of the DLL, so you can check you have the real file.
- Not affiliated with SCS Software.

## Which game

- **ETS2 only, and only the Steam beta branch `temporary_1_60`** (ETS2 1.60). The regular Steam
  version (1.61) is **not supported yet**. To switch: Steam > right-click Euro Truck Simulator 2 >
  Properties > Betas > pick `temporary_1_60`. A save made with a newer game version may not load in
  an older one, so copy your profile folder (`Documents\Euro Truck Simulator 2`) somewhere safe first.
- On any other version the plugin should find nothing to hook and do nothing. It says so in
  `plugins\dazdrive\dazdrive.log` and in the game's own `game.log.txt`. That is not a guarantee.
- American Truck Simulator is not supported in this release: the plugin logs "not supported" and
  stays inactive.
- **Multiplayer:** made for singleplayer. **Not on TruckersMP**: its rules forbid tools that change
  gameplay and its client scans memory, so you would risk a ban; remove the DLL before you join.
  **SCS Convoy** is not tested yet: host your own session and tell the people you drive with. The
  project this one builds on (car_automatic_transmission) advises against using it in Convoy.

## What it does

| Feature | Stage | Default | What it does |
|---|---|---|---|
| **TrueShift** gearbox | stable | on | The game's automatic shifts like a real truck AMT: gear from engine torque, load and grade instead of fixed rpm. Low rpm when it can, learns when an upshift does not pull, skips gears, kickdown, engine brake and retarder aware. Modes Eco / Normal / Power / M, shown as a letter on the dash. |
| Eco-roll | stable | on | Neutral on gentle descents. |
| Smooth stop | stable | on | The clutch opens around idle when you brake to a halt. |
| **SteadyCruise** | stable | on | A calm cruise throttle: no bursts after shifts, no hunting. Most cruise features build on it. |
| Downhill speed control | stable | on | Holds the set speed downhill with engine brake and retarder. |
| Hill start assist | stable | on | The brake holds briefly on a slope. |
| Emergency stop signal | stable | on | Hazards flash after a hard stop. |
| AEB (automatic emergency braking) | stable | on | Warning tones, then partial, then full braking. Replaces the game's own emergency braking. |
| Gearbox looks ahead | stable | off (~2015) | Predictive shifting: gears chosen for the climbs and tops on your navigation route. |
| Cruise looks ahead (PCC, predictive cruise control) | stable | off (~2015) | Carries speed into climbs and eases off before tops. Needs a route. |
| Slow for bends | stable | off (~2020) | Eases off, and brakes gently if needed, before sharp bends. Needs a route. |
| ACC (adaptive cruise control) | beta | off (~2015) | Follows the vehicle ahead smoothly and brakes early. |
| Stop & Go | beta | off (~2015) | The ACC keeps going in a jam: stops behind the vehicle ahead and drives off with it. |
| Auto Hold | beta | off (~2015) | Stays braked at a stop until you give gas. |
| Follow speed limits (ISA, intelligent speed assistance) | beta | off (2024) | The cruise set speed follows the signs. |
| Crawl mode | experimental | off (~2020) | Creeps at walking pace in 1, 2 and R, for parking and docking. |
| **JunctionSense** | experimental | off (~2020) | Slows for junctions and roundabouts on your route. |
| Lane centering | experimental | off (~2020) | Keeps the truck in its lane (self-learning). Keep your hands near the wheel: it is an assistant, not an autopilot. |

Stable = ridden a lot. Beta = built and ridden a little, not confirmed yet. Experimental = not
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
3. Gearbox on **Automatic** in the game (Options > Gameplay). Start the game. The first time, it shows
   a warning about plugins ("advanced SDK features"): click OK.

Updating: close the game first. Copying the new files over the old ones replaces your
`dazdrive.ini`, so keep a copy of yours and put it back, or make a new one with the setup page.
To remove it, delete `plugins\dazdrive.dll` and the `plugins\dazdrive` folder.

The included `dazdrive.ini` matches the year "~2010 (Modern)" on the setup page: the features marked
"on" above. To change that, open `plugins\dazdrive\dazdrive-setup.html` in a browser (works offline,
sends nothing): pick how smart your truck is (~1995, ~2000, ~2010, ~2015, ~2020 or 2024), switch
single features, detect your wheel buttons, save the ini next to the page and restart the game.
"Guide me" walks you through it one step at a time. Full details, controls, dash letters and tones:
`README.txt` in the zip ([source](release/README-player.txt)).

## Reporting a problem

[Open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose) with:

- `plugins\dazdrive\dazdrive.log` (and `dazdrive.log.1` if it exists). **The log is overwritten every
  time the game starts, so copy it before you start the game again.**
- the game version, truck and engine/gearbox (mods?), trailer or not, whether a route was active,
  what you pressed and roughly when.
- if it is about emergency braking: also the folder `plugins\dazdrive\aeb\` (zipped).
- for driving behavior: pick "Bug hunt" under Logging on the setup page, drive again and zip the
  `.csv` files from the same folder. They only exist at log level 2 (`log_level=2`).

## Build from source

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

## Repository layout

- `plugin/dazdrive/` – the plugin: `core/` (hooks, telemetry, route, input, config, log) and
  `modules/` (`trueshift.cpp`, `cruise.cpp`, `lane.cpp`, `creep.cpp`).
- `plugin/sdk/` – SCS telemetry SDK headers. `plugin/upstream/` – the original
  `car_automatic_transmission` for reference.
- `tools/` – build, release, ini/setup-page generator (`gen_ini.py` + `dazdrive-setup.tmpl.html`),
  ride-log analysis, simulators (`tools/sim/`).
- `docs/` – design notes and reverse-engineering notes per feature
  ([`trueshift.md`](docs/trueshift.md), [`aeb.md`](docs/aeb.md),
  [`predictive-cruise.md`](docs/predictive-cruise.md), [`lane.md`](docs/lane.md),
  [`research.md`](docs/research.md), ...), [`eras.md`](docs/eras.md) for when real trucks got what,
  [`horizon.md`](docs/horizon.md) for what could still be built.
- `release/` – the player README template (`README-player.txt`).

## Credits and license

MIT, see [LICENSE](LICENSE).

- The hook layer for the game's automatic gearbox is derived from
  [car_automatic_transmission](https://github.com/deppelover/car_automatic_transmission) by
  Deppelover (MIT): thanks.
- [ets2la_plugin](https://gitlab.com/ETS2LA/ets2la_plugin) (MIT) was used as a reference for the
  route and memory layouts.
- The SCS telemetry SDK is by SCS Software (MIT). DazDrive is not affiliated with SCS Software.
