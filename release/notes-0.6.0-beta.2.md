Second public beta of DazDrive: one in-game plugin for Euro Truck Simulator 2 that makes the game's own automatic gearbox, cruise control and lane assist behave like the real thing. **Use at your own risk.** It patches the game's code in memory and is built and tested by one driver with one wheel.

## What changed since beta.1

These changes were built and run in a simulator; most of them have been driven only a little so far. Treat this as a beta, and please report anything odd.

- **No button setup any more.** DazDrive uses the keys you already bound in the game. Speed-limit following (ISA) switches on and off by holding the game's ACC distance key for about a second; crawl mode by holding the gearbox-mode key at a standstill. Old `*_button` lines in an ini are ignored (the log says so).
- **Incident files.** When something goes wrong (hunting gearbox, a shift you fought, rpm limiter, stuck in neutral, AEB, lane sway) the plugin saves the 20 s before and 15 s after in `plugins\dazdrive\incidents\`. Attach that folder to a bug report. The log of the previous session is kept as `dazdrive.prev.log`, and the log is quiet by default.
- **Gearbox:** an early skip of two or more gears may land up to 100 rpm under the plateau; fixed Auto Hold staying braked on a ~4% slope in a low gear with the pedal flat.
- **Cruise and AEB:** the AEB leaves the braking to the ACC when that already brakes hard enough; the cruise gives no throttle toward a junction or roundabout it has to slow for, nor while you are steering hard; softer braking for bends and junctions; the ACC closes up to about 4 m at a standstill.
- **Crawl mode:** with a heavy load it picks the lowest gear that keeps the engine under 950 rpm.
- **New, experimental, off:** *Smooth start* closes the clutch itself when you pull away (the engine follows the pedal, the clutch slips, then closes gently). Switch it on with the setup page.
- Docs trimmed to what explains behavior.

## Requirements

- **ETS2 1.60 only: the Steam beta branch `temporary_1_60`.** The regular Steam version (1.61) is not supported yet, and neither is American Truck Simulator. On any other version the plugin finds nothing to hook and stays inactive (it says so in `plugins\dazdrive\dazdrive.log`).
- Gearbox on **Automatic** (Options > Gameplay).
- Singleplayer. **Not on TruckersMP** (their rules forbid it). SCS Convoy is not tested yet.

## Install

1. Close the game and unzip the download.
2. Copy what is inside the zip's `plugins` folder into `<ETS2>\bin\win_x64\plugins\` (create the folder if it is missing). This replaces your `dazdrive.ini`: keep a copy of yours or make a new one with the setup page.
3. Start the game and click OK on the plugin warning.

Full instructions, the feature list and the setup page are in the [README](https://github.com/DamienDutch/dazdrive#readme). Problems: [open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose) and attach the `incidents` folder and `plugins\dazdrive\dazdrive.log`.

## What is in the zip

`dazdrive.dll`, the standard `dazdrive.ini` (a ~2010 truck: TrueShift, SteadyCruise, downhill speed control, AEB, hill start assist and eco-roll on; the look-ahead and ACC features off), the setup page `dazdrive-setup.html` to make your own ini, `README.txt`, `LICENSE` and `BUILD-INFO.txt` with the SHA-256 of the DLL. The DLL is not signed, so Windows may warn about it.

SHA-256 of `dazdrive-0.6.0-beta.2.zip`: `f970dfd5e49319b331d072c2304ded12e9aa351f401814b79fbf7f6517a75fad`
