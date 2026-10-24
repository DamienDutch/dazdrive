Third public beta of DazDrive: one in-game plugin for Euro Truck Simulator 2 that makes the game's own automatic gearbox, cruise control and lane assist behave like the real thing. **Use at your own risk.** It patches the game's code in memory and is built and tested by one driver with one wheel.

## What changed since beta.2

Most of this was built in a simulator and driven only a little. Treat it as a beta, and please report anything odd (attach the `incidents` folder).

- **Gearbox**
  - Pulling away: it now takes the start gear the physics ask for when the game picked a lower one, so a heavy trailer no longer climbs 2 > 3 > 4 > 5 right after the start.
  - Light throttle: it waits a moment for a skip of two or more gears instead of climbing every single step, and it no longer shifts up while you roll out toward a junction.
  - Fewer hunts after braking; skip downshifts while braking; calmer gears below about 12 km/h when maneuvering.
  - Climbs: it shifts down earlier when a gear can only just hold the speed, and at once when it cannot hold it.
  - M mode has three styles (`m_style`: European, protected, assisted), an overspeed warning tone, and it now honors your shift hint at the rev limiter.
- **Eco-roll:** rolls more often in Eco, gets a short speed boost before a roll, and never rolls toward a junction, roundabout or bend you have to slow for.
- **Cruise:**
  - The throttle eases by the speed being gained, so a pull to the set speed overshoots less.
  - A roundabout always cancels the cruise when it is approached too fast; there is no throttle toward a junction or roundabout from 400 m out.
  - The look-ahead reads the road between two nodes as a smooth curve, so tight bends are seen earlier.
- **Sounds:** soft bells for pitch cues, dry tones for counts, one climbing AEB alarm, and quiet-event sounds for lane assist pause and for the cruise dropping by itself.
- **Setup page:** the wizard is the page (open your ini, pick a truck era, switch features, fine-tune in three layers: Presets, Advanced, Custom).
- **Lane assist (still experimental):** a runaway guard hands the wheel back to the game's own assist when the request hits its ceiling in alternating directions or the heading error grows. Keep-right, shared steering and bend blend exist but are off by default.
- **Internal:** the unused button reading is gone, which also drops the DirectInput thread.

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

`dazdrive.dll`, the standard `dazdrive.ini`, the setup page `dazdrive-setup.html` to make your own ini, `README.txt`, `LICENSE` and `BUILD-INFO.txt`.

SHA-256 of `dazdrive-0.6.0-beta.3.zip`: `3502e10b33fb57a75869e0102df5fe7e1f35d7d07a72fd7cc21ee4d1129e5db9`
