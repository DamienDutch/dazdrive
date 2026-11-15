First beta of DazDrive 0.7: one in-game plugin for Euro Truck Simulator 2 that makes the game's own automatic gearbox, cruise control and lane assist behave like the real thing. **Use at your own risk.** It patches the game's code in memory and is built and tested by one driver with one wheel.

## What changed since 0.6.0-beta.4

0.7 is a rewrite of the gearbox, the pedal and the cruise's braking. Much of it was built in a simulator and driven by one driver. Treat it as a beta, and please report anything odd (attach the `incidents` folder).

- **The standard `dazdrive.ini` is now the developer's own setup.** Much that was off is on. The setup page has a new "DazDrive" tile for it, preselected, after the truck years. The truck years mean what they did before: pick ~2010 for the earlier standard.
  - **ACC:** DazDrive's own controller instead of the game's, gaps 0.9 / 1.15 / 1.4 s. Stop & Go drives off by itself after a stop under 2 s. Auto Hold keeps the brake on at a stop and sets the parking brake after 60 s.
  - **Speed limit following (ISA):** the cruise set speed follows the signs, and Resume goes to the limit.
  - **Cruise speeds and engine:** the cruise can be switched on from 5 km/h, and the lowest set speed is 20. The "layers" cruise engine (experimental) is on; it adds an adjustable speed limiter, switched on with three quick presses of the cruise on/off key.
  - **AEB:** it steps in by the deceleration needed to stop short of the vehicle ahead, so you stay in charge until late.
  - **Pulling away and braking:** smooth start (the plugin's clutch) and the deceleration assist: a retarder or engine-brake lever step is a deceleration, and the stop assist brakes and holds. The brake pedal asks for a deceleration too, the same at any load.
  - **Start-stop and the automatic parking brake** now press the game's controls: under Auto Hold the engine stops at a standstill, and the throttle starts it again.
  - **Lane keeping:** the game steers, as before, but only while the cruise is on. The lane key switches the cruise on at the speed limit, and the cruise going off switches lane keeping off.
  - **Off now:** rollover protection and the speed cap for the bend the truck is in. **Still off:** gearbox and cruise look-ahead (PCC), bends, junctions, own lane centering.
- **ETS2 1.62:** the regular Steam version. No beta branch any more.
- **Gearbox (TrueShift), rewritten**
  - Every gear is weighed on time and diesel. Eco puts diesel first, Power puts time first, Normal is in between.
  - It skips gears only when that pays off. With a light throttle it shifts one gear at a time.
  - Pulling away: the start gear follows the load and the slope (an empty truck starts higher).
  - With the game's speed limiter, it holds the limit in a high gear instead of kicking down.
  - The dashboard shows **P** for Power again (was S).
- **Torque pedal (new, on):** the throttle asks for a share of what the truck can pull at this speed. Half throttle feels the same loaded or empty, in any gear.
- **Engine brake and retarder:** the lever's stage sets the rpm band, and a downshift under the lever no longer overshoots it.
- **Emergency braking (AEB):** no more braking for a vehicle beside your path, such as the truck you are passing. It reads the game's traffic list for that (`[traffic] probe`, on).
- **Cruise:** the game's allowed overspeed downhill is the room left before a speeding fine. With the cruise on, about 5 km/h. The highest set speed follows the game's own speed limiter (90 km/h for a truck); a bus gets 100.
- **Lane centering (still experimental, off):** calibration finds more usable stretches.

## Known issues

- **Buses:** they are driven as trucks. The 100 km/h set speed cap is new and not driven yet.
- **Stutter:** the beta logs a lot for bug reports. If the game stutters, set `level=short` under `[core.logging]` and `csv_trace_enabled=false` in `dazdrive.ini`.

## Requirements

- **ETS2 1.62.** American Truck Simulator 1.62 loads it too, but it is barely tested there. On any other version the plugin should find nothing to hook and stay inactive (it says so in `plugins\dazdrive\dazdrive.log`).
- Gearbox on **Automatic** (Options > Gameplay).
- Singleplayer. **Not on TruckersMP** (their rules forbid it). SCS Convoy is not tested yet.

## Install

1. Close the game and unzip the download.
2. Copy what is inside the zip's `plugins` folder into `<ETS2>\bin\win_x64\plugins\` (create the folder if it is missing). This replaces your `dazdrive.ini`: an ini from 0.6 still loads, but make a new one with the setup page to get the new defaults.
3. Start the game and click OK on the plugin warning.

Full instructions, the feature list and the setup page are in the [README](https://github.com/DamienDutch/dazdrive#readme). Problems: [open an issue](https://github.com/DamienDutch/dazdrive/issues/new/choose) and attach the `incidents` folder and `plugins\dazdrive\dazdrive.log`.

## What is in the zip

`dazdrive.dll`, the standard `dazdrive.ini`, the setup page `dazdrive-setup.html` to make your own ini, `README.txt`, `LICENSE` and `BUILD-INFO.txt`.

SHA-256 of `dazdrive-0.7.0-beta.1.zip`: (filled in by the release)
