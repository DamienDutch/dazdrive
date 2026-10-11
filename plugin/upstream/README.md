# ETS2 Improved Automatic Transmission

A plugin for Euro Truck Simulator 2 that makes the automatic gearbox behave like a car's automatic instead of a truck's. Written for car mods, but it applies to every vehicle.

## Requirements

Developed and verified against Euro Truck Simulator 2, Windows 64-bit, Version 1.61.0.254. Tested with Pedals and Keyboard.

It locates the gearbox code by byte patterns instead of fixed addresses, so it normally survives game updates and if it cannot find what it expects, it does nothing at all.

## Install

Download the latest zip from [Releases](https://github.com/deppelover/car_automatic_transmission/releases), then copy the `bin` folder from it into your ETS2 installation directory, so the DLL ends up here:

```
Euro Truck Simulator 2\bin\win_x64\plugins\car_automatic_transmission.dll
```

Create the `plugins` folder if it does not exist. Start the game once and a `car_automatic_transmission.cfg` holding the defaults appears next to the DLL.

To uninstall, just delete the DLL.

Please provide feedback if you encounter any bugs or if something isn't working properly. I will fix it. 

## Configuration

Edit `car_automatic_transmission.cfg` and restart the game. Every key is optional and anything you leave out keeps its default.

### Gear selection

| Key                  | Default |   Range   | What it does                                             |
| :------------------- | :-----: | :-------: | :------------------------------------------------------- |
| `enabled`            |   `1`   | `0` / `1` | `0` turns the plugin off completely                      |
| `up_step`            |   `1`   |   `0`+    | Gears per upshift. `1` = sequential, `0` = unlimited     |
| `down_step`          |   `2`   |   `0`+    | Gears per downshift                                      |
| `kick_step`          |   `0`   |   `0`+    | Gears per kickdown. `0` = as far as needed               |
| `downshift_on_accel` |   `1`   | `0` / `1` | `1` allows downshifting while accelerating, `0` is stock |

### Shift map

Moves the rpm window up with the throttle, so the gearbox downshifts earlier under load and holds gears longer.

| Key          | Default |    Range    | What it does                                                                |
| :----------- | :-----: | :---------: | :-------------------------------------------------------------------------- |
| `min_rpm_hi` | `0.65`  | `0.0`-`1.0` | Main control. Minimum rpm at full pedal, as a fraction of the rev limit     |
| `thr_lo`     | `0.45`  | `0.0`-`1.0` | Below this pedal position, shift points stay stock                          |
| `thr_hi`     | `0.85`  | `0.0`-`1.0` | At this pedal position the map is fully applied                             |
| `thr_curve`  |  `1.6`  | `0.5`-`3.0` | Above `1` makes the lower pedal range gentler                               |
| `thr_peak`   | `0.90`  | `0.0`-`1.0` | Ceiling for the top of the window, as a fraction of the rev limit           |
| `thr_gain`   | `3.00`  |    `0`+     | Safety ceiling on the shift. `0` disables the map                           |

Tuning: Downshifts too eager -> lower `min_rpm_hi` or raise `thr_lo`. Too lazy -> raise `min_rpm_hi` or lower `thr_hi`.

### Shift timing

| Key               | Default |  Range  | What it does                               |
| :---------------- | :-----: | :-----: | :----------------------------------------- |
| `upshift_delay`   | `0.80`  | seconds | Lockout before the next upshift. `0` = off |
| `downshift_delay` | `0.00`  | seconds | Same for downshifts                        |

### Diagnostics

| Key           | Default |   Range   | What it does                                          |
| :------------ | :-----: | :-------: | :---------------------------------------------------- |
| `status_file` |   `0`   | `0` / `1` | `1` logs one line to `car_automatic_transmission.txt` |

## Multiplayer

Single player only. Do not use with TruckersMP or Convoy.

## Antivirus

The DLL writes to the game's code in memory, which some scanners flag by behaviour alone. It is unsigned. The full source is included so you can read it and build the DLL yourself.

## Building

Requires a MinGW-w64 toolchain such as w64devkit:

```
cd src
build.bat
```

## License

MIT, see LICENSE.
