DazDrive @VERSION@
========================

One plugin for Euro Truck Simulator 2 that makes the game's own systems behave more like the real
thing. It adds nothing to the screen and changes nothing in your save: it only adjusts how the
automatic gearbox, the cruise control, the lane assist, the brakes and the low-speed clutch behave,
and at a standstill it can set the parking brake and stop the engine.
Remove the DLL and everything is stock again.

Project page:   https://github.com/DamienDutch/dazdrive
New versions:   https://github.com/DamienDutch/dazdrive/releases
Problems:       https://github.com/DamienDutch/dazdrive/issues/new/choose


PLEASE READ FIRST
-----------------

- DazDrive patches the game's code in memory. It can crash the game, and there is no guarantee that
  it works on your setup. You use it at your own risk.
- The DLL is not signed, so Windows Defender or SmartScreen may warn about it. BUILD-INFO.txt in
  this zip has the version and the SHA-256 of the DLL, so you can check you have the real file.
- Not affiliated with SCS Software.


WHICH GAME
----------

ETS2 1.62, the regular Steam version. No beta branch needed. DazDrive writes nothing into profiles
or saves.

Mods: map and truck mods are fine. Only another automatic-gearbox plugin conflicts (see KNOWN
LIMITS).

On any other version the plugin should find nothing to hook and do nothing. It says so in
plugins\dazdrive\dazdrive.log and in the game's own game.log.txt. That is not a guarantee, though.
After a game update DazDrive may go quiet (the dash shows A again) until a new version is out.

American Truck Simulator 1.62 loads it too, but it is barely tested there. Use ETS2 for now.

Multiplayer:
- Singleplayer is what it is made for.
- NOT on TruckersMP. Their rules forbid tools that change gameplay (rule 2.1) and their client scans
  memory, so you would risk a ban. Remove the DLL before you join.
- SCS Convoy: untested. For now drive singleplayer; Convoy comes later.


INSTALL (2 minutes)
-------------------

1. Close the game.
2. Unzip this file.
3. Copy what is inside the "plugins" folder of the zip into <ETS2>\bin\win_x64\plugins\
   (make that folder if it is missing): dazdrive.dll and the dazdrive folder.
     Steam tip:      right-click the game > Manage > Browse local files, then open bin > win_x64.
     Windows:        C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins\
     Linux / Proton: ~/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/win_x64/plugins/
   Check: you now have ...\win_x64\plugins\dazdrive.dll and ...\win_x64\plugins\dazdrive\dazdrive.ini
   (not plugins\plugins\...). If Windows removed dazdrive.dll: Windows Security > Protection history >
   Allow (or Restore), then copy it again.
4. In the game: Options > Gameplay > Transmission type = Automatic. On Sequential or Manual the
   gearbox part does nothing; the cruise parts still work. Leave the game's own adaptive cruise
   control and emergency braking switched on there too: DazDrive uses their radar.
5. Start the game. Every start shows a warning about plugins ("advanced SDK features"): click OK,
   that is normal.
6. Check it works: the gear display on the dashboard shows D (or E / P) instead of A, and
   plugins\dazdrive\dazdrive.log has a line with "applied". Nothing else appears on screen. Still A?
   Read WHICH GAME.

That is all: the included dazdrive.ini is the standard preset, the developer's own setup (see WHAT
IS ON BY DEFAULT).

Updating: close the game, then copy the new files over the old ones. That replaces your
dazdrive.ini, so keep a copy of yours first and put it back, or make a new one with the setup page.
Never used the setup page? Just overwrite everything.
Off for a while (a stock evening, TruckersMP): move dazdrive.dll out of plugins, for example into a
folder plugins_off next to it, and put it back later.
Remove: delete plugins\dazdrive.dll and the plugins\dazdrive folder.


SETTINGS: THE SETUP PAGE
------------------------

- Open plugins\dazdrive\dazdrive-setup.html in any browser (works offline, installs nothing).
- It walks you through it step by step. First it asks for your current dazdrive.ini: open it and
  you only change what you want. Then keep the DazDrive setup (the ini in this zip) or pick how
  smart your truck is (~1995, ~2000, ~2010, ~2015, ~2020 or 2024), switch single features and, if
  you like, fine-tune each module: Presets (a few good choices), Advanced (a handful of easy
  controls) or Custom (every raw setting). There is no button setup: DazDrive uses the keys you
  already bound in the game (see CONTROLS).
- Press "Save dazdrive.ini", take it from your Downloads and put it next to the page
  (overwrite the old one).
- Restart the game.


WHAT IS ON BY DEFAULT
---------------------

The dazdrive.ini in this zip is the developer's own setup: the "DazDrive" tile on the setup page. For
the earlier, more careful standard, pick the ~2010 year there. The stage says how much a feature has
been driven: stable = tested a lot; beta = light testing, not confirmed yet; experimental = not
proven yet. The standard setup has some beta and experimental features on.

Short names: ACC = adaptive cruise control, AEB = automatic emergency braking, ISA = intelligent
speed assistance (following the speed limits), PCC = predictive cruise control.

ON in the included ini
  TrueShift (the gearbox)        beta (rewritten in 0.7)
  Torque pedal                   beta (new in 0.7)
  Eco-roll                       stable
  Smooth stop                    stable
  SteadyCruise                   stable
  Downhill speed control         stable
  Hill start assist              stable (not used while Auto Hold is on)
  Emergency stop signal          stable
  AEB (emergency braking)        stable
  ACC (DazDrive's own)           beta
  Stop & Go                      beta
  Auto Hold                      beta
  Follow speed limits (ISA)      beta
  Smooth start                   experimental
  Deceleration assist            beta
  Brake pedal                    experimental
  Start-stop and parking brake   experimental
  Lane keeping with the cruise   experimental
  Adjustable speed limiter       experimental

OFF in the included ini (switch on with the setup page; the year that turns it on is in brackets)
  Rollover protection            beta           (~2010)
  Gearbox looks ahead            stable         (~2015)
  Cruise looks ahead (PCC)       stable         (~2015)
  Slow for bends                 beta           (~2020)
  Crawl mode                     experimental   (~2020)
  JunctionSense                  experimental   (~2020)
  Lane centering                 experimental   (~2020)


WHAT IT DOES
------------

Every module can be switched off in dazdrive.ini.

TrueShift (gearbox)
  The game's automatic shifts like a real truck automatic: it picks the gear from engine torque,
  load and grade instead of fixed rpm. Lower rpm when it can, skips gears only when it pays off,
  kickdown, engine brake and retarder aware, clutch opens near idle when you brake to a stop (Smooth
  stop). Modes Eco (diesel first), Normal (D, in between), Power (time first), M (manual: every shift is yours, like Volvo or Mercedes; a warning tone when the wheels
  push the engine over its limit). Eco-roll puts it in neutral on gentle descents with the pedal
  released; with the cruise on it rolls only with "Cruise looks ahead" (the ~2015 year), which knows
  how long the roll lasts. With "Gearbox looks ahead" it also uses the navigation route to pick gears
  for climbs and tops.

Torque pedal
  The throttle asks for a share of what the truck can pull at this speed: half throttle is half of
  what it can pull, loaded or empty, in any gear. Not with the cruise on, not in reverse.

Cruise
  SteadyCruise: a calm cruise throttle (no bursts after shifts, no hunting). Extras, each with its
  own switch: cruise looks ahead (PCC), slowing for bends, JunctionSense (junctions and
  roundabouts, from the navigation route), downhill speed control (engine brake and retarder hold
  the speed on descents), emergency stop signal, hill start assist, AEB, ACC with Stop & Go, Auto
  Hold and following the speed limits (ISA).
  - Cruise + / -: a tap changes the set speed by 5 km/h, holding it steps 1 km/h at a time. A lower
    set speed is reached by easing down.
  - Lowest speeds: the cruise can be switched on from 5 km/h (the game: 30), and the set speed
    never goes below 20 km/h.
  - ACC distances: 1 = 0.9 s, 2 = 1.15 s, 3 = 1.4 s (the "Dutch" preset on the setup page; its other
    presets are wider). It follows the vehicle ahead smoothly, brakes early and stays calm when
    someone cuts in. With Stop & Go it also stops behind a stopping vehicle and drives off with it
    by itself after a stop under 2 s; after a longer stop, press resume or the throttle.
  - AEB: warning tones, then firm braking to a stop behind the vehicle ahead. It steps in by the
    deceleration it would take to stop short of it, so you stay in charge until late (mode
    required_deceleration in [cruise.aeb]; time_to_collision gives warning, partial and full braking
    by time instead). It replaces the game's own emergency braking, which empties the brake air. It
    needs the game's radar, so leave an ACC distance, blind spot or emergency braking switched on in
    the game's options.
  - Rollover protection (beta, off in the included ini): when the truck starts to sway in a bend (a
    growing sideways swing, also when you drive by hand) a tone, then no throttle, then a soft or
    firm brake until it is calm again. It does not judge a single bend by a fixed limit.
  - Emergency stop signal: the hazards flash after braking very hard from above 50 km/h, go off
    again if you brake softer before the stop, and stay on after a full stop.
  - Auto Hold: stays braked after a stop until you give gas. After 60 s the parking brake goes on
    and the service brake lets go (no air spent); gas or resume releases it.
  - ISA: the cruise set speed follows the signs (plus 3 km/h). It starts on in the included ini.
    Setting the speed keeps your current speed, resume goes to the limit. On or off is remembered.
  - Speed limiter (experimental, comes with the "layers" cruise engine, on in the included ini):
    three quick presses of the cruise on/off key switch the cruise into an adjustable speed limiter,
    three more switch back. The pedal drives, the throttle is held so the speed settles on the limit,
    and it never brakes. + and - step the limit.

Braking
  Deceleration assist: a retarder or engine-brake lever step is a share of what the truck can brake
  now (step k of n is k/n of it, the top step is all of it), and the service brake adds what they
  lack, also across gearbox shifts. Stop assist: when you lift off at low speed and the gearbox opens
  the clutch, the truck rolls out to a stop on its own resistance (on a descent the brake only keeps
  the speed from rising) and Auto Hold holds it there.
  Brake pedal (experimental): the pedal asks for a deceleration, linear over the travel and the same
  at any load; the first part brakes with the retarder, the service brake adds the rest. It learns
  the truck's brakes first; until then the pedal is the game's.

Lane
  Lane keeping with the cruise: in the included ini the game steers as always, but only while the
  cruise control is on. The lane key with the cruise off switches the cruise on (at the speed limit;
  without a known limit, at the last set speed or your speed) and lane keeping with it. The cruise
  going off (key, brake, AEB, a junction) switches lane keeping off. Resume or set alone leave it
  off. needs_cruise_enabled=false in [lane.activation] gives the game's own behavior back.
  Lane centering that sways less (experimental, off): mode 0 leaves the game's steering alone, mode 2
  is a calmer law that learns how your wheel responds. Keep your hands near the wheel: it is an
  assistant, not an autopilot.

Creep
  Crawl / maneuver mode: in gears 1, 2 and R the clutch slips at idle so the truck crawls by
  itself, the brake lowers the speed, the throttle asks for a bit more.
  Smooth start (experimental, on in the included ini): when you pull away in a forward gear from a
  standstill, DazDrive closes the clutch itself. The engine rises to a speed that follows the
  pedal, the clutch slips to hold it until the wheels catch up, then closes gently. A shift, lifting
  off, the brake or about 14 km/h ends it and the game takes over. Works with or without crawl mode.

Standstill
  Start-stop (experimental): while Auto Hold holds the truck at a standstill, the engine switches
  off after 3 s. It starts again by itself on the throttle, when the vehicle ahead drives off, when
  the air runs low, or after 3 minutes at the latest.
  Automatic parking brake (experimental): switch the engine off while standing and the parking
  brake goes on; it comes off when you drive off.
  Both press the game's own controls.


EDITING THE INI BY HAND
-----------------------

Every line in plugins\dazdrive\dazdrive.ini is key=value with a comment line above it saying what
it does; 0/1 are off/on. Changes apply after restarting the game. Delete the file and the game
writes the standard preset again; a setting missing from an old ini also gets its standard value.


CONTROLS
--------

Keep your bindings. There is no button setup: DazDrive uses the keys and buttons you bound in the
game (Options > Controls), on a keyboard, a wheel or a controller. Check that these are bound (search
the Controls list for "Cruise" and "Adaptive"), otherwise the part that needs them does nothing
(everything else keeps working):

  Cruise control: on/off, + and -, resume   Needed for every cruise feature (SteadyCruise, ACC, AEB, lane keeping, ...)
  Adaptive Cruise Control Mode              Changes the ACC distance; held for a second: speed-limit
                                            following (ISA) on/off. Without it: no distance change, no ISA switch
  Adaptive Automatic Transmission Mode      Steps the gearbox mode Eco / Normal / Power / M; held at a
                                            standstill: crawl mode on/off. Without it: no mode change, no
                                            crawl switch
  Shift hint up / down (optional)           A hinted gear is held for 10 s

- Gearbox mode: the game's own key for the adaptive automatic ("Adaptive Automatic Transmission
  Mode") steps Eco / Normal / Power / M. TrueShift follows it. A shift hint (up or down) is an
  ordinary hint: the gear it picks is held for 10 s (hint_hold_seconds in [trueshift.manual]).
- Speed-limit following (ISA, beta): it starts on in the included ini. While driving, hold the
  game's ACC distance key ("Adaptive Cruise Control Mode") for about a second to switch it off or on.
  A falling or rising pair of tones tells you which way it went, the distance stays as it was, and
  the choice is remembered. If your cruise speed suddenly stops following the signs, you held that
  key: hold it again.
- Crawl mode (if enabled in the setup page): at a standstill hold the game's gearbox-mode key for
  about a second.
- Speed limiter (experimental): three quick presses of the cruise on/off key switch between the
  cruise and the speed limiter (see "Cruise" above). Pressed by accident? Press it three times again.
- Cruise + / - and the ACC distance key: see "Cruise" above. Nothing else needs a button.

DASHBOARD LETTERS AND TONES
---------------------------

The plugin writes a letter on the dashboard's gear indicator and plays short, quiet tones
(tones_enabled=false in [trueshift.general] silences the mode tones, dashboard_letter_enabled=false
there stops the mode letter, tone_enabled=false in [lane.general] the lane tones).

Letters
  D / E / P / M   Gearbox mode: Normal / Eco / Power / Manual (A in reverse)
  1 / 2 / 3       ACC distance just chosen: close / medium / far (for 1.5 s).
                  C = plain cruise, ACC off. dashboard_letter=letter_a in [cruise.acc] shows A instead
  I / X           Speed-limit following (ISA) on / off (1.5 s). X also for crawl and lane off
  L               Lane keeping switched on (1.5 s)
  C               Crawl mode is on (steady)
  B               Auto Hold / hill hold is braking (steady)

Tones
  A soft bell is used where pitch is the message, a dry steady tone where the count or rhythm is.
  low / middle / rising pair / two equal bells    Gearbox mode Eco / Normal / Power / M
  one, two or three soft tones (slide down: off)  ACC distance close / medium / far (off)
  rising / falling pair of bells                  Feature switched on / off
                                                  (speed-limit following, lane keeping, crawl)
  three short pips                                AEB warning; also the rollover protection's tone
  a quick run of alternating pulses               AEB is braking (faster and harsher when it brakes fully)
  one steady tone (M mode)                        The wheels push the engine over its limit: shift up


KNOWN LIMITS
------------

- ETS2 1.62 only. A game update can stop the plugin from finding its
  hooks; it should then log why and do nothing (nothing is changed in the game's files).
- Route features (predictive shifting, bends, junctions, roundabouts, descents) need an active
  navigation route. Without one the plugin falls back to the live grade and stays careful.
  Roundabout detection is a heuristic and not perfect.
- Lane centering mode 2 is experimental and only tuned on one truck/wheel; the standard
  preset keeps the game's own steering.
- Trucks the plugin cannot model are left to the game's own automatic, with a line in the log:
  electric trucks, 1-2 gear drivelines, torque-converter automatics (it notices when the engine
  rpm does not follow the road speed). Engines with a second torque curve (multi-torque mods) work
  but the second curve is not modeled.
- After the pause menu, loading a save, changing truck or trailer, a ferry/train or quick travel
  the plugin resets its timers and learned values on purpose; the first seconds afterwards drive
  like the game.
- The DLL stays loaded until the game closes; to update it, close the game first.
- Compatibility: only one plugin should hook the automatic gearbox. Do not run it together with
  other automatic-transmission plugins (it expects car_automatic_transmission to be absent, or in
  plugins_off).


REPORTING A PROBLEM
-------------------

Open an issue: https://github.com/DamienDutch/dazdrive/issues/new/choose
(or send the same things to whoever gave you this zip).

Include: game version, truck and engine/gearbox (mods?), trailer or not, whether a route was
active, and what you were doing.

Attach the folder plugins\dazdrive\incidents\ (zipped: right-click the folder > Send to > Compressed
(zipped) folder) and plugins\dazdrive\dazdrive.log. Too big for your chat? Only incidents and the
log, leave the .csv files out. When
something goes wrong (the gearbox hunts between gears, you fight a shift, the rpm hits the limiter,
the truck is stuck in neutral, emergency braking, a ringing lane assist) DazDrive saves that moment
as one file in the incidents folder: the 20 seconds before and 15 seconds after, with the log lines
of that time. The file name has the local date and time, so you can say "it was about then". The
folder is kept under 4 MB (oldest files go first) and dazdrive.log holds one line per incident. The
log of the previous session is kept as dazdrive.prev.log, so you do not have to copy anything
before you start the game again.

If something felt wrong but there is no incident file, say so in the issue: that is useful to know
too. The betas ship with "Bug hunt" on (level=traces in [core.logging] and csv_trace_enabled=true in
[trueshift.general], [cruise.general] and [crawl.general]), so the .csv files in the same folder (trueshift.csv, cruise.csv, lane.csv, creep.csv)
already have the data: zip them and send them along (a few MB each). The first full release will
ship with logging quiet; "Quiet" under Logging on the setup page does that now.


CREDITS AND LICENSE
-------------------

MIT license (see LICENSE). The hook layer for the game's automatic gearbox is derived from
car_automatic_transmission by Deppelover (MIT); thanks.
https://github.com/deppelover/car_automatic_transmission
ets2la_plugin (MIT, gitlab.com/ETS2LA/ets2la_plugin) was used as a reference for the route and
memory layouts.
The SCS telemetry SDK is by SCS Software (MIT). DazDrive is not affiliated with SCS Software.
