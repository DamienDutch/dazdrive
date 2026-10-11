DazDrive @VERSION@
========================

One plugin for Euro Truck Simulator 2 that makes the game's own systems behave more like the real
thing. It adds nothing to the screen and changes nothing in your save: it only adjusts how the
automatic gearbox, the cruise control, the lane assist and the low-speed clutch behave.
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

ETS2 only, and only the Steam beta branch temporary_1_60 (ETS2 1.60). The regular Steam version
(1.61) is NOT supported yet. To switch: Steam > right-click Euro Truck Simulator 2 > Properties >
Betas > pick temporary_1_60. A save made with a newer game version may not load in an older one, so
copy your profile folder (Documents\Euro Truck Simulator 2) somewhere safe first.

On any other version the plugin should find nothing to hook and do nothing. It says so in
plugins\dazdrive\dazdrive.log and in the game's own game.log.txt. That is not a guarantee, though.

American Truck Simulator is not supported in this release: the plugin logs "not supported" and
stays inactive.

Multiplayer:
- Singleplayer is what it is made for.
- NOT on TruckersMP. Their rules forbid tools that change gameplay (rule 2.1) and their client scans
  memory, so you would risk a ban. Remove the DLL before you join.
- SCS Convoy: not tested yet. Host your own session and tell the people you drive with. The project
  this one builds on (car_automatic_transmission) advises against using it in Convoy.


INSTALL (2 minutes)
-------------------

0. Put the game on the Steam beta temporary_1_60 (see above).
1. Close the game.
2. Unzip this file.
3. Copy what is inside the "plugins" folder of the zip into <ETS2>\bin\win_x64\plugins\
   (make that folder if it is missing): dazdrive.dll and the dazdrive folder.
     Steam tip:      right-click the game > Manage > Browse local files, then open bin > win_x64.
     Windows:        C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins\
     Linux / Proton: ~/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/win_x64/plugins/
4. In the game: gearbox on Automatic (Options > Gameplay).
5. Start the game. The first time, it shows a warning about plugins ("advanced SDK features"):
   click OK.

That is all: the included dazdrive.ini is the standard preset (see WHAT IS ON BY DEFAULT).

Updating: close the game, then copy the new files over the old ones. That replaces your
dazdrive.ini, so keep a copy of yours first and put it back, or make a new one with the setup page.
Remove: delete plugins\dazdrive.dll and the plugins\dazdrive folder.


SETTINGS: THE SETUP PAGE
------------------------

- Open plugins\dazdrive\dazdrive-setup.html in any browser (works offline, installs nothing).
- Pick how smart your truck is (~1995, ~2000, ~2010, ~2015, ~2020 or 2024), switch single features,
  set up your wheel buttons ("Detect" finds them). "Guide me" walks you through it step by step.
- Press "Save dazdrive.ini", take it from your Downloads and put it next to the page
  (overwrite the old one).
- Restart the game.


WHAT IS ON BY DEFAULT
---------------------

The dazdrive.ini in this zip matches the year "~2010 (Modern)" on the setup page. The stage says how
much a feature has been driven: stable = ridden a lot; beta = built and ridden a little, not
confirmed yet; experimental = not proven yet.

Short names: ACC = adaptive cruise control, AEB = automatic emergency braking, ISA = intelligent
speed assistance (following the speed limits), PCC = predictive cruise control.

ON in the included ini
  TrueShift (the gearbox)        stable
  Eco-roll                       stable
  Smooth stop                    stable
  SteadyCruise                   stable
  Downhill speed control         stable
  Hill start assist              stable
  Emergency stop signal          stable
  AEB (emergency braking)        stable

OFF in the included ini (switch on with the setup page; the year that turns it on is in brackets)
  Gearbox looks ahead            stable         (~2015)
  Cruise looks ahead (PCC)       stable         (~2015)
  Slow for bends                 stable         (~2020)
  ACC                            beta           (~2015)
  Stop & Go                      beta           (~2015)
  Auto Hold                      beta           (~2015)
  Follow speed limits (ISA)      beta           (2024)
  Crawl mode                     experimental   (~2020)
  JunctionSense                  experimental   (~2020)
  Lane centering                 experimental   (~2020)


WHAT IT DOES
------------

Every module can be switched off in dazdrive.ini.

TrueShift (gearbox)
  The game's automatic shifts like a real truck automatic: it picks the gear from engine torque,
  load, grade and the route ahead instead of fixed rpm. Lower rpm when it can, learns when an
  upshift does not pull, skips gears when the box is slow, kickdown, engine brake and retarder
  aware, clutch opens near idle when you brake to a stop (Smooth stop). Modes Eco, Normal (D),
  Power, M (manual with help). Eco-roll puts it in neutral on gentle descents. With "Gearbox looks
  ahead" it also uses the navigation route to pick gears for climbs and tops.

Cruise
  SteadyCruise: a calm cruise throttle (no bursts after shifts, no hunting). Extras, each with its
  own switch: cruise looks ahead (PCC), slowing for bends, JunctionSense (junctions and
  roundabouts, from the navigation route), downhill speed control (engine brake and retarder hold
  the set speed), emergency stop signal, hill start assist, AEB, ACC with Stop & Go, Auto Hold and
  following the speed limits (ISA).
  - Cruise + / -: a tap changes the set speed by 5 km/h, holding it steps 1 km/h at a time. A lower
    set speed is reached by easing down.
  - ACC distances: 1 = 1.0 s, 2 = 1.25 s, 3 = 1.5 s. It follows the vehicle ahead smoothly, brakes
    early and stays calm when someone cuts in. With Stop & Go it also stops behind a stopping
    vehicle and drives off with it by itself (the "real truck" tuning on the setup page asks for
    resume or the throttle after a longer stop instead).
  - AEB: warning tones, then partial braking, then full braking. It replaces the game's own
    emergency braking, which empties the brake air.
  - Emergency stop signal: the hazards flash after braking very hard from above 50 km/h, go off
    again if you brake softer before the stop, and stay on after a full stop.
  - Auto Hold: stays braked after a stop until you give gas.
  - ISA: the cruise set speed follows the signs (plus 3 km/h). Setting the speed keeps your current
    speed, resume goes to the limit. On or off is remembered.

Lane
  Lane centering that sways less. Mode 0 leaves the game's assist alone, mode 2 is a calmer law
  that learns how your wheel responds (experimental). Keep your hands near the wheel: it is an
  assistant, not an autopilot.

Creep
  Crawl / maneuver mode: in gears 1, 2 and R the clutch slips at idle so the truck crawls by
  itself, the brake lowers the speed, the throttle asks for a bit more.


EDITING THE INI BY HAND
-----------------------

Every line in plugins\dazdrive\dazdrive.ini is key=value with a comment line above it saying what
it does; 0/1 are off/on. Changes apply after restarting the game. Delete the file and the game
writes the standard preset again; a setting missing from an old ini also gets its standard value.


CONTROLS AND FINDING YOUR BUTTON NUMBERS
----------------------------------------

The plugin reads wheel/joystick buttons directly. Numbers in the ini count from 1, like the Windows
game controller panel.

Easiest: the setup page's "Detect" button. Recommended: use the same buttons you bound in the game
for Shift Up Hint, Shift Down Hint and Adaptive Cruise Control Mode: a short press does what the
game does, holding it 0.8 s adds the extra. The game's controls count from 0, the plugin from 1
("Joy Button 22" there = 23 here). Buttons 1 to 128 can be read.

By hand: press the button once in the game, open plugins\dazdrive\dazdrive.log and look for
"input: joystick 0 button 14 pressed".

- Gearbox mode: the game's own key for the adaptive automatic (Options > Controls) steps
  Eco / Normal / Power / M. TrueShift follows it. Or hold the shift-hint buttons (mode_up_button /
  mode_down_button; 0 = off in the standard preset) for 0.8 s to step Eco / Normal / Power (M only
  with the game's key); a short press stays an ordinary shift hint, and in M it is a manual shift.
  A hint also holds its gear for 10 s (hint_hold_s).
- Keyboard only? The mode key and the cruise keys of the game work. The long-press on hint buttons
  and the wheel-button options need a joystick/wheel button. The crawl toggle also has toggle_key
  (a Windows virtual-key code, e.g. 67 = C).
- Speed-limit following (ISA): hold speed_limit_button (0 = off in the standard preset) for 0.8 s.
- Crawl mode (if enabled): at a standstill hold the game's gearbox-mode key (or toggle_button)
  for 0.8 s.


DASHBOARD LETTERS AND TONES
---------------------------

The plugin writes a letter on the dashboard's gear indicator and plays short, quiet tones
(tones=0 in the [trueshift] section silences the mode tones, dash_letter=0 stops the mode letter,
tone=0 in [lane] the lane tones).

Letters
  D / E / P / M   Gearbox mode: Normal / Eco / Power / Manual with help (A in reverse)
  1 / 2 / 3       ACC distance just chosen: close / medium / far (for 1.5 s).
                  C = plain cruise, ACC off. acc_letter=1 shows A instead
  S / X           Speed-limit following on / off (1.5 s). X also for crawl and lane off
  L               Lane centering switched on (1.5 s)
  C               Crawl mode is on (steady)
  B               Auto Hold / hill hold is braking (steady)

Tones
  one low / one middle / two rising / two short   Gearbox mode Eco / Normal / Power / M
  one, two or three equal beeps (a low one: off)  ACC distance close / medium / far (off)
  rising / falling pair                           Feature switched on / off
                                                  (speed-limit following, lane centering, crawl)


KNOWN LIMITS
------------

- ETS2 on the Steam beta temporary_1_60 only. A game update can stop the plugin from finding its
  hooks; it should then log why and do nothing (nothing is changed in the game's files).
- Route features (predictive shifting, bends, junctions, roundabouts, descents) need an active
  navigation route. Without one the plugin falls back to the live grade and stays careful.
  Roundabout detection is a heuristic and not perfect.
- Lane centering mode 2 is experimental and only tuned on one truck/wheel; the standard
  preset keeps the game's own.
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
active, what you pressed, and the in-game time or odometer of the problem.

Attach plugins\dazdrive\dazdrive.log (and dazdrive.log.1 if it exists). The log is overwritten every
time the game starts, so copy it somewhere BEFORE you start the game again. The first lines say
which version and which modules are running.

If it is about emergency braking, also attach the folder plugins\dazdrive\aeb\ (zipped).

For a driving problem, pick "Bug hunt" under Logging on the setup page (or set log_level=2 and
trace=1 in [trueshift], [cruise] and [creep]), drive again and also send the .csv files from the
same folder (trueshift.csv, cruise.csv, lane.csv, creep.csv); they are large, so zip them. The
csv traces only exist at log_level 2.


CREDITS AND LICENSE
-------------------

MIT license (see LICENSE). The hook layer for the game's automatic gearbox is derived from
car_automatic_transmission by Deppelover (MIT); thanks.
https://github.com/deppelover/car_automatic_transmission
ets2la_plugin (MIT, gitlab.com/ETS2LA/ets2la_plugin) was used as a reference for the route and
memory layouts.
The SCS telemetry SDK is by SCS Software (MIT). DazDrive is not affiliated with SCS Software.
