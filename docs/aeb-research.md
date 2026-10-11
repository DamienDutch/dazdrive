# DazDrive AEB: research and design proposal

Goal: our own AEB that brakes earlier and softer than the game's (which empties the air: 135 -> 25 psi in 4 s, 138 -> 5 psi in 1.3 s)
and does not react to crossing traffic on roundabouts. Hard rule: own code only (MIT). Other projects = ideas only.
Radar/brake hook background: `docs/autohold-acc.md`.

Confidence tags: **[src]** read in the linked source, **[mem]** from general knowledge, not verified online in this pass (check R131 text before quoting).

## 1. MonoCruise AEB (ideas only)

Sources: [repo](https://github.com/luuukske/MonoCruise) (MIT; bundles CC0/MIT/BSD-3 third-party code), [project page](https://ld-tech.org/projects/monocruise/).
Only the README and project page could be read, not the source (GitHub code search needs auth). Source dirs `core/`, `shared/`, `tests/` exist; reading them is the
next step if more detail is wanted (license allows reading; we still write our own).

- It is an external Python program (Windows, physical pedals) sitting between pedals and game. Data comes from the ETS2LA plugin (AI + multiplayer vehicles),
  not from the game's radar. Different data source than ours.
- **Detection:** "trajectory based": it looks at traffic "along the path your truck is taking" (predicted path, not just straight ahead). No TTC or thresholds are published.
- **Staging:** beep + flashing cruise panel, then "brakes as hard as needed to stop behind the vehicle". Braking force is shaped by the same learned pedal model as its ACC
  (so a ramp, not a step). No separate partial stage documented.
- **Override:** fully pressing the gas cancels it; pressing the brake does NOT cancel it (it keeps braking, the driver can only add).
- **False positives (their own words):** "highly sensitive when left unfiltered", brakes needlessly at intersections and for vehicles changing lane beside you. They add
  hand-made filters to exclude non-dangerous vehicles; a neural net is planned. Off by default, marked experimental, opt-in upload of AEB event recordings.
- **Lessons for us:** (a) the crossing/lane-change false positive is the known hard problem for everybody; (b) default off or conservative; (c) log every event so
  filters can be tuned from real data; (d) braking should follow a ramp.

## 2. Other ETS2/ATS tools and player complaints

- SCS shipped ACC + emergency braking hidden in 1.44, official in 1.47 ([coverage](https://traxion.gg/american-truck-simulator-and-euro-truck-simulator-2-v1-47-update-drops-featuring-real-world-truck-tech/)).
  Stock behavior: if the gap to the vehicle ahead is too small it applies full brakes + hazard lights; accelerator cancels a false alarm
  ([gamepretty guide](https://gamepretty.com/euro-truck-simulator-2-how-to-enable-adaptive-cruise-control-and-emergency-brake/)).
- **The game has a switch:** console cvar `g_emergency_brake` 0/1 (same source), and per the Steam thread an option "emergency brake detection" with a "vehicle only"
  setting that moderators recommend against phantom stops ([Steam thread, 2024](https://steamcommunity.com/app/227300/discussions/0/4416424435093571055)).
  Complaints there: truck "slows and stops unexpectedly", beeps and will not move (jitters like the handbrake is on). Roundabout-specific complaints: found none online;
  the roundabout issue is our own observation (AI cutting in front at roundabouts is a known general complaint).
  In our own log (`autohold-acc.md`): with the game's AEB option off the request at end was always 0, so the option works for the game's AEB.
- **Open question:** is the cvar `g_emergency_brake` settable from the plugin (SDK has no cvar API; we already read cvars like `g_acc`, see `docs/acc.md`)?
  Proposal: when our AEB is enabled, set `g_emergency_brake 0` ourselves and restore the player's value on unload; otherwise tell the player in the setup page to switch the game's off.
- [ETS2LA](https://ets2la.com/) is the other big one (AI vehicle data, plugins); its AEB-like code was not examined (idea source only, MonoCruise uses its data).
- Real-world context for the false-positive problem: NHTSA probes into phantom braking
  ([Freightliner Cascadia AEB](https://www.truckingdive.com/news/freightliner-cascadia-aeb-system-identified-in-nhtsa-investigation/652016/)): static objects overhead, bridges.

## 3. Real truck AEBS

Sources: [EU 2015/562 (R131 test table, level 2)](https://www.legislation.gov.uk/eur/2015/562/data.html), [Daimler ABA 5 press release](https://www.daimlertruck.com/en/newsroom/pressrelease/trucks-from-mercedes-benz-safe-as-never-before-44842137),
[Mercedes Econic ABA 5](https://special.mercedes-benz-trucks.com/en/econic/econic-safety-assistance-systems/active-brake-assist-5.html),
[GRVA proposals](https://globalautoregs.com/documents/17246), [Japan MLIT summary of the 2022 R131 amendment](https://www.mlit.go.jp/report/press/content/001579809.pdf).

**UN R131 / EU 2015/562 (heavy: M3, N3, N2 > 8 t, pneumatic) [src]:**
- Warning before the emergency braking phase starts: >= 1 warning (haptic/acoustic) at least **1.4 s** before; >= 2 warning modes at least **0.8 s** before. Both for stationary and moving targets.
- Level 2 test (older table): stationary target must produce a **speed reduction >= 20 km/h**; moving target (12 km/h) must not be hit. Lighter N2/M2: >= 10 km/h.
- Amended 2022 (WP.29): speed reduction raised; approaching a slower vehicle at <= 60 km/h must slow by >= 40 km/h or stop; tests with stationary and moving targets at ~70 km/h
  plus a pedestrian test (Japan summary, translated). The 2018 German proposal asked 70/40 km/h dry/wet. So "20 km/h at 80 km/h" is the old minimum; modern systems do much more.

**From general knowledge [mem], verify before quoting:**
- Operating range: from ~15 km/h up to the vehicle max; the warning/braking must not trigger in the false-reaction tests (e.g. adjacent lane vehicle, overhead objects, passing a stationary vehicle in a curve).
- Emergency braking phase deceleration for heavy trucks: roughly 4-6 m/s2 achievable (limited by loaded mass and road); the regulation measures speed reduction, not a fixed m/s2.
- Driver override: the system may be interrupted/ended by a deliberate driver action (accelerator kick-down or a strong pedal move, steering input beyond a threshold, or the brake
  pedal when it demands more than the system). Manufacturers may let the driver switch it off but it re-arms on next ignition.
- Typical car-style TTC staging in the literature: warning ~2.5-3 s, partial brake ~3-4 m/s2 at ~1.8-2.2 s, full brake >=6 m/s2 at ~1.0-1.2 s (trucks use the lower end of decel).

**Mercedes ABA 5 [src]:** 3 stages: visual + acoustic warning; **partial braking ~3 m/s2 (~50 % of max)** if the driver does not react; then full-stop braking if the threat continues.
Stationary obstacles in own lane up to 90 km/h; full braking for moving pedestrians up to 50 km/h; radar + camera. Notes it cannot make up for an insufficient following distance.
Volvo/Scania: same pattern (warning -> partial -> full), warnings via display + sound + brake jerk; No figures were found [not verified].

## 4. Design for DazDrive

### 4.1 Inputs (all available per frame)
`gap` (m, radar), `v_lead` (m/s absolute), `v_own` (m/s), `air_psi`, route: junction/roundabout distance ahead, own pedal/brake/steer, ACC/cruise state, rain/wet (the wiper-based wet flag), mass estimate (TrueShift).
Derived: closing speed `vc = v_own - v_lead` (> 0 closing), `TTC = gap / vc`, and the braking-distance based margin:
`a_need = vc^2 / (2*(gap - d0))` with `d0 = 3 m` standstill margin. `a_need` is the best single number (it is what the driver physically has to do), TTC is the human-readable one.
Filter `v_lead` (the derivative-based hook value lags; use a 0.3 s low-pass, and reject frames where gap jumps > 5 m/frame = new target).

### 4.2 Stages (use max of TTC rule and a_need rule; thresholds are start values to tune in sim, scaled by `grip` 1.0 dry / 0.7 wet)

| Stage | Entry | Action | Exit (hysteresis) |
|---|---|---|---|
| 0 idle | - | nothing | - |
| 1 warning | TTC < 3.0 s or a_need > 1.5 m/s2, and v_own > 20 km/h | beep + hazard? no: beep + HUD/log; **no braking**; pre-fill: ACC/cruise throttle cut | TTC > 3.6 s and a_need < 1.2 for 1 s |
| 2 partial | TTC < 2.2 s or a_need > 2.5 m/s2 | brake request ramps to the level for decel ~3 m/s2 (request ~0.35-0.5 of full, ramp 1.5 m/s2/s); engine brake / retarder first (game does this on `+0x17c`) | TTC > 2.8 s and a_need < 2.0 for 1 s |
| 3 full | TTC < 1.2 s or a_need > 4.0 m/s2 (tune to the truck: brake_max measured from logs) | request ramp to a_need * 1.15 capped by brake_max, hazard lights on at the first frame, hold until v_own < 5 km/h or threat gone | threat gone for 1.5 s, then ramp down 2 m/s2/s |

Rules:
- **Command from a_need, not from the stage.** Stage only sets a floor/cap: partial caps decel at ~3.5 m/s2, full is uncapped. This gives the smooth ramp the stock AEB lacks and uses the minimum needed.
- Stages only go up fast, down slowly (exit thresholds above + dwell 1 s; never drop two stages at once).
- Warning at 3.0 s corresponds to the R131 1.4 s/0.8 s pre-warning with margin, since a game truck has no pedal/driver delay but the player does.
- Goal for a 80 km/h approach on a stopped queue: warning ~65 m, partial ~49 m, full ~27 m. Check that full braking at 27 m can still stop: v^2/(2a) = 247/(2*6) = 20 m ok with a=6; with a=4.5 it needs 27 m, so keep full entry at max(1.2 s, distance for a = 0.75*brake_max).

### 4.3 Crossing traffic and roundabouts
The radar only sees a lead that is in front, so a crossing car shows as a lead whose `v_lead` (absolute) is high while the gap closes. Cases:
- Real stationary/slow lead: `v_lead` ~ 0..low, steady gap closing = vc ~ v_own. Brake normally.
- Crossing/entering car: `v_lead` large (it drives at 5-15 m/s) but on a crossing heading the radar gap drops quickly and then the car leaves. Signature: lead speed
  absolute is **>= 3 m/s while the gap closes faster than (v_own - v_lead) predicts**, or the target appears with a short life (< 1 s old) at gap < 40 m.
- Rules:
  1. **Target age:** a new target (gap jump or first seen) must be stable for >= 0.4 s (>= 4 frames at 10 Hz sample, more at frame rate) before it counts for stage 2/3; stage 1 only.
  2. **Consistency check:** `d(gap)/dt` measured over 0.5 s must be within 1.5 m/s of `-(v_own - v_lead)`. If not (crossing / lane change geometry) cap at stage 1 unless gap < 8 m.
  3. **Junction context:** the route knows junctions/roundabouts ahead. Within ~60 m before and inside a roundabout/junction, a lead with `v_lead > 2 m/s` (it is moving, it will clear) is capped at stage 1;
     stage 2 only if TTC < 1.0 s; stage 3 only if gap < 5 m and still closing. A lead with `v_lead < 1 m/s` (really stopped) is not capped.
  4. **Steering/curve:** in a curve with radius < 200 m (we know the route) require the lead to be within the lateral corridor; else cap at stage 1 (guardrail/oncoming problem).
  5. Never go to stage 3 when `v_own < 20 km/h` unless gap < 4 m (kruip/queue behavior belongs to Auto Hold/ACC).
- Decision rule of thumb: moving leads give a lower stage unless very close; stationary ones always get the full ladder. This is the main false-positive filter, log it.

### 4.4 Driver override
- **Throttle** > 80 % (kick-down style) or > 50 % held 0.5 s: cancels stages 2 and 3 immediately (ramp down 4 m/s2/s), stage 1 stays as warning. Stock and MonoCruise both use gas as the cancel; R131 allows the same.
- **Brake pedal:** driver brake adds to ours; we do not cancel (MonoCruise does the same). If the driver brakes harder than a_need, we stand down.
- **Steering:** a strong steering input (> 25 % wheel after 0.3 s, evasive) cancels stage 3 to stage 2 (lets the player swerve), re-arm after 1.5 s.
- After an override: lockout 3 s for same target unless gap < 5 m. Ini: `[aeb] enabled`, `aeb_default_off` on first release (like MonoCruise: off unless chosen), stages scale `aeb_sensitivity` 0.8..1.2.
- Disabled when: reversing, `v_own < 5 km/h`, radar data missing/inconsistent (same checks as ACC), electric/handbrake states, the player switched DazDrive off.

### 4.5 Air-pressure awareness
Brake air is a budget: logged stock AEB drains 100+ psi in a few seconds.
- Read `air_psi` every frame. Our brake request goes through the game's mapping, so we cannot change the psi per bar of request, but we can limit the request and prefer retarder/engine brake.
- Energy plan: stage 2 decel (3 m/s2) comes mostly from engine brake + retarder (zero or little air); ask for service brake only for the remainder.
- If `air_psi < 90`: cap request at 0.6 outside stage 3; `< 75` (game's low-air line): go straight to the lowest necessary request, **never pulse** (no ABS-like on/off), hold steady.
- Prefer one smooth ramp over a short hard pulse: air use scales with applications, not only force; avoid repeated releases (hysteresis above). Log the psi drop per event to calibrate.
- Pre-brake: when stage 1 starts, ask for 0.05-0.1 request (lets engine brake/retarder engage) if `v_own > 50 km/h`. Optional, tune in the sim.

### 4.6 Interaction with the game's AEB
- The player can switch the game's emergency brake off (cvar `g_emergency_brake`, also "emergency brake detection: vehicle only" option). Preferred flow: setup page tells the player to disable it, plus
  option `[aeb] disable_game_aeb = 1` that sets the cvar to 0 on start and restores it on unload (needs verification that the SDK/our cvar write path works for this cvar; we do it for `g_acc`).
- If the game's AEB stays on, ours should act first: warning at TTC 3 s and partial from 2.2 s means the game's late full brake rarely triggers; when `ga+0x17c` is above our request
  and our stage is >= 2, treat the game as in control: do not fight it (we lowered request is ignored anyway), log `game_aeb_active` with the air psi so we see how often it still wins.
- Where our request is merged with the game's (max), expect the game to win on crossing cars (it ignores our context); that is the reason to turn it off.

### 4.7 What to log (extend `cruise.csv` or a separate `aeb.csv`, 10 Hz while stage > 0, plus one summary line per event)
`t, kmh, v_lead, gap, vc, ttc, a_need, stage, req_ours, req_ga, decel_meas, air_psi, thr, brake_pedal, steer, junction_dist, target_age, gap_slope_err, cap_reason, override`.
Event summary: start/end time, max stage, min gap, min TTC, speed at start/end, psi drop, why it ended (cleared / override / stopped), false-positive tag from the player (key press: "AEB was wrong").
Add a hotkey that tags the last event; tuning in sim from these.

### 4.8 Sim test scenarios (`tools/sim`, add `sim_aeb.cpp`; pass criteria in brackets)
1. **Stopped queue at 80 km/h:** lead v=0 from 200 m. [warn at ~65 m, partial, ends stopped >= 3 m behind, peak decel <= brake_max, psi drop < 40, no stage 3 unless needed].
2. **Slow truck 40 km/h at 80 km/h:** lead 11 m/s. [stage 1-2 only, reaches same speed with gap >= 10 m, never full].
3. **Lead brakes hard (cut-in from 20 m at 60 km/h own, lead 30 km/h):** sudden target. [stage 2 after target age 0.4 s, stage 3 if a_need > 4, no collision].
4. **Roundabout crossing car:** own 40 km/h approaching a roundabout, car crosses at 10 m/s from the side at 25 m. [max stage 1, no brake request, stops only if gap < 5 m closing].
5. **Driver override:** scenario 1 with throttle 90 % at stage 2; then brake pedal harder than request. [request ramps to 0 on gas; brake pedal wins on brake; 3 s lockout].
6. **Low air:** scenario 1 with air_psi 80 and 70. [request capped, no oscillation, still stops or minimizes speed; log shows cap_reason=air].
7. (extra) **Curve ghost:** guardrail/oncoming vehicle seen at 30 m in a tight curve. [stage 1 only].

## 5. Open points
- Read MonoCruise `core/` for the actual filters (public MIT; only ideas): how it excludes lane-change vehicles and its path prediction.
- Verify R131 override and false-reaction wording in the UNECE text (unece.org R131 PDFs were 403 for the fetcher; try the lexaris or EUR-Lex copy).
- Measure `brake_max` and request-to-decel curve for loaded/unloaded trucks from existing logs (`cruise.csv` req_ga vs measured decel).
- Check the cvar write for `g_emergency_brake` and the exact option name for "detection: vehicle only" in 1.60 config.
