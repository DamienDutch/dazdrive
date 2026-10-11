# Cruise keys: tap/hold steps and retarder off (ETS2 1.60.1.7, static RE, nothing run in game)

[V] = seen in disassembly, [I] = inference. Image base 0x140000000. Code: `plugin/dazdrive/modules/cruise.cpp` ("cruise keys"), sim: `tools/sim/sim_cruise.cpp` `keys_test`.

## Findings
- Key handler `0x14099b660` runs once per tick (before the pedal fn). It reads one **action object** per key from game_actor: toggle `+0x1e710`, **inc `+0x1e720`**, **dec `+0x1e730`**, resume `+0x1e740`, accmode `+0x1e750` [V].
  Pattern per key: `cmp [rcx+0xf8],2; jne; call 0x1401ed460; mov cl,1` (return value of the call is ignored).
- Action state `+0xf8`: 2 = fresh event, set by the generic updater `0x1401ed4d0` -> `0x1401ed660` [V]. `0x1401ed460` (consume) re-checks the input value (`[action+0xf0]+0x178` within `+0x40..+0x48`)
  and writes `+0xf8` = 1 (still down) or 0 [V]. If action `+0x50 == 1` the updater does not refire while down (state stays 1); for other values consume writes 0 so the event **refires every tick** (`0x1401ed5a0`) [V].
  Which type cruise inc/dec have is not decidable statically [I]: the module works for both (see below). No repeat timer exists in the handler [V].
- inc `0x14099b74d` / dec `0x14099b89a`: step = `g_cruise_control_grid` cvar (UI 1/2/5; the test profile uses 2), formula in `cruise-re.md` § 4. No tap/hold distinction in the game [V].
- **Retarder:** manual retarder level = `game_actor+0x180` (u32, stage 0..`tx_def+0x1c4`), changed by the retarder up/down keys `0x14099c1f3` / `0x14099c255` through the setter
  **`0x1405c74c0(game_actor, level, snap)`** (writes `+0x180`, fraction `+0x184`, `+0x174`, `+0x178`, target `+0x370`, with snap=1 also `+0x36c`) [V].
  **The activate fn `0x14099b5c0` (toggle/inc/dec when off) refuses to switch on while `+0x180 > 0`** (calls `0x1406669a0`, a message, returns) [V]. Resume `0x14099b971` does not check it, but the cancel hub `0x140993321` cancels on `+0x180 > 0` next tick [V].
  So the level must be 0 before the handler looks at `+0x199`; a plain store to `+0x180` would skip the fraction/target fields, so we call the game's setter.

## Design (what is built)
- Hook: the four `call 0x1401ed460` of toggle/inc/dec/resume are re-pointed to `consume_stub(action)` (`game_patch_call`; stub calls the original, then `key_event`). Targets of all four verified equal; each site byte checked `E8`.
  Sites (all patterns unique in .text, scanned):
  | key | VA of E8 | pattern |
  |---|---|---|
  | toggle | 0x14099b6d4 | `48 85 C9 74 12 83 B9 F8 00 00 00 02 75 09 E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 1A 80 BF 99 01 00 00 00 75 0A` (hit 0x14099b6c6, E8 at +14) |
  | inc | 0x14099b712 | `48 8B 8F 20 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01` (hit 0x14099b6fd, E8 at +21) |
  | dec | 0x14099b866 | same with `30 E7` (hit 0x14099b851) |
  | resume | 0x14099b950 | same with `40 E7` (hit 0x14099b93b) |
  | retarder setter | 0x1405c74c0 | `48 89 5C 24 08 48 89 74 24 18 89 54 24 10 57 48 83 EC 50 48 8B 41 18 48 8B F9 41 0F B6 F0 8B DA` |
  The action is matched against `[ga+0x1e7x0]`, so only the player's truck counts.
- **Retarder off** (`retarder_off_on_set`, 1): on a press of toggle (only when cruise is off), inc, dec or resume, if `+0x180 > 0` -> `0x1405c74c0(ga, 0, 1)` inside the stub, i.e. before the handler's activate/enable. Logged ("pressed with the retarder on").
  Residual risk: if the lever were an analog axis rewriting `+0x180` each tick it would cancel again; the keys-based setter suggests stages [I].
- **Steps** (`step_tap_kmh` 5, `step_hold_kmh` 1, `hold_s` 0.4, `hold_rate` 8/s; `step_tap_kmh` 0 = game's own steps): the game still does its grid step on the event; `keys_tick` (called from `stub_cruise` before the controller reads the set speed, and from the frame) puts `+0x19c` back to the value from before the event, then:
  - key released before `hold_s` (tap): ONE step on the 5 km/h grid with the game's own formula (inc `(floor((s+0.5)/5)+1)*5`, dec `floor((s-0.5)/5)*5`): 83 -> 85, 85 -> 90, dec 83 -> 80, 85 -> 80. **Rounding to multiples of 5 chosen** (matches `g_cruise_control_grid`=5). Applied on release (latency = press length).
  - held >= `hold_s`: from there a step of `step_hold_kmh` (same formula, grid 1) every 1/`hold_rate` s; no 5 jump at the start. Works whether the game refires the event each tick or not, because the module generates the repeat itself.
  - Release is detected as: no event for 0.08 s and action `+0xf8 == 0`.
  - A press that switches cruise on (was off) or a key still down from the moment cruise came on is not a step (`block` until release).
  - Limiter (`+0x198`/`+0x190`) caps inc as in the game; dec: `min_set_kmh` clamps (as the frame already does), without it below `min_kmh` cruise goes off like the game's dec.
  - ISA (speed-limit following) writes `+0x19c` in the frame; it does not touch the key state, a key sequence uses the set speed from its start (tap) or the current one (hold).
  - Units: km/h assumed (mph unit factor `0x142589dec` not handled).

## Checks
- Tap + / - : exactly one 5 km/h step to the next multiple, appears when you let go. Hold: nothing for ~0.4 s, then 1 km/h at 8/s (log `Cruise: hold + from x to y`). Tune `hold_rate`.
- Retarder lever on (stage 1-3), then set/resume/on-off: retarder drops to 0, cruise engages; log line "retarder off". Check that the dashboard retarder indicator and sound follow.
- Switch cruise on with + : no extra step. Dec far down with min_set_kmh: stops at min_set_kmh.

## Unverified
- Points to confirm in the game: the +0xf8 semantics, the ordering "handler -> keys_tick -> controller" within a tick (a 1-tick glitch of one step on the set speed is possible if the controller runs between), the retarder setter's side effects (sound/HUD), mph.
