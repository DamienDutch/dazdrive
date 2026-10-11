# Leaving neutral at speed, the hard stop with hazards, and the ACC distance key (ETS2 1.60.1.7, static RE)

Exe `eurotrucks2.exe`, base 0x140000000. Static disassembly only, nothing run. [V] = seen in code, [I] = inference. Names as in
`lka-roll-re.md` / `clutch-mode.md` / `cruise-re.md`: `self` = automatic gearbox (rcx of 0x14098c640), `inner` = `self+0x10` = vehicle,
`ga` = game_actor `[[0x143550398]+0x31a8]`, `ga+0x18 == inner`. Evidence: logs of two test drives (`trueshift.csv`, `dazdrive.log`).
All patterns below were checked unique in .text.

## Summary

1. Neutral -> gear at speed is decided in the **generic gearbox gap state** (`0x140601a10`, `self+0x18 == 1`), not in the automatic. With gear 0, target != 0 and an
   automatic box (A mode) it waits **until `inner+0x1424` (merged throttle, written by the throttle fn 0x1405c8260) is > 0**. Nothing else releases it.
   The `0x1405b98a5` feed only changes the automatic's *argument*, never `inner+0x1424`. Our own cruise stub returns plain `pedal` while rolling, so `inner+0x1424` stays 0.
2. Recommended: engage ourselves from the tail hook (call the box's own engage step, 3 lines), fallback: return a small throttle from the cruise stub while in gap state.
3. The hard stop + hazards + dead pedal fit one mechanism: **`ga+0x17c` (brake request) > 0**. It brakes (not via the SDK brake channel), switches the hazards on
   (auto-hazard rule), forces `inner+0x1424 = 0`. The game recomputes it every tick *only if `g_emergency_brake != 0`*; **our cruise module also writes it
   (curve brake 0.3, hill start 0.6) and never clears it**. Prime suspect: latched by us. Kept cruise flag is not the cause.
4. Accmode key: **our `min_kmh` patch changes the shared `xmm7` = 30.0 that the accmode block also uses as its first threshold** (becomes 5.0). That reproduces all three observations exactly.

## 1. How the automatic leaves neutral at speed

### 1.1 Data flow per tick [V]

```
throttle fn 0x1405c8260(ga, dt, analog, digital) -> 0x140991c90 (cruise PI; our stub_cruise sits at its call 0x1405c8550)
    stores inner+0x1424 = result   ONLY IF  (def(inner+0x1f8)+0x1b0 + inner+0x10f0) > 0  AND  ga+0x150 == 2  AND  ga+0x17c <= 0   (0x1405c8575..0x1405c8596)
    else inner+0x1424 = 0
ga+0x1cc0 (driver throttle for the box) = analog + ga+0x1ce4 (0x140993128), ga+0x1cc4 = brake     (NOT the cruise throttle)
automatic update 0x14098c640(self, xmm1 = ga+0x1cc0, xmm2 = ga+0x1cc4)   (vtbl slot +0x60 of 0x1421b96f0; call 0x1405b98e4)
    ... ends with  jmp 0x140601a10   (0x14098cd48)    generic gearbox, reads self+0x68 (target) and inner+0x1424
```

### 1.2 What the automatic does with throttle > 0.01 (xmm1) [V]
`0x14098c716..0x14098c75f`: drive state `self+0x110` in {2,3} (or 4 with gear 0) and xmm1 > 0.01 -> `self+0x68 = max(0x14098dbf0, 0x14098deb0)`, `self+0x110 = 4`.
That only **sets the target** (what stub_start returns). Same with the 0x1405b98a5 feed. Our tail hook runs after it and overwrites `self+0x68` anyway.
Without throttle, in state 3 nothing touches `self+0x68` (`0x14098c78f..0x14098c7ba` only handle reverse/latch), so a target written by us stays.

### 1.3 The gate: generic gearbox `0x140601a10` [V]
`self+0x18` state: 0 idle -> 2 opening -> 1 gap -> 3 closing -> 0. State 0 (`0x140601c99`): `self+0x68 != gear` -> state 2. State 2 with clutch already 1.0 -> state 1
(`0x140601bbc..bd3`, timer `self+0x38`). **State 1** (`0x140601be3..c97`), `ecx` = current gear, `edx` = target, `rax` = tx def, `r8` = inner:

```
timer -= dt
if gear != 0 && target != 0 && gear != target && timer > 0:  wait                  (c1a..c29)
c2f:  if tx+0x1b8 > 0                      -> ENGAGE     (def float, 0 on the test truck: observed)
      if gear != 0                         -> ENGAGE
      if target == 0                       -> ENGAGE
      if inner+0x1424 > 0                  -> ENGAGE     (c40: comiss xmm0(0), [r8+0x1424]; jb c7c)
      if vfunc+0x78(self) == 2 && abs(speed*3.6) >= 2.0 -> ENGAGE        (c4a..c7a)  -- for the automatic vfunc+0x78 = 0x14098d880 returns 0 in A mode (self+0x100 != 0), 3 otherwise: never 2
      else WAIT (return, state stays 1)                                                    (cb3)
c7c ENGAGE: vfunc+0x70(self, target) = 0x1406019c0 (stores gear in inner+0x1448 if within range), self+0x18 = 3, byte self+0x2c = 1
```
So with gear 0 at speed, a target in `self+0x68` and clutch 1.0 gives exactly the observed picture: state 1, **parked until `inner+0x1424 > 0`**. [V code; the state itself was not logged: log `self+0x18`.]
`inner+0x1424` is positive when the driver presses (pedal -> 0x1405c8260) or when cruise asks for throttle (PI output). It is 0 when cruise asks for nothing (set speed below speed), when our stub
returns `pedal` (see below), or when one of the three store gates (1.1) is closed.

### 1.4 Why the earlier attempts failed
- Target written every update: needed, not sufficient (gate above).
- `xmm1 = max(xmm1, 0.05)` at 0x1405b98a5: reaches only 0x14098c640's argument (1.2). `inner+0x1424` is not touched. [V]
- `stub_cruise` (cruise.cpp): `if (g_rolling) { C.u = 0; ... return pedal; }` and `g_rolling = S.rolling || S.roll_exit > 0` (trueshift.cpp:1132). During the exit the cruise
  throttle is deliberately 0, so even with cruise on (set 60, speed 42 in the second test drive) `inner+0x1424` is 0. The cruise PI would otherwise have engaged the box by itself. [V]
- SDK `thr_eff` = `inner+0x1428` is no evidence about `+0x1424`: the clutch integrator `0x140622900` forces `+0x1428 = 0` in gap state with gear 0 or target 0
  (`0x140622c0c..c16 -> 0x140622d8f`, pattern `C7 86 28 14 00 00 00 00 00 00 0F 57 F6`). Log `inner+0x1424` directly.
- After the hard stop (second test drive, 250-268 s) pedal 0.2..1.0 did not engage either with cruise off -> the store gate was closed (section 3).

## 2. Minimal, safe way to engage a chosen gear from neutral without the pedal

**Recommended (b): engage ourselves from the tail hook.** In `tail_pre` (hook at 0x14098cd48), when `gb_state(self) == 1 && gear == 0 && self+0x68 == pick > 0`
(the game would sit there), do what 0x140601c7c does:
```
((void(**)(void*,int))*(void***)self)[0x70/8](self, pick);   // 0x1406019c0: inner+0x1448 = pick if in range
*(int*)(self+0x18) = 3;  *(unsigned char*)(self+0x2c) = 1;   // closing; clutch then closes at the rate of 0x140601cc0 (rpm-match scaled)
```
Pattern for the gap-state branch (unique, `0x140601c2f`): `0F 2F 80 B8 01 00 00 72 ?? 85 C9 75 ?? 85 D2 74 ?? 41 0F 2F 80 24 14 00 00 72 ?? 48 8B 03 48 8B CB FF 50 78`
(the `call [rax+0x78]` is at +0x22; engage block `mov rax,[rbx]; mov rcx,rbx; mov edx,[rbx+0x68]; call [rax+0x70]` at `0x140601c7c`).
- Needs no throttle and is independent of the store gates (17c etc.), replicates the game's own engage 1:1 [V], so no half-states. State 2/0 need no help (they pass by themselves in one or two ticks).
- Revving: none. Engine stays at idle until the clutch closes, the box drags it up exactly as in the stock throttle case.
- Undo: one shot, no persistent state. Stop calling it.

**Fallback (a): feed the merged throttle** from `stub_cruise` (the value it returns is what 0x1405c858c stores): while `gb_state == 1 && gear == 0 && target > 0 && roll exit`, return `max(value, 0.03)`.
Must be placed before the early returns (cruise off). Needs all three store gates open. Harmless to the engine: the integrator forces `+0x1428 = 0` in that state. Drop it the tick `gear != 0`
(else 3 % throttle goes to the engine and seeds the cruise PI, which integrates on `inner+0x1424`).

**Not recommended (c): input-mix throttle.** The pedal fn 0x140992d40 reads the input objects at `0x140992f2e..0x140992fc1`: `[ga+0x1eb18]+0x178` (aforward, analog, ends in `xmm11` = `xmm2` of 0x1405c8260, goes straight into the max with the cruise value),
`[ga+0x1eaf8]+0x178` (dforward, digital, ramped by timers `ga+0x1ce0`), `[ga+0x1eb28]/[ga+0x1eb08]` (reverse). Writing `value` (+0x178) must happen before that read in the same tick (the SDK frame callback order vs the tick is unknown).
It also feeds `ga+0x1cc0`, the store gates still apply, and the engine sees the pedal after engaging. Only (b)/(a) are cleaner.

## 3. Hard stop, hazards, dead pedal

### 3.1 What each symptom maps to [V code]
| symptom | mechanism |
|---|---|
| strong braking, SDK brake 0 | pedal fn `0x140993509`: brake request `ga+0x17c` > 0 -> brake slewed +-5/s (consts `0x1423d8418` / `0x1423d8f30`) to it, applied through `[inner vtbl+0x178]` (per-wheel brake array read by 0x14083db10). The SDK `effective.brake` getter (0x14058dc30) reads other fields, so it stays 0. Decel in the second test drive plateaus at ~5.9 m/s2 within ~1.5 s. |
| hazards on by themselves, toggling does nothing | key handler 0x14099b660 (runs every tick before the pedal fn): at `0x14099d5c0` if cvar `g_emergency_brake_mode` (0x142bf56b0, name at 0x142115308, only read here and in the UI 0x140407512) != 0 and hazard (`ga+0x1d0` bit 4 = SDK `truck.hazard.warning`, getter 0x1404ba990) is off and **`ga+0x17c > 0`** -> it presses the hazard (toggles bit 4, sound 0xc, flasher `ga+0x340`). Switch them off by hand and the same check switches them on again next tick. Pattern `85 C0 74 17 F6 87 D0 01 00 00 10 75 0E 44 0F 2F 8F 7C 01 00 00 73 04 B2 01` (0x14099d5cc). |
| pedal 1.0 but no throttle, box stays in neutral | 1.1: `ga+0x17c > 0` zeroes `inner+0x1424` (`comiss xmm7,[rbx+0x17c]; jb`, pattern `0F 2F CF 76 ?? 83 BB 50 01 00 00 02 75 ?? 0F 2F BB 7C 01 00 00 72 0A F3 0F 11 81 24 14`, 0x1405c8575) -> gap state never sees throttle (1.3). Start-gear fn was called and returned 1 (pedal reaches the automatic) but state 1 only exits on `inner+0x1424`. |

So the three symptoms together are the signature of **`ga+0x17c` stuck > 0**. `ga+0x150 != 2` and the energy gate (`def+0x1b0 + inner+0x10f0 <= 0`, meaning not identified, probably fuel/engine health) are the two other closers; hazards point at 17c.

### 3.2 Who writes `ga+0x17c`
Game: `0x1409958dd` (0, engine off: ADAS update returns early with `ga+0x154 == 0`), `0x140995e01` (0 every tick, but only inside the block entered when `g_emergency_brake != 0`, `0x140995c62..c70`),
`0x140995e52` and `0x140995ee5` (1.0, forward-collision brake: obstacle inside braking distance for > 0.1 s with `g_emergency_brake == 2`, or closing speed on the radar lead with 1/2).
**Us**: `cruise.cpp` `hill_start` (`*req = max(*req, 0.6)`) and the curve brake (`if (b > *req) *req = b`, b <= 0.3) - both only ever raise it.
- If `g_emergency_brake != 0` the game zeroes it every tick *before* the pedal fn, our raise lives one tick: fine, and the hazard rule (which runs between ADAS and our write) never sees it.
- If `g_emergency_brake == 0` (AEB option off) nothing in the game ever clears it: **our write latches**: brake stays on, throttle dead, hazards forced on, box parked in neutral. This reproduces everything seen in the second test drive
  from 244.1 s (brake ramp, hazards, 250-268 s pedal ignored, cruise flag irrelevant). The route look-ahead started at 242.16 s and the descent/curve code fired at 244.02 s [log]. Request 0.3 giving ~5.9 m/s2 on a 13 t truck is plausible (ETS2 brakes are strong) but not proven.
- Alternative: a genuine game AEB (request 1.0, e.g. radar lead closing) at 244.1 s. It would clear itself at standstill when AEB is on (v=0 fails every condition at 0x140995e22 / 0x140995e7c), so it cannot explain the dead pedal 5 s later. [I]

### 3.3 Release
Stock: nothing to release, the request is recomputed to 0 each tick. A latched value is released by switching the engine off (ADAS clears at 0x1409958dd) or by writing 0.
**Skipped cancel stores are not a cause**: the 17c logic is gated by `g_emergency_brake` only (0x140995c62), not by `ga+0x199`. They can only indirectly add reasons to brake (ACC emergency part 0x1409937ea
brakes via the same vtbl+0x178 when `0 < gap +0x1a8 < 0.5*D(v)`, for a cruise that cannot accelerate in neutral), but that part does not touch 17c and does not switch the hazards.

### 3.4 Plugin fix and the discriminating log
- Never leave 17c raised: remember what we wrote (`ours`), and on the next tick set it back to 0 unless we still want it (write only inside the cruise stub, `*req = want` where `want` is 0 when no longer needed;
  do not use `max` against a stale value). Also write 0 on module stop and when speed < ~1 km/h.
- Add to the trace per tick: `*(float*)(ga+0x17c)`, `ga+0x114` (bit 0x10 collision warning), `ga+0x1a8`, `ga+0x1d0` bit 4 (hazard), `ga+0x150`, `ga+0x154`, `inner+0x1424`, `self+0x18`, `self+0x110`,
  the cvars `g_emergency_brake` (0x142bf5570) and `g_emergency_brake_mode`, and the two energy-gate floats. One run decides AEB vs latch.

## 4. ACC distance key: only the first press works

### 4.1 Handler [V] (`0x14099b9e3`, inside 0x14099b660, runs once per tick)
Event: action object `[ga+0x1e750]`, `+0xf8 == 2` (pressed) -> `0x1401ed460` consumes it (-> 1/0). Then, with `xmm7` = a float loaded **once at the handler head** and `xmm6 = 20.0` (`0x14099b9cb`):
```
g = float(g_acc)                       (0x1401d6ad0, follows the +0x120 chain)
g > xmm7  -> 30        (message "normal")      stock xmm7 = 30.0
g > 20    -> 20        ("close")
g > 0     -> 0         ("disabled")
else      -> 40        ("far")
snprintf("%g") ; [g_acc vtbl+8](g_acc, text)           (0x14099ba6b)
```
No internal index, no other state: the next value is a pure function of the current `g_acc`. The setter (`0x14030def0`) fails only for a read-only/deleted node or when its validator callback `[cvar+0x98]` returns false (not the case here).

### 4.2 Root cause [V code, matches the logs]
`xmm7` is the register loaded at **`0x14099b871 movss xmm7,[0x1423d8640]` (30.0)**. cruise.cpp patches exactly that load (`game_patch_rip_float(head, 4, 8, p_min_kmh)`, min_kmh = 5) so that dec (`comiss xmm1,xmm7` at 0x14099b920) and resume
(`0x14099b97a`, `0x14099b998`) accept low speeds. **The accmode compare `0x14099b9ef comiss xmm0,xmm7` uses the same register**, so its first threshold became 5.
Reachable transitions now: `g>5 -> 30`, `0<g<=5 -> 0`, `g<=0 -> 40` (the `>20` branch is dead). Check against the log: 45 -> 30 [ok], 20 -> 30 [ok, stock would give 0], 30 -> 30 (no change, "later presses do nothing") [ok].
Our own non-round `g_acc` writes (+0.1) are irrelevant to this.

### 4.3 Fix
1. Give the accmode compare its own 30.0: patch `0x14099b9ef` (5 bytes `0F 2F C7 76 16`, unique via `0F 2F C7 76 ?? 48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? F2 0F 10 1D`, whole-block anchor `48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C7 76 ?? 48 8D 0D ?? ?? ?? ?? E8` at 0x14099b9e3)
   with `E9 rel32` to a stub: `comiss xmm0,[30.0 (the pooled 0x1423d8640 read only, or an own copy)]; jbe 0x14099ba0a; jmp 0x14099b9f4` (jbe target = `0x14099b9f4 + disp`, read the disp8 at +4 and compute). xmm7 is not read again until 0x14099bf3e (reloaded), so alternatively set `xmm7 = 30.0` there.
2. Keep the player's value in sync: the game cycles from whatever `g_acc` holds when the key is pressed. While cruise runs we hold a non-round computed value (e.g. 27.6), so the key moves relative to that (27.6 -> 20) instead of the player's level.
   Patch the **call** `0x14099b9ea call 0x1401d6ad0` (E8, `game_patch_call`) with a wrapper: if cvar == g_acc and `g_acc_ours` is set, first `game_cvar_set(g_acc, g_acc_player)` and clear `g_acc_ours`, then return the original read. The press then steps from the player's last round value and
   the existing poll in `cr_frame` (round value we did not write = player pick) sees the result.
3. Detecting presses without polling is possible from that same wrapper (it runs only on a press, `cl == 1`).

## Patch table (all unique)
| what | VA | bytes |
|---|---|---|
| generic gearbox gap-state exit (throttle compare) | 0x140601c2f | `0F 2F 80 B8 01 00 00 72 ?? 85 C9 75 ?? 85 D2 74 ?? 41 0F 2F 80 24 14 00 00 72 ?? 48 8B 03 48 8B CB FF 50 78` |
| throttle store gates (17c) | 0x1405c8575 | `0F 2F CF 76 ?? 83 BB 50 01 00 00 02 75 ?? 0F 2F BB 7C 01 00 00 72 0A F3 0F 11 81 24 14` |
| auto-hazard from 17c | 0x14099d5cc | `85 C0 74 17 F6 87 D0 01 00 00 10 75 0E 44 0F 2F 8F 7C 01 00 00 73 04 B2 01` |
| effective throttle forced 0 in gap state | 0x140622d8f | `C7 86 28 14 00 00 00 00 00 00 0F 57 F6` |
| accmode compare / call | 0x14099b9ef / 0x14099b9ea | see 4.3 |
| AEB 17c writes | 0x140995e52 / 0x140995ee5 | `C7 83 7C 01 00 00 00 00 80 3F` (both, not unique alone) |
| cvars | `g_emergency_brake` 0x142bf5570, `g_emergency_brake_mode` 0x142bf56b0 | name-at-+8 via init fns 0x14003a04c for the latter |

## Open
- `self+0x18` state and `inner+0x1424` were never logged in those test drives: the gap-state diagnosis is code-derived (and fits every observation), not observed.
- Meaning of the energy gate floats (`def+0x1b0`, `inner+0x10f0`) and of `tx_def+0x1b8`.
- Whether the pedal fn runs before or after the automatic update inside one tick (fallback (a) is insensitive to it, (b) is not affected).
- Which exact event raised 17c at 244.1 s (our curve brake vs the game's AEB): needs the log of 3.4.
