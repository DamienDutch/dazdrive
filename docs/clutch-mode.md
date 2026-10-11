# Clutch and adaptive mode (ETS2 1.60.1.7, static RE)

Exe `eurotrucks2.exe`, image base 0x140000000. Static disassembly only, nothing run. [V] = seen in disassembly, [I] = inference.
"inner" = vehicle object (`gearbox+0x10`), "drive" = `inner+0x150`, "engine" = `drive+0x360`, "tx def" = `resolver(inner+0xfe8)` (resolver `0x1408c4550`).

## 1. Clutch

### Where the value lives [V]
- SDK channel `truck.effective.clutch` getter = `0x14058dc90`: `movss xmm0,[rcx+0x142c]; ret`. So **`inner+0x142c` (float) = effective clutch**.
  Neighbors: `truck.effective.throttle` = `inner+0x1428` (`0x14058dc70`), `truck.effective.steering` = `+0x640`.
  Channel table is built in `0x14058ea50` (registration pairs name string / getter).
- Scale: 0 = engaged, 1 = fully open. Evidence: shift opening sets target 1.0 (gearbox state 2); drivetrain `0x1408c8270` treats clutch > 0.9 (`0x1423d7cbc`) with a gear engaged as decoupled (free-revving engine).
- Drive object mirror: `drive+0x330 = inner+0x142c`, copied each tick in the drivetrain update at `0x1408c82cf`
  (`8B 83 2C 14 00 00 89 86 30 03 00 00`, unique). Torque coupling uses `1 - drive+0x330` (`0x1408c89a2`). [V] copy, [I] coupling.

### Companion fields on inner [V]
| off | meaning |
|---|---|
| +0x142c | effective clutch (current) |
| +0x1430 | clutch **target** (0 = close, 1.0 = open) |
| +0x1434 | clutch **rate** (per second; multiplied by dt) |
| +0x1424 / +0x1428 | input / effective throttle |
| +0x1448 | current gear |

### Who writes +0x142c [V]
1. **Integrator `0x140622900`** (inner method, called every physics tick from `0x140620840` via `call 0x140622900` at `0x14062085f`). Prolog (unique):
   `48 8B C4 48 89 58 18 48 89 70 20 57 48 81 EC A0 00 00 00 48 8B 99 50 01 00 00`.
   Tail (the only live writers):
   - `0x140622ef4  F3 0F 11 8E 2C 14 00 00` plain path: `clutch += clamp(target-clutch, +-rate*dt)` (rate=`+0x1434` times dt). Unique.
   - `0x140622eac  F3 0F 11 86 2C 14 00 00` slip path (below). Unique.
   - dt = `cvtsi2sd [0x143518768]*[0x1423d7a00]` (global frame ticks).
2. State restore (save-load) only: `0x14098d8e1`, `0x1409aa6f1`, `0x1406025e1` (copy `+0x1430=+0x142c=saved`, `+0x1434=0`). Not per frame.
3. `0x140601eb0` (reset) zeroes `+0x142c/+0x1430`.

### Launch slip / closing rule (automatic) [V code, I meaning]
In `0x140622900` after the throttle part (`0x140622dff`): if gearbox vfunc `+0x88` is true (automatic active) AND `tx_def+0x1b8 (float) <= 0` AND `clutch > target` (closing) AND `drive+0x33c != 0`:
 - lower rpm bound of the current gear window `W.lo` = `[rsp+0xb0]` from `0x140602090(gearbox,&out,gear)` (engine def rpm ranges blended by `g_adaptive_shift`, see 2).
 - engine rpm = `drive+0x2e4 * 9.5493` (rad/s to rpm, `0x1423d84e4`).
 - if rpm > W.lo AND (`drive+0x2e4 - drive+0x2e8 >= 0`): `clutch = slew(clutch,target,rate*dt)` via `0x14045ccc0` (store at `0x140622eac`).
 - else: **clutch is left unchanged** (held slipping) = launch slip while the engine is below the gear's low rpm bound. [I]
Otherwise (opening, or `tx_def+0x1b8 > 0`) the plain path at `0x140622ef4`.

### Shift sequencer (sets target/rate) [V]
`generic gearbox update 0x140601a10`, `self+0x18` state:
- 0 idle: if `self+0x68 (target gear) != inner+0x1448` then `self+0x34=1; state=2`. Writes no clutch fields.
- 2 opening: each tick `+0x1434 = rate` (from tx def `+0x1c0`, default 100.0 `0x1423d8774`, doubled for 1<->2 and halved to 0.x for neutral), `+0x1430 = 1.0`. When clutch >= 1.0 (`0x1423d7d40`): `state=1`, timer `self+0x38` = `0x140601e20()`.
- 1 gap: `self+0x38 -= dt`; when <= 0: vfunc `+0x70` (`0x1406019c0`: stores gear to `inner+0x1448` if within `[-tx+0x228, tx+0x200]`), `state=3`.
- 3 closing: each tick `+0x1434 = f(0x140601cc0)`, `+0x1430 = 0`; when `clutch <= 0`: `state=0`; if target gear changed again: `state=2`.
- Closing rate `0x140601cc0` (returns xmm0): default `0x1423d7924` (0.2) / `5.0` / `3.0/tx+0x1c0`; for non-electric engines it is scaled by an rpm-match factor `(rpm - (idle+0x228...)) / (150 rpm)` (`0x1423d87e0`=150, `0x1423d8828`=200). [I] on exact meaning.
Because states 0 and 1 never write +0x1430/+0x1434, **in state 0 (gear engaged, idle) a plugin that writes `inner+0x1430`/`+0x1434` makes the integrator drive the clutch** to that target. [I, not tested]

### Stop behavior (automatic update `0x14098c640`) [V]
Args: xmm1 = throttle, xmm2 = brake (both compared to 0.01, `0x1423d76b4`). `self+0x100` = automatic active (A mode), `self+0x101` = at-rest/neutral latch, `self+0x110` drive state, `+0x114` saved.
- If A mode: `|inner+0x2f0| < 0.1` (`0x1423d786c`, speed m/s) AND throttle < 0.01 AND brake < 0.01: `self+0x68 = 0` (neutral), latch `+0x101 = 1`; drive state 4 -> 3, 0 -> 1. Neutral target makes the sequencer open the clutch (state 2).
- Pedal > 0.01 while latched: start gear = `0x14098dbf0` (start-gear fn) vs `0x14098deb0`, `max` of them -> `self+0x68`, drive state 4.
- Brake > 0.01 in drive state 0/1/2 with throttle < 0.01: `self+0x68 = -1`, state 0.
- Near-stop gear hold/drop: threshold math at `0x14098c8c3..0x14098c957` uses `(1 - inner+0x142c)` (the clutch) and `petrol`-engine special case. No explicit rpm-threshold constant found that opens the clutch; it is the neutral selection + sequencer. Open: the "stall protection" / engine-near-idle behavior you saw (clutch dragging to ~2 km/h) probably comes from the closing-rate / slip rule above (clutch held while rpm < W.lo).

### Can a plugin control the clutch?
- (a) hook a call site returning clutch: not applicable, no getter in the loop. The best single hook is the **drive copy** `0x1408c82cf` (`mov eax,[rbx+0x142c]`): replace eax before it is stored to `drive+0x330` (affects torque coupling only, leaves the sequencer state intact). Hook points in the integrator: `0x140622ef4`, `0x140622eac` (modify xmm1/xmm0 before the store).
- (b) write a field each frame: write `inner+0x1430` (target) and `+0x1434` (rate) in gearbox state 0, or `+0x142c` directly. Ordering vs the physics tick is unknown (SDK frame callback is outside the tick). A code patch at the stores is safer. [I]
- (c) input-mix route: **there is a "clutch" input entry**, but the automatic does not read a pedal clutch (its clutch is computed). Constructor of the input-reference block, function containing `0x1405b7549` (extent `0x1405b5330..0x1405b788b`), builds entries of 0x10 bytes (vtable `0x1420d4278`, `+8` = pointer from registry lookup `0x1425e6160 vtbl+0x48` by token) on object `rdi`:
  | obj off | token | name |
  |---|---|---|
  | +0x1eac0 | 0x1d60f5850b5 | steering |
  | +0x1eaf0 | 0x188ccfd7c42 | "dforward" (ets2la throttle pattern) |
  | +0x1eb00 | 0x3a4e6cb791ca | (dbackward) |
  | +0x1eb10 | 0x188ccfd7c3f | aforward |
  | +0x1eb20 | 0x3a4e6cb791c7 | abackward (ets2la brake pattern) |
  | +0x1eb30 | 0x56ba050d | **clutch** |
  ets2la patterns still hit: throttle `0x1405b7549`, brake `0x1405b7637` (unique). Clutch entry pattern (unique, `0x1405b7672`): `48 C7 45 80 0D 05 BA 56 48 89 87 20 EB 01 00` then `lea rdx,[rbp-0x80]` / `lea rcx,[rdi+0x1eb30]` (offset 0x1eb30 is the 4 bytes after `48 8D 8F` further on). Use as fallback; effect on the automatic is unproven. [I]
- The player throttle/pedal path: `0x1405c8260` writes `inner+0x1424` (`0x1405c858c`/`0x1405c8596`); it does not touch the clutch.

## 2. Adaptive mode (Eco / Normal / Power)

- Cvar object **`g_adaptive_shift` at `0x142bf4b70`** [V]: name copied into the object at `0x14003962c` (`lea r8,[0x142115398]` = "g_adaptive_shift" into `0x142bf4b78`). Reader: `float 0x1401d6ad0(cvar)` (follows override chain `+0x120`). Writer: virtual `[vtbl+8](cvar, "text")`; value is held as text (the game uses `snprintf "%g"` then this setter). Plugin: call the setter with "10", "3" or "1.6667" (or "0").
- Mode = this float only; no per-truck field. Game cycle handler in `0x14099b660` (key action state `[ctx+0x1ea40]+0xf8 == 2`), code at `0x14099ccc2` [V]:
  `v <= 0`: set **10.0** -> message "Auto transmision power mode set to 'Eco'";
  `0 < v <= 1.6667`: set 0 -> "...mode disabled";
  `1.6667 < v <= 3.0`: set **1.6667** -> "'Power'";
  `v > 3.0`: set **3.0** -> "'Normal'".
  Cycle: disabled -> Eco(10) -> Normal(3.0) -> Power(1.6667) -> disabled. Unique pattern `41 0F 2F C1 4C 8D 05 ?? ?? ?? ?? BA 80 00 00 00 48 8D 4C 24 60` at `0x14099ccce`; the cvar address is the rip-rel `lea rcx` 14 bytes earlier (`48 8D 0D rel32 E8 ...`).
- How it is used: `0x140602090(gearbox,&out,gear)` (shift rpm window per gear). Coefficient `c`: cvar value if `cvar > 0` and `game_actor+0x199 == 0` (`game_actor=[[0x143550398]+0x31a8]`), else **2.0** (`0x1423d814c`) when disabled. Window is blended between engine def rpm pairs (`engine+0x23c/0x244` low pair, `+0x24c..+0x258` high pair) by weight `w = pow(f, c)`-like of throttle `inner+0x1424` (call `0x141c6db90`). Larger c (Eco 10) = stays near the low window at part throttle; smaller c (Power 1.67) = high window earlier. [I] for the exact blend.
- Default gear/automatic behavior with cvar=0 therefore equals c=2.0, between Normal and Power.
- Dashboard letter: **not found**. `auto_mode_name` / `manual_mode_name` are transmission-def fields (property table at `0x142c36240`: `auto_mode_name` def offset 0x270, `manual_mode_name` 0x290; strings "A"/"M" at runtime `tx+0x258/+0x278`, [I] header shift 0x18). They have no code xref (`0x1421b2c88` / `0x1421b2c98` only in the table). Their selector is `self+0x100` (A mode vs M mode) toggled by the input action `o_im_auto_transmission_mode`. No function mapping cvar value -> "E"/"N"/"P" was found; the "E" shown on the stock dashboard is unexplained (possibly UI script keyed off the cvar, or a HUD widget outside the exe's strings). Open.
- Telemetry `truck.displayed.gear` = gearbox vfunc `+0xb8` (`0x14098d9a0`), no letter.

## 3. Token 0x6b1a3ffc
SCS token = base-38 little-endian, alphabet `\0 0-9 a-z _` (index 0 = end). Verified: `0x6b1a3ffc` = **"petrol"**; `0x16829f66307` = "electric"; `0x56ba050d` = "clutch"; `0x188ccfd7c42` = "dforward", `0x188ccfd7c3f` = "aforward", `0x3a4e6cb791c7` = "abackward", `0x1d60f5850b5` = "steering". So `engine+0x1b8` is the engine **fuel-type** token (the update has a petrol-engine branch at `0x14098c8d4`, electric engines skip clutch/rpm logic).
Pattern for the petrol check: `48 81 BF B8 01 00 00 FC 3F 1A 6B` (unique, `0x14098c8d4`).

## Quick pattern table (all unique in .text)
| what | VA | bytes |
|---|---|---|
| automatic update head | 0x14098c6a4 | `40 38 A9 00 01 00 00 0F 84 ?? ?? ?? ?? 48 8B 51 10 F3 0F 10 35` |
| clutch integrator prolog | 0x140622900 | `48 8B C4 48 89 58 18 48 89 70 20 57 48 81 EC A0 00 00 00 48 8B 99 50 01 00 00` |
| integrator plain store | 0x140622ef4 | `F3 0F 11 8E 2C 14 00 00` |
| integrator slip store | 0x140622eac | `F3 0F 11 86 2C 14 00 00` |
| drive clutch copy | 0x1408c82cf | `8B 83 2C 14 00 00 89 86 30 03 00 00` |
| sequencer target/rate write | 0x140601a77 | `F3 0F 11 80 34 14 00 00 C7 80 30 14 00 00 00 00 00 00` |
| mode-cycle code | 0x14099ccce | `41 0F 2F C1 4C 8D 05 ?? ?? ?? ?? BA 80 00 00 00 48 8D 4C 24 60` |
| clutch input ctor | 0x1405b7672 | `48 C7 45 80 0D 05 BA 56 48 89 87 20 EB 01 00` |
