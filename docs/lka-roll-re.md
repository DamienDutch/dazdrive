# LKA, eco-roll (neutral) and stop threshold (ETS2 1.60.1.7, static RE)

Exe `eurotrucks2.exe`, image base 0x140000000. Static disassembly only, nothing run. [V] = seen in disassembly, [I] = inference.
Names as in `clutch-mode.md`/`engine-brake-re.md`: "inner" = vehicle `gearbox+0x10`, "drive" = `inner+0x150`, "engine" = `resolve(inner+0xff0)`,
"tx" = `resolve(inner+0xfe8)`. **`game_actor` = `[[0x143550398]+0x31a8]`** (called `ga` below). `ga+0x18` = physics vehicle `veh`
(= `inner`, same object: `veh+0x2f0` speed m/s, `veh+0x640` effective steering, `veh+0x1448` gear).
All patterns below were verified unique in `.text`.

## Part A. Lane keeping assist

### A1. Data flow (all on `ga`, per frame) [V]

```
ga update (0x1405ba990..0x1405bb0bc) --call 0x1405bacf4--> ADAS update 0x1409958b0(ga, dt)
  ADAS update:  0x140995140(ga,&o1,&o2,&o3)  ->  3 floats          (only if g_emergency_brake>1 | g_lane_assistant!=0 | ga+0x1d1&0x20)
                state machine for warnings, emergency brake, blind spot
                call 0x140995f80(ga, o1, o2, o3, dt?) = LKA CONTROL LAW  -> writes ga+0x1d5c  (u, float, -1..1)
steering input fn 0x1409913f0(ga, dt)   (called from player pedal fn 0x140992d40 at 0x1409933c9)
  copies u into veh+0x1414, |u|*0.33*k into veh+0x1418, wheel input into veh+0x1408, rate limit into veh+0x140c/0x141c
vehicle steering physics 0x140625820(veh)
  integrates an "assist" state A = veh+0x1420 (float, -1..1) and writes the effective steering veh+0x640
FFB update 0x14098ae60(ga, ...)
  if ga+0x1d5c != 0: ga+0x59c = clamp(veh+0x1420 + veh+0x640, -1, 1) (wheel position target), ga+0x5a0 = 1; block sent by 0x1401f7860
```

So the output goes to BOTH: (a) the steering physics (through A) and (b) force feedback as a position target. [V]

**Pilot-induced-oscillation mechanism (the game's own, with an FFB wheel).** Physics path `0x140625a6f..0x140625c4a`
reads the options `c_mousesteer` (token cell `0x142c13ef0`) and `c_ff_enabled` (`0x142c13f08`) [V, tokens decoded]. With FFB on and mouse
steering off (the usual setup with a wheel) the assist is NOT added into `veh+0x640` (`0x140625c29: test bl / test al`): it only becomes a wheel position
target `veh+0x1420 + veh+0x640` for the FFB motor (`0x14098b5d8`: `F3 0F 10 80 20 14 00 00 F3 0F 58 80 40 06 00 00`). The wheel moves, the wheel
position comes back as `veh+0x1408` and then `veh+0x640`, so the target moves along with the wheel and always leads it by A. [V code, I meaning]
Per frame (`0x140625b75..0x140625c20`): `A' = clamp(A + dt*( sign(u)*|u|*0.33*kv + decay ), -1, 1)` with
`decay = clamp(-sign(A)*|A|*P*speed_factor)`, `kv` = `veh+0x141c` (speed-dependent rate from `0x1409913f0`). That is an **integrator** (leaky) on u,
feeding a position target that leads the wheel (the wheel velocity follows A), which feeds the road-wheel angle, i.e. a chain of integrators
under a P/PD law with delay = guaranteed oscillation unless the driver holds the wheel. [I, but every link is in the code]
This matches SCS' blog remark about "control-force feedback loops".

### A2. Control law `0x140995f80` (LKA control, arguments: rcx=ga, xmm1=o1, xmm2=o2, xmm3=o3, float [rsp+0x20]=dt) [V]
Prolog pattern (unique, `0x140995f80`):
`48 8B C4 53 48 81 EC C0 00 00 00 0F 29 70 E8 48 8B D9 F3 0F 10 35 ?? ?? ?? ?? 0F 29 78 D8 F3 0F 10 3D`

```
v = |veh+0x2f0| * 3.6                       (km/h)
if v < 55 (0x1423d86d8): u = 0, LKA "active" flag ga+0x1d1&0x20 cleared (message "Active lane assistant disabled"), arming timer ga+0x11c = 0.5, return
if |o1| > 1.95 (0x1423d8130): return (u is left unchanged!)
passive warning: g_lane_assistant != 0 and not active: o1 > 1.25 && !(ga+0x1d0&4)  or  o1 < -1.25 && !(ga+0x1d0&8)  ->  ga+0x114 |= 0x20 (warning)
|o1| < 0.3 (0x1423d79a4): ga+0x118 = 1 (re-arm)
active branch (g_lane_assistant > 1, or ga+0x1d1&0x20 toggled by key):  gains depend on the profile:
   profile F (ga+0x3f0 = c_ff_enabled != 0  and  ga+0x1cb5 = c_mousesteer == 0):  a=0.075   b=0.00275  c=0.5       (consts 0x1423d7830, 0x1423d7634, 0x1423d7ae4)
   otherwise                                                                    :  a=0.027   b=0.00099  c=0.18      (0x1423d7764, 0x1423d75d4, 0x1423d7908)
   u_raw = (a*v)*o2 - (b*v)*o1 - (c*v)*o3
   u = clamp(u_raw, lo, hi);  lo = 0 if ga+0x1d0&4 (blinker) else -1 ;  hi = 0 if ga+0x1d0&8 else +1      -> store ga+0x1d5c
   active only while arming timer ga+0x11c > 0 ... (timer 0.5 s, counted down by dt in the branch, reset to 0.5 at disengage)
```
**Gains are all hard-coded floats, all multiplied by speed in km/h.**
**An FFB wheel gets the strong set: 2.78x higher** gains on all three terms than a non-FFB/mouse setup. [V]
Stores of u: `F3 0F 11 BB 5C 1D 00 00` hits `0x14099628f`, `0x14099629c`, `0x1409962e9` (unique with tails `EB 5D` resp. `40 84`).

Meaning of the three inputs [I]: o1 = signed **lateral offset to the lane center in meters** (it is compared to 0.3/1.25/1.75/1.95; it is the
`.x` of the lane-curve point rotated into the truck frame, `0x140995140` output 1 = `[r13]`), o2 = a second error term (output 2 = `[r12]`,
probably heading error of the lane tangent in the truck frame), o3 = **mean of 7 samples along the lane ahead** (output 3 = `[r15]`;
sample distance = `v_kmh*k*0.01` m for k=1..7, i.e. only ~0.25 s ahead; `0x14090d010` advances along the lane, `0x1407006d0`
evaluates). The lane object is `[[[0x1435503e8]+0x208+8]]` (+0x88 weak pointer to the lane curve, +0x90 float lane value (default -1.0 = none),
+0x98..0xec lane geometry copy). Sign conventions of o2/o3 not verified (no run).

### A3. Existing cvars / options [V]
- `g_lane_assistant` cvar object `0x142bf57f0` (name cell `0x142bf57f8`; int getter `0x1401c7650(cvar)`, same style as g_adaptive_shift; setter virtual with text).
  Readers: ADAS update `0x140995a4d`, LKA `0x140996059/0x1409960b6/0x140996227`, UI `0x14099bb41..`, `0x140407539` (option sync).
  Semantics [I]: 0 = off, 1 = warning only, >1 = lane keeping support; `ga+0x1d1 & 0x20` = "active" toggle from key `laneassist` forces the steering part on.
- Related: `g_emergency_brake` `0x142bf5570`, `g_blind_spot` `0x142bf5a70`.
- **No cvar for gains/strength.** FFB strength `c_ff_assist` (token `0x2ae77dfb66e167b`, cell `0x142c3a6c0`, also `0x142c03fd0`, `0x142c77400`) is read from the
  config registry (`call [vtbl+0x48]` of `0x1425e67f0`, float at `[entry+0x50]`) in the FFB device setup `0x140988a70..0x140989ef5` (read at `0x1409892bd`,
  stored in a local block `[rbp+0xdc]`), so it is applied when the device block is built; whether it is re-read per frame is not known.
  Writing the registry float live may work but is unproven. `c_ff_gain` cell `0x142c3a5d0`, `c_ff_enabled` `0x142c13f08`.

### A4. Hook points (patterns unique) and recommended design
| what | VA | pattern |
|---|---|---|
| ADAS update | 0x1409958b0 | `48 8B C4 F3 0F 11 48 10 53 57 48 81 EC 18 01 00 00 80 B9 54 01 00 00 00` |
| offset fn (reads o1..o3) | 0x140995140 | `48 89 7C 24 20 55 41 54 41 55 41 56 41 57 48 8D 6C 24 80 48 81 EC 80 01 00 00 48 8B 41 18` |
| call of offset fn (outputs: o1=[rsp+0x130], o2=[rsp+0x140], o3=[rsp+0x148] of the caller) | 0x140995a83 | `4C 8D 8C 24 48 01 00 00 48 8B CB 4C 8D 84 24 40 01 00 00 48 8D 94 24 30 01 00 00 E8 ?? ?? ?? ??` (the `E8` is at +0x1b) |
| LKA control law | 0x140995f80 | see A2 |
| call of control law | 0x140995f11 | `41 0F 28 DF F3 44 0F 11 64 24 20 41 0F 28 D6 41 0F 28 CD 48 8B CB E8 ?? ?? ?? ??` (`E8` at +0x16) |
| steering input -> veh fields | 0x1409913f0 | `48 8B C4 48 89 58 18 48 89 78 20 F3 0F 11 48 10 55 48 8D 68 C8 48 81 EC 30 01 00 00 80 B9` |
| store of `veh+0x1418/0x1408/0x1410` | 0x1409917ae | `F3 0F 11 B7 18 14 00 00 F3 44 0F 11 8F 08 14 00 00 F3 44 0F 11 AF 10 14 00 00` |
| store of u into `veh+0x1414` | 0x1409917c8 | `F3 44 0F 11 9F 14 14 00 00` |
| vehicle steering physics | 0x140625820 | `48 8B C4 57 48 81 EC A0 00 00 00 48 89 58 08 48 8B F9 48 8B 89 F8 01 00 00` |
| **store of assist state A** | 0x140625c20 | `F3 44 0F 11 87 20 14 00 00 84 DB 75 04` |
| FFB: u != 0 test | 0x14098b5c1 | `44 0F 2E 8B 5C 1D 00 00` |
| FFB: target = A + steering | 0x14098b5d8 | `F3 0F 10 80 20 14 00 00 F3 0F 58 80 40 06 00 00` |

Recommended designs (cheapest first):
1. **Post-hook the control-law call (0x140995f11)**: stub gets o1..o3 in xmm1..3 (log them: that finally gives lateral offset and heading data),
   calls the original, then edits `ga+0x1d5c`. Free variables: scale u by a speed schedule (lower gain = more stability margin), add damping from the
   derivative of o1 (u -= Kd*d(o1)/dt, low-passed), rate-limit u, or replace u entirely with an own PD + curvature feed-forward law. u == 0 also switches the FFB
   position target off (`0x14098b5c1`), so a dead zone around 0 gives the wheel back to the driver. Deterministic frame order (inside the producer).
2. **Kill the integrator windup at the source (0x140625c20)**: stub on the store of A: clamp |A| <= Amax(v) and leak faster (A *= 1 - leak*dt), or
   replace A by a proportional term (A = Kp*u) so the chain is only P-to-position and cannot run away. This is the real fix for the loop; (1) alone only
   lowers the gain. Combine: (1) with a damped u, (2) with a clamp.
3. Do not scale only `veh+0x1418` (`0x1409917ae`): that only changes the integrator rate and leaves the windup.
Verification needed first (cheap probe run, no hook): log `ga+0x1d5c`, `veh+0x1420`, `veh+0x640`, `veh+0x1408`, o1..o3 with LKA on, to confirm
signs, units of o2/o3 and the oscillation period.

## Part B. Eco-roll: coasting in neutral through the automatic

### B1. What the code does if the target gear becomes 0 while driving (A mode, `self+0x100 != 0`) [V]
Automatic update `0x14098c640` (self = rsi; arguments xmm1 = throttle, xmm2 = brake, xmm3 passed through). Fields: `self+0x68` target gear,
`self+0x18` shift state, `self+0x100` A mode, `self+0x101` at-rest latch, **`self+0x110` drive state**, `self+0x114` saved/manual state.
Drive state values (from the save-restore `0x14098d8b0`: `gear>0 -> 4`, `gear==0 -> 2`, `gear<0 -> 0`): **4 = forward drive, 3 = neutral after
drive, 2 = neutral, 1 = neutral after reverse, 0 = reverse**.
- The update ends with a **tail call into the generic gearbox**: `0x14098cd48  E9 rel32` -> `0x140601a10` with rcx = self, xmm1..3 = the args.
  Pattern (unique, `0x14098cd40`): `49 8B E3 41 5E 5F 5E 5B E9 ?? ?? ?? ??`. Whatever wrote `self+0x68` earlier, this is the last point before the gearbox
  reads it. **Best place to force neutral or any gear.**
- Generic gearbox `0x140601a10`, state 0: `self+0x68 != inner+0x1448` -> state 2 (`0x140601c99`). State 2: opening rate for a neutral target is multiplied by
  100.0 (`0x140601b98 mulss xmm0,xmm1`, const `0x1423d8774`) = effectively instant. State 1 (gap): `if gear==0 or target==0` the wait timer is skipped
  (`0x140601c1e..0x140601c2f`). Then `vfunc+0x70` = `0x1406019c0` stores the gear (accepts `-tx+0x228 .. tx+0x200`, 0 included). State 3 closes the clutch
  (nothing is transmitted in neutral). So **a 0 target shifts to neutral cleanly and fast**. [V]
- **Clamp caveat**: chooser call site 1 (`0x14098cc82`) clamps the result to `[gear-1, gear+1]` (`0x14098cc87..0x14098cc92`), so returning 0 from a gear > 1 gives gear-1 there.
  Call site 2 (`0x14098ccc6`) stores the raw result (`0x14098cccb mov [rsi+0x68],eax`). So neutral must be written to `self+0x68` (decide stub or tail hook), not returned. [V]
- Next frame (gear == 0): the chooser part is skipped (`0x14098ca67 test r14d,r14d; je 0x14098ccce`), the update does `0x14098c95a: if self+0x68==0: state 4->3 (0->1)`. [V]
  Speed >= 0.1 m/s clears the at-rest latch (`0x14098c70f`). So the update keeps running and the drive state becomes 3.
- **Re-engagement path (stock, any speed)** `0x14098c716..0x14098c75f`: state 2/3 (or 4 with gear 0), `throttle > 0.01`:
  `target = max(start_gear_fn 0x14098dbf0, 0x14098deb0)`, state = 4. No speed condition. [V]
  - `0x14098dbf0`: picks the gear whose predicted engine rpm (`drive+0x2ec*9.5493 * ratio`) is nearest to `sqrt(window.lo*window.hi)` of that gear (window fn `0x140602090`),
    scanning from gear 1 up and stopping when the predicted rpm drops below 150. So **at speed it returns an rpm-matched gear**, not gear 1. Early `return 0` if rolling forward
    and `self+0x114 == 0` (or backward and `+0x114 == 4`): `self+0x114` is only written by save-restore and the manual-mode code, so normally it is not 0 in A mode. [V code, I about the runtime value: log it].
  - `0x14098deb0`: weight/grade start gear (`self+0x104` solo / `+0x108` with trailer when grade <= 0, else `0x140602470` for the slope). `max()` of the two. So the result is >= the weight start gear
    (4 solo, 1 with trailer, as seen), i.e. an upshift-biased pick if the rpm-matched gear is lower. [V]
  - Brake does not re-engage; braking in state 3 leaves neutral as is (`0x14098c764..0x14098c78f`). At standstill with brake the stock goes to reverse (`target=-1`).
- HUD: `0x14098d9a0` (`truck.displayed.gear`) returns the start gear instead of 0 while `self+0x110 >= 3`, so the display does not flicker to N. [V code]

### B2. Is returning 0 unsafe?
Not by itself: it is the game's own neutral (stops, automatic stall protection, reverse changes all use it). Risks, in order of importance [I unless noted]:
1. **Engagement at speed**: closing the clutch from idle rpm onto a gear needing ~1300 rpm. The closing rule (`clutch-mode.md` § launch slip) keeps the clutch slipping while the engine is
   below the gear's low window bound, so expect a drag-up (slip) of the engine, not a bang. Pick the engage gear ourselves (our chooser's gear) instead of the stock `max(start gear)`.
2. **No engine brake / retarder effect in neutral**: eco-roll must end (re-engage) on service brake use, a downhill grade, retarder/engine-brake request, and speed over a limit.
3. **Stock re-engage on any throttle > 0.01** (cruise control counts as throttle for the update arguments, it was seen to make the update see full throttle): good as a fallback, but the picked
   gear (`max(rpm-matched, weight start gear)`) may differ from ours. If we want ours, the tail hook runs after it and can overwrite `self+0x68`.
4. **Hill-hold timer** `ga+0x1d00` (set by `g_hill_brake_assist`, `0x1409933f0..0x1409935a7`): when active and `0x14098bd10(ga) <= 0` the update itself selects neutral (`0x14098c93f`). Only at standstill; no interaction when rolling.

### B3. Recommended design (primary)
- **Roll entry** in a tail hook at `0x14098cd48` (replace the `E9`, jump on to `0x140601a10` afterwards): when the roll conditions hold (gear != 0, shift state `self+0x18 == 0`, throttle ~0, brake ~0,
  grade small/positive, no engine-brake/retarder, speed in range, eco-roll enabled) write `[self+0x68] = 0`. The gearbox shifts to neutral next call, the update moves the drive state to 3 on its own.
- **Roll exit**: while `inner+0x1448 == 0` and drive state 3, write `[self+0x68] = our_gear` from the same hook when brake, grade, engine brake, speed or a timer says so (the gearbox then shifts
  0 -> g through the normal sequencer), or simply let the stock path engage on throttle > 0.01 (it also sets state 4). If our gear is written while state is 3 the next frame's stock path will
  recompute and overwrite it only if throttle > 0.01, and the tail hook runs last, so ours wins.
- Pattern for the tail jump (unique): `49 8B E3 41 5E 5F 5E 5B E9 ?? ?? ?? ??` (the `E9` is at +8; self = rcx at that point, current gear `[[rcx+0x10]+0x1448]`, drive state `[rcx+0x110]`).
- Alternative (not recommended): keep the gear and hold the clutch open (`inner+0x1430 = 1.0`, `+0x1434 = rate` in shift state 0, clutch-mode.md). Then the automatic keeps running its gear logic
  on a disengaged driveline (its rpm/stall checks use `1 - clutch` and would misfire), and any shift request rewrites the clutch fields. Evidence for the neutral route is stronger.

## Part C. Where the automatic opens the clutch when stopping ("pushing at 2 km/h")

[V] In the update (`0x14098c8c3..0x14098c957`), executed every frame when gear != 0 (skipped only for petrol engines with `tx+0x1b8 > 0` in M mode, `0x14098c8d4..0x14098c8f2`):
```
rpm   = |drive+0x2e4| * 9.5493          (engine speed; 0x14098c8a0)
idle  = engine+0x228 + drive+0x2dc       (xmm11)
base  = idle * 0.8                       (0x14098c8c3: mulss xmm7,[0x1423d7c50]  -> xmm7)
avgBrake = 0x14083db10(inner)            (mean of the per-wheel brake floats at inner+0x6c8, count wheels at vehicle+0x200+0x608)
T     = base + avgBrake^2 * (1 - inner+0x142c) * idle          (0x14098c8f4..0x14098c925, xmm6)
neutral (self+0x68 = 0, 0x14098c957) if
     (rpm <  T  and  drive+0x33c == 0)                          (0x14098c929 comiss, 0x14098c92f cmp)
  or (rpm <  base)                                              (0x14098c939 comiss xmm9,xmm7)
  or (ga+0x1d00 > 0 and 0x14098bd10(ga) <= 0)                   (hill hold, 0x14098c93f..0x14098c951)
```
- `drive+0x33c` is written in the drivetrain `0x1408ca690..0x1408ca960`: set to 1 when `|drive+0x2c8|*9.55 < throttle*idle` (pedal launch zone) or in gearbox state 3 with throttle > 0; cleared
  (`0x1408ca759`) when the gearbox is idle and `|drive+0x2c8|*9.55 > idle - 20` (`0x1423d85b8 = 20.0`) or when any wheel brake is applied (`0x1408ca74f call 0x14083db10`). So with brake applied and no throttle the
  `T` rule is live.
- So the stock rule is: the clutch is only opened (via target 0 -> sequencer state 2) when the **engine** speed (dragged down by the closed clutch) falls under **0.8 * idle** (about 480 rpm at idle 600;
  in gear 1 this is the ~2 km/h seen in game). Brake raises T quadratically, but only 0.04*idle at 20 % brake, so light braking behaves like 0.8 * idle. [V values, I for the 2 km/h arithmetic]
- **Constant `0x1423d7c50` (0.8) is shared** (the LKA uses it too, `0x14099624b`), so do not patch the data.
- **Hook**: the instruction `0x14098c8c3` (8 bytes) `F3 0F 59 3D rel32` = `mulss xmm7,[0x1423d7c50]`; pattern unique:
  `F3 0F 59 3D ?? ?? ?? ?? 45 85 F6 0F 84 ?? ?? ?? ??`. Replace it with `call stub` (E8 rel32, 5 bytes) + 3-byte nop. The stub must preserve every register except xmm7
  (rax holds `tx` and is used right after at `0x14098c8e1`; rbx, rcx, rdx, r8-r15, xmm6-xmm15 are live) and return `xmm7 = max(idle*0.8, idle + 150 + k*brake)`. At that point xmm7 = idle,
  xmm10 = throttle, xmm12 = brake (so "earlier when braking harder" is available in the stub), xmm9 = rpm, xmm11 = idle. This moves both `base` and `T`.
  Simplest variant: `addss xmm7,[ours = 150.0]` in place of the `mulss` (same length `F3 0F 58 3D`) gives idle+150 with no stub, but loses the brake dependence and the "do not go below 0.8*idle" safety.
  Do not raise it above where the engine can still drive the truck in gear 1 on a launch: the launch zone is protected by `drive+0x33c == 1` only for the `T` rule, **not for `base`**
  (`rpm < base` has no 33c exemption: with throttle held at low rpm a raised base would drop to neutral). Make the stub return the stock value when `throttle > 0.01`.
- Not found: any other place that opens the clutch for stopping. The target-0 selection in the first block (`0x14098c6de`: speed < 0.1 m/s, throttle and brake < 0.01 -> neutral + latch) is the
  at-rest case, after the truck has already stopped.

## Could not find / open
- Exact meaning and sign of o2/o3 (A2) and units of lane value `+0x90`; needs the probe run.
- How `c_ff_assist` and `c_ff_gain` scale the FFB packet inside the device layer (`vfunc +0x170` of the device in `0x1401f7860`) and whether `c_ff_assist` is re-read per frame.
- Value of `self+0x114` in A mode at runtime (start gear fn early return).
- What the avg per-wheel float array at `inner+0x6c8` is exactly (assumed brake level from its use in the hill-hold code, `0x14099351d`).
