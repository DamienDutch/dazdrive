# Cruise control / ACC (ETS2 1.60.1.7, static RE)

Exe `eurotrucks2.exe`, image base 0x140000000. Static disassembly only, nothing run. [V] = seen in disassembly, [I] = inference.
Naming as in `clutch-mode.md` / `engine-brake-re.md`: "inner" = vehicle `game_actor+0x18`, game_actor = `[[0x143550398]+0x31a8]`,
cvar reader `float 0x1401d6ad0(cvar)`, int reader `0x1401c7650(cvar)`, text setter `[[cvar]+8](cvar, "text")`.
All byte patterns below were scanned against .text and are unique.

## Summary

- Cruise is **not bang-bang by design**: it is an incremental PI controller (Kp 0.8 per km/h, Ki 0.4 per km/h*s) on a speed error saturated to +-2.5 km/h,
  integrating on the last throttle (`inner+0x1424`), clamped to [0, cap]. Cap = estimated throttle for ~2 m/s2 acceleration. The observed bursts 0->1 are
  probably integrator windup during gear shifts (the PI knows nothing about the open clutch) [I].
- One function computes it: **`0x140991c90`**, called from exactly one place (`0x1405c8550`, inside the throttle function `0x1405c8260`). Clean to detour.
- Cruise state on 1.60: **enabled = `game_actor+0x199` (byte), set speed = `game_actor+0x19c` (float, km/h)**. (1.55 had +0x175/+0x178: shift 0x24.)
- ACC = the same controller with the set speed replaced by a computed target `game_actor+0x1a0` (km/h). Gap to lead = `game_actor+0x1a8` (m, 0 = none).
- Minimum 30 km/h is checked only in the **key handlers** (activate, dec, resume), not in the control loop. A plugin can write `+0x199/+0x19c` directly and
  cruise runs below 30. What really kills cruise at a stop is the **gearbox going neutral** (cancel list, section 5).
- `g_cruise_control_smart` = overspeed allowance (km/h) before cruise starts braking (retarder / engine brake / 0.15 service brake). Options: off(-1), 0, 5, 10.
  `g_cruise_control_grid` = step of + / - in display units; options 1, 2, 5.
- `g_acc` is **not plain meters**: gap D(v) = max(4 m, g_acc * (0.25 + v/30)), v in m/s. So g_acc = the gap at ~22.5 m/s (81 km/h). 0 = ACC off.

## 1. Controller `0x140991c90` (game_actor method)

Signature: `float f(game_actor* rcx, float dt /*xmm1*/, float pedal_throttle /*xmm2*/)`, returns xmm0 = throttle for `inner+0x1424`.
Prolog (19 bytes, no rip-relative inside, trampoline-safe): `48 8B C4 48 89 58 10 48 89 70 18 57 48 81 EC F0 00 00 00` then `80 B9 99 01 00 00 00` (cmp enabled).

| step | what | evidence |
|---|---|---|
| inputs | v = abs(`inner+0x2f0`) * 3.6 (km/h); T = g_acc > 0 ? `+0x1a0` : `+0x19c` (km/h) | `0x140991d1f..d5d` [V] |
| cap | `a_need = g*sin(pitch) + 2.0 + 0.5*v2*drag/mass + roll`, `a_full = engine torque(rpm)*ratio*eff/(R*mass)` (max 0.001), `cap = min(1, a_need/a_full)`. Mass = `0x1405c25d0(game_actor,1)` (total incl. trailers), pitch `0x14062bea0(inner)`. | `0x140991d73..ef8` [V] structure, [I] meaning (sin, mass) |
| error | `e = clamp(T, v-2.5, v+2.5) - v` (so e in [-2.5, +2.5] km/h, no deadband) | `0x140991ee1..f18`, const `0x1423d8240`=2.5 [V] |
| law | `thr = inner+0x1424 + 0.8*(e - e_prev) + 0.4*e*dt`; the D term is skipped when dt <= 1e-4; `e_prev` = `+0x1ac` | `0x140991f1d..f6d`, consts `0x1423d7c50`=0.8, `0x1423d7a44`=0.4, `0x1423d7538`=1e-4 [V] |
| clamp | `thr = clamp(thr, 0, cap)` | `0x140991f71..f85` [V] |
| inactive | `+0x199 == 0`: cruise part = 0 (and e state is not updated) | `0x140991d3b je` [V] |
| limiter | if `0x1405c86a0(game_actor)` (speed limiter on) and v > limit-2.5: same PI against `+0x190` (limit) with state `+0x1b0` -> cap `xmm11` | `0x140991f93..2012` [V] |
| output | `min(max(cruise, pedal), limiter_cap)` | `0x140992018..203e` [V] |

Caller `0x1405c8260(game_actor, dt, xmm2, xmm3)`: pedal curve/ramp -> `+0x1ce4`, then `call 0x140991c90` (`0x1405c8550`), then
`inner+0x1424 = result` only if `inner->0x1f8+0x1b0 + inner+0x10f0 > 0`, `game_actor+0x150 == 2` and `0 <= game_actor+0x17c` (brake request, see 3),
else 0 (`0x1405c858c` / `0x1405c8596`).

Why it can burst: the integrator state is the real throttle, e saturates at 2.5 km/h so it ramps 1.0/s whenever the speed is 2.5+ km/h low, and during a shift
(clutch open, speed falling) it keeps ramping to the cap, then dumps that throttle on the closing clutch. [I] Fix in own controller: freeze integrator while
`inner+0x142c` (clutch) > 0.05 or the gearbox shifts, rate-limit output, anti-windup.

## 2. Overspeed braking ("smart") - in pedal function `0x140992d40`

Block `0x14099372f..0x140993b3a` (rsi = game_actor). Runs when `+0x199 != 0` and smart >= 0 (`0x14099373c`). Note `0x1409936f1`: if g_acc > 0 and smart < 0 then smart = 1.0.
- `A = max(T + smart - 0.25, T + 0.25)` (T as in section 1; hysteresis const `0x1423d7960`=0.25).
- Skipped while the driver throttle pedal (`0x14098bd10` result, stored `[rsp+0x24]`) > 0.
- Truck has a retarder (`tx_def+0x1c4 > 0`, tx_def = resolve(inner+0xfe8)) [I]: retarder demand = `clamp(v - A, 0, 1)` (1 km/h over = full) -> `inner+0x1444`. `0x140993774..0x1409937d0`.
- No retarder, engine brake allowed: engine brake demand `min(v - A, stage)` -> `inner+0x143c` / `[rbp+0x67]` flag. `0x1409938bf..0x140993952`.
- Else (or `0x1405c1fc0` true): **service brake** `clamp((v - A) * 0.15, 0, 0.15)` (const `0x1423d78d0`=0.15). `0x140993957..0x1409939d9`.
- ACC emergency part (g_acc > 0, driver throttle <= 0.1): `D2 = 0.5 * D(v)`; if `0 < gap(+0x1a8) < D2`: brake `clamp((D2 - gap)/D2, 0, 1)`. `0x1409937ea..0x140993892`.
- Final: `0x140993c47 call 0x14083db10` (current brake of vehicle) and `if cur < demand: [vehicle vtbl+0x178](vehicle, demand)` = set service brake.
  Pattern `41 0F 2F C1 73 10 48 8B 03 41 0F 28 C9 48 8B CB FF 90 78 01 00 00` (`0x140993c4c`).
- **Brake request `game_actor+0x17c`** (float 0..1): read at `0x140993509`; the brake is slewed to it at +5/s up, -5/s down (consts `0x1423d8418`, `0x1423d8f30`),
  final brake = max(that, pedal). While `+0x17c > 0` the throttle store in `0x1405c8260` writes 0. The stock ACC sets it to 1.0 (forward-collision, `0x140995e52`) and 0 otherwise.
  A plugin can write it from inside the controller hook (same physics tick, before the brake block) [I, order not tested].

## 3. ACC `0x1409958b0` (game_actor method, xmm1 = dt)

- Radar `0x140998ce0(game_actor, out)` (scene query over nearby vehicles, ~2000 lines, not decoded): out = {gap m, lead speed m/s, 2 more, 1 more}; default FLT_MAX.
  Runs when the truck has the sensor (`0x14052b610`) and (g_acc > 0 or g_emergency_brake or g_blind_spot or g_automatic_high_beams).
- Lead gap used only if `< 100 m` (const `0x1423d8774`). Indicator on (`+0x1d0` bit 4/8) = no following (`+0x1a8 = 0`, `+0x1a0 = +0x19c`).
- Target: `e_gap = gap - D(v)`; ignored (target = set) if the lead is faster than the set speed, lead speed < 0 or gap <= 0.
  Else `T = vlead*3.6 + min(exp(|e_gap| * ln10 / a), 10) * e_gap`, `a = 132840 / mass` (`0x1423d8b74`, `0x1423d81ec`=ln10, `0x141ca03e0` = exp [I]), clamped to [0, set].
  Result `game_actor+0x1a0` (`0x140995dd5`). So: 1..10 km/h of speed offset per meter of gap error, growing exponentially with the error. [V] flow, [I] exp.
- `D(v)` = `0x140992d00(v_mps)` = `max(4.0, g_acc * (v*0.03333 + 0.25))` (consts `0x1423d7788`, `0x1423d7960`, `0x1423d83d4`). [V]
- Lead speed is only a local (`[rsp+0x3c]`); not stored. Hooked: the call to the radar at `0x1409959ea` is wrapped (pattern in `docs/autohold-acc.md` § 2); radar `out` = {gap m, lead speed m/s absolute, FLT_MAX, 0, 0} (defaults set at `0x140998d4c`, args only rcx/rdx).
- Flags: `+0x114` bit 0x10 = collision warning, bits 1/2 = blind spot; `+0x17c` = forward-collision brake request.

## 4. Minimum speed 30 km/h

Constant `0x1423d8640` = 30.0f, **pooled with ~50 other users: never patch the constant**, patch the compare. Enforced only in the key handler `0x14099b660` and its helper:

| place | check | patch to remove |
|---|---|---|
| activate `0x14099b5c0` (called by toggle / inc / dec / resume when off): sets `+0x199=1`, `+0x1a4=0`, `+0x19c = abs(inner+0x2f0)*3.6`; `0x14099b624 comiss xmm0,[30]` | speed < 30 -> `+0x199=0`, `+0x19c=0`, returns 0 | `0x14099b633 73 xx` (jae) -> `EB xx` |
| dec `o_im_cruise_ctrl_dec` `0x14099b920` | new set < 30 -> cruise off (`+0x199=0, +0x19c=0`) | `73 xx` after the `comiss` -> `EB xx` |
| resume `o_im_cruise_ctrl_resume` `0x14099b971`/`b995` | needs saved set (`+0x19c`) >= 30 AND abs(v) >= 30 | NOP both `77 xx` / `72 xx` |
| HUD `0x1409a41e6` | below 30 shows "passive" color/indicator | cosmetic |

inc (`0x14099b74d`): `set = (floor((unit*set + 0.5)/grid) + 1) * grid / unit` (unit `0x142589dec` = 1.0 for km/h, [I] mph factor otherwise), capped by the limiter (`+0x190`) if on.
dec: `set = floor((unit*set - 0.5)/grid) * grid / unit`. Pattern anchors: toggle/inc/dec/resume handler head `F3 0F 10 3D ?? ?? ?? ?? 45 33 FF 84 C9 0F 84 ?? ?? ?? ?? 44 38 BF 99 01 00 00` (`0x14099b871`).

**No auto-cancel on speed drop** in the control loop or pedal function [V: no compare with 30 near the controller or the cancel block]. Plugin can simply write the fields:
`+0x199 = 1`, `+0x19c = set_kmh`, `+0x1a4 = 0` (timer, `+= dt` in `0x140992dcc`; no reader found).

### What cancels cruise (pedal function, all `mov byte [rsi+0x199],0`)

| cancel | condition | store VA |
|---|---|---|
| hub | service brake pedal (`0x140990ee0`) > 0.01; manual gearbox with clutch pedal (`+0x1cc8`) > 0.01; `+0x165` (manual engine brake) != 0; `+0x166` engine-brake toggle (depends on `+0x179` and smart < 0); parking brake `+0x15c`; `+0x160 > 0`; manual retarder `+0x180 > 0` | `0x140993321` |
| manual box shifting | gearbox vfunc `+0x88` false and `+0xb0(1)` true | `0x14099334a` |
| neutral/reverse idle | gearbox `+0x18 == 0` (idle) and `inner+0x1448 (gear) <= 0` | `0x140993367` |
| auto box not in drive | gearbox vfunc `+0xa0` (A mode: `+0x110 <= 1`; M mode: `+0x114 == 0`) or `+0xa8` (`+0x110 == 2`) | `0x14099338e` |
| engine stalled | `drive+0x33d` set and `+0x150 == 2` (also stops the engine) | `0x1409933bf` |
| engine key | start/stop handlers in `0x14099b660` | `0x14099c8ab/c8d0/c99d/ca50`, `c941` |
| keys | toggle off `0x14099b6f6`; dec < 30 `0x14099b92d` | |

**Stop&Go blocker:** the automatic's update `0x14098c640` is fed the *driver pedals* `game_actor+0x1cc0` (throttle) and `+0x1cc4` (brake) at `0x1405b98a5..b98ae`
(`movss xmm2,[r14+0x1cc4]` / `movss xmm1,[r14+0x1cc0]`, then `call [vtbl+0x60]` at `0x1405b98e4`), not the cruise throttle or brake request. At standstill
(abs `inner+0x2f0` < 0.1) with both pedals < 0.01 it selects neutral (`clutch-mode.md` stop behavior) -> cancel `0x14099338e`/`0x140993367` kills cruise. So at a stop
cruise always dies in stock. [V code path, not tested in game]

## 5. game_actor fields on 1.60 (offsets from `game_actor`)

| off | type | meaning | evidence |
|---|---|---|---|
| +0x18 | ptr | inner (vehicle) | everywhere |
| +0x150 | u32 | state (2 = player driven) | `0x140993005`, `0x14099311b` |
| +0x154 | u8 | engine running | engine key handlers; `0x1409958c1` |
| +0x15c, +0x160 | u8, f32 | parking brake flag / value [I] | cancel hub |
| +0x165/+0x166/+0x168 | u8,u8,u32 | engine brake flags / level (see engine-brake-re.md) | |
| +0x17c | f32 | brake request 0..1 (ACC forward-collision; plugin-usable) | `0x140993509`, `0x140995e52` |
| +0x180 | u32 | retarder level | cancel hub |
| +0x190, +0x198 | f32, u8 | speed limiter value km/h / limiter on | `0x1405c86a0`, `0x140991f9c` |
| **+0x199** | u8 | **cruise enabled** (SDK `truck.cruise_control` getter `0x14058db00` returns `enabled ? +0x19c * 0.27778 : 0` m/s) | [V] |
| **+0x19c** | f32 | **cruise set speed, km/h** | [V] |
| +0x1a0 | f32 | ACC target speed km/h (= set when no lead / g_acc = 0) | `0x140995dd5`, `0x140995c5c` |
| +0x1a4 | f32 | seconds since activation (`+= dt` per tick) [I] | |
| **+0x1a8** | f32 | ACC lead gap m (0 = none) | `0x140995c3b` |
| +0x1ac, +0x1b0 | f32 | previous error of cruise PI / limiter PI | `0x140991f57`, `0x140991fe6` |
| +0x1cc0, +0x1cc4 | f32 | driver throttle / brake pedal (to the gearbox) | `0x1405b98a5` |
| +0x1ce4 | f32 | pedal-curved throttle before cruise merge | `0x1405c8547` |

Reader / writer patterns (all unique):
| what | VA | bytes |
|---|---|---|
| SDK cruise getter (reader) | 0x14058db00 | `80 B9 99 01 00 00 00 74 ?? F3 0F 10 81 9C 01 00 00 F3 0F 59 05` |
| activate fn (writer, min-speed compare at `0x14099b624`) | 0x14099b5c0 | `40 53 48 83 EC 20 48 8D 91 88 E8 01 00 48 8B D9 E8 ?? ?? ?? ?? 83 BB 80 01 00 00 00 88 83 79 01 00 00` |
| min-speed compare + store | 0x14099b624 | `0F 2F 05 ?? ?? ?? ?? F3 0F 11 83 9C 01 00 00 73` |
| dec: compare + cancel | 0x14099b920 | `0F 2F CF F3 0F 11 8F 9C 01 00 00 73 ?? 44 88 BF 99 01 00 00 44 89 BF 9C 01 00 00` |
| resume: set >= 30 check | 0x14099b971 | `44 38 BF 99 01 00 00 75 ?? 0F 2F BF 9C 01 00 00 77` |
| resume: speed >= 30 check, enable | 0x14099b995 | `0F 54 C5 0F 2F C7 72 ?? C6 87 99 01 00 00 01` |
| handler head (loads 30) | 0x14099b871 | `F3 0F 10 3D ?? ?? ?? ?? 45 33 FF 84 C9 0F 84 ?? ?? ?? ?? 44 38 BF 99 01 00 00` |
| controller prolog | 0x140991c90 | `48 8B C4 48 89 58 10 48 89 70 18 57 48 81 EC F0 00 00 00 80 B9 99 01 00 00 00 48 8B F9` |
| controller call site (E8 at +0x12) | 0x1405c853e | `F3 44 0F 11 83 E4 1C 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8 ?? ?? ?? ?? 48 8B 4B 18 48 85 C9` |
| throttle store `inner+0x1424` | 0x1405c858a | `72 0A F3 0F 11 81 24 14 00 00 EB 07 44 89 B1 24 14 00 00` |
| overspeed block gate | 0x14099372f | `44 38 A6 99 01 00 00 0F 84 ?? ?? ?? ?? 44 0F 2F D6 0F 82` |
| cancel: hub / manual shift / neutral idle / not in drive / stalled | 0x140993321 / 0x14099334a / 0x140993367 / 0x14099338e / 0x1409933bf | `C6 86 99 01 00 00 00 48 8B 0B 48 8B 01 FF 90 88 00 00 00 84` / `... 48 8B 0B 83 79 18 00 75 17` / `... 48 8B 0B 48 8B 01 FF 90 A0 00 00 00 84` / `... 48 8B 46 18 48 8B 88 50 01 00 00 80 B9 3D 03` / `... 48 8B CE E8 ?? ?? ?? ?? 48 8B 4E 18` |
| brake request read | 0x140993509 | `F3 0F 10 BE 7C 01 00 00 0F 2F FE 48 8B 5E 18` |
| final brake set (`call [vtbl+0x178]`) | 0x140993c4c | `41 0F 2F C1 73 10 48 8B 03 41 0F 28 C9 48 8B CB FF 90 78 01 00 00` |
| gearbox pedal args | 0x1405b98a5 | `F3 41 0F 10 96 C4 1C 00 00 F3 41 0F 10 8E C0 1C 00 00` (18 bytes) |
| ACC update fn | 0x1409958b0 | `48 8B C4 F3 0F 11 48 10 53 57 48 81 EC 18 01 00 00 80 B9 54 01 00 00 00` |
| ACC target store `+0x1a0` | 0x140995dd5 | `F3 0F 11 B3 A0 01 00 00 E9 ?? ?? ?? ??` |
| ACC gap store `+0x1a8` | 0x140995c3b | `F3 0F 11 BB A8 01 00 00 0F 28 CF E9` |
| D(v) fn | 0x140992d00 | `48 83 EC 38 0F 29 74 24 20 48 8D 0D ?? ?? ?? ?? 0F 28 F0 E8` |

## 6. Cvars

| cvar | object VA | values / meaning | reader site (cvar = rip-rel of the leading lea) |
|---|---|---|---|
| `g_acc` | **0x142bf5430** (name at +8, init `0x140039e00`) | float; 0 = ACC off; stock options 40 / 30 / 20 (and 0). D(v) = max(4, g*(v/30+0.25)) | `0x1409936e1`: `48 8D 0D ?? ?? ?? ?? 44 0F 28 D0 E8 ?? ?? ?? ?? 0F 2F C6 76 0A 44 0F 2F D6 73` |
| `g_cruise_control_smart` | **0x142bf4df0** (init `0x140039860`) | km/h allowance; UI options -1 (off), 0, 5, 10; the test profile uses 3 (manual) | `0x1409936bf`: `48 8D 0D ?? ?? ?? ?? F3 44 0F 11 4C 24 30 44 88 7D 77 F3 0F 11 74 24 20 F3 0F 11 75 7F E8` |
| `g_cruise_control_grid` | **0x142bf4cb0** (init `0x140039740`) | + / - step in display units; UI options 1, 2, 5; the test profile uses 2 | `0x14099b74d`: `48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? F3 0F 10 15 ?? ?? ?? ?? 48 8D 8D C0 00 00 00 41 0F 2F C4` |
| `g_emergency_brake` | 0x142bf5570 | int 0/1/2 | |
| `g_lane_assistant` / `g_blind_spot` | 0x142bf57f0 / 0x142bf5a70 | | |

UI option mapping: `0x14106fe64` (grid: 1.0 / 2.0 / 5.0), `0x14106ff01` (smart: -1 / 0 / 5 / 10). Distance key (`accmode`) handler `0x14099b9e3`: `> 30` sets 30 ("ACC normal"), `> 20` sets 20 ("close"), `> 0` sets 0 ("disabled"), else 40 ("far");
so the cycle is 40 -> 30 -> 20 -> 0 -> 40 (matches the in-game 20/30/40 m options).

### Setting g_acc live (same mechanism as g_adaptive_shift) [V]
The game itself does it at `0x14099ba48..ba7e`: `snprintf(buf, 0x80, "%g", double)` (fmt `0x1420bf018`), then `mov rax,[g_acc]` (first qword = vtable) `; lea rdx,buf; lea rcx,g_acc; call [rax+8]`.
So: `typedef void(*set_t)(void* cvar, const char* text); (*(set_t**)cvar)[1](cvar, "27.5");`. Takes effect on the next tick (the controller reads the cvar every tick via `0x1401d6ad0`).
Pattern of the call (cvar = both rip-rel operands): `48 8B 05 ?? ?? ?? ?? 48 8D 54 24 60 48 8D 0D ?? ?? ?? ?? FF 50 08 48 8B 8F 50 E8 01 00` (`0x14099ba6b`).
Plugin steps: scan the pattern, resolve the two rip-rel operands (must be equal), verify the name at `cvar+8` == "g_acc". Same for smart (`0x1409936bf`) and grid (`0x14099b74d`) via their reader patterns.
To hold a time gap h seconds + standstill gap s0: `g = (s0 + h*v) / (0.25 + v/30)` clamped to > 0 (g = 0 turns ACC off); D never goes below 4 m.

## 7. Recommended design (smooth PI + Stop&Go)

1. **Throttle: detour `0x140991c90`** (14-byte absolute jmp over the 19-byte prolog, copy the 19 bytes into the trampoline). Own signature `float hook(void* ga, float dt, float pedal)`.
   - Module off: call the original.
   - On: skip the original; compute T (set speed, or own ACC target from `+0x1a8` + lead speed), run own PI/feed-forward (can use `0x1405c25d0(ga,1)` for mass), freeze integrator while
     clutch (`inner+0x142c`) > 0.05 or shifting, smooth the output, return `min(max(own, pedal), limiter)`; if `0x1405c86a0(ga)` (limiter) is on, call the original for the limiter part.
   - Mode (Eco/Normal/Power) scales the cap (acceleration margin 2.0 m/s2 -> 1.0/2.0/3.0), the gains and the allowed overspeed.
   - Still the right place to write `+0x17c` (brake request, slewed +-5/s by the game) for comfort braking, instead of relying on retarder/0.15 service brake.
2. **Smart cruise brake path** stays stock (retarder/engine brake/0.15) unless the module wants more: use `+0x17c` (up to 1.0) and set `g_cruise_control_smart` to 0..1 while ACC follows.
3. **Below 30 / Stop&Go**: no patch needed for the controller; write `+0x199 = 1`, `+0x19c = set`, `+0x1a4 = 0` directly (own inc/dec/resume logic, ignore the 30 km/h handlers).
   To survive standstill:
   - patch `0x1405b98a5` (18 bytes -> stub): after the two loads, `xmm1 = max(xmm1, 0.02)` while the module holds a stop (stops `0x14098c640` choosing neutral and
     lets the start gear logic run once throttle > 0.01),
   - NOP the stores at `0x140993367` and `0x14099338e` (7 bytes `C6 86 99 01 00 00 00`) while the module is active (conditional stub on a plugin flag), keep `0x140993321`
     (brake pedal / clutch / parking brake / retarder must still cancel),
   - hold the truck with `+0x17c` (or the brake-set call) at standstill, release when the lead moves; start with the PI from throttle 0 (the cap limits the launch to ~2 m/s2).
4. Reads for the plugin without any hook: `+0x199`, `+0x19c`, `+0x1a0`, `+0x1a8`, `inner+0x1424/0x142c/0x2f0`, `g_acc` text. Lead speed: differentiate `+0x1a8` plus own speed.

## Not found / open

- Radar `0x140998ce0` internals (which objects count, lateral width, the 4th/5th out fields) not decoded; I relied on its outputs.
- Meaning of `0x14062bea0` (pitch?), `0x141ca1f30` (sin?), `0x141ca03e0` (exp?) and mass in `[rsp+0x118]` are [I] from structure only.
- Whether the physics order is "pedal fn -> gearbox update" or the reverse within one tick (affects when a write to `+0x17c`/`+0x1cc0` is seen). Hook at the read sites to avoid it.
- Vehicle vtable address of `+0x178` (set brake) not resolved; call it through the object (`[[inner]+0x178](inner, level)`) like the game.
- Unit factor `0x142589dec` for mph and the exact grid/smart option text ids not checked. `+0x1a4` has no reader.
- Everything static; no in-game test of the patches or of cruise below 30 km/h.
