# Engine brake, retarder and mode letter (ETS2 1.60.1.7, static RE)

Exe `eurotrucks2.exe`, image base 0x140000000. Static disassembly only, nothing run. [V] = seen in disassembly, [I] = inference.
Same naming as `clutch-mode.md`: "inner" = vehicle `gearbox+0x10`, "engine" = `resolve(inner+0xff0)`, "tx" = `resolve(inner+0xfe8)`
(resolver `0x1408c4550`), "game_actor" = `[[0x143550398]+0x31a8]`.

## Summary

- **There is no cvar "downshift with engine brake".** Strings checked: every `g_*` name in .rdata; only `g_motor_brake_auto`,
  `g_retarder_auto`, `g_downshift_coef`/`g_upshift_coef` (manual intelligent-shift tuning, unrelated) are close. The downshift on
  engine brake is hard-wired: flag `inner+0x1440` + demand `inner+0x143c` + engine def rpm range (section 1).
- The two options that exist are about *auto-engaging* the brakes when the service brake pedal is pressed:
  `g_motor_brake_auto` (cvar obj `0x142bf34f0`) and `g_retarder_auto` (`0x142bf33b0`), int getter `0x1401c7650(cvar)` [V].
- Engine brake level: `game_actor+0x168` (u32), max = `engine+0x264` (u32). Effective demand = `inner+0x143c` (float). [V]
- Mode letter: `tx+0x258` (char*, auto_mode_name) is read **live** through gearbox vfunc `+0xe0` by the HUD code. [V] for the read, [I] per frame.

## 1. Engine brake and the automatic gearbox

### Cvars [V]
| cvar | obj VA | name copied at | reader | meaning |
|---|---|---|---|---|
| `g_motor_brake_auto` | `0x142bf34f0` (name at +8 = `0x142bf34f8`) | `0x1400381bc` | `int 0x1401c7650(cvar)` | "Automatic engine brake" option (`@@auto_motor_brake@@`); int 0/1 |
| `g_retarder_auto` | `0x142bf33b0` | `0x14003809c` | same | "Automatic retarder" |

`0x1401c7650` = int twin of the float reader `0x1401d6ad0`: follows the override chain `+0x120`, caches the parsed value
(`+0x116` flag, `+0x118` int). Set the same way as g_adaptive_shift (virtual setter with text). The readers sit in the player pedal
function **`0x140992d40`** (extent to `0x140993cac`, `rsi` = game_actor):
- engine brake: `0x140993a6c`: `if cvar && brake_pedal(xmm14) > 0` then demand `xmm11 = (engine+0x264 > 1) ? pedal : 1.0`
  and auto flag `[rbp+0x77] = (pedal >= const && engine+0x260 != 0)`.
- retarder: `0x140993b01`: same shape, feeds `inner+0x1444`.
- Electric engines (`engine+0x1b8 == 0x16829f66307`) add regen from the pedal (`0x140993b3a`), no cvar.

Patterns (each unique in .text, verified):
- motor_brake_auto read, cvar = rip-rel of the leading lea (`0x140993a6c`):
  `48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 85 C0 74 ?? 44 0F 2F F6 76 ?? 83 BF 64 02 00 00 01`
- retarder_auto read (`0x140993b01`):
  `48 8D 0D ?? ?? ?? ?? F3 0F 11 44 24 2C E8 ?? ?? ?? ?? 85 C0 0F 84 ?? ?? ?? ?? 44 0F 2F F6`
  (the call target in both is the int reader; resolve rel32 of the `E8`).

### What the automatic does with an active engine brake [V]
1. **Window swap** (`0x140602090`, the window fn already patterned in discover.py `window`, `0x1406020bc`):
   `if inner+0x1440 != 0 (byte) return pair at engine+0x23c` = `rpm_range_engine_brake` (down, up). So while the engine brake
   is on, the window the chooser uses is the engine-brake range (the chooser calls it at `0x14098d174`, `0x14098d595`).
2. **Upper bound cap** in the update at `0x14098cb43` (right after the decide site call `0x14098cb3a`):
   `if (0 < inner+0x143c) hi = min(window.hi, engine+0x240)` (engine-brake range up value). So with demand > 0 it never stays
   in a gear above the brake range top, i.e. it downshifts. Pattern (unique):
   `44 0F 2F 80 3C 14 00 00 73 ?? F3 0F 10 84 24 F4 00 00 00 48 8D 83 40 02 00 00`
3. Nothing else in `0x14098c640..0x14098e000` touches the engine-brake state (grep of 0x165/0x166/0x168/0x1440/0x260/0x264: none besides above).
4. The **retarder does not touch the gear logic**: `inner+0x1444` is only read by the drivetrain (`0x1408c87a8`, retarder torque),
   the HUD (`0x14052df2a`) and the pedal fn. [V] (no read in the automatic code). Stock downshift on retarder happens only via the
   normal decel/speed logic. [I]

### Where the state is written [V]
Pedal fn `0x140992d40`, tail:
- `0x140993c3a` `88 81 40 14 00 00` (unique): `inner+0x1440 = flag` (byte; picks the first set of three flags:
  `[rbp+0x67]` auto-brake path (`0x14099394e`, needs `engine+0x260 != 0`), `[rbp+0x6f]` manual (hold/toggle/stage), `[rbp+0x77]` pedal auto).
- `0x140993c16` `inner+0x143c = engine-brake demand` (float 0..1, max of manual stage value `game_actor+0x16c/0x174` and pedal auto).
- `0x140993bed` `inner+0x1444 = retarder demand` (float).
- drivetrain: `0x1408c887f` uses `inner+0x143c + inner+0x1424` (throttle) as the engine brake torque term, so 0x143c is the real physical demand.
- Init to 1 in the vehicle ctor at `0x14061b484` (ctor default, overwritten on the first tick).
Rewritten every physics tick, so a plugin cannot hold a changed value; read it in the hook or patch at `0x140993c3a`.

## 2. Engine-brake level / stages

### Level object: game_actor (not inner) [V]
Setter `0x1405c7680(game_actor, u32 level)` (`rcx+0x18` = vehicle, `[vehicle+0xff0]` -> engine):
```
cap     = engine+0x264 (u32)
level   = min(level, cap);  game_actor+0x168 = level
if engine+0x278 (positions array count) == 0:   v0 = level/cap, v1 = (float)engine+0x260
else: entry = positions[min(level-1, count-1)] (12-byte float3 at [engine+0x270 data ptr]): v0, v1, v2
game_actor+0x16c = v0 ;  game_actor+0x188 = v2 ;  game_actor+0x170 (u8) = (v1 > 0)
game_actor+0x378 (float) = game_actor+0x165 ? 1.0 : (game_actor+0x166 ? level/cap : 0)
```
| game_actor off | type | meaning |
|---|---|---|
| +0x165 | u8 | engine brake **held** (momentary key `o_im_engine_brake`, state==2), forces 1.0 |
| +0x166 | u8 | engine brake **on** (toggle / stage lever active) |
| +0x168 | u32 | current stage, 0..cap (0 = off) |
| +0x16c / +0x188 | float | stage values from positions[] (v0, v2) |
| +0x170 | u8 | stage has brake force (v1 > 0) |
| +0x378 | float | brake fraction used by the HUD/sound (`0x1405c3dc8`) |
| +0x179 | u8 | last input was "stage via up/down" (set in the handler) |
| +0x180 | u32 | **retarder level** (SDK `truck.brake.retarder` getter `0x140485260`: `mov eax,[rcx+0x180]`) |

SDK `truck.brake.motor` (bool) getter `0x1404baa00` = `+0x165 || +0x166` (pattern unique:
`80 B9 65 01 00 00 00 75 0C 80 B9 66 01 00 00 00 75 03 32 C0 C3 B0 01 C3`). So the SDK bool is true in manual use only;
the **auto** engine brake (pedal) is *not* in it, only `inner+0x1440` shows it.

Engine def (runtime, `engine`): `+0x260` u8 `engine_brake` (has brake), `+0x264` u32 = max stage (table name
`engine_brake_downshift`, type 0x27: [I] the name is misleading, the code clamps the level with it and treats `>1` as multi-stage),
`+0x268` dyn array `engine_brake_positions` (header at +0x268, data ptr +0x270, count +0x278, 12 bytes per entry),
`+0x23c` rpm range engine brake (2 floats; up value at +0x240), `+0x288` u8 `auxiliary_brakes`-related (many readers, not chased).
Property table `0x142c35c28` (40-byte rows: name, parser `0x141d27040`, offset, type). Caution: for fields *before* the `power`
pair the table offset is runtime+8 (e.g. `type` 0x1c0 -> runtime 0x1b8, `rpm_range_engine_brake` 0x244 -> 0x23c, `rpm_limit`
0x234 -> 0x22c); from `motor_brake` (0x25c) on, table == runtime (the power pair at runtime 0x254 has no table offset). [V] via code.
Max number of stages for real trucks: not readable statically (per-engine .sii value); read `engine+0x264` at runtime.

### Input handler `0x14099b660` (same fn as the Eco/Normal/Power cycle) [V]
`rdi` = game_actor; actions are input refs at `rdi+0x1e880..0x1e8b0`:
- `+0x1e880` hold: `+0x165 = pressed; setter(+0x168)` (`0x14099bd43..bd52`)
- `+0x1e890` toggle (`o_im_engine_brake_tgl`): flips `+0x166`; if now on and level==0, level = cap (`engine+0x264`) (`0x14099bd7b..bdd8`)
- `+0x1e8a0` up: `if level < cap: setter(level+1)`, sets `+0x166 = 1` if the lever call returns false (`0x14099be01..be52`)
- `+0x1e8b0` down: `setter(level-1)`; level 0 -> `+0x166 = 0` (`0x14099be79..bec7`)
- `g_motor_brake_auto` / `g_retarder_auto` are also read here (`0x14099bbeb`, `0x14099bc68`) only to print "...auto %s" messages.
Patterns (unique): setter store `44 89 8F 68 01 00 00` (`0x1405c76c3`); setter head
`4C 8D 98 64 02 00 00 41 3B 1B 0F B6 9F 66 01 00 00 4D 0F 43 C3 45 8B 08 44 89 8F 68 01 00 00` (`0x1405c76ab`);
toggle `44 38 BF 66 01 00 00 48 8B 8F 70 1C 00 00 0F 94 C0 C6 87 79 01 00 00 01` (`0x14099bd7b`);
up `8B 97 68 01 00 00 3B 90 64 02 00 00 73 ?? FF C2` (`0x14099be11`).
The setter is a plain function: a plugin can call `0x1405c7680(game_actor, level)` to set a stage (game-side, same as the key). [I, untested]

### Retarder
Level `game_actor+0x180` (u32, SDK). Demand float `inner+0x1444`. Max stage count: string `retarder.steps` (`0x14213aaf0`) and
`Auxiliary brakes override retarder state count` exist, retarder def at `drive+0x4d0` (drive torque `+0xa8`); not chased.

## 3. Dashboard mode letter

### Which object [V]
Automatic gearbox vfunc **slot `+0xe0` = `0x14098db70`** (vtable `0x1421b96f0`; update `0x14098c640` is slot `+0x60`, displayed gear
`0x14098d9a0` is slot `+0xb8`). Signature `(self, string_dyn* out, bool as_gear_number)`:
```
if !as_gear_number: tx = resolve([self->inner + 0xfe8]);  sprintf(out, "%s", *(char**)(tx+0x258))   ; 0x14098db82..db98
else: gear = vfunc+0xb8(self); out = (gear<0) ? "R" : (gear==0 ? "N" : number)
```
So the object returned by the resolver on `inner+0xfe8` (the same one with shift time `+0x1c0`, ratios `+0x1f0`) **holds
`auto_mode_name` at `tx+0x258`** as a `char*` (format "%s", `0x1420ad848`). Def table offset 0x270 -> runtime 0x258 (shift 0x18) confirmed.
`manual_mode_name`: generic/manual gearbox vfunc `+0xe0` = `0x140602790` reads `[tx+0x278]` the same way (also in vtables
`0x142157728`, `0x142161958`, `0x1421bb890`). The automatic's name function does **not** look at `self+0x100` nor at `g_adaptive_shift`,
so the letter never follows E/N/P by itself.
Patterns: name fn `48 8D 15 ?? ?? ?? ?? 48 8B CB 4C 8B 80 58 02 00 00 48 83 C4 20 5B E9` (`0x14098db8e`, unique; the bare
`4C 8B 80 58 02 00 00` is not: 7 hits). `string_dyn` of the char* itself is `{char* @+0x258, u32 size @+0x260}` [I].

### Live or copied? [V read, I per-frame]
HUD builder `0x14052bd10` (called from `0x140522eb0`, itself called from `0x1405bad5a` / `0x14160ce9c`, per-frame style update with vfunc `+0x88` calls) does, every time it runs:
```
0x14052c8c4: gb = [[ [r13+0x118] +0x150 ] +0x3a8]        ; [I] inner -> drive -> gearbox (self)
             gb->vfunc(+0xe0)(out, as_gear=0)             ; mode letter  -> widget 0x1f
0x14052c98d: same with as_gear=1                          ; gear string  -> widget 0x26
```
then it strcmps the new string with the widget's current text (`[widget+0x108]`) and only pushes a change (`call [widget+0xf8]`).
So the letter is **recomputed from `tx+0x258` on every HUD update and re-applied when it differs**; nothing is copied once. Patterns (unique):
- letter call `45 33 C0 48 8B 88 50 01 00 00 48 8B 89 A8 03 00 00 48 8B 01 FF 90 E0 00 00 00 BA 1F 00` (`0x14052c8cf`)
- gear call `41 B0 01 48 8B 88 50 01 00 00 48 8B 89 A8 03 00 00 48 8B 01 FF 90 E0 00 00 00 BA 26 00` (`0x14052c998`)

### How a plugin can drive D/E/P (not tested)
(a) Overwrite the pointer `*(char**)(tx+0x258)` with a plugin-owned static string ("D"/"E"/"P"), restore on unload (def memory may
free the old one; do not free it ourselves). Takes effect next HUD update. tx object is per truck def, shared by that truck only.
(b) Patch the load at `0x14098db98` (`4C 8B 80 58 02 00 00`, 7 bytes) with a call into a stub that returns our string.
(a) is simpler; the plugin picks the letter from its own mode.
Not found: why the stock dashboard showed "E"; the vfunc cannot produce it from the cvar, so it must be the truck's own `auto_mode_name`.

## Open / not done
- Retarder max stages and the meaning of `game_actor+0x184/0x188`, `+0x17c/+0x178` not chased.
- `engine_brake_downshift` vs `engine_brake_positions` naming mismatch (offset facts above are from code, names from the property table).
- Whether `0x14052bd10` runs every frame (call chain only checked two levels up).
- Everything is static; nothing run in the game.
