# Speed limit ahead: static RE, no reliable source found

[V] = seen in disassembly/dumps, [I] = inferred. Image base 0x140000000, ETS2 1.60.

## Found
- [V] SDK channel `truck.navigation.speed.limit` is registered at `0x140590b50` (string `0x142111948`; other site `0x1403f975f` is unrelated).
  Its getter is `0x1404e8410`: `movss xmm0,[rcx+0x194]; ret`. rcx is the same object as the hazard getter `0x1404ba990`
  (`[rcx+0x1d0]` bit 4), i.e. **game_actor+0x194 (f32, m/s) = current limit** [I, the object is game_actor by the shared +0x1d0 field].
  `truck.navigation.distance/time` come from a different object (`[0x143550398]+0x40f8`, vtable +0xa0 = GPS/route).
- [V] +0x194 is only the CURRENT limit. Stores to a `[reg+0x194]` float exist at ~50 sites in .text; the ones inspected
  (0x140851b9e/bf4 timer countdown, 0x140905d06, 0x140af7aa2, 0x1406383f7, 0x14046224c) are other classes. The writer for the
  game_actor field was not isolated.
- [V] Data names in the exe: `/def/country/%s/speed_limits.sii` (`country_speed_limit`, `truck_speed_limits`/`car_speed_limits` = traffic AI),
  `sign_speed_limit_attribute` (sign models carry a limit), `speed_signal`, `navigation_*_road_max_speed_usage` (route cost, not a limit).
  Their property descriptors (`0x142c1e168/190/6e8`, `0x142c45618`) hold no code references (reflection tables).
- [I] In ETS2 the HUD limit changes when the truck passes a sign / enters a road type. If so, the limit ahead lives in sign items near
  the road (world item grid), NOT in route_task (physical items = road nodes; logical items = nodes + prefabs; neither has a limit field
  in the route dumps of the first probe run). A reliable static path from route nodes to sign items was not found.

## Built
- `route_limit_ahead(max_m, &dist_m)` in `core/route.cpp`: **returns NAN (unknown)**. The ISA code in `modules/cruise.cpp` is ready:
  with a limit L at distance d it sets `set = min(set, sqrt(v_t^2 + 2*0.5*d))` (v_t = (L+offset) m/s, 0.5 m/s2), never below v_t,
  stops when the driver touches +/- ; log `Cruise: limit X ahead in N m`. ini `speed_limit_ahead` 1 (inactive until the source exists).
- Probe (ini `speed_limit_probe` 1): at every SDK limit change (max 30) `route_probe_limit` logs `limit probe ...` lines: words of the
  route node items seg-2..seg+5 and, one level down, of every pointer inside them (8 words each).

## Method to settle it
1. Drive a route (GPS on) over 3-4 limit changes (town exit, 50 -> 80 -> 90). `dazdrive.log` gets the dumps at each change.
2. Compare the node / pointee words before and after a change (`tools/` diff of consecutive `limit probe` blocks): a field that
   differs between nodes with different limits (or a pointer to a sign/road-look item with a small integer 30..130) is the source.
3. If nothing differs, next probe: dump the world sign items around the truck (find the sign list from a node's pointee chain) or
   breakpoint-free: log `game_actor+0x180..0x1a0` words each frame at the change to see which field leads the +0x194 change (a "next limit" field next to it, [I]).

## Resume while on (separate feature, built)
`resume_to_limit` 1: resume pressed with cruise on (wrapped consume call, `key_event`) -> `keys_tick` puts the set speed back and sets
limit + `speed_limit_offset_kmh` (>= `min_set_kmh`, <= `limiter_kmh` when the game limiter is on). Lower than now: eases via `isa_goal`;
higher: at once; limit unknown (0): nothing. Works with ISA off. Log `Cruise: resume while on: set speed X (limit Y)`. Sim: `keys_test`.

## Probe result: no predictor in the route nodes - closed
A drive with the probe on (16 dumps `limit probe #1..#16`, 937 lines; limits 90/80/90/60/0/90/80/90/80/60/40/90 then a 30/0 flicker when parked at 741 s):
- **The dumps do not change when the limit does.** #9 (80), #10 (60) and #11 (40) have the same node seg 78 and are byte-identical (57 lines); #2/#3 (80 -> 90, seg 1) differ in 4 lines of pointers only. The limit changes while the route node
  index stays the same, so the cause is not in the nodes seg-2..+5 or the items one level down.
- **The item words are constant per road piece, not per limit.** Low byte of word 1 in the items (vtable 14217b518/14211c170): 95 (almost everywhere, at limits 90, 80, 60 and 40 alike), 90, 140, 123, 120, 100, 30, 20, 60, 40, 68 ... with
  type 3/4 in byte 2; they identify prefab/road model (e.g. 30/20 = a ramp piece at +2 in #9-#12, 140 = a junction piece), values in range 20-183 that do not follow 30/40/60/80/90/130. No node/item word equals the new limit or the next one.
- The limit 0 at #5 (130.4 s) and #14/#16 is the SDK reporting "none" (route end / parked), not a limit.
- **Decision: no route source for the limit ahead.** `route_limit_ahead` stays NAN, the ISA easing code stays dormant; `speed_limit_probe` default **0** (ini `speed_limit_probe=1` if someone wants more dumps).
  A real source would have to be the world sign items (next probe idea 3 above: walk from a node's pointee chain to the sign list, or log `game_actor+0x180..0x1a0` at the change to find a "next limit" field). Not pursued.
