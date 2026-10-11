# Horizon: what the game gives and does not give

Belongs to `docs/horizon.md` (the candidates A2-D4 are there). Only the data sources and their status.

| Needed for | Source | Status |
|---|---|---|
| Lead gap / lead speed | `ga+0x1a8` gap; lead speed from the radar-out hook (`0x1409959ea`, `docs/autohold-acc.md`) | in use (own ACC, AEB) |
| AI traffic (pos, speed, accel) | `game_traffic_u` via pattern, ets2la layout | pattern known, structs not verified (A2) |
| Traffic-light state | semaphore actors in the same list | not decoded (A7) |
| Route height/bends/junctions | `gps_manager → route_task → node_item_t` | in use (route.cpp); roundabout flag missing |
| Speed limit ahead | ? (sign items / road look) | not found (`speedlimit-ahead-re.md`), no predictor in the route nodes |
| Blind spot / collision warning | `ga+0x114` bits 1/2 / 0x10 | known, not used for tones (C1) |
| Brake temp, fuel, wheel slip/substance, lights, trailer placement/angular velocity | SDK channels | available, not registered |
| Gameplay events (job delivered, ferry, train, fined) | SDK | available, not used (D1, reset triggers) |
| Setting the engine-brake / retarder level | setters `0x1405c7680` / `0x1405c74c0` | known; canceling cruise via the hub (A4) |
| Braking | `ga+0x17c` (slew ±5/s), hazards-mute patch | in use; never latch |
| Throttle | cruise call-site hook (`stub_cruise`) | in use |
| Steering | LKA law hook + A-store thunk (FFB path) | in use; non-FFB path not tested |
| Rain intensity, tire temperature, distance to objects, lane width | – | not in the SDK or any known struct |
