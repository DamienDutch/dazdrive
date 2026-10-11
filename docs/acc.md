# ETS2 ACC research (ETS2 1.60.1.7)
## Stock ACC
- Hidden in 1.44, official 1.47 (with emergency brake). Sources: https://traxion.gg/american-truck-simulator-and-euro-truck-simulator-2-v1-47-update-drops-featuring-real-world-truck-tech/ , https://gamepretty.com/euro-truck-simulator-2-how-to-enable-adaptive-cruise-control-and-emergency-brake/ (console `g_acc N`, "meters 1-100"), a Steam guide says 1-10 (conflict; unverified).
- The test profile's `config.cfg` (`profiles/<profile-id>/config.cfg`): `g_acc "17.5"`, `g_cruise_control_smart "3"`, `g_cruise_control_grid "2"`, `g_adaptive_shift "0.0"`. 17.5 = free float, so not a few discrete steps. Unit (m vs s vs coef) NOT verified.
- controls.sii has only `mix accmode semantical.accmode?0` (a key binding "accmode", unbound). No distance-step binding found. No ACC accessory/def string found in exe (no "adaptive_cruise" token); only cvars.
- exe strings (file off): g_acc 0x21138e4, g_cruise_control_smart 0x2113950, g_cruise_control_grid 0x21139c8, "acc: %.2f" 0x212dc50 (debug HUD), indicator_cruise_control 0x21a0f00, cruise_ctrl_active/passive_color 0x21b9160/80, truck.cruise_control 0x210fae0.
## cvar storage (static analysis, RVAs; image base 0x140000000)
- Settings cvars are static global objects, stride 0x140, constructed by per-cvar static-init functions. g_acc: init at RVA 0x39e00; object vtable ptr at RVA 0x2bf5430 (vtable 0x20f7fb0), name inline char[0x20] at +0x08 (RVA 0x2bf5438), value string char[0x65] at +0x28 (RVA 0x2bf5458) initialized from default "0.0"; more fields (type=2 at +0x90, flags, 2x16B at +0x120/0x130 - likely parsed-value/callback cache).
- Values look string-based ("uset g_acc "17.5""). Not verified whether the game re-parses the string on read or uses a cached float updated via a setter (vtable). Not found: the function reading it (the xref scan was shallow).
- Runtime approach: scan module .data for "g_acc\0" (name at +8), check vtable ptr at -8 points into .rdata; then either write string (risky: cache) or call the console/uset handler with "g_acc <v>" (best: same path as typed console command). Need to find console exec function: xref "uset" string in exe (not done).
## Own ACC fallback
- ets2la_plugin: game_actor_u::set_throttle_input / set_brake_input write mix_fixed_ref_t->input->value (patterns in src/patterns.*.hpp, offsets 0x10 / -0x08 from an imm in the ctor). cruise_control_enabled at game_actor+0x175, cruise_control_speed +0x178 (1.55 layout; re-verify on 1.60). Lead vehicle: game_traffic instance -> AI vehicles with physics->acceleration/pos (processing/traffic.cpp), player_vehicle; ETS2LA Python ACC source not fetched (gitlab 403).
## Recommendation
1. Cheapest: keep stock ACC, plugin changes g_acc per speed (time gap * v, clamped). First test manually: in-game console `g_acc 10` vs `g_acc 40` while ACC follows a car to learn unit and whether it applies live. Confidence the cvar is live-effective: ~65%; unit unknown until tested.
2. Fallback: own ACC via throttle/brake inputs + traffic list; works but loses stock smoothness; ~85% feasible, more work.

## Confirmed in game
- `g_acc` is the gap in **meters**. The in-game ACC distance options are 20 m / 30 m / 40 m (they set `g_acc`).
  `g_acc 5` in the console makes the truck follow very close, so a new value takes effect in the game.
  The test profile had 17.5 as its stored value.
- So the ACC module = set `g_acc` from speed (time gap × v, clamped). Open: how the plugin sets a cvar safely
  (console `uset` handler or the cvar object's setter), and whether a change mid-follow applies at once.
- Minimum cruise speed: 30 km/h (observed in game; older versions 40). Stop&Go below that = plugin takes over
  throttle/brake, see docs/smart.md § Stop&Go.
