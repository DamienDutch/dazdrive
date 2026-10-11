# ETS2 1.60 lane assist research

## Facts (verified)
- Feature added in 1.50 ("Lane assistant feature added"; SCS blog 1.50 beta, 2024-04-11, blog.scssoft.com): SCS says making it work with FFB wheels was hard
  "because of possible problems like control-force feedback loops". They do not say how it steers.
- Modes (locale tooltip lane_assistant_mode_op_tooltip): passive "Lane departure warning" (sound) and "Lane keeping support"
  (also steers); both only above 55 km/h; turn signal is considered. cvar `g_lane_assistant` (profile config.cfg),
  string in exe at file 0x2113b48, one code xref (rva 0x3a16c = registration). Input mixes `laneassist` (toggle active LKA, a short press of a bound button)
  and `laneassmode`. Messages "Active lane assistant %s", "Passive lane assistant ..." are UI-hint code (rva ~0x99bb52-0x99bd14), not the controller.
- Tunables in-game: only FFB slider "Assisted steering" (`c_ff_assist`, locale comment: "Slider controlling strength of the force
  feedback when lane assistant is active"), the LKA mode, and the usual FFB sliders (`c_ff_gain`, high/low-speed strength, friction, understeer).
  Also `g_steer_autocenter` and `g_semantical_ff_inactivity` (global config.cfg).
- No LKA gains in def/*.sii: def/force_feedback.sii (ffb_data) has only engine/surface/collision/bump/grind/understeer vibration params. def.scs
  `steering_sensitivity_multiplier_*` apply to relative steering only (keyboard/pad). No accessory def parameters found. No named gains/strings in the exe:
  the controller's gains are hard-coded floats (rip-relative constants in code) -> not exposed as cvars.
- With a direct-drive wheel (absolute steering axis) the profile's controls.sii has `mix steering = dsteering - memory(j_steer_c?1, pow(normalize(...))) - semantical.steering?0`, steer func 0, dz 0.
  The `semantical.*` mechanism is the game's way to inject UI/system-driven inputs into mixes; the "- semantical.steering" term and
  `g_semantical_ff_inactivity` suggest LKA acts as an injected steering input plus FFB torque that moves the wheel (MEDIUM-LOW confidence, not proven).
- ets2la_plugin (MIT): override = write float at vehicle_shared + `steering_angle_offset` (pattern "eb ? f3 0f 10 ? ? ? ? ? f3 0f 5c ? ? ? ? ? 0f 2f d3", +6)
  on game_actor->game_physics_vehicle (core.cpp override_inputs, ~every plugin tick, with 1.5 s ease-in lerp from current angle). This is the physics/wheel-angle
  value AFTER input mixing; it does not touch FFB or the mix. Whether the game recomputes/overwrites it in the same frame order is unknown (ETS2LA works with it,
  so the write sticks at least for their update timing).
- Lane data: the plugin has none. ETS2LA app uses map data from node uid + distance (own map files); we have route nodes (coords, quaternion), see research.md.
  No in-memory lane-center/lateral-offset structure used by the LKA was found (not reversed; the LKA presumably reads prefab/road lane curves from the
  game's navigation/traffic lane data - `traffic_lane_data_u`, `lane_route_item_t` exist as RTTI names in the exe, which is a lead for reading it).

## Why it oscillates (analysis, not measured)
- Typical: P (maybe PD) on lateral offset/heading -> wheel torque, through a heavy truck steering lag + FFB loop (driver's hands/wheel inertia
  and the game's own centering/friction FFB feed back into the steering input). P-only with delay (actuator lag + wheel mass, ~0.2-0.5 s) goes unstable
  past a gain margin; too low gain -> cannot hold a curve (no curvature feedforward, steady-state error). Setting Assisted steering lower
  trades one for the other: the symptom seen in play. With absolute wheel the wheel position IS the steering input, so LKA torque -> wheel motion -> input is a closed loop.
- A good controller needs: curvature feedforward (steer angle ~ atan(L*kappa)), PD/PID on offset+heading error (lookahead point ~ 1-1.5 s ahead, gain scaled
  by 1/v), rate limit on steering (~ 15-30 deg/s of road wheel), lowpass, speed scheduling, hands-off blending.

## Recommendation
1. Cannot tune the stock LKA live (no cvars/gains) -> not worth a code patch (gains are inlined floats; 1 per version; high maintenance).
2. Own controller in plugin via set_steering_angle (ETS2LA route) is feasible technically but the key unknowns are (a) FFB wheel interaction
   (physical wheel will not follow the written angle; the FFB centering/assist force would fight or be inconsistent unless we also drive FFB) and
   (b) lateral lane offset source (needs lane geometry; route nodes give center-line direction but not the truck's lateral offset within lane without
   decoding prefab/road lane curves). Large effort.
3. Simplest robust first step: experiments on the stock one: set c_ff_assist around 0.3-0.5 with c_ff_gain lower / friction up (damping), c_ff_lo/hi_spd 1.0, so the wheel
   isn't fighting; test g_steer_autocenter. And measure: log steering angle + lateral offset to see the oscillation period; then a *damping-only* plugin: low-pass/rate-limit the
   steering physics angle (a plugin filter that smooths the game's steering angle when LKA active) - adds damping w/o needing lane data. Confidence: medium on feasibility, 
   low on the exact LKA mechanism until a probe run logs steering input vs physics angle with LKA on.
Sources: https://blog.scssoft.com (1.50 beta post), game files locale.scs/def.scs (extracted via zlib scan), eurotrucks2.exe offsets above,
 ets2la_plugin src/core.cpp, src/prism/vehicles/vehicle_shared.cpp, src/patterns.win32.hpp.
