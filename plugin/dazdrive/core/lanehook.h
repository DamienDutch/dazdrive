// The game surface of Lane's hooks (WP-LA2): finding the lane-keeping control law's call site and putting the module's stub in its place, the law's minimum-speed
// constant (55 km/h) and the store of the assist state A. Moved out of modules/lane.cpp (ln_start) as it was: the same patterns, the same guards, the same order, so the
// module has no offset, no scan and no patch of its own (the layer lint, tools/check_layers.py, has no Lane line any more). The module decides what the stub does with the
// law's inputs and what it writes back through act_lka_u; this file only installs the hook and moves the one constant.
#pragma once

typedef void (*lane_law_fn)(void *ga, float o1, float o2, float o3, float dt);

enum { LH_OK = 0,
       LH_NO_LAYOUT,    // the layout has no LKA offsets (a build the fields are not known for): the game's lane assist stays the game's
       LH_NOT_FOUND,    // the call site of the control law was not found
       LH_FAILED };     // found, but the patch did not take (o_law is still the game's function)

// Find the call site of the control law and put `stub` there. *game_law = the function the call site called (the game's own law the stub calls through), set whenever the
// site was found (also when the patch failed: the module's other code goes by it).
int lanehook_law(lane_law_fn stub, lane_law_fn *game_law);
// The law switches itself off below 55 km/h (a compare against a pooled 55.0): point that one load at a constant of ours, starting at `kmh`. 1 = patched.
int lanehook_min_speed_patch(float kmh);
int lanehook_min_speed_ok();            // the patch took
void act_lane_min_speed(float kmh);     // the constant now (the module moves it with the speed hysteresis)
// The store of the assist state A (the game integrates our request into it): `filter` sees the value on its way and may limit it. 1 = hooked.
int lanehook_assist_store(float (*filter)(float));
