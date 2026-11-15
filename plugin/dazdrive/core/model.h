#pragma once
// The model layer (docs/architecture.md): what the plugin knows about the truck, the road and the driver, kept up to date ONCE per frame by core
// before the modules run, read-only for the modules.
//
//   vehicle  core/vehicle.h   V: mass, ratios, torque curve, force and resistance, what was learned about them
//   bands    core/bands.h     bands_now(): the engine's rpm bands
//   ahead    core/ahead.h     ahead_now(): the road ahead over the next 30 s, with the time each sample arrives
//   demand   core/demand.h    demand_now(): how hard the driver has asked lately
//   wish     core/wish.h      wish_get(): the demand of the driver now (a force)
//   driver   core/driver.h    the driver profile: the demand history's feed and the lift / press / launch learners, kept per game profile in a file
//
// Learners take their samples through the vehicle model's own steps (vehicle_frame); no module writes V.
#include "vehicle.h"
#include "bands.h"
#include "ahead.h"
#include "demand.h"
#include "wish.h"
#include "driver.h"

// Once per frame, before the modules' frame(): the vehicle model's step (vehicle_frame), the bands when the idle or the floor moved since they
// were computed, the demand history's view. The road ahead is NOT stepped here: ahead_now() computes on its first call after its 0.25 s cache
// ran out, as ahead_get always did (a step every frame would sample the route at other moments and change what the layers see).
void model_frame();
