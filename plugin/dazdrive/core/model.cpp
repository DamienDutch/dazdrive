// The model layer's frame step (core/model.h).
#include "model.h"

void model_frame()
{
    vehicle_frame();
    // (TrueShift sets the idle and the floor offset and refreshes right after; this catches a change that nobody refreshed)
    if (V.eng_ok && bands_stale()) bands_refresh(&V.eng);
    // The driver profile (core/driver.h): the demand history takes the EFFECTIVE share (after the limiter and the actuator caps: what the gear actually
    // plans with, owner point 21) once per simulation second, the second's largest. NAN = the pedal is not the source of the throttle (cruise, standing,
    // neutral, reverse, a pass-through): that second is not recorded. Coasting with the pedal off in gear is a live demand of 0 (wish_tick shapes the
    // released pedal), so a calm driver's lifts are recorded as zeros: the memory counts seconds of the DRIVER's pedal, which is what the reserve is for.
    // It also loads the profile's file on the first frame and saves it on a timer.
    driver_frame();
    demand_frame();
}
