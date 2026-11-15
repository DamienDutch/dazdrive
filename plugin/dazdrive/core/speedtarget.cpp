// The speed chooser's limit knowledge in the model layer (core/speedtarget.h). speed_top_kmh was Cruise's cruise_top_kmh: the same
// expression, moved here so the wish (core/wish.cpp) and the gearbox read the model instead of the cruise module (keuzes 12). The
// only module state it needs are the two ini knobs the cruise publishes (g_speed_limiter_kmh, g_car_trailer_lim_kmh).
#include "speedtarget.h"
#include "sense.h"
#include "vclass.h"

volatile float g_speed_limiter_kmh = 0;
volatile float g_car_trailer_lim_kmh = 0;

int speed_limiter_on()
{
    static void *cv;
    static double at;
    static float use = NAN;
    static int tried;
    if (!cv && !tried) { tried = 1; cv = sense_cvar("g_use_speed_limiter"); if (!cv) logline("Speed target: g_use_speed_limiter not found (the speed limiter counts as off)"); }   // one scan: a miss must not scan the game's memory every frame
    if (cv && wall_s() - at > 1) { at = wall_s(); use = sense_cvar_get(cv); }
    return use == use && use > 0.5f;
}

float speed_top_kmh()
{
    if (vprof()->trailer_cap) return T.trailer ? g_car_trailer_lim_kmh : 0;   // a car: the trailer cap or nothing (no truck limiter)
    if (g_speed_limiter_kmh > 0) return g_speed_limiter_kmh;
    char *ga = (char *)sense_actor();
    if (ga && speed_limiter_on() && sense_lim_on(ga) && sense_lim(ga) > 0) return sense_lim(ga);   // the option g_use_speed_limiter decides (ATS 01-11: the byte stays set with the option off)
    return 0;
}
