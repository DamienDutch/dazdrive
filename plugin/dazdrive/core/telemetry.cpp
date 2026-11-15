// SDK telemetry: channels and the truck configuration, shared by all modules.
#include "suite.h"
#include <math.h>
#include <string.h>
#include <math.h>
#include "route.h"
#include "vehicle.h"
#include "vclass.h"
#include "roll.h"
#include "driver.h"

#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_common_channels.h"
#include "common/scssdk_telemetry_truck_common_channels.h"
#include "common/scssdk_telemetry_common_gameplay_events.h"

telemetry T;

static SCSAPI_VOID ch_float(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(float *)ctx = v->value_float.value; }
static SCSAPI_VOID ch_s32(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(int *)ctx = v->value_s32.value; }
static SCSAPI_VOID ch_u32(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(unsigned *)ctx = v->value_u32.value; }
static SCSAPI_VOID ch_bool(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(int *)ctx = v->value_bool.value != 0; }
// channels whose absence matters (lift axles, lights, game time): the callback also marks that the channel delivered
struct seen_dst { int *dst; unsigned bit; };
static unsigned g_ch_seen;
static SCSAPI_VOID ch_bool_seen(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) { const seen_dst *d = (const seen_dst *)ctx; *d->dst = v->value_bool.value != 0; g_ch_seen |= d->bit; } }
static SCSAPI_VOID ch_gametime(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t)
{ if (v) { T.game_min = v->value_u32.value; g_ch_seen |= 16; } }
static SCSAPI_VOID ch_wheel_w(const scs_string_t, const scs_u32_t i, const scs_value_t *const v, const scs_context_t)
{ if (v && i < 16) T.wheel_w[i] = v->value_float.value; }
static SCSAPI_VOID ch_wheel_lift(const scs_string_t, const scs_u32_t i, const scs_value_t *const v, const scs_context_t)
{ if (v && i < 16) T.wheel_lift[i] = v->value_float.value; }
static SCSAPI_VOID ch_twheel_lift(const scs_string_t, const scs_u32_t i, const scs_value_t *const v, const scs_context_t)
{ if (v && i < 12) T.t_wheel_lift[i] = v->value_float.value; }
unsigned telemetry_seen() { return g_ch_seen; }

static SCSAPI_VOID ch_place(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t)
{ if (v) T.place = v->value_dplacement; }
static SCSAPI_VOID ch_tplace(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t)
{ if (v) { T.trailer_place = v->value_dplacement; T.trailer_place_t = wall_s(); } }
static SCSAPI_VOID ch_acc(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t)
{ if (v) T.acc = v->value_fvector; }
static SCSAPI_VOID ch_angvel(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t)
{ if (v) T.yaw = v->value_fvector.y; }

static SCSAPI_VOID ev_frame_start(const scs_event_t, const void *const info, const scs_context_t)
{
    T.sim_time = ((const scs_telemetry_frame_start_t *)info)->simulation_time;
}

// Continuity: whatever makes a module's timers, filters or learned values wrong calls session_reset.
static double g_paused_at = -1;

static SCSAPI_VOID ev_paused(const scs_event_t ev, const void *const, const scs_context_t)
{
    int p = ev == SCS_TELEMETRY_EVENT_paused;
    if (p && !T.paused) g_paused_at = wall_s();
    if (!p && T.paused && g_paused_at >= 0) {
        // back from the pause menu / overlay: wall-clock timers, rate limits and holds are stale
        if (wall_s() - g_paused_at > 0.5) session_reset(RESET_SOFT, "resumed after a pause");
        g_paused_at = -1;
    }
    T.paused = p;
}

static SCSAPI_VOID ev_gameplay(const scs_event_t, const void *const info, const scs_context_t)
{
    const scs_telemetry_gameplay_event_t *g = (const scs_telemetry_gameplay_event_t *)info;
    if (!g || !g->id) return;
    DEV_NOTE(gameplay(g));
    if (!strcmp(g->id, SCS_TELEMETRY_GAMEPLAY_EVENT_player_use_ferry) ||
        !strcmp(g->id, SCS_TELEMETRY_GAMEPLAY_EVENT_player_use_train))
        session_reset(RESET_HARD, "ferry / train (teleport)");
}

void session_reset(int level, const char *why)
{
    static const char *const name[] = { "soft", "load", "hard" };
    if (level >= RESET_HARD) logline("session reset (%s): %s", name[RESET_HARD], why);
    else logv(level >= RESET_LOAD ? 1 : 2, "session reset (%s): %s", name[level], why);
    if (level >= RESET_HARD) {
        route_reset();
        g_truck_unsupported = NULL;  // judged again for the (possibly other) truck
    }
    vehicle_reset(level);
    roll_reset(level);
    driver_break(level);
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->reset && module_active(g_modules[m])) g_modules[m]->reset(level, why);   // (a module outside the class's set is absent: nothing to reset)
    DEV_NOTE(reset(level, why));
}

// The vehicle class changed (core/vehicle.cpp decided another one, vclass_sync() reported it) and the module set changes with it. The
// modules that just left the set get one hard reset to let go of what they held (pedal wishes, shared flags, holds); then every module of
// the set that comes in starts clean (session_reset), and gets the configuration the vehicle already announced (config(NULL): it came
// before the class was known). The set is logged once.
static void log_module_set(const char *what)
{
    char set[256] = "";
    size_t n = 0;
    for (int m = 0; m < g_n_modules; m++) {
        if (!module_active(g_modules[m])) continue;
        n += snprintf(set + n, sizeof set - n, "%s%s", n ? ", " : "", g_modules[m]->name);
        if (n >= sizeof set) break;
    }
    logline("%s: %s; modules: %s", what, vprof()->name, set);
}
void vclass_switched()
{
    log_module_set("vehicle class");
    // What the truck-only modules publish for the others (eco-roll, crawl, launch control, the decel assist, the planner's climb need): a
    // module outside the set is absent, so the flags read as when it never ran (the modules' own resets below clear their wishes)
    g_rolling = 0; g_lv.crawl_toggle = 0; g_lv.crawl = 0; g_lv.lcx = 0; g_roll_boost_kmh = 0; g_pcc_need_kmh = NAN; g_sa_nohold = 0; g_lv.assist = 0;
    memset(&g_planner_drive, 0, sizeof g_planner_drive);
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->reset && !module_active(g_modules[m])) g_modules[m]->reset(RESET_HARD, "left the module set");
    session_reset(RESET_HARD, "vehicle class changed");
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->config && module_active(g_modules[m])) g_modules[m]->config(NULL);
}
void vclass_log_initial() { log_module_set("vehicle class (until the model tells)"); }

static SCSAPI_VOID ev_config(const scs_event_t, const void *const info, const scs_context_t)
{
    const scs_telemetry_configuration_t *c = (const scs_telemetry_configuration_t *)info;
    if (!strcmp(c->id, SCS_TELEMETRY_CONFIG_truck)) {
        // a new configuration replaces the old one completely (stale gears of a previous truck must not survive)
        memset(T.ratio, 0, sizeof T.ratio);
        memset(T.wheel_radius, 0, sizeof T.wheel_radius);
        memset(T.wheel_powered, 0, sizeof T.wheel_powered);
        T.fwd_gears = T.rev_gears = T.wheels = 0;
        T.fuel_cap = 0;
        T.cfg_gen++;
        T.shifter[0] = 0;
        T.retarder_steps = 0;
        T.air_warn_cfg = T.air_emerg_cfg = 0;
        memset(T.wheel_liftable, 0, sizeof T.wheel_liftable);
        for (int k = 0; k < 16; k++) T.wheel_z[k] = NAN;
        for (const scs_named_value_t *a = c->attributes; a->name; a++) {
            const char *n = a->name;
            unsigned i = a->index;
            if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_liftable) && i < 16 && a->value.type == SCS_VALUE_TYPE_bool) T.wheel_liftable[i] = a->value.value_bool.value != 0;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_position) && i < 16 && a->value.type == SCS_VALUE_TYPE_fvector) T.wheel_z[i] = a->value.value_fvector.z;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_gear_count)) T.fwd_gears = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_reverse_gear_count)) T.rev_gears = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_differential_ratio)) T.diff = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_rpm_limit)) T.rpm_limit = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_fuel_capacity) && a->value.type == SCS_VALUE_TYPE_float) T.fuel_cap = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_retarder_step_count)) T.retarder_steps = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_air_pressure_warning) && a->value.type == SCS_VALUE_TYPE_float)
                T.air_warn_cfg = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_air_pressure_emergency) && a->value.type == SCS_VALUE_TYPE_float)
                T.air_emerg_cfg = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count)) T.wheels = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_ratio) && i < 32) T.ratio[i] = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_radius) && i < 16) T.wheel_radius[i] = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_powered) && i < 16) T.wheel_powered[i] = a->value.value_bool.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_shifter_type) && a->value.type == SCS_VALUE_TYPE_string)
                snprintf(T.shifter, sizeof T.shifter, "%s", a->value.value_string.value);
        }
        logline("truck config: %d forward gears, diff %.3f, rpm limit %.0f, wheel radius %.3f m, retarder %u steps, "
                "shifter %s", T.fwd_gears, T.diff, T.rpm_limit, wheel_radius(), T.retarder_steps, T.shifter);
        logline("truck config: brake air warning %.0f psi, emergency %.0f psi (0 = not reported); our requests pause below %.0f psi%s",
                T.air_warn_cfg, T.air_emerg_cfg, air_pause_psi(), g_air_from_truck >= 0.5f && air_truck_ok() ? " (from the truck)" : "");
        // Same truck again (a reload of the config) is nothing; another truck resets everything we learned.
        double sig = T.fwd_gears * 1000.0 + T.diff * 7.0 + T.rpm_limit * 0.013 + wheel_radius() * 31.0 + T.wheels;
        for (int i = 0; i < T.fwd_gears && i < 32; i++) sig += T.ratio[i] * (i + 3);
        static double last_sig = -1;
        if (last_sig >= 0 && fabs(sig - last_sig) > 1e-6) session_reset(RESET_HARD, "another truck");
        last_sig = sig;
    } else if (!strcmp(c->id, "trailer.0")) {   // the first trailer: how many wheels, where, and which can be lifted (if the game says)
        memset(T.trailer_wheel_liftable, 0, sizeof T.trailer_wheel_liftable);
        for (int k = 0; k < 16; k++) T.t_wheel_z[k] = NAN;
        T.t_wheels = 0;
        for (const scs_named_value_t *a = c->attributes; a->name; a++) {
            const char *n = a->name;
            unsigned i = a->index;
            if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count) && a->value.type == SCS_VALUE_TYPE_u32) T.t_wheels = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_liftable) && i < 16 && a->value.type == SCS_VALUE_TYPE_bool) T.trailer_wheel_liftable[i] = a->value.value_bool.value != 0;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_position) && i < 16 && a->value.type == SCS_VALUE_TYPE_fvector) T.t_wheel_z[i] = a->value.value_fvector.z;
        }
    } else if (!strcmp(c->id, SCS_TELEMETRY_CONFIG_job)) {
        for (const scs_named_value_t *a = c->attributes; a->name; a++)
            if (!strcmp(a->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_cargo_mass)) T.cargo_mass = a->value.value_float.value;
    }
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->config && module_active(g_modules[m])) g_modules[m]->config(c);
    DEV_NOTE(config(c));
}

float wheel_radius()
{
    float r = 0;
    for (int i = 0; i < T.wheels && i < 16; i++)
        if (T.wheel_powered[i] && T.wheel_radius[i] > r) r = T.wheel_radius[i];
    return r;
}

// Discontinuities seen in the data itself: hitches, a time that went back (load), a jump in place (quick travel,
// ferry, teleport), the engine starting, a trailer (de)coupled. The first frames only learn the starting values.
static void watch_continuity(double t)
{
    static double last_t = -1, last_wall;
    static double px, py, pz;
    static int have_pos, was_engine = -1, was_trailer = -1;
    double w = wall_s();
    if (last_t >= 0) {
        double dt = t - last_t;
        if (dt < -0.5) session_reset(RESET_HARD, "game time went back (save loaded)");
        else if (dt > 0.5 && !T.paused) session_reset(RESET_SOFT, "long frame (hitch / loading)");
        else if (w - last_wall > 1.5 && !T.paused && dt < 0.5) session_reset(RESET_SOFT, "frames stalled");
    }
    last_t = t;
    last_wall = w;
    double x = T.place.position.x, y = T.place.position.y, z = T.place.position.z;
    if (have_pos) {
        double d = sqrt((x - px) * (x - px) + (y - py) * (y - py) + (z - pz) * (z - pz));
        if (d > 150 && d > 4 * fabsf(T.speed) * 0.5 + 80) {
            char b[96];
            snprintf(b, sizeof b, "position jumped %.0f m (teleport, quick travel, load)", d);
            session_reset(RESET_HARD, b);
        }
    }
    px = x; py = y; pz = z;
    have_pos = x != 0 || z != 0;
    if (was_engine >= 0 && T.engine_on && !was_engine) session_reset(RESET_SOFT, "engine started");
    was_engine = T.engine_on;
    if (was_trailer >= 0 && T.trailer != was_trailer) session_reset(RESET_LOAD, T.trailer ? "trailer coupled" : "trailer uncoupled");
    was_trailer = T.trailer;
}

void telemetry_frame()
{
    watch_continuity(sim_s());
    // Longitudinal acceleration from speed, low-pass ACCEL_LAG_S (0.5 s).
    static double last_t;
    static float last_v;
    double t = sim_s();
    double dt = t - last_t;
    if (dt > 0.0 && dt < 0.5) {
        float a = (float)((fabsf(T.speed) - last_v) / dt);
        float k = (float)(dt / (ACCEL_LAG_S + dt));
        T.accel += k * (a - T.accel);
        T.accel_fast += (float)(dt / (0.12 + dt)) * (a - T.accel_fast);
    }
    last_t = t;
    last_v = fabsf(T.speed);
}

void telemetry_register(const scs_telemetry_init_params_v101_t *p)
{
    p->register_for_event(SCS_TELEMETRY_EVENT_frame_start, ev_frame_start, NULL);
    p->register_for_event(SCS_TELEMETRY_EVENT_paused, ev_paused, NULL);
    p->register_for_event(SCS_TELEMETRY_EVENT_started, ev_paused, NULL);
    p->register_for_event(SCS_TELEMETRY_EVENT_configuration, ev_config, NULL);
    p->register_for_event(SCS_TELEMETRY_EVENT_gameplay, ev_gameplay, NULL);
#define REG(name, type, cb, ctx) p->register_for_channel(name, SCS_U32_NIL, type, SCS_TELEMETRY_CHANNEL_FLAG_none, cb, ctx)
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_engine_rpm, SCS_VALUE_TYPE_float, ch_float, &T.rpm);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, SCS_VALUE_TYPE_s32, ch_s32, &T.gear);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_displayed_gear, SCS_VALUE_TYPE_s32, ch_s32, &T.dgear);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, SCS_VALUE_TYPE_float, ch_float, &T.in_thr);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle, SCS_VALUE_TYPE_float, ch_float, &T.eff_thr);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, SCS_VALUE_TYPE_float, ch_float, &T.in_brake);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_effective_brake, SCS_VALUE_TYPE_float, ch_float, &T.eff_brake);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_input_clutch, SCS_VALUE_TYPE_float, ch_float, &T.in_clutch);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_effective_clutch, SCS_VALUE_TYPE_float, ch_float, &T.eff_clutch);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_speed, SCS_VALUE_TYPE_float, ch_float, &T.speed);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, SCS_VALUE_TYPE_float, ch_float, &T.cruise);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_motor_brake, SCS_VALUE_TYPE_bool, ch_bool, &T.motor_brake);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, SCS_VALUE_TYPE_bool, ch_bool, &T.parking_brake);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_light_brake, SCS_VALUE_TYPE_bool, ch_bool, &T.light_brake);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_brake_temperature, SCS_VALUE_TYPE_float, ch_float, &T.brake_temp);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_fuel, SCS_VALUE_TYPE_float, ch_float, &T.fuel);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled, SCS_VALUE_TYPE_bool, ch_bool, &T.engine_on);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_retarder_level, SCS_VALUE_TYPE_u32, ch_u32, &T.retarder);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure, SCS_VALUE_TYPE_float, ch_float, &T.air_psi);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure_warning, SCS_VALUE_TYPE_bool, ch_bool, &T.air_warning);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_speed_limit, SCS_VALUE_TYPE_float, ch_float, &T.speed_limit);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_VALUE_TYPE_dplacement, ch_place, NULL);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_acceleration, SCS_VALUE_TYPE_fvector, ch_acc, NULL);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_local_angular_velocity, SCS_VALUE_TYPE_fvector, ch_angvel, NULL);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, SCS_VALUE_TYPE_float, ch_float, &T.in_steer);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_lblinker, SCS_VALUE_TYPE_bool, ch_bool, &T.blink_l);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_rblinker, SCS_VALUE_TYPE_bool, ch_bool, &T.blink_r);
    REG("trailer.0.world.placement", SCS_VALUE_TYPE_dplacement, ch_tplace, NULL);
    // lift axles, lights, the clock and the wheels (axle.cpp / lights.cpp). A channel the game lacks just stays unregistered:
    // telemetry_seen() says which ones delivered, the modules log that once.
    static seen_dst d_lift = { &T.lift_axle, 1 }, d_tlift = { &T.trailer_lift_axle, 2 }, d_low = { &T.light_low, 4 }, d_high = { &T.light_high, 8 };
    int reg = 0, tot = 0;
#define REGC(name, type, cb, ctx) do { tot++; if (p->register_for_channel(name, SCS_U32_NIL, type, SCS_TELEMETRY_CHANNEL_FLAG_none, cb, ctx) == SCS_RESULT_ok) reg++; } while (0)
    REGC(SCS_TELEMETRY_TRUCK_CHANNEL_lift_axle, SCS_VALUE_TYPE_bool, ch_bool_seen, &d_lift);
    REGC(SCS_TELEMETRY_TRUCK_CHANNEL_trailer_lift_axle, SCS_VALUE_TYPE_bool, ch_bool_seen, &d_tlift);
    REGC(SCS_TELEMETRY_TRUCK_CHANNEL_light_low_beam, SCS_VALUE_TYPE_bool, ch_bool_seen, &d_low);
    REGC(SCS_TELEMETRY_TRUCK_CHANNEL_light_high_beam, SCS_VALUE_TYPE_bool, ch_bool_seen, &d_high);
    REGC(SCS_TELEMETRY_CHANNEL_game_time, SCS_VALUE_TYPE_u32, ch_gametime, NULL);
    T.game_min = -1;
    int wreg = 0;
    for (unsigned i = 0; i < 16; i++) {
        T.wheel_w[i] = T.wheel_lift[i] = NAN;
        if (p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_wheel_velocity, i, SCS_VALUE_TYPE_float, SCS_TELEMETRY_CHANNEL_FLAG_none, ch_wheel_w, NULL) == SCS_RESULT_ok) wreg++;
        if (p->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_wheel_lift, i, SCS_VALUE_TYPE_float, SCS_TELEMETRY_CHANNEL_FLAG_none, ch_wheel_lift, NULL) == SCS_RESULT_ok) wreg++;
    }
    for (unsigned i = 0; i < 12; i++) {
        T.t_wheel_lift[i] = NAN;
        if (p->register_for_channel("trailer.0.wheel.lift", i, SCS_VALUE_TYPE_float, SCS_TELEMETRY_CHANNEL_FLAG_none, ch_twheel_lift, NULL) == SCS_RESULT_ok) wreg++;
    }
#undef REGC
    logline("telemetry: lift axle, light and clock channels registered %d of %d; wheel channels (velocity, lift, trailer lift) %d of 44", reg, tot, wreg);
#undef REG
    p->register_for_channel("trailer.0.connected", SCS_U32_NIL, SCS_VALUE_TYPE_bool, SCS_TELEMETRY_CHANNEL_FLAG_none,
                            ch_bool, &T.trailer);
}
