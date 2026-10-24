// SDK telemetry: channels and the truck configuration, shared by all modules.
#include "suite.h"
#include <math.h>
#include <string.h>
#include <math.h>
#include "route.h"

#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_common_channels.h"
#include "common/scssdk_telemetry_truck_common_channels.h"

telemetry T;

static SCSAPI_VOID ch_float(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(float *)ctx = v->value_float.value; }
static SCSAPI_VOID ch_s32(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(int *)ctx = v->value_s32.value; }
static SCSAPI_VOID ch_u32(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(unsigned *)ctx = v->value_u32.value; }
static SCSAPI_VOID ch_bool(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t ctx)
{ if (v) *(int *)ctx = v->value_bool.value != 0; }
static SCSAPI_VOID ch_place(const scs_string_t, const scs_u32_t, const scs_value_t *const v, const scs_context_t)
{ if (v) T.place = v->value_dplacement; }
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
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->reset) g_modules[m]->reset(level, why);
    DEV_NOTE(reset(level, why));
}

static SCSAPI_VOID ev_config(const scs_event_t, const void *const info, const scs_context_t)
{
    const scs_telemetry_configuration_t *c = (const scs_telemetry_configuration_t *)info;
    if (!strcmp(c->id, SCS_TELEMETRY_CONFIG_truck)) {
        // a new configuration replaces the old one completely (stale gears of a previous truck must not survive)
        memset(T.ratio, 0, sizeof T.ratio);
        memset(T.wheel_radius, 0, sizeof T.wheel_radius);
        memset(T.wheel_powered, 0, sizeof T.wheel_powered);
        T.fwd_gears = T.rev_gears = T.wheels = 0;
        T.shifter[0] = 0;
        T.retarder_steps = 0;
        for (const scs_named_value_t *a = c->attributes; a->name; a++) {
            const char *n = a->name;
            unsigned i = a->index;
            if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_gear_count)) T.fwd_gears = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_reverse_gear_count)) T.rev_gears = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_differential_ratio)) T.diff = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_rpm_limit)) T.rpm_limit = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_retarder_step_count)) T.retarder_steps = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count)) T.wheels = a->value.value_u32.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_ratio) && i < 32) T.ratio[i] = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_radius) && i < 16) T.wheel_radius[i] = a->value.value_float.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_powered) && i < 16) T.wheel_powered[i] = a->value.value_bool.value;
            else if (!strcmp(n, SCS_TELEMETRY_CONFIG_ATTRIBUTE_shifter_type) && a->value.type == SCS_VALUE_TYPE_string)
                snprintf(T.shifter, sizeof T.shifter, "%s", a->value.value_string.value);
        }
        logline("truck config: %d forward gears, diff %.3f, rpm limit %.0f, wheel radius %.3f m, retarder %u steps, "
                "shifter %s", T.fwd_gears, T.diff, T.rpm_limit, wheel_radius(), T.retarder_steps, T.shifter);
        // Same truck again (a reload of the config) is nothing; another truck resets everything we learned.
        double sig = T.fwd_gears * 1000.0 + T.diff * 7.0 + T.rpm_limit * 0.013 + wheel_radius() * 31.0 + T.wheels;
        for (int i = 0; i < T.fwd_gears && i < 32; i++) sig += T.ratio[i] * (i + 3);
        static double last_sig = -1;
        if (last_sig >= 0 && fabs(sig - last_sig) > 1e-6) session_reset(RESET_HARD, "another truck");
        last_sig = sig;
    } else if (!strcmp(c->id, SCS_TELEMETRY_CONFIG_job)) {
        for (const scs_named_value_t *a = c->attributes; a->name; a++)
            if (!strcmp(a->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_cargo_mass)) T.cargo_mass = a->value.value_float.value;
    }
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->config) g_modules[m]->config(c);
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
    // Longitudinal acceleration from speed, low-pass ~0.5 s.
    static double last_t;
    static float last_v;
    double t = sim_s();
    double dt = t - last_t;
    if (dt > 0.0 && dt < 0.5) {
        float a = (float)((fabsf(T.speed) - last_v) / dt);
        float k = (float)(dt / (0.5 + dt));
        T.accel += k * (a - T.accel);
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
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled, SCS_VALUE_TYPE_bool, ch_bool, &T.engine_on);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_retarder_level, SCS_VALUE_TYPE_u32, ch_u32, &T.retarder);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure, SCS_VALUE_TYPE_float, ch_float, &T.air_psi);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure_warning, SCS_VALUE_TYPE_bool, ch_bool, &T.air_warning);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_speed_limit, SCS_VALUE_TYPE_float, ch_float, &T.speed_limit);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_VALUE_TYPE_dplacement, ch_place, NULL);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_acceleration, SCS_VALUE_TYPE_fvector, ch_acc, NULL);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_local_angular_velocity, SCS_VALUE_TYPE_fvector, ch_angvel, NULL);
    REG(SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, SCS_VALUE_TYPE_float, ch_float, &T.in_steer);
#undef REG
    p->register_for_channel("trailer.0.connected", SCS_U32_NIL, SCS_VALUE_TYPE_bool, SCS_TELEMETRY_CHANNEL_FLAG_none,
                            ch_bool, &T.trailer);
}
