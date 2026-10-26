// SDK entry points and the module registry. The game loads every DLL in
// bin/win_x64/plugins through the telemetry SDK; this one then finds and
// hooks the code it needs. Single player and SCS Convoy; not TruckersMP.
#include "suite.h"
#include "game.h"
#include "route.h"
#include "vehicle.h"
#include <string.h>

#include "common/scssdk_telemetry_common_configs.h"

extern module m_core, m_trueshift, m_cruise, m_lane, m_creep, m_decel, m_lowspeed;
module *g_modules[] = { &m_core, &m_trueshift, &m_cruise, &m_lane, &m_creep, &m_decel, &m_lowspeed };  // LowSpeed last: it reads what the others decided this frame
const char *g_truck_unsupported;
int g_n_modules = sizeof g_modules / sizeof g_modules[0];

// Build flavor, as a plain string in the DLL: tools/make_release.sh refuses a DLL without "dazdrive-flavor=public"
// (build.sh defines DAZDRIVE_FLAVOR=public only for a release build, which never has the local extras).
#ifndef DAZDRIVE_FLAVOR
#define DAZDRIVE_FLAVOR dev
#endif
#define DZ_STR_(x) #x
#define DZ_STR(x) DZ_STR_(x)
static const char k_flavor[] = "dazdrive-flavor=" DZ_STR(DAZDRIVE_FLAVOR);

static int g_started, g_inactive;  // g_inactive: nothing hooked, so nothing is registered either (the plugin sits idle)
volatile int g_rolling, g_cruise_smooth, g_crawl_on, g_pcc_on, g_pedal_mapped, g_lcx;
volatile float g_roll_boost_kmh;
volatile double g_rb_cancel_at = -1e9;
volatile float g_overspeed_cap = 5;
volatile float g_pcc_need_kmh = __builtin_nanf("");
volatile char g_dash_flash, g_dash_status;
double g_dash_flash_until;

static SCSAPI_VOID ev_frame_end(const scs_event_t, const void *const, const scs_context_t)
{
    telemetry_frame();
    if (L) route_update();
    vehicle_frame();
    input_poll();
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->frame) g_modules[m]->frame();
    incident_frame();
    DEV_NOTE(frame());
}

SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t *const params)
{
    if (version != SCS_TELEMETRY_VERSION_1_01) return SCS_RESULT_unsupported;
    const scs_telemetry_init_params_v101_t *p = (const scs_telemetry_init_params_v101_t *)params;

    if (!g_started) {
        g_started = 1;
        log_open();
        unsigned stamp = 0, size = 0;
        game_exe_stamp(&stamp, &size);
        // The telemetry version is the SDK's, not the game's patch level; the exe stamp and size identify the build.
        logline("%s %s (%s), game %s \"%s\", telemetry %u.%u, exe stamp %08x size %08x", SUITE_NAME, SUITE_VERSION, k_flavor,
                p->common.game_id, p->common.game_name, SCS_GET_MAJOR_VERSION(p->common.game_version),
                SCS_GET_MINOR_VERSION(p->common.game_version), stamp, size);
        config_load();
        DEV_INIT();
        const char *err;
        if (strcmp(p->common.game_id, "eut2")) {
            err = "ETS2 only in this release";
            logline("game %s is not supported in this release (ETS2 only): nothing patched, nothing registered", p->common.game_id);
            p->common.log(SCS_LOG_TYPE_warning, "[" SUITE_NAME "] this game is not supported yet (ETS2 only), plugin inactive");
        } else if ((err = game_discover(p->common.game_id)) != NULL) {
            logline("game code not recognized, nothing patched, nothing registered (%s)", err);
            p->common.log(SCS_LOG_TYPE_warning, "[" SUITE_NAME "] game build not supported, plugin inactive");
        }
        if (err) g_inactive = 1;
        else {
            logline("layout %s: chooser %p (%d callers), decide site %p, start-gear %p (%d callers), mass fn %p, "
                    "grade fn %p, game_actor +0x%x",
                    L->name, (void *)G.chooser_fn, G.n_choose, (void *)G.decide_site, (void *)G.start_fn, G.n_start,
                    (void *)G.mass_fn, (void *)G.grade_fn, G.game_actor_off);
            for (int m = 0; m < g_n_modules; m++)
                if (g_modules[m]->start) g_modules[m]->start();
            // One status line per module (level 0: it is the line a bug report starts from).
            for (int m = 0; m < g_n_modules; m++) {
                module *mod = g_modules[m];
                if (!mod->start) continue;
                const char *st = mod->status ? mod->status() : NULL;
                if (st) { logline("%s: %s", mod->title, st); continue; }
                for (int i = 0; i < mod->n_params; i++)
                    if (!strcmp(mod->params[i].key, "enabled") && *mod->params[i].val <= 0) st = "off (ini)";
                logline("%s: %s", mod->title, st ? st : "on (see the lines above for hooks)");
            }
        }
    }
    if (g_inactive) return SCS_RESULT_ok;  // inactive = no channels, no frame callback, no input thread
    telemetry_register(p);
    DEV_NOTE(register(p));
    p->register_for_event(SCS_TELEMETRY_EVENT_frame_end, ev_frame_end, NULL);
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_telemetry_shutdown(void)
{
    // The hooks stay in place (the DLL is pinned); modules only flush files.
    if (g_inactive) return;
    for (int m = 0; m < g_n_modules; m++)
        if (g_modules[m]->stop) g_modules[m]->stop();
    incident_stop();
    DEV_NOTE(stop());
}

BOOL WINAPI DllMain(HINSTANCE inst, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(inst);
        GetModuleFileNameA(inst, g_dir, sizeof g_dir);
        char *slash = strrchr(g_dir, '\\');
        if (slash) strcpy(slash + 1, SUITE_NAME "\\");
        else strcpy(g_dir, SUITE_NAME "\\");
        HMODULE dummy;
        // The hooks point into this DLL, so it must never unload.
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN | GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
                           (LPCWSTR)(void *)&DllMain, &dummy);
    }
    return TRUE;
}
