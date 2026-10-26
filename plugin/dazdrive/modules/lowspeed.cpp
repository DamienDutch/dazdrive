// LowSpeed: the owner of everything below the hand-off to TrueShift (stopping, standing, pulling away, crawling), as a SHADOW.
//
// One state machine (the states and transitions of the low-speed design) computed every frame from the telemetry, the vehicle
// model and what the other modules expose (suite.h: creep_info_get, cruise_info_get, ts_*, g_rolling, g_crawl_on, g_lcx). It works
// out what the owner WOULD command into g_low and applies nothing: no act_* call, no gear, clutch or brake write, no hook. The
// modules keep driving as they always did; lowspeed.csv shows next to them what the owner would have done.
//
// No fixed km/h edges: the stop point and the top of the owner's world come from the vehicle model (the gear the truck would start
// in, where it sits at idle and where its shift window ends). Only the near-zero measurement thresholds are fixed.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/vehicle.h"
#include <math.h>
#include <string.h>
#include <windows.h>

static float p_owner, p_trace;

static param g_params[] = {
    { "owner", &p_owner, 0, "1 = the low-speed owner drives the stop, the standstill and the pull-away (not wired yet: nothing happens). 0 = Creep, Cruise and TrueShift keep doing it" },
    { "trace", &p_trace, 0, "1 = the low-speed owner runs beside the other modules without driving anything: lowspeed.csv (10 per second) and one log line per state change (needs the log at its highest level)" },
};

low_owner g_low = { LOW_GAME, 0, NAN, 0, NAN, 0, "not started" };

static const char *const k_state[LOW_N] = { "GAME", "HANDED", "STOPPING", "STOPPED_HELD", "STOPPED_FREE", "CRAWL", "MANEUVER", "LAUNCH", "ROLLING_LOW", "REVERSE" };
const char *low_state_name(int s) { return s >= 0 && s < LOW_N ? k_state[s] : "?"; }

// Near-zero measurement thresholds (noise, not physics) and the pedal and timer levels of the transitions.
static const float LS_STOP_KMH = 0.3f;      // stopped below this (km/h) ...
static const float LS_MOVE_KMH = 0.5f;      // ... and moving again above this
static const float LS_PEDAL_FREE = 0.02f;   // the pedal is free below this
static const float LS_PEDAL_GO = 0.06f;     // the pedal asks for a launch above this
static const float LS_BRAKE_ON = 0.05f;     // braking above this
static const float LS_BRAKE_HOLD = 0.10f;   // the driver holds the truck with the brake above this
static const float LS_BRAKE_LAUNCH = 0.20f; // a launch ends on a brake above this
static const float LS_DECEL = 0.10f;        // decelerating faster than this (m/s2, filtered)
static const float LS_SYNC_RPM = 80;        // the clutch is locked when the engine is within this of the driveline (rpm) ...
static const float LS_LOCKED = 0.02f;       // ... and the clutch value is under this
static const float LS_LAUNCH_END_S = 0.15f; // locked this long ends a launch
static const float LS_HANDOFF_S = 0.5f;     // locked this long, with the rpm below, is the hand-off to TrueShift
static const float LS_HANDOFF_RPM = 150;    // the hand-off needs the engine at least this far over idle (rpm)
static const float LS_PEDAL_REL_S = 0.4f;   // a pedal released this long ends a launch

// What one frame sees: telemetry, the vehicle model and the other modules' states, gathered in one place (the sim sets the same
// fields through the real accessors).
struct low_in {
    float dt, kmh, thr, brake, rpm, idle, accel, clutch;
    int gear, cruise, rolling, crawl_toggle, parking, hill;
    int start_gear;          // ts_start_gear for a gentle pull-away, 0 = unknown
    float top_kmh;           // the top of the owner's world: where the start gear reaches the end of the low-gear shift window
    int locked;              // the clutch is closed and the engine follows the driveline
    creep_info cr;
    cruise_info cu;
    float act_req;           // the brake request the actuator holds
    const char *game_why;    // NULL = the owner can work; else why it stands aside (state GAME)
};

static FILE *g_csv;
static int g_fresh = 1;      // the state has to be derived from the telemetry alone (start, after a reset)
static double g_last;        // sim_s() of the last frame
static float g_lock_s;       // how long the clutch has been locked
static float g_rel_s;        // how long the pedal has been released during a launch
static int g_n_trans;

static void to_state(int s, const char *why)
{
    if (s == g_low.state) return;
    logv(2, "LowSpeed: %s -> %s (%s)", low_state_name(g_low.state), low_state_name(s), why);
    g_low.state = s;
    g_low.since_s = 0;
    g_low.reason = why;
    g_n_trans++;
}

// The state of a moment from the telemetry alone: what a state that just ended falls back to, and the first frame after a start or a
// reset (nothing is remembered then: no timer, no latch).
static int rolling_state(const low_in &in)
{
    return g_lock_s >= LS_HANDOFF_S && in.rpm >= in.idle + LS_HANDOFF_RPM ? LOW_HANDED : LOW_ROLLING_LOW;
}

static int held_now(const low_in &in)
{
    return in.brake > LS_BRAKE_HOLD || in.cu.hold == 1 || in.cu.sg_hold || in.act_req > 0.01f || in.parking || in.hill;
}

static int settle(const low_in &in, const char **why)
{
    int stopped = in.kmh < (g_low.state == LOW_STOPPED_HELD || g_low.state == LOW_STOPPED_FREE ? LS_MOVE_KMH : LS_STOP_KMH);
    if (stopped) {
        if (in.thr > LS_PEDAL_GO || in.cu.feed > 0.02f) { *why = "pedal at a standstill"; return LOW_LAUNCH; }
        *why = "standstill";
        return held_now(in) ? LOW_STOPPED_HELD : LOW_STOPPED_FREE;
    }
    if (in.kmh >= in.top_kmh) { *why = "above the top of the low-speed range"; return LOW_HANDED; }
    if (in.locked) { *why = "clutch locked"; return rolling_state(in); }
    if (in.thr > LS_PEDAL_GO) { *why = "pedal and clutch not locked"; return LOW_LAUNCH; }
    *why = "pedal free and clutch not locked";
    return LOW_STOPPING;
}

static void low_step(const low_in &in)
{
    const char *why = "";
    int prev = g_low.state;
    if (in.game_why) { to_state(LOW_GAME, in.game_why); return; }
    if (g_fresh) {   // no history: derive it
        g_fresh = 0;
        g_lock_s = in.locked ? LS_HANDOFF_S : 0;   // a truck that is rolling locked has been so for a while
        g_rel_s = 0;
        int s = in.rolling ? LOW_HANDED : in.gear < 0 ? LOW_REVERSE : in.cr.crawl ? (in.thr >= LS_PEDAL_FREE ? LOW_MANEUVER : LOW_CRAWL)
              : in.cr.launch ? LOW_LAUNCH : settle(in, &why);
        to_state(s, in.rolling ? "eco-roll" : in.gear < 0 ? "reverse gear" : in.cr.crawl ? "crawl on" : in.cr.launch ? "smooth start running" : why);
        return;
    }
    // the sources that decide whatever the last state was
    if (in.rolling) { to_state(LOW_HANDED, "eco-roll (TrueShift's neutral)"); return; }
    if (in.gear < 0) { to_state(LOW_REVERSE, "reverse gear"); return; }
    if (in.cr.crawl && in.brake < in.cr.brake_off) {
        to_state(in.thr >= LS_PEDAL_FREE ? LOW_MANEUVER : LOW_CRAWL, in.thr >= LS_PEDAL_FREE ? "crawl on with the pedal" : "crawl on and pedals free");
        return;
    }
    if (in.cr.launch) {
        if (g_low.state != LOW_LAUNCH) g_rel_s = 0;
        to_state(LOW_LAUNCH, g_lcx == 2 ? "launch control" : "smooth start running");
        return;
    }
    if (g_lcx == 1) { to_state(LOW_STOPPED_HELD, "launch control armed"); return; }
    int stopped = in.kmh < (prev == LOW_STOPPED_HELD || prev == LOW_STOPPED_FREE ? LS_MOVE_KMH : LS_STOP_KMH);
    int s = prev;
    switch (prev) {
    case LOW_GAME: case LOW_REVERSE: case LOW_CRAWL: case LOW_MANEUVER:
        s = settle(in, &why);   // the owner just got the truck back, or the crawl or the reverse ended
        break;
    case LOW_HANDED:
        if (in.kmh < in.top_kmh && in.thr < LS_PEDAL_FREE && (in.brake > LS_BRAKE_ON || in.accel < -LS_DECEL)) { s = LOW_STOPPING; why = "pedal free and slowing"; }
        break;
    case LOW_ROLLING_LOW:
        if (in.thr < LS_PEDAL_FREE && (in.brake > LS_BRAKE_ON || in.accel < -LS_DECEL || in.clutch > 0.5f)) { s = LOW_STOPPING; why = "pedal free and slowing or clutch open"; }
        else if (g_lock_s >= LS_HANDOFF_S && in.rpm >= in.idle + LS_HANDOFF_RPM) { s = LOW_HANDED; why = "hand-off: clutch locked and engine over idle"; }
        break;
    case LOW_STOPPING:
        if (in.kmh >= in.top_kmh) { s = LOW_HANDED; why = "above the top of the low-speed range"; }
        else if (in.thr > LS_PEDAL_GO) {
            if (in.clutch > 0.5f || in.gear == 0) { s = LOW_LAUNCH; why = "pedal before the stop with the clutch open"; }
            else { s = rolling_state(in); why = "pedal before the stop with the clutch still closed"; }
        } else if (stopped) { s = held_now(in) ? LOW_STOPPED_HELD : LOW_STOPPED_FREE; why = "standstill"; }
        break;
    case LOW_STOPPED_HELD: case LOW_STOPPED_FREE:
        if (in.thr > LS_PEDAL_GO || in.cu.feed > 0.02f) { s = LOW_LAUNCH; why = in.thr > LS_PEDAL_GO ? "pedal" : "Stop&Go drives off"; g_rel_s = 0; }
        else if (!stopped) { s = LOW_STOPPING; why = "rolling without the pedal"; }
        else { s = held_now(in) ? LOW_STOPPED_HELD : LOW_STOPPED_FREE; why = held_now(in) ? "held" : "nothing holds"; }
        break;
    case LOW_LAUNCH:
        g_rel_s = in.thr < LS_PEDAL_FREE && in.cu.feed < 0.02f ? g_rel_s + in.dt : 0;
        if (in.brake > LS_BRAKE_LAUNCH) { s = stopped ? LOW_STOPPED_HELD : LOW_STOPPING; why = "brake during the launch"; }
        else if (g_lock_s >= LS_LAUNCH_END_S && in.locked) { s = rolling_state(in); why = "clutch locked"; }
        else if (g_rel_s > LS_PEDAL_REL_S) { s = settle(in, &why); if (s == LOW_LAUNCH) s = LOW_STOPPING; why = "pedal released"; }
        break;
    }
    to_state(s, why);
}

static const char *fallback(const low_in &in)
{
    if (g_truck_unsupported) return "truck not supported";
    if (!T.engine_on) return "engine off";
    if (T.shifter[0] && strcmp(T.shifter, "automatic") && strcmp(T.shifter, "arcade")) return "not an automatic";
    if (V.electric) return "electric motor";
    if (!V.eng_ok) return "no engine data";
    if (!(in.idle > 0)) return "idle rpm unknown";
    if (in.start_gear < 1) return "no start gear";
    if (!(in.top_kmh > 0)) return "no shift window";
    return NULL;
}

static void gather(low_in *in, float dt)
{
    memset(in, 0, sizeof *in);
    void *self = game_gearbox();
    in->dt = dt;
    in->kmh = fabsf(T.speed) * 3.6f;
    in->thr = T.in_thr;
    in->brake = T.in_brake;
    in->rpm = T.rpm;
    in->idle = ts_idle_rpm();
    in->accel = T.accel;
    in->clutch = clutch_seen();
    in->gear = self ? gb_gear(self) : T.gear;
    in->cruise = T.cruise > 0.1f;
    in->rolling = g_rolling;
    in->crawl_toggle = g_crawl_on;
    in->parking = T.parking_brake;
    creep_info_get(&in->cr);
    cruise_info_get(&in->cu);
    in->act_req = act_request();
    in->start_gear = ts_start_gear(fmaxf(T.in_thr, 0.3f));
    // the top of the owner's world: where the start gear reaches the end of the game's low-gear shift window
    float up = V.eng.range[R_LOW][1];
    in->top_kmh = in->start_gear > 0 && up > in->idle + LS_HANDOFF_RPM ? vehicle_kmh_at(in->start_gear, up) : 0;
    float lockrpm = in->gear > 0 ? vehicle_rpm_at(in->gear, T.speed) : 0;
    in->locked = in->gear > 0 && in->clutch < LS_LOCKED && fabsf(T.rpm - lockrpm) < LS_SYNC_RPM;
    char *ga = game_actor_safe();   // the game's own hill hold (1.60 layout: only where the actuator hooks are in)
    in->hill = ga && act_clutch_ok() && mem_readable(ga + 0x1d00, 4) && *(float *)(ga + 0x1d00) > 0;
    in->game_why = !self ? "no gearbox" : !act_hooked() ? "no actuator hook" : fallback(*in);
}

static void ls_frame()
{
    if (p_trace <= 0) return;
    double now = sim_s();
    float dt = (float)(now - g_last);
    g_last = now;
    if (dt <= 0) return;                                    // paused: nothing moves
    if (dt > 0.5f) g_fresh = 1;                             // a hitch: derive again
    low_in in;
    gather(&in, dt > 0.5f ? 0 : dt);
    g_lock_s = in.locked ? g_lock_s + in.dt : 0;
    g_low.since_s += in.dt;
    low_step(in);
    // what the owner would command (pass 1: the gear that is in; the clutch and the brake follow the state)
    g_low.would_gear = in.gear;
    g_low.would_clutch = NAN;
    g_low.would_brake = 0;
    g_low.would_thr = NAN;
    static double csv_at;
    if (g_csv && now - csv_at >= 0.1) {
        csv_at = now;
        fprintf(g_csv, "%.2f,%s,%.2f,%d,%.3f,%.3f,%.0f,%.0f,%.3f,%d,%.2f,%.2f,%.3f,%.3f,%d,%d,%d,%d,%.2f,%.2f,%.3f,%.1f,%s\n", wall_s(), low_state_name(g_low.state), in.kmh, in.gear,
                in.thr, in.brake, in.rpm, in.idle, in.clutch, in.start_gear, in.top_kmh, g_low.would_clutch, g_low.would_brake, g_low.would_thr, in.cr.crawl, in.cr.launch,
                in.cu.hold, in.cu.sg_hold, in.cu.feed, in.accel, in.act_req, g_low.since_s, g_low.reason);
    }
    static double flushed;
    if (g_csv && wall_s() - flushed > 2) { flushed = wall_s(); fflush(g_csv); csv_cap(&g_csv, "lowspeed.csv"); }
}

// Continuity broke (pause menu, hitch, other truck, teleport, ...): no timer or latch carries over; the state is derived again.
static void ls_reset(int level, const char *why)
{
    g_fresh = 1;
    g_low.since_s = 0;
}

static char g_status[120] = "off";
static const char *ls_status() { return g_status; }

static void ls_start()
{
    if (p_owner > 0) logline("lowspeed owner: not wired yet");
    if (p_trace <= 0) { snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    g_csv = open_out("lowspeed.csv", "w");
    if (g_csv) fputs("t,state,kmh,gear,thr,brake,rpm,idle,clutch,start_gear,top_kmh,would_clutch,would_brake,would_thr,crawl,launch,ah,sg_hold,feed,accel,act_req,since_s,reason\n", g_csv);
    snprintf(g_status, sizeof g_status, "shadow: watches and decides, applies nothing%s", g_csv ? "" : " (no csv)");
}

static void ls_stop()
{
    if (g_csv) fflush(g_csv);
}

module m_lowspeed = { "lowspeed", "LowSpeed", g_params, sizeof g_params / sizeof g_params[0], ls_start, ls_frame, NULL, ls_stop, ls_reset, ls_status };
