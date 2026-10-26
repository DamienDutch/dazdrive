// LowSpeed: the owner of everything below the hand-off to TrueShift (stopping, standing, pulling away, crawling).
//
// One state machine (the states and transitions of the low-speed design) computed every frame from the telemetry, the vehicle
// model and what the other modules expose (suite.h: creep_info_get, cruise_info_get, ts_*, g_rolling, g_crawl_on, g_lcx). It works
// out what the owner WOULD command into g_low; with trace=1 lowspeed.csv shows that next to what the other modules did.
//
// With owner=1 the stop and the standstill are the owner's (step 2 of the design): the game's stop threshold (TrueShift's hook asks
// low_stop_rpm), the gear preselected with the clutch open (the update's tail hook), the hold behind a lead (a brake wish through the
// actuator) and the one definition of "stopped" that Auto Hold and Stop&Go read. The pull-away, the crawl and the hand-off to TrueShift
// are still the old modules' (steps 3-5): in those states the owner posts nothing.
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
    { "owner", &p_owner, 0, "1 = the low-speed owner drives the stop and the standstill: the clutch opens where the gear the truck would start in sits near idle, that gear stays preselected, one hold behind a lead with the cruise on. The pull-away and the crawl stay with Creep and TrueShift. 0 = Creep, Cruise and TrueShift keep doing it all" },
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
static const float LS_STOP_MARGIN = 150;    // the clutch opens when the start gear would sit this far over idle (rpm), as the stop threshold does
static const float LS_ASSIST_REQ = 0.15f;   // the brake request of the stop assist, a placeholder until it is measured (request unit 0..1)
static const float LS_HOLD_REQ = 0.6f;      // the brake request of a hold when Auto Hold does not say (its own default)
static const float LS_SG_HOLD_REQ = 0.2f;   // the brake request of the hold behind a lead with the cruise on (the ACC's own standing hold)
static const float LS_ARM_RPM = 100;        // Stop&Go's hold is set when the driveline is this near the stop threshold (rpm), before the box can go to neutral
static const float LS_R_BRAKE = 0.01f;      // the game puts R in on a brake above this at a standstill (its own rule)

// What one frame sees: telemetry, the vehicle model and the other modules' states, gathered in one place (the sim sets the same
// fields through the real accessors).
struct low_in {
    float dt, kmh, thr, brake, eff_brake, rpm, idle, accel, clutch;
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
static int g_latch;          // the clutch is commanded open (STOPPING and the standstill); only the driver's pedal ends it
static int g_assist;         // stop assist: 0 not asked, 1 running, 2 cancelled by the driver or not wanted this stop
static int g_pre_gear;       // the gear preselected with the open clutch
static float g_open_kmh;     // the speed where the start gear sits at the clutch-open rpm now
static int g_dis_clutch, g_dis_gear;   // frames where what it would command differs from what the truck does (clutch open or closed, gear)
// What the owner drives with owner=1: set once per frame, read by the game's threads (the stop hook, the update's tail) and the other
// modules through the accessors at the bottom.
static volatile int g_o_on;          // the owner has the stop and the standstill (owner=1 and not in GAME)
static volatile int g_o_stopped;     // the standstill states
static volatile int g_o_zone;        // Stop&Go's hold is due
static volatile int g_o_pre;         // the gear to hold preselected in the box's gap (0 = none)
static volatile int g_o_back;        // hand the at-rest neutral back to the game once (the driver brakes after the pedals were free: R)
static volatile int g_o_start;       // the start gear the stop hook converts (0 = unknown)
static volatile int g_o_crawl, g_o_cruise;   // crawl switched on, cruise on: the hook leaves the stock threshold then
static int g_rest_free;              // at a standstill with the brake pedal free since the stop: a brake press then is the driver asking for R
static int g_o_holding;              // we wrote the preselect and hold it
static float g_o_hold_wish;          // the brake wish posted to the actuator
static void *g_o_self;               // the player's gearbox (the tail hook runs for every vehicle's)
static int g_tail_ok;

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
    return in.brake > LS_BRAKE_HOLD || in.cu.hold == 1 || in.cu.sg_hold || in.act_req > 0.01f || in.parking || in.hill || g_assist == 1;
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
        if (in.kmh >= in.top_kmh && !g_latch) { s = LOW_HANDED; why = "above the top of the low-speed range"; }   // an open clutch is ours until the pedal
        else if (in.thr > LS_PEDAL_GO) {
            if (g_latch || in.clutch > 0.5f || in.gear == 0) { s = LOW_LAUNCH; why = "pedal before the stop with the clutch open"; }
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
    g_o_self = self;
    in->dt = dt;
    in->kmh = fabsf(T.speed) * 3.6f;
    in->thr = T.in_thr;
    in->brake = T.in_brake;
    in->eff_brake = T.eff_brake;
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

// The speed where the start gear sits at the rpm the clutch opens at. Like the stop threshold: idle + 150 rpm and the game's own term
// for the brake (brake squared times idle); with the pedals free the box opens by itself only with crawl off and no cruise, else at the
// stock 0.8 idle.
static float open_margin(float brake, float idle, int crawl, int cruise)
{
    brake = fminf(brake, 1.0f);
    return !cruise && (brake >= LS_BRAKE_ON || !crawl) ? LS_STOP_MARGIN + brake * brake * idle : -0.2f * idle;
}
static float open_point_kmh(const low_in &in)
{
    float brake = fminf(fmaxf(in.brake, in.eff_brake), 1.0f);
    return vehicle_kmh_idle(in.start_gear, in.idle, open_margin(brake, in.idle, in.crawl_toggle, in.cruise));
}

// What the owner would command in this state. The clutch is a latch: once it is open (the start gear sat at its idle point, or the
// box had it open) it stays open whatever the start gear does after that: a lower start gear is too late to matter, a higher one has its
// idle point at a higher speed, so we are already under it and the clutch opens at once. Only the pedal ends it.
static void low_command(const low_in &in)
{
    static int was;
    int st = g_low.state, entered = st != was;
    int was_rest = was == LOW_STOPPED_HELD || was == LOW_STOPPED_FREE;
    was = st;
    float pedal = fmaxf(in.thr, in.cu.feed);
    g_open_kmh = 0;
    if (st != LOW_STOPPING && st != LOW_STOPPED_HELD && st != LOW_STOPPED_FREE) { g_latch = 0; g_assist = 0; }
    g_low.would_gear = 0; g_low.would_clutch = NAN; g_low.would_brake = 0; g_low.would_thr = NAN;
    switch (st) {
    case LOW_STOPPING: {
        g_open_kmh = open_point_kmh(in);
        if (!g_latch && (in.kmh <= g_open_kmh || in.gear == 0)) {
            g_latch = 1;
            g_pre_gear = in.start_gear;
            // the box opened the clutch by itself with both pedals free: it finishes the stop (the pedal and the brake always win)
            g_assist = in.thr < LS_PEDAL_FREE && in.brake < LS_BRAKE_ON && !in.cruise && !in.crawl_toggle ? 1 : 2;
            logv(2, "LowSpeed: clutch open at %.1f km/h (start gear %d, its idle point %.1f km/h, brake %.2f)%s", in.kmh, in.start_gear, g_open_kmh,
                 g_assist == 1 ? ", stop assist" : "");
        }
        if (g_assist == 1 && (in.thr >= LS_PEDAL_FREE || in.brake >= LS_BRAKE_ON || in.cruise)) g_assist = 2;
        g_low.would_gear = g_latch ? g_pre_gear : in.gear;
        g_low.would_clutch = g_latch ? 1.0f : 0.0f;
        g_low.would_brake = g_assist == 1 ? LS_ASSIST_REQ : 0;
        g_low.would_thr = 0;
        break;
    }
    case LOW_STOPPED_HELD: case LOW_STOPPED_FREE: {
        g_latch = 1;
        // held by us (Auto Hold, Stop&Go, the stop assist) the owner asks for the hold; the driver's brake, the parking brake and the game's
        // hill hold hold without a request of ours. Behind a lead with the cruise on (Auto Hold stands aside then) the hold is the light one
        // the ACC used to post at a standstill.
        int by_us = in.cu.hold == 1 || in.cu.sg_hold || g_assist == 1;
        if (!was_rest) g_pre_gear = in.start_gear;   // the gear preselected at rest is the start gear at the moment of the stop
        g_low.would_gear = g_pre_gear;
        g_low.would_clutch = 1;
        g_low.would_brake = st == LOW_STOPPED_HELD && by_us ? (in.cu.hold == 1 ? (in.cu.hold_req > 0 ? in.cu.hold_req : LS_HOLD_REQ) : in.cu.sg_hold ? LS_SG_HOLD_REQ : LS_HOLD_REQ) : 0;
        g_low.would_thr = 0;
        break;
    }
    case LOW_LAUNCH:
        if (entered) g_pre_gear = in.kmh < LS_MOVE_KMH || in.gear < 1 ? in.start_gear : in.gear;   // the start gear for this pedal, held until the clutch is locked
        g_low.would_gear = g_pre_gear;
        g_low.would_clutch = in.cr.launch ? in.cr.launch_clutch : NAN;
        g_low.would_thr = pedal;
        break;
    case LOW_ROLLING_LOW:
        g_low.would_gear = in.gear;
        g_low.would_clutch = 0;
        g_low.would_thr = pedal;
        break;
    case LOW_CRAWL: case LOW_MANEUVER:
        g_low.would_gear = in.gear;
        g_low.would_clutch = in.cr.floor;
        g_low.would_brake = st == LOW_CRAWL ? in.cr.brake : 0;
        g_low.would_thr = st == LOW_CRAWL ? in.cr.add_thr : in.thr;
        break;
    default:   // GAME, HANDED, REVERSE: the game's and TrueShift's own: no wish
        break;
    }
    // the shadow's account: how often the truck does something else than the owner would
    if (g_low.would_clutch == g_low.would_clutch && (st == LOW_STOPPING || st == LOW_STOPPED_HELD || st == LOW_STOPPED_FREE) && (g_low.would_clutch > 0.5f) != (in.clutch > 0.5f)) g_dis_clutch++;
    if (g_low.would_gear > 0 && (g_latch || st == LOW_LAUNCH) && g_low.would_gear != in.gear) g_dis_gear++;
}

// ---------------------------------------------------------------- what the owner drives (owner=1)

// The game's stop threshold (TrueShift's hook, in rpm of the gear that is in): the clutch opens at the open point of the owner, the
// speed where the start gear sits at idle + 150 rpm + the brake term, converted to that gear's rpm (less the brake term, which the game adds). In the start gear that is the old
// smooth stop's number; in a higher gear it is lower, so the box does not go to neutral long before the start gear would be at idle.
// Never under the stock 0.8 idle (the engine would stall on the clutch). Stock with a pedal and with the cruise on (in a jam the ACC rolls
// at walking pace, and Stop&Go's hold has to be set before the box can go to neutral: low_sg_zone arms it LS_ARM_RPM above this stock
// number). With crawl on it is Creep's gear and brake rule: idle + 150 on the brake, else stock. The latch: once the clutch is open the threshold stays above
// idle + 150, so a start gear that gets lower afterwards (a grade change at the last metre) cannot close it again; a higher start gear has
// its idle point at a higher speed and opens it at once, by the same arithmetic.
float low_stop_rpm(float idle, float thr, float brake, int gear)
{
    float stock = 0.8f * idle;
    if (!g_o_on || thr > 0.01f || !(idle > 200 && idle < 2000) || g_o_start < 1 || gear < 1 || g_o_cruise) return stock;
    if (g_o_crawl) return brake >= LS_BRAKE_ON ? fmaxf(stock, idle + LS_STOP_MARGIN) : stock;   // Creep's: the crawl gear is not the start gear
    float v = vehicle_kmh_idle(g_o_start, idle, open_margin(brake, idle, g_o_crawl, g_o_cruise)) / 3.6f;
    float b = fminf(brake, 1.0f);
    float r = fmaxf(stock, vehicle_rpm_at(gear, v) - b * b * idle);   // the game adds its brake term on top of the hook's value: it is in the open point already
    if (g_latch) r = fmaxf(r, idle + LS_STOP_MARGIN);
    return r;
}
int low_owner_wanted() { return p_owner > 0; }
int low_owner_on() { return g_o_on; }
int low_stopped() { return g_o_on && g_o_stopped; }
int low_sg_zone() { return g_o_on && g_o_zone; }

// The update's tail (the game's at-rest rule and stop rule have run, the generic gearbox is next): with the clutch open and nothing in
// gear, the start gear is the target, so the box waits in its gap for the pedal (or Stop&Go's fed throttle) with the gear it will start in,
// not in neutral. The at-rest rule sets neutral every update; written back here it never reaches the generic gearbox (as Creep's tail does).
// A brake press at a standstill after the pedals were free is the game's R: the at-rest neutral goes back to it once.
static void low_tail(void *self)
{
    if (!g_o_on || !self || self != g_o_self) return;
    if (g_o_back) {
        g_o_back = 0;
        gb_set_target(self, 0);
        gb_rest_state(self, 3, 1);
        return;
    }
    int want = g_o_pre;
    if (want < 1 || gb_gear(self) != 0) return;
    if (gb_target(self) != want) gb_set_target(self, want);
    gb_rest_state(self, 4, 0);
}

// Once per frame, after the state and its command: what the hook, the tail and the other modules read, and the brake wish.
static void low_publish(const low_in &in)
{
    g_o_start = in.start_gear;
    g_o_crawl = in.crawl_toggle || in.cr.crawl;
    g_o_cruise = in.cruise;
    int st = g_low.state, rest = in.kmh < LS_MOVE_KMH;
    g_o_on = p_owner > 0 && st != LOW_GAME;
    g_o_stopped = st == LOW_STOPPED_HELD || st == LOW_STOPPED_FREE;
    // Stop&Go: the box goes to neutral at the stop threshold and the game then cancels the cruise unless the hold is set. Due when the
    // driveline is within LS_ARM_RPM of that threshold, or the box already is in neutral below the top of the owner's world, or the truck stands.
    int zone = g_o_stopped;
    if (g_o_on && !zone && in.gear == 0 && in.kmh < in.top_kmh) zone = 1;
    float bk = fminf(fmaxf(in.brake, in.eff_brake), 1.0f);
    if (g_o_on && !zone && in.gear > 0) zone = in.kmh <= vehicle_kmh_at(in.gear, low_stop_rpm(in.idle, 0, bk, in.gear) + bk * bk * in.idle + LS_ARM_RPM);   // the game adds its brake term
    g_o_zone = zone;
    // the preselect
    if (!rest) g_rest_free = 0;
    else if (in.brake <= LS_R_BRAKE && in.thr < LS_PEDAL_FREE) g_rest_free = 1;
    int pre = 0;
    if (g_o_on && !g_o_crawl && !in.rolling && in.gear == 0 && ((st == LOW_STOPPING && g_latch) || g_o_stopped)) pre = g_pre_gear;
    g_o_back = 0;
    if (pre && rest && g_rest_free && in.brake > LS_R_BRAKE) { pre = 0; g_o_back = g_o_holding; }
    g_o_holding = pre != 0;
    g_o_pre = g_tail_ok ? pre : 0;
    // the hold behind a lead
    g_o_hold_wish = g_o_on && st == LOW_STOPPED_HELD && in.cu.sg_hold && in.cu.hold != 1 ? LS_SG_HOLD_REQ : 0;
    if (p_owner > 0) act_brake(ACT_LOW, g_o_hold_wish);
}

static void ls_frame()
{
    if (p_trace <= 0 && p_owner <= 0) return;
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
    low_command(in);
    if (p_owner > 0) low_publish(in);
    static double csv_at;
    if (g_csv && now - csv_at >= 0.1) {
        csv_at = now;
        fprintf(g_csv, "%.2f,%s,%.2f,%d,%.3f,%.3f,%.0f,%.0f,%.3f,%d,%.2f,%d,%.2f,%d,%d,%.2f,%.3f,%.3f,%d,%d,%d,%d,%.2f,%.2f,%.3f,%d,%d,%.1f,%s\n", wall_s(), low_state_name(g_low.state),
                in.kmh, in.gear, in.thr, in.brake, in.rpm, in.idle, in.clutch, in.start_gear, in.top_kmh, g_low.would_gear, g_open_kmh, g_latch, g_assist,
                g_low.would_clutch, g_low.would_brake, g_low.would_thr, in.cr.crawl, in.cr.launch, in.cu.hold, in.cu.sg_hold, in.cu.feed, in.accel, in.act_req,
                g_dis_clutch, g_dis_gear, g_low.since_s, g_low.reason);
    }
    static double flushed;
    if (g_csv && wall_s() - flushed > 2) { flushed = wall_s(); fflush(g_csv); csv_cap(&g_csv, "lowspeed.csv"); }
}

// Continuity broke (pause menu, hitch, other truck, teleport, ...): no timer or latch carries over; the state is derived again.
static void ls_reset(int level, const char *why)
{
    g_fresh = 1;
    g_latch = g_assist = 0;   // derived again too: a stop that is already under the start gear's idle point opens the clutch at once
    g_o_pre = g_o_back = g_o_holding = g_rest_free = 0;
    if (p_owner > 0) act_brake(ACT_LOW, 0);
    g_lock_s = g_rel_s = 0;
    g_low.since_s = 0;
}

static char g_status[120] = "off";
static const char *ls_status() { return g_status; }

static void ls_start()
{
    if (p_owner > 0) {
        g_tail_ok = G.tail_site && game_patch_tail(G.tail_site, (void *)low_tail);
        logline("lowspeed owner: drives the stop and the standstill (stop threshold%s, preselected start gear %s, hold behind a lead, Auto Hold and Stop&Go arming); %s",
                ts_stop_hooked() ? " in TrueShift's hook" : " NOT hooked: the game's own rule opens the clutch", g_tail_ok ? "in the update's tail" : "NOT hooked: the box waits in neutral",
                "pull-away, crawl and hand-off stay with Creep and TrueShift");
    }
    if (p_trace <= 0) {
        if (p_owner > 0) snprintf(g_status, sizeof g_status, "owner: drives the stop and the standstill (stop threshold, preselected gear, hold behind a lead)");
        else snprintf(g_status, sizeof g_status, "off (ini)");
        return;
    }
    g_csv = open_out("lowspeed.csv", "w");
    if (g_csv) fputs("t,state,kmh,gear,thr,brake,rpm,idle,clutch,start_gear,top_kmh,would_gear,open_kmh,latch,assist,would_clutch,would_brake,would_thr,crawl,launch,ah,sg_hold,feed,accel,act_req,dis_clutch,dis_gear,since_s,reason\n", g_csv);
    snprintf(g_status, sizeof g_status, "%s%s", p_owner > 0 ? "owner: drives the stop and the standstill, trace on" : "shadow: watches and decides, applies nothing", g_csv ? "" : " (no csv)");
}

static void ls_stop()
{
    if (g_csv) fflush(g_csv);
}

module m_lowspeed = { "lowspeed", "LowSpeed", g_params, sizeof g_params / sizeof g_params[0], ls_start, ls_frame, NULL, ls_stop, ls_reset, ls_status };
