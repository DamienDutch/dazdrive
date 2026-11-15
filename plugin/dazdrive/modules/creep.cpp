// Creep: the bottom of the pull-away (03-11, owner decision: "crawl should simply be part of regular pulling away").
//
// Like a Volvo/Eaton/ZF AMT: at a standstill and at low speed the pedal asks the torque pedal's demand (a force, core/wish), and below
// the speed at which the start gear runs at idle the clutch slips and delivers that force: a light pedal is a gentle crawl, a full
// pedal the full launch at once. With both pedals free, ONE setting decides what the truck does: [creep] pedals_free = hold (the truck
// is held still with a light brake, the owner's choice) or creep (the truck creeps at creep_kmh through the slipping clutch; the brake
// pedal lowers it). The gearbox-mode key (toggle_mode_key) flips the setting for the session.
//
// How (worked out by static RE; the crawl is driven daily now):
//  - The drivetrain gets its clutch from one copy (inner+0x142c -> drive+0x330, 0x1408c82cf). We raise
//    that copy to a floor for the player's truck only (the actuator's clutch hook, core/actuate.cpp: act_clutch).
//    The gearbox sequencer and the automatic's own rules keep seeing the clutch they set. The engine's idle
//    governor holds the idle speed against the slipping clutch, so there is creep torque without any throttle.
//  - The floor comes from a slow speed controller (creep_control): too slow = close the clutch a bit,
//    too fast or braking = open it. Engine rpm under idle = open (never drag the engine down).
//  - The automatic selects neutral at a standstill when no pedal is down, and in neutral waits for
//    throttle. A tail hook (chained behind TrueShift's) holds gear 1 (or R) at rest while we creep, and
//    after a stop engages gear 1 by itself once both pedals have been free for a moment.
//  - v2: light loads creep in a higher gear (clutch only), heavy loads in gear 1-2 with a little throttle (a throttle floor posted
//    to the actuator, core/actuate.cpp, like the descent brake and launch control's throttle cap).
//  - Hand back: the floor ramps to 0 over release_s whenever the mode ends.
#define LOG_BOOST 1  // experimental: more detail in the log by default
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/sense.h"
#include "../core/act.h"
#include "../core/vehicle.h"
#include "../core/vclass.h"
#include "../core/wish.h"
#include <math.h>
#include <string.h>
#include <windows.h>

extern module m_creep;

static float p_pedals_free, p_creep, p_brake, p_rest, p_delay, p_rev, p_close, p_open, p_release, p_trace, p_letter, p_heavy_t, p_max_gear, p_heavy_gear, p_heavy_thr, p_arm, p_start_on, p_tog_hold, p_descent_brake, p_descent_kmh, p_tog_mode;
static float p_crawl_rpm, p_launch, p_l_light, p_l_full, p_l_heavy, p_l_bite, p_l_gain, p_l_creep, p_l_final, p_l_rate, p_l_sync, p_l_max_s, p_l_end;

// The pedals-free setting (0 hold, 1 creep). The old keys enabled / start_on / max_pedal / manoeuvre_kmh / on_kmh / off_kmh are read and
// ignored (start_on 1 maps to pedals_free = creep): the crawl is not a mode any more, it is the bottom of the pull-away.
static const float PF_HOLD = 0.0f, PF_CREEP = 1.0f;

// Eco / Power change the smooth start a little (Normal and M the middle value)
static float lmode(float eco, float normal, float power)
{
    int m = sense_mode();
    return m == MODE_ECO ? eco : m == MODE_POWER ? power : normal;
}

static param g_params[] = {
    { "pedals_free", &p_pedals_free, 0.0f, "with both pedals free at a standstill (or below the creep speed) the truck either holds still or creeps. hold = a light brake holds the truck (the owner's choice; like the light brake help in neutral). creep = the truck creeps at creep_kmh through the slipping clutch, and the brake pedal lowers it. The gearbox-mode key (toggle_mode_key) flips this for the session. The old keys enabled / start_on / max_pedal / manoeuvre_kmh / on_kmh / off_kmh are read and ignored; start_on 1 is read as creep." },
    { "creep_kmh", &p_creep, 5.0f, "creep speed with both pedals released (km/h); the brake pedal lowers it" },
    { "brake", &p_brake, 0.30f, "brake pedal (0..1) that takes the creep away completely; below it the brake lowers the creep speed" },
    { "from_rest", &p_rest, 1, "1 = after a stop, with both pedals free, engage gear 1 and creep off (the stock automatic waits in neutral)" },
    { "descent_kmh", &p_descent_kmh, 8.0f, "downhill: the creep starts braking above this speed (km/h)" },
    { "descent_brake", &p_descent_brake, 0.4f, "downhill: brake request (0..1) the creep may use to hold its speed; 0 = off" },
    { "start_on", &p_start_on, 0, "legacy (ignored except as below): the old 'crawl on at start' is pedals_free = creep now; 1 here reads as creep" },
    { "toggle_mode_key", &p_tog_mode, 1, "1 = hold the game's gearbox-mode key (Eco/Normal/Power/M) at a standstill to switch the pedals-free setting (hold <-> creep); the mode it stepped to is put back" },
    { "toggle_hold_s", &p_tog_hold, 0.8f, "how long to hold it" },
    { "arm_kmh", &p_arm, 0, "from_rest only after the truck has been driven faster than this since the engine started (km/h); 0 = always" },
    { "rest_delay_s", &p_delay, 0.3f, "from_rest: both pedals free this long (s) before the truck starts creeping" },
    { "reverse", &p_rev, 1, "1 = creep in reverse too (keeps going after you reverse, never starts it: the game's brake-at-rest makes R)" },
    { "close_rate", &p_close, 0.3f, "clutch closing speed at most (per second)" },
    { "open_rate", &p_open, 1.0f, "clutch opening speed at most (per second)" },
    { "release_s", &p_release, 0.8f, "handing the clutch back to the game takes this long (s)" },
    { "heavy_t", &p_heavy_t, 20, "loads from this mass (metric tons, truck + trailer) are heavy: creep in a low gear with a little throttle; lighter: clutch only, in a higher gear" },
    { "max_gear", &p_max_gear, 5, "light load: highest gear used for creeping (the lowest gear whose idle lock-up speed reaches creep_kmh)" },
    { "heavy_gear", &p_heavy_gear, 2, "heavy load: highest gear used for creeping" },
    { "crawl_max_rpm", &p_crawl_rpm, 950, "heavy load: creep in the lowest gear in which the engine stays at or under this speed (rpm) at the creep speed: the clutch is closed there and the engine pulls from a higher speed with a little throttle. Lighter or when no gear fits: the clutch slips at idle in a higher gear as before. 0 = old rule (heavy_gear)" },
    { "heavy_throttle", &p_heavy_thr, 0.15f, "heavy load: most throttle the plugin adds when the clutch alone is not enough (0 = none; needs the Cruise module on)" },
    { "smooth_start", &p_launch, 1, "1 = smooth start: pulling away (forward, from a standstill, automatic) the plugin closes the clutch itself: the engine rises to a speed that follows the pedal, the clutch slips to hold it until the wheels catch up, then closes gently. 0 = the game's own clutch" },
    { "smooth_start_rpm_light", &p_l_light, 200, "smooth start: engine speed above idle (rpm) the slip aims for with a light pedal" },
    { "smooth_start_rpm_full", &p_l_full, 1200, "smooth start: engine speed (rpm) the slip aims for with the pedal flat" },
    { "smooth_start_rpm_heavy", &p_l_heavy, 100, "smooth start: extra rpm for a heavy load (heavy_t), the engine needs more torque to get it going" },
    { "smooth_start_bite", &p_l_bite, 0.96f, "smooth start: clutch position (1 = open) where the clutch starts to carry, the engine rises against it first" },
    { "smooth_start_gain", &p_l_gain, 0.5f, "smooth start: how fast the clutch closes (per second) per 100 rpm the engine is above its aim; and opens when below" },
    { "smooth_start_creep", &p_l_creep, 0.10f, "smooth start: the clutch always closes at least this fast (per second) so the slip ends; grows after 1 s" },
    { "smooth_start_final", &p_l_final, 1.2f, "smooth start: closing speed (per second) of the last part, once engine and wheels are within smooth_start_sync_rpm; only used at the very end" },
    { "smooth_start_max_rate", &p_l_rate, 2.0f, "smooth start: the fastest the clutch ever closes while it holds the engine at its aim (per second)" },
    { "smooth_start_sync_rpm", &p_l_sync, 80, "smooth start: engine and wheel speed this close (rpm) = the clutch closes the rest" },
    { "smooth_start_max_s", &p_l_max_s, 2.5f, "smooth start: longest slip time (s), then it closes at smooth_start_final whatever the engine does" },
    { "smooth_start_end_kmh", &p_l_end, 14, "smooth start: at this speed (km/h) it is over, the clutch is closed and handed back to the game" },
    { "letter", &p_letter, 1, "1 = show C (creep) on the dashboard while the mode is on" },
    { "trace", &p_trace, 1, "1 = write creep.csv (10 per second while the mode is on)" },
};

static int g_hooked, g_tail_ok, g_engage_ok;   // g_hooked: the actuator's clutch hook is in
static void *g_self;                // the player's gearbox (set every frame)
static FILE *g_csv;

// ---------------------------------------------------------------- the mode (no game memory below this line until ck_frame)

struct creep_in {
    float dt, speed, thr, brake, rpm, idle;  // s, m/s signed (forward +), 0..1, 0..1, engine rpm, idle rpm
    int gear, engine_on, blocked, can_engage;
    int crawl_gear, set_max, heavy;  // gear to creep in, highest gear that counts as creeping, heavy load
};

static int g_user_on = -1;  // the pedals-free setting of the session (0 hold, 1 creep), -1 = not read from the ini yet
static int g_on;          // the idle creep is running (pedals free, the creep setting)
static int g_hold;        // gear the tail hook must hold / engage (0 = none)
static int g_armed;       // the truck has been driven since the engine started (no creep-off at game start)
static int g_last_dir = 1;
static float g_cbrake;     // brake request for a descent (0..descent_brake)
static float g_hbrake;     // brake request of the hold setting (0..1)
static float g_cthr;
static float g_heavy_integ;  // review 30-10: heavy creep PI, file-static so ck_reset / ck_stop can drop it (was a function-static that leaked across episodes)
static double g_on_at;
static float g_brake_off_s;  // how long the brake pedal has been released     // when the mode came on (from rest: a gear must be in within 1.5 s)
static int g_rest_fail;    // engaging from rest did not hold: wait until the pedals are touched      // throttle we add (heavy loads), 0..heavy_throttle
static float g_f;         // clutch floor 0 (closed) .. 1 (open)
static float g_vt;        // idle creep target speed m/s
static float g_free_s;    // both pedals free for this long
static const char *g_why = "";

static float clampv(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// The pedals-free setting of the session (0 hold, 1 creep), the driver's switch (g_lv.crawl_toggle).
static inline int session_creep() { return g_user_on > 0; }

// The idle creep target: creep_kmh, lowered by the brake pedal (a light brake holds it at 0). The pedal itself never sets this speed
// any more: a pressed pedal is the launch (a force, the torque pedal's demand), not a creep speed.
static float idle_target(float brake)
{
    float b = 1 - clampv(brake / fmaxf(p_brake, 0.05f), 0, 1);  // the brake pedal lowers the creep speed
    return p_creep * b / 3.6f;
}

// d(floor)/dt, integrated: below the target close a little (at most close_rate); above it open, unless the
// wheels already drive the engine above idle (then closed is better: engine drag); brake: open fast.
static void creep_control(const creep_in &in, float vd, float thr, float brake)
{
    g_vt = idle_target(brake);
    float err = g_vt - vd, d;
    if (brake >= p_brake) d = 4.0f;
    else if (err > 0) d = -fminf(p_close, err);
    else if (in.idle > 200 && in.rpm > 1.04f * in.idle) d = -p_close;  // the wheels drive the engine (downhill, momentum): closed = engine drag
    else d = fminf(p_open, -err * 3.0f);                                // the slip itself is too strong: open
    // never drag the engine down. Between the idle and the game's own stall line (0.8 idle, the stock stop rule of core/shiftcommon.cpp sc_stop_threshold): the clutch stops closing a quarter of the way down
    // (0.95 idle: the governor holds idle against a slipping clutch, a dip of 5 % is the bite) and opens fast at 0.88 idle (40 % of the way, with the engine falling). No rpm number: both are shares of the idle.
    if (in.idle > 200 && in.rpm > 100) {
        if (in.rpm < 0.95f * in.idle && d < 0) d = 0;
        if (in.rpm < 0.88f * in.idle) d = fmaxf(d, 2.0f);
    }
    g_f = clampv(g_f + d * in.dt, 0, 1);
    // Heavy load: the clutch is closed and still too slow -> a little throttle (raised idle), gone when fast enough.
    // A PI on the speed (ts4c: the up/down ramp pumped 4-7.5 km/h and the game shifted 2-1-2), rate limited.
    if (in.heavy && p_heavy_thr > 0 && thr < PEDAL_NOISE && brake < p_brake && in.gear > 0) {
        float e = err * 3.6f;                                           // km/h
        if (g_f > 0.3f) g_heavy_integ -= 0.05f * in.dt;                 // the clutch is slipping: no more throttle
        else if (!(g_cthr >= p_heavy_thr && e > 0) && !(g_cthr <= 0 && e < 0)) g_heavy_integ += 0.012f * e * in.dt;
        g_heavy_integ = clampv(g_heavy_integ, 0, p_heavy_thr);
        float want = clampv(0.02f * e + g_heavy_integ, 0, p_heavy_thr);
        float step = 0.08f * in.dt;                                     // at most 0.08 per second either way
        g_cthr = clampv(want, g_cthr - step, g_cthr + step);
    } else g_cthr = 0;
    // Descent (Damien: like hill descent control): faster than the creep speed with the pedals free ->
    // a gentle brake request (the actuator applies it, hazards stay off), gone when you brake or give gas.
    float over = vd * 3.6f - fmaxf(p_descent_kmh, g_vt * 3.6f + 1);  // km/h above where braking starts (Damien: from 8)
    if (p_descent_brake > 0 && thr < PEDAL_NOISE && brake < 0.01f && (over > 0 || (g_cbrake > 0 && over > -1)))
        g_cbrake = fminf(p_descent_brake, g_cbrake + 0.15f * in.dt);
    else g_cbrake = fmaxf(0, g_cbrake - 0.5f * in.dt);
}

static void creep_tick(const creep_in &in)
{
    static int heavy_prev;
    if (in.heavy != heavy_prev) { if (!in.heavy) g_heavy_integ = 0; heavy_prev = in.heavy; }   // review 30-10: the heavy creep integral must not survive a heavy -> light switch
    float v = in.speed, vk = fabsf(v) * 3.6f, vd = in.gear < 0 ? -v : v;
    float thr = clampv(in.thr, 0, 1), brake = in.brake;
    if (!in.engine_on) g_armed = 0;
    else if (p_arm <= 0 || (in.gear != 0 && vk > p_arm)) g_armed = 1;
    if (vk > 1.5f) g_last_dir = v > 0 ? 1 : -1;
    int free = thr < PEDAL_NOISE && brake < p_brake, rest = fabsf(v) < 0.15f;
    g_brake_off_s = brake < 0.01f ? g_brake_off_s + in.dt : 0;
    g_free_s = free ? g_free_s + in.dt : 0;
    int in_set = (in.gear >= 1 && in.gear <= in.set_max) || (in.gear == -1 && p_rev > 0);

    // the idle creep (pedals free, the creep setting). The pedal ends it: a pressed pedal is the launch (a force), below.
    if (g_on) {
        const char *why = NULL;
        if (!session_creep()) why = "the setting flipped to hold";
        else if (in.blocked) why = "blocked (parking brake, cruise, engine, hill hold, eco-roll)";
        else if (!in_set && !(in.gear == 0 && rest && wall_s() - g_on_at < 1.5)) {
            why = "gear";
            if (in.gear == 0 && rest) { why = "no gear engaged from rest"; g_rest_fail = 1; }
        }
        else if (thr >= PEDAL_NOISE) why = "throttle";  // the pedal: the launch takes over (a force, not a creep speed)
        if (why) {
            g_on = 0;
            g_why = why;
            logline("Creep: off (%s) gear %d, %.1f km/h, throttle %.2f, floor %.2f", why, in.gear, vk, thr, g_f);
        }
    // not while braking to a stop (ts4o 654.9 s: it took over during a stop; Damien: "braking = brake, not crawl"):
    // the brake must have been off for 0.5 s before the creep starts while rolling
    } else if (session_creep() && !in.blocked && in_set && thr < PEDAL_NOISE && brake < 0.01f && g_brake_off_s > 0.5f &&
               (in.gear > 0 || vd > 0.1f)) {
        g_on = 1;
        if (g_f < 0.01f && vd < 0.25f * idle_target(brake)) g_f = 0.9f;  // from rest: start nearly open, close gently
        logline("Creep: on (gear %d, %.1f km/h, brake %.2f, target %.1f km/h)", in.gear, vk, brake, idle_target(brake) * 3.6f);
    }
    if (g_on) creep_control(in, vd, thr, brake);
    else { g_f = fmaxf(0, g_f - in.dt / fmaxf(p_release, 0.1f)); g_cthr = 0; }

    // Gear to hold at rest (the automatic would pick neutral) or to engage from neutral.
    int hold = 0;
    // from rest a low gear (ts4h: a light truck in its crawl gear 4 at idle never moved), up to the crawl gear
    // once rolling
    int start = in.crawl_gear < (in.heavy ? 1 : 2) ? in.crawl_gear : (in.heavy ? 1 : 2);
    if (g_on && free && in.gear < 0 && rest) hold = in.gear;
    else if (g_on && free && in.gear > 0 && rest) hold = in.gear > start ? start : in.gear;
    // up one gear at a time, only when this gear is at its idle lock-up (clutch nearly closed, rpm at idle) and
    // still too slow (ts4j: 1 -> 4 in one go at 3.7 km/h was a jolt)
    else if (g_on && free && in.gear > 0 && in.gear < in.crawl_gear) {
        static double up_at;
        int at_lockup = g_f < 0.15f && in.idle > 200 && in.rpm < 1.06f * in.idle && vd < g_vt;
        hold = in.gear;
        if (at_lockup && wall_s() - up_at > 1.0) {
            up_at = wall_s();
            hold = in.gear + 1;
        }
    }  // up to the crawl gear, never down while rolling (the game's stall rules own that)
    // from rest only with the brake fully released (ts4c: at rest with the brake at 0.2 the game put it back in
    // neutral every tick and we engaged again: a flicker); once moving the brake modulates the creep
    else if (session_creep() && p_rest > 0 && in.can_engage && g_armed && !in.blocked && vk < p_creep && free && brake < 0.05f &&
             g_free_s >= p_delay && g_last_dir > 0 && !g_rest_fail && (in.gear == 0 || (in.gear == -1 && rest))) {
        hold = start;
        // On at once (ts4e: the game's at-rest neutral rule won every tick while we only engaged: the dash
        // flickered 1/2 in N); the mode then holds the gear. If it still has no gear after 1.5 s: give up
        // until the pedals are touched.
        if (!g_on) {
            g_on = 1;
            g_on_at = wall_s();
            if (g_f < 0.01f) g_f = 0.9f;
            logline("Creep: on from rest (crawl gear %d)", hold);
        }
    }
    if (!free) g_rest_fail = 0;
    if (hold != g_hold) {
        static double said;
        if (hold && wall_s() - said > 2) {  // at most every 2 s (ts4c: 13 lines in 0.3 s)
            said = wall_s();
            logv(1, "Creep: %s gear %d (%s, pedals free %.1f s)", in.gear == hold ? "holding" : "engaging", hold,
                          rest ? "at rest" : "rolling", g_free_s);
        }
        g_hold = hold;
    }
}

// ---------------------------------------------------------------- launch (no game memory in here either)
//
// Pulling away without crawl (Damien 12-10: "the box hangs on a fixed rpm and then bangs the clutch shut"). A heavy-truck AMT
// does it the other way round: the engine speed aimed for follows the pedal (idle + ~200 light, ~1200 flat), the clutch slips to
// hold the engine there while the wheels speed up, and when engine and wheels are close it closes with a limited rate (jerk).
struct launch_in {
    float dt, speed, thr, brake, rpm, idle, rpm_per_ms;  // s, m/s forward, 0..1, 0..1, engine rpm, idle rpm, rpm of a locked clutch per m/s
    int gear, heavy, blocked, shifting;
    float handover;  // >= 0: the idle creep just ended with this clutch value: carry on from it (rolling start), else only from a standstill
    // [lowspeed] owner=1: this is the owner's launch (every pull-away, also Stop&Go's drive-off on the cruise throttle). thr is then the pedal or the
    // fed cruise throttle; eff_thr the engine's own throttle (the game cuts it at a full pedal, 25-10 16:50); end_kmh is where the owner's world ends.
    int owner;
    float eff_thr, end_kmh;
    // The shaped share of the pedal (wish_launch_share): what the rpm aim follows, so the wish pedal's curve and rise limit hold at a
    // pull-away too. thr stays the raw pedal / fed cruise throttle for the gates (launch control's flat pedal, the release, the start threshold).
    float wish;
};

static int g_ln;            // the launch owns the clutch
static float g_lc;          // clutch 0 (closed) .. 1 (open), what the drivetrain sees while g_ln
static float g_ln_t, g_ln_rel, g_ln_shut, g_ln_ra, g_ln_rpm0, g_ln_rate;
static int g_ln_up, g_ln_done;  // engine has risen to its aim; ended normally (clutch closed): the game's value is set to ours once
static const char *g_ln_why = "";

// The light-pedal aim's margin over idle (the launch_aim low end minus idle).
static float launch_light_margin(int heavy)
{
    return p_l_light + lmode(-50, 0, 150) + (heavy ? p_l_heavy : 0);
}

// The engine speed the slip aims for, by the shaped share of the pedal (core/wish.h). Smoothstep over (share - 0.05) / 0.85: flat under a share of 0.05 and from 0.90 up (a pedal of 0.72 in Power,
// 0.77 in Normal, 0.80 in Eco). KEPT as it was (WP-LS, reported with the numbers): a linear aim lowered the mid-pedal aim by 8 % and moved the light truck's launch jerk (sim_lowspeed L1b, 5 t: 2270 ->
// 2757 m/s3). The engine's throttle follows the pedal over the whole travel since WP-LS (modules/pedal.cpp: throttle = the shaped share while the law holds the clutch), so a harder pedal still pulls
// harder where the aim is at its top; the aim's top is the engine speed a launch never exceeds.
static float launch_aim(float share, float idle, int heavy)
{
    float s = clampv((share - 0.05f) / 0.85f, 0, 1);
    s = s * s * (3 - 2 * s);
    float lo = idle + launch_light_margin(heavy), hi = fmaxf(p_l_full + lmode(-150, 0, 150) + (heavy ? p_l_heavy : 0), lo + 100);
    return lo + (hi - lo) * s;
}

static float lcx_aim() { return fminf(fmaxf(T.rpm_limit, 1900.0f) * 0.82f, 1700.0f); }

// The engine speed under which the smooth start opens the clutch and never closes it: a sensible margin above idle, a share of the
// engine's own span (idle -> torque plateau), so a tall gear cannot lock the engine just above idle and drag it under. The margin is
// capped at half of the light-pedal aim's margin over idle, so the threshold can never sit above the aim and hold the clutch open
// through a light-pedal launch. The fallback is idle itself where the engine is unknown (the simple sim plants). No fixed rpm on a known engine.
static float launch_drag_rpm(float idle, int heavy)
{
    float span = V.eng_ok && V.plateau > idle + 60 ? V.plateau - idle : 0;
    float margin = span > 60 ? 0.10f * span : 0;
    return idle + fminf(margin, 0.5f * launch_light_margin(heavy));
}

static float g_lcx_cap = 1;   // launch control's throttle cap (1 = none), posted to the actuator as ACT_LCX
static void lcx_cap(float v) { g_lcx_cap = v; act_thr_cap(ACT_LCX, v); }
static void lcx_off() { g_lv.lcx = 0; lcx_cap(1); }   // launch control off and its throttle cap released (every path that drops g_ln)

static void launch_end(const char *why, int normal)
{
    lcx_off();
    if (!g_ln) return;
    logv(1, "Smooth start: over (%s) after %.1f s, clutch %.2f", why, g_ln_t, g_lc);
    g_ln = 0; g_ln_done = normal; g_ln_why = why;
}

static void launch_tick(const launch_in &in)
{
    float vk = fabsf(in.speed) * 3.6f;
    if (!g_ln) {
        g_ln_done = 0;
        // Launch control (not documented): standing still, the brake held and the pedal flat for 0.7 s: the clutch opens, the engine is held
        // at lcx_aim by a throttle cap and TrueShift picks a higher start gear; letting go of the brake launches with that aim.
        static float lcx_t;
        if (p_launch > 0 && !in.blocked && !in.shifting && in.gear > 0 && vk < 1.2f && in.speed > -0.1f && in.brake > 0.5f && in.thr > 0.9f && in.idle > 200) {
            lcx_t += in.dt;
            if (lcx_t > 0.7f) {
                lcx_t = 0; g_ln = 1; g_lv.lcx = 1; g_ln_t = g_ln_rel = g_ln_shut = g_ln_rate = 0; g_ln_rpm0 = in.rpm; g_ln_up = 1; g_lc = 1.0f;
                g_ln_ra = lcx_aim(); lcx_cap(0.5f);
                logline("Launch control: armed, holding %.0f rpm (gear %d)", g_ln_ra, in.gear);
                return;
            }
        } else lcx_t = 0;
        int rest = vk < 1.2f && in.speed > -0.1f, roll = in.handover >= 0 && vk < p_l_end;
        if (p_launch > 0 && !in.blocked && !in.shifting && in.gear > 0 && in.rpm_per_ms > 0 && in.thr > PEDAL_NOISE && in.brake < 0.05f &&
            (rest || roll) && in.idle > 200) {
            g_ln = 1; g_ln_t = g_ln_rel = g_ln_shut = g_ln_rate = 0; g_ln_rpm0 = in.rpm;
            g_ln_up = roll;
            g_lc = roll ? fmaxf(in.handover, 0.02f) : p_l_bite;
            g_ln_ra = launch_aim(in.wish, in.idle, in.heavy);
            logv(1, "Smooth start: on (gear %d, %.1f km/h, pedal %.2f, aim %.0f rpm%s)", in.gear, vk, in.thr, g_ln_ra, roll ? ", from the creep" : "");
        }
        return;
    }
    g_ln_t += in.dt;
    if (g_lv.lcx == 1) {  // armed: the clutch stays open, the throttle cap holds the revs, the brake holds the truck
        g_lc = 1.0f;
        float aim = lcx_aim();
        lcx_cap(clampv(g_lcx_cap + (aim - in.rpm) / 100 * 0.4f * in.dt, 0.08f, 1.0f));
        g_ln_rpm0 = in.rpm;   // keep the rate estimate's reference fresh: a stale one made the first "go" frame see thousands of rpm/s and slam the clutch
        if (in.blocked || in.gear <= 0 || in.thr < 0.5f || g_ln_t > 25 || vk > 2.0f) { launch_end("launch control cancelled", 0); return; }
        if (in.brake < 0.15f) {  // brake released: launch with the held revs as the aim
            g_lv.lcx = 2; lcx_cap(1); g_ln_t = 0; g_ln_up = 0; g_lc = p_l_bite; g_ln_ra = aim; g_ln_rate = 0;
            logline("Launch control: go, %.0f rpm", in.rpm);
        }
        return;
    }
    g_ln_rel = in.thr < 0.03f ? g_ln_rel + in.dt : 0;
    if (in.blocked) { launch_end("blocked", 0); return; }
    if (in.shifting || in.gear <= 0) { launch_end("shift", 0); return; }
    if (g_ln_rel > 0.4f) { launch_end("pedal released", 0); return; }
    if (in.brake > 0.2f) { launch_end("brake", 0); return; }
    float end_kmh = in.owner && in.end_kmh > 0 ? in.end_kmh : p_l_end;   // (the owner has no fixed speed edge: where the start gear's low-gear window ends)
    if (vk > end_kmh + 4) { launch_end("speed", 0); return; }
    // The game cuts the engine's throttle at a full pedal for 0.6-1.1 s at walking pace (25-10 16:50, 35.8 t: thr_eff 0 at pedal 1.0; why is not
    // known). The law would read the sagging engine as dragged and open the clutch, and close it again at sync: the jerk. With the owner, while
    // the pedal is down and the engine's throttle is not, the clutch is frozen, the aim is the engine speed of now and the slip time does not run;
    // the engine under the drag margin still opens it (no stall).
    if (in.owner && in.thr > 0.8f && in.eff_thr < 0.2f && g_lc >= 0.02f) {   // (a closed clutch has nothing to freeze: the launch ends as it always does)
        g_ln_t -= in.dt;
        g_ln_ra = in.rpm;
        g_ln_rpm0 = in.rpm; g_ln_rate = 0;
        if (in.rpm > 100 && in.rpm < launch_drag_rpm(in.idle, in.heavy)) g_lc = clampv(g_lc + 2.0f * in.dt, 0, 1);
        return;
    }
    float aim = g_lv.lcx == 2 ? lcx_aim() : launch_aim(in.wish, in.idle, in.heavy);
    g_ln_ra += (aim - g_ln_ra) * fminf(1, in.dt / 0.25f);  // the aim follows the pedal, not jumps with it
    // the engine speed 0.2 s from now (a flat pedal lifts it 1500+ rpm/s against a clutch that carries almost nothing at first):
    // control that, not the speed of this moment, or the engine flares past its aim before the clutch has started to bite
    if (in.dt > 0) g_ln_rate += ((in.rpm - g_ln_rpm0) / in.dt - g_ln_rate) * fminf(1, in.dt / 0.1f);
    g_ln_rpm0 = in.rpm;
    float rpm_p = in.rpm + clampv(g_ln_rate, 0, 3000) * 0.2f;
    float lock = in.speed * in.rpm_per_ms, slip = in.rpm - lock, d;
    if (!g_ln_up) {  // the engine rises against the bite point first (a real clutch pedal's first part)
        g_lc = p_l_bite;
        if (rpm_p >= g_ln_ra || g_ln_t > 1.2f) g_ln_up = 1;  // a little early: the engine is still rising when the clutch starts to carry
        return;
    }
    // under its aim the engine is being dragged (24.9 t ride 12-10: 637 rpm against an aim of 932, 803 against 1299): open twice as hard
    // as it closes, and never keep closing while it sags
    float err = rpm_p - g_ln_ra;
    d = -(p_l_creep * (1 + fmaxf(0, g_ln_t - 1.0f) * 3) + p_l_gain * lmode(0.8f, 1.0f, 1.3f) * (err < 0 ? 2.0f : 1.0f) * err / 100);
    if (err < -150 && d < 0.3f) d = 0.3f;
    d = fmaxf(d, -p_l_rate);
    if (slip < p_l_sync || vk > end_kmh || g_ln_t > p_l_max_s) d = -p_l_final;
    if (in.rpm > 100 && in.rpm < launch_drag_rpm(in.idle, in.heavy)) d = 2.0f;  // never drag the engine under the margin
    d = clampv(d, -fmaxf(p_l_rate, p_l_final), 2.0f);
    g_lc = clampv(g_lc + d * in.dt, 0, 1);
    if (g_lc < 0.02f) { g_ln_shut += in.dt; if (g_ln_shut > 0.15f) launch_end("closed", 1); } else g_ln_shut = 0;
}

// ---------------------------------------------------------------- game side (1.60 layout only)

// Crawl gear: the lowest gear whose lock-up speed at idle (closed clutch, engine at idle) reaches creep_kmh, so the
// clutch slips a little and the engine stays at idle. Heavy loads are capped at heavy_gear (then the plugin adds throttle).
static int pick_gear(int heavy, float idle)
{
    int cap = (int)(heavy ? p_heavy_gear : p_max_gear), n = T.fwd_gears;
    if (cap > n) cap = n;
    if (cap < 1) cap = 1;
    float r = wheel_radius();
    if (!(r > 0.1f && idle > 200 && T.diff > 0)) return 1;
    // Heavy: the lowest gear that keeps the engine under crawl_max_rpm at the creep speed (higher rpm = more torque); the clutch
    // then sits closed and the speed controller adds a little throttle (creep_control). Below idle it is the slipping clutch again.
    if (heavy && p_crawl_rpm > idle * 1.05f)
        for (int g = 1; g <= n && g <= (int)p_max_gear; g++) {
            float rpm_at = p_creep / 3.6f / r * T.ratio[g - 1] * T.diff * 9.5493f;
            if (T.ratio[g - 1] > 0 && rpm_at <= p_crawl_rpm) return g;
        }
    for (int g = 1; g <= cap; g++) {
        float ratio = T.ratio[g - 1] * T.diff;
        if (ratio > 0 && idle * 0.10472f * r / ratio * 3.6f >= p_creep * 1.05f) return g;
    }
    return cap;
}

// Runs at the end of every automatic update, right before the generic gearbox reads its target gear.
void creep_tail(void *self)
{
    static int held;
    if (!module_active(&m_creep) || !self || self != g_self) return;
    int gear = sense_gb_gear(self);
    if (g_hold) {
        int want = g_hold;
        if (sense_gb_target(self) != want) act_gb_set_target(self, want);
        // In neutral the generic gearbox waits for throttle in its gap state: do its engage step ourselves.
        if (gear == 0 && g_engage_ok && sense_gb_state(self) == 1 && act_gb_engage(self, want))
            logline("Creep: engaged gear %d without throttle (%.1f km/h)", want, fabsf(T.speed) * 3.6f);
        act_gb_drive_state(self, want > 0 ? 4 : 0);  // drive state: forward / reverse (the game's own values)
        act_gb_latch(self, 0);      // not "at rest, neutral"
        held = 1;
    } else if (held) {
        held = 0;
        // Brake at a standstill in our held gear: back to what the game does at a stop (neutral, latch;
        // its next update then picks R on the brake, as it always has).
        // Also when the creep just went off (switched off, blocked) at walking pace: neutral, so the engine
        // cannot push the truck on (Damien: it rolled with creep off).
        if (gear > 0 && fabsf(T.speed) < 0.5f && (fmaxf(T.in_brake, T.eff_brake) >= 0.01f || !g_on)) {
            act_gb_set_target(self, 0);
            act_gb_drive_state(self, 3);
            act_gb_latch(self, 1);
            logline("Creep: brake at rest, back to the game's stop (neutral)");
        }
    }
}

// The pedals-free setting (hold <-> creep): hold the game's gearbox-mode key at a standstill (no wheel button to set up).
static void crawl_toggle()
{
    if (g_user_on < 0) g_user_on = (p_pedals_free >= PF_CREEP || p_start_on > 0) ? 1 : 0;
    if (p_tog_mode <= 0) return;
    static double down_at, up_at;
    static int fired;
    static float mode_before = NAN;
    double now = wall_s();
    // The game's gearbox-mode key: its action object game_actor+0x1ea40 on 1.60, +0x1eb90 on 1.61 (ETS2 and ATS, the
    // game_is_161() shift); key handler: +0xf8 == 2 on a press, then g_adaptive_shift steps. ts4e: right after a press
    // the state is 1 and +0x178 stays 0, so the state is the signal: 2 = press event, 1 = held after the game took the
    // event (GUESS: 0 when released).
    int mode_held = 0;
    if (p_tog_mode > 0) {
        char *ga = sense_actor(), *act = sense_mode_key_action(ga);
        if (act && sense_action_readable(act)) {
            int st = sense_action_state(act);
            mode_held = st != 0;
            static int last_st = -1, n_st;
            if (st != last_st && n_st < 12) { n_st++; logv(2, "Creep: mode key state %d", st); }  // check the guess
            last_st = st;
        }
        static int said;
        float a = sense_adaptive();
        static float last_a = NAN;
        if (act && a == a && last_a == last_a && a != last_a && said < 3) {  // check the guess on the first presses
            said++;
            logline("Creep: mode key pressed: action state %d, value %.2f", sense_action_state(act), sense_action_value(act));
        }
        last_a = a;
    }
    int held = mode_held;
    static double keep_until;  // after a toggle the mode is kept a moment past the release too (ts4n: N -> E)
    if (!held) {  // a held wheel button flickers under Proton: 0.3 s up ends a hold
        if (!up_at) up_at = now;
        if (fired) keep_until = now + 0.6;
        if (now - up_at > 0.3) { down_at = 0; fired = 0; }
        if (now < keep_until && mode_before == mode_before) {
            static void *cv2;
            if (!cv2) cv2 = sense_cvar("g_adaptive_shift");
            float a = sense_adaptive();
            if (cv2 && a == a && fabsf(a - mode_before) > 0.01f) act_cvar_set("creep", cv2, mode_before);
        } else if (!down_at) mode_before = sense_adaptive();
        return;
    }
    up_at = 0;
    if (!down_at) down_at = now;
    // after the toggle the held key keeps stepping the mode (the game repeats it, ts4g: E-P-M-E every 0.26 s):
    // keep putting the mode back until the key is released
    static void *cv;
    if (fired && mode_held && mode_before == mode_before) {
        if (!cv) cv = sense_cvar("g_adaptive_shift");
        float a = sense_adaptive();
        if (cv && a == a && fabsf(a - mode_before) > 0.01f) act_cvar_set("creep", cv, mode_before);
    }
    // at a standstill or while creeping (ts4g: creep was on and moving, so the toggle never fired)
    if (fired || now - down_at < p_tog_hold || fabsf(T.speed) * 3.6f > p_creep) return;
    fired = 1;
    g_user_on = !g_user_on;
    if (mode_held && mode_before == mode_before) {  // the press stepped the gearbox mode: put it back
        if (!cv) cv = sense_cvar("g_adaptive_shift");
        if (cv) act_cvar_set("creep", cv, mode_before);
    }
    play_cue(g_user_on ? CUE_ON : CUE_OFF);
    disp_post(DISP_CREEP, DISP_FLASH, g_user_on ? 'C' : 'X', 1.5);
    logline("Creep: pedals free %s", g_user_on ? "creep (drive ~%.0f km/h)" : "hold (light brake)");
}

// Continuity broke (pause menu, hitch, other truck, teleport, ...): no pedal timers or one-off latches may carry over.
// SOFT keeps a running creep (the controller works from fresh speed next frame); HARD lets everything go.
static void ck_reset(int level, const char *why)
{
    g_free_s = 0;
    g_rest_fail = 0;
    g_on_at = wall_s();
    g_heavy_integ = 0;   // review 30-10: a pause / hitch / teleport / trailer change must not keep the heavy creep PI
    if (level >= RESET_HARD) {
        g_on = 0; g_lv.crawl = 0;
        g_hold = 0;
        g_ln = 0; g_ln_done = 0; lcx_off();
        g_f = g_cthr = g_cbrake = g_hbrake = g_vt = 0;
        g_armed = 0;
        g_last_dir = 1;
        act_clutch(NULL, 0, 0);
        act_thr_floor(ACT_CREEP, 0);
        act_brake(ACT_CREEP, 0);
        disp_post(DISP_CREEP, DISP_STATUS, 0, 0);
    }
}

// The hold setting's brake request (pedals_free = hold), both pedals free below the creep speed (WP-FX1, ride 04-11). The owner: "when I release the gas below ~5 km/h it stops far too hard, I can't
// roll out; just roll out, and only at a standstill (really 0 km/h) the hold". The old request was the one that holds a truck at REST on this grade (twice the slope's pull plus 0.8 m/s2,
// at least 0.2, vehicle_brake_hold_req) and was asked from the creep speed down, rolling: 1.2 - 1.8 m/s2 on the flat (16.5 t reversing 589.6, 616.0, 622.4 s; 41.9 t forward 987.0 s).
// Now it is one continuous function of the speed, two parts, the larger of them:
//  - ROLLING: only what the grade needs so that the truck does not run away. The grade's acceleration along the way the truck moves (+ = it speeds up: downhill ahead, or backwards uphill) less
//    what rolling and the air take by themselves (the level road's resistance of the vehicle model) is the deceleration to ask; 0 on the flat and uphill, where the truck rolls out on its own drag.
//    Where something has to brake the stop is finished gently, the way the stop assist did until decision 65 (min(0.5 m/s2, sqrt(2 j v)), j = 0.5 of the jerk limit's default 2 m/s3; the stop assist now rolls out and holds only the speed on a descent): a truck that the slope
//    would never stop is eased to a standstill, not kept at a walking pace and not grabbed. The part fades in over the last fifth of the creep speed (no step where the setting begins).
//  - AT REST: the old request, weighted by how near the truck is to rest: 0 at creep's own rest speed (0.15 m/s, `rest` in creep_tick) and above, 1 at a standstill (linear in between), so the hold
//    comes in as the truck comes to rest and is complete when it stands; on a slope that starts to roll again the rolling part takes it back to a stop.
// speed is signed (m/s, - = backwards), pitch_pct the grade of the nose (+ = up); fallback is vehicle_brake_hold_req's while the brake gain is not confident.
#define CREEP_HOLD_REQ 1
static const float CR_REST_V = 0.15f;      // m/s, creep_tick's `rest`
static const float CR_TAPER_LEVEL = 0.5f, CR_TAPER_J = 1.0f;   // the level of the stop assist's old taper (stop_assist_decel default, under the 0.7 where a car's brake lights stay off) and its taper jerk (0.5 x 2 m/s3); the stop assist itself rolls out since decision 65 (WP-K2), these are Creep's own now
static float creep_hold_req(float speed, float pitch_pct, float fallback)
{
    const float v = fabsf(speed), dir = speed < 0 ? -1.0f : 1.0f;
    const float run = fmaxf(0.0f, -dir * 9.81f * sinf(atanf(pitch_pct / 100.0f)) - (V.mass > 0 ? vehicle_resist_grade(v, 0.0f) / V.mass : 0.0f));   // m/s2 the slope speeds the truck up beyond its own resistance
    const float kmh = v * 3.6f, fade = clampv((p_creep - kmh) / fmaxf(0.2f * p_creep, 0.1f), 0.0f, 1.0f);
    const float roll = run > 0 ? fminf((run + fminf(CR_TAPER_LEVEL, sqrtf(2.0f * CR_TAPER_J * v))) / vehicle_brake_cautious(), 1.0f) * fade : 0.0f;
    const float at_rest = clampv(1.0f - v / CR_REST_V, 0.0f, 1.0f) * vehicle_brake_hold_req(pitch_pct, fallback);
    return fmaxf(roll, at_rest);
}

static void ck_frame()
{
    static double last, csv_at, log_at;
    double now = sim_s();
    float dt = (float)(now - last);
    last = now;
    if (!g_hooked) return;
    crawl_toggle();
    g_lv.crawl_toggle = session_creep();
    g_lv.crawl = g_on;   // (the idle creep running now; every path below that drops g_on writes it again at the end of the frame)
    void *self = sense_gearbox();
    char *inner = self ? sense_gb_inner(self) : NULL;
    g_self = self;
    act_clutch_release();  // posted again below only while creeping or launching: our clutch value then replaces the game's
    if (vehicle_tc()) {   // a torque converter's creep and launch are its own: never drive the clutch on one
        if (g_on || g_ln || g_hold) logv(1, "Creep: torque converter: the creep and the launch are the converter's, the clutch is not driven");
        g_on = 0; g_hold = 0; g_ln = 0; lcx_off(); g_f = g_cthr = g_cbrake = g_hbrake = 0; g_lv.crawl = 0;
        act_thr_floor(ACT_CREEP, 0);
        act_brake(ACT_CREEP, 0);
        disp_post(DISP_CREEP, DISP_STATUS, 0, 0);
        return;
    }
    if (!inner) { act_clutch(NULL, 0, 0); g_f = 0; g_cthr = 0; act_thr_floor(ACT_CREEP, 0); act_brake(ACT_CREEP, 0); g_cbrake = g_hbrake = 0; g_on = 0; g_hold = 0; g_ln = 0; lcx_off(); g_lv.crawl = 0; return; }
    if (dt > 0.5f) ck_reset(RESET_SOFT, "long frame");
    if (dt <= 0 || dt > 0.5f) return;  // paused or a long hitch: no time step

    char *ga = sense_actor();
    float hill = sense_hill_timer(ga);
    int gear = sense_gb_gear(self), st = sense_gb_drive_state(self);
    // Automatic or arcade box (SDK shifter type; empty = not reported, the hooks only see the automatic anyway).
    // self+S_AMODE was 0 in Damien's automatic (ride 0.4.1: "not in automatic mode" all the time), so it is not used.
    int amode = !T.shifter[0] || !strcmp(T.shifter, "automatic") || !strcmp(T.shifter, "arcade");
    creep_in in;
    in.dt = dt;
    in.speed = T.speed;
    in.thr = T.in_thr;
    in.brake = T.in_brake;  // the driver's pedal (the effective brake also carries the game's own braking)
    in.rpm = T.rpm;
    in.idle = sense_game_idle_rpm(inner);
    in.gear = gear;
    in.engine_on = T.engine_on;
    in.blocked = g_truck_unsupported || T.paused || !T.engine_on || T.parking_brake || T.cruise > 0.1f || g_rolling || !amode || hill > 0 ||
                 !(in.idle > 0);
    int heavy = !V.mass_ok || V.mass >= p_heavy_t * 1000;  // the vehicle model's mass; unknown counts as heavy
    in.heavy = heavy;
    in.set_max = (int)(heavy ? p_heavy_gear : p_max_gear);
    in.crawl_gear = pick_gear(heavy, in.idle);
    if (in.crawl_gear > in.set_max) in.set_max = in.crawl_gear;  // a heavy creep gear picked by rpm may lie above heavy_gear
    in.can_engage = g_tail_ok && g_engage_ok && ((st >= 1 && st <= 3) || gear == -1);
    int was_on = g_on;
    float was_f = g_f;
    creep_tick(in);
    // the hold setting: both pedals free, below the creep speed (or at a standstill): the truck is HELD at a standstill and only kept from running away on a grade while it rolls (creep_hold_req;
    // the creep setting creeps instead, above). Only when the truck would otherwise roll or creep: not while the driver brakes, not in the launch.
    g_hbrake = 0;
    if (!session_creep() && !g_on && !g_ln && in.thr < PEDAL_NOISE && in.brake < 0.01f && fabsf(in.speed) * 3.6f < p_creep && in.engine_on && !T.parking_brake && !T.cruise)
        g_hbrake = creep_hold_req(in.speed, V.pitch_pct, 0.2f);
    {   // launch: pulling away is ours too, creep or not (the hook below carries the value; the game's own is set along)
        launch_in li = {};
        // [lowspeed] owner=1: every pull-away is the owner's launch. Stop&Go's drive-off has no pedal: the cruise throttle fed to the automatic is the
        // pedal then, and the cruise being on does not stop it. No fixed speed edge: the launch ends where the start gear's low-gear window ends.
        int own = low_owner_on();
        float feed = 0;
        if (own) { cruise_info cu; cruise_info_get(&cu); feed = cu.feed; }
        li.owner = own; li.eff_thr = T.eff_thr; li.end_kmh = own ? low_top_kmh() : 0;
        li.dt = dt; li.speed = gear < 0 ? -T.speed : T.speed; li.thr = fmaxf(T.in_thr, feed); li.brake = T.in_brake; li.rpm = T.rpm; li.idle = in.idle;
        li.wish = fmaxf(wish_launch_share(T.in_thr), feed);   // the shaped share of the driver's pedal; the fed cruise throttle (Stop&Go's drive-off) goes through as it is
        float r = wheel_radius();
        li.rpm_per_ms = gear > 0 && gear <= T.fwd_gears && r > 0.1f && T.diff > 0 && T.ratio[gear - 1] > 0 ? T.ratio[gear - 1] * T.diff / r * 9.5493f : 0;
        li.gear = gear; li.heavy = heavy;
        li.handover = was_on && !g_on && was_f > 0.05f ? was_f : -1;
        // the game's gearbox must be quiet: gear in, no shift under way (states 0 = idle, 3 = closing; 2 = opening the clutch)
        int gst = sense_gb_state(self);
        li.shifting = (gst != 0 && gst != 3) || sense_gb_target(self) != gear;
        li.blocked = p_launch <= 0 || g_truck_unsupported || T.paused || !T.engine_on || T.parking_brake || (T.cruise > 0.1f && !own) || g_rolling || !amode || hill > 0 ||
                     !(in.idle > 0) || !g_hooked;
        launch_tick(li);
        // a launch that ended normally hands the game a closed clutch, once (while it runs, act_clutch below mirrors ours into the game's)
        if (!g_ln && g_ln_done && act_clutch_vehicle(inner, 0)) g_ln_done = 0;
    }
    // Why not creeping while slow (ride 0.4: it never came on): every 5 s in the log.
    if (!g_on && session_creep() && T.engine_on && fabsf(T.speed) * 3.6f < p_creep && fabsf(T.speed) > 0.3f) {
        static double said;
        if (now - said > 5) {
            said = now;
            const char *why = T.paused ? "paused" : T.parking_brake ? "parking brake" : T.cruise > 0.1f ? "cruise on"
                            : g_truck_unsupported ? "this truck is not supported" : g_rolling ? "eco-roll" : !amode ? "gearbox not in automatic mode" : hill > 0 ? "game hill hold"
                            : !(in.idle > 0) ? "idle rpm unknown" : !(gear == 1 || gear == 2 || (gear == -1 && p_rev > 0)) ? "gear"
                            : in.thr >= PEDAL_NOISE ? "throttle" : in.brake >= p_brake ? "brake" : "?";
            logv(2, "Creep: not on at %.1f km/h: %s (gear %d, throttle %.2f, brake %.2f, hill %.2f)", fabsf(T.speed) * 3.6f, why,
                    gear, in.thr, in.brake, hill);
        }
    }
    // our clutch value on the drivetrain while creeping (the floor) or launching (the launch's clutch, mirrored into the game's own
    // value too: Auto Hold, TrueShift and the sequencer read it); otherwise the game's own
    g_lv.crawl = g_on;
    act_clutch(g_on || g_ln ? inner : NULL, g_ln ? g_lc : g_f, g_ln);
    // heavy loads: a little throttle on top of the pedal (a floor on the merged throttle, 0..heavy_throttle; anything odd above 0.3 is
    // not posted), and the descent brake / the hold brake: both through the actuator (core/actuate.cpp), also with the Cruise module off
    act_thr_floor(ACT_CREEP, g_cthr > 0 && g_cthr < 0.3f ? g_cthr : 0);
    act_brake(ACT_CREEP, g_on ? g_cbrake : g_hbrake);
    disp_post(DISP_CREEP, DISP_STATUS, p_letter > 0 && g_on ? 'C' : 0, 0);

    if (g_csv && (g_on || was_on || g_f > 0 || g_ln) && now - csv_at >= 0.1) {
        csv_at = now;
        fprintf(g_csv, "%.2f,%.2f,%d,%.3f,%.3f,%d,%.3f,%.2f,%.0f,%.0f,%.3f,%d,%d,%d,%d,%.3f,%d,%d,%d,%.3f,%.0f\n", wall_s(), T.speed * 3.6f, gear, in.thr,
                in.brake, g_on, g_f, g_vt * 3.6f, T.rpm, in.idle, T.eff_clutch, g_hold, sense_gb_state(self), st, hill > 0, g_cthr, in.crawl_gear, heavy, g_ln, g_ln ? g_lc : -1.0f,
                g_ln ? g_ln_ra : 0.0f);
    }
    if (g_on && now - log_at >= 2) {
        log_at = now;
        logv(2, "Creep: gear %d (crawl gear %d, %s) %.1f km/h (target %.1f), floor %.2f, added throttle %.2f, rpm %.0f (idle %.0f), game clutch %.2f, throttle %.2f",
                gear, in.crawl_gear, heavy ? "heavy" : "light", T.speed * 3.6f, g_vt * 3.6f, g_f, g_cthr, T.rpm, in.idle, T.eff_clutch, in.thr);
    }
    static double flushed;
    if (g_csv && wall_s() - flushed > 2) { flushed = wall_s(); fflush(g_csv); csv_cap(&g_csv, "creep.csv"); }
}

static char g_status[160] = "off";
static const char *ck_status() { return g_status; }

static void ck_start()
{
    if (p_launch <= 0 && p_pedals_free < PF_CREEP && p_start_on <= 0) { logline("Creep: launch and creep disabled in the ini"); snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    snprintf(g_status, sizeof g_status, "off (no clutch hook, see the Actuator lines)");
    if (!act_clutch_ok()) { logline("Creep: no clutch hook (see the Actuator lines), off"); return; }
    g_hooked = 1;
    g_engage_ok = act_gb_engage_init();
    if (p_trace > 0) {
        g_csv = open_out("creep.csv", "w");
        if (g_csv) fputs("t,kmh,gear,thr,brake,on,floor,target_kmh,rpm,idle,game_clutch,hold,gb_state,drive_state,hillhold,add_thr,crawl_gear,heavy,launch,launch_clutch,launch_aim\n", g_csv);
    }
    g_user_on = (p_pedals_free >= PF_CREEP || p_start_on > 0) ? 1 : 0;
    snprintf(g_status, sizeof g_status, "on, hook in place; launch %s, pedals free %s, from rest %s, reverse %s", p_launch > 0 ? "on" : "off",
             g_user_on > 0 ? "creep (drive ~%.0f km/h)" : "hold (light brake)", p_rest > 0 && g_engage_ok ? "on (needs the update's tail, installed by LowSpeed)" : "off", p_rev > 0 ? "on" : "off");
    logline("Creep: on (pedals free %s at %.1f km/h, from rest %s, reverse %s)",
            g_user_on > 0 ? "creep" : "hold", p_creep, p_rest > 0 && g_engage_ok ? "on" : "off", p_rev > 0 ? "on" : "off");
}

// The update's tail hook is LowSpeed's (one hook for the creep's hold and engage and the owner's preselect): it asks whether the creep needs it and says whether it is in.
void creep_set_switch(int on) { g_user_on = on < 0 ? (p_pedals_free >= PF_CREEP || p_start_on > 0) : on; }
int creep_wants_tail() { return g_hooked && (p_pedals_free >= PF_CREEP || p_start_on > 0 || p_tog_mode > 0); }
void creep_tail_set(int ok)
{
    g_tail_ok = ok;
    if (!g_hooked) return;
    if (!ok) logline("Creep: update tail not found, no holding at rest / no start from rest");
    else if (!g_engage_ok) logline("Creep: gearbox engage code not found, no start from rest");
}

static void ck_stop()
{
    act_clutch(NULL, 0, 0);  // give everything back
    g_ln = 0; lcx_off(); g_lv.crawl = 0;
    act_thr_floor(ACT_CREEP, 0);
    act_brake(ACT_CREEP, 0);
    g_hold = 0;
    g_heavy_integ = 0;   // review 30-10: give back the heavy creep PI too
    if (g_csv) fflush(g_csv);
}

void creep_info_get(creep_info *o)
{
    if (!module_active(&m_creep)) { memset(o, 0, sizeof *o); o->brake_off = p_brake; return; }   // absent: no creep, no launch
    o->crawl = g_on;
    o->launch = g_ln;
    o->floor = g_f;
    o->launch_clutch = g_lc;
    o->launch_aim = g_ln_ra;
    o->add_thr = g_cthr;
    o->brake = g_cbrake;
    o->brake_off = p_brake;
    o->target_kmh = g_vt * 3.6f;
}

module m_creep = { "creep", "Creep", g_params, sizeof g_params / sizeof g_params[0], ck_start, ck_frame, NULL, ck_stop, ck_reset, ck_status, NULL, VCM_TRUCK };
