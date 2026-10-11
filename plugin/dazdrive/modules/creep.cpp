// Creep: crawl and maneuver mode for the game's automatic (0.3 concept, docs/creep-re.md, docs/lowspeed.md).
//
// Like a Volvo/Eaton/ZF AMT: in gears 1, 2 and R, below ~5 km/h and with little throttle, the clutch
// slips at idle and the truck crawls along by itself; the brake takes the crawl away, the throttle asks
// for a bit more speed (maneuvering) and above ~30 % takes over completely.
//
// How (all static RE, nothing of this has run in the game yet):
//  - The drivetrain gets its clutch from one copy (inner+0x142c -> drive+0x330, 0x1408c82cf). We raise
//    that copy to a floor for the player's truck only (game_patch_clutch_floor). The gearbox sequencer
//    and the automatic's own rules keep seeing the clutch they set. The engine's idle governor holds
//    the idle speed against the slipping clutch, so there is crawl torque without any throttle.
//  - The floor comes from a slow speed controller (creep_control): too slow = close the clutch a bit,
//    too fast or braking = open it. Engine rpm under idle = open (never drag the engine down).
//  - The automatic selects neutral at a standstill when no pedal is down, and in neutral waits for
//    throttle. A tail hook (chained behind TrueShift's) holds gear 1 (or R) at rest while we crawl, and
//    after a stop engages gear 1 by itself once both pedals have been free for a moment.
//  - v2: light loads crawl in a higher gear (clutch only), heavy loads in gear 1-2 with a little throttle (g_creep_thr, via Cruise's stub).
//  - Hand back: the floor ramps to 0 over release_s whenever the mode ends.
#define LOG_BOOST 1  // experimental: more detail in the log by default
#include "../core/suite.h"
#include "../core/game.h"
#include <math.h>
#include <string.h>
#include <windows.h>

static float p_enabled, p_creep, p_man, p_on, p_off, p_maxped, p_brake, p_rest, p_delay, p_rev, p_close, p_open, p_release, p_trace, p_letter, p_heavy_t, p_max_gear, p_heavy_gear, p_heavy_thr, p_arm, p_start_on, p_tog_btn, p_tog_key, p_tog_hold, p_descent_brake, p_descent_kmh, p_tog_mode;

static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = crawl / maneuver mode on (automatic only; the clutch slips at idle in gears 1, 2 and R)" },
    { "creep_kmh", &p_creep, 5.0f, "crawl speed with both pedals released (km/h); the brake pedal lowers it" },
    { "manoeuvre_kmh", &p_man, 10.0f, "speed the throttle can ask for while maneuvering: at max_pedal (km/h), in between it follows the pedal" },
    { "on_kmh", &p_on, 8.0f, "the mode starts below this speed (km/h)" },
    { "off_kmh", &p_off, 17.0f, "and ends above this speed (km/h); with throttle it ends above manoeuvre_kmh" },
    { "max_pedal", &p_maxped, 0.30f, "throttle (0..1) above which you have the full engine and the mode ends" },
    { "brake", &p_brake, 0.30f, "brake pedal (0..1) that takes the crawl away completely; below it the brake lowers the crawl speed" },
    { "from_rest", &p_rest, 1, "1 = after a stop, with both pedals free, engage gear 1 and crawl off (the stock automatic waits in neutral)" },
    { "descent_kmh", &p_descent_kmh, 8.0f, "downhill: the crawl starts braking above this speed (km/h)" },
    { "descent_brake", &p_descent_brake, 0.4f, "downhill: brake request (0..1) the crawl may use to hold its speed; 0 = off" },
    { "start_on", &p_start_on, 0, "crawl switched on when the game starts (else hold the toggle to switch it on)" },
    { "toggle_mode_key", &p_tog_mode, 1, "1 = hold the game's gearbox-mode key (Eco/Normal/Power/M) at a standstill to switch crawl on/off; the mode it stepped to is put back" },
    { "toggle_button", &p_tog_btn, 0, "wheel button to hold at a standstill to switch crawl on/off (number from dazdrive.log); 0 = none" },
    { "toggle_key", &p_tog_key, 0, "or a keyboard key to hold (Windows virtual-key code, e.g. 67 = C); 0 = none" },
    { "toggle_hold_s", &p_tog_hold, 0.8f, "how long to hold it" },
    { "arm_kmh", &p_arm, 0, "from_rest only after the truck has been driven faster than this since the engine started (km/h); 0 = always" },
    { "rest_delay_s", &p_delay, 0.3f, "from_rest: both pedals free this long (s) before the truck starts crawling" },
    { "reverse", &p_rev, 1, "1 = crawl in reverse too (keeps going after you reverse, never starts it: the game's brake-at-rest makes R)" },
    { "close_rate", &p_close, 0.3f, "clutch closing speed at most (per second)" },
    { "open_rate", &p_open, 1.0f, "clutch opening speed at most (per second)" },
    { "release_s", &p_release, 0.8f, "handing the clutch back to the game takes this long (s)" },
    { "heavy_t", &p_heavy_t, 20, "loads from this mass (metric tons, truck + trailer) are heavy: crawl in a low gear with a little throttle; lighter: clutch only, in a higher gear" },
    { "max_gear", &p_max_gear, 5, "light load: highest gear used for crawling (the lowest gear whose idle lock-up speed reaches creep_kmh)" },
    { "heavy_gear", &p_heavy_gear, 2, "heavy load: highest gear used for crawling" },
    { "heavy_throttle", &p_heavy_thr, 0.15f, "heavy load: most throttle the plugin adds when the clutch alone is not enough (0 = none; needs the Cruise module on)" },
    { "letter", &p_letter, 1, "1 = show C (crawl) on the dashboard while the mode is on" },
    { "trace", &p_trace, 1, "1 = write creep.csv (10 per second while the mode is on)" },
};

static clutch_floor g_cf;           // read by the drivetrain hook: player vehicle + floor
static int g_hooked, g_tail_ok, g_engage_ok;
static void *g_self;                // the player's gearbox (set every frame)
static FILE *g_csv;

// ---------------------------------------------------------------- the mode (no game memory below this line until ck_frame)

struct creep_in {
    float dt, speed, thr, brake, rpm, idle;  // s, m/s signed (forward +), 0..1, 0..1, engine rpm, idle rpm
    int gear, engine_on, blocked, can_engage;
    int crawl_gear, set_max, heavy;  // gear to crawl in, highest gear that counts as crawling, heavy load
};

static int g_on;          // the mode is on
static int g_hold;        // gear the tail hook must hold / engage (0 = none)
static int g_armed;       // the truck has been driven since the engine started (no crawl-off at game start)
static int g_last_dir = 1;
static float g_cbrake;     // brake request for a descent (0..descent_brake)
static float g_cthr;
static double g_on_at;
static float g_brake_off_s;  // how long the brake pedal has been released     // when the mode came on (from rest: a gear must be in within 1.5 s)
static int g_rest_fail;    // engaging from rest did not hold: wait until the pedals are touched      // throttle we add (heavy loads), 0..heavy_throttle
static float g_f;         // clutch floor 0 (closed) .. 1 (open)
static float g_vt;        // target speed m/s
static float g_free_s;    // both pedals free for this long
static const char *g_why = "";

static float clampv(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// Speed the controller aims for: crawl speed with free pedals, rising to manoeuvre_kmh at max_pedal.
static float target_speed(float thr, float brake)
{
    float s = clampv((thr - 0.02f) / fmaxf(p_maxped - 0.02f, 0.05f), 0, 1);
    s = s * s * (3 - 2 * s);
    float b = 1 - clampv(brake / fmaxf(p_brake, 0.05f), 0, 1);  // the brake pedal lowers the crawl speed
    return (p_creep * b + (p_man - p_creep * b) * s) / 3.6f;
}

// d(floor)/dt, integrated: below the target close a little (at most close_rate); above it open, unless the
// wheels already drive the engine above idle (then closed is better: engine drag); brake: open fast.
static void creep_control(const creep_in &in, float vd, float thr, float brake)
{
    g_vt = target_speed(thr, brake);
    float err = g_vt - vd, d;
    if (brake >= p_brake) d = 4.0f;
    else if (err > 0) d = -fminf(p_close, err);
    else if (in.idle > 200 && in.rpm > 1.04f * in.idle) d = -p_close;  // the wheels drive the engine (downhill, momentum): closed = engine drag
    else d = fminf(p_open, -err * 3.0f);                                // the slip itself is too strong: open
    if (in.idle > 200 && in.rpm > 100) {  // never drag the engine down: stop closing, then open
        if (in.rpm < 0.95f * in.idle && d < 0) d = 0;
        if (in.rpm < 0.88f * in.idle) d = fmaxf(d, 2.0f);
    }
    g_f = clampv(g_f + d * in.dt, 0, 1);
    // Heavy load: the clutch is closed and still too slow -> a little throttle (raised idle), gone when fast enough.
    // A PI on the speed (ts4c: the up/down ramp pumped 4-7.5 km/h and the game shifted 2-1-2), rate limited.
    if (in.heavy && p_heavy_thr > 0 && thr < 0.02f && brake < p_brake && in.gear > 0) {
        static float integ;
        float e = err * 3.6f;                                           // km/h
        if (g_f > 0.3f) integ -= 0.05f * in.dt;                         // the clutch is slipping: no more throttle
        else if (!(g_cthr >= p_heavy_thr && e > 0) && !(g_cthr <= 0 && e < 0)) integ += 0.012f * e * in.dt;
        integ = clampv(integ, 0, p_heavy_thr);
        float want = clampv(0.02f * e + integ, 0, p_heavy_thr);
        float step = 0.08f * in.dt;                                     // at most 0.08 per second either way
        g_cthr = clampv(want, g_cthr - step, g_cthr + step);
    } else g_cthr = 0;
    // Descent (Damien: like hill descent control): faster than the crawl speed with the pedals free ->
    // a gentle brake request (Cruise applies it, hazards stay off), gone when you brake or give gas.
    float over = vd * 3.6f - fmaxf(p_descent_kmh, g_vt * 3.6f + 1);  // km/h above where braking starts (Damien: from 8)
    if (p_descent_brake > 0 && thr < 0.02f && brake < 0.01f && (over > 0 || (g_cbrake > 0 && over > -1)))
        g_cbrake = fminf(p_descent_brake, g_cbrake + 0.15f * in.dt);
    else g_cbrake = fmaxf(0, g_cbrake - 0.5f * in.dt);
}

static void creep_tick(const creep_in &in)
{
    float v = in.speed, vk = fabsf(v) * 3.6f, vd = in.gear < 0 ? -v : v;
    float thr = clampv(in.thr, 0, 1), brake = in.brake;
    if (!in.engine_on) g_armed = 0;
    else if (p_arm <= 0 || (in.gear != 0 && vk > p_arm)) g_armed = 1;
    if (vk > 1.5f) g_last_dir = v > 0 ? 1 : -1;
    int free = thr < 0.02f && brake < p_brake, rest = fabsf(v) < 0.15f;
    g_brake_off_s = brake < 0.01f ? g_brake_off_s + in.dt : 0;
    g_free_s = free ? g_free_s + in.dt : 0;
    int in_set = (in.gear >= 1 && in.gear <= in.set_max) || (in.gear == -1 && p_rev > 0);

    if (g_on) {
        const char *why = NULL;
        if (in.blocked) why = "blocked (parking brake, cruise, engine, hill hold, eco-roll)";
        else if (!in_set && !(in.gear == 0 && rest && wall_s() - g_on_at < 1.5)) {
            why = "gear";
            if (in.gear == 0 && rest) { why = "no gear engaged from rest"; g_rest_fail = 1; }
        }
        else if (vk > p_off) why = "speed";
        else if (thr >= p_maxped && vk > p_man) why = "throttle";  // Damien: only when you really drive on
        if (why) {
            g_on = 0;
            g_why = why;
            logline("Creep: off (%s) gear %d, %.1f km/h, throttle %.2f, floor %.2f", why, in.gear, vk, thr, g_f);
        }
    // not while braking to a stop (ts4o 654.9 s: it took over during a stop; Damien: "braking = brake, not crawl"):
    // the brake must have been off for 0.5 s before the crawl starts while rolling
    } else if (!in.blocked && in_set && vk < p_on && thr < p_maxped * 0.8f && brake < 0.01f && g_brake_off_s > 0.5f &&
               (in.gear > 0 || vd > 0.1f)) {
        g_on = 1;
        if (g_f < 0.01f && vd < 0.25f * target_speed(thr, brake)) g_f = 0.9f;  // from rest: start nearly open, close gently
        logline("Creep: on (gear %d, %.1f km/h, throttle %.2f, brake %.2f, target %.1f km/h)", in.gear, vk, thr, brake, target_speed(thr, brake) * 3.6f);
    }
    if (g_on) creep_control(in, vd, thr, brake);
    else { g_f = fmaxf(0, g_f - in.dt / fmaxf(p_release, 0.1f)); g_cthr = 0; }

    // Gear to hold at rest (the automatic would pick neutral) or to engage from neutral.
    int hold = 0;
    // from rest a low gear (ts4h: a light truck in its crawl gear 4 at idle never moved), up to the crawl gear
    // once rolling
    int start = in.crawl_gear < (in.heavy ? 1 : 2) ? in.crawl_gear : (in.heavy ? 1 : 2);
    if (p_enabled > 0 && g_on && free && in.gear < 0 && rest) hold = in.gear;
    else if (p_enabled > 0 && g_on && free && in.gear > 0 && rest) hold = in.gear > start ? start : in.gear;
    // up one gear at a time, only when this gear is at its idle lock-up (clutch nearly closed, rpm at idle) and
    // still too slow (ts4j: 1 -> 4 in one go at 3.7 km/h was a jolt)
    else if (p_enabled > 0 && g_on && free && in.gear > 0 && in.gear < in.crawl_gear) {
        static double up_at;
        int at_lockup = g_f < 0.15f && in.idle > 200 && in.rpm < 1.06f * in.idle && vd < g_vt;
        hold = in.gear;
        if (at_lockup && wall_s() - up_at > 1.0) {
            up_at = wall_s();
            hold = in.gear + 1;
        }
    }  // up to the crawl gear, never down while rolling (the game's stall rules own that)
    // from rest only with the brake fully released (ts4c: at rest with the brake at 0.2 the game put it back in
    // neutral every tick and we engaged again: a flicker); once moving the brake modulates the crawl
    else if (p_enabled > 0 && p_rest > 0 && in.can_engage && g_armed && !in.blocked && vk < p_on && free && brake < 0.05f &&
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

// ---------------------------------------------------------------- game side (1.60 layout only)

enum { S_AMODE = 0x100, S_LATCH = 0x101, S_DRIVE = 0x110, I_DRIVE = 0x150, D_ENGINE = 0x360, D_IDLE_ADD = 0x2dc, E_IDLE = 0x228, GA_HILL = 0x1d00 };

// Crawl gear: the lowest gear whose lock-up speed at idle (closed clutch, engine at idle) reaches creep_kmh, so the
// clutch slips a little and the engine stays at idle. Heavy loads are capped at heavy_gear (then the plugin adds throttle).
static int pick_gear(int heavy, float idle)
{
    int cap = (int)(heavy ? p_heavy_gear : p_max_gear), n = T.fwd_gears;
    if (cap > n) cap = n;
    if (cap < 1) cap = 1;
    float r = wheel_radius();
    if (!(r > 0.1f && idle > 200 && T.diff > 0)) return 1;
    for (int g = 1; g <= cap; g++) {
        float ratio = T.ratio[g - 1] * T.diff;
        if (ratio > 0 && idle * 0.10472f * r / ratio * 3.6f >= p_creep * 1.05f) return g;
    }
    return cap;
}

static float idle_rpm(char *inner)
{
    if (!inner || !mem_readable(inner + I_DRIVE, 8)) return 0;
    char *drive = *(char **)(inner + I_DRIVE);
    if (!drive || !mem_readable(drive + D_ENGINE, 8)) return 0;
    char *eng = *(char **)(drive + D_ENGINE);
    if (!eng || !mem_readable(eng + E_IDLE, 4) || !mem_readable(drive + D_IDLE_ADD, 4)) return 0;
    float i = *(float *)(eng + E_IDLE) + *(float *)(drive + D_IDLE_ADD);
    return i > 200 && i < 2000 ? i : 0;
}

// Runs at the end of every automatic update, right before the generic gearbox reads its target gear.
static void tail_pre(void *self)
{
    static int held;
    if (!self || self != g_self) return;
    char *s = (char *)self;
    int gear = gb_gear(self);
    if (g_hold) {
        int want = g_hold;
        if (gb_target(self) != want) gb_set_target(self, want);
        // In neutral the generic gearbox waits for throttle in its gap state: do its engage step ourselves.
        if (gear == 0 && g_engage_ok && gb_state(self) == 1 && gb_engage(self, want))
            logline("Creep: engaged gear %d without throttle (%.1f km/h)", want, fabsf(T.speed) * 3.6f);
        *(int *)(s + S_DRIVE) = want > 0 ? 4 : 0;  // drive state: forward / reverse (the game's own values)
        *(unsigned char *)(s + S_LATCH) = 0;      // not "at rest, neutral"
        held = 1;
    } else if (held) {
        held = 0;
        // Brake at a standstill in our held gear: back to what the game does at a stop (neutral, latch;
        // its next update then picks R on the brake, as it always has).
        // Also when the crawl just went off (switched off, blocked) at walking pace: neutral, so the engine
        // cannot push the truck on (Damien: it rolled with crawl off).
        if (gear > 0 && fabsf(T.speed) < 0.5f && (fmaxf(T.in_brake, T.eff_brake) >= 0.01f || !g_on)) {
            gb_set_target(self, 0);
            *(int *)(s + S_DRIVE) = 3;
            *(unsigned char *)(s + S_LATCH) = 1;
            logline("Creep: brake at rest, back to the game's stop (neutral)");
        }
    }
}

// Crawl on/off: hold the toggle button / key at a standstill (Damien 11-10: toggle, not always on).
static int g_user_on = -1;
static void crawl_toggle()
{
    if (g_user_on < 0) g_user_on = p_start_on > 0;
    if (p_tog_btn <= 0 && p_tog_key <= 0 && p_tog_mode <= 0) return;
    static double down_at, up_at;
    static int fired;
    static float mode_before = NAN;
    double now = wall_s();
    // The game's gearbox-mode key: its action object game_actor+0x1ea40 (key handler 0x14099cca6: +0xf8 == 2 on a
    // press, then g_adaptive_shift steps). ts4e: right after a press the state is 1 and +0x178 stays 0, so the
    // state is the signal: 2 = press event, 1 = held after the game took the event (GUESS: 0 when released).
    int mode_held = 0;
    if (p_tog_mode > 0) {
        char *ga = game_actor_safe(), *act = ga && mem_readable(ga + 0x1ea40, 8) ? *(char **)(ga + 0x1ea40) : NULL;
        if (act && mem_readable(act + 0xf8, 4)) {
            int st = *(int *)(act + 0xf8);
            mode_held = st != 0;
            static int last_st = -1, n_st;
            if (st != last_st && n_st < 12) { n_st++; logv(2, "Creep: mode key state %d", st); }  // check the guess
            last_st = st;
        }
        static int said;
        float a = game_adaptive();
        static float last_a = NAN;
        if (act && a == a && last_a == last_a && a != last_a && said < 3) {  // check the guess on the first presses
            said++;
            logline("Creep: mode key pressed: action state %d, value %.2f", *(int *)(act + 0xf8), *(float *)(act + 0x178));
        }
        last_a = a;
    }
    int held = mode_held || (p_tog_btn > 0 && input_held((int)p_tog_btn)) ||
               (p_tog_key > 0 && (GetAsyncKeyState((int)p_tog_key) & 0x8000));
    static double keep_until;  // after a toggle the mode is kept a moment past the release too (ts4n: N -> E)
    if (!held) {  // a held wheel button flickers under Proton: 0.3 s up ends a hold
        if (!up_at) up_at = now;
        if (fired) keep_until = now + 0.6;
        if (now - up_at > 0.3) { down_at = 0; fired = 0; }
        if (now < keep_until && mode_before == mode_before) {
            static void *cv2;
            if (!cv2) cv2 = game_cvar("g_adaptive_shift");
            float a = game_adaptive();
            if (cv2 && a == a && fabsf(a - mode_before) > 0.01f) game_cvar_set(cv2, mode_before);
        } else if (!down_at) mode_before = game_adaptive();
        return;
    }
    up_at = 0;
    if (!down_at) down_at = now;
    // after the toggle the held key keeps stepping the mode (the game repeats it, ts4g: E-P-M-E every 0.26 s):
    // keep putting the mode back until the key is released
    static void *cv;
    if (fired && mode_held && mode_before == mode_before) {
        if (!cv) cv = game_cvar("g_adaptive_shift");
        float a = game_adaptive();
        if (cv && a == a && fabsf(a - mode_before) > 0.01f) game_cvar_set(cv, mode_before);
    }
    // at a standstill or while crawling (ts4g: crawl was on and moving, so the toggle never fired)
    if (fired || now - down_at < p_tog_hold || fabsf(T.speed) * 3.6f > p_on) return;
    fired = 1;
    g_user_on = !g_user_on;
    if (mode_held && mode_before == mode_before) {  // the press stepped the gearbox mode: put it back
        if (!cv) cv = game_cvar("g_adaptive_shift");
        if (cv) game_cvar_set(cv, mode_before);
    }
    static const float on[] = { 523, 659 }, off[] = { 659, 523 };
    play_tones(g_user_on ? on : off, 2, 80);
    g_dash_flash = g_user_on ? 'C' : 'X';
    g_dash_flash_until = wall_s() + 1.5;
    logline("Creep: crawl switched %s", g_user_on ? "on" : "off");
}

// Continuity broke (pause menu, hitch, other truck, teleport, ...): no pedal timers or one-off latches may carry over.
// SOFT keeps a running crawl (the controller works from fresh speed next frame); HARD lets everything go.
static void ck_reset(int level, const char *why)
{
    g_free_s = 0;
    g_rest_fail = 0;
    g_on_at = wall_s();
    if (level >= RESET_HARD) {
        g_on = 0;
        g_hold = 0;
        g_f = g_cthr = g_cbrake = g_vt = 0;
        g_armed = 0;
        g_last_dir = 1;
        g_cf.floor = 0;
        g_cf.inner = NULL;
        g_creep_thr = 0;
        g_creep_brake = 0;
        if (g_dash_status == 'C') g_dash_status = 0;
    }
}

static void ck_frame()
{
    static double last, csv_at, log_at;
    double now = sim_s();
    float dt = (float)(now - last);
    last = now;
    if (!g_hooked) return;
    crawl_toggle();
    g_crawl_on = p_enabled > 0 && g_user_on > 0;
    void *self = game_gearbox();
    char *inner = self ? gb_inner(self) : NULL;
    g_self = self;
    g_cf.inner = NULL;  // set below only while crawling: our clutch value then replaces the game's
    if (!inner) { g_cf.floor = 0; g_f = 0; g_cthr = 0; g_creep_thr = 0; g_creep_brake = 0; g_cbrake = 0; g_on = 0; g_hold = 0; return; }
    if (dt > 0.5f) ck_reset(RESET_SOFT, "long frame");
    if (dt <= 0 || dt > 0.5f) return;  // paused or a long hitch: no time step

    char *ga = game_actor_safe();
    float hill = ga && mem_readable(ga + GA_HILL, 4) ? *(float *)(ga + GA_HILL) : 0;
    int gear = gb_gear(self), st = *(int *)((char *)self + S_DRIVE);
    // Automatic or arcade box (SDK shifter type; empty = not reported, the hooks only see the automatic anyway).
    // self+S_AMODE was 0 in Damien's automatic (ride 0.4.1: "not in automatic mode" all the time), so it is not used.
    int amode = !T.shifter[0] || !strcmp(T.shifter, "automatic") || !strcmp(T.shifter, "arcade");
    creep_in in;
    in.dt = dt;
    in.speed = T.speed;
    in.thr = T.in_thr;
    in.brake = T.in_brake;  // the driver's pedal (the effective brake also carries the game's own braking)
    in.rpm = T.rpm;
    in.idle = idle_rpm(inner);
    in.gear = gear;
    in.engine_on = T.engine_on;
    in.blocked = g_user_on <= 0 || g_truck_unsupported || T.paused || !T.engine_on || T.parking_brake || T.cruise > 0.1f || g_rolling || !amode || hill > 0 ||
                 !(in.idle > 0);
    static double mass_at = -100;
    static int heavy = 1;
    if (now - mass_at > 1) {  // load changes slowly; unknown mass counts as heavy
        mass_at = now;
        float m = game_mass();
        heavy = !(m == m) || m >= p_heavy_t * 1000;
    }
    in.heavy = heavy;
    in.set_max = (int)(heavy ? p_heavy_gear : p_max_gear);
    in.crawl_gear = pick_gear(heavy, in.idle);
    in.can_engage = g_tail_ok && g_engage_ok && ((st >= 1 && st <= 3) || gear == -1);
    int was_on = g_on;
    creep_tick(in);
    // Why not crawling while slow (ride 0.4: it never came on): every 5 s in the log.
    if (!g_on && g_user_on > 0 && T.engine_on && fabsf(T.speed) * 3.6f < p_on && fabsf(T.speed) > 0.3f) {
        static double said;
        if (now - said > 5) {
            said = now;
            const char *why = g_user_on <= 0 ? "switched off (hold the toggle at a standstill)" : T.paused ? "paused" : T.parking_brake ? "parking brake" : T.cruise > 0.1f ? "cruise on"
                            : g_truck_unsupported ? "this truck is not supported" : g_rolling ? "eco-roll" : !amode ? "gearbox not in automatic mode" : hill > 0 ? "game hill hold"
                            : !(in.idle > 0) ? "idle rpm unknown" : !(gear == 1 || gear == 2 || (gear == -1 && p_rev > 0)) ? "gear"
                            : in.thr >= p_maxped * 0.8f ? "throttle" : in.brake >= p_brake ? "brake" : "?";
            logv(2, "Creep: not on at %.1f km/h: %s (gear %d, throttle %.2f, brake %.2f, hill %.2f)", fabsf(T.speed) * 3.6f, why,
                    gear, in.thr, in.brake, hill);
        }
    }
    g_cf.floor = g_f;
    g_cf.inner = g_on ? inner : NULL;
    g_creep_thr = g_cthr;
    g_creep_brake = g_on ? g_cbrake : 0;
    if (p_letter > 0 && g_on) g_dash_status = 'C';
    else if (g_dash_status == 'C') g_dash_status = 0;

    if (g_csv && (g_on || was_on || g_f > 0) && now - csv_at >= 0.1) {
        csv_at = now;
        fprintf(g_csv, "%.2f,%.2f,%d,%.3f,%.3f,%d,%.3f,%.2f,%.0f,%.0f,%.3f,%d,%d,%d,%d,%.3f,%d,%d\n", wall_s(), T.speed * 3.6f, gear, in.thr,
                in.brake, g_on, g_f, g_vt * 3.6f, T.rpm, in.idle, T.eff_clutch, g_hold, gb_state(self), st, hill > 0, g_cthr, in.crawl_gear, heavy);
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
    if (p_enabled <= 0) { logline("Creep disabled in the ini"); snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    if (p_off < p_on + 2) { p_off = p_on + 5; logline("Creep: off_kmh must be above on_kmh, using %.0f", p_off); }  // else it flips every frame
    snprintf(g_status, sizeof g_status, "off (not the 1.60 layout)");
    if (!L || strcmp(L->name, "1.60")) { logline("Creep: only for the 1.60 layout, off"); return; }
    snprintf(g_status, sizeof g_status, "off (clutch code not found)");
    uintptr_t site = game_scan("8B 83 2C 14 00 00 89 86 30 03 00 00");
    if (!site) { logline("Creep: clutch copy not found, off"); return; }
    snprintf(g_status, sizeof g_status, "off (clutch hook failed)");
    if (!game_patch_clutch_floor(site, &g_cf)) { logline("Creep: clutch hook failed, off"); return; }
    g_hooked = 1;
    g_engage_ok = game_engage_init();
    if (G.tail_site && game_patch_tail(G.tail_site, (void *)tail_pre)) g_tail_ok = 1;
    if (!g_tail_ok) logline("Creep: update tail not found, no holding at rest / no start from rest");
    else if (!g_engage_ok) logline("Creep: gearbox engage code not found, no start from rest");
    if (p_trace > 0) {
        g_csv = open_out("creep.csv", "w");
        if (g_csv) fputs("t,kmh,gear,thr,brake,on,floor,target_kmh,rpm,idle,game_clutch,hold,gb_state,drive_state,hillhold,add_thr,crawl_gear,heavy\n", g_csv);
    }
    snprintf(g_status, sizeof g_status, "on, hook in place; crawl %s at start, from rest %s, reverse %s", p_start_on > 0 ? "on" : "off (hold the toggle)",
             p_rest > 0 && g_tail_ok && g_engage_ok ? "on" : "off", p_rev > 0 ? "on" : "off");
    logline("Creep: hook in place at %p (crawl %.1f km/h, on below %.0f, off above %.0f, from rest %s, reverse %s)", (void *)site,
            p_creep, p_on, p_off, p_rest > 0 && g_tail_ok && g_engage_ok ? "on" : "off", p_rev > 0 ? "on" : "off");
}

static void ck_stop()
{
    g_cf.floor = 0;  // give everything back
    g_creep_thr = 0;
    g_creep_brake = 0;
    g_hold = 0;
    if (g_csv) fflush(g_csv);
}

module m_creep = { "creep", "Creep", g_params, sizeof g_params / sizeof g_params[0], ck_start, ck_frame, NULL, ck_stop, ck_reset, ck_status };
