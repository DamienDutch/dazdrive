// The actuator: the one place that moves the truck's pedals for every module. A module posts a wish (a brake request, a
// throttle floor, cap or scale) under its own owner tag; the wish is held until the owner changes it; this layer merges the
// wishes once per game tick and writes them. It is hooked whenever its code patterns are found, whatever the [cruise]
// section says (26-10: until then every brake request and every throttle tweak of the suite went through the Cruise module's
// hook, so `[cruise] enabled=0` or `smooth=0` silently switched off the crawl brake, the rollover guard, Auto Hold, hill
// start, the AEB and the pedal curves).
//
// The tick is the game's cruise fn call (cruise-re notes): the game calls cruise_fn(game_actor, dt, pedal) every physics step,
// cruise on or off, and stores what it returns as the merged throttle (inner+0x1424). Our stub runs the modules' ACT_PRE hooks
// (inputs the game's controller must see), the game's own controller, the rollover guard, the ACT_POST hooks (the modules post
// their wishes; Cruise may replace the base throttle with its own controller's), writes the brake request, merges the
// throttle (base, then floors, then caps, then scales) and hands the result to the ACT_DONE hooks (traces).
//
// Brake wishes are in the game's own request unit (game_actor+0x17c, 0..1: the game slews to it and uses the engine brake and
// retarder first), not in m/s2. A hold at a standstill (hill start, Auto Hold) has no deceleration to express, and the only
// conversion the code knows is the AEB's fit on 26-30 t logs (0.4 + 10 r / (mass / 30 t) m/s2, trimmed by a slow loop on
// the measured deceleration), too rough to sit between every module and the brake. The ini keys in 0..1 stay what they are.
#include "suite.h"
#include "game.h"
#include <math.h>
#include <string.h>

// Fields of game_actor (1.60, cruise-re notes § 4) the layer writes.
enum { A_REQ = 0x17c };   // the brake request: 0..1, slewed by the game, zeroes the merged throttle while > 0 (neutral-exit-re notes § 3.1)

static const char *const k_owner[ACT_N] = { "cruise", "hill start", "Auto Hold", "bend", "ACC", "AEB", "rollover guard", "crawl", "launch control", "retarder (deceleration)", "retarder (mass)", "deceleration assist", "low-speed owner" };
static_assert(ACT_N == 13, "one initializer per owner below");
static act_wish W[ACT_N] = { { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 },
                             { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 } };
float g_air_min_psi = 75;
float g_retarder_hook;           // [core] retarder_hook
volatile float g_ret_scale = 1.0f;   // read by the drivetrain hook: the lowest retarder factor of all owners

static int owner_ok(int o) { return o >= 0 && o < ACT_N; }
void act_brake(int owner, float want) { if (owner_ok(owner)) W[owner].brake = want; }
void act_thr_floor(int owner, float v) { if (owner_ok(owner)) W[owner].floor = v; }
void act_thr_cap(int owner, float v) { if (owner_ok(owner)) W[owner].cap = v; }
void act_thr_scale(int owner, float k) { if (owner_ok(owner)) W[owner].scale = k; }
void act_retarder_scale(int owner, float f) { if (owner_ok(owner)) W[owner].ret = f != f ? 1.0f : f < 0 ? 0.0f : f > 1 ? 1.0f : f; }
const act_wish *act_wishes() { return W; }
const char *act_owner_name(int o) { return owner_ok(o) ? k_owner[o] : "?"; }

// The retarder torque factor the drivetrain hook applies (see "the retarder" below): the lowest of what the owners hold, 1 with
// nobody posting, and only to the player's vehicle (game_actor -> gearbox -> inner, the path of game_gearbox). The drivetrain runs
// this code for other vehicles too: a vehicle that is not s_rs.inner keeps its demand, and with the player's vehicle not found
// (inner NULL) every vehicle does. Another thread reads the pair: a new vehicle clears the old one first, the scale goes in before it.
static retarder_scale s_rs = { NULL, 1.0f };   // read by the drivetrain hook: player vehicle + the factor
static char *act_player_inner(char *ga)
{
    if (!ga || !L || !L->a_gearbox || !mem_readable(ga + L->a_gearbox, 8)) return NULL;
    char *gb = *(char **)(ga + L->a_gearbox);
    if (!gb || !mem_readable(gb + L->s_inner, 8)) return NULL;
    char *in = *(char **)(gb + L->s_inner);
    return in && mem_readable(in + 0x1444, 4) ? in : NULL;   // (the demand the stub reads)
}
static void act_retarder_merge(char *ga)
{
    float f = 1.0f;
    for (int o = 0; o < ACT_N; o++) if (W[o].ret < f) f = W[o].ret;
    g_ret_scale = f;
    char *in = act_player_inner(ga);
    if (s_rs.inner != in) s_rs.inner = NULL;
    s_rs.scale = in ? f : 1.0f;
    s_rs.inner = in;
}

// ---------------------------------------------------------------- the brake request
// Every brake wish meets here, once per tick, and what we wrote goes back to 0 when nobody wants it any more. The game
// never clears the request itself when its emergency brake option is off: what we write stays (ts3c: brakes on, hazards
// forced on, throttle dead, neutral-exit-re notes § 3), so there is exactly one writer and it remembers its own value.
static float s_req_ours;         // the request we hold in game_actor+0x17c (0 = none)
static int s_air_low;            // brake air low: our requests paused
static double s_br_at;           // time of the last write (slew)
static int s_hill_was, s_hold_was, s_aeb_was;   // hill start / Auto Hold / the AEB was asking at the last write (they ramp by themselves)
static int s_dec_was;            // the deceleration assist led at the last write (it ramps by itself)
static const float k_dec_slew = 20.0f;  // request per second: only a backstop above the assist's own ramps (its steps at a shift, up to max_brake 0.6, come within 0.03 s: the brake's own lag of ~0.15 s is the slew)
static unsigned char s_thr_pass;     // Auto Hold alone holds the brake: the driver's throttle still reaches the engine (patch of the game's 0x17c throttle gate)
static unsigned char s_hazard_mute;  // our request must not switch the hazards on (game rule for 0x17c > 0)
static int s_log_owner = -1;     // the owner last named in the log (-1 = nobody braking)
static double s_log_at;
float act_request() { return s_req_ours; }

void act_brake_write(char *g)
{
    float hill = W[ACT_HILL].brake, hold = W[ACT_HOLD].brake, bend = W[ACT_BEND].brake, crawl = W[ACT_CREEP].brake, acc = W[ACT_ACC].brake;
    float aeb = fmaxf(W[ACT_AEB].brake, W[ACT_RSC].brake);   // the rollover guard's brake rides on the AEB's fast request path
    float low = W[ACT_LOW].brake;                              // the low-speed owner's hold (Stop&Go behind a lead), a plain request like the ACC's
    float want = fmaxf(fmaxf(fmaxf(fmaxf(fmaxf(fmaxf(hill, hold), bend), crawl), acc), aeb), low);
    float dec = W[ACT_DECEL].brake;   // the deceleration assist (modules/decel.cpp): shapes its own ramps, see the branch below
    if (dec > 0) want = fmaxf(want, dec);
    float *req = (float *)(g + A_REQ);
    // Save the air (Damien 11-10: the tank ran empty): no brake requests of ours while the pressure is low; a
    // request that holds still (hill start) is kept, it uses no new air.
    if (T.air_warning || (T.air_psi > 1 && T.air_psi < g_air_min_psi)) {
        if (!s_air_low) {
            logline("Actuator: brake air low (%.0f psi): our braking paused", T.air_psi);
            if (fabsf(T.speed) * 3.6f > 10)  // standing after a start or a teleport the tank is simply still filling
                incident("air", "brake air low: %.0f psi at %.0f km/h, our braking paused", T.air_psi, fabsf(T.speed) * 3.6f);
        }
        s_air_low = 1;
    } else if (s_air_low && T.air_psi > g_air_min_psi + 15) { s_air_low = 0; logline("Actuator: brake air back (%.0f psi)", T.air_psi); }
    // the AEB is never paused: it brakes with what is left (and holds its request steady instead of pulsing)
    if (s_air_low && hill <= 0 && hold <= 0 && aeb <= 0) want = 0;
    // the assist leads when its wish is the biggest one; it closes a loop on the measured deceleration, where the 0.05 dead band
    // below would be a limit cycle and the 0.25 / 0.6 per second slews far too slow for a gap of 0.2 s (it ramps by itself)
    int dec_lead = dec > 0 && want > 0 && dec >= want - 1e-6f, dec_done = s_dec_was && !dec_lead;
    s_dec_was = dec_lead;
    // a small change is not worth a new brake application (each one costs air): hold it. Auto Hold shapes its own ramp.
    if (hold <= 0 && aeb <= 0 && !dec_lead && want > 0 && s_req_ours > 0 && fabsf(want - s_req_ours) < 0.05f) want = s_req_ours;
    // soft onset and release (Damien: subtler, smoother); the hill-start hold comes at once
    {
        double now = wall_s(), dt = now - s_br_at;  // the hill-start hold lets go at once too (else it fights the driver's gas)
        s_br_at = now;
        if (hill > 0) s_hill_was = 1;
        else if (aeb > 0 && aeb >= want - 1e-6f) { s_aeb_was = 1; s_hill_was = 0; }  // the AEB ramps by itself (up fast, down at its own pace)
        else if (s_aeb_was && want <= 0) s_aeb_was = 0;                  // ... its last step to 0 is its own
        else if (hold > 0) { s_hold_was = 1; s_hill_was = 0; }        // Auto Hold ramps by itself: no extra filter
        else if (s_hold_was) s_hold_was = 0;                           // ... and its last step to 0 is not dragged out either
        else if (s_aeb_was) s_aeb_was = 0;                             // a bigger wish took over from the AEB
        else if (s_hill_was) { s_hill_was = 0; want = 0; }
        else if (dec_lead) {                                           // the assist: a wide slew, its own ramps do the shaping
            if (dt > 0 && dt < 0.5) { float lim = k_dec_slew * (float)dt; want = fminf(fmaxf(want, s_req_ours - lim), s_req_ours + lim); }
        }
        else if (dec_done && want <= 0) { }                             // ... and its last step to 0 is its own
        else if (dt > 0 && dt < 0.5) {
            float up = acc > 0 || low > 0 ? 1.0f : 0.25f;  // following a lead or holding a stop behind one: firm enough, still ramped (no valve dump)
            float lo = s_req_ours - 0.6f * (float)dt, hi = s_req_ours + up * (float)dt, asked = want;
            want = want < lo ? fmaxf(lo, 0) : (want > hi ? hi : want);
            // only a fading request is cut off: a rising one starts below 0.01 (0.25/s × 1/60 s = 0.004), and
            // cutting that kept bend / roundabout braking at 0 for good (ts4q 643-648 s: 63 km/h held to 50 m before
            // a roundabout, then a hard stop to 16)
            if (want < 0.01f && asked < 0.01f) want = 0;
        }
    }
    if (want > 0) {
        if (*req < want || *req == s_req_ours) *req = want;
        s_req_ours = want;
    } else if (s_req_ours > 0) {
        if (*req == s_req_ours) *req = 0;
        s_req_ours = 0;
    }
    s_hazard_mute = s_req_ours > 0 && *req <= s_req_ours;  // a stronger request (the game's AEB) keeps its hazards
    // the driver's gas reaches the engine while only the Auto Hold asks (it lets go as the torque builds)
    s_thr_pass = s_hazard_mute && hold > 0 && hill <= 0 && bend <= 0 && crawl <= 0;
    // who brakes (for the log and the incident files): the owner with the biggest wish, named when it changes
    {
        int owner = -1;
        if (s_req_ours > 0) {
            float best = 0;
            for (int o = 0; o < ACT_N; o++) if (W[o].brake > best) { best = W[o].brake; owner = o; }
        }
        double now = wall_s();
        if (owner != s_log_owner && now - s_log_at > 1.0) {
            if (owner >= 0) logv(1, "Actuator: brake request %.2f for %s", s_req_ours, k_owner[owner]);
            else logv(1, "Actuator: brake request released (%s)", s_log_owner >= 0 ? k_owner[s_log_owner] : "?");
            s_log_owner = owner; s_log_at = now;
        }
    }
    act_retarder_merge(g);   // the retarder factors, at the same tick
}

// Our request back to 0 (what we wrote, only if it is still ours) and the writer's memory cleared.
static void release_request()
{
    if (s_req_ours > 0) {
        char *ga = game_actor_safe();
        if (ga) { float *req = (float *)(ga + A_REQ); if (*req == s_req_ours) *req = 0; }
    }
    s_req_ours = 0;
    s_hazard_mute = 0;
    s_thr_pass = 0;
}

// ---------------------------------------------------------------- the tick: the game's cruise fn call
typedef float (*cruise_fn)(void *ga, float dt, float pedal);
static cruise_fn o_cruise;         // the game's own cruise controller (returns its throttle, or the pedal with cruise off)
static int s_hooked;
static float s_base = NAN;         // this tick's base throttle from an ACT_POST hook (NAN = the game's)
enum { ACT_MAX_HOOKS = 4 };
static void (*s_fn[3][ACT_MAX_HOOKS])(act_tick *);
static int s_n[3];

void act_hook(int phase, void (*fn)(act_tick *))
{
    if (phase < 0 || phase > ACT_DONE || !fn || s_n[phase] >= ACT_MAX_HOOKS) return;
    for (int i = 0; i < s_n[phase]; i++) if (s_fn[phase][i] == fn) return;
    s_fn[phase][s_n[phase]++] = fn;
}
int act_hooked() { return s_hooked; }
void act_thr_base(float v) { s_base = v; }

static float act_stub(void *ga, float dt, float pedal)
{
    act_tick a = { (char *)ga, dt, pedal, NAN, NAN };
    s_base = NAN;
    for (int i = 0; i < s_n[ACT_PRE]; i++) s_fn[ACT_PRE][i](&a);
    a.stock = o_cruise(ga, dt, pedal);
    rsc_step(dt);   // the rollover guard (core/rsc.cpp) decides from the telemetry alone and posts its brake wish and throttle cap
    for (int i = 0; i < s_n[ACT_POST]; i++) s_fn[ACT_POST][i](&a);
    if (ga) act_brake_write((char *)ga);   // (merges the retarder factors too)
    else act_retarder_merge(NULL);
    // the throttle: the base (the game's, or what Cruise posted), then every floor, then every cap, then every scale
    float r = s_base == s_base ? s_base : a.stock;
    for (int o = 0; o < ACT_N; o++) if (W[o].floor > 0 && W[o].floor > r) r = W[o].floor;
    for (int o = 0; o < ACT_N; o++) if (W[o].cap < 1) r = fminf(r, W[o].cap);
    for (int o = 0; o < ACT_N; o++) if (W[o].scale != 1) r *= W[o].scale;
    a.thr = r;
    for (int i = 0; i < s_n[ACT_DONE]; i++) s_fn[ACT_DONE][i](&a);
    return r;
}

// ---------------------------------------------------------------- the clutch
// The drivetrain gets its clutch from one copy (inner+0x142c -> drive+0x330, 0x1408c82cf, creep-re notes); the hook
// (game_patch_clutch_floor, 1.60 only) makes that copy our value for the player's vehicle while s_cf.inner is set and leaves
// every other vehicle alone. The vehicle's own clutch state (the gearbox sequencer, the stop rule) stays as the game has it,
// unless the owner asks for a mirror: the smooth start does, so Auto Hold, TrueShift and the sequencer read its value too.
enum { V_CLUTCH = 0x142c };   // the vehicle's clutch, 0 = closed .. 1 = open
static clutch_floor s_cf;     // read by the drivetrain hook: player vehicle + our value
static int s_clutch_in;
volatile float g_clutch_ours = __builtin_nanf("");
int act_clutch_ok() { return s_clutch_in; }

void act_clutch(char *inner, float v, int mirror)
{
    s_cf.floor = v;
    s_cf.inner = inner;
    g_clutch_ours = inner ? v : NAN;
    if (mirror && inner && mem_readable(inner + V_CLUTCH, 4)) *(volatile float *)(inner + V_CLUTCH) = v;
}
// The game's own clutch until the owner posts again; the value and g_clutch_ours stay (the owner is still deciding this frame).
void act_clutch_release() { s_cf.inner = NULL; }
// The vehicle's own clutch value, once (the smooth start hands a closed clutch back when it is over); 1 = written.
int act_clutch_vehicle(char *inner, float v)
{
    if (!inner || !mem_readable(inner + V_CLUTCH, 4)) return 0;
    *(volatile float *)(inner + V_CLUTCH) = v;
    return 1;
}

// ---------------------------------------------------------------- the retarder
// The drivetrain reads the retarder demand (inner+0x1444) once for the torque it applies (0x1408c87a8, ETS2 1.60.1.7; the patch is
// game_patch_retarder, only with [core] retarder_hook = 1). Our factor scales that one read: the dashboard, the animation and the
// lever keep the game's own step. The factor reaches the player's vehicle only (s_rs, act_retarder_merge). The one owner that
// posts it is the deceleration assist (its soften and its ramp after a shift, [decel] soften and ramp_s, modules/decel.cpp).
static bool s_ret_in;
bool act_retarder_hooked() { return s_ret_in; }
// The pattern ends at the torque read after the curve call; the demand read is the 8 bytes at +7 (unique in the exe too).
static const char k_ret_pat[] = "48 8B BF D0 04 00 00 F3 0F 10 80 44 14 00 00 E8 ?? ?? ?? ?? F3 44 0F 10 87 A8 00 00 00";
enum { RET_READ_AT = 7 };

// ---------------------------------------------------------------- hooks (m_core, config.cpp)

static int s_gate_in, s_haz_in;
int act_gate_ok() { return s_gate_in; }

void act_start()
{
    // The clutch copy (1.60 only; Creep's crawl and smooth start post their clutch value through it).
    if (L && !strcmp(L->name, "1.60")) {
        uintptr_t cs = game_scan("8B 83 2C 14 00 00 89 86 30 03 00 00");
        if (!cs) logline("Actuator: clutch copy not found: no crawl, no smooth start");
        else if (!game_patch_clutch_floor(cs, &s_cf)) logline("Actuator: clutch hook failed: no crawl, no smooth start");
        else { s_clutch_in = 1; logline("Actuator: clutch hook in place at %p", (void *)cs); }
    } else logline("Actuator: no clutch hook (1.60 only): no crawl, no smooth start");
    // The retarder demand read of the drivetrain (1.60 only, off by default): our factor scales the retarder torque.
    if (g_retarder_hook >= 0.5f && L && !strcmp(L->name, "1.60")) {
        uintptr_t rs = game_scan(k_ret_pat);
        if (!rs) logline("Actuator: retarder demand read not found: retarder scale unavailable");
        else if (!game_patch_retarder(rs + RET_READ_AT, &s_rs)) logline("Actuator: retarder hook failed: retarder scale unavailable");
        else { s_ret_in = true; logline("Actuator: retarder hook in place at %p", (void *)(rs + RET_READ_AT)); }
    } else if (g_retarder_hook >= 0.5f) logline("Actuator: no retarder hook (1.60 only): retarder scale unavailable");
    // The cruise fn call in the game's pedal code (cruise-re notes; the pattern ends at the E8 of the call).
    uintptr_t h = game_scan("48 8B CB F3 44 0F 11 83 E4 1C 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8");
    uintptr_t site = h ? h + 21 : 0;
    if (site) o_cruise = (cruise_fn)(site + 5 + (intptr_t) * (const int32_t *)(site + 1));
    if (site && game_patch_call(site, (void *)act_stub)) {
        s_hooked = 1;
        logline("Actuator: throttle hook in place at %p (the game's cruise fn %p)", (void *)site, (void *)o_cruise);
    } else {
        logline("Actuator: cruise throttle call not found: no throttle or brake of ours (cruise, Auto Hold, hill start, AEB, crawl brake, rollover guard)");
        return;
    }
    // Our brake request (bends, hill start, crawl) is > 0 for a while; the game's auto-hazard rule (neutral-exit-re notes § 3)
    // would switch the hazards on every time (ts3d). Its compare is skipped while only we ask.
    uintptr_t hz = game_scan("85 C0 74 17 F6 87 D0 01 00 00 10 75 0E 44 0F 2F 8F 7C 01 00 00 73 04 B2 01");
    if (hz && game_patch_cf0_if(hz + 13, 8, &s_hazard_mute)) { s_haz_in = 1; logline("Actuator: our brake requests leave the hazards alone"); }
    else logline("Actuator: hazard rule not found: braking for bends / hill start / crawl may switch the hazards on");
    // Auto Hold: the game zeroes the merged throttle while the brake request is > 0. Skip that one compare while only
    // Auto Hold holds, so the pedal builds torque against the brake and the release can follow it (autohold-acc notes).
    uintptr_t gt = game_scan("0F 2F CF 76 ?? 83 BB 50 01 00 00 02 75 ?? 0F 2F BB 7C 01 00 00 72 0A F3 0F 11 81 24 14");
    if (gt && game_patch_cf0_if(gt + 14, 7, &s_thr_pass)) { s_gate_in = 1; logline("Actuator: throttle gate in place (the pedal reaches the engine while only Auto Hold holds)"); }
    else logline("Actuator: throttle gate not found: Auto Hold lets go at the pedal's pace only (no drive feedback)");
}

void act_stop()
{
    release_request();
    g_ret_scale = 1.0f;
    s_rs.inner = NULL; s_rs.scale = 1.0f;
}

// Continuity broke (see RESET_*): the writer's slew memory and our request go; the owners clear their own wishes, except the
// retarder factors: they go here (a factor held through a reset would weaken the retarder of whatever comes next).
void act_reset(int level, const char *why)
{
    (void)why;
    s_aeb_was = 0; s_hill_was = s_hold_was = s_dec_was = 0; s_br_at = 0;
    release_request();
    for (int o = 0; o < ACT_N; o++) W[o].ret = 1.0f;
    g_ret_scale = 1.0f;
    s_rs.inner = NULL; s_rs.scale = 1.0f;
    s_log_owner = -1; s_log_at = 0;
    if (level >= RESET_HARD) s_air_low = 0;
}

const char *act_status()
{
    static char b[260];
    snprintf(b, sizeof b, "throttle hook %s (%d/%d/%d module hooks), brake request: one writer, hazard rule %s, Auto Hold throttle gate %s, clutch hook %s, retarder hook %s",
             s_hooked ? "in" : "NOT in", s_n[ACT_PRE], s_n[ACT_POST], s_n[ACT_DONE], s_haz_in ? "patched" : "not found", s_gate_in ? "patched" : "not found",
             s_clutch_in ? "in" : "not in", s_ret_in ? "in" : "off");
    return b;
}
