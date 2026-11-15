// The vehicle model: mass, grade, engine and the forces, one estimate for every module. See vehicle.h.
#include "vehicle.h"
#include "bands.h"
#include "vclass.h"
#include "brakemodel.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

float g_brake_learn = 1;   // [core] brake_learn, ON by default (WP-M2, owner 03-11): the brake model of this file (config.cpp has the same default for the ini)
static void brake_learn_reset(int level);
static void brake_learn_step();
static void bm_reset(int level);
static void bm_step();
static void bm_save(const char *why);
static void fuel_learn_reset(int level);
static void fuel_learn_step();
static void ratio_cal_reset(int level);
static void ratio_cal_step();
static float rc_q(int g);
vehicle V = { NULL, 0, 0, -1e9, 0, 0, 0, {}, 0, 0, -1e9, 0, 0, 0.006f, 6.0f, 0.9f, 1, 0, 1, 0, 0, 0, 0, 0 };

// ---------------------------------------------------------------- the vehicle class (see vehicle.h)
float g_vehicle_class = VC_AUTO;          // [core] vehicle_class (config.cpp fills it; the words auto/truck/light map to 0/1/2)
static int s_vclass = VC_TRUCK;           // the decided class (never VC_AUTO); truck until the mass tells
static float s_vclass_own = NAN;          // the own mass the class was decided on (for the log)
static int s_vclass_known;
static float s_vclass_prov_rpm;           // the rpm limit the class was last decided provisionally on (before the mass read); 0 = none since the last reset

// The decision itself, once the game has told the mass (or a car definition). Called from the mass read; the override wins.
static void vehicle_class_decide(float own)
{
    int light;
    if (g_vehicle_class >= VC_TRUCK) light = g_vehicle_class == VC_LIGHT;   // the [core] vehicle_class override
    else if (game_vehicle_is_car() > 0) light = 1;
    else if (T.rpm_limit > 3000 && !(own >= 10000)) light = 1;   // a car or pickup engine under 10 t (Damien 31-10: trucks never rev past ~3000)
    else if (own == own && own > 0) light = own <= VEHICLE_CLASS_LIGHT_KG;
    else light = 0;
    int c = light ? VC_LIGHT : VC_TRUCK;
    if (!s_vclass_known || c != s_vclass) {
        logv(1, "Vehicle class: %s (own mass %.1f t%s, %s)", light ? "light" : "truck", own == own ? own / 1000 : 0,
             T.trailer ? "with a trailer, " : "",
             g_vehicle_class >= VC_TRUCK ? g_vehicle_class == VC_LIGHT ? "ini override light" : "ini override truck"
             : game_vehicle_is_car() > 0 ? "a car by definition" : T.rpm_limit > 3000 && !(own >= 10000) ? "a car engine (rpm limit > 3000, under 10 t)" : light ? "own mass <= 3.5 t" : "own mass > 3.5 t");
        s_vclass = c; s_vclass_known = 1;
    }
    s_vclass_own = own;
}

int vehicle_class()
{
    if (g_vehicle_class >= VC_LIGHT) return VC_LIGHT;
    if (g_vehicle_class >= VC_TRUCK) return VC_TRUCK;
    return s_vclass;
}
int vehicle_light() { return vehicle_class() == VC_LIGHT; }

// ---------------------------------------------------------------- the module set and the profile of a class (see vclass.h)
// The class the module set runs for. It follows vehicle_class() only in vclass_sync(), which the dispatcher calls once a frame (right after
// the vehicle model), so a module and its hooks never see the class change in the middle of a frame. A truck until the model tells.
static int s_vc_active = VC_TRUCK;

const vclass_profile *vprof() { return s_vc_active == VC_LIGHT ? &k_vprof_car : &k_vprof_truck; }
int vclass_active() { return s_vc_active; }
int vclass_sync()
{
    int c = vehicle_class();
    if (c == s_vc_active) return 0;
    s_vc_active = c;
    return 1;
}
int module_active(const module *m)
{
    unsigned mask = m && m->classes ? m->classes : (unsigned)VCM_BOTH;
    return (mask & (s_vc_active == VC_LIGHT ? VCM_CAR : VCM_TRUCK)) != 0;
}

float road_limit_ms()
{
    if (!vprof()->car_speed_limit) return T.speed_limit;
    float c = game_car_speed_limit_ms();
    return c == c && c > 1.0f ? c : 0.0f;
}
float vehicle_own_mass() { return s_vclass_own; }

// ---------------------------------------------------------------- an engine definition that gives less than the automatic needs (WP-TC1)
//
// A mod engine can define `torque:` and nothing else (ride 04-11, the Spectrans Scania: dc13_450.sii has no torque_curve, no rpm_range_* and no rpm_limit). The game's
// automatic then runs on its own defaults, and so must the model: without them the engine read failed, V.eng_ok stayed 0 and TrueShift was blind for the whole ride
// (every shift was the game's, "no data"). The defaults are the game's documented ones (notes/game-defs.md, tools/gamedefs/parse_defs.py): the torque curve (a share
// of the torque value, in absolute rpm, so the read torque scales it and the limit only cuts it), the four shift ranges and the rpm limit. What the definition does give
// stays as read; only what is missing or insane is replaced, per part, and the engine's own learned correction (f_scale, f_bias) takes up the rest.
static const float k_def_curve[7][2] = { { 300, 0 }, { 440, 0.5f }, { 1000, 1 }, { 1300, 1 }, { 1900, 0.77f }, { 2400, 0.4f }, { 2600, 0 } };
static const float k_def_range[4][2] = { { 1500, 2500 }, { 700, 1500 }, { 1000, 1350 }, { 1400, 1900 } };   // R_BRAKE, R_LOW, R_HIGH, R_POWER (the game's order of the doc: low, high, power, brake)
static const float k_def_limit = 2500;
static char s_eng_read[200];   // what the read of the definition held, for the line that names what was missing
static unsigned s_eng_logged; static float s_eng_logged_torque;   // the line is said once per engine and per set of missing parts (reset with the truck)

static int range_sane(const float *r) { return r[0] > 200 && r[1] > r[0] && r[1] < 10000; }

// The raw read of the engine object (game.cpp gb_engine; the sims' stubs call it too): which parts hold nothing usable. 1 = the torque is usable, so the rest can be stood in for.
int engine_raw_check(engine_info *e)
{
    e->missing = 0;
    if (!(e->torque > 50 && e->torque < 20000)) { snprintf(s_eng_read, sizeof s_eng_read, "torque %.1f (unusable)", e->torque); return 0; }   // nothing to stand in for
    if (!(e->rpm_limit > 800 && e->rpm_limit < 10000)) e->missing |= EM_LIMIT;
    for (int r = 0; r < 4; r++) if (!range_sane(e->range[r])) e->missing |= EM_RANGE0 << r;
    if (e->n_curve < 2) e->missing |= EM_CURVE;
    snprintf(s_eng_read, sizeof s_eng_read, "read: torque %.0f, limit %.0f, %d curve points, ranges brake %.0f-%.0f low %.0f-%.0f high %.0f-%.0f power %.0f-%.0f",
             e->torque, e->rpm_limit, e->n_curve, e->range[0][0], e->range[0][1], e->range[1][0], e->range[1][1], e->range[2][0], e->range[2][1], e->range[3][0], e->range[3][1]);
    if (e->electric && e->missing) return 0;   // (an electric motor has no gearbox to manage: no defaults of a combustion engine for it, it stays as it was)
    return 1;
}

// The game's defaults for what engine_raw_check found missing. sdk_limit = the limit the SDK reports for the truck (T.rpm_limit; the memory's may be what is missing).
// 1 = the engine is usable now.
int engine_fill_defaults(engine_info *e, float sdk_limit)
{
    if (e->missing & EM_LIMIT) e->rpm_limit = sdk_limit > 800 && sdk_limit < 10000 ? sdk_limit : k_def_limit;
    if (e->missing & EM_CURVE) {
        e->n_curve = 7;
        for (int i = 0; i < 7; i++) { e->curve[i][0] = k_def_curve[i][0]; e->curve[i][1] = k_def_curve[i][1]; }
    }
    for (int r = 0; r < 4; r++)
        if (e->missing & (EM_RANGE0 << r)) {
            e->range[r][0] = k_def_range[r][0];
            e->range[r][1] = fminf(k_def_range[r][1], e->rpm_limit);   // (a range above the limit means nothing: the brake range ends at the limit)
        }
    if (!(e->rpm_limit > 800 && e->rpm_limit < 10000) || e->n_curve < 2) return 0;
    for (int r = 0; r < 4; r++) if (!range_sane(e->range[r])) return 0;
    return 1;
}

// ---------------------------------------------------------------- the converter's stall ratio, read from the game (WP-TC2, owner decision 61)
//
// The transmission accessory object (the one whose float at L->t_shift_time the plugin already reads) holds, in the 12 bytes that end at it, stall_torque_ratio (-8), the
// differential_ratio (-4) and the shift time (live RE, ETS2 1.61, the Spectrans Scania's Allison 4000: 2.42, 3.91, 1.0). Nothing marks the layout, but the differential beside
// the stall is a figure the plugin also has (the telemetry's, the truck config): a stall is trusted only where that diff equals it. The SDK reports the diff as the same
// 32-bit float the object holds, so a true layout matches to the bit; 0.005 only leaves room for a mod's rounding of it (the diffs of the game's trucks are 2.5 .. 6, and two
// different ones differ by more than a hundredth). A diff the config has not told yet (0) cannot check anything, and a zeroed object must not match it.
static char s_tc_note[160];     // the line for this read (engine_read_stall); vehicle_update logs it once per truck
static char s_tc_logged[160];
static int s_tc_note_final;     // 0 = the truck config was not known yet: the read says nothing yet, no line

float trans_stall_read(const char *tr, unsigned off, float tel_diff)
{
    s_tc_note_final = 1;
    if (!tr || off < 8) { snprintf(s_tc_note, sizeof s_tc_note, "transmission: stall torque ratio not read (no transmission object in this layout)"); return NAN; }
    if (!(tel_diff > 0.0f)) { s_tc_note_final = 0; snprintf(s_tc_note, sizeof s_tc_note, "transmission: stall torque ratio not read yet (the truck config has no diff)"); return NAN; }
    float stall, diff;
    memcpy(&stall, tr + off - 8, 4);
    memcpy(&diff, tr + off - 4, 4);
    if (!(fabsf(diff - tel_diff) <= 0.005f)) {
        snprintf(s_tc_note, sizeof s_tc_note, "transmission: stall torque ratio not trusted (the diff beside it reads %.3f, the truck config's is %.3f: another layout)", diff == diff ? diff : 0.0f, tel_diff);
        return NAN;
    }
    if (!(stall >= 0.0f && isfinite(stall))) {   // (a ratio is not negative or infinite; a matching diff makes anything else a number we can use as it is)
        snprintf(s_tc_note, sizeof s_tc_note, "transmission: stall torque ratio not trusted (diff %.3f checks, but the stall reads %g)", tel_diff, stall == stall ? stall : 0.0f);
        return NAN;
    }
    snprintf(s_tc_note, sizeof s_tc_note, stall > 1.0f ? "transmission: stall torque ratio %.2f (diff %.2f checks)" : "transmission: stall torque ratio %.2f (diff %.2f checks): no converter", stall, tel_diff);
    return stall;
}

void engine_read_stall(engine_info *e, const char *tr, unsigned off, float tel_diff)
{
    const float s = trans_stall_read(tr, off, tel_diff);
    e->tc_stall = s == s ? fmaxf(s, 1.0f) : 0.0f;   // (a trusted 0 or 1 is "no converter": 1, no multiplication; an untrusted read is 0, as a zeroed engine_info says)
}

// Start of the torque plateau: first curve point within 3% of the peak.
static float find_plateau(const engine_info *e)
{
    float peak = 0;
    for (int i = 0; i < e->n_curve; i++) peak = fmaxf(peak, e->curve[i][1]);
    for (int i = 0; i < e->n_curve; i++)
        if (e->curve[i][1] >= 0.97f * peak && e->curve[i][0] > 300) return e->curve[i][0];
    return e->range[R_HIGH][0];
}

// The engine's peak power as torque factor x rpm (the unit of the power curve here: share of the peak torque times rpm). Computed once per engine read.
static float find_peak_pw(const engine_info *e)
{
    float best = 0;
    for (float r = 100; r <= e->rpm_limit; r += 5) best = fmaxf(best, curve_factor(e, r) * r);
    return best;
}

void vehicle_mass_stale() { V.mass_at = -1e9; }

static void tc_stall_reset();

// Continuity broke (core/telemetry.cpp session_reset): trailer (de)coupled (LOAD: the mass, and what was learned against
// it); other truck, save/load, teleport (HARD: the engine too, read again at the next update, and the rpm correction).
void vehicle_reset(int level)
{
    brake_learn_reset(level);
    bm_reset(level);
    fuel_learn_reset(level);
    ratio_cal_reset(level);
    double none = -1e9;
    if (level >= RESET_LOAD) {
        V.mass_at = none;
        V.eng_at = none;   // a load, a teleport, a trailer: read the engine again too (cheap; the config event covers the rest)
        V.f_bias = 0;
    }
    if (level >= RESET_HARD) {
        s_vclass_prov_rpm = 0;   // the next truck's class is decided afresh, from its engine, before its mass is read
        s_eng_logged = 0;        // ... and the line about its engine's missing parts is said again
        s_tc_logged[0] = 0;      // ... and the line about its transmission's stall ratio (WP-TC2)
        V.self = NULL;
        V.mass = 0; V.mass_ok = 0;
        V.eng_ok = 0;
        V.eng_at = none;
        V.rpm_k = 1;
        V.f_scale = 1;   // another truck: what was learned against its engine is void
        V.tc = 0; V.tc_stall = 0; V.tc_lock_gear = 0; V.tc_stall_rpm = 0; V.tc_stall_fixed = 0; V.tc_lock_fixed = 0;   // the converter is re-detected on the next truck
        tc_stall_reset();
    }
}

void vehicle_update(void *gb)
{
    double now = sim_s();
    if (gb && gb != V.self) {
        if (V.self) vehicle_reset(RESET_HARD);   // another gearbox object: another truck
        V.self = gb;
    }
    // The class from the 'truck config' event (the engine's rpm limit), without waiting for the mass: after a load or a reset the first seconds are not
    // truck rules (ride 31-10, the ATS pickup: truck until the mass read, a cruise capped at the truck's 65 mph). The mass read decides again, as before
    if (!V.mass_ok && T.rpm_limit > 3000 && T.rpm_limit != s_vclass_prov_rpm) {   // (only a car-like engine decides light early; the rest waits for the mass as before)
        s_vclass_prov_rpm = T.rpm_limit;
        vehicle_class_decide(NAN);
    }
    if (now - V.mass_at > 0.5 || now < V.mass_at) {
        float m = game_mass();
        if (m == m) {
            if (!(V.mass > 0) || fabsf(m - V.mass) > 0.03f * V.mass)
                logv(1, "mass %.1f t (trailer %s, cargo %.1f t)", m / 1000, T.trailer ? "yes" : "no", T.cargo_mass / 1000);
            V.mass = m;
            V.mass_ok = 1;
            vehicle_class_decide(game_mass_own());   // the class follows the mass (and the car definition); once per 0.5 s at most
        } else if (!(V.mass > 0)) V.mass = VEHICLE_MASS_FALLBACK;
        V.mass_at = now;
    }
    // the engine changes only with the truck configuration (another truck, a dealer swap): read it then, plus a slow check every 30 s
    if (gb && ((!V.eng_ok && now - V.eng_at > 0.5) || V.eng_gen != T.cfg_gen || now - V.eng_at > 30.0 || now < V.eng_at)) {   // (an engine that never reads sane: twice a second, not every frame)
        engine_info e;
        memset(&e, 0, sizeof e);
        s_tc_note[0] = 0;
        int ok = gb_engine(gb, &e);
        V.electric = e.electric;
        // the transmission's stall ratio (WP-TC2): ONE line per truck, and none while the truck config has not told its diff (the check has nothing to compare with; the config event re-reads)
        if (ok && s_tc_note[0] && s_tc_note_final && strcmp(s_tc_note, s_tc_logged)) { snprintf(s_tc_logged, sizeof s_tc_logged, "%s", s_tc_note); logline("%s", s_tc_logged); }
        // a mod engine that defines less than the game's automatic needs (its torque only): the game's own defaults for the rest, and ONE line saying what was missing
        if (ok && e.missing) {
            const int miss = e.missing;
            char rd[sizeof s_eng_read]; snprintf(rd, sizeof rd, "%s", s_eng_read);
            ok = engine_fill_defaults(&e, T.rpm_limit);
            if (s_eng_logged != (unsigned)miss + 1u || s_eng_logged_torque != e.torque) {
                s_eng_logged = (unsigned)miss + 1u; s_eng_logged_torque = e.torque;
                char what[120] = ""; size_t k = 0;
                if (miss & EM_CURVE) k += snprintf(what + k, sizeof what - k, "%storque curve", k ? ", " : "");
                if (miss & EM_LIMIT) k += snprintf(what + k, sizeof what - k, "%srpm limit", k ? ", " : "");
                for (int r = 0; r < 4; r++) if (miss & (EM_RANGE0 << r)) k += snprintf(what + k, sizeof what - k, "%s%s range", k ? ", " : "", r == R_BRAKE ? "engine brake" : r == R_LOW ? "low gear" : r == R_HIGH ? "high gear" : "power");
                logline("engine definition gives %.0f N m but no %s: the game's own defaults stand in for it (limit %.0f, torque plateau from %.0f rpm), %s; %s",
                        e.torque, what, e.rpm_limit, find_plateau(&e), ok ? "TrueShift decides on them" : "STILL NOT USABLE", rd);
            }
        }
        // another engine: what was learned against the old one is void (not after one failed read of the same engine: V.eng keeps the last good one)
        if (ok && (e.torque != V.eng.torque || e.rpm_limit != V.eng.rpm_limit)) { V.f_bias = 0; V.f_scale = 1; }
        if (ok) { V.eng = e; V.plateau = find_plateau(&e); V.peak_pw = find_peak_pw(&e); bands_refresh(&e); }
        V.eng_ok = ok;
        V.eng_at = now;
        V.eng_gen = T.cfg_gen;
    }
    if (gb) {
        float gd = game_grade_deg(gb);
        V.grade_deg = gd == gd ? gd : 0;
        V.grade_pct = tanf(V.grade_deg * 0.0174533f) * 100;
    }
    // Grade (%) from the SDK placement: pitch in turns, positive = nose up.
    V.pitch_pct = tanf((float)T.place.orientation.pitch * 6.2831853f) * 100;
}

void vehicle_frame()
{
    vehicle_update(game_gearbox());
    ratio_cal_step();
    if (g_brake_learn > 0) { brake_learn_step(); bm_step(); }
    fuel_learn_step();
}

// ---------------------------------------------------------------- gear, speed, forces

static int n_gears()
{
    int n = T.fwd_gears;
    return n > 0 && n < 32 ? n : 0;
}

// Engine rpm in gear g at speed v (m/s), from the driveline, corrected by the
// measured ratio to the real rpm.
float vehicle_rpm_at(int g, float v)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0) return 0;
    return v / r * T.ratio[g - 1] * T.diff * 9.5493f * V.rpm_k * (V.tc ? rc_q(g) : 1.0f);
}

// The modelled driveline rpm per m/s of a gear before the converter box's calibration (the ratios, the final drive, the wheel radius, the tire
// correction): what vehicle_ratio_sample compares the measured rpm with.
float vehicle_ratio_base(int g)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0 || !(T.ratio[g - 1] > 0)) return 0;
    return T.ratio[g - 1] * T.diff * 9.5493f / r * V.rpm_k;
}

float vehicle_kmh_at(int g, float rpm)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0 || !(T.ratio[g - 1] > 0) || !(V.rpm_k > 0)) return 0;
    return rpm / (T.ratio[g - 1] * T.diff * 9.5493f * V.rpm_k * (V.tc ? rc_q(g) : 1.0f)) * r * 3.6f;
}

float vehicle_kmh_idle(int g, float idle_rpm, float margin_rpm) { return vehicle_kmh_at(g, idle_rpm + margin_rpm); }

// Tractive force (N) in gear g at speed v with a share of the available torque.
static float tc_couple_rpm(float thr);
// The gears over the converter's lock gear that the gear choice judges as coupled (vehicle_tc_coupled_from, set by gc_choose around one decision) and that are skip landings: no slip, no multiplication, the
// engine at the driveline's rpm. 0 = the model as it was everywhere (the launch, the detection, every other reader). The ride of 06-11 (the MAN with the Allison, lock gear 1): the
// model held the engine of a landing in 4th at 41 km/h at the 1053 rpm of the stall line of a 0.82 throttle (and multiplied the torque 1.4 times) where the game ran 756 rpm
// coupled, so 3rd (1104 rpm) and 4th looked equal in the band and force terms and the skip saved a shift: 2 > 4 into the lug band, 9 of 10 such skips taken back within 3 s.
// Only the SKIP landings (from_gear = the gear we are in + 2 and up): the gear we are in and the next one keep the model as it was (a single step is what the box always did, and the
// launch rows and the lift's reserve are tuned on it).
static int s_coupled_from = 0, s_coupled_min = 0;
static int tc_open(int g) { return V.tc && V.tc_stall > 1.0f && !(s_coupled_from > 0 && g > s_coupled_from && g >= s_coupled_min); }
void vehicle_tc_coupled_from(int lock_gear, int from_gear) { s_coupled_from = lock_gear; s_coupled_min = from_gear; }
float vehicle_force_at(int g, float v, float share)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f) return 0;
    float rpm = vehicle_rpm_at(g, v);
    float t = V.eng.torque * curve_factor(&V.eng, rpm) * share;
    if (rpm > V.eng.rpm_limit) t = 0;
    float mult = 1.0f;
    if (tc_open(g)) {   // below the coupling the converter multiplies, and the engine runs above the driveline
        float couple = tc_couple_rpm(share);
        float sr = couple > 0 ? fminf(1.0f, rpm / couple) : 1.0f;   // (a standstill, rpm 0, is full stall: SR 0, the multiplication at its largest -- it read as coupled, no multiplication and the curve's 0 below 300 rpm, so a gear at rest could carry nothing)
        if (sr < 1.0f) {
            float erpm = fmaxf(rpm, couple);
            t = V.eng.torque * curve_factor(&V.eng, erpm) * share;
            if (erpm > V.eng.rpm_limit) t = 0;
            mult = V.tc_stall + (1.0f - V.tc_stall) * sr;   // the torque ratio K(slip): the stall ratio at stall, 1 at coupling
        }
    }
    return t * mult * T.ratio[g - 1] * T.diff * V.eta * V.f_scale / r;
}

// Air drag area: the truck value from the ini, a car-like one for light vehicles.
static float cda() { return V.mass > 0 && V.mass < 4500 ? 1.0f : V.cda; }

float vehicle_resist(float v)
{
    float s = sinf(V.grade_deg * 0.0174533f);
    return V.mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * cda() * v * v + V.f_bias;
}

float vehicle_resist_grade(float v, float pct)
{
    float s = sinf(atanf(pct / 100));
    return V.mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * cda() * v * v + V.f_bias;
}

float vehicle_accel(int g, float v, float share, float pct)
{
    if (!(V.mass > 0)) return 0;
    return (vehicle_force_at(g, v, share) - (pct == pct ? vehicle_resist_grade(v, pct) : vehicle_resist(v))) / V.mass;
}

// ---------------------------------------------------------------- torque converter

int vehicle_tc() { return V.tc; }
float vehicle_tc_stall() { return V.tc_stall > 1.0f ? V.tc_stall : 1.0f; }
int vehicle_tc_lock_gear() { return V.tc_lock_gear; }

// The engine's stall speed: the rpm the converter holds the engine at with the turbine stopped and the pedal floored. The def's
// own value where it is known (the sims set it); else a sensible point on the engine: 85 % of the power range's bottom, never under
// the plateau (the engine must make near-peak torque there).
float vehicle_tc_stall_rpm()
{
    if (V.tc_stall_rpm > 0) return V.tc_stall_rpm;
    return fmaxf(V.eng.range[R_POWER][0] * 0.85f, V.plateau);
}

// The engine speed the converter holds when the turbine is slower, at a throttle thr (0..1): from idle with the pedal shut to the
// stall speed floored. The slip ratio SR = turbine rpm / this, the torque ratio K = stall + (1 - stall) * SR (the stall ratio at
// stall, 1 once the driveline catches up). A light throttle couples early (a cruise in the top gear is not kept slipping and
// multiplying), a floored pedal stalls it.
static float tc_couple_rpm(float thr)
{
    float stall = vehicle_tc_stall_rpm();
    float idle = 550.0f;
    return idle + (stall - idle) * (thr < 0 ? 0 : thr > 1 ? 1 : thr);
}

// The converter's slip ratio SR (0 at stall .. 1 coupled) in gear g at speed v; 1 = coupled (or not a converter).
float vehicle_tc_slip(int g, float v)
{
    if (!tc_open(g)) return 1.0f;
    float couple = tc_couple_rpm(T.eff_thr), rpm = vehicle_rpm_at(g, v);
    if (!(couple > 0) || !(rpm > 0)) return 1.0f;
    return fminf(1.0f, rpm / couple);
}

// What the driver's own foot pulls in gear g at speed v: the raw pedal's throttle through the model (T.in_thr). The torque pedal's throttle passes the pedal through while the
// converter slips (modules/pedal.cpp, "shift": the engine is off the driveline's rpm), so a slipping gear delivers this and not the shaped demand (WP-0911-K1 round 4).
float vehicle_foot_force_at(int g, float v)
{
    if (!(T.in_thr > 0.02f)) return vehicle_force_at(g, v, 1.0f);   // (no foot on it: nothing is passed through, the throttle is somebody else's)
    return vehicle_force_at(g, v, T.in_thr > 1 ? 1.0f : T.in_thr);
}

int vehicle_tc_locked(int g, float v)
{
    if (vehicle_tc_slip(g, v) < 1.0f) return 0;
    return V.tc_lock_gear <= 0 ? 0 : g >= V.tc_lock_gear;   // a converter that never locks, or a gear under the lock gear, is never "locked"
}

float vehicle_slip_rpm()
{
    const eng_bands *b = bands_now();
    if (V.tc || !b->ok || !(b->idle > 200.0f)) return 0.0f;
    return CLUTCH_SLIP_K * b->idle;
}
int vehicle_slipping()
{
    if (V.tc || T.gear < 1 || !(T.rpm > 0.0f) || !(vehicle_slip_rpm() > 0.0f)) return 0;
    const float drv = vehicle_rpm_at(T.gear, fabsf(T.speed));
    const eng_bands *b = bands_now();
    return T.rpm > 1.05f * drv + 20.0f && T.rpm > 1.1f * b->idle && T.rpm < 1.6f * vehicle_slip_rpm() ? 1 : 0;   // (the engine above the wheels, and not a free rev: the clutch holds it near the slip rpm)
}
float vehicle_engine_rpm_at(int g, float v)
{
    float rpm = vehicle_rpm_at(g, v);
    if (!tc_open(g)) return rpm;
    return fmaxf(rpm, tc_couple_rpm(T.eff_thr));   // below the coupling the engine sits at the throttle-scaled stall speed
}

// THE converter test (header). The gear's most force at this speed is the full pedal's, with the converter's multiplication at the slip the gear lands in (vehicle_force_at: K = stall +
// (1 - stall) SR against the stall speed); the road is what holds the speed (vehicle_resist: the grade of now, the air, the learned bias), the wish what the pedal asks. The gear carries
// when it gives more than the larger of the two: a wish under the road's resistance is a gear that holds the speed, a wish over it one that also pulls. With the pedal off the wish is 0 and
// only the road is asked: a converter does not stall in a gear that holds a coast (the engine idles under the slip), so there is no floor for it. The lockup is the model's: a gear
// under the lockup gear (0 = not learned: none locks), or any gear while the converter is slipping.
int vehicle_tc_carries(int g, float v, float wish_n)
{
    if (!V.tc || !(V.tc_stall > 1.0f) || g < 1 || g > n_gears()) return 0;
    // Locked is the model's lockup (vehicle_tc_locked) with a deadband: a gear that has only just coupled (the driveline within a tenth over the rpm the pedal holds the engine at) is
    // still the converter's. A lockup clutch does not engage on the coupling point itself, it would cycle around it; and a landing that the converter carried a moment ago must not be
    // refused the moment it couples (sim: 1 > 2 at 11 km/h accepted, 2nd coupled at 15 km/h on 687 rpm under the 720 floor, 2 > 3 > 2).
    if (V.tc_lock_gear > 0 && g >= V.tc_lock_gear && vehicle_tc_slip(g, v) >= 1.0f && vehicle_rpm_at(g, v) >= 1.1f * tc_couple_rpm(T.eff_thr)) return 0;
    return vehicle_force_at(g, v, 1.0f) > fmaxf(wish_n, vehicle_resist(v));
}

void vehicle_tc_prime(float stall, int lock_gear, float stall_rpm)
{
    V.tc = stall > 1.0f;
    V.tc_stall = stall > 1.0f ? stall : 0.0f;
    V.tc_stall_fixed = stall > 1.0f;   // (as set: a figure given is not learned over)
    V.tc_lock_fixed = lock_gear > 0;
    V.tc_lock_gear = lock_gear;
    V.tc_stall_rpm = stall_rpm > 0 ? stall_rpm : 0.0f;
}

// ---------------------------------------------------------------- the stall torque ratio, learned from the launches (WP-TC1, owner decision 61)
//
// "Stall ratio from the ini is exactly what we do not want: with every other truck you would have to enter it again, I want it dynamic." The ini value counts when it is set (> 1,
// V.tc_stall_fixed); else the ratio is learned while the converter slips, from what the launches pull. In the model (vehicle_force_at) the engine's torque is multiplied by
// K = stall + (1 - stall) SR, SR = the driveline's rpm over the rpm the converter holds the engine at (tc_couple_rpm): so with F1 = the engine force at K = 1,
//     F_pull - F1 = (stall - 1) F1 (1 - SR)             (the converter adds that, and only that, below the coupling)
// is a line through the origin in x = F1 (1 - SR), and the slope is stall - 1. F_pull = m a + the resistance, from the telemetry's acceleration, which is a 0.5 s first-order lag
// of the true one: F1 and the resistance are put through the same lag, so a launch (the force and SR move fast) is compared at the same time, not 0.5 s apart. Normalised by the
// lagged F1 (a 10 t and a 41 t launch weigh by their time, not their force), one least-squares slope over everything seen, started from a prior (2.0, or what the file of the last drive
// had) worth PRIOR_S seconds of a typical launch (1 - SR about 0.5): a few launches move it, one noisy second does not.
// Samples: the clutch closed (a converter has none to slip), in gear and steady in it, no brake, no engine brake, the pedal on (the throttle is the engine's torque; at 0 there is
// nothing to multiply) and SR under 0.9 (near 1 the added force is under the noise of the acceleration).
// Bounds: 1.3 .. 3.0. The def's own list of Allison options is 2.42 / 2.34 / 1.9 / 1.79 / 1.58, a car's converter is 1.7 .. 2.2, a heavy-duty one tops at ~2.8, and under 1.3 the
// converter multiplies too little to be told from the model's noise; a figure outside is the model's error (the throttle-to-torque curve of the game, a wrong mass) showing in the
// fit, not a converter, and the clamp keeps it from steering the shift logic.
static const float TCS_MIN = 1.3f, TCS_MAX = 3.0f, TCS_PRIOR = 2.0f;
static const double TCS_PRIOR_S = 0.125;   // the prior's weight in the fit's units (seconds x (1 - SR)^2): 0.5 s at (1 - SR) = 0.5
static struct { double sxx, sxy, at, settle, brake_at; float xl, zl, rl, prior; int g, init, loaded; } SL;

static void tc_stall_reset() { memset(&SL, 0, sizeof SL); }

// Called by the gear brains with every update (sm_learn_rpm, core/shiftmodel.cpp; clutch = clutch_seen(): the model layer has no such view of its own). A second call in the same frame is nothing.
void vehicle_tc_stall_step(float clutch)
{
    const double now = sim_s();
    const double dt = now - SL.at;
    if (dt <= 0 && SL.at > 0) return;
    SL.at = now;
    if (!V.tc || V.tc_stall_fixed || !(V.tc_stall > 1.0f)) { SL.init = 0; return; }
    if (!SL.loaded) { SL.loaded = 1; SL.prior = V.tc_stall; }   // the figure the converter came with (the prior of 2.0, or what the last drive stored)
    const int g = T.gear;
    const float v = fabsf(T.speed), r = wheel_radius();
    if (!(dt > 0 && dt < 0.25) || g < 1 || g > n_gears() || !(r > 0.1f) || !(V.mass > 0) || T.paused || T.parking_brake || clutch > 0.02f) { SL.init = 0; return; }
    if (T.in_brake > 0.02f || T.eff_brake > 0.02f || T.motor_brake || T.retarder > 0) SL.brake_at = now;   // (a brake takes force the model does not count: its 1.5 s are out)
    const float thr = T.eff_thr;
    const float couple = tc_couple_rpm(thr), rpm_d = vehicle_rpm_at(g, v);   // (standing: the driveline does not turn, SR is 0 and the lags below run on from the pedal)
    if (!(couple > 0)) { SL.init = 0; return; }
    const float sr = fminf(1.0f, rpm_d / couple);
    const float erpm = fmaxf(rpm_d, couple);
    float t1 = V.eng.torque * curve_factor(&V.eng, erpm) * thr;
    if (erpm > V.eng.rpm_limit) t1 = 0;
    const float f1 = t1 * T.ratio[g - 1] * T.diff * V.eta * V.f_scale / r;   // the engine force at K = 1
    const float x1 = f1 * (1.0f - sr), r1 = vehicle_resist(v);
    if (!SL.init || g != SL.g) {   // a new stretch (another gear, the clutch was open, a pause): the lags start from now, and 1.5 s (3 lags) are given to settle (the shift's torque gap)
        SL.init = 1; SL.g = g; SL.xl = x1; SL.zl = f1; SL.rl = r1; SL.settle = now + 1.5; SL.brake_at = fmax(SL.brake_at, 0.0);
        return;
    }
    const float k = (float)(dt / (ACCEL_LAG_S + dt));   // the telemetry acceleration's own lag
    SL.xl += k * (x1 - SL.xl); SL.zl += k * (f1 - SL.zl); SL.rl += k * (r1 - SL.rl);
    if (now < SL.settle || now - SL.brake_at < 1.5 || !(v > 0.3f) || !(thr > 0.12f) || !(sr < 0.9f) || !(SL.zl > 500.0f)) return;
    const double y = ((double)V.mass * T.accel + SL.rl - SL.zl) / SL.zl, x = (double)SL.xl / SL.zl;   // normalised by the lagged F1
    // A sample whose own figure is over twice the largest the clamp allows is the model's miss, not a converter (noise on the low side is symmetric and stays): the ride of 04-11 (41.5 t, +5.5 %, pedal 0.23, 3.6 .. 8 km/h)
    // pulls 6 .. 12 times the model's engine force at that throttle (the game's throttle-to-torque and the real stall speed are not the model's), and the fit would sit on its clamp.
    // Those samples are left out: the figure stays where it was (the prior, or what the other launches said).
    if (!(x > 0.05) || 1.0 + y / x > 2.0 * TCS_MAX) return;
    SL.sxx += dt * x * x; SL.sxy += dt * x * y;
    const double est = (SL.sxy + TCS_PRIOR_S * (SL.prior - 1.0)) / (SL.sxx + TCS_PRIOR_S);
    V.tc_stall = fminf(fmaxf(1.0f + (float)est, TCS_MIN), TCS_MAX);
}

// ---------------------------------------------------------------- the effective ratios of a converter box (ratio_cal.txt)
//
// A torque-converter box has no clutch to tell the driveline's rpm from the engine's: the engine sits ABOVE the driveline while the converter
// pulls (ride 31-10, ATS Crown Victoria: 1.03-1.05 times the ratios' rpm per speed in 1st and 2nd at a steady pedal) and BELOW it while
// the pedal is up (0.89-0.96, falling with the speed, and never under ~870 rpm: the idle in gear). The ratios, the final drive and the wheel
// radius the SDK reports say what the driveline does; the rpm the shift logic predicts for a gear at a speed has to be the driveline's
// (what that gear will turn at, once the converter is out of the way), and the tire correction (rpm_k) must never absorb the converter's slip:
// it is not learned on a converter at all. Instead each gear's effective ratio is measured against the modelled one (q = measured rpm /
// modelled rpm per speed) from the two sides that bracket the true figure: coasting with the pedal up and rolling (the converter is
// driven by the wheels, a little under 1:1), and a steady moderate pull (the converter drives, a little over). A gear with both sides takes
// their mean; a gear with one takes it less the offset the calibrated gears show for that side (a prior of 5 % counts as one gear); a gear
// with none takes the mean of the calibrated ones (1 when there are none: the ratios as the SDK gives them). TrueShift feeds the samples (it knows the shifts and the
// pedal); the table is kept per truck (the key of brake_gain.txt) and read back when the converter is detected.
enum { RC_N = 24, RC_COAST = 0, RC_PULL = 1 };
static const float RC_MIN_S[2] = { 2.0f, 4.0f };      // seconds of samples a side needs to count
static const float RC_PRIOR_COAST = 0.07f, RC_PRIOR_PULL = 0.05f;   // the prior offset of a coast (under) and a pull (over) from the driveline, as a share (rides 31-10: coast 0.88-0.96, pull 1.03-1.05)
static const float RC_W_MAX = 20.0f;                  // seconds the average remembers (an EMA of about this)
static const char *const RC_FILE = "ratio_cal.txt";
static struct {
    char key[96]; int loaded, dirty, primed;
    double save_at;
    float qs[2][RC_N], ws[2][RC_N];   // per side and gear: the measured / modelled ratio (1 = none seen) and the seconds behind it
    float q[RC_N];                    // the table the model reads (1 = as modelled)
    int ready[RC_N];                  // 1 = this gear's figure is its own (both sides), 2 = one side, 0 = a fallback
    float mean;                       // the mean q of the calibrated gears (1 = none)
    int tc_stored; float tc_stall; int tc_lock;   // the converter the last drive of this truck found (the line's tc=), until the next detection says it again
    int saved_tc, saved_lock;         // what the file holds of it
    float saved_stall;                // ... and its stall ratio (learned from the launches, it moves)
    float idle, saved_idle;           // the idle rpm TrueShift learned for this truck (idle=), 0 = none
} RC = { "", 0, 0, 0, 0, {}, {}, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, {}, 1.0f, 0, 0, 0, 0, 0, 0, 0 };

static void rc_table()
{
    int n = T.fwd_gears > 0 && T.fwd_gears < RC_N ? T.fwd_gears : RC_N - 1;
    // (what the one ride this is read from showed, a coast ~5 % under the driveline and a pull ~4-5 % over, stands for one gear before the first
    // calibrated one says better: a gear with a coast alone is not left 5 % low)
    float off[2] = { -RC_PRIOR_COAST, RC_PRIOR_PULL }; int no = 1;
    for (int g = 1; g <= n; g++) {
        bool c = RC.ws[RC_COAST][g] >= RC_MIN_S[RC_COAST], p = RC.ws[RC_PULL][g] >= RC_MIN_S[RC_PULL];
        if (c && p) {
            float m = 0.5f * (RC.qs[RC_COAST][g] + RC.qs[RC_PULL][g]);
            RC.q[g] = m; RC.ready[g] = 1;
            off[RC_COAST] += RC.qs[RC_COAST][g] - m; off[RC_PULL] += RC.qs[RC_PULL][g] - m; no++;
        } else RC.ready[g] = 0;
    }
    float sum = 0; int cnt = 0;
    for (int g = 1; g <= n; g++) {
        if (RC.ready[g] == 1) { sum += RC.q[g]; cnt++; continue; }
        bool c = RC.ws[RC_COAST][g] >= RC_MIN_S[RC_COAST], p = RC.ws[RC_PULL][g] >= RC_MIN_S[RC_PULL];
        if (c) { RC.q[g] = RC.qs[RC_COAST][g] - off[RC_COAST] / no; RC.ready[g] = 2; }      // (a coast alone: the calibration's own side, less its offset)
        else if (p) { RC.q[g] = RC.qs[RC_PULL][g] - off[RC_PULL] / no; RC.ready[g] = 2; }
        if (RC.ready[g] == 2) { RC.q[g] = fminf(1.2f, fmaxf(0.8f, RC.q[g])); sum += RC.q[g]; cnt++; }
    }
    RC.mean = cnt ? sum / cnt : 1.0f;   // (every calibrated gear, one side too: gears 3 and 4 are measured rarely, and the box's other gears say what its converter does)
    for (int g = 1; g <= n; g++) if (RC.ready[g] == 0) RC.q[g] = RC.mean;
    for (int g = n + 1; g < RC_N; g++) RC.q[g] = 1.0f;
}

static float rc_q(int g) { return g >= 1 && g < RC_N && RC.q[g] > 0 ? RC.q[g] : 1.0f; }
float vehicle_ratio_q(int g) { return V.tc ? rc_q(g) : 1.0f; }
int vehicle_ratio_ready(int g) { return V.tc && g >= 1 && g < RC_N ? RC.ready[g] : 0; }

static void rc_zero()
{
    for (int s = 0; s < 2; s++) for (int g = 0; g < RC_N; g++) { RC.qs[s][g] = 1.0f; RC.ws[s][g] = 0; }
    for (int g = 0; g < RC_N; g++) { RC.q[g] = 1.0f; RC.ready[g] = 0; }
    RC.mean = 1.0f; RC.dirty = 0;
    RC.tc_stored = 0; RC.tc_stall = 0; RC.tc_lock = 0; RC.saved_tc = 0; RC.saved_lock = 0; RC.saved_stall = 0; RC.idle = RC.saved_idle = 0;
}

// The idle rpm of this truck as TrueShift learned it in an earlier drive (0 = none). A car that never stands for the 5 s the learner wants (ride
// 31-10 21:00: 550 all session, the lug floor 900 with the real idle at 840) starts the next drive with the right one.
float vehicle_idle_stored() { return RC.idle; }
void vehicle_idle_note(float idle)
{
    if (!(idle > 300 && idle < 2000) || !RC.loaded) return;
    if (fabsf(idle - RC.idle) > 15) { RC.idle = idle; RC.dirty = 1; }
}

// The converter the last drive of this truck found, for a start that knows it from the first launch (the detection needs a second of slip, and the
// first upshifts of a launch were a manual box's). 1 = stored (stall ratio and the lock gear).
int vehicle_tc_stored(float *stall, int *lock)
{
    if (!RC.tc_stored) return 0;
    *stall = RC.tc_stall; *lock = RC.tc_lock;
    return 1;
}
void vehicle_tc_stored_prime(float stall, int lock) { RC.tc_stored = stall > 1.0f; RC.tc_stall = stall; RC.tc_lock = lock; RC.primed = 1; }   // sims

// A sample of the measured driveline rpm of gear g at speed v (m/s) with dt seconds behind it; side RATIO_COAST or RATIO_PULL (the caller has
// checked that the state is the right one: pedal up or steady, no shift, rpm steady). The measured figure is the engine's own rpm.
void vehicle_ratio_sample(int g, float v, float rpm, float dt, int side)
{
    if (!V.tc || g < 1 || g >= RC_N || side < 0 || side > 1 || !(dt > 0 && dt < 0.5f) || !(v > 1) || !(rpm > 0)) return;
    float base = vehicle_ratio_base(g);
    if (!(base > 0)) return;
    float q = rpm / (v * base);
    if (side == RC_COAST ? !(q > 0.7f && q < 1.12f) : !(q > 0.88f && q < 1.15f)) return;
    float &w = RC.ws[side][g], &m = RC.qs[side][g];
    m = w <= 0 ? q : m + (q - m) * dt / (fminf(w, RC_W_MAX) + dt);
    w = fminf(w + dt, 2 * RC_W_MAX);
    RC.dirty = 1;
    int was = RC.ready[g];
    rc_table();
    if (RC.ready[g] == 1 && was != 1) logv(1, "Ratio calibration: gear %d effective ratio x%.3f (coast %.3f, pull %.3f)", g, RC.q[g], RC.qs[RC_COAST][g], RC.qs[RC_PULL][g]);
}

void vehicle_ratio_prime(int g, float q)   // sims: a gear's figure as if calibrated
{
    if (g < 1 || g >= RC_N) return;
    RC.qs[RC_COAST][g] = RC.qs[RC_PULL][g] = q; RC.ws[RC_COAST][g] = RC.ws[RC_PULL][g] = 99;
    RC.primed = 1;
    rc_table();
}

static void rc_summary(const char *why)
{
    char b[320] = "";
    int n = T.fwd_gears > 0 && T.fwd_gears < RC_N ? T.fwd_gears : 0, k = 0;
    for (int g = 1; g <= n && k < (int)sizeof b - 40; g++) k += snprintf(b + k, sizeof b - k, "%s%d: x%.3f%s", g > 1 ? ", " : "", g, RC.q[g], RC.ready[g] == 1 ? "" : RC.ready[g] == 2 ? " (one side)" : " (assumed)");
    logv(1, "Ratio calibration (%s) for '%s': %s", why, RC.key, b);
}

static void rc_key()
{
    if (RC.primed || !V.eng_ok || T.fwd_gears <= 0) return;
    char k[96];
    snprintf(k, sizeof k, "E%.0f-%.0f-G%d-W%d", V.eng.torque, V.eng.rpm_limit, T.fwd_gears, T.wheels);
    if (RC.loaded && !strcmp(k, RC.key)) return;
    snprintf(RC.key, sizeof RC.key, "%s", k);
    RC.loaded = 1;
    rc_zero();
    FILE *f = open_out(RC_FILE, "r");
    if (!f) { logv(1, "Ratio calibration: nothing stored for '%s' yet (the ratios as the game reports them)", RC.key); return; }
    char buf[4096];
    int n = (int)fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", RC.key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            for (int g = 1; g < RC_N; g++) {
                char tag[12]; snprintf(tag, sizeof tag, " g%d=", g);
                const char *q = strstr(ln, tag);
                float qc = 0, qp = 0, wc = 0, wp = 0;
                if (q && sscanf(q + strlen(tag), "%f,%f,%f,%f", &qc, &qp, &wc, &wp) == 4 && qc > 0.7f && qc < 1.3f && qp > 0.7f && qp < 1.3f && wc >= 0 && wp >= 0) {
                    RC.qs[RC_COAST][g] = qc; RC.qs[RC_PULL][g] = qp;
                    RC.ws[RC_COAST][g] = fminf(wc, RC_W_MAX); RC.ws[RC_PULL][g] = fminf(wp, RC_W_MAX);   // (what was learned weighs as the seconds it came from, up to the average's span)
                }
            }
            float ts = 0; int tl = 0;
            const char *t = strstr(ln, " tc=");
            if (t && sscanf(t + 4, "%f,%d", &ts, &tl) == 2 && ts > 1.0f && ts < 6.0f && tl >= 0 && tl <= RC_N) {
                RC.tc_stored = 1; RC.tc_stall = ts; RC.tc_lock = tl; RC.saved_tc = 1; RC.saved_lock = tl; RC.saved_stall = ts;
            }
            float id = 0;
            const char *ip = strstr(ln, " idle=");
            if (ip && sscanf(ip + 6, "%f", &id) == 1 && id > 300 && id < 2000) { RC.idle = RC.saved_idle = id; }
            rc_table();
            rc_summary("loaded");
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Ratio calibration: nothing stored for '%s' yet (the ratios as the game reports them)", RC.key);
}

static void rc_save(const char *why)
{
    if (!RC.loaded || !RC.dirty || !RC.key[0]) return;
    char buf[4096], out[4096]; int n = 0, m = 0, lines = 0;
    FILE *f = open_out(RC_FILE, "r");
    if (f) { n = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", RC.key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + (int)strlen(ln) + 2 < 3300) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "truck=%s v=1", RC.key);
    if (V.tc && V.tc_stall > 1.0f) m += snprintf(out + m, sizeof out - m, " tc=%.2f,%d", V.tc_stall, V.tc_lock_gear);
    else if (RC.tc_stored) m += snprintf(out + m, sizeof out - m, " tc=%.2f,%d", RC.tc_stall, RC.tc_lock);
    if (RC.idle > 300) m += snprintf(out + m, sizeof out - m, " idle=%.0f", RC.idle);
    for (int g = 1; g < RC_N; g++)
        if (RC.ws[RC_COAST][g] > 0 || RC.ws[RC_PULL][g] > 0)
            m += snprintf(out + m, sizeof out - m, " g%d=%.4f,%.4f,%.1f,%.1f", g, RC.qs[RC_COAST][g], RC.qs[RC_PULL][g], RC.ws[RC_COAST][g], RC.ws[RC_PULL][g]);
    m += snprintf(out + m, sizeof out - m, "\n");
    RC.save_at = wall_s();
    f = open_out(RC_FILE, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Ratio calibration: cannot write %s (the figures stay in memory)", RC_FILE); return; }
    size_t w = fwrite(out, 1, (size_t)m, f);
    fclose(f);
    if (w == (size_t)m) { RC.dirty = 0; RC.saved_idle = RC.idle; if (V.tc) { RC.saved_tc = 1; RC.saved_lock = V.tc_lock_gear; RC.saved_stall = V.tc_stall; } rc_summary(why); }
}

static void ratio_cal_step()
{
    if (!RC.loaded && (V.tc || vehicle_light())) rc_key();   // (a truck without a converter never reads or writes the file)
    if (V.tc && RC.loaded && (!RC.saved_tc || RC.saved_lock != V.tc_lock_gear || (!V.tc_stall_fixed && fabsf(V.tc_stall - RC.saved_stall) > 0.05f))) RC.dirty = 1;   // (a converter found, its lock gear learned, its stall ratio moved by 0.05)
    if (!V.tc) return;
    if (RC.loaded && RC.dirty && wall_s() - RC.save_at > 60) rc_save("periodic");
}

static void ratio_cal_reset(int level)
{
    if (level < RESET_HARD) return;
    if (RC.loaded) { RC.save_at = -1e9; rc_save("reset"); }
    RC.loaded = RC.primed = 0; RC.key[0] = 0;
    rc_zero();
}

// ---------------------------------------------------------------- the service brake's gain (B1 of the dynamic-threshold audit)
//
// A brake request (0..1, game_actor+0x17c or the driver's pedal) gives a deceleration that depends on the truck, its brakes and the
// load; modules used to guess it. Here it is measured: in a steady stretch of braking (one request, the clutch and the gear not
// moving, the grade steady, no throttle, no retarder or engine brake) the force the brakes gave is what the measured deceleration
// needs on top of the model's resistance:  gm = (m a_measured - vehicle_resist) / request  (N per unit request).
// It is kept as a force (gain x mass), not as a deceleration, so a load change keeps it valid (the brakes' torque does not follow the
// cargo); the trailer's brakes are another set, so a truck alone and with a trailer each get a slot. The engine's drag in gear is
// not modelled: measured in the game it is ~0.03 m/s2 (a coast in gear on the flat is 0.17 m/s2 in all, nearly all of it rolling and
// air, brake-measure notes), far under the 1 m/s2 or more of a stretch, so a closed clutch and an open one count alike.
// One window = 1.0 s after 0.4 s of settling in the same state; a window that breaks (request, clutch, gear, grade, mass, yaw, low brake air, speed
// floor, a pedal, the retarder) is thrown away. Windows of one stretch count 0.3 event each after the first: n is in events.
// A sample outside 1.2-16 m/s2 per request (at the truck's mass) or over 7.5 m/s2 of deceleration is not taken (a slip, ABS, a hill the
// model does not know: the telemetry has no wheel slip to ask).
// The filter is robust: the first three events give the median; later a window that differs from the estimate by more than 35 %
// is an outlier and counts only if four in a row agree with each other (then the brakes changed: a physics mod, a new trailer
// brake) and replace the estimate. An accepted window moves the estimate by max(1/(n+1), 0.12). Confidence =
// min(1, n / 5) x (1 - spread / 0.3), spread = the running mean of |sample / estimate - 1|; the gain is offered from 0.5 (4 clean
// events). Slots persist in brake_gain.txt per truck (engine, gears, wheels), as lane keeps steer_profile.txt, so the next session
// starts confident.
struct bg_slot { float gm, dev, n; float first[3]; int nf; float rej[4], rejt[4]; int nr; int dirty; float told; int was_conf; float tc; int tn; };
// tc = the brake temperature the accepted samples had (a running mean over tn of them, this session; not saved). A hot brake may give less
// (fade): a sample well above it is left out, and outliers taken while hot do not count as 'the brakes changed'. Relative to the truck's own
// cool samples, no fixed degree figure: whether the game fades at all is for the logs to show.
static struct {
    char key[96]; int loaded, primed;
    bg_slot s[2];
    int cur;                     // slot of the window (0 truck alone, 1 with a trailer)
    double last, save_at;
    // the window
    double settle, wt;           // seconds in the same state before the window / in it
    float r_ref, v0, vmid, rsum, fsum, cl_ref, grade0, mass0;
    int mid_done;
    int gear0, first_in_stretch, in_stretch;
} B;

static const float BG_SETTLE_S = 0.4f, BG_WIN_S = 1.0f, BG_MIN_R = 0.1f, BG_MIN_V = 3.0f, BG_MAX_A = 7.5f, BG_MIN_NET = 0.3f;
static const float BG_GAIN_LO = BM_SVC_GAIN_LO, BG_GAIN_HI = BM_SVC_GAIN_HI;
static const float BG_OUTLIER = 0.35f, BG_CONF_MIN = 0.5f;
static const char *const BG_FILE = "brake_gain.txt";

static float bg_conf(const bg_slot &s)
{
    if (s.n <= 0) return 0;
    return fminf(1.0f, s.n / 5.0f) * fmaxf(0.0f, 1.0f - s.dev / 0.3f);
}

// The truck changed (or the first frame): its key, and what the file knows about it.
static void bg_key()
{
    if (B.primed || !V.eng_ok || T.fwd_gears <= 0) return;
    char k[96];
    snprintf(k, sizeof k, "E%.0f-%.0f-G%d-W%d", V.eng.torque, V.eng.rpm_limit, T.fwd_gears, T.wheels);
    if (B.loaded && !strcmp(k, B.key)) return;
    snprintf(B.key, sizeof B.key, "%s", k);
    B.loaded = 1;
    memset(B.s, 0, sizeof B.s);
    FILE *f = open_out(BG_FILE, "r");
    if (!f) { logv(1, "Brake gain: nothing stored for '%s' yet", B.key); return; }
    char buf[4096];
    int n = (int)fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", B.key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            for (int t = 0; t < 2; t++) {
                char tag[8]; snprintf(tag, sizeof tag, " t%d=", t);
                const char *q = strstr(ln, tag);
                float gm = 0, dev = 0, cnt = 0;
                if (q && sscanf(q + 4, "%f,%f,%f", &gm, &dev, &cnt) == 3 && gm == gm && dev == dev && cnt == cnt &&
                    gm > 1000 && gm < 2e6f && dev >= 0 && dev < 1 && cnt > 0) {
                    B.s[t].gm = gm; B.s[t].dev = fminf(dev, 0.3f); B.s[t].n = fminf(cnt, 8.0f); B.s[t].nf = 3;
                    B.s[t].told = gm; B.s[t].was_conf = bg_conf(B.s[t]) >= BG_CONF_MIN;
                }
            }
            logv(1, "Brake gain: loaded for '%s': %.0f N per request alone (%.1f events), %.0f with a trailer (%.1f events)", B.key, B.s[0].gm, B.s[0].n, B.s[1].gm, B.s[1].n);
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Brake gain: nothing stored for '%s' yet", B.key);
}

// One line per truck, a few lines in all (never MB); the other trucks' lines are kept. Errors are logged once and otherwise ignored.
static void bg_save(const char *why)
{
    if (!B.loaded || !(B.s[0].dirty || B.s[1].dirty)) return;
    char buf[4096], out[4096]; int n = 0, m = 0, lines = 0;
    FILE *f = open_out(BG_FILE, "r");
    if (f) { n = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", B.key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + (int)strlen(ln) + 2 < 3300) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "truck=%s v=1", B.key);
    for (int t = 0; t < 2; t++) if (B.s[t].n > 0) m += snprintf(out + m, sizeof out - m, " t%d=%.0f,%.3f,%.1f", t, B.s[t].gm, B.s[t].dev, B.s[t].n);
    m += snprintf(out + m, sizeof out - m, "\n");
    B.save_at = wall_s();
    f = open_out(BG_FILE, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Brake gain: cannot write %s (the estimate stays in memory)", BG_FILE); return; }
    size_t w = fwrite(out, 1, m, f);
    fclose(f);
    if (w == (size_t)m) { B.s[0].dirty = B.s[1].dirty = 0; logv(2, "Brake gain: saved (%s)", why); }
}

static void bg_break() { B.settle = 0; B.wt = 0; B.in_stretch = 0; }

// A window's sample x (N per request) into slot t; first = the first window of its stretch.
static const float BG_HOT = 1.25f;   // a sample is hot above this x the accepted samples' temperature
static void bg_add(int t, float x, int first, float temp)
{
    bg_slot &s = B.s[t];
    int known = temp == temp && temp > 0;
    if (known && s.tn >= 3 && temp > BG_HOT * s.tc) {
        static double told_at = -1e9;
        if (sim_s() - told_at > 60) { told_at = sim_s(); logv(2, "Brake gain: sample left out, the brakes are hot (%.0f C, the learned samples had %.0f C): %.0f N per request", temp, s.tc, x); }
        return;
    }
    float add = first ? 1.0f : 0.3f;
    if (s.nf < 3) {                                  // the first events: the median of them (a later window of the same stretch refines its event)
        if (first || s.nf == 0) s.first[s.nf++] = x;
        else s.first[s.nf - 1] = 0.5f * (s.first[s.nf - 1] + x);
        float a[3] = { s.first[0], s.nf > 1 ? s.first[1] : s.first[0], s.nf > 2 ? s.first[2] : s.first[0] };
        if (a[0] > a[1]) { float w = a[0]; a[0] = a[1]; a[1] = w; }
        if (a[1] > a[2]) { float w = a[1]; a[1] = a[2]; a[2] = w; }
        if (a[0] > a[1]) { float w = a[0]; a[0] = a[1]; a[1] = w; }
        s.gm = s.nf == 2 ? 0.5f * (s.first[0] + s.first[1]) : a[1];
        float d = 0;
        for (int i = 0; i < s.nf; i++) d = fmaxf(d, fabsf(s.first[i] / s.gm - 1));
        s.dev = s.nf > 1 ? d : 0.15f;
        s.n += add;
        s.dirty = 1;
        if (known) { s.tn++; s.tc += (temp - s.tc) / (float)s.tn; }
        return;
    }
    float rel = x / s.gm - 1;
    if (fabsf(rel) > BG_OUTLIER) {
        if (s.nr < 4) { s.rejt[s.nr] = known ? temp : 0; s.rej[s.nr++] = x; }
        else { memmove(s.rej, s.rej + 1, 3 * sizeof(float)); memmove(s.rejt, s.rejt + 1, 3 * sizeof(float)); s.rej[3] = x; s.rejt[3] = known ? temp : 0; }
        if (s.nr >= 4) {
            float lo = s.rej[0], hi = s.rej[0], mean = 0, tmean = 0;
            for (int i = 0; i < 4; i++) { lo = fminf(lo, s.rej[i]); hi = fmaxf(hi, s.rej[i]); mean += 0.25f * s.rej[i]; tmean += 0.25f * s.rejt[i]; }
            if (hi - lo <= 0.15f * mean && mean < s.gm && s.tn >= 3 && tmean > 1.1f * s.tc) {
                logv(1, "Brake gain: 4 weaker samples (%.0f -> %.0f N per request) while the brakes were warmer (%.0f C against %.0f C): not taken as new brakes", s.gm, mean, tmean, s.tc);
                s.nr = 0;
            } else if (hi - lo <= 0.15f * mean) {
                logv(1, "Brake gain: the brakes changed (%.0f -> %.0f N per request, %.0f C): the estimate starts over", s.gm, mean, tmean);
                s.gm = mean; s.n = 3; s.dev = 0.15f; s.nr = 0; s.dirty = 1;
            }
        }
        return;
    }
    s.nr = 0;
    if (known) { s.tn++; s.tc += (temp - s.tc) / (float)(s.tn < 20 ? s.tn : 20); }
    float a = fmaxf(1.0f / (s.n + 1.0f), 0.12f);
    s.gm += a * (x - s.gm);
    s.dev += 0.25f * (fabsf(rel) - s.dev);
    s.n = fminf(s.n + add, 100.0f);
    s.dirty = 1;
}

static void bg_announce(int t)
{
    bg_slot &s = B.s[t];
    int conf = bg_conf(s) >= BG_CONF_MIN;
    if (conf && (!s.was_conf || fabsf(s.gm / fmaxf(s.told, 1.0f) - 1) > 0.10f)) {
        logv(1, "Brake gain: %.0f N per request %s (%.1f m/s2 at %.1f t, %.1f events, spread %.0f %%)", s.gm, t ? "with a trailer" : "alone",
             V.mass > 0 ? s.gm / V.mass : 0, V.mass / 1000, s.n, s.dev * 100);
        s.told = s.gm;
        B.save_at = -1e9;     // saved at the next step
    }
    s.was_conf = conf;
}

static void brake_learn_reset(int level)
{
    bg_break();
    B.last = 0;
    if (level >= RESET_HARD) {
        if (g_brake_learn > 0) { B.save_at = -1e9; bg_save("reset"); }
        B.loaded = B.primed = 0; B.key[0] = 0;
        memset(B.s, 0, sizeof B.s);
    }
}

static void brake_learn_step()
{
    double now = sim_s(), dt = now - B.last;
    B.last = now;
    if (!B.loaded) bg_key();
    if (B.loaded && (B.s[0].dirty || B.s[1].dirty) && wall_s() - B.save_at > 30) bg_save("periodic");
    int tr = T.trailer ? 1 : 0;
    if (tr != B.cur) { bg_break(); B.cur = tr; }
    float v = T.speed;
    // (with the brake channel's pedal provider mapping the pedal the wheels get T.eff_brake, not the raw pedal T.in_brake)
    float r = g_brake_pedal_mapped ? fmaxf(T.eff_brake, act_request()) : fmaxf(fmaxf(T.in_brake, T.eff_brake), act_request());
    int ok = !(g_brake_fed > 0) && dt > 0 && dt < 0.25 && B.loaded && V.mass_ok && V.self && !T.paused && !T.parking_brake && !T.air_warning && !(T.air_psi > 1 && T.air_psi < 60) && T.gear >= 0 && T.dgear >= 0 &&
             v == v && v >= BG_MIN_V && T.retarder == 0 && !T.motor_brake && fmaxf(T.in_thr, T.eff_thr) < 0.02f && fabsf(T.yaw) < 0.04f &&
             fabsf(V.grade_deg) < 12.0f && r >= BG_MIN_R && r <= 1.0f;
    if (!ok) { bg_break(); return; }
    float cl = T.eff_clutch;
    if (B.settle < BG_SETTLE_S) {
        // settling: the request, the clutch and the gear must stay put before a window opens
        if (B.settle == 0 || fabsf(r - B.r_ref) > 0.03f + 0.05f * B.r_ref || fabsf(cl - B.cl_ref) > 0.03f || T.gear != B.gear0) {
            B.settle = 0; B.r_ref = r; B.cl_ref = cl; B.gear0 = T.gear; B.in_stretch = 0;
        }
        B.settle += dt;
        if (B.settle >= BG_SETTLE_S) {
            B.wt = 0; B.v0 = v; B.mid_done = 0; B.rsum = B.fsum = 0; B.grade0 = V.grade_deg; B.mass0 = V.mass;
            B.first_in_stretch = !B.in_stretch;
        }
        return;
    }
    // inside a window: anything that moved throws it away
    if (fabsf(r - B.r_ref) > 0.03f + 0.05f * B.r_ref || fabsf(cl - B.cl_ref) > 0.03f || T.gear != B.gear0 ||
        fabsf(V.grade_deg - B.grade0) > 0.4f || fabsf(V.mass - B.mass0) > 0.03f * B.mass0) { bg_break(); return; }
    B.wt += dt;
    B.rsum += r * (float)dt;
    B.fsum += vehicle_resist(v) * (float)dt;
    if (!B.mid_done && B.wt >= 0.5f * BG_WIN_S) { B.vmid = v; B.mid_done = 1; }
    if (B.wt < BG_WIN_S) return;
    float a = (B.v0 - v) / (float)B.wt, rm = B.rsum / (float)B.wt, fres = B.fsum / (float)B.wt;
    float net = V.mass * a - fres;                       // the force the brakes gave
    float x = net / rm, gain = x / V.mass;
    // the two halves must agree (a brake that gave way in the middle -- a slip, a patch of ice, an ABS cycle -- is not a sample)
    float a1 = (B.v0 - B.vmid) / (0.5f * BG_WIN_S), a2 = (B.vmid - v) / (0.5f * BG_WIN_S);
    if (a > 0 && fabsf(a1 - a2) <= 0.12f * a + 0.15f && a <= BG_MAX_A && net / V.mass >= BG_MIN_NET && gain >= BG_GAIN_LO && gain <= BG_GAIN_HI) {
        bg_add(B.cur, x, B.first_in_stretch, T.brake_temp);
        bg_announce(B.cur);
        B.in_stretch = 1;
    } else B.in_stretch = 0;
    B.first_in_stretch = 0;
    // the next window follows at once (the state is settled)
    B.wt = 0; B.v0 = v; B.mid_done = 0; B.rsum = B.fsum = 0; B.grade0 = V.grade_deg; B.mass0 = V.mass;
}

float vehicle_brake_conf()
{
    if (!(g_brake_learn > 0) || !B.loaded) return 0;
    return bg_conf(B.s[T.trailer ? 1 : 0]);
}

// The mass the model sizes by: the game's figure, else the cautious fallback (a heavier truck brakes less per unit of mass above the rated one)
static float bg_mass() { return V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK; }

float vehicle_brake_prior() { return bm_svc_prior(bg_mass()); }

// The estimate and the learned slot's share in it. The slot counts from its first event (the weight ramps with its confidence up to 0.5, where it
// counted in full before) times its plausibility against the prior (a slot well ABOVE the prior is down-weighted, see core/brakemodel.h).
static void bg_blend(float *est, float *w)
{
    float prior = vehicle_brake_prior();
    *est = prior; *w = 0;
    if (!(g_brake_learn > 0) || !B.loaded || !V.mass_ok || !(V.mass > 0)) return;
    const bg_slot &s = B.s[T.trailer ? 1 : 0];
    if (!(s.n > 0)) return;
    float learned = s.gm / V.mass;
    float wt = bm_svc_weight(bg_conf(s), bm_svc_plaus(learned, prior, s.n));
    *w = wt; *est = prior + wt * (learned - prior);
}

float vehicle_brake_est() { float e, w; bg_blend(&e, &w); return e; }
float vehicle_brake_weight() { float e, w; bg_blend(&e, &w); return w; }
float vehicle_brake_cautious() { return fminf(vehicle_brake_est(), vehicle_brake_prior()); }
float vehicle_brake_usable() { return fminf(vehicle_brake_est(), BM_PLAN_DECEL_MS2); }

float vehicle_brake_gain()
{
    if (!(g_brake_learn > 0) || !B.loaded || !V.mass_ok || !(V.mass > 0)) return NAN;
    const bg_slot &s = B.s[T.trailer ? 1 : 0];
    if (bg_conf(s) < BG_CONF_MIN) return NAN;
    return vehicle_brake_est();
}

float vehicle_brake_req(float decel, float fallback)
{
    float g = vehicle_brake_gain();
    if (g != g) return fallback;
    float r = decel / g;
    return r < 0 ? 0 : r > 1 ? 1 : r;
}

float vehicle_brake_hold_req(float grade_pct, float fallback)
{
    float g = vehicle_brake_gain();
    if (g != g) return fallback;
    // holding with margin: twice the pull of the slope plus 0.8 m/s2 (a push, the grade sensor's error), never under 0.2 and never over 1
    float need = 2.0f * 9.81f * fabsf(sinf(atanf(grade_pct / 100))) + 0.8f;
    float r = need / g;
    return r < 0.2f ? 0.2f : r > 1 ? 1 : r;
}

void vehicle_brake_prime(int trailer, float gm, float dev, float n)
{
    bg_slot &s = B.s[trailer ? 1 : 0];
    s = bg_slot();
    s.gm = gm; s.dev = dev; s.n = n; s.nf = 3; s.was_conf = bg_conf(s) >= BG_CONF_MIN; s.told = gm;
    B.loaded = B.primed = 1;
}

// ---------------------------------------------------------------- the retarder and the engine brake per stage, the natural deceleration (WP-M2.2-M2.4)
//
// A stage's force is its start value (core/brakemodel.h) times the stage's learned correction (what was measured over what the start value says).
// The corrections are kept per truck in brake_model.txt, one line per truck under the key of brake_gain.txt and fuel_learn.txt:
//   truck=E<torque>-<rpm_limit>-G<gears>-W<wheels> v=1 rs=<retarder stages> r<i>=<correction>,<events> ... es=<engine brake stages> e<i>=<correction>,<events> ...
// (only the stages with events). The correction belongs to the stage count it was learned under: a line with another count (a retarder option, another
// engine brake) is not used, and a token that is not a number in range makes its whole part fall back to the start values. A line is never longer than
// ~200 bytes; the file keeps the other trucks' lines (the last 16, under 3300 bytes).
static const char *const BM_FILE = "brake_model.txt";
static struct {
    char key[96]; int loaded, dirty;
    double save_at;
    int rsteps, ecap;               // the stage counts the live corrections stand for (0 = not known yet)
    bm_stages ret, eb;              // live
    int s_rsteps, s_ecap;           // as stored for this truck: used when the live count turns out to be the same
    bm_stages s_ret, s_eb;
} BMS;


// One part of a line ('r' retarder, 'e' engine brake): the stage count and its corrections. 1 = fine or absent, 0 = a bad token.
static int bm_parse(const char *ln, char tag, int *count, bm_stages *out, float lo, float hi)
{
    *count = 0; bm_stages_reset(out);
    char key[8]; snprintf(key, sizeof key, " %cs=", tag);
    const char *q = strstr(ln, key);
    if (!q) return 1;
    int n = 0;
    if (sscanf(q + 4, "%d", &n) != 1 || n < 1 || n > BM_STAGES_MAX) return 0;
    for (int i = 1; i <= n; i++) {
        char t[8]; snprintf(t, sizeof t, " %c%d=", tag, i);
        const char *p = strstr(ln, t);
        if (!p) continue;
        float c = 0, e = 0;
        if (sscanf(p + 4, "%f,%f", &c, &e) != 2 || !(c == c) || !(e == e) || c < lo || c > hi || e <= 0 || e > 100) return 0;
        out->c[i - 1] = c; out->n[i - 1] = e;
    }
    *count = n;
    return 1;
}

// The model's stage state (BMS) has more than one thread at it: the note functions run in the actuator's ACT_POST hooks (the channel's retarder and engine-brake learners, the
// engine-brake stage) on the game's pedal thread, Decel's bridge on the frame thread, and the key, the save and the reset on the frame thread. A getter also writes (bm_sync_*
// resets the arrays when the stage count changes). One small lock, tried with a bounded wait: a note that cannot get it is dropped (a sample lost, the next comes), a getter
// that cannot get it reads without syncing (the start values and corrections of the last sync, a float at a time: no torn value, only a stale one for a moment), the save and the
// key wait their turn (a few spins: they are the long ones, a note waits for them at most a file read). Before, a reset or a truck change could memset the state under a note.
static volatile char bm_lk;
static bool bm_try(int spins = 400) { for (int i = 0; i < spins; i++) if (!__atomic_test_and_set(&bm_lk, __ATOMIC_ACQUIRE)) return true; return false; }
static void bm_rel() { __atomic_clear(&bm_lk, __ATOMIC_RELEASE); }
struct bm_guard { bool ok; explicit bm_guard(int spins = 400) : ok(bm_try(spins)) {} ~bm_guard() { if (ok) bm_rel(); } };
static void bm_save_locked(const char *why);

static void bm_key()
{
    if (!(g_brake_learn > 0) || !V.eng_ok || T.fwd_gears <= 0) return;
    char k[96];
    snprintf(k, sizeof k, "E%.0f-%.0f-G%d-W%d", V.eng.torque, V.eng.rpm_limit, T.fwd_gears, T.wheels);
    if (BMS.loaded && !strcmp(k, BMS.key)) return;
    bm_guard lk(100000);   // (the long one: it waits for a note, a note never waits for it long)
    if (!lk.ok) return;     // (the next frame tries again)
    if (BMS.loaded && BMS.dirty) { BMS.save_at = -1e9; bm_save_locked("another truck"); }
    memset(&BMS, 0, sizeof BMS);
    bm_stages_reset(&BMS.ret); bm_stages_reset(&BMS.eb); bm_stages_reset(&BMS.s_ret); bm_stages_reset(&BMS.s_eb);
    snprintf(BMS.key, sizeof BMS.key, "%s", k);
    BMS.loaded = 1;
    FILE *f = open_out(BM_FILE, "r");
    if (!f) { logv(1, "Brake model: nothing stored for '%s' yet, starting at the start values", BMS.key); return; }
    char buf[4096];
    int n = (int)fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", BMS.key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            // each part on its own: a bad retarder part leaves the engine brake's alone
            int okr = bm_parse(ln, 'r', &BMS.s_rsteps, &BMS.s_ret, BM_RET_ACCEPT_LO, BM_RET_ACCEPT_HI);
            int oke = bm_parse(ln, 'e', &BMS.s_ecap, &BMS.s_eb, BM_EB_ACCEPT_LO, BM_EB_ACCEPT_HI);
            if (!okr) { BMS.s_rsteps = 0; bm_stages_reset(&BMS.s_ret); }
            if (!oke) { BMS.s_ecap = 0; bm_stages_reset(&BMS.s_eb); }
            logv(1, "Brake model: loaded for '%s': retarder %d stages%s, engine brake %d stages%s", BMS.key, BMS.s_rsteps, okr ? "" : " (a bad token: the start values)", BMS.s_ecap, oke ? "" : " (a bad token: the start values)");
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Brake model: nothing stored for '%s', starting at the start values", BMS.key);
}

// The live stage counts against what is stored: the stored corrections come in once the count is known and the same.
static void bm_sync_ret()
{
    int rs = vehicle_ret_steps();
    if (rs == BMS.rsteps) return;
    bm_guard lk(2000);   // (busy: read without syncing, the next call does)
    if (!lk.ok) return;
    bm_stages_reset(&BMS.ret); BMS.rsteps = rs;
    if (rs > 0 && BMS.s_rsteps == rs) BMS.ret = BMS.s_ret;
}
static void bm_sync_eb(int cap)
{
    if (cap == BMS.ecap) return;
    bm_guard lk(2000);   // (busy: read without syncing, the next call does)
    if (!lk.ok) return;
    bm_stages_reset(&BMS.eb); BMS.ecap = cap;
    if (cap > 0 && BMS.s_ecap == cap) BMS.eb = BMS.s_eb;
}

// A correction read for a getter: under the lock, so a reset's memset (the arrays at 0 for a moment) is never read as a figure; busy for the whole wait = the start value this once.
static float bm_corr_safe(const bm_stages &s, int stage, int n)
{
    bm_guard lk(20000);
    return lk.ok ? bm_stage_corr(s, stage, n) : 1.0f;
}

static int bm_any(const bm_stages &s, int n) { for (int i = 0; i < n && i < BM_STAGES_MAX; i++) if (s.n[i] > 0) return 1; return 0; }

static void bm_save(const char *why)
{
    if (!BMS.loaded || !BMS.dirty || !BMS.key[0]) return;
    bm_guard lk(100000);
    if (lk.ok) bm_save_locked(why);
}
static void bm_save_locked(const char *why)
{
    if (!BMS.loaded || !BMS.dirty || !BMS.key[0]) return;
    char buf[4096], out[4096]; int n = 0, m = 0, lines = 0;
    FILE *f = open_out(BM_FILE, "r");
    if (f) { n = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", BMS.key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + (int)strlen(ln) + 2 < 3300) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    // a part whose live count is not known (this session never saw it) keeps what was stored
    const bm_stages &r = BMS.rsteps > 0 ? BMS.ret : BMS.s_ret, &e = BMS.ecap > 0 ? BMS.eb : BMS.s_eb;
    int rs = BMS.rsteps > 0 ? BMS.rsteps : BMS.s_rsteps, es = BMS.ecap > 0 ? BMS.ecap : BMS.s_ecap;
    m += snprintf(out + m, sizeof out - m, "truck=%s v=1", BMS.key);
    if (rs > 0 && bm_any(r, rs)) {
        m += snprintf(out + m, sizeof out - m, " rs=%d", rs);
        for (int i = 0; i < rs && i < BM_STAGES_MAX; i++) if (r.n[i] > 0) m += snprintf(out + m, sizeof out - m, " r%d=%.3f,%.1f", i + 1, r.c[i], r.n[i]);
    }
    if (es > 0 && bm_any(e, es)) {
        m += snprintf(out + m, sizeof out - m, " es=%d", es);
        for (int i = 0; i < es && i < BM_STAGES_MAX; i++) if (e.n[i] > 0) m += snprintf(out + m, sizeof out - m, " e%d=%.3f,%.1f", i + 1, e.c[i], e.n[i]);
    }
    m += snprintf(out + m, sizeof out - m, "\n");
    BMS.save_at = wall_s();
    f = open_out(BM_FILE, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Brake model: cannot write %s (the corrections stay in memory)", BM_FILE); return; }
    size_t w = fwrite(out, 1, (size_t)m, f);
    fclose(f);
    if (w == (size_t)m) { BMS.dirty = 0; logv(2, "Brake model: saved (%s)", why); }
}

static void bm_step()
{
    bm_key();   // (a new truck: the old one's samples are saved first)
    if (BMS.loaded && BMS.dirty && wall_s() - BMS.save_at > 30) bm_save("periodic");
}

static void bm_reset(int level)
{
    if (level < RESET_HARD) return;
    bm_guard lk(100000);
    if (g_brake_learn > 0) { BMS.save_at = -1e9; bm_save_locked("reset"); }
    memset(&BMS, 0, sizeof BMS);
    bm_stages_reset(&BMS.ret); bm_stages_reset(&BMS.eb); bm_stages_reset(&BMS.s_ret); bm_stages_reset(&BMS.s_eb);
}

// ---- the retarder
int vehicle_ret_steps() { int s = (int)T.retarder_steps; return s > 0 && s <= 16 ? s : 0; }
float vehicle_ret_prior(int stage) { return bm_ret_prior(stage, vehicle_ret_steps()); }
float vehicle_ret_force(int stage)
{
    int steps = vehicle_ret_steps();
    if (steps <= 0 || stage <= 0) return 0;
    if (stage > steps) stage = steps;
    bm_sync_ret();
    return bm_ret_prior(stage, steps) * bm_corr_safe(BMS.ret, stage, steps);
}
float vehicle_ret_force_full()
{
    int steps = vehicle_ret_steps();
    return steps > 0 ? vehicle_ret_force(steps) : bm_ret_table_n(4);
}
float vehicle_ret_force_frac(float frac)
{
    int steps = vehicle_ret_steps();
    if (steps <= 0) return 0;
    int stage = (int)lroundf(fminf(fmaxf(frac, 0.0f), 1.0f) * (float)steps);
    if (stage < 1) stage = 1;
    bm_sync_ret();
    return bm_ret_prior_frac(frac) * bm_corr_safe(BMS.ret, stage, steps);
}
int vehicle_ret_events(int stage)
{
    int steps = vehicle_ret_steps();
    if (steps <= 0 || stage < 1 || stage > steps || stage > BM_STAGES_MAX) return 0;
    bm_sync_ret();
    return (int)BMS.ret.n[stage - 1];
}
void vehicle_ret_note(int stage, float force_n)
{
    int steps = vehicle_ret_steps();
    if (!(g_brake_learn > 0) || steps <= 0 || stage < 1 || stage > steps || stage > BM_STAGES_MAX || !(force_n > 0)) return;
    bm_sync_ret();
    bm_guard lk;
    if (!lk.ok || BMS.rsteps != steps) return;   // (busy, or not synced to this count yet: the sample is dropped, the next comes)
    float prior = bm_ret_prior(stage, steps), r = force_n / prior;
    if (!(r >= BM_RET_ACCEPT_LO && r <= BM_RET_ACCEPT_HI)) return;
    float old = bm_stage_corr(BMS.ret, stage, steps);
    bm_stage_add(&BMS.ret, stage, steps, r);
    BMS.dirty = 1;
    float now = BMS.ret.c[stage - 1];
    if (fabsf(now - old) > 0.1f * old || BMS.ret.n[stage - 1] == 5) logv(1, "Brake model: retarder stage %d of %d: %.0f kN (the start value %.0f kN, %.0f samples)", stage, steps, prior * now / 1000, prior / 1000, BMS.ret.n[stage - 1]);
}

// The retarder against the road speed (core/brakemodel.h has the derivation): the knee is the engine's peak power over the top stage's force. V.peak_pw is the peak of the
// torque factor times rpm, so the power is the peak torque x that x 2 pi / 60.
float vehicle_ret_knee_ms()
{
    if (vehicle_ret_steps() <= 0 || !V.eng_ok || !(V.eng.torque > 0) || !(V.peak_pw > 0)) return 0.0f;
    return bm_ret_knee(V.eng.torque * V.peak_pw * 0.10471976f, vehicle_ret_force_full());
}
float vehicle_ret_shape(float v) { return bm_ret_shape(v, vehicle_ret_knee_ms()); }
float vehicle_ret_shape_stage(float v, float f) { return bm_ret_shape_stage(v, vehicle_ret_knee_ms(), f); }

// ---- the engine brake
static float bm_ratio_total(int gear) { return gear >= 1 && gear <= n_gears() && T.diff > 0 ? T.ratio[gear - 1] * T.diff : 0.0f; }
float vehicle_eb_prior(int stage, int cap, int gear)
{
    return bm_eb_prior(stage, cap, V.eng_ok ? V.eng.torque : 0.0f, bm_ratio_total(gear), V.eta, wheel_radius());
}
float vehicle_eb_force(int stage, int cap, int gear)
{
    if (cap <= 0 || cap > 16 || stage <= 0) return 0;
    if (stage > cap) stage = cap;
    bm_sync_eb(cap);
    return vehicle_eb_prior(stage, cap, gear) * bm_corr_safe(BMS.eb, stage, cap);
}
float vehicle_eb_decel(int stage, int cap, int gear) { return vehicle_eb_force(stage, cap, gear) / bg_mass(); }
// The engine brake gives its full torque only where the engine's own brake range begins (the game's definition of where it works: the stock automatic downshifts into
// it, TrueShift keeps the gear there for a stage the driver holds). Below it the torque falls with the revs (a compression brake's torque follows the revs), to nothing at
// idle: a straight line from idle to the range's low edge. The model's start value is the full-torque figure (BM_EB_SHARE is fitted at 1500 rpm, the range's low edge of the
// engine it was fitted on), so inside the bands a stage gives less (the 1.3 modelled against 0.5 measured of the BR sims at 60 t, 5 %). Unknown range or idle: 1, the figure as it was.
float vehicle_eb_rpm_share(float rpm)
{
    const eng_bands *b = bands_now();
    const float lo = V.eng_ok ? V.eng.range[R_BRAKE][0] : 0.0f, idle = b && b->ok ? b->idle : 0.0f;
    if (!(rpm == rpm) || !(lo > idle + 100.0f)) return 1.0f;
    return fminf(fmaxf((rpm - idle) / (lo - idle), 0.0f), 1.0f);
}
float vehicle_eb_decel_at(int stage, int cap, int gear, float rpm) { return vehicle_eb_decel(stage, cap, gear) * vehicle_eb_rpm_share(rpm); }
int vehicle_eb_events_total()
{
    int n = 0;
    for (int s = 1; s <= BMS.ecap && s <= BM_STAGES_MAX; s++) n += (int)BMS.eb.n[s - 1];
    return n;
}
int vehicle_eb_events(int stage)
{
    if (BMS.ecap <= 0 || stage < 1 || stage > BMS.ecap || stage > BM_STAGES_MAX) return 0;
    return (int)BMS.eb.n[stage - 1];
}
void vehicle_eb_note(int stage, int cap, int gear, float force_n)
{
    if (!(g_brake_learn > 0) || cap <= 0 || cap > 16 || stage < 1 || stage > cap || stage > BM_STAGES_MAX || !(force_n > 0)) return;
    float prior = vehicle_eb_prior(stage, cap, gear);
    if (!(prior > 0)) return;
    bm_sync_eb(cap);
    bm_guard lk;
    if (!lk.ok || BMS.ecap != cap) return;   // (busy, or not synced to this count yet: the sample is dropped, the next comes)
    float r = force_n / prior;
    if (!(r >= BM_EB_ACCEPT_LO && r <= BM_EB_ACCEPT_HI)) return;
    float old = bm_stage_corr(BMS.eb, stage, cap);
    bm_stage_add(&BMS.eb, stage, cap, r);
    BMS.dirty = 1;
    float now = BMS.eb.c[stage - 1];
    if (fabsf(now - old) > 0.1f * old || BMS.eb.n[stage - 1] == 5) logv(1, "Brake model: engine brake stage %d of %d: %.0f%% of the start value in gear %d (%.0f samples)", stage, cap, now * 100, gear, BMS.eb.n[stage - 1]);
}

// ---- the natural deceleration
float vehicle_drag_force(int g, float v)
{
    const float full = vehicle_force_at(g, v, 1.0f), rpm = vehicle_engine_rpm_at(g, v);
    if (!(full > 1) || !(fuel_kd_rpm() > 0)) return 0.0f;
    return fuel_kd() * (rpm / fuel_kd_rpm()) * full / fmaxf(0.05f, curve_factor(&V.eng, rpm) * V.f_scale);
}
float vehicle_natural_decel(int gear, float v)
{
    float m = bg_mass();
    float a = vehicle_resist(v) / m;
    if (gear > 0) a += vehicle_drag_force(gear, v) / m;
    return a;
}

// ---------------------------------------------------------------- fuel & drag learning (fuel_learn.txt)
//
// While it drives, the suite learns what the engine costs in fuel and what it brakes in gear, robustly against the game's map
// scale. The game's fuel use is a x omega x (T_net + T_f): the engine's work torque T_net = peak torque x curve(rpm) x effective
// throttle plus a friction torque T_f ~ k x peak torque (k ~ 0.05-0.12), both at the rpm of now. In gear with the throttle shut
// (< 0.02) the game burns nothing (overrun cut); out of gear / clutch open it burns the idle, a x omega_idle x T_f. The scale
// 'a' differs by map zone (cities ~1:3, outside ~1:19) and can switch abruptly mid-drive, so it must follow fast (a two-time-
// scale scheme: 'a' is re-estimated from every pull window, forgetting ~15 s, while k = T_f / peak torque follows slowly, ~5 min,
// and is pinned by the idle windows, which measure the product a x k directly). kd = the engine's drag torque in gear with the
// throttle shut (the force that slows the truck beyond rolling + air resistance, on the Scania ~6 % of peak), measured as a share
// of peak torque, also slowly. Nothing here changes a gear or eco-roll decision; the next task reads fuel_k()/fuel_kd()/fuel_a().
//
// The fuel level telemetry has 1 mL resolution and the tank sloshes, so the slope is taken over a 2-5 s window with a least-
// squares fit (the jitter averages out; a refuel, a rise of more than a couple of litres, resets the window). Windows are excluded
// when a shift, a slipping clutch, braking, a throttle blip or a refuel would corrupt them. k and kd persist per truck in
// fuel_learn.txt, keyed as brake_gain.txt has it; 'a' is a session figure and starts over.

static const float FL_WIN_S = 3.0f;       // s, a window (2-5 s: the fuel jitter is averaged out, the slope stays fresh)
static const float FL_SETTLE_S = 0.5f;    // s, the state must hold before a window opens (a throttle blip, a shift's tail)
// A window that ends before FL_WIN_S (the lift ends, the throttle moves) still counts once it holds FL_MIN_WIN_S of settled samples. A real ride has no 3.5 s lifts: of 63
// lifts in gear in the owner's ride (02-11) the median lasted 0.7 s and 2 lasted 3.5 s, so the drag (kd) never saw a window. 1 s is the least fl_finish ever accepted for a fuel
// window (the slope needs a spread of times), and the weight of a window falls with its length (fl_finish), so a short one moves the estimate by what it knows.
static const float FL_MIN_WIN_S = 1.0f;
static const float FL_ACCEL_TAU = ACCEL_LAG_S;   // s, the low-pass telemetry_frame puts on the acceleration (T.accel): a lag the drag window corrects for (fl_finish)
// The prior counts as this many seconds of full-information samples (an idle window, where the friction is all of the burn): the prior of k is +-40 % of itself (0.02..0.25
// around 0.09 is as far as the clamp goes), a full window of 3 s knows it to the scale's error, 5-10 %: the ratio of the variances is (0.4 / 0.07)^2 ~ 30, so the prior is
// worth 3 s / 30 = 0.1 s. The first window that carries more than that moves the estimate more than the prior holds it.
static const float FL_PRIOR_W = 0.1f;
static const float FL_A_TAU = 15.0f;      // s, the zone scale 'a' follows this fast (seconds to tens of seconds)
static const float FL_K_TAU = 300.0f;     // s (~5 min), the friction share k follows this slowly
static const float FL_KD_TAU = 300.0f;    // s, the drag share kd follows this slowly
static const float FL_K_PRIOR = 0.09f, FL_K_LO = 0.02f, FL_K_HI = 0.25f;
static const float FL_KD_PRIOR = 0.06f, FL_KD_LO = 0.01f, FL_KD_HI = 0.2f;
static const float FL_KDR_PRIOR = 1100.0f;   // rpm the drag prior stands for (an engine drags more the faster it turns: kd is taken at the windows' mean rpm)
static const float FL_THR = 0.05f;        // a pull window needs this much throttle
static const float FL_REFUEL_L = 2.0f;    // a fuel rise over this between frames = a refuel (reset the window)
static const float FL_MIN_V = 8.0f, FL_MAX_V = 33.0f;   // m/s, the drag windows' speed band (the drag is drowned in the air drag higher up)
static const float FL_KD_GRADE = 3.0f;    // deg, no drag window on a steeper grade
static const char *const FL_FILE = "fuel_learn.txt";

enum { FL_NONE, FL_PULL, FL_IDLE, FL_OVERRUN };

static struct {
    char key[96]; int loaded, primed, dirty;
    double last, save_at, log_at;
    float k, kd, a;                       // k = T_f / peak, kd = drag / peak, a = the zone scale
    float kdr;                            // the rpm kd was measured at (the drag windows' mean, as slowly)
    int n_k, n_kd, n_a, zones;
    double kw, kdw, aw;                   // the wall-clock forgetting weights (s of samples)
    double last_k, last_kd, last_a;       // sim_s() of the last update of each
    // the current window
    int kind;                             // FL_*
    double settle, t0, wt;
    double ft, ff, ftt, ftf; int fn;      // the fuel/time least-squares fit (t relative to t0)
    double u, v;                          // omega x T_net dt and omega x peak dt (the pull / idle model terms)
    double acc, Rd, rpm;                  // the drag window: mean lagged accel, lagged resistance, rpm
    float a0, a_last;                     // the drag window: T.accel when it opened and at its last sample (the lag correction)
    float thr0, cl0; int gear0;           // the window's reference state (stability)
    float prev_fuel;                      // for the refuel detection
    double fl_Rd;                         // the resistance lagged 0.5 s, in step with T.accel (see learn_force)
} F;

static void fl_save(const char *why);
static void fl_finish();
static void fl_break_window()
{
    F.settle = 0; F.wt = 0; F.fn = 0;
    F.ft = F.ff = F.ftt = F.ftf = F.u = F.v = F.acc = F.Rd = F.rpm = 0;
}
// The state under the window ended (the lift is over, the throttle moved, a shift): what it measured up to the last sample is clean, so it counts if it is long enough.
// (A refuel, a pause or a skipped frame corrupt the window: fl_break_window throws it away.)
static void fl_close()
{
    if (F.kind != FL_NONE && F.settle >= FL_SETTLE_S && F.wt >= FL_MIN_WIN_S) fl_finish();
    fl_break_window();
}

static void fl_key()
{
    if (F.primed || !V.eng_ok || T.fwd_gears <= 0) return;
    char k[96];
    snprintf(k, sizeof k, "E%.0f-%.0f-G%d-W%d", V.eng.torque, V.eng.rpm_limit, T.fwd_gears, T.wheels);
    if (F.loaded && !strcmp(k, F.key)) return;
    if (F.loaded && F.dirty) { F.save_at = -1e9; fl_save("another engine"); }   // (the old key's samples are kept before it changes)
    snprintf(F.key, sizeof F.key, "%s", k);
    F.loaded = 1;
    F.k = FL_K_PRIOR; F.kd = FL_KD_PRIOR; F.kdr = FL_KDR_PRIOR; F.a = 1.0f;
    F.n_k = F.n_kd = F.n_a = F.zones = 0; F.dirty = 0;
    F.kw = F.kdw = F.aw = 0; F.last_k = F.last_kd = F.last_a = 0;
    fl_break_window();
    FILE *f = open_out(FL_FILE, "r");
    if (!f) { logv(1, "Fuel learn: nothing stored for '%s' yet", F.key); return; }
    char buf[4096];
    int n = (int)fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", F.key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            float kv = 0, kdv = 0; int nk = 0, nkd = 0;
            const char *q = strstr(ln, " k="), *d = strstr(ln, " kd="), *m = strstr(ln, " nk="), *o = strstr(ln, " nkd=");
            if (q && d && m && o && sscanf(q + 3, "%f", &kv) == 1 && sscanf(d + 4, "%f", &kdv) == 1 &&
                sscanf(m + 4, "%d", &nk) == 1 && sscanf(o + 5, "%d", &nkd) == 1 &&
                nk >= 0 && nkd >= 0 && (nk == 0 || (kv >= FL_K_LO && kv <= FL_K_HI)) && (nkd == 0 || (kdv >= FL_KD_LO && kdv <= FL_KD_HI))) {
                // each coefficient on its own: one with no samples yet (n 0) keeps its prior, the other keeps what it learned (owner's ride 02-11:
                // the Scania's line had nk=4 nkd=0, was read as a bad line every session and the learned k was thrown away each time)
                if (nk > 0) { F.k = kv; F.n_k = nk; }
                if (nkd > 0) { F.kd = kdv; F.n_kd = nkd; }
                const char *r = strstr(ln, " kdr="); float kr = 0;   // (older lines have none: the prior's rpm)
                F.kdr = nkd > 0 && r && sscanf(r + 5, "%f", &kr) == 1 && kr > 300 && kr < 4000 ? kr : FL_KDR_PRIOR;
                // (what was learned weighs as the seconds of samples it came from, up to the slow time constant: the first window of
                // a new session blends with it instead of replacing it)
                F.kw = fmin((double)nk * FL_WIN_S, (double)FL_K_TAU); F.kdw = fmin((double)nkd * FL_WIN_S, (double)FL_KD_TAU);
                F.last_k = F.last_kd = sim_s();
                logv(1, "Fuel learn: loaded for '%s': k %.3f (%d), kd %.3f (%d)", F.key, F.k, F.n_k, F.kd, F.n_kd);
            } else logv(1, "Fuel learn: a bad line for '%s', starting at the priors", F.key);
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Fuel learn: nothing stored for '%s', starting at the priors", F.key);
}

static void fl_save(const char *why)
{
    if (!F.loaded || !F.dirty || !F.key[0]) return;
    char buf[4096], out[4096]; int n = 0, m = 0, lines = 0;
    FILE *f = open_out(FL_FILE, "r");
    if (f) { n = (int)fread(buf, 1, sizeof buf - 1, f); fclose(f); }
    buf[n > 0 ? n : 0] = 0;
    char want[110]; snprintf(want, sizeof want, "truck=%s ", F.key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, "truck=", 6) && strncmp(ln, want, strlen(want)) && m + (int)strlen(ln) + 2 < 3300) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "truck=%s v=1 k=%.4f kd=%.4f nk=%d nkd=%d kdr=%.0f\n", F.key, F.k, F.kd, F.n_k, F.n_kd, F.kdr);
    F.save_at = wall_s();
    f = open_out(FL_FILE, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Fuel learn: cannot write %s (the estimates stay in memory)", FL_FILE); return; }
    size_t w = fwrite(out, 1, (size_t)m, f);
    fclose(f);
    if (w == (size_t)m) { F.dirty = 0; logv(2, "Fuel learn: saved (%s)", why); }
}

// The current window's kind (and the throttle / clutch of now, for the stability check). FL_NONE = dirty (or no window fits).
static int fl_state(float *thr, float *cl)
{
    *thr = T.eff_thr;   // (the engine's throttle: the pedal can be far from it under the wish map, a cap or a cut)
    *cl = T.eff_clutch;   // (the fuel learner reads the channel directly; it has no business with Creep's clutch override)
    int br = fmaxf(fmaxf(T.in_brake, T.eff_brake), act_request()) > 0.02f;
    if (!(V.mass_ok && V.eng_ok && V.self && !T.paused && T.engine_on && T.gear >= 0 &&
          T.gear == T.dgear && T.fuel == T.fuel && T.fuel > 0 && !T.retarder && !(g_brake_fed > 0) && !T.motor_brake && !(game_ebrake_level(V.self) > 0.01f))) return FL_NONE;   // (the automatic engine brake the SDK flag misses)
    // The idle burn does not care about the brakes or the parking brake: the engine turns at its idle with the throttle shut whatever holds the truck, and a truck that
    // waits at a light is braked or parked every time. Those stops are the idle windows (with them excluded a ride had none longer than 3.5 s in a quarter of an hour).
    // The drag window measures a deceleration, which a brake or a parking brake is part of; the pull window is the engine's alone, but a braking truck is not in the steady
    // state it is after.
    if ((T.gear < 1 || *cl > 0.5f) && *thr < 0.02f) return FL_IDLE;
    if (T.parking_brake || br) return FL_NONE;
    if (T.gear >= 1 && *cl < 0.1f && *thr >= FL_THR) return FL_PULL;
    if (T.gear >= 1 && *cl < 0.1f && *thr < 0.02f && T.in_thr < 0.02f && T.speed >= FL_MIN_V && T.speed <= FL_MAX_V &&
        fabsf(V.grade_deg) < FL_KD_GRADE && fabsf(T.yaw) < 0.04f) return FL_OVERRUN;   // (straight: a bend's tire scrub is no engine drag, as the brake learner)
    return FL_NONE;
}

static void fl_open(double now)
{
    F.t0 = now; F.wt = 0; F.fn = 0;
    F.ft = F.ff = F.ftt = F.ftf = F.u = F.v = F.acc = F.Rd = F.rpm = 0;
    F.a0 = F.a_last = T.accel;
}

// One estimate x toward *val with a wall-clock forgetting of tau seconds (the weight of a sample is its window's seconds;
// between windows the accumulated weight decays with e^-dt/tau, so the value converges over ~tau seconds of driving, however
// bursty the windows). The first sample sets it (the weight starts at 0) unless there is a prior: then the weight starts at the prior's (FL_PRIOR_W).
static void fl_blend(float *val, double *wsum, double *last, float tau, float win, float x, float prior = 0.0f)
{
    double now = sim_s();
    if (!(*wsum > 0)) *wsum = prior;
    if (*last > 0 && now > *last) *wsum *= exp(-(now - *last) / tau);
    *wsum += win;
    *val += (x - *val) * (win / (float)*wsum);
    *last = now;
}

static void fl_finish()
{
    if (F.kind == FL_OVERRUN) {
        // T.accel is telemetry_frame's low-pass (time constant FL_ACCEL_TAU) of the truck's acceleration, so after the lift it is still on its way down to the drag: the
        // true acceleration is a_meas + tau x da_meas/dt, and its mean over the window is the mean measured + tau x (a_end - a_start) / wt (exact for a first-order lag).
        // Uncorrected, the first second of a lift reads 16 % of the step from the pull to the drag too high (a 1 s window after the settle) and 6 % over the 3 s ones.
        float accel = (float)(F.acc / F.wt) + FL_ACCEL_TAU * (F.a_last - F.a0) / (float)F.wt, Rd = (float)(F.Rd / F.wt), rpm = (float)(F.rpm / F.wt);
        float Fd = -(V.mass * accel + Rd);                       // the force that slows the truck beyond the model's resistance
        if (Fd > 0 && rpm > 0 && V.eng.torque > 0) {
            float r = wheel_radius(), ratio = T.gear >= 1 && T.gear <= 32 ? T.ratio[T.gear - 1] : 0;
            if (r > 0.1f && ratio > 0 && T.diff > 0 && V.eta > 0) {
                float kd_win = Fd * r / (ratio * T.diff * V.eta) / V.eng.torque;
                if (kd_win >= FL_KD_LO && kd_win <= FL_KD_HI) {
                    // the weight is the window's seconds: the noise of a mean acceleration falls with the seconds it averages
                    fl_blend(&F.kd, &F.kdw, &F.last_kd, FL_KD_TAU, (float)F.wt, kd_win, FL_PRIOR_W);
                    F.kdr += (rpm - F.kdr) * (float)(F.wt / F.kdw);   // (the rpm follows with the same weights)
                    F.n_kd++; F.dirty = 1;
                }
            }
        }
        return;
    }
    double n = F.fn, ft = F.ft, ff = F.ff, ftt = F.ftt, ftf = F.ftf, denom = n * ftt - ft * ft;
    if (denom < 1e-9 || F.wt < FL_MIN_WIN_S) return;
    double slope = (n * ftf - ft * ff) / denom, y = -slope;      // the fuel rate, L/s (positive = burning)
    double u = F.u / F.wt, v = F.v / F.wt;
    if (!(y > 0) || !(v > 0) || V.eng.torque <= 0) return;
    // The weight of a fuel window: the slope of a ramp the telemetry rounds to 1 mL has a variance that falls with the cube of the time it is fitted over (n samples at a
    // fixed rate: sigma^2 / (n W^2) with n ~ W), so a window of wt seconds weighs wt x (wt / FL_WIN_S)^2 seconds: a full window as before, a 1 s one a ninth of it.
    const double wf = F.wt * (F.wt / FL_WIN_S) * (F.wt / FL_WIN_S);
    if (F.kind == FL_PULL) {
        double a_win = y / (u + (double)F.k * v);                // the zone scale this window implies, at the current k
        if (a_win > 0) {
            if (F.n_a >= 2 && fabsf(a_win / (double)F.a - 1.0) > 0.4) F.zones++;   // an abrupt change of the scale: a zone jump (against the value before this window)
            fl_blend(&F.a, &F.aw, &F.last_a, FL_A_TAU, (float)wf, (float)a_win);
            F.n_a++;
        }
        // k from a pull window too, with the weight of what the window knows about it. The friction share of the burn is share = a k v / y: the window's k is
        // k_win = k - eps (x + k) for a relative error eps of the scale 'a' (x the work torque share), so its error relative to k is eps / share and its weight
        // (1 / variance) goes with share^2. An idle window has share 1 (the friction is all of the burn). The hard gate this replaces (share >= 0.4) let nothing
        // through at the throttle a loaded truck holds in a cruise (share 10-25 %): the Scania's line in the owner's ride read nk=4 after a whole ride.
        if (F.n_a >= 1 && F.a > 0) {
            const double share = fmin(1.0, (double)F.a * F.k * v / y), wk = wf * share * share;
            double k_win = (y - (double)F.a * u) / ((double)F.a * v);
            if (k_win >= FL_K_LO && k_win <= FL_K_HI) {
                fl_blend(&F.k, &F.kw, &F.last_k, FL_K_TAU, (float)wk, (float)k_win, FL_PRIOR_W);
                if (wk >= FL_PRIOR_W) F.n_k++;   // (counted once it holds the estimate more than the prior does)
                F.dirty = 1;
            }
        }
    } else {   // idle / neutral: the product a x k directly
        if (F.n_a >= 1 && F.a > 0) {
            double k_win = y / ((double)F.a * v);
            if (k_win >= FL_K_LO && k_win <= FL_K_HI) {
                fl_blend(&F.k, &F.kw, &F.last_k, FL_K_TAU, (float)wf, (float)k_win, FL_PRIOR_W);
                F.n_k++; F.dirty = 1;
            }
        }
    }
}

static void fuel_learn_step()
{
    double now = sim_s();
    float dt = (float)(now - F.last);
    F.last = now;
    if (!(dt > 0 && dt < 0.25)) { fl_break_window(); return; }
    // the resistance lagged 0.5 s, kept in step with T.accel (which the telemetry lags by the same): see TrueShift's learn_force
    if (V.mass_ok && V.eng_ok && T.gear >= 1) {
        float Rd = vehicle_resist(T.speed);
        float kl = dt / (FL_ACCEL_TAU + dt);
        F.fl_Rd += kl * (Rd - F.fl_Rd);
    }
    fl_key();
    if (F.loaded && F.dirty && wall_s() - F.save_at > 30) fl_save("periodic");
    // a refuel (the level jumped up) resets the window; the learned k / kd are truck figures and survive it
    if (F.prev_fuel == F.prev_fuel && T.fuel > F.prev_fuel + FL_REFUEL_L) fl_break_window();
    F.prev_fuel = T.fuel;

    float thr, cl;
    int kind = fl_state(&thr, &cl);
    if (kind != F.kind) { fl_close(); F.kind = kind; }
    if (kind == FL_NONE) return;
    // the state must hold still before a window opens, and through it
    if (kind == FL_PULL && (F.settle > 0 ? fabsf(thr - F.thr0) > 0.05f || fabsf(cl - F.cl0) > 0.05f || T.gear != F.gear0 : 0)) {
        fl_close(); return;
    }
    if (F.settle < FL_SETTLE_S) {
        if (F.settle == 0 || kind == FL_PULL) { F.thr0 = thr; F.cl0 = cl; F.gear0 = T.gear; }
        F.settle += dt;
        if (F.settle >= FL_SETTLE_S) fl_open(now);
        return;
    }
    if (F.wt == 0) fl_open(now);   // (in case a settle edge fell through)
    double rt = now - F.t0;
    float fuel = T.fuel;
    F.ft += rt; F.ff += fuel; F.ftt += rt * rt; F.ftf += rt * fuel; F.fn++;
    float omega = T.rpm * 0.10472f;
    if (kind == FL_PULL) {
        float tq = V.eng.torque * curve_factor(&V.eng, T.rpm) * thr;
        F.u += omega * tq * dt;
        F.v += omega * V.eng.torque * dt;
    } else if (kind == FL_IDLE) {
        F.v += omega * V.eng.torque * dt;
    } else {
        F.acc += T.accel * dt;
        F.a_last = T.accel;
        F.Rd += F.fl_Rd * dt;
        F.rpm += T.rpm * dt;
    }
    F.wt += dt;
    if (F.wt >= FL_WIN_S) { fl_finish(); fl_break_window(); }

    if (now - F.log_at >= 180.0) {
        F.log_at = now;
        logv(1, "fuel learn: k %.3f (%d), kd %.3f (%d), a %.3g (%d), zone changes %d", F.k, F.n_k, F.kd, F.n_kd, F.a, F.n_a, F.zones);
    }
}

static void fuel_learn_reset(int level)
{
    fl_break_window();
    F.last = 0; F.prev_fuel = NAN; F.fl_Rd = 0; F.log_at = sim_s();
    if (level >= RESET_HARD) {
        if (F.loaded) { F.save_at = -1e9; fl_save("reset"); }
        F.loaded = F.primed = 0; F.key[0] = 0;
        F.k = FL_K_PRIOR; F.kd = FL_KD_PRIOR; F.kdr = FL_KDR_PRIOR; F.a = 1.0f;
        F.n_k = F.n_kd = F.n_a = F.zones = 0; F.dirty = 0;
        F.kw = F.kdw = F.aw = 0; F.last_k = F.last_kd = F.last_a = 0;
    }
}

float fuel_k() { return F.k; }
float fuel_kd() { return F.kd; }
float fuel_kd_rpm() { return F.kdr; }
void vehicle_fuel_prime(float k, float kd, float kdr) { F.k = k; F.kd = kd; F.kdr = kdr; F.n_k = F.n_kd = 50; F.n_a = 3; }   // sims: as if learned
float fuel_a() { return F.a; }
void vehicle_fuel_counts(int *n_k, int *n_kd, int *n_a) { *n_k = F.n_k; *n_kd = F.n_kd; *n_a = F.n_a; }
int fuel_learned() { return F.n_k >= 5 && F.n_kd >= 5 && F.n_a >= 3; }   // (and the zone scale of this session seen)

void vehicle_stop()
{
    if (BMS.loaded && BMS.dirty) { BMS.save_at = -1e9; bm_save("stop"); }
    if (F.loaded && F.dirty) fl_save("stop");
    if (RC.loaded && RC.dirty) { RC.save_at = -1e9; rc_save("stop"); }
}
