// Offline test of TrueShift's decisions (modules/trueshift.cpp) on a simple
// longitudinal truck model. Emulates the game's automatic: per tick the decide
// stub, the stock window trigger, the chooser (a rough stock chooser below the
// takeover speed), a shift with a torque gap.
// Build+run: tools/sim/run.sh [-v]   (-v prints every log line)
#include "core/suite.h"
#include "fuel_fit.h"  // SW_FUEL=fit: the fitted fuel model (port of the root's fuel_model.hpp) for the plant
#include "sim_env.h"  // SIM_SET=section.key=value,... overrides ini defaults (sweeps)
#include "core/game.h"
#include "sim_layout.h"
#include "core/route.h"
#include "core/vehicle.h"
#include "core/vclass.h"
#include "core/gearscore.h"
#include "core/rpmspan.h"
#include "core/bands.h"
#include "core/powertrain.h"
#include "core/gearchoice.h"
#include "core/demand.h"
#include "core/driver.h"
#include "core/wish.h"
#include "core/gearreq.h"
extern module m_pedal;   // the real torque pedal (modules/pedal.cpp), included at the end of this file
#include <math.h>
#include <vector>
#include <string>
#include <unistd.h>
#include <initializer_list>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

telemetry T;
volatile int g_rolling, g_cruise_smooth;
volatile float g_roll_boost_kmh;
volatile double g_rb_cancel_at = -1e9;
volatile int g_pcc_on;
volatile int g_pedal_mapped;
volatile float g_overspeed_cap = 5;
volatile float g_overspeed_pct = 0;
volatile float g_pcc_need_kmh = NAN;
int g_log_level = 2;
incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 0;
int g_n_incidents;
static float g_eb_power = 150000;   // the plant's engine brake: power at full strength and full rpm (W; sim_pcc PCC_EBKW)
static int g_sdk_motor_brake = -1;    // >= 0: the SDK's motor_brake flag (the manual engine brake: on or held), instead of the plant's M.ebrake (sim_pcc PCC_EB)
static int g_quiet;   // the SCORES table runs print no shift lines and no incidents
void logring(const char *, ...) {}
void incident(const char *kind, const char *fmt, ...)
{
    g_n_incidents++;
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (!g_quiet) printf("   INCIDENT %s: %s\n", kind, b);
}
const char *g_truck_unsupported;
#ifdef SIM_PCC
volatile float sim_g_clutch_ours = __builtin_nanf("");   // (sim_pcc links the real actuator: these stubs are parked)
float sim_g_air_from_truck;
#else
volatile float g_clutch_ours = __builtin_nanf("");
float g_air_from_truck;   // [core] air_from_truck (core/actuate.cpp, not linked here)
#endif
static int g_cues;   // the cues played (sim_promises.cpp, promise P5: the tones knob)
void play_cue(int) { g_cues++; }
void input_poll() {}
int input_pressed(int, int) { return 0; }
static int sim_btn;   // a wheel button held down in the sim (0 = none)
int input_held(int b) { return sim_btn && b == sim_btn; }
char g_dir[MAX_PATH];
const layout *L = &sim_layout;
game_addrs G;
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *name, const char *mode) { return NULL; }  // no trace file
static int verbose, n_failed, g_x_failed;
static float g_stuck_s = -1;   // the last 'stuck in neutral for N s' line (tail_pre's gap-state timer)
static int engage_downs; static double g_eng_t;   // WP-G6c round 2: downshifts within 5 s (the hunt metric's window) of an engage out of neutral: the gear the engage took was left again at once (B4)
static double g_dn1_t; static char g_dn1_why[48];   // set g_dn1_t to -1: the first downshift of ours after it, when and its reason, lands here
// g_rec_on: every shift of ours is recorded (when, from, to, why, rpm); the SCORES pull-away and stop rows read it
enum { REC_MAX = 128 };
static int g_rec_on, g_rec_n; static struct { double t; int a, b; char why[32]; float rpm, kmh, pred; int wall; float dm, dev; } g_rec[REC_MAX];   // wall: the layers took it because the gear was at the rev wall; dm: the driver's demand when it was recorded, dev: how far the demand strayed from the previous record's since (the ONE hunt definition, hunt_kind)
static float sim_demand();                          // (defined with the run statistics below)
static float g_rec_dm_ref, g_rec_dev;               // the recorder's own foot track: the demand at the last record and the furthest it strayed since
static int g_pred_a, g_pred_b, g_pred_wall; static float g_pred_rpm;   // the last "layers: a->b" line: the landing rpm the layers expected
static float g_cf_rpm = -1, g_cf_share, g_lift_med = -1; static int g_cf_varied;   // SCORES part 4: the last 'choice floor' and 'lifts: median' lines
static int g_votes; static char g_vote[600];   // the last HINTVOTE line TrueShift logged (core/driver.h (e)) and how many
static void (*g_sim_log_tap)(const char *line);   // sim_promises.cpp: every log line, formatted (null = off; no other effect)
void logline(const char *fmt, ...)
{
    if (g_sim_log_tap) { char tb[900]; va_list tp; va_start(tp, fmt); vsnprintf(tb, sizeof tb, fmt, tp); va_end(tp); g_sim_log_tap(tb); }
    if (!strcmp(fmt, "%s")) {
        va_list ap; va_start(ap, fmt);
        const char *l = va_arg(ap, const char *);
        va_end(ap);
        if (l && !strncmp(l, DRV_HINT_PREFIX, strlen(DRV_HINT_PREFIX))) { snprintf(g_vote, sizeof g_vote, "%s", l); g_votes++; }
    }
    if (!strncmp(fmt, "choice floor:", 13)) {
        va_list ap; va_start(ap, fmt); g_cf_varied = !strcmp(va_arg(ap, const char *), "varies"); va_arg(ap, double);
        g_cf_rpm = (float)va_arg(ap, double); g_cf_share = (float)va_arg(ap, double); va_end(ap);
    }
    if (!strncmp(fmt, "lifts: median", 13)) { va_list ap; va_start(ap, fmt); g_lift_med = (float)va_arg(ap, double); va_end(ap); }
    if (strstr(fmt, "no pull after")) n_failed++;
    static const bool sh_tr = getenv("SW_TRACE") != NULL;   // the sweep's trace: every shift with its reason
    static const char *lg_tr = getenv("SW_LOGTR");           // SW_LOGTR="t0,t1": every log line between t0 and t1 s
    if (lg_tr) { float a = 0, b = 0; sscanf(lg_tr, "%f,%f", &a, &b); if (t_now >= a && t_now <= b) { va_list ap; va_start(ap, fmt); printf("LG %7.2f ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap); } }
    if (sh_tr && !strncmp(fmt, "shift %d->%d (%s)", 17)) { va_list ap; va_start(ap, fmt); int a = va_arg(ap, int), b = va_arg(ap, int); const char *w = va_arg(ap, const char *); va_end(ap); printf("SH %7.1f %d->%d (%s)\n", t_now, a, b, w); }
    if (!strncmp(fmt, "layers: %d->%d", 14)) {
        va_list ap; va_start(ap, fmt); g_pred_a = va_arg(ap, int); g_pred_b = va_arg(ap, int); va_arg(ap, const char *); const char *act = va_arg(ap, const char *); g_pred_wall = !strncmp(act, "wall", 4); g_pred_rpm = (float)va_arg(ap, double); va_end(ap);
    }
    if (g_rec_on && g_rec_n < REC_MAX && !strncmp(fmt, "shift %d->%d (%s) at", 20)) {
        va_list ap; va_start(ap, fmt);
        g_rec[g_rec_n].t = t_now; g_rec[g_rec_n].a = va_arg(ap, int); g_rec[g_rec_n].b = va_arg(ap, int);
        snprintf(g_rec[g_rec_n].why, sizeof g_rec[g_rec_n].why, "%s", va_arg(ap, const char *));
        g_rec[g_rec_n].kmh = (float)va_arg(ap, double); g_rec[g_rec_n].rpm = (float)va_arg(ap, double); va_end(ap);
        g_rec[g_rec_n].pred = g_pred_a == g_rec[g_rec_n].a && g_pred_b == g_rec[g_rec_n].b ? g_pred_rpm : 0;
        g_rec[g_rec_n].wall = g_pred_a == g_rec[g_rec_n].a && g_pred_b == g_rec[g_rec_n].b && g_pred_wall;
        g_rec[g_rec_n].dm = sim_demand(); g_rec[g_rec_n].dev = g_rec_n ? g_rec_dev : 0.0f; g_rec_dm_ref = g_rec[g_rec_n].dm; g_rec_dev = 0;
        g_rec_n++;
    }
    if (!strncmp(fmt, "shift %d->%d (%s)", 17)) {
        va_list ap; va_start(ap, fmt); int a = va_arg(ap, int), b = va_arg(ap, int); const char *w = va_arg(ap, const char *); va_end(ap);
        if (!strcmp(w, "engage")) g_eng_t = t_now;
        else if (b < a && g_eng_t > 0 && t_now - g_eng_t < 5.0) engage_downs++;
    }
    if (g_dn1_t < 0 && !strncmp(fmt, "shift %d->%d (%s)", 17)) {
        va_list ap; va_start(ap, fmt); int a = va_arg(ap, int), b = va_arg(ap, int); const char *w = va_arg(ap, const char *); va_end(ap);
        if (b < a) { g_dn1_t = t_now; snprintf(g_dn1_why, sizeof g_dn1_why, "%s", w); }
    }
    if (!strncmp(fmt, "stuck in neutral for", 20)) { va_list ap; va_start(ap, fmt); g_stuck_s = (float)va_arg(ap, double); va_end(ap); }
    int shift = !strncmp(fmt, "shift", 5) || strstr(fmt, "no pull after") || !strncmp(fmt, "load changed", 12);
    if ((!verbose && !shift) || g_quiet) return;
    va_list ap; va_start(ap, fmt);
    printf("%7.1f  ", t_now); vprintf(fmt, ap); printf("\n");
    va_end(ap);
}
float wheel_radius();
static float g_radius_err = 1;   // the plugin's wheel radius over the plant's (the calibration of a converter box finds the difference)
extern module m_trueshift, m_carshift;
// The gear brain of the vehicle class now (core/vclass.h): TrueShift for a truck, CarShift for a car or pickup
static module &brain() { return vclass_active() == VC_LIGHT ? m_carshift : m_trueshift; }

// Torque-converter helpers (the model is defined with the sim's engine below; the use_* fleet functions call these first)
static void tc_reset();
static void tc_set(float stall, int lock_gear, float stall_rpm);

// ---- truck model (Volvo FH 2400 Nm from probe runs)
static float RATIO[16] = { 11.73f, 9.21f, 7.09f, 5.57f, 4.35f, 3.41f, 2.7f, 2.12f, 1.63f, 1.28f, 1.0f, 0.78f };
static int NCURVE = 8;
static float CURVE[24][2] = { {0,0},{600,0.62f},{900,1},{1230,1},{1600,0.77f},{1700,0.69f},{1800,0.44f},{2000,0} };
static float CDA = 6, TORQUE = 2400, LIMIT = 1900, DIFF = 2.85f, RADIUS = 0.506f, RANGES[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1650} };
static int NG = 12;
static void use_renault()  // Renault T, Optidriver Xtended AT02614F (12+2), DE13 TC 480: 2700 Nm, diff 3.40 (24-10 ride)
{
    tc_reset();
    float r[16] = { 32.04f, 17.2f };
    for (int k = 0; k < 12; k++) r[2 + k] = 11.73f * powf(0.79f / 11.73f, k / 11.0f);
    float c[8][2] = { {0,0},{700,0.60f},{900,0.97f},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2700; CDA = 6; LIMIT = 1900; DIFF = 3.40f; RADIUS = 0.538f; NG = 14;
    T.fwd_gears = 14; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_ato()  // Renault T, Optidriver ATO2612F R (12 speeds, no crawl gears), the same DE13 TC 480, diff 3.08 (25-10)
{
    tc_reset();
    float r[16];
    for (int k = 0; k < 12; k++) r[k] = 11.73f * powf(0.79f / 11.73f, k / 11.0f);
    float c[8][2] = { {0,0},{700,0.60f},{900,0.97f},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2700; CDA = 6; LIMIT = 1900; DIFF = 3.08f; RADIUS = 0.538f; NG = 12;
    T.fwd_gears = 12; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static float RATIO0[16], CURVE0[24][2], RANGES0[4][2], CDA0, TORQUE0, LIMIT0, DIFF0, RADIUS0; static int NG0;   // the Volvo as the file starts
static void save_volvo()
{
    memcpy(RATIO0, RATIO, sizeof RATIO); memcpy(CURVE0, CURVE, sizeof CURVE); memcpy(RANGES0, RANGES, sizeof RANGES);
    CDA0 = CDA; TORQUE0 = TORQUE; LIMIT0 = LIMIT; DIFF0 = DIFF; RADIUS0 = RADIUS; NG0 = NG;
}
static void use_pete()  // an ATS Peterbilt (01-11 ride): 2900 Nm, limit 2000, plateau from 1150, 12 speeds (steps of 1.31), diff 2.58, 15.5 t
{
    tc_reset();
    float r[16];
    for (int k = 0; k < 12; k++) r[k] = 14.5f / powf(1.31f, (float)k);
    float c[8][2] = { {0,0},{550,0.5f},{800,0.8f},{1150,1},{1500,1},{1800,0.77f},{2000,0},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    NCURVE = 7;
    float g[4][2] = { {1500,2000},{800,1500},{1100,1500},{1500,2000} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2900; CDA = 6; LIMIT = 2000; DIFF = 2.58f; RADIUS = 0.5f; NG = 12;
    T.fwd_gears = 12; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_volvo()
{
    tc_reset();
    memcpy(RATIO, RATIO0, sizeof RATIO); memcpy(CURVE, CURVE0, sizeof CURVE); memcpy(RANGES, RANGES0, sizeof RANGES);
    CDA = CDA0; TORQUE = TORQUE0; LIMIT = LIMIT0; DIFF = DIFF0; RADIUS = RADIUS0; NG = NG0;
    T.fwd_gears = NG; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_car()  // a pickup-like car: 6 speeds, 450 Nm
{
    tc_reset();
    float r[16] = { 4.17f, 2.34f, 1.52f, 1.14f, 0.87f, 0.69f };
    float c[8][2] = { {0,0},{700,0.55f},{1500,0.9f},{2000,1},{4000,1},{5000,0.85f},{5800,0.6f},{6200,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {3000,5000},{1500,3000},{1800,2600},{3000,5000} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 450; CDA = 1.0f; LIMIT = 6000; DIFF = 3.55f; RADIUS = 0.39f; NG = 6;
    T.fwd_gears = 6; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_dc16()  // Scania DC16 770 (3700 Nm), GRS905R 14-speed (crawler, 1-12, overdrive), ~20 t solo (30-10 ride)
{
    tc_reset();
    float r[16] = { 13.2f, 11.32f, 9.13f, 7.37f, 6.11f, 4.93f, 3.98f, 3.21f, 2.59f, 2.09f, 1.69f, 1.36f, 1.10f, 0.78f };
    float c[8][2] = { {0,0},{600,0.62f},{900,1},{1450,1},{1600,0.9f},{1700,0.78f},{1900,0.6f},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1650} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 3700; CDA = 6; LIMIT = 2000; DIFF = 3.0f; RADIUS = 0.52f; NG = 14;
    T.fwd_gears = 14; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_mercedes16()  // Mercedes Actros 1855 LS, G280-16 16-speed AMT (results/fleet.json), the light-truck case of WP-G5c
{
    tc_reset();
    float r[16] = { 11.722f, 9.747f, 7.916f, 6.583f, 5.291f, 4.4f, 3.636f, 3.023f, 2.664f, 2.215f, 1.799f, 1.496f, 1.203f, 1.0f, 0.826f, 0.687f };
    float c[8][2] = { {0,0},{440,0.5f},{1000,1},{1300,1},{1800,0.85f},{2000,0.65f},{2200,0},{0,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    NCURVE = 7;
    float g[4][2] = { {1500,2000},{700,1450},{1000,1300},{1400,1800} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2600; CDA = 6; LIMIT = 2000; DIFF = 4.833f; RADIUS = 0.538f; NG = 16;
    T.fwd_gears = 16; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}

// ---------------------------------------------------------------- torque-converter fleet (notes/tc-report.md)
// The Ford cars (ATS Road Trip Ford, def/vehicle/car): a gasoline V8 behind a 3-, 4- and 10-speed automatic. The 3-speed is a
// slushbox (never locks), the 4-speed locks in 4th, the 10-speed locks from 2nd (a modern converter). The engines and ratios are
// estimates (the defs were not on disk); the stall ratios follow the game's car boxes, the trucks' are the game's own (2.42).
static void car_gas(float torque, float diff, float radius, int n, const float *ratios, float stall, int lock_gear, float stall_rpm)
{
    tc_reset();
    for (int i = 0; i < 16; i++) RATIO[i] = i < n ? ratios[i] : 0;
    float c[8][2] = { {0,0},{800,0.5f},{1800,0.85f},{3000,1},{4500,1},{5200,0.9f},{5800,0.65f},{6200,0} };
    memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {3500,5500},{1200,3000},{1500,3000},{3500,5500} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = torque; CDA = 1.0f; LIMIT = 6000; DIFF = diff; RADIUS = radius; NG = n;
    T.fwd_gears = n; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    tc_set(stall, lock_gear, stall_rpm);
}
static void use_tc_ford3()   // 3-speed slushbox (never locks)
{
    float r[3] = { 2.46f, 1.46f, 1.00f };
    car_gas(520, 3.0f, 0.34f, 3, r, 2.0f, 0, 2600);
}
static void use_tc_ford4()   // 4-speed, locks in 4th
{
    float r[4] = { 2.84f, 1.55f, 1.00f, 0.70f };
    car_gas(520, 3.08f, 0.34f, 4, r, 1.9f, 4, 2600);
}
static void use_tc_ford4b()  // 4-speed overdrive, locks in 4th
{
    float r[4] = { 3.06f, 1.62f, 1.00f, 0.68f };
    car_gas(600, 3.31f, 0.34f, 4, r, 1.9f, 4, 2500);
}
static void use_tc_ford10()  // 10-speed (modern converter, locks from 2nd)
{
    float r[10] = { 4.70f, 2.99f, 2.15f, 1.80f, 1.52f, 1.28f, 1.00f, 0.85f, 0.69f, 0.64f };
    car_gas(600, 3.31f, 0.34f, 10, r, 2.1f, 2, 2500);
}
static void use_tc_allison(float torque, const char *who)   // an ETS2 Allison 4500 (6 speeds, stall 2.42), a truck engine
{
    tc_reset();
    float r[16] = { 4.7f, 2.21f, 1.53f, 1.0f, 0.76f, 0.67f };
    float c[8][2] = { {0,0},{600,0.62f},{900,1},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = torque; CDA = 6; LIMIT = 1900; DIFF = 3.91f; RADIUS = 0.538f; NG = 6;
    T.fwd_gears = 6; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    tc_set(2.42f, 2, 1500);   // the game def gives the stall; the lock gear is an assumption (locks from 2nd)
    (void)who;
}
static void use_tc_allison_2700() { use_tc_allison(2700, "2700 Nm"); }
static void use_tc_allison_1800() { use_tc_allison(1800, "1800 Nm"); }
static void use_tc_pickup()  // the RAM 3500 HD of the ride (6 speeds, 2700 Nm, limit 3200, 8.3 t with the trailer)
{
    tc_reset();
    float r[16] = { 3.23f, 1.84f, 1.41f, 1.00f, 0.82f, 0.63f };
    float c[8][2] = { {0,0},{500,0.55f},{900,1},{1400,1},{1800,0.97f},{2400,0.82f},{3000,0.45f},{3300,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,2500},{1000,1700},{1000,1700},{2200,3000} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2700; CDA = 2.0f; LIMIT = 3200; DIFF = 3.73f; RADIUS = 0.398f; NG = 6;
    T.fwd_gears = 6; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    tc_set(2.0f, 1, 2000);   // locks in 1st, as the ride showed (rpm proportional to the speed from ~17 km/h)
}
static struct {
    int gear, target, state, parked, engaged; double shift_end; float v, mass, grade_pct, pedal, cc, brake; int force, ebrake, ret;
    float cvar, shift_time, cruise_kmh; int eb_toggle; float eb_level, mass_seen, thr_floor; int hint, wish_on; float wish_thr, brake_unseen;   // brake_unseen: a brake the telemetry does not show (AEB in the ride)   // hint: a shift-hint press the game's handler turns into target +-1 before its update
    float ret_demand, ret_build;   // the game's continuous smart-overspeed retarder demand (0..1) and the built-up value after the ~1 s lag
    float smart_brake;             // the game's smart-overspeed service brake (0..0.15, only on a truck with no retarder / engine brake)
    float brake_applied;           // the service brake after the game's +-5/s slew toward M.brake
    int ret_gap; double ret_gap_t0, ret_gap_t1;   // the retarder's shift-gap (clutch open) state: in a gap, when it started, when the last ended
    double s;                 // distance driven (m)
    float (*road)(double s);  // grade (%) along the road, or NULL (scenario sets grade)
    int route_on;
    double junction_s;        // a junction at this distance (0 = none)
} M;

// ---- the route API on the sim road: an absolute height table built once per route (the road is a pure function of the
// distance), so h_ahead is an O(1) lookup and route_grade / route_crest_m / route_valley_m no longer re-integrate the grade
// from the start on every call. The table is the same 5 m midpoint rule h_ahead used before, just on a fixed grid, so the
// grades stay within float noise of the old per-call integration.
static const float R_STEP = 5.0f;   // a height sample every 5 m
#ifdef SIM_PCC
static float g_rh[16384];           // absolute height (m) at i*R_STEP along the route (81920 m: a replayed ride + look-ahead)
#else
static float g_rh[8192];            // (40960 m; the plugin's view of the road is flat beyond it. Some sim_pmsweep eco 'hill' / 'ovt' runs
                                    // get there: a larger table changes 5 of 612 of their lines, so it stays as it was outside sim_pcc)
#endif
static int g_rh_n;                  // valid samples (0 = not built)
static double g_hs_s = NAN; static float g_hs_h;   // h_abs at the truck (M.s), cached: route_grade asks it twice per call, ~200 calls a tick
static void route_build(float (*road)(double))
{
    g_hs_s = NAN;
    g_rh_n = (int)(sizeof g_rh / sizeof g_rh[0]);
    g_rh[0] = 0.0f;
    for (int i = 1; i < g_rh_n; i++) g_rh[i] = g_rh[i - 1] + road((i - 0.5f) * R_STEP) / 100 * R_STEP;
}
static float h_abs(float s)   // absolute height at route distance s (m)
{
    if (g_rh_n <= 0) return 0.0f;
    float x = s / R_STEP;
    int i = (int)x;
    if (i < 0) return g_rh[0];
    if (i >= g_rh_n - 1) return g_rh[g_rh_n - 1];
    return g_rh[i] + (g_rh[i + 1] - g_rh[i]) * (x - i);
}
static inline float h_here() { if (M.s != g_hs_s) { g_hs_s = M.s; g_hs_h = h_abs((float)M.s); } return g_hs_h; }   // (the same value h_abs gives)
static double h_ahead(float d) { return (double)(h_abs((float)(M.s + d)) - h_here()); }
void route_update() {}
static long g_rt_reads;   // G1: how often the code under test asked the route anything (route_ok, the grades, the crest); with [trueshift] lookahead 0 the gearbox asks nothing
int route_ok() { g_rt_reads++; return M.route_on && M.road; }
float route_ahead_m() { return route_ok() ? 1575 : 0; }
float route_grade(float d0, float d1)
{
    if (!route_ok() || d1 <= d0) return NAN;
    return (float)((h_ahead(d1) - h_ahead(d0)) / (d1 - d0) * 100);
}
static float ext_grade(float d0, float d1, int mx)
{
    if (!route_ok()) return NAN;
    float best = NAN;
    for (float d = d0; d + 50 <= d1 && d + 50 <= 1575; d += 25) {
        float g = route_grade(d, d + 50);
        if (best != best || (mx ? g > best : g < best)) best = g;
    }
    return best;
}
float route_max_grade(float d0, float d1) { return ext_grade(d0, d1, 1); }
float route_min_grade(float d0, float d1) { return ext_grade(d0, d1, 0); }
float route_crest_m(float max_d)
{
    if (!route_ok() || route_grade(0, 50) < 0.5f) return -1;
    for (float d = 25; d + 50 <= fminf(max_d, 1575); d += 25)
        if (route_grade(d, d + 50) < 0.2f) return d + 25;
    return -1;
}
// a junction on the route is a 30 km/h bend (only when the scenario has one; g_jn_kmh: another speed)
// (sim_pcc has its own route layer with bends, junctions and limit steps, pcc_route.h: these three are parked there)
#ifdef SIM_PCC
#define route_curve_speed sim_route_curve_speed
#define route_junction_m sim_route_junction_m
#define route_bend_speed sim_route_bend_speed
#endif
static float g_jn_kmh = 30;
enum { RR_BEND, RR_CURVE, RR_JUNCTION_M, RR_JN_SPEED, RR_LIMIT, RR_STRAIGHT, RR_JN_KNOWN, RR_N };
static long g_route_reads[RR_N];   // how often the code under test read each non-grade route function (sim_promises.cpp, P3: the gearbox must read none of them)
static int g_route_rich;   // sim_promises.cpp, promise P3: 1 = the route carries a junction, a bend and a roundabout every 500 m and a speed limit ahead (below); 0 = as the scenarios have it
static float rich_to_next(float every) { return every - (float)fmod(M.s, every); }
float route_curve_speed(float, float a_dec, float)
{
    g_route_reads[RR_CURVE]++;
    if (g_route_rich && route_ok()) { float vj = g_jn_kmh / 3.6f; return sqrtf(vj * vj + 2 * a_dec * rich_to_next(500)); }
    if (!route_ok() || !(M.junction_s > M.s)) return NAN;
    float d = (float)(M.junction_s - M.s), vj = g_jn_kmh / 3.6f;
    return sqrtf(vj * vj + 2 * a_dec * d);
}
float route_junction_m() { g_route_reads[RR_JUNCTION_M]++; return g_route_rich && route_ok() ? rich_to_next(500) : route_ok() && M.junction_s > M.s ? (float)(M.junction_s - M.s) : -1; }
// sim_promises.cpp, promise P3: g_route_rich = 1 makes the route carry what the scenarios do not have: a bend, a junction and a roundabout every 500 m, a speed limit ahead. The
// gearbox reads none of it (keuzes 12); the row is that it stays so. 0 = the scenarios as they were (the stubs below answer as they did).
float route_bend_speed(float, float) { g_route_reads[RR_BEND]++; return g_route_rich && route_ok() ? 25 / 3.6f : NAN; }   // the scenarios have no road bends, only the junction above
#ifndef SIM_PCC
float route_limit_ahead(float max_m, float *dist_m) { g_route_reads[RR_LIMIT]++; float d = route_ok() && g_route_rich ? rich_to_next(500) + 100 : NAN; if (dist_m) *dist_m = d; return d == d && d <= max_m ? 50 / 3.6f : NAN; }
float route_straight_m(float) { g_route_reads[RR_STRAIGHT]++; return route_ok() ? (g_route_rich ? rich_to_next(500) : 4000.0f) : -1; }
float route_valley_m(float) { return -1; }
int route_junction_known(float) { g_route_reads[RR_JN_KNOWN]++; return route_ok() && g_route_rich ? 1 : 0; }
float route_junction_speed(float a_dec, route_junction_t *info)
{
    g_route_reads[RR_JN_SPEED]++;
    if (!route_ok() || !g_route_rich) return NAN;
    const float d = rich_to_next(500);
    if (info) { info->dist_m = d; info->turn_deg = 90; info->limit_ms = 20 / 3.6f; info->ratio = 1; info->roundabout = 1; info->curves = 2; }
    return sqrtf((20 / 3.6f) * (20 / 3.6f) + 2 * a_dec * d);
}
#endif
#ifdef SIM_PCC
#undef route_curve_speed
#undef route_junction_m
#undef route_bend_speed
#endif
static float g_act_req;   // the brake request the suite itself holds (the deceleration assist in the ride; the telemetry does not show it)
#ifdef SIM_PCC
float sim_act_request() { return g_act_req; }
#else
float act_request() { return g_act_req; }
#endif
// The actuator's throttle wishes (core/actuate.cpp is not linked here): a scenario sets a cap or scale (the own speed limiter posts
// ACT_CRUISE's scale), and the throttle model below applies it, so TrueShift's thr_capped() sees it as in the game.
static act_wish g_act[ACT_N] = { {0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1} };
#ifdef SIM_PCC
const act_wish *sim_act_wishes() { return g_act; }
#else
const act_wish *act_wishes() { return g_act; }
#endif
static float sim_thr_cap()   // the merged throttle cap and scale, as the actuator applies them (base, then caps, then scales)
{
    float r = 1.0f;
    for (int o = 0; o < ACT_N; o++) if (g_act[o].cap < r) r = g_act[o].cap;
    for (int o = 0; o < ACT_N; o++) r *= g_act[o].scale;
    return r;
}
// The game's own speed limiter (g_use_speed_limiter), as the game applies it: a cap on the throttle it hands the engine. It is NOT an
// act_wish, so TrueShift's thr_capped() does not see it (the game's limiter is not one of our caps). A scenario sets it per tick; 1 = off.
static float g_game_lim = 1.0f;
static int g_thr_merged;   // sim_pcc PCC_LIM (WP-SP1): the engine gets the throttle the actuator merged (M.cc: the driver's pedal under the caps, as the actuator applies them), not the pedal times the caps again
static float sim_throttle()   // the throttle the engine gets: the base (pedal / wish / cruise), the actuator's caps and scales, the game's limiter
{
    // SW_GAMELIM=1 with SW_TOP (the limiter rows, WP-1111-limiter): the game's limiter holds the truck at its top speed as sim_pedalmix.cpp models it, a P on the overspeed (it holds ~top + 0.5 km/h)
    { static const bool gl = getenv("SW_GAMELIM") && getenv("SW_TOP"); static const float top = getenv("SW_TOP") ? (float)atof(getenv("SW_TOP")) : 0.0f;
      if (gl && top > 1.0f) g_game_lim = fmaxf(0.0f, fminf(1.0f, 0.5f + (top - M.v * 3.6f))); }
    float r = g_thr_merged ? M.cc : (M.wish_on ? M.wish_thr : fmaxf(M.pedal, M.cc)) * sim_thr_cap();
    return r < g_game_lim ? r : g_game_lim;
}
static void *SELF = (void *)&M;
static char letter = 'A';

char *gb_inner(void *) { return (char *)&M; }
int gb_gear(void *) { return M.gear; }
int gb_target(void *) { return M.target; }
int gb_state(void *) { return M.parked ? 1 : M.state; }
int gb_engage(void *, int g) { if (!M.parked) return 0; M.target = g; M.engaged = 1; M.parked = 0; return 1; }
int game_engage_init() { return 1; }
void gb_force(void *) { M.force = 1; }
float gb_thr_in(void *) { return M.pedal; }
float gb_thr_eff(void *) { return fmaxf(sim_throttle(), M.thr_floor); }  // thr_floor: the game's 0.08 at pedal 0 (24-10, 1831 s); wish_on: the wish pedal's throttle
float wheel_radius() { return RADIUS * g_radius_err; }
// WP-TC1: a mod engine that defines its torque only (the Spectrans Scania of the ride of 04-11, dc13_450.sii): what the PLUGIN reads of the game's engine object holds no
// curve, no ranges (and, g_mod_limit0, no limit); the plant (called with self = 0) keeps its real curve. The stub then does what game.cpp's gb_engine does with such a read
// (engine_raw_check); TC1_BASE builds the stub of 2aa64e7 (a read with n_curve < 2 failed), so the rows can be run against that commit.
static int g_mod_engine, g_mod_limit0;
// WP-TC2: the game's transmission object as the plugin reads it (self != 0 only; the plant asks with 0). g_fake_tr_on = the stub reads it with engine_read_stall like game.cpp's
// gb_engine does (the 12 bytes ending at the shift time at TC2_SHIFT_OFF, the ETS2 1.61 layout); off = no transmission object (e->tc_stall stays 0: not read, as every other stub).
static char g_fake_tr[0x240]; static int g_fake_tr_on;
#define TC2_SHIFT_OFF 0x1c8u
static void fake_tr_set(float stall, float diff, float shift) { memset(g_fake_tr, 0, sizeof g_fake_tr); memcpy(g_fake_tr + TC2_SHIFT_OFF - 8, &stall, 4); memcpy(g_fake_tr + TC2_SHIFT_OFF - 4, &diff, 4); memcpy(g_fake_tr + TC2_SHIFT_OFF, &shift, 4); g_fake_tr_on = 1; }
int gb_engine(void *self, engine_info *e)
{
    e->torque = TORQUE; e->rpm_limit = LIMIT; e->n_curve = NCURVE;
    memcpy(e->curve, CURVE, sizeof CURVE);
    memcpy(e->range, RANGES, sizeof RANGES);
    e->shift_time = M.shift_time;
    e->tc_stall = 0;
    if (self && g_fake_tr_on) engine_read_stall(e, g_fake_tr, TC2_SHIFT_OFF, T.diff);
    if (g_mod_engine && self) {
        e->n_curve = 0; memset(e->curve, 0, sizeof e->curve); memset(e->range, 0, sizeof e->range);
        if (g_mod_limit0) e->rpm_limit = 0;
#ifdef TC1_BASE
        return 0;
#else
        return engine_raw_check(e);
#endif
    }
    return 1;
}
float game_mass() { return M.mass_seen > 0 ? M.mass_seen : M.mass; }  // mass_seen: a wrong estimate
float game_mass_own() { return game_mass(); }
int game_vehicle_is_car() { return 0; }
float game_car_speed_limit_ms() { return NAN; }
#ifdef SIM_PCC
static float g_grade_add;   // sim_pcc's pitch artefact (PCC_NOISE_PITCH): what the gearbox's grade reads on top of the road (0 = off)
float game_grade_deg(void *) { return g_grade_add != 0 ? atanf((M.grade_pct + g_grade_add) / 100) * 57.2958f : atanf(M.grade_pct / 100) * 57.2958f; }
#else
float game_grade_deg(void *) { return atanf(M.grade_pct / 100) * 57.2958f; }
#endif
float game_adaptive() { return M.cvar; }
int game_adaptive_blocked() { return 0; }
int game_letter(void *, char c) { letter = c; return 1; }
void *game_gearbox() { return (void *)&M; }
int game_ebrake_downshift(void *) { return M.eb_toggle; }
float game_ebrake_level(void *) { return M.ebrake ? M.eb_level : (M.eb_level >= 0 ? 0 : -1); }
float curve_factor(const engine_info *e, float rpm)
{
    if (rpm <= e->curve[0][0]) return e->curve[0][1];
    for (int i = 1; i < e->n_curve; i++)
        if (rpm <= e->curve[i][0])
            return e->curve[i-1][1] + (rpm - e->curve[i-1][0]) / (e->curve[i][0] - e->curve[i-1][0]) * (e->curve[i][1] - e->curve[i-1][1]);
    return e->curve[e->n_curve-1][1];
}

static float g_idle_sim = 600;   // the idle of the engine of the sim (SCORES part 7 has one with 700)
// Torque converter (the plant model; the plugin's vehicle model is primed with the same numbers by sw_tc_apply / use_tc_*). Below the
// lockup gear (or always, with TC_LOCK_GEAR 0) the engine runs at TC_STALL_RPM and the converter multiplies the torque by TC_STALL
// fading to 1 as the driveline (turbine) catches up: the slip ratio SR = turbine rpm / stall rpm, K = TC_STALL + (1 - TC_STALL) * SR.
static float TC_STALL = 0;        // stall torque ratio (0 = no converter)
static int TC_LOCK_GEAR = 0;      // first gear with lockup (0 = never locks)
// The game's converter off the stall (ride 31-10, Crown Victoria): coupled and pulling the engine sits TC_PULL_K over the driveline, with
// the pedal up it sits TC_COAST_K under it and never under TC_COAST_FLOOR (the idle in gear). 0 = the ideal coupling (the fleet)
static float TC_COAST_K = 0, TC_PULL_K = 0, TC_COAST_FLOOR = 0;
static int TC_PLANT_LOCK = 0;     // > 0: the plant runs the gears over this one locked (no slip, no multiplication), as the game's Allison does (ride 06-11, tools/sim/sim_allison.inc); 0 = the plant slips in every gear (the other converter rows)
static float TC_STALL_RPM = 0;    // engine speed at full stall (0 = derive: 85 % of the power range's bottom, never under 60 % of the limit)
static float tc_stall_rpm() { return TC_STALL_RPM > 0 ? TC_STALL_RPM : fmaxf(RANGES[3][0] * 0.85f, 0.6f * LIMIT); }
// The engine speed the converter holds when the turbine is slower, at the throttle of now: from idle with the pedal shut to the
// stall speed floored (matches the plugin's vehicle model; idle 550 as there).
static float tc_couple_rpm() { float t = sim_throttle(); if (t < 0) t = 0; if (t > 1) t = 1; return 550.0f + (tc_stall_rpm() - 550.0f) * t; }
static float tc_slip_now()
{
    if (!(TC_STALL > 1.0f)) return 1.0f;
    if (TC_PLANT_LOCK > 0 && M.gear > TC_PLANT_LOCK) return 1.0f;
    float rpm = M.v / RADIUS * RATIO[M.gear - 1] * DIFF * 9.5493f;   // the turbine (driveline) rpm
    float couple = tc_couple_rpm();
    if (!(couple > 0) || !(rpm > 0)) return 1.0f;
    return fminf(1.0f, rpm / couple);
}
static void tc_reset() { g_mod_engine = g_mod_limit0 = 0; g_fake_tr_on = 0; TC_STALL = 0; TC_LOCK_GEAR = 0; TC_PLANT_LOCK = 0; TC_STALL_RPM = 0; TC_COAST_K = TC_PULL_K = TC_COAST_FLOOR = 0; g_radius_err = 1; NCURVE = 8; }
static void tc_set(float stall, int lock_gear, float stall_rpm) { TC_STALL = stall; TC_LOCK_GEAR = lock_gear; TC_STALL_RPM = stall_rpm; vehicle_tc_prime(stall, lock_gear, stall_rpm); }
static void use_tc_crownvic(bool prime)   // the ATS Crown Victoria 2006 of the ride (31-10): 4R70W 4-speed, diff 3.27, 0.348 m, 2.0 t; prime = the converter known, else the plugin detects it
{
    tc_reset();
    float r[16] = { 2.84f, 1.55f, 1.00f, 0.70f };
    float c[9][2] = { {0,0},{1000,0.47f},{1500,0.71f},{2000,0.83f},{3000,0.95f},{4100,1},{4500,0.96f},{5200,0.77f},{6100,0} }; NCURVE = 9;
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {3500,5600},{1200,3000},{1600,3400},{3500,5600} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 389; CDA = 1.0f; LIMIT = 6000; DIFF = 3.27f; RADIUS = 0.348f; NG = 4;
    T.fwd_gears = 4; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    if (prime) tc_set(2.0f, 4, 4300);
    else { TC_STALL = 2.0f; TC_LOCK_GEAR = 4; TC_STALL_RPM = 4300; vehicle_tc_prime(0, 0, 0); }   // the plant has the converter, the plugin must find it
}
static void use_tc_crownvic_game(bool prime)   // the same with the game's converter off the stall: over the driveline pulling, under it coasting, an idle in gear under which it never falls
{
    use_tc_crownvic(prime);
    TC_COAST_K = 0.94f; TC_PULL_K = 1.04f; TC_COAST_FLOOR = 870;
}
// The game's clutch at a launch (1.62; the eight pull-aways of the ride of 08-11, notes/rit-0811-analyse.md section 4): in gear and below the speed where the driveline reaches it, the clutch slips and the engine
// is held at g_slip_k x idle (636 .. 740 rpm at an idle of 550: 1.16 .. 1.35, 1.25 the mean); the telemetry says the clutch is nearly open (0.95; the ride's column read 0.90 .. 0.99), the engine gives the torque it
// has at that rpm and the pedal's throttle, and above it the clutch is closed and the engine follows the driveline. 0 = the clutch of the older sims (the rpm is the driveline's, floored at the idle by tick()).
// Set by the launch scenario of the sweep (sim_sweep.inc; the figure is CLUTCH_SLIP_K of core/vehicle.h, SW_SLIP_K overrides it).
static float g_slip_k = 0.0f;
static bool g_slipping;   // set by rpm_now()
static float rpm_now()
{
    g_slipping = false;
    if (M.gear <= 0) return TORQUE < 1000 ? 750 : g_idle_sim;
    float rpm = M.v / RADIUS * RATIO[M.gear - 1] * DIFF * 9.5493f;
    if (g_slip_k > 0.0f && !M.state) { const float rs = g_idle_sim * g_slip_k; if (rpm < rs) { g_slipping = true; return rs; } }
    if (TC_STALL > 1.0f) {
        float sr = tc_slip_now();
        if (sr < 1.0f && sr > 0.01f) return rpm / sr;   // the engine sits above the driveline while the converter slips
        if (TC_COAST_K > 0) {
            float t = sim_throttle();
            if (t < 0.03f) return fmaxf(TC_COAST_FLOOR, rpm * TC_COAST_K);
            if (TC_PULL_K > 0 && t < 0.7f) return rpm * TC_PULL_K;
        }
    }
    return rpm;
}

// hooks: capture the stubs
static window_fn s_decide; static gear_fn s_choose, s_start; static void (*s_tail)(void *);
static float *f_window(void *, float *out, unsigned g)
{
    float k = (g - 1) / (float)(NG - 1);
    out[0] = RANGES[1][0] + k * (RANGES[2][0] - RANGES[1][0]); out[1] = RANGES[1][1] + k * (RANGES[2][1] - RANGES[1][1]);
    return out;
}
// Rough stock chooser: one gear toward the window (two up when light).
static int f_choose(void *)
{
    M.force = 0;
    if (M.gear == 0) {  // from neutral at speed: the gear nearest 1100 rpm (a guess at the game)
        int best = 1; float be = 1e9f;
        for (int g = 1; g <= NG; g++) { float r = M.v / RADIUS * RATIO[g-1] * DIFF * 9.5493f; if (fabsf(r - 1100) < be) { be = fabsf(r - 1100); best = g; } }
        return best;
    }
    float w[2], r = rpm_now();
    if (TC_STALL > 1.0f) r = M.v / RADIUS * RATIO[M.gear - 1] * DIFF * 9.5493f;   // a converter shifts by the turbine (driveline) speed, not the engine's
    f_window(0, w, M.gear);
    if (r > w[1] && M.gear < NG) return M.gear + (M.pedal < 0.6f && M.gear < NG - 1 ? 2 : 1);
    if (r < w[0] && M.gear > 1) return M.gear - 1;
    return M.gear;
}
static int f_start(void *) { return M.mass > 20000 ? 1 : 4; }
int game_patch_call(uintptr_t site, void *stub)
{
    if (site == 1) s_decide = (window_fn)stub;
    if (site == 2) s_choose = (gear_fn)stub;
    if (site == 4) s_start = (gear_fn)stub;
    return 1;
}
int game_patch_tail(uintptr_t site, void *pre) { s_tail = (void (*)(void *))pre; return 1; }
int game_patch_stop(uintptr_t, float (*)(float, float, float)) { return 0; }
int low_owner_on() { return 0; }   // the low-speed owner (lowspeed.cpp) is not part of this sim
int low_owner_wanted() { return 0; }
int low_start_gear() { return 0; }
int low_launch_hold() { return 0; }
int low_handed() { return 0; }
float low_stop_rpm(float idle, float, float, int) { return 0.8f * idle; }
void gb_set_target(void *, int g) { M.target = g; }
#ifdef SIM_PCC
void sim_game_cvar_set(void *, float v) { M.cvar = v; }   // (sim_pcc links the real game_cvar setter: this stub is parked)
#else
void game_cvar_set(void *, float v) { M.cvar = v; }
#endif

module *g_modules[] = { &m_trueshift, &m_carshift };
int g_n_modules = 2;

void telemetry_frame()
{
    static double last_t; static float last_v;
    double dt = t_now - last_t;
    if (dt > 0.0 && dt < 0.5) { float a = (float)((fabsf(T.speed) - last_v) / dt); float k = (float)(dt / (0.5 + dt)); T.accel += k * (a - T.accel); }
    last_t = t_now; last_v = fabsf(T.speed);
}

// ---- run statistics
static int shifts, ups, hunts, hunts_drv, last_dir; static double last_shift_t;   // hunts: reversals at an unchanged foot (hunt_kind); hunts_drv: reversals after the driver's own foot change, printed apart
// A row that asks (g_row_extra) also says what the plant burnt and the shortest time between two shifts (WP-1011-T6C: a row on a cruise over hills is judged on the gears AND on the diesel and the
// distance they cost). The fuel is the plant's own (g_fuel_raw, the model the game has: a x omega x (T_net + T_f), or the fitted one with SW_FUEL=fit), in the plant's liters.
// The owner's yardstick (10-11): the top gear where the cruise holds its set speed and 12 carries the road with a 5 % margin (hold_s: the seconds the cruise is on the throttle and 12 can, hold12_s: of them
// in 12; a cruise that coasts is not counted: 11 as a precaution is fine), the shifts on a climb (grade >= 0.5 %: per climb, the shortest gap on one), and the seconds 12 sits at a full throttle under
// the plateau start P1 (lug1_s) and under P1 + 50 rpm (lug_s): lugging where it cannot cope.
static double g_row_hold_s, g_row_hold12_s, g_row_lug_s, g_row_lug1_s, g_row_cl_last, g_row_cl_gap; static int g_row_climbs, g_row_in_climb, g_row_cl_n[256], g_row_cl_id = -1;
static char g_rowb[24];
static int g_row_extra; static double g_row_gap_min, g_row_fuel0, g_row_t0, g_row_flat_s, g_row_flat_top_s;   // (flat_s: seconds in gear on a road that is flat or falls, flat_top_s: of them in the top gear)
// the climbs of the row in a line: how many, the shifts on them (in all, the most on one, the shortest gap between two on one), the 12 share where it carries the cruise's hold, the lugging seconds
static void row_yard(char *b, size_t n)
{
    int tot = 0, mx = 0, used = 0;
    for (int i = 1; i <= g_row_climbs && i < 256; i++) { tot += g_row_cl_n[i]; if (g_row_cl_n[i] > mx) mx = g_row_cl_n[i]; used++; }
    snprintf(b, n, "climbs %d, shifts on them %d (most on one %d, shortest gap %s), 12 where it carries the hold %.0f %% of %.0f s, 12 at full throttle under P1 %.1f s, under P1 + 50 rpm %.1f s",
             g_row_climbs, tot, mx, g_row_cl_gap < 1e8 ? (snprintf(g_rowb, sizeof g_rowb, "%.1f s", g_row_cl_gap), g_rowb) : "none", g_row_hold_s > 0 ? 100 * g_row_hold12_s / g_row_hold_s : 100.0, g_row_hold_s, g_row_lug1_s, g_row_lug_s);
}
static double up_rpm_sum; static int up_n;
static double up_rpm_min, rpm_all_sum, rpm_all_t, skip_rpm_min, skip_pull_min; static int skips_up, hunts_ts, shifts_ts;   // RELAX study: landing rpm, mean rpm below 50 km/h, skips, and shifts/hunting above the takeover speed
static double rpm_sum, rpm_t;   // time-weighted rpm in the second half
// The demand the driver or the cruise/planner asks now, as TrueShift reads it (the fair hunt metric): a share 0..1 of what the truck can give
// at this speed - the wish pedal's share, else the cruise's wish share, else the throttle. A share, not a force, so a held pedal stays the same
// demand while the speed changes. A direction reversal within 5 s is only a hunt when this stayed about the same between the two shifts; a clear
// press or release is the driver driving, not the box hunting.
static float sim_demand()
{
    float f = wish_share_eff();
    if (!(f == f)) f = cruise_wish_share();
    if (!(f == f)) f = M.gear > 0 ? fmaxf(M.pedal, M.cc) : 0.0f;
    return f;
}
// THE ONE HUNT DEFINITION (K2 step 1; core value 4, one fact one place). Every counter of the sim and of the rows (the run's `hunts`, 18c back, 18i-2, 18i-6, run.sh's printed totals, the sweep's field 18)
// calls hunt_kind, nothing else decides what a hunt is. A reversal is a shift in the direction opposite to the one before it, made within HUNT_WINDOW_S of that one's completion (the longer of the
// plugin's two dwells, dwell_up_down_s 3 s, and two shift durations of the sim's boxes: about 5 s). It is a HUNTING PAIR when the driver's foot did not move between the two DECISIONS: the demand
// stayed within the noise of the demand at the first decision the whole way (the path, not only the two end points: a lift and a re-press to the same level is a moved foot).
// It is a DRIVER REVERSAL when the foot moved (a lift, a press): the box answering the driver, printed apart and never a defect; its own rows judge it (lift_rows.py).
// The noise is the plugin's (core/shiftcommon.cpp sc_reversal_weight / dm_thresh_at): a quarter of the demand at the first decision, never under 3 % of full demand.
static const double HUNT_WINDOW_S = 5.0;
static float hunt_noise(float dm) { return fmaxf(0.03f, 0.25f * fabsf(dm)); }
enum { HUNT_NONE = 0, HUNT_PAIR = 1, HUNT_DRIVER = 2 };
static int hunt_kind(int prev_dir, int dir, double gap_s, float foot_dev, float foot_ref)
{
    if (!prev_dir || dir == prev_dir || gap_s >= HUNT_WINDOW_S) return HUNT_NONE;
    return foot_dev <= hunt_noise(foot_ref) ? HUNT_PAIR : HUNT_DRIVER;
}
static float g_foot_ref, g_foot_dev, g_foot_ref_pend, g_foot_dev_pend;   // the run's foot track (tick): the demand at the last decision, how far it strayed since; the same for the shift in progress
// The recorded shifts of a row (g_rec) counted with the one definition: consecutive real gears only (not an eco-roll through neutral). *drv gets the driver reversals, printed apart.
static int rec_pairs(int *drv = NULL)
{
    int n = 0; if (drv) *drv = 0;
    for (int i = 0; i + 1 < g_rec_n; i++) {
        if (g_rec[i].a <= 0 || g_rec[i].b <= 0 || g_rec[i + 1].a <= 0 || g_rec[i + 1].b <= 0) continue;
        const int k = hunt_kind(g_rec[i].b > g_rec[i].a ? 1 : -1, g_rec[i + 1].b > g_rec[i + 1].a ? 1 : -1, g_rec[i + 1].t - g_rec[i].t, g_rec[i + 1].dev, g_rec[i].dm);
        if (k == HUNT_PAIR) n++; else if (k == HUNT_DRIVER && drv) (*drv)++;
    }
    return n;
}
static struct { int on, on60; double kj, rs, rt, pk, kj60, t60; } EN;   // the energy table: engine energy (kJ, torque x rpm x time), rpm in gear, over a window a controller opens and closes
static double g_sh_t, g_rpm_s, g_rpm_t; static int g_dskips;   // SCORES table: seconds in shifts, rpm in gear over the whole run, downward skips; g_quiet = run() prints nothing

// A session reset as core/telemetry.cpp session_reset does it: the vehicle model first, then the module.
// The driver profile as the plugin feeds it (core/driverstore.cpp driver_frame, without the file): the demand history takes the effective share once a second and the learners
// the driver's pedal, brake, speed and gear. Off for every part but the layers' (part 18..18i switch it on: the history and the lift habits are what those rows measure; SC_NOFEED=1
// switches it off there too, for a comparison with the uniform prior of the first rewrite rounds). Every run starts with a fresh profile (g_driver_inject: a profile's text to
// start from instead, a learned driver).
static int g_driver_feed;
static const char *g_driver_inject;
static void (*g_driver_pre)();   // a row that starts from a prepared profile (a calm driver) hands it over here: it runs after the fresh start of every run
static void sim_driver_feed()
{
    demand_tick(sim_s(), wish_share_eff());
    driver_sample s;
    s.t = sim_s(); s.pedal = T.in_thr; s.brake = T.in_brake; s.v = fabsf(T.speed); s.mass = V.mass_ok ? V.mass : 0.0f; s.gear = T.gear;
    s.own = T.engine_on && !T.paused && !(T.cruise > 0.1f) && !g_lv.crawl_toggle && !g_lv.lcx;
    s.launch_share = wish_launch_share(T.in_thr);
    driver_step(&s);
}
static void sim_reset(int level, const char *why) { vehicle_reset(level); m_trueshift.reset(level, why); m_carshift.reset(level, why); if (g_driver_feed) driver_drop_open(); if (level >= RESET_HARD) demand_ctl_reset(); }

static int g_rst_every = 0, g_rst_level = 0;   // robustness runs: inject a session reset every N seconds
static float g_plant_eta = getenv("SW_PLANT_ETA") ? (float)atof(getenv("SW_PLANT_ETA")) : 0.9f, g_thr_k = getenv("SW_THR_K") ? (float)atof(getenv("SW_THR_K")) : 1.0f;
static float g_plant_shift_drag = 0, g_plant_shift_extra = 0;   // part 18e: the plant loses this much extra speed in a shift gap (m/s2) and the gap lasts this much longer (s) than the plugin's engine info says
static float g_plant_res = getenv("SW_PLANT_RES") ? (float)atof(getenv("SW_PLANT_RES")) : 0.0f;   // an extra plant resistance (N) the model does not know (SW_PLANT_RES)
static float g_accel_lag = 0;   // (kept for the FBIAS part: T.accel is always a 0.5 s lag of the plant acceleration, telemetry_frame)
static float g_sw_tf = getenv("SW_TF") ? (float)atof(getenv("SW_TF")) : 0.09f;                     // the plant's friction torque as a share of peak torque (fuel model)
static float g_sw_drag = getenv("SW_DRAG") ? (float)atof(getenv("SW_DRAG")) : 0.12f;               // the plant's in-gear engine drag as a share of peak torque (at the (r/LIMIT) shape below)
static float g_fuel_scale = getenv("SW_FUEL_SCALE") ? (float)atof(getenv("SW_FUEL_SCALE")) : 1.0f; // the map scale 'a'
static float g_fuel_scale2 = getenv("SW_FUEL_SCALE2") ? (float)atof(getenv("SW_FUEL_SCALE2")) : 0.0f;   // the scale after the switch (0 = no switch)
static float g_fuel_switch_s = getenv("SW_FUEL_SWITCH_S") ? (float)atof(getenv("SW_FUEL_SWITCH_S")) : 0.0f;
static const bool g_fuel_fit = getenv("SW_FUEL") && !strcmp(getenv("SW_FUEL"), "fit");   // SW_FUEL=fit: the fitted model (fuel_fit.h) instead of the constant-friction form below
static const float FL_C = 1.0e-6f;   // the fuel model's liter constant (Nm*rad -> L): big enough that a window's burn is far above the 1 mL resolution
static double g_fuel_used;   // everything the plant has burnt since the start (the tank above reads 0 once it is drained and is refilled by some rows; this one only counts: a row's burn is the difference)
static double g_fuel_raw = 1000.0;   // the tank level the plant drains (a double: 1 mL of a 1000 L tank is under a float's resolution)
static inline float plant_thr(float thr) { return g_thr_k == 1.0f ? thr : powf(fmaxf(thr, 0.0f), g_thr_k); }   // the share of the curve's torque the plant gives
static void (*g_tick_hook)(double);   // SWEEP: a per-tick statistics hook (after the physics)
static int g_dw_n, g_dw_bad, g_dw_before, g_dw_last; static double g_dw_t;   // DW rows: decisions inside the dwell, those that name the wrong gear, the gear the last shift left, the gear before, when it was left
static void dw_track(double)
{
    if (M.gear > 0 && M.gear != g_dw_last) { if (g_dw_last > 0) { g_dw_before = g_dw_last; g_dw_t = t_now; } g_dw_last = M.gear; }
    double at = -1; const gc_out *go = ts_last_choice(&at);
    if (g_dw_before > 0 && go && at > g_dw_t + 0.05 && at >= t_now - 0.1 && t_now - g_dw_t < 1.5) { g_dw_n++; if (go->prev != g_dw_before) g_dw_bad++; }
}
#ifdef SIM_PCC
static void (*g_pcc_tick)(float);     // sim_pcc.cpp: the real Cruise (cr_frame + act_stub) runs each tick and posts M.cc and the service brake
static void (*g_pcc_pre)(double);     // sim_pcc.cpp: the telemetry the route layer adds (limit, yaw, steering, pitch noise), before the modules' frames
#endif
static int g_start_floor;   // R0611: the 1.62 start wrapper puts the max of the start gear and its lower bound in; > 0: the gear the box takes at a launch is at least this
static int g_hint_neutral;   // 1: a +/- press in neutral moves the target as in gear (the ER1 rows: the driver's press ends a roll nobody asked for)
static int g_sim_parked;   // run()'s standing ticks (the gear registered at a standstill before the speed is set) are a parked truck: TrueShift does not act in them (its ours is 0 with the parking brake), as before WP-TC1
static void tick(double dt, int stats)
{
    t_now += dt;
    if (g_rst_every > 0 && fmod(t_now, g_rst_every) < dt) sim_reset(g_rst_level, "sim");
    T.sim_time = (scs_timestamp_t)(t_now * 1e6);
    float rpm = rpm_now();
    float idle = TORQUE < 1000 ? 750 : g_idle_sim;
    if (rpm < idle) rpm = idle;   // clutch slips below idle
    T.rpm = rpm; T.speed = M.v; T.in_thr = M.pedal; T.eff_thr = sim_throttle(); g_pedal_mapped = M.wish_on;
    T.rpm_limit = LIMIT;   // the telemetry's engine rpm limit, for the vehicle class (a pickup at 3200 rpm is a light vehicle)
    T.in_brake = M.brake; T.eff_brake = M.brake; T.cruise = M.cruise_kmh / 3.6f;
    T.motor_brake = g_sdk_motor_brake >= 0 ? g_sdk_motor_brake : M.ebrake; T.retarder = M.ret; T.eff_clutch = M.state ? 1 : g_slipping ? 0.95f : 0;
    T.engine_on = 1; T.gear = M.gear; T.dgear = M.gear; T.parking_brake = g_sim_parked;
    if (M.road) M.grade_pct = M.road(M.s);
#ifdef SIM_PCC
    T.place.orientation.pitch = atanf(M.grade_pct / 100) / 6.2831853f;   // the real Cruise reads the grade from the placement pitch (V.pitch_pct)
    if (g_pcc_pre) g_pcc_pre(dt);   // sim_pcc's route layer: the limit sign passed, the path's yaw rate and steering, the pitch artefact (pcc_route.h)
#endif
    telemetry_frame();
    vehicle_frame();
    if (g_driver_feed) sim_driver_feed();   // (the model layer's frame, core/model.cpp: after the vehicle model, before the modules)
    {   // the controller's memory (core/driverstore.cpp driver_frame): the game feeds it in every run, the driver's only where a row hands over a profile
        float cs = NAN;
        if (T.engine_on && !T.paused && T.cruise > 0.1f && T.in_thr <= 0.02f) { const float w = cruise_wish_share(); cs = w == w ? w : T.eff_thr; }
        demand_ctl_tick(sim_s(), cs);
    }
    // the dispatcher (core/main.cpp): the class moves here, a change resets both brains hard, and only the brain of the class runs
    vclass_sync();
    brain().frame();
    { const float d = sim_demand(); g_foot_dev = fmaxf(g_foot_dev, fabsf(d - g_foot_ref)); if (g_rec_on) g_rec_dev = fmaxf(g_rec_dev, fabsf(d - g_rec_dm_ref)); }   // the driver's foot, tracked for the hunt definition
    // gearbox
    if (M.state && t_now >= M.shift_end) {
        int dir = M.target > M.gear ? 1 : -1;
        if (dir > 0 && M.gear > 0) { ups++; up_rpm_sum += rpm_now(); up_n++; if (M.target > M.gear + 1) skips_up++;
            float lr = M.v / RADIUS * RATIO[M.target-1] * DIFF * 9.5493f; if (lr < up_rpm_min && M.v * 3.6f >= 8) up_rpm_min = lr;
            if (M.target > M.gear + 1 && M.v * 3.6f >= 8) {   // a skip of ours: landing rpm and the pull right after it at the pedal of now
                engine_info e; gb_engine(0, &e);
                float F = TORQUE * curve_factor(&e, lr) * fmaxf(M.pedal, M.cc) * RATIO[M.target-1] * DIFF * 0.9f / RADIUS;
                float Fr = M.mass * 9.81f * (sinf(atanf(M.grade_pct / 100)) + 0.006f) + 0.5f * 1.2f * CDA * M.v * M.v;
                if (lr < skip_rpm_min) skip_rpm_min = lr;
                if ((F - Fr) / M.mass < skip_pull_min) skip_pull_min = (F - Fr) / M.mass;
            } }
        if (dir < 0 && M.gear > 0 && M.gear - M.target > 1) g_dskips++;
        const int hk = hunt_kind(last_dir, dir, t_now - last_shift_t, g_foot_dev_pend, g_foot_ref_pend);   // (the foot's path between the two decisions, recorded at this shift's initiation)
        if (M.v * 3.6f >= 8) { shifts_ts++; if (hk == HUNT_PAIR) hunts_ts++; }
        if (hk == HUNT_PAIR) hunts++; else if (hk == HUNT_DRIVER) hunts_drv++;
        if (last_dir && t_now - last_shift_t < g_row_gap_min) g_row_gap_min = t_now - last_shift_t;
        if (g_row_extra && M.grade_pct >= 0.5f) {   // a shift on a climb: counted for that climb, and the gap to the one before it when that was on the same climb
            if (g_row_cl_id == g_row_climbs && t_now - g_row_cl_last < g_row_cl_gap) g_row_cl_gap = t_now - g_row_cl_last;
            g_row_cl_id = g_row_climbs; g_row_cl_last = t_now; g_row_cl_n[g_row_climbs < 255 ? g_row_climbs : 255]++;
        }
        last_dir = dir; last_shift_t = t_now;
        { static const bool sl = getenv("SIM_SHIFTLOG") != NULL; if (sl) printf("SHIFT %.1f g%d>%d v %.1f grade %.1f thr %.2f\n", t_now, M.gear, M.target, M.v * 3.6f, M.grade_pct, M.cc); }
        M.gear = M.target; M.state = 0; shifts++;
    }
    // The game's update: in gear the decide site + chooser; in neutral (game,
    // lka-roll-re notes B) re-engage on throttle via the start-gear fn; then
    // the tail (our eco-roll hook) right before the gearbox.
    if (!M.state) {
        M.target = M.gear;
        // a hint press (the exe: target +-1 in the handler, before the update); the chooser below overwrites it, as the game's call site does
        if (M.hint && (M.gear > 0 || g_hint_neutral)) { M.target = M.gear + M.hint; if (M.target < 1) M.target = 1; if (M.target > NG) M.target = NG; }   // (g_hint_neutral: the handler also writes the target in neutral, test_er1.cpp; no other row sets M.hint at gear 0)
        M.hint = 0;
        if (M.gear > 0) {
            float w[2];
            s_decide(SELF, w, M.gear);
            int stock_trigger = rpm < w[0] || rpm > w[1];
            if (M.force || stock_trigger) { M.target = s_choose(SELF); M.force = 0; }
        } else if (M.pedal > 0.01f) { M.target = s_start(SELF); if (g_start_floor > M.target) M.target = g_start_floor; }
        M.engaged = 0;
        if (s_tail) s_tail(SELF);
        // The generic gearbox parks in its gap state with gear 0 and a target
        // until there is throttle (neutral-exit-re notes § 1); gb_engage leaves it.
        if (M.gear == 0 && !(M.pedal > 0.01f) && !M.engaged) { M.parked = M.target > 0; M.target = 0; }
        else M.parked = 0;
        if (M.target != M.gear) { M.state = 1; M.shift_end = t_now + M.shift_time + g_plant_shift_extra; { const float d = sim_demand(); g_foot_ref_pend = g_foot_ref; g_foot_dev_pend = g_foot_dev; g_foot_ref = d; g_foot_dev = 0; } }   // the foot's path since the last decision, kept for the hunt definition
    }
#ifdef SIM_PCC
    if (g_pcc_tick) g_pcc_tick(dt);   // the real Cruise drives M.cc and the service brake (after TrueShift's eco-roll tail, before the physics)
#endif
    // physics
    float F = 0, thr = sim_throttle();
    if (!M.state && M.gear > 0) {
        float r = rpm_now();
        engine_info e; gb_engine(0, &e);
        float f = r > LIMIT ? 0 : curve_factor(&e, r < idle ? idle : r);
        float mult = 1.0f;
        if (TC_STALL > 1.0f) { float sr = tc_slip_now(); if (sr < 1.0f) mult = TC_STALL + (1.0f - TC_STALL) * sr; }   // the converter multiplies below the coupling
        // (SW_PLANT_ETA / SW_THR_K: the plant differs from the plugin's model, as the game may: its driveline efficiency, and a torque that is not
        // linear in the throttle, torque ~ thr^k; the defaults 0.9 / 1 are the model itself)
        F = TORQUE * f * plant_thr(thr) * mult * RATIO[M.gear-1] * DIFF * g_plant_eta / RADIUS;
        if (M.ebrake && thr < 0.05f) F -= (r / LIMIT) * g_eb_power / (M.v + 1) * (M.eb_level >= 0 ? M.eb_level : 0.5f);
        if (thr < 0.05f) F -= TORQUE * g_sw_drag * RATIO[M.gear-1] * DIFF / RADIUS * (r / LIMIT);  // engine drag in gear
    }
    // The driver's lever (M.ret, the D-series scenarios) stays the old step model. The game's smart-overspeed demand
    // (M.ret_demand, sim_pcc) uses the measured retarder model from sim_decel.cpp: the top step is 3.0 m/s2 at 18 t
    // (proportional in the demand - the notes give no step mapping, so this is an assumption), scaled by 18 / mass,
    // built up with a ~1 s lag, faded out between 6 and 4.5 km/h, dropped to a floor during a shift gap (clutch open) and
    // recovered over the first 0.3 s after it closes. Only in gear: in neutral the drivetrain retarder is disconnected.
    static const float RET18_MAX = 3.0f, RET18_FLOOR = 0.4f, RET_BUILD_S = 0.4f, RET_FALL_S = 0.2f, RET_RISE_S = 0.3f, RET_FADE_LO = 4.5f, RET_FADE_HI = 6.0f;
    if (M.ret && thr < 0.05f) F -= M.ret / 5.0f * 25000 * fminf(1, M.v / 10);
    M.ret_build += (M.ret_demand - M.ret_build) * fminf(1.0f, (float)dt / RET_BUILD_S);
    if (M.ret_build > 1e-6f && thr < 0.05f && M.gear > 0) {
        int gap = M.state != 0;
        if (gap && !M.ret_gap) { M.ret_gap = 1; M.ret_gap_t0 = t_now; }
        if (!gap && M.ret_gap) { M.ret_gap = 0; M.ret_gap_t1 = t_now; }
        float env = 1.0f;
        if (M.ret_gap) env = fmaxf(0.0f, 1.0f - (float)(t_now - M.ret_gap_t0) / RET_FALL_S);
        else if (M.ret_gap_t1 > 0.0 && t_now - M.ret_gap_t1 < RET_RISE_S) env = (float)(t_now - M.ret_gap_t1) / RET_RISE_S;
        float fade = fminf(fmaxf((M.v * 3.6f - RET_FADE_LO) / (RET_FADE_HI - RET_FADE_LO), 0.0f), 1.0f);
        float k_m = 18000.0f / M.mass;   // (M.mass in kg: the measured steps are for 18 t)
        float full = M.ret_build * RET18_MAX * k_m, fl = fminf(RET18_FLOOR * k_m, full);
        F -= (fl + (full - fl) * env) * fade * M.mass;
    }
    float s = sinf(atanf(M.grade_pct / 100));
    // The service brake. In sim_pcc the request M.brake is the game's +0x17c (act_request), which gives 10 m/s2 per unit at
    // 30 t (capped at that 30 t force, i.e. scaled by 1 / max(1, m / 30 t)) and is slewed at +-5/s like the game; the AEB
    // (brake_unseen) and the game's smart-overspeed service brake (0.15) go on top unslewed. The other sims (run.sh, run_scores)
    // use M.brake as the driver's pedal, which keeps the old 4 m/s2, so their braking scenarios do not change.
#ifdef SIM_PCC
    M.brake_applied += fmaxf(-5.0f * (float)dt, fminf(5.0f * (float)dt, M.brake - M.brake_applied));
    float Fbrake = (M.brake_applied + M.brake_unseen + M.smart_brake) * 10.0f * fminf(M.mass, 30000.0f);
#else
    float Fbrake = (M.brake + M.brake_unseen) * M.mass * 4.0f;
#endif
    float Fres = M.mass * 9.81f * (s + 0.006f) + 0.5f * 1.2f * CDA * M.v * M.v + Fbrake + g_plant_res;
    // (the telemetry acceleration is already a 0.5 s first-order lag of the plant's, as in the game: telemetry_frame)
    if ((EN.on || EN.on60) && M.gear > 0 && !M.state) {
        engine_info e; gb_engine(0, &e); float r = rpm_now();
        double kj = TORQUE * curve_factor(&e, r < idle ? idle : r) * thr * r * 0.10472f * dt / 1000;
        if (EN.on) { EN.kj += kj; EN.rs += r * dt; EN.rt += dt; EN.pk = fmax(EN.pk, r); }
        if (EN.on60) EN.kj60 += kj;
    }
    if (EN.on60) { EN.t60 += dt; if (EN.t60 >= 60) EN.on60 = 0; }
    if (M.state) Fres += g_plant_shift_drag * M.mass;
    M.v += (F - Fres) / M.mass * dt;
    if (M.v != M.v) M.v = 0;
    if (M.v < 0) M.v = 0;
    M.s += M.v * dt;
    // The fuel the plant burns, as the game does: a x omega x (T_net + T_f), T_net = torque x curve x plant throttle, T_f = g_sw_tf x TORQUE.
    // In gear with the throttle shut (< 0.02) nothing (overrun cut); out of gear / in a shift the idle: a x omega_idle x T_f. The liter
    // constant FL_C is arbitrary (only the shape and the scale ratio matter). SW_FUEL_SCALE / SW_FUEL_SCALE2 (after SW_FUEL_SWITCH_S s) are 'a'.
    // SW_FUEL=fit: the fitted model (fuel_fit.h) instead: fuel ~ torque curve x g(thr) x omega with a concave g, idle only near idle rpm.
    {
        float scale = (g_fuel_scale2 > 0 && t_now >= g_fuel_switch_s) ? g_fuel_scale2 : g_fuel_scale;
        if (g_fuel_fit) {
            FuelEngine fe;
            fe.tpk = TORQUE; fe.idle_rpm = idle; fe.n = NCURVE; fe.curve = CURVE; fe.a = fuel_fit_a(TORQUE);
            const double x = fuel_rate_lph(fe, rpm_now(), thr, M.gear >= 1 && !M.state, scale) * (float)dt / 3600.0f; g_fuel_raw -= x; g_fuel_used += x;
        } else {
            float tf = g_sw_tf * TORQUE;
            if (M.gear < 1 || M.state) {
                { const double x = scale * FL_C * idle * 0.10472f * tf * dt; g_fuel_raw -= x; g_fuel_used += x; }
            } else {
                engine_info e; gb_engine(0, &e);
                float r = rpm_now();
                float f = r > LIMIT ? 0 : curve_factor(&e, r < idle ? idle : r);
                if (thr >= 0.02f) { const double x = scale * FL_C * r * 0.10472f * (TORQUE * f * plant_thr(thr) + tf) * dt; g_fuel_raw -= x; g_fuel_used += x; }
            }
        }
        if (g_fuel_raw < 0) g_fuel_raw = 0;
        T.fuel = (float)(round(g_fuel_raw * 1000.0) / 1000.0);   // the 1 mL resolution the telemetry reports (the tank sloshes on top; the learner averages it)
    }
    if (g_row_extra && M.gear > 0 && !M.state && M.grade_pct <= 0.2f) { g_row_flat_s += dt; if (M.gear == NG) g_row_flat_top_s += dt; }
    if (g_row_extra && M.gear > 0 && !M.state) {
        const bool climbing = M.grade_pct >= 0.5f;
        if (climbing && !g_row_in_climb && g_row_climbs < 255) g_row_climbs++;
        g_row_in_climb = climbing;
        const float fc = fmaxf(M.cc, M.pedal);
        if (fc > 0.05f && vehicle_force_at(NG, M.v, 1.0f) >= 1.05f * vehicle_resist(M.v)) { g_row_hold_s += dt; if (M.gear == NG) g_row_hold12_s += dt; }
        if (M.gear == NG && fc >= 0.95f) { const float p1 = bands_now()->p1, r = rpm_now(); if (r < p1 + 50.0f) g_row_lug_s += dt; if (r < p1) g_row_lug1_s += dt; }
    }
    if (stats) { rpm_sum += rpm_now() * dt; rpm_t += dt; }
    if (M.state) g_sh_t += dt; else if (M.gear > 0) { g_rpm_s += rpm_now() * dt; g_rpm_t += dt; }
    if (M.gear > 0 && !M.state && M.v * 3.6f < 50) { rpm_all_sum += rpm_now() * dt; rpm_all_t += dt; }
    if (g_tick_hook) g_tick_hook(dt);
}

struct setup { float mass, v0; int g0; double secs; float cvar = 3, shift_time = 0.5f, mass_seen = 0; int eb_toggle = -1; float eb_level = -1;
               float (*road)(double) = NULL; int route = 0; double junction = 0; float pedal0 = 0; double settle = 0; };   // pedal0: the pedal from the first tick on (the module's demand starts from it)
static int rolls; static double roll_time, min_kmh, t_30, t_50, t_80, d_80;
static int short_rolls; static double roll_t0;   // rolls (neutral at speed) that ended within 5 s of going in (the hunt metric's window): a roll for nothing, two shifts for a coast the pedal ended (B3, B3d)

// [trueshift] lookahead is 0 by default since WP-0911-G1 (the box is good on its own; the route is a layer on top). A row that hands TrueShift a route (setup.route) is a row about that
// layer (the climb ahead, the top, the roll the route confirms, the junction): with g_route_rows it runs with the key on, as every such row did while the default was 1. The rows of
// G1 (the key itself, on and off) and every sim that does not set g_route_rows run with the key as it stands.
static int g_route_rows;
static float *lookahead_val() { for (int i = 0; i < m_trueshift.n_params; i++) if (!strcmp(m_trueshift.params[i].key, "lookahead")) return m_trueshift.params[i].val; return NULL; }
static void run_row(const char *name, setup su, void (*ctl)(double));
static void run(const char *name, setup su, void (*ctl)(double))
{
    float *la = lookahead_val(); const float was = la ? *la : 0.0f;
    if (la && g_route_rows && su.route) *la = 1.0f;
    run_row(name, su, ctl);
    if (la) *la = was;
}
static void run_row(const char *name, setup su, void (*ctl)(double))
{
    if (!g_quiet) printf("\n=== %s ===\n", name);
    memset(&M, 0, sizeof M);
    if (g_driver_feed) { driver_reset(); if (g_driver_inject) driver_parse(g_driver_inject); if (g_driver_pre) g_driver_pre(); }
    g_game_lim = 1.0f;
    for (int o = 0; o < ACT_N; o++) g_act[o] = act_wish{0, 0, 1, 1, 1};
    M.mass = su.mass; M.gear = M.target = su.g0; M.cvar = su.cvar; M.shift_time = su.shift_time;
    M.eb_level = su.eb_level; M.eb_toggle = su.eb_toggle; M.mass_seen = su.mass_seen;
    M.road = su.road; M.route_on = su.route; M.junction_s = su.junction; M.pedal = su.pedal0;
    if (su.road) route_build(su.road);   // the height table once per route (the road is fixed for the run)
    g_sim_parked = 1;
    M.v = 0; tick(1 / 60.0, 0);  // register the gear at standstill (no hint)
    for (int i = 0; i < (int)(su.settle * 60); i++) tick(1 / 60.0, 0);   // (settle: standing for a while first, so the plugin has read the mode before the run starts; it reads it a few times a second)
    g_sim_parked = 0;
    M.state = 0; M.gear = M.target = su.g0; M.force = 0;
    M.v = su.v0 / 3.6f; tick(1 / 60.0, 0);
    M.state = 0; M.gear = M.target = su.g0; M.force = 0;
    M.v = su.v0 / 3.6f; shifts = ups = hunts = hunts_drv = last_dir = 0; up_rpm_sum = 0; up_n = 0; rpm_sum = rpm_t = 0;
    g_foot_ref = sim_demand(); g_foot_dev = 0; g_foot_ref_pend = 0; g_foot_dev_pend = 0;
    M.s = 0; rolls = short_rolls = 0; roll_t0 = 0; engage_downs = 0; g_eng_t = 0; roll_time = 0; min_kmh = 1e9; t_30 = t_50 = t_80 = d_80 = -1;
    up_rpm_min = skip_rpm_min = skip_pull_min = 1e9; rpm_all_sum = rpm_all_t = 0; skips_up = hunts_ts = shifts_ts = 0;
    g_sh_t = g_rpm_s = g_rpm_t = 0; g_dskips = 0;
    int f0 = n_failed;
    double t0 = t_now, next_print = t0;
    g_row_gap_min = 1e9; g_row_fuel0 = g_fuel_used; g_row_t0 = t0; g_row_flat_s = g_row_flat_top_s = 0;
    g_row_hold_s = g_row_hold12_s = g_row_lug_s = g_row_lug1_s = 0; g_row_cl_gap = 1e9; g_row_climbs = g_row_in_climb = 0; g_row_cl_id = -1; memset(g_row_cl_n, 0, sizeof g_row_cl_n);
    while (t_now - t0 < su.secs) {
        ctl(t_now - t0);
        int was0 = M.gear == 0;
        tick(1 / 60.0, t_now - t0 > su.secs / 2);
        if (M.gear == 0 && M.v > 1) { roll_time += 1 / 60.0; if (!was0) { rolls++; roll_t0 = t_now; } }
        if (was0 && M.gear != 0 && roll_t0 > 0 && t_now - roll_t0 < 5.0) short_rolls++;   // (gear 0 was entered at roll_t0 with M.v > 1; the in-gear start of a run never sets it)
        if (was0 && M.gear != 0) roll_t0 = 0;
        if (t_now - t0 > 3 && M.v * 3.6 < min_kmh) min_kmh = M.v * 3.6;
        if (t_30 < 0 && M.v * 3.6f >= 30) t_30 = t_now - t0;
        if (t_50 < 0 && M.v * 3.6f >= 50) t_50 = t_now - t0;
        if (t_80 < 0 && M.v * 3.6f >= 80) { t_80 = t_now - t0; d_80 = M.s; }
        if (t_now >= next_print) { next_print += 10;
            if (!g_quiet) printf("        t=%5.1f  %5.1f km/h  gear %2d  rpm %4.0f  pedal %.2f cc %.2f grade %.1f%% %c\n", t_now - t0,
                   M.v * 3.6f, M.gear, rpm_now(), M.pedal, M.cc, M.grade_pct, letter); }
    }
    if (g_quiet) return;
    printf("  -> %d shifts (%d up, avg up-landing %.0f rpm), %d hunting pairs (%d more reversals after the driver moved the foot), %d no-pull lessons, end %.1f km/h gear %d, "
           "mean rpm 2nd half %.0f, min %.0f km/h, %d rolls %.0f s, %.0f m\n", shifts, ups, up_n ? up_rpm_sum / up_n : 0,
           hunts, hunts_drv, n_failed - f0, M.v * 3.6f, M.gear, rpm_t > 0 ? rpm_sum / rpm_t : 0, min_kmh, rolls, roll_time, M.s);
    char gapb[24];
    if (g_row_extra) printf("     fuel %.3f L (the plant's liters), %.2f L / 100 km over %.0f m; shortest time between two shifts %s; top gear on the flat and downhill %.0f %% of %.0f s\n", g_fuel_used - g_row_fuel0,
                            M.s > 1 ? (g_fuel_used - g_row_fuel0) / M.s * 1e5 : 0.0, M.s, shifts > 1 ? (snprintf(gapb, sizeof gapb, "%.1f s", g_row_gap_min), gapb) : "none", g_row_flat_s > 0 ? 100 * g_row_flat_top_s / g_row_flat_s : 0.0, g_row_flat_s);
    if (g_row_extra) { char yb[400]; row_yard(yb, sizeof yb); printf("     %s\n", yb); }
    if (getenv("ACCEL")) printf("  accel: 30 km/h at %.1f s, 50 at %.1f s, 80 at %.1f s (%.0f m)\n", t_30, t_50, t_80, d_80);
}

// T6C_TR=a,b,p: while a row with g_row_extra runs, between a and b seconds of the row, every p seconds and at every shift, the state and the layers' last decision with the terms of every
// candidate (tools/sim: how to read why a gear stays out; the terms are seconds, lowest wins).
static void t6c_trace(double)
{
    static const char *e = getenv("T6C_TR");
    if (!e || !g_row_extra) return;
    static double a, b, p; static int init;
    if (!init) { init = 1; a = 0; b = 1e9; p = 1; sscanf(e, "%lf,%lf,%lf", &a, &b, &p); if (p <= 0) p = 1; }
    static int g_prev; static double next, row = -1;
    if (g_row_t0 != row) { row = g_row_t0; next = 0; }
    const double t = t_now - g_row_t0;
    const bool shifted = M.gear != g_prev; g_prev = M.gear;
    if (t < a || t > b || (!shifted && t < next)) return;
    if (t >= next) next = t + p;
    double at = -1; const gc_out *go = ts_last_choice(&at);
    { static double hdr = -1; if (hdr != g_row_t0) { hdr = g_row_t0; const eng_bands *bb = bands_now(); printf("BANDS idle %.0f floor %.0f p1 %.0f p2 %.0f p3 %.0f cap %.0f limit %.0f (ok %d); power range %.0f..%.0f; mass %.0f kg, eta %.2f\n", bb->idle, bb->floor, bb->p1, bb->p2, bb->p3, bb->cap, bb->limit, bb->ok, V.eng.range[R_POWER][0], V.eng.range[R_POWER][1], V.mass, V.eta); } }
    printf("TR %7.2f v %5.1f gear %2d rpm %4.0f cc %.2f grade %+.1f accel %+.2f%s", t, M.v * 3.6f, M.gear, rpm_now(), M.cc, M.grade_pct, T.accel, shifted ? "  <- shift in" : "");
    if (!go || at < t_now - 0.1) { printf("   (no choice this tick)\n"); return; }
    printf("   choice->%d why %d lvl %.0f hold %.2f need %.2f bscale %.2f urg %.2f\n", go->best, go->why, go->level_n, go->hold_share, go->need, go->band_scale, go->urgency);
    for (int g = 1; g <= go->n; g++) {
        const gc_term &x = go->t[g];
        if (!x.allowed || !x.in_win) continue;
        printf("      g%-2d%s %4.0f rpm %-7s q%d c%d F %6.0f  sh %5.2f ac %5.2f bd %5.2f ex %5.2f fu %5.2f st %4.2f = %6.2f  (fuel_l %.4f, %d shifts)\n", g, g == M.gear ? "*" : " ", x.rpm, gc_band_name(x.band), x.qualifies, x.candidate, x.force,
               x.shiftc, x.accc, x.bandc, x.expc, x.fuelc, x.stay, x.score, x.fuel_l, x.shifts);
    }
}

// A driver holding a speed with the pedal (P controller, a bit lazy).
static void hold(float kmh, float gain = 0.25f)
{
    float e = kmh / 3.6f - M.v;
    M.pedal = fminf(1, fmaxf(0, M.pedal + (e * gain - (M.pedal - 0.45f) * 0.0f) * (1 / 60.0f) * 4));
    if (e < -1.5f) M.pedal = 0;
}
// The game's cruise control: bang-bang (full below, zero above, half at target).
// The game's cruise holding a speed through ITS throttle (M.cc, an integrator like hold()), the driver's pedal off: T.cruise is on and T.in_thr is 0, as in the game.
static void hold_cc(float kmh, double t, float gain = 0.25f)
{
    static float u;
    if (t <= 0) u = M.pedal;   // (the throttle the setup starts with, as hold() has it)
    float e = kmh / 3.6f - M.v;
    M.cruise_kmh = kmh; M.pedal = 0;
    u = fminf(1, fmaxf(0, u + e * gain * (1 / 60.0f) * 4));
    if (e < -1.5f) u = 0;
    M.cc = u;
}
static void cruise(float kmh)
{
    M.cruise_kmh = kmh; M.pedal = 0;
    float e = (kmh + g_roll_boost_kmh) / 3.6f - M.v;   // the cruise module raises its target by the roll boost TrueShift asks for
    M.cc = e > 0.3f ? 1.0f : e < -0.3f ? 0.0f : 0.5f;
    if (g_cruise_smooth) M.cc = fminf(1, fmaxf(0, 0.45f + e * 0.3f));  // a smooth controller
    if (g_rolling) M.cc = 0;                                            // the cruise module gives no throttle in neutral
}

static void c_full(double) { M.pedal = 1; }
static void c_p70(double) { M.pedal = 0.7f; }
static void c_p20(double) { M.pedal = 0.2f; }
static void c_p30(double) { M.pedal = 0.3f; }
static void c_p40(double) { M.pedal = 0.4f; }
static void c_p50(double) { M.pedal = 0.5f; }
static float g_ped; static void c_ped(double t) { M.pedal = g_ped >= 0 ? g_ped : 0.42f + 0.12f * sinf((float)t * 6.2832f / 5); }   // < 0: a wavering foot, 0.30-0.54 every 5 s
// Adaptive cruise set to 80 behind a truck doing 50: bursts, then long gaps.
static void c_acc(double t) { M.cruise_kmh = 80; M.pedal = 0; float e = 50 / 3.6f - M.v; M.cc = e > 0.3f ? 1.0f : e < -0.3f ? 0.0f : 0.3f; }
static void c_light(double) { M.pedal = 0.4f; }
static void c_solo80(double) { hold(80); }
static void c_cruise85(double) { M.grade_pct = 1.0f; hold(85); }
static void c_beamng(double t) { M.grade_pct = t < 30 ? 0 : t < 90 ? 3.5f : 0; hold(80); }
static void c_hills(double t) { M.grade_pct = 3 * sinf((float)t / 60 * 6.283f); hold(80); }
static void c_climb(double t) { M.grade_pct = t < 10 ? 0 : 6; M.pedal = 1; }
static void c_kick(double t) { M.pedal = t < 10 ? 0.35f : 1.0f; }
static void c_brake(double) { M.pedal = 0; M.brake = 0.25f; }
static void down70() { M.grade_pct = -5; M.pedal = 0; float e = M.v - 70 / 3.6f; M.brake = fminf(0.5f, fmaxf(0, e * 0.1f)); }
static void c_eb(double) { down70(); M.ebrake = 1; }
static void c_ret1(double) { down70(); M.ret = 1; }
static void c_ret5(double) { down70(); M.ret = 5; }
static void c_cc(double) { M.grade_pct = 1.0f; cruise(80); }
static void c_hint(double t)
{
    M.pedal = 0.6f;
    static int done;
    if (t < 0.1) done = 0;
    if (t > 15 && !done && !M.state) { done = 1; M.target = M.gear - 1; M.state = 1; M.shift_end = t_now + 0.6; printf("  (hint down)\n"); }
}
static void c_lowspeed(double t)
{
    if (t < 25) { M.pedal = 0.5f; M.brake = 0; }
    else if (t < 40) { M.pedal = 0; M.brake = 0.15f; }
    else if (t < 50) { M.pedal = 0.3f; M.brake = 0; }   // creep back up through the handover band
    else { M.pedal = 0; M.brake = 0.2f; }
}
static void c_m_full(double t) { M.pedal = 1; }
// roads (grade % along the distance)
static float r_climb(double s) { return s < 1500 ? 0 : s < 2500 ? 4.0f : 0; }
static float r_crest(double s) { return s < 700 ? 5.0f : s < 1700 ? -1.5f : 0; }
static float r_down(double s) { return -0.8f; }
static float r_climb10(double s) { return s < 300 ? 0 : s < 600 ? 10.5f : 0; }   // a steep 300 m climb after a flat (24-10, 545 s: 10-11 % at 43 km/h)
static float r_climb6(double s) { return s < 300 ? 0 : s < 700 ? 6.0f : 0; }
static float r_climb3(double s) { return s < 300 ? 0 : s < 900 ? 3.0f : 0; }   // a long 3 % climb   // a moderate 400 m climb: the gear can just hold it
static float r_hills(double s) { return 1.5f * sinf((float)(s / 1500 * 6.2832)); }
static float r_flat(double s) { return 0; }
static float r_hills25(double s) { return 2.5f * sinf((float)(s / 1000 * 6.2832)); }
static float r_short3(double s) { return s < 1500 ? 0 : s < 1900 ? 3.0f : 0; }   // 400 m at 3 %: a top within reach
static float r_short5(double s) { return s < 1500 ? 0 : s < 1800 ? 5.0f : 0; }   // 300 m at 5 %
static float r_hump3(double s) { return s < 1500 ? 0 : s < 1650 ? 3.0f : 0; }   // 150 m at 3 %
static float r_hump15(double s) { return s < 1500 ? 0 : s < 1900 ? 1.5f : 0; }  // 400 m at 1.5 %
static float r_mid3(double s) { return s < 1500 ? 0 : s < 2300 ? 3.0f : 0; }    // 800 m at 3 %
static float r_long4(double s) { return s < 1500 ? 0 : s < 3500 ? 4.0f : 0; }    // 2 km at 4 %: must downshift
// Ride 3 (11-10, 395-402 s): a 5-6 % climb with a short 4.1 % dip; 30.4 t on cruise 73 went 8 -> 9 in the dip and back after 1.6 s.
static float g_dipg = 5.8f;   // the climb's grade (H1 runs it at a few, the sim truck is not ride 3's)
static float r_dip(double s) { return s < 300 ? 1.0f : s < 900 ? g_dipg : s < 960 ? g_dipg - 1.7f : s < 1500 ? g_dipg - 0.2f : s < 1800 ? 2.0f : 0; }
static void c_cci73(double) { M.cruise_kmh = 73; M.pedal = 0; float e = 73 / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.5f + e * 1.0f)); }
static void c_cci80(double) { M.cruise_kmh = 80; M.pedal = 0; float e = 80 / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.5f + e * 1.0f)); }
static float param_def(const char *n)
{
    for (int i = 0; i < m_trueshift.n_params; i++) if (!strcmp(m_trueshift.params[i].key, n)) return m_trueshift.params[i].def;
    return 0;
}
// ---- the owner's yardstick as a verdict (Damien 10-11, WP-1011-T6C): the game's cruise or the plugin's on hills 12th cannot quite climb. The numbers come from values the box already has: the anti-hunt
// dwell (the longer of dwell_up_down_s and dwell_down_up_s), the plateau start P1 of the engine's bands and the int-1011 run of the same row.
//   FAIL  12 sits at full throttle under P1 + 50 rpm for more than one dwell per climb (lugging where it cannot cope: the box held a gear that sags on the plateau's edge),
//   FAIL  a climb gets more than 2 shifts, or two shifts on one climb are closer than the dwell (a gear for the climb, and the way back: not a hunt),
//   FAIL  more diesel than int-1011 burnt on the row, or more than 1 % short of its distance (the twin's route has a fixed length: more than 1 % longer in time),
//   (11 while the cruise coasts is nobody's failure: it is not counted anywhere here)
//   GAP   the 12 share on the holds (the cruise on the throttle and 12 carrying the road with a 5 % margin) is printed, soft: an open gap of the gear choice's fuel term (zero under a controller) and
//         taste (none on a descent), not a failure of this change.
struct yard_t { int shifts, hunts, climbs, cl_shifts, cl_max; double cl_gap, hold, hold12, lug1, lug, fuel, dist, secs; };
static yard_t yard_snap()   // the row that just ran, from the row's own counters
{
    yard_t y = {};
    y.shifts = shifts; y.hunts = hunts; y.climbs = g_row_climbs; y.cl_gap = g_row_cl_gap;
    for (int i = 1; i <= g_row_climbs && i < 256; i++) { y.cl_shifts += g_row_cl_n[i]; if (g_row_cl_n[i] > y.cl_max) y.cl_max = g_row_cl_n[i]; }
    y.hold = g_row_hold_s; y.hold12 = g_row_hold12_s; y.lug1 = g_row_lug1_s; y.lug = g_row_lug_s;
    y.fuel = g_fuel_used - g_row_fuel0; y.dist = M.s; y.secs = t_now - g_row_t0;
    return y;
}
// ref: int-1011's run of the same row (799bae4); fixed_route: the row has a fixed distance (the PCC twin), so the time stands for it. Returns 1 for a pass, prints the verdict and the soft GAP line.
#define B4_BASE_FUEL 32.783   // B4 on the base 43f2524 (the plant's liters, printed 32.782) and its distance: the reference of 'diesel not up, distance not down'
#define B4_BASE_DIST 5472
static const char *g_yard_ref = "int-1011";   // the name of the run the reference numbers are from (B4: the K2 base)
static int yard_verdict(const char *name, const yard_t &y, const yard_t &ref, bool fixed_route)
{
    const double dwell = fmax(param_def("dwell_up_down_s"), param_def("dwell_down_up_s")), lug_max = dwell * y.climbs;
    char why[400] = ""; size_t w = 0;
    auto bad = [&](const char *fmt, double a, double b) { w += (size_t)snprintf(why + w, sizeof why - w, "%s", w ? "; " : ""); w += (size_t)snprintf(why + w, sizeof why - w, fmt, a, b); };
    if (y.lug > lug_max) bad("12 at full throttle under P1 + 50 rpm for %.1f s (expect <= %.1f s, one dwell a climb)", y.lug, lug_max);
    if (y.cl_max > 2) bad("%.0f shifts on one climb (expect <= %.0f)", (double)y.cl_max, 2.0);
    if (y.cl_gap < dwell) bad("%.1f s between two shifts on a climb (expect >= %.1f s, the dwell)", y.cl_gap, dwell);
    if (ref.shifts && y.shifts > ref.shifts) bad("%.0f shifts (expect <= %.0f, not up against the reference)", (double)y.shifts, (double)ref.shifts);
    // diesel per metre against the reference, with the same 1 % the distance and time bars allow (0.7 release: the bar compared liters at 3 digits while the distance differed)
    if (y.dist > 0 && ref.dist > 0 ? y.fuel / y.dist > 1.01 * ref.fuel / ref.dist : y.fuel > ref.fuel)
        bad("%.3f L/km (expect <= %.3f L/km, the reference + 1 %%)", 1000 * y.fuel / fmax(y.dist, 1.0), 1010 * ref.fuel / fmax(ref.dist, 1.0));
    if (fixed_route ? y.secs > 1.01 * ref.secs : y.dist < 0.99 * ref.dist)
        bad(fixed_route ? "%.0f s for the route (expect <= %.0f s, 1 %% over the reference)" : "%.0f m (expect >= %.0f m, 1 %% under the reference)", fixed_route ? y.secs : y.dist, fixed_route ? 1.01 * ref.secs : 0.99 * ref.dist);
    char gap[24]; if (y.cl_gap < 1e8) snprintf(gap, sizeof gap, "%.1f s", y.cl_gap); else snprintf(gap, sizeof gap, "none");
    if (!w) printf("  %s  ok: %d shifts, %d on the %d climbs (most on one %d, shortest gap %s, dwell %.1f s), 12 at full throttle under P1 + 50 rpm %.1f s (<= %.1f s), %.2f L (%s %.2f), %s\n", name, y.shifts, y.cl_shifts, y.climbs,
                   y.cl_max, gap, dwell, y.lug, lug_max, y.fuel, g_yard_ref, ref.fuel, fixed_route ? "the route in the time of the reference +1 %" : "the distance of the reference -1 %");
    else printf("  %s  FAIL: %s\n", name, why);
    printf("  %s  %s: 12 on the holds %.0f %% of %.0f s (the cruise on the throttle, 12 carrying the road with a 5 %% margin; 11 while it coasts is not counted)\n", name, y.hold12 + 1e-6 >= y.hold ? "ok" : "GAP", y.hold > 0 ? 100 * y.hold12 / y.hold : 100.0, y.hold);
    return !w;
}
static void set_param(const char *n, float v)
{
    if (!strncmp(n, "pd_", 3)) {   // pd_<key>: a [pedal] key (a sweep config), applied at once
        for (int i = 0; i < m_pedal.n_params; i++) if (!strcmp(m_pedal.params[i].key, n + 3)) { *m_pedal.params[i].val = v; if (m_pedal.applied) m_pedal.applied(n + 3, 0); return; }
        fprintf(stderr, "PMP: no pedal key %s\n", n + 3);
        if (getenv("SW_STRICT")) exit(1);
        return;
    }
    int hit = 0;   // (the car automatic has the same keys under [carshift]: a scenario's setting reaches whichever brain the vehicle class runs)
    for (int i = 0; i < m_carshift.n_params; i++) if (!strcmp(m_carshift.params[i].key, n)) { *m_carshift.params[i].val = v; hit = 1; }
    for (int i = 0; i < m_trueshift.n_params; i++) if (!strcmp(m_trueshift.params[i].key, n)) { *m_trueshift.params[i].val = v; return; }
    if (hit) return;
    if (getenv("SW_STRICT")) { fprintf(stderr, "SW_STRICT: no trueshift key %s\n", n); exit(1); }
}
static void c_hold80(double) { hold(80); }
static void c_full_road(double) { M.pedal = 1; }
static void c_half(double) { M.pedal = 0.5f; }
// lift off at t=10 towards a junction 60 m ahead, full pedal again at t=14 (ride 24-10, 1830-1834 s: 14>16 while rolling out, then back)
static void c_lift_jn(double t) { if (t < 10) hold(70); else if (t < 14) M.pedal = 0; else M.pedal = 1.0f; }
static void c_lift_jn_bug(double t) { M.thr_floor = 0.08f; c_lift_jn(t); }
static void c_lift_jn_free(double t) { if (t < 10) hold(70); else M.pedal = 0; }
static void c_release(double t) { if (t < 10 || t > 50) hold(80); else M.pedal = 0; }
// the yard (24-10, 3853-3877 s): 13 km/h, a short brake, then gentle pedal through 5-25 km/h
static void c_yard(double t) { if (t < 2) { M.pedal = 0.2f; M.brake = 0; } else if (t < 3.5) { M.pedal = 0; M.brake = 0.15f; } else { M.pedal = 0.3f; M.brake = 0; } }
static void c_pull03(double t) { M.pedal = t < 3 ? 0 : 0.3f; M.brake = 0; }
static void c_cc46(double) { cruise(46); }   // an ACC target behind a slow lead (24-10: lead 40-46 km/h)
static void c_cc52(double) { cruise(52); }
static void c_cc63(double) { cruise(63); }
static void c_cc80(double) { cruise(80); }
static void c_car_light(double) { M.pedal = 0.35f; }
static void c_car_full(double) { M.pedal = 1; }
static void c_car_100(double) { hold(100, 0.05f); }
static void c_m_climb(double t) { M.pedal = 1; M.grade_pct = t < 5 ? 0 : 7; }
static void c_m_stop(double) { M.pedal = 0; M.brake = 0.2f; }
// The game ignores the hint in M (ride 3): only the button is pressed, TrueShift must shift itself
static void c_m_btn(double t)
{
    M.pedal = 0.5f;
    sim_btn = (t > 8 && fmod(t, 4) < 0.15 && t < 20) ? 14 : 0;
}
static void c_m_hint(double t)
{
    M.pedal = 0.5f;
    static int done;
    if (t < 0.1) done = 0;
    if (t > 8 && done < 3 && !M.state && fmod(t, 4) < 0.05) { done++; M.target = M.gear + 1; M.state = 1; M.shift_end = t_now + 0.5; printf("  (hint up)\n"); }
}

// the rev-limiter bug of the 24-10 ride (124-130 s): full pedal in M against the limiter, three + presses; the chooser runs every tick out of the window
static void c_m_limiter(double t)
{
    M.pedal = 1.0f;
    static int done;
    if (t < 0.1) done = 0;
    if (t > 8 && done < 3 && !M.state && fmod(t, 3) < 0.06) { done++; M.hint = 1; printf("  (hint up at %.1f s, rpm %.0f)\n", t, rpm_now()); }
}

// X4: full pedal up a 6 % climb, at t = 20 s a teleport (HARD reset) onto a 2 % climb at the same speed. The grade peak decide_gear
// reckons the next upshift with belongs to the road we left: the upshift must come soon on the 2 % road, not ~27 s later when a 6 % peak
// has faded (it used to be a function-local static that lived through the reset).
static int g_tp_done, g_tp_ups, g_tp_downs, g_tp_snap, g_tp_downs1; static double g_tp_up_at;   // (g_tp_downs1: the downshifts counted from 1.5 s after the teleport)
static float g_tp_pedal = 1, g_tp_g1 = 6, g_tp_g2 = 2;
static int g_tp_f0, g_tp_i0;   // the no-pull lessons and incidents at the start of the run: X4 counts the whole run (the 6 % climb before the teleport too)
static void c_teleport(double t)
{
    if (t <= 0) { g_tp_f0 = n_failed; g_tp_i0 = g_n_incidents; }
    M.pedal = g_tp_pedal; M.grade_pct = t < 20 ? g_tp_g1 : g_tp_g2;
    if (t >= 20 && !g_tp_done) { g_tp_done = 1; g_tp_ups = ups; g_tp_downs = shifts - ups; sim_reset(RESET_HARD, "sim: teleport onto a 2 % climb"); if (!g_quiet) printf("  (teleport: 6 %% -> 2 %%)\n"); }
    if (g_tp_done && !g_tp_snap && t >= 21.5) { g_tp_snap = 1; g_tp_downs1 = shifts - ups; }
    if (g_tp_done && g_tp_up_at < 0 && ups > g_tp_ups) g_tp_up_at = t;
}

static void run_m()
{
    run("M1 M mode, 40 t, full throttle from 50 in 8", {40000, 50, 8, 40, 0}, c_m_full);
    run("M2 M mode, 40 t, full throttle, 7% climb", {40000, 60, 10, 50, 0}, c_m_climb);
    run("M3 M mode, 40 t, braking to a stop from 60 in 10", {40000, 60, 10, 30, 0}, c_m_stop);
    run("M4 M mode, 40 t, 0.5 pedal, three hint-ups", {40000, 40, 7, 25, 0}, c_m_hint);
    run("M5 M mode, 40 t, 0.5 pedal, three hint-up presses the game ignores (ride 3)", {40000, 40, 7, 25, 0}, c_m_btn);
    run("M6 M mode, 40 t, FULL pedal against the rev limiter in gear 3, three hint-up presses (ride 24-10, 124-130 s)", {40000, 12, 3, 25, 0}, c_m_limiter);
    sim_btn = 0;
}

// ---- TC: the torque-converter plant and fleet (run: tools/sim/run_tc.sh). The converter model (the slip and the multiplication
// below the lockup, the lockup from the lock gear) is in the plant's rpm_now()/force above; here it is checked directly and the fleet
// is driven through launch, overtake and cruise scenarios against TrueShift (no hunting, lockup, shift points that follow the driveline).
static int g_tc_failed;
static void tc_check(bool ok, const char *fmt, ...)
{
    printf("  %-4s ", ok ? "ok" : "FAIL");
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); printf("\n");
    if (!ok) g_tc_failed = 1;
}
// Direct check of the plant's converter: from a standstill in gear 1 at full throttle the engine must sit at the stall speed (slip far
// above 1) and, once the driveline catches up past the lock gear / coupling point, the engine must follow the driveline (slip ~ 1).
static void tc_model_check(const char *name, void (*use)(), float mass, int g0, int lock_gear)
{
    sim_reset(RESET_HARD, "sim: tc model");
    use();
    memset(&M, 0, sizeof M);
    for (int o = 0; o < ACT_N; o++) g_act[o] = act_wish{0, 0, 1, 1, 1};
    g_game_lim = 1.0f;
    M.mass = mass; M.cvar = 3; M.shift_time = 0.5f; M.pedal = 1.0f; M.gear = M.target = g0;
    T.fwd_gears = NG; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    float slip_low = 0, slip_high = 1e9f; int low_n = 0, high_n = 0;
    for (int i = 0; i < 120 * 60 && M.v * 3.6f < 60; i++) {
        tick(1 / 60.0, 0);
        if (!M.state && M.gear > 0 && M.v > 0.5f) {
            float drv = M.v / RADIUS * RATIO[M.gear - 1] * DIFF * 9.5493f;
            float slip = rpm_now() / fmaxf(drv, 1.0f);
            if (M.v < 3) { slip_low += slip; low_n++; } else if (M.v > 10) { slip_high = fminf(slip_high, slip); high_n++; }
        }
    }
    float low = low_n ? slip_low / low_n : 0, hi = high_n ? slip_high : 0;
    tc_check(low > 1.5f, "%s: below the lockup the engine slips (mean slip %.2f over the driveline, expect > 1.5)", name, low);
    tc_check(high_n > 0 && hi < 1.5f, "%s: above ~36 km/h the converter couples (min slip %.2f, expect < 1.5)", name, hi);
    (void)lock_gear;
}
// Fleet scenarios: launch (the converter slips then locks), an overtake and a cruise, none of them hunting or leaving no-pull lessons.
static float g_tc_hold;
static void c_tc_hold(double t) { if (t <= 0.02) M.pedal = 0.3f; hold(g_tc_hold); }   // start already holding: no eco-roll on the first tick of a fresh cruise
static void tc_fleet_run(const char *name, void (*use)(), float mass, int g0, float hold_kmh)
{
    sim_reset(RESET_HARD, "sim: tc fleet truck");
    use();
    int f0 = n_failed, i0 = g_n_incidents;
    run("launch", {mass, 8, g0, 60}, c_full);
    tc_check(hunts == 0 && n_failed == f0 && g_n_incidents == i0, "%s: launch 8->80: %d shifts (%d up), %d hunting pairs, %d no-pull lessons (expect no hunts, no lessons)",
             name, shifts, ups, hunts, n_failed - f0);
    f0 = n_failed; i0 = g_n_incidents;
    sim_reset(RESET_HARD, "sim: tc fleet truck"); use(); g_tc_hold = hold_kmh; run("cruise", {mass, hold_kmh, NG, 90}, c_tc_hold);
    tc_check(hunts == 0 && n_failed == f0 && g_n_incidents == i0, "%s: hold %.0f: %d shifts, %d hunting pairs (expect no hunts, no lessons)", name, hold_kmh, shifts, hunts);
}
// ---- CAR FEEL: the light-vehicle checks (the ATS Road Trip Ford cars). The owner's points:
// a calm drive shifts low and holds the top gear, a steady 65 mph cruise with a small press never skips more than 1-2
// gears down (the ride's 7 -> 3 was a cruising press), a hard drive revs high, Eco caps the revs, Sport calm behaves
// like Normal calm (just a little more eager), the 4-speed uses all its gears, and M hints are always taken.
static int rec_max_drop()   // the deepest downshift the recorder saw, in gears
{
    int m = 0;
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b < g_rec[i].a && g_rec[i].a - g_rec[i].b > m) m = g_rec[i].a - g_rec[i].b;
    return m;
}
static int rec_max_gear()   // the highest gear the recorder saw reached
{
    int m = 0;
    for (int i = 0; i < g_rec_n; i++) { if (g_rec[i].a > m) m = g_rec[i].a; if (g_rec[i].b > m) m = g_rec[i].b; }
    return m;
}
static int rec_upshifts()   // upshifts the recorder saw
{
    int u = 0;
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a) u++;
    return u;
}
static float mean_rpm() { return rpm_t > 0 ? (float)(rpm_sum / rpm_t) : 0; }
static float g_car_press;   // the pedal the cruise press reaches
static void c_car_cruise_press(double t)
{
    if (t < 8) hold(104.6f);                              // 65 mph, settle into the top gear
    else if (t < 12) M.pedal = 0.25f + (float)(t - 8) * 0.075f;   // a gentle press while cruising: 0.25 -> 0.55 over 4 s
    else M.pedal = g_car_press;
}
static void c_car_105(double t) { if (t <= 0.02) M.pedal = 0.3f; hold(104.6f); }
// M mode: press hints down then up, alternately, only where the target gear's rpm stays safe (the box must honor every
// safe hint and refuse only an over-rev or a stall). g_rpm(g) = the driveline rpm of a gear at the speed of now.
static float g_rpm(int g) { return M.v / RADIUS * RATIO[g - 1] * DIFF * 9.5493f; }
static int g_mh_press, g_mh_ok, g_mh_miss;
static void c_car_manual(double t)
{
    static int dir, waiting, want; static double press_t;
    if (t < 0.1) { dir = -1; waiting = 0; }
    hold(70);   // a steady 70 km/h: gears 3..10 are all safe, so every hint here must be honored
    if (t < 1.0) M.pedal = fmaxf(M.pedal, 0.3f);   // (the first second: the pedal on, so the plugin sees no coasting car before it has read the mode, M; else a start-up phase decides whether it rolls in neutral at once)
    if (waiting) {
        if (M.gear == want) { g_mh_ok++; waiting = 0; }
        else if (t - press_t > 2.0) { g_mh_miss++; waiting = 0; }
    } else if (t > 6 && M.state == 0 && M.gear > 2 && M.gear < NG - 1) {
        int d = dir, g = M.gear + d;
        if (g < 2) { g = 2; d = g - M.gear; }
        if (g > NG - 1) { g = NG - 1; d = g - M.gear; }
        if (g != M.gear && g_rpm(g) < 0.9f * LIMIT && g_rpm(g) > 900) {
            M.hint = d; want = g; waiting = 1; press_t = t; g_mh_press++; dir = -d;
        } else dir = -d;
    }
}
static void run_car_feel()
{
    printf("\n=== TC: car feel (the Ford cars) ===\n");
    // 1. steady 65 mph cruise, then a small press: no deep kickdown
    use_tc_ford10();
    g_rec_on = 1; g_rec_n = 0; g_car_press = 0.55f;
    run("65 mph cruise, then a small press", {1800, 104.6f, 10, 30}, c_car_cruise_press);
    tc_check(rec_max_drop() <= 2, "65 mph cruise + a small press: deepest downshift %d gears (expect <= 2, the ride's 7 -> 3 was 4)", rec_max_drop());
    g_rec_on = 0;
    // 2. a calm drive shifts low: hold 65 mph and stay in the top gear at a low rpm (Normal)
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("calm hold 65 mph (Normal)", {1800, 104.6f, 10, 60}, c_car_105);
    tc_check(M.gear == 10 && rpm_now() < 2000, "calm 65 mph: gear %d at %.0f rpm (expect the top gear under 2000 rpm)", M.gear, rpm_now());
    float calm_norm = rpm_now();
    // 3. Sport calm behaves like Normal calm, within a little (the top gear at a similar low rpm)
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("calm hold 65 mph (Sport)", {1800, 104.6f, 10, 60, 1.67f}, c_car_105);
    tc_check(M.gear == 10 && fabsf(rpm_now() - calm_norm) < 350, "Sport calm 65 mph: gear %d at %.0f rpm (Normal calm %.0f, expect within ~350 rpm)", M.gear, rpm_now(), calm_norm);
    // 4. a hard drive revs high: a full launch upshifts well up the rev range (Normal)
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("hard launch (Normal)", {1800, 8, 1, 40}, c_full);
    float mean_norm = mean_rpm();
    tc_check(mean_norm > 4000, "hard launch: mean rpm in the 2nd half %.0f (expect revs up, over ~4000)", mean_norm);
    // 5. Eco caps the revs for a firm pedal (0.7): the same launch stays well below Normal's; a FLOORED pedal is a kickdown in every mode and revs as high
    // (ride 31-10: the Eco floor shifted up at once at the calm points)
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("firm launch 0.7 (Normal)", {1800, 8, 1, 40}, c_p70);
    float mean_norm70 = mean_rpm();
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("firm launch 0.7 (Eco)", {1800, 8, 1, 40, 10.0f}, c_p70);
    float mean_eco = mean_rpm();
    tc_check(mean_eco < mean_norm70 - 400, "Eco firm launch (pedal 0.7): mean rpm %.0f (Normal %.0f, expect clearly lower: Eco caps the aggression)", mean_eco, mean_norm70);
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("hard launch (Eco)", {1800, 8, 1, 40, 10.0f}, c_full);
    float mean_eco_floor = mean_rpm();
    tc_check(mean_eco_floor > mean_norm - 600, "Eco floored launch: mean rpm %.0f (Normal %.0f, expect about the same: a floored pedal is a kickdown in any mode)", mean_eco_floor, mean_norm);
    // 6. Sport is a little more eager, never lower than Normal on a hard drive
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    run("hard launch (Sport)", {1800, 8, 1, 40, 1.67f}, c_full);
    float mean_sport = mean_rpm();
    tc_check(mean_sport >= mean_norm - 150, "Sport hard launch: mean rpm %.0f (Normal %.0f, expect at least as eager)", mean_sport, mean_norm);
    // 7. the 4-speed uses all four gears (the ride's Crown Victoria stayed in the top gear)
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford4();
    g_rec_on = 1; g_rec_n = 0;
    run("4-speed launch", {1800, 8, 1, 40}, c_full);
    g_rec_on = 0;
    tc_check(rec_max_gear() == 4 && rec_upshifts() >= 3, "4-speed launch: reached gear %d with %d upshifts (expect all four gears)", rec_max_gear(), rec_upshifts());
    // 8. the 3-speed (Mustang) uses all three gears
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford3();
    g_rec_on = 1; g_rec_n = 0;
    run("3-speed launch", {1800, 8, 1, 40}, c_full);
    g_rec_on = 0;
    tc_check(rec_max_gear() == 3 && rec_upshifts() >= 2, "3-speed launch: reached gear %d with %d upshifts (expect all three gears)", rec_max_gear(), rec_upshifts());
    // 9. M mode: the driver's hints are always taken, never reverted, never fought
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford10();
    int f0 = n_failed, i0 = g_n_incidents;
    g_mh_press = g_mh_ok = g_mh_miss = 0;
    { setup su{1800, 60, 6, 90, 0.0f}; su.settle = 1.0; run("M mode hints (10-speed)", su, c_car_manual); };
    tc_check(g_mh_press > 10 && g_mh_miss == 0, "M mode: %d/%d hints honored, %d missed (expect all honored)", g_mh_ok, g_mh_press, g_mh_miss);
    tc_check(n_failed == f0 && g_n_incidents == i0, "M mode: %d no-pull lessons, %d incidents (expect none)", n_failed - f0, g_n_incidents - i0);
    // 10. a steady Power hold does not hunt (the ride's 3-4-2-4 at 75-85 km/h)
    sim_reset(RESET_HARD, "sim: car feel"); use_tc_ford4b();
    f0 = n_failed; i0 = g_n_incidents; g_tc_hold = 80;
    run("Sport hold 80 (4-speed OD)", {1800, 80, 3, 90, 1.67f}, c_tc_hold);
    tc_check(hunts == 0 && n_failed == f0 && g_n_incidents == i0, "Sport hold 80: %d hunting pairs, %d no-pull lessons (expect none)", hunts, n_failed - f0);
}
// ---- the Crown Victoria (ATS, ride 31-10: 4R70W, rpm limit 6000, 2.0 t, idle 823), Power. At the floor a car holds each gear to high revs
// and kicks down when the pedal goes down; the ride shifted up at once at 1800-2400 rpm (a converter taken for a tire correction -> the
// model's rpm 18 % high all session; the power range's top lost to the idle's 5 rpm grid; the aggression and the demand lagging a stomp).
float ts_up_pt();
static void c_cv_kick(double t) { M.pedal = t < 4 ? 0.3f : 1.0f; }
static void c_cv_stomp(double)   // 0.42 to 29 km/h, a calm 0.2 around the upshift, the stomp at 32 km/h in 1st (0.12 s to the floor)
{
    static double t_st = -1;
    if (M.v * 3.6f < 0.5f) t_st = -1;
    if (t_st < 0 && M.v * 3.6f >= 32) t_st = t_now;
    if (t_st < 0) M.pedal = M.v * 3.6f < 29 ? 0.42f : 0.2f;
    else M.pedal = t_now - t_st < 0.12 ? 0.2f + (float)(t_now - t_st) / 0.12f * 0.8f : 1.0f;
}
static int rec_first_up_rpm()   // the rpm of the first upshift the recorder saw (0 = none)
{
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a) return (int)g_rec[i].rpm;
    return 0;
}
static int rec_min_up_rpm()     // the lowest rpm of any upshift the recorder saw (0 = none)
{
    int m = 0;
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a && (m == 0 || (int)g_rec[i].rpm < m)) m = (int)g_rec[i].rpm;
    return m;
}
static void run_crownvic()
{
    printf("\n=== TC: the Crown Victoria at the floor (Power) ===\n");
    const float choice0 = param_def("choice");
    for (int ch = 0; ch < 1; ch++) {   // the cascade: the car automatic has no score choice (that is the truck brain's, choice=1)
        set_param("choice", (float)ch);
        const char *cn = ch ? "choice 1" : "choice 0";
        // 1. the converter is found by the plugin (not primed): the slip of the first seconds must not stay as a tire correction
        sim_reset(RESET_HARD, "sim: crown victoria"); g_idle_sim = 823; set_param("idle_rpm", 823);
        use_tc_crownvic(false);
        g_rec_on = 1; g_rec_n = 0;
        run("stomp at 32 km/h in 1st (converter found by the plugin)", {1963, 0, 1, 16, 1.67f}, c_cv_stomp);
        g_rec_on = 0;
        tc_check(V.tc == 1 && fabsf(V.rpm_k - 1.0f) < 0.03f, "%s: converter found (tc %d), the tire correction back at %.3f (expect 1.00: the slip of the first seconds is not a tire size)", cn, V.tc, V.rpm_k);
        int up1 = rec_first_up_rpm();
        tc_check(up1 >= 4800, "%s: the first upshift after the stomp at %d rpm (expect >= 4800, near the power peak; the ride went up at 1840)", cn, up1);
        tc_check(rec_max_drop() <= 1 && hunts == 0, "%s: after the stomp: deepest downshift %d gears, %d hunting pairs (expect no hunting)", cn, rec_max_drop(), hunts);
        // 2. the same with the converter known: a launch at the floor holds every gear to high revs, a 4-speed takes its steps one by one
        sim_reset(RESET_HARD, "sim: crown victoria"); g_idle_sim = 823; set_param("idle_rpm", 823);
        use_tc_crownvic(true);
        g_rec_on = 1; g_rec_n = 0;
        run("launch at the floor", {1963, 0, 1, 16, 1.67f}, c_full);
        g_rec_on = 0;
        tc_check(rec_upshifts() >= 1 && rec_min_up_rpm() >= 4800, "%s: launch at the floor: %d upshifts, the lowest at %d rpm (expect >= 4800)", cn, rec_upshifts(), rec_min_up_rpm());
        bool skip = false;
        for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a + 1) skip = true;
        tc_check(!skip, "%s: launch at the floor: no gear skipped on a 4-speed box (the ride skipped 1->3 and 2->4)", cn);
        // 3. 60 km/h in 3rd at a light pedal, then the floor: a kickdown, and no upshift before the revs are high (the cascade does not kick a converter down)
        if (!ch) continue;
        sim_reset(RESET_HARD, "sim: crown victoria"); g_idle_sim = 823; set_param("idle_rpm", 823);
        use_tc_crownvic(true);
        g_rec_on = 1; g_rec_n = 0;
        run("kickdown from 3rd at 60 km/h", {1963, 60, 3, 14, 1.67f}, c_cv_kick);
        g_rec_on = 0;
        tc_check(rec_max_drop() >= 1 && (rec_min_up_rpm() == 0 || rec_min_up_rpm() >= 4800), "%s: kickdown: deepest downshift %d, lowest upshift %d rpm (expect a kickdown, then no upshift under 4800)", cn, rec_max_drop(), rec_min_up_rpm());
    }
    g_idle_sim = 600; set_param("idle_rpm", 600); set_param("choice", choice0);
}
// ---- the Crown Victoria in every mode, with the game's converter (pulling over the driveline, coasting under it, an idle in gear), ride 31-10 (later):
// at the floor it went 1->2->3->4 inside a second at 34-37 km/h in Eco (an aggression capped at 0.3 gave the shift points of a calm drive to a floored
// pedal and the converter's stall speed was over them), a lift at 52 km/h in 4th ended in a 'stall' downshift, and a 3->4 landed at 870 rpm.
static int rec_whys(const char *w)   // the shifts the recorder saw with this reason
{
    int n = 0;
    for (int i = 0; i < g_rec_n; i++) if (!strncmp(g_rec[i].why, w, strlen(w))) n++;
    return n;
}
static void c_cv_ride(double t)   // 0.5 to 7 s, the floor to 13 s, then the pedal up: the coast of the ride
{
    M.pedal = t < 7 ? 0.5f : t < 13 ? 1.0f : 0.0f;
}
static void c_cv_cycle(double t)   // pull, coast, pull, coast: the samples the calibration takes (a different speed each time)
{
    double ph = fmod(t, 40.0);
    M.pedal = ph < 6 ? 0.45f : ph < 14 ? 0.3f : ph < 24 ? 0.0f : ph < 32 ? 0.35f : 0.0f;
    if (M.v * 3.6f > 80) M.pedal = 0;
}
static void run_crownvic_modes()
{
    printf("\n=== TC: the Crown Victoria in every mode (the game's converter) ===\n");
    const float choice0 = param_def("choice");
    static const struct { const char *n; float cvar; } modes[] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.67f } };
    for (int ch = 0; ch < 1; ch++) {   // (the cascade only: see run_crownvic)
        set_param("choice", (float)ch);
        for (int mi = 0; mi < 3; mi++) {
            char cn[40]; snprintf(cn, sizeof cn, "choice %d %s", ch, modes[mi].n);
            // 1. the floor from a standstill: every gear is held to high revs, in every mode
            sim_reset(RESET_HARD, "sim: crown victoria modes"); g_idle_sim = 823; set_param("idle_rpm", 823);
            use_tc_crownvic_game(false);
            g_rec_on = 1; g_rec_n = 0;
            run("launch at the floor", {1963, 0, 1, 16, modes[mi].cvar}, c_full);
            g_rec_on = 0;
            tc_check(rec_upshifts() >= 1 && rec_min_up_rpm() >= 4500, "%s: launch at the floor: %d upshifts, the lowest at %d rpm (expect >= 4500 in every mode: a floored pedal is a kickdown, the power peak is 4500-5200)", cn, rec_upshifts(), rec_min_up_rpm());
            tc_check(rec_max_drop() <= 1 && hunts == 0, "%s: launch at the floor: deepest downshift %d gears, %d hunting pairs", cn, rec_max_drop(), hunts);
            // 2. the ride: a calm pull, the floor, then the pedal up in 4th: no stall downshift, no hunting, no dive into the lug range (no eco-roll:
            // the pedal-up stretch is a coast in gear, which is what the box has to handle)
            sim_reset(RESET_HARD, "sim: crown victoria modes"); g_idle_sim = 823; set_param("idle_rpm", 823);
            const float roll_0 = param_def("eco_roll");
            set_param("eco_roll", 0);
            use_tc_crownvic_game(true);   // (the converter known, as the file of the last drive has it; the floor launch above is the one where the plugin finds it)
            g_rec_on = 1; g_rec_n = 0;
            run("calm pull, the floor, then the pedal up", {1963, 0, 1, 50, modes[mi].cvar}, c_cv_ride);
            g_rec_on = 0;
            float th = M.gear > 0 ? vehicle_rpm_at(M.gear, M.v) : 0;
            tc_check(rec_whys("stall") == 0 && hunts == 0, "%s: pedal up after the floor: %d stall downshifts, %d hunting pairs (expect none), end gear %d at %.0f km/h, driveline %.0f rpm", cn, rec_whys("stall"), hunts, M.gear, M.v * 3.6f, th);
            int lug_up = 0; float lug_min = 1e9f;
            for (int i = 0; i < g_rec_n; i++) {
                if (!(g_rec[i].b > g_rec[i].a)) continue;
                float land = vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f);   // the driveline's rpm in the gear it went up to
                lug_min = fminf(lug_min, land);
                if (land < 1000) lug_up++;
            }
            tc_check(lug_up == 0, "%s: no upshift of the ride lands under 1000 rpm of driveline (the lowest %.0f; the ride went 1->2->3->4 at 21-33 km/h to 576)", cn, lug_min);
            set_param("eco_roll", roll_0);
        }
    }
    g_idle_sim = 600; set_param("idle_rpm", 600); set_param("choice", choice0);
    // 3. the calibration, the model: samples of the game's converter (pulling 1.04, coasting 0.94 over a plugin radius 5 % too big) for gears 1 and 2,
    // a coast alone for 3, nothing for 4: the gears with both sides take the mean, 3 its coast less the offset the calibrated gears show, 4 their mean
    sim_reset(RESET_HARD, "sim: crown victoria calibration");
    use_tc_crownvic_game(true);
    g_radius_err = 1.05f;   // the plugin's radius is 5 % too big: its rpm per speed is 5 % too low
    {
        const float err = 1.05f, kc = 0.94f, kp = 1.04f;
        for (int g = 1; g <= 3; g++) {
            float v = 12.0f, base = vehicle_ratio_base(g);
            for (int i = 0; i < 400; i++) vehicle_ratio_sample(g, v, v * base * err * kc, 0.05f, RATIO_COAST);
            if (g < 3) for (int i = 0; i < 400; i++) vehicle_ratio_sample(g, v, v * base * err * kp, 0.05f, RATIO_PULL);
        }
        float want = err * 0.5f * (kc + kp);
        tc_check(vehicle_ratio_ready(1) == 1 && vehicle_ratio_ready(2) == 1 && fabsf(vehicle_ratio_q(1) - want) < 0.005f && fabsf(vehicle_ratio_q(2) - want) < 0.005f,
                 "calibration (model): gears 1 and 2 with both sides: q %.3f, %.3f (expect %.3f: the mean of the coast and the pull)", vehicle_ratio_q(1), vehicle_ratio_q(2), want);
        tc_check(vehicle_ratio_ready(3) == 2 && fabsf(vehicle_ratio_q(3) - want) < 0.012f, "calibration (model): gear 3 with a coast alone: q %.3f (expect %.3f: its coast less the offset of the coast)", vehicle_ratio_q(3), want);
        tc_check(vehicle_ratio_ready(4) == 0 && fabsf(vehicle_ratio_q(4) - want) < 0.012f, "calibration (model): gear 4 with nothing: q %.3f (expect %.3f: the mean of the calibrated gears)", vehicle_ratio_q(4), want);
        tc_check(fabsf(vehicle_rpm_at(4, 20.0f) - 20.0f * vehicle_ratio_base(4) * want) < 0.01f * 20.0f * vehicle_ratio_base(4), "calibration (model): vehicle_rpm_at carries it (%.0f rpm in 4th at 72 km/h)", vehicle_rpm_at(4, 20.0f));
        tc_check(fabsf(vehicle_kmh_at(4, vehicle_rpm_at(4, 20.0f)) - 72.0f) < 0.2f, "calibration (model): vehicle_kmh_at is its inverse (%.1f km/h)", vehicle_kmh_at(4, vehicle_rpm_at(4, 20.0f)));
        vehicle_ratio_prime(1, 1.0f); vehicle_ratio_prime(2, 1.0f); vehicle_ratio_prime(3, 1.0f);
        tc_check(fabsf(vehicle_ratio_q(4) - 1.0f) < 0.005f, "calibration (model): primed back to the ratios as given (gear 4 q %.3f)", vehicle_ratio_q(4));
    }
    // ... and the pipeline: TrueShift takes the samples itself in a drive (the plugin's radius 5 % too big, the converter found by the plugin, no eco-roll so the
    // pedal-up stretches are coasts in gear): the top gear, which this drive uses, gets both sides, and the tire correction stays out of it
    sim_reset(RESET_HARD, "sim: crown victoria calibration"); g_idle_sim = 823; set_param("idle_rpm", 823);
    const float roll0 = param_def("eco_roll");
    set_param("eco_roll", 0);
    use_tc_crownvic_game(false);
    g_radius_err = 1.05f;
    run("pull and coast cycles", {1963, 0, 1, 240, 3.0f}, c_cv_cycle);
    set_param("eco_roll", roll0);
    printf("        q by gear:");
    for (int g = 1; g <= 4; g++) printf(" %d: %.3f (%d)", g, vehicle_ratio_q(g), vehicle_ratio_ready(g));
    printf("   rpm_k %.3f\n", V.rpm_k);
    float want = 1.05f * 0.99f;
    tc_check(V.tc == 1 && vehicle_ratio_ready(4) >= 1 && fabsf(vehicle_ratio_q(4) - want) < 0.04f, "calibration (drive): the top gear's q %.3f from the drive's own samples (expect %.3f +-0.04)", vehicle_ratio_q(4), want);
    tc_check(fabsf(V.rpm_k - 1.0f) < 0.005f, "calibration (drive): the tire correction stays at %.3f on a converter (expect 1.000)", V.rpm_k);
    g_radius_err = 1;
    g_idle_sim = 600; set_param("idle_rpm", 600);
}
static void run_crownvic_cruise();
static void run_car_power();
static void run_car_lug();
static void run_car_launch();
static void run_car_wot();
static void run_car_protect();
static void run_crownvic_braking();
static int run_tc()
{
    printf("\n=== TC: the converter model ===\n");
    g_tc_failed = 0;
    // the pickup locks in 1st: slip at low speed, locked above ~the stall speed / coupling point
    tc_model_check("pickup (stall 2.0, lock gear 1)", use_tc_pickup, 8300, 1, 1);
    // the Allison truck locks from 2nd: in 1st the converter still slips at launch, then locks
    tc_model_check("allison (stall 2.42, lock gear 2)", use_tc_allison_2700, 20000, 1, 2);
    // the 3-speed slushbox never locks: the engine always sits above the driveline
    {
        sim_reset(RESET_HARD, "sim: tc model");
        use_tc_ford3();
        memset(&M, 0, sizeof M); for (int o = 0; o < ACT_N; o++) g_act[o] = act_wish{0, 0, 1, 1, 1}; g_game_lim = 1.0f;
        M.mass = 1800; M.cvar = 3; M.shift_time = 0.7f; M.pedal = 1.0f; M.gear = M.target = 1;
        T.fwd_gears = NG; T.diff = DIFF; for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
        float slip_sum = 0; int n = 0;
        for (int i = 0; i < 60 * 60 && M.v * 3.6f < 60; i++) {
            tick(1 / 60.0, 0);
            if (!M.state && M.gear > 0 && M.v > 3 && M.v < 20) { float drv = M.v / RADIUS * RATIO[M.gear - 1] * DIFF * 9.5493f; slip_sum += rpm_now() / fmaxf(drv, 1.0f); n++; }
        }
        tc_check(n > 0 && slip_sum / n > 1.15f, "ford 3-speed (never locks): the engine slips at all speeds (mean slip %.2f, expect > 1.15)", slip_sum / fmaxf(n, 1));
    }
    printf("\n=== TC: the fleet ===\n");
    tc_fleet_run("pickup 8.3 t", use_tc_pickup, 8300, 1, 80);
    tc_fleet_run("allison 20 t (2700 Nm)", use_tc_allison_2700, 20000, 1, 80);
    tc_fleet_run("allison 13 t (1800 Nm)", use_tc_allison_1800, 13000, 1, 85);
    tc_fleet_run("ford 3-speed", use_tc_ford3, 1800, 1, 80);
    tc_fleet_run("ford 4-speed", use_tc_ford4, 1800, 1, 80);
    tc_fleet_run("ford 4-speed OD", use_tc_ford4b, 1800, 1, 80);
    tc_fleet_run("ford 10-speed", use_tc_ford10, 1800, 1, 80);
    run_car_feel();
    run_crownvic();
    run_crownvic_modes();
    // (run_crownvic_cruise tests the score choice on a car (choice=1); the car automatic has no score choice, so it is not run)
    run_crownvic_braking();
    run_car_power();
    run_car_lug();
    run_car_launch();
    run_car_wot();
    run_car_protect();
    printf("\nTC: %s\n", g_tc_failed ? "FAILED" : "ok");
    return g_tc_failed;
}

// ---- SCORES: the score-based gear choice ([trueshift] choice = 1, core/gearscore.h). Run: tools/sim/run_scores.sh
// Part 1: the pure score function on the vehicle model. Part 2 (further down): choice 0 against choice 1 in the simulator.
static int g_sc_failed;
static void sc_check(bool ok, const char *what) { printf("  %-4s %s\n", ok ? "ok" : "FAIL", what); if (!ok) g_sc_failed = 1; }
static void sc_prime(float mass, float shift_time, float kmh, int gear)   // the vehicle model reads this truck now
{
    memset(&M, 0, sizeof M);
    M.mass = mass; M.shift_time = shift_time; M.cvar = 3; M.v = kmh / 3.6f; M.gear = M.target = gear;
    t_now += 10; T.speed = M.v; T.engine_on = 1;
    vehicle_reset(RESET_HARD); vehicle_update(SELF);
}
// the settings of a mode as TrueShift builds them from the ini defaults (set_param changes win)
static gs_cfg sc_cfg(int mode, int skips)   // mode: 0 Normal, 1 Eco, 2 Power
{
    float v[9]; const char *k[9] = { "choice_comfort", "choice_comfort_eco", "choice_comfort_power", "choice_rpm_weight", "choice_shift_weight", "choice_stay", "choice_stay_power", "hysteresis_rpm", "choice_floor_rpm" };
    for (int i = 0; i < 9; i++) for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, k[i])) v[i] = *m_trueshift.params[j].val;
    gs_cfg c = {};
    c.comfort = v[mode == 1 ? 1 : mode == 2 ? 2 : 0]; c.w_rpm = v[3]; c.w_shift = v[4]; c.stay = v[mode == 2 ? 6 : 5]; c.hyst = v[7]; c.floor = v[8]; c.floor_cur = 1;
    c.idle = 550; c.skips = c.skips_up = c.skips_later = skips; c.horizon = 8; c.learn_res = 0;
    return c;
}
// the force a pedal asks in gear cur at the speed of now, as TrueShift's f_need holds it in steady state
static float sc_need(int cur, float pedal) { return fmaxf(vehicle_force_at(cur, M.v, pedal), vehicle_resist(M.v)); }
static void sc_show(const char *title, const gs_out &o, int cur)
{
    printf("  %s\n", title);
    for (int g = 1; g <= NG; g++) {
        if (g < cur - 3 || g > cur + 3) continue;
        const gs_term &t = o.t[g];
        if (!t.g) continue;
        printf("    gear %2d%s rpm %5.0f share %4.2f  %s  gas %.3f rpm %.3f shift %.3f (%d) stay %.3f = %7.3f\n", g, g == cur ? "*" : " ", t.rpm, t.share > 8 ? 9.0 : t.share,
               t.ok ? "ok " : "out", t.gas, t.rpmc, t.shiftc, t.shifts, t.stay, t.score);
    }
}
static int run_scores_layers();
static int run_scores()
{
    // The SCORES run is the layered powertrain (part 18 and its sub-parts). Parts 1-17 judged the old gear choice (the shift points, the scores of
    // core/gearscore.h through TrueShift's choice_* keys) and went with it; the sim of the tag old-gear-logic still has them.
    use_ato();
    return run_scores_layers();
}

// ---- part 2: choice 0 (the cascade) against choice 1 (the scores) in the simulator
struct sc_res { int shifts, skips; double shift_s, rpm, t50; int hunts; };
static sc_res sc_last;
static void sc_do(const char *name, setup su, void (*ctl)(double), int choice)
{
    set_param("choice", (float)choice);
    // a short drive first: the module keeps a once-per-standstill flag (the launch correction) until the truck moves, and a run that ends
    // in a crawl would give the one after it a different start than its twin. In the scenario's mode: the module reads the mode once a
    // second, at a phase the run before sets, so the first second of an Eco or Power run was Normal or not depending on that run
    setup wash = {20000, 30, 6, 2}; wash.cvar = su.cvar;
    g_quiet = 1; run("wash", wash, c_hold80); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: next scores scenario");   // every run starts from a module that knows nothing of the one before
    g_quiet = getenv("SC_VERBOSE") ? 0 : 1;
    run(name, su, ctl);
    g_quiet = 0;
    sc_last = { shifts, skips_up + g_dskips, g_sh_t, g_rpm_t > 0 ? g_rpm_s / g_rpm_t : 0, t_50, hunts };
}
static void sc_print(const char *label, int choice, const sc_res &r)
{
    char t50[16]; if (r.t50 < 0) snprintf(t50, sizeof t50, "   -"); else snprintf(t50, sizeof t50, "%4.1f", r.t50);
    printf("  %-44s %d  %6d %6d %8.1f %8.0f  %s %6d\n", label, choice, r.shifts, r.skips, r.shift_s, r.rpm, t50, r.hunts);
}
static sc_res g_cmp[2];
static void sc_pair(const char *label, setup su, void (*ctl)(double))   // both choices on the same scenario; the results stay in g_cmp
{
    for (int i = 0; i < 2; i++) { int ch = getenv("SC_REV") ? 1 - i : i; sc_do(label, su, ctl, ch); g_cmp[ch] = sc_last; sc_print(label, ch, sc_last); }
}
// the regression of a real ride: the pedal ramps up, the truck skips at 45 km/h, the driver lifts to 0.00 a second after the skip landed
static int g_rg_roll0, g_rg_roll10; static double g_rg_lift, g_rg_first; static float g_rg_p0 = 0.45f; static int g_rg_g0, g_rg_sh0, g_rg_sh10; static bool g_rg_done10;
static int sc_gear_for(float kmh, float rpm_min)   // the highest gear that still gives at least rpm_min at kmh
{
    int g0 = 1;
    for (int g = 1; g <= NG; g++) if ((kmh / 3.6f) / RADIUS * RATIO[g - 1] * DIFF * 9.5493f >= rpm_min) g0 = g;
    return g0;
}
static void c_hold45(double) { hold(45); }
static void c_hold70(double) { hold(70); }
// c_fullramp: pedal 0.55 (from the first tick: setup.pedal0) for 3 s, then a ramp by 0.4/s to 1.0 (too slow for a kickdown). g_fr_full: when
// it reached 1.0 (-1 = not yet); the first downshift from the start of the ramp on is caught (g_dn1_t)
static double g_fr_ramp, g_fr_full;
// c_cc83: the cruise control holding 83 km/h (a smooth controller); r_cruise3: 2 km flat, then 1 km at 3 %, then flat
static void c_cc83(double) { M.cruise_kmh = 83; M.pedal = 0; float e = 83 / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.25f + e * 1.0f)); }
static float r_cruise3(double s) { return s < 2000 ? 0 : s < 3000 ? 3.0f : 0; }
static double g_c12_up; static int g_c12_g12;   // the first time in 12; 1 = 12 at the end of the climb
// c_pull: a fixed pedal (g_pl_ped) from a standstill; the shifts until 70 km/h (or the end) are counted from the recorder. g_pl_wish = 1: the
// engine's throttle is the wish pedal's (a power wish, the throttle that delivers it in this gear, as modules/pedal.cpp does it in Normal,
// in gear above 12 km/h), the driver's pedal stays where it is, and TrueShift reads the wish (core/wish.cpp)
static float g_pl_ped = 0.7f; static int g_pl_wish; static double g_pl_t0, g_pl_t50, g_pl_t70, g_pl_sh70;
extern module m_pedal;
// The torque pedal as the plugin runs it: the real modules/pedal.cpp (included at the end of this file) gets the driver's pedal M.pedal every tick, as Cruise calls it, and
// the plant's engine takes the throttle it returns (M.wish_on, M.wish_thr); core/wish.cpp keeps the demand TrueShift reads. Not with the cruise on, as in the game. A call after
// a gap (a new run, the cruise handing the pedal back) starts the pedal's state over, with the truck's curve factor taken as at a standstill.
static void sim_pedal_wish()
{
    static double last = -1e9;
    if (t_now < last || t_now - last > 0.5) {
        // (the telemetry of the plant as tick() hands it to the plugin, so the pedal sees the truck as it is now and not the last run's end; the foot is already on the pedal
        // the stand-in used to have: the shaping starts at the pedal, no rise from zero)
        T.engine_on = 1; T.gear = M.gear; T.speed = M.v; T.rpm = fmaxf(rpm_now(), TORQUE < 1000 ? 750.0f : g_idle_sim); T.in_thr = M.pedal; T.cruise = M.cruise_kmh / 3.6f; T.eff_clutch = M.state ? 1 : 0;
        m_pedal.reset(RESET_HARD, "sim: a new run");
        wish_follow(M.pedal);
    }
    last = t_now;
    M.wish_on = 0;
    if (M.cruise_kmh > 0) return;
    float out = M.pedal;
    M.wish_on = pedal_wish(M.pedal, 1 / 60.0f, 0, &out);
    M.wish_thr = out;
    if (getenv("SIM_DBGP") && M.pedal > 0.125f && M.pedal < 0.135f) { static int n; if (n++ < 12) printf("DBGP t %.3f p %.2f on %d thr %.3f share %.3f eff %.3f k %.3f level %.0f in(g) %.0f gear %d v %.2f rpm %.0f\n", t_now, M.pedal, M.wish_on, M.wish_thr, wish_share(), wish_share_eff(), wish_limit_k(), wish_level(), wish_in(M.gear, M.v), M.gear, M.v, T.rpm); }
}
static float cf_wish_force(float v);
#ifdef SIM_PCC
float sim_cruise_wish_force(float v) { return cf_wish_force(v); }   // (sim_pcc links the real Cruise: these part-4 stubs are parked)
float sim_cruise_top_kmh() { static float t = getenv("SW_TOP") ? (float)atof(getenv("SW_TOP")) : 0.0f; return t; }
#else
float cruise_wish_force(float v) { return cf_wish_force(v); }   // [cruise] force_mode 1 (part 4's cruise model; -1 elsewhere)
float cruise_wish_ahead(float v) { return cf_wish_force(v); }   // (part 4's model has no rate limit to look past)
static float g_sim_top = 0;   // the truck's limiter (km/h) a scenario sets (0 = none; the game's g_use_speed_limiter value)
static float sim_top_env() { static float t = getenv("SW_TOP") ? (float)atof(getenv("SW_TOP")) : 0.0f; return t; }
float speed_top_kmh() { return g_sim_top > 0 ? g_sim_top : sim_top_env(); }   // SW_TOP: the truck's limiter (km/h; 0 = none)
float cruise_hold_kmh(float) { return NAN; }   // (the game's smart overspeed: part 4's cruise model has none)
int cruise_overtake_on() { return 0; }          // [cruise] overtake_assist (not part of this sim: no overtake, so the cruise's force wish is never skipped)
#endif
// The rows of part 18i were written for the pedal map of d99b2fd (Eco p^1.1, Normal p, Power p^0.95; the truck factor was 1 for all their trucks): "a light pull", "a lug pull" are a DEMAND, the
// share the box plans for, and the map is the owner's choice (P6: s = p^e, Eco 0.8 / Normal 0.7 / Power 0.6). With g_pl_demand set, g_pl_ped is that old share and the foot is the pedal whose
// share under the map in force is the same, so those rows keep their situation (a calm pull, a lug pull) and judge the box's answer to a demand on its own. What the new map does to a FOOT
// is judged by the rows that stay on the foot (18b, 18c, 18f, pedalmix, the p6 table).
static int g_pl_demand = 0;
static float foot_for_old_map(float p)
{
    if (!(p > 0) || p >= 1) return p;
    const int md = ts_mode();
    const char *key = md == MODE_ECO ? "curve_eco" : md == MODE_POWER ? "curve_power" : "curve_normal";
    float c = 1000.0f, k = 1.0f;   // (the diagonal if the module has no such key)
    for (int i = 0; i < m_pedal.n_params; i++) {
        if (!strcmp(m_pedal.params[i].key, key)) c = *m_pedal.params[i].val;
        if (!strcmp(m_pedal.params[i].key, "curve_toe")) k = *m_pedal.params[i].val;
    }
    const float e_old = md == MODE_ECO ? 1.1f : md == MODE_POWER ? 0.95f : 1.0f;
    const float s = powf(p, e_old);   // the share the old map gave
    return powf(s * c / (1.0f + c - s), 1.0f / k);   // the pedal whose share under the map s = (1 + c) p^k / (p^k + c) is that share (P6c; k 1 is P6b's rational map)
}
static void c_pull(double t)
{
    if (t <= 0) { g_pl_t0 = t_now; g_pl_t50 = g_pl_t70 = -1; g_pl_sh70 = -1; g_rec_n = 0; EN = {}; EN.on = EN.on60 = 1; }
    M.pedal = g_pl_demand ? foot_for_old_map(g_pl_ped) : g_pl_ped; M.wish_on = 0;
    if (g_pl_wish) sim_pedal_wish();   // the driver's pedal stays g_pl_ped (T.in_thr), the engine gets the torque pedal's throttle
    if (g_pl_t50 < 0 && M.v * 3.6f >= 50) g_pl_t50 = t;
    if (g_pl_t70 < 0 && M.v * 3.6f >= 70) { g_pl_t70 = t; g_pl_sh70 = g_sh_t; EN.on = 0; }
}
struct pl_res { int shifts, skips; double shift_s, t50, t70, gap; int gap_ok; };
static const char *g_pl_label[3] = { "choice 0", "choice 1, same force", "choice 1, time" };
// c_stop: from the start speed with the pedal off, g_st_coast seconds coasting, then the brake g_st_brake to a stop; the stall rule and the lowest
// rpm in gear above 5 km/h (the game's own range below the handback) are counted
static float g_st_brake = 0.2f, g_st_coast = 0, g_st_rpm_min; static int g_st_stall, g_st_unseen;   // g_st_unseen: the brake is not in the telemetry

// ---- part 4: the power-reserve floor, our own brake request, the route's cap and the upshift on a lift
// the ride's hunt (33.1 t ATO, 1.0 s box, 0.7 %, a junction ahead; 166.8 s: 8->7->9->7 in 8.1 s at 33 km/h): in 8 at 45 km/h the pedal off and
// the engine brake at 0.25, then the deceleration assist brakes (its request does not show in the telemetry) to 33 km/h, the driver pulls at
// 0.45, then lifts (2.5 s after an upshift landed, else after 7 s) with the engine brake at 0.25 and the assist braking a little
static float r_07(double) { return 0.7f; }
static int g_hn_ph; static double g_hn_t, g_hn_up, g_hn_up_s, g_hn_hold = 7;   // g_hn_up_s: where the first upshift to 9 landed (m)
static void sc_seq(char *out, int n)   // the recorded shifts as 9>8>7 ...
{
    int k = 0; out[0] = 0;
    for (int i = 0; i < g_rec_n && k < n - 8; i++) k += snprintf(out + k, n - k, i ? ">%d" : "%d>%d", i ? g_rec[i].b : g_rec[i].a, g_rec[i].b);
}
// the lifts: the pedal at 0.45 (g_lf_ped), off at g_lf_at for g_lf_dur seconds, then back; the grade g_lf_grade
static float g_lf_at = 2, g_lf_dur = 4, g_lf_grade = 0, g_lf_ped = 0.45f;
// a driver who lifts the same way again and again (the session's lifts are learned): on for g_tc_on s (0.45 under 45 km/h, else 0.2), off for g_tc_off s
static float g_tc_on = 4, g_tc_off = 3;
// a steady foot at 0.15, or one wavering 0.05-0.25 every 5 s (g_fv_amp 0.1); the floor lines are caught from the start of the run on
static float g_fv_amp; static void c_floorped(double t)
{
    if (t <= 0) { g_cf_rpm = -1; g_cf_varied = 0; }
    M.grade_pct = 0.6f; M.pedal = 0.15f + g_fv_amp * sinf((float)t * 6.2832f / 5);
}

// ---- part 5: the cruise control's demand as a force ([cruise] force_mode, modules/cruise.cpp) with choice 1
// The own cruise controller as modules/cruise.cpp runs it in Normal (PI per km/h: kp 0.25, ki 0.08, output rate 0.6/s, twice that down,
// the look-ahead error of approach_s 2.5 s, held in a shift, no wind-up at the limits, 0 while rolling in neutral). force_mode 0: the output
// is the throttle and TrueShift sees only that; 1: the output is a share of a full wish (the wish pedal's reference, wish_full), the
// throttle is that force in the gear of now, the gains and the rate per throttle the same, and TrueShift reads the force (cruise_wish_force)
static struct { int on, fm; float i, u, k, acc_f, v_prev; double at; } CF = { 0, 0, 0, 0, 1, 0, 0, -1 };
static float cf_wish_force(float v) { return CF.on && CF.fm && CF.at >= 0 && t_now - CF.at <= 0.3 ? CF.u * wish_full(v) : -1; }
#ifndef SIM_PCC
float cruise_wish_share() { return CF.on && CF.fm && CF.at >= 0 && t_now - CF.at <= 0.3 ? fminf(1.0f, fmaxf(0.0f, CF.u)) : NAN; }   // [cruise] force_mode 1 as a share 0..1
#endif
static float g_cf_set = 83;
// [cruise] approach_by_mode as modules/cruise.cpp has it (g_cf_app 1; 2 = a test-only variant that asks all the pull to spare in every mode):
// the throttle the approach may use is the force that holds the speed plus the mode's share of what a full wish has to spare, over what the
// gear gives; the share is the mode's own key (approach_share_eco / approach_share / approach_share_power). More when the truck gains under 0.05 m/s2 while 1 km/h under.
static int g_cf_app = 0; static float g_cf_ex = 0, g_cf_ceil = 1;
static float g_cf_sh[3] = { 0.9f, 0.95f, 1.0f };   // approach_share_eco / approach_share / approach_share_power at their defaults (SC_SHARES=a,b,c overrides, for tuning)
static float cf_ceiling(int md, float e, float acc_kmhs, float dt, int shifting)
{
    bool slow = acc_kmhs < 0.18f && e > 1.0f;
    g_cf_ex = slow ? fminf(1.0f, g_cf_ex + 0.5f * dt) : fmaxf(0.0f, g_cf_ex - 0.5f * dt);
    if (shifting) return g_cf_ceil;
    float v = M.v, full = wish_full(v), fg = M.gear > 0 ? vehicle_force_at(M.gear, v, 1.0f) : 0;
    if (!(full > 0) || !(fg > 0)) return g_cf_ceil = 1;
    float fh = fmaxf(0.0f, vehicle_resist(v)), share = g_cf_app == 2 ? 1.0f : g_cf_sh[md == MODE_ECO ? 0 : md == MODE_POWER ? 2 : 1];
    if (full - fh < 0.25f * full) return g_cf_ceil = 1;
    share += (1 - share) * g_cf_ex;
    return g_cf_ceil = fminf(1.0f, fmaxf(0.0f, (fh + share * fmaxf(0.0f, full - fh)) / fg));
}
static void c_cforce(double t)
{
    const float dt = 1 / 60.0f;
    M.cruise_kmh = g_cf_set; M.pedal = 0;
    float kmh = M.v * 3.6f, e = g_cf_set + g_roll_boost_kmh - kmh;
    float a = t <= 0 ? 0 : (M.v - CF.v_prev) / dt * 3.6f; CF.v_prev = M.v;
    if (t <= 0) { CF.on = 0; CF.at = -1; CF.acc_f = 0; }
    CF.acc_f += (a - CF.acc_f) * fminf(1.0f, dt / 0.5f);
    int md = mode_of(M.cvar);   // (Eco / Power: the gains and the rate as cruise_body has them)
    float ep = CF.acc_f > 0 ? e - 2.5f * (md == MODE_ECO ? 1.2f : md == MODE_POWER ? 0.6f : 1.0f) * CF.acc_f : e;
    float kp = 0.25f, ki = 0.08f, rate = 0.6f, k = 1;
    if (md == MODE_ECO) { kp = 0.18f; ki = 0.05f; rate = 0.4f; } else if (md == MODE_POWER) { kp = 0.35f; ki = 0.12f; rate = 1.0f; }
    int shifting = M.state != 0;
    if (CF.fm) {
        float full = wish_full(M.v), fg = M.gear > 0 ? vehicle_force_at(M.gear, M.v, 1.0f) : 0;
        k = !shifting && full > 0 && fg > 0 ? full / fg : CF.k;
        CF.k = k; kp /= k; ki /= k; rate /= k;
    }
    if (!CF.on) { CF.on = 1; CF.u = M.cc / k; CF.i = CF.u - kp * ep; }
    if (g_rolling) { CF.u = 0; CF.i = -kp * e; M.cc = 0; CF.at = -1; return; }
    float u_raw = kp * ep + CF.i, u_cap = CF.fm ? fmaxf(1.0f, 1 / k) : 1.0f;
    if (g_cf_app && e > 2.5f) u_cap = fminf(u_cap, cf_ceiling(md, e, CF.acc_f, dt, shifting) / (CF.fm ? k : 1.0f)); else g_cf_ex = 0;
    float u = u_raw < 0 ? 0 : fminf(u_raw, u_cap), step = shifting ? 0 : rate * dt;
    if (u > CF.u + step) u = CF.u + step;
    if (u < CF.u - 2 * step) u = CF.u - 2 * step;
    int held_up = (u < u_raw - 1e-4f || (CF.fm && u * k > 1)) && e > 0, held_down = u > u_raw + 1e-4f && e < 0;
    if (!shifting && !held_up && !held_down) CF.i += ki * ep * dt;
    CF.i = fminf(CF.fm ? fmaxf(1.0f, 1 / k) : 1.0f, fmaxf(CF.fm ? -0.5f / k : -0.5f, CF.i));
    CF.u = u;
    M.cc = CF.fm ? fminf(1.0f, u * k) : u;
    CF.at = M.cc > 0 ? t_now : -1;
}
// ---- the Crown Victoria on the cruise (ride 31-10, 21:00): set 75 on a 5-8 % climb at 52-60 km/h in Eco, the box hunted 2<->1 for 25 s (1st at
// 59 km/h) and later took 4th at 52 km/h. The cruise's force wish (force_mode 1) is a speed controller: 23 km/h under the set speed it asks 8 kN
// for a car that holds the grade with 1.5, and the score read that as a pull to be answered from the lowest gear.
static void c_cv_ride_coast(double t) { M.pedal = t < 9 ? 0.5f : t < 12 ? 1.0f : 0.0f; }   // up to about 80 km/h, then the pedal up for good
static float g_cvc_grade;
static float g_cvc_set = 75;
static void c_cv_cruise(double t) { M.grade_pct = g_cvc_grade; g_cf_set = g_cvc_set; c_cforce(t); }
// ride 31-10 21:04: down a 5-7 % descent, the pedal 0.65-0.75, Eco: 1->2 at 32, 2->3 at 38 and 3->4 at 52 km/h (907 rpm of driveline, 1397 on the tach)
static void c_cv_down(double) { M.grade_pct = -6.0f; M.pedal = 0.7f; }
static void c_cv_down_wob(double t) { M.grade_pct = -6.0f; M.pedal = 0.45f + 0.25f * sinf((float)t * 0.9f); }
// ride 31-10 21:20: lifting off at 110-150 km/h towards a junction / a bend, the car went 3->2->1 at 82-85 km/h and 4000-5800 rpm (a truck's engine-brake
// rule: the plateau of a V8 is at 4100 rpm, so the 'keep the revs for the junction' floor was 4300), then held 1st for seconds
static void c_cv_brake_down(double t) { M.grade_pct = 0; if (t < 3) hold(120); else { M.pedal = 0; M.brake = t < 18 ? 0.18f : 0.5f; } }
static void run_crownvic_braking()
{
    printf("\n=== TC: a car lifts and brakes for a junction from 120 km/h (the game's converter) ===\n");
    static const struct { const char *n; float cvar; } modes[] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Sport", 1.67f } };
    for (int mi = 0; mi < 3; mi++) for (int car = 0; car < 2; car++) {
        char cn[60]; snprintf(cn, sizeof cn, "%s %s braking from 120", car ? "3-speed" : "Crown Victoria", modes[mi].n);
        sim_reset(RESET_HARD, "sim: car braking"); g_idle_sim = 823; set_param("idle_rpm", 823);
        if (car) use_tc_ford3(); else use_tc_crownvic_game(true);
        g_rec_on = 1; g_rec_n = 0;
        run("lift and brake for a junction", {car ? 1800.0f : 1963.0f, 120, car ? 3 : 4, 40, modes[mi].cvar, 0.5f, 0, -1, -1, NULL, 1, 700}, c_cv_brake_down);
        g_rec_on = 0;
        int to1_fast = 0; float hi_rpm = 0;
        for (int i = 0; i < g_rec_n; i++) {
            if (!(g_rec[i].b < g_rec[i].a)) continue;
            if (g_rec[i].b == 1 && g_rec[i].kmh > 30) to1_fast++;
            // the rpm the gear lands on: the driveline's in the gear it went to
            hi_rpm = fmaxf(hi_rpm, vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f));
        }
        tc_check(to1_fast == 0 && hi_rpm <= 0.65f * LIMIT, "%s: %d shifts, %d into 1st above 30 km/h, the highest rpm a downshift landed on %.0f (expect none into 1st at speed, <= %.0f = 65 %% of the limiter)", cn, shifts, to1_fast, hi_rpm, 0.65f * LIMIT);
    }
}
// ---- a light vehicle at the floor shifts at the power crossover (Damien 31-10: 'at full throttle it may rev a bit higher', 'in car mode look at the
// horsepower'): the wheel power in a gear is the engine's power at its rpm (torque curve x rpm), the next gear gives the same power at rpm / step, so the
// upshift is due where P(r / step) = P(r), past the power peak, capped just under the limiter. The cars: the Crown Victoria (4-speed, 6000), the Mustang's
// 3-speed (6000), the F-150 10-speed (6000) and a diesel pickup like the owner's (10 speeds, 3900, 2700 Nm)
static void use_tc_diesel39()   // the ATS pickup of the ride: 2700 Nm, limit 3900, plateau from 1600, 10 speeds, diff 3.31, 0.413 m, converter locking from 2nd
{
    float r[10] = { 4.70f, 2.99f, 2.15f, 1.80f, 1.52f, 1.28f, 1.00f, 0.85f, 0.69f, 0.64f };
    tc_reset();
    for (int i = 0; i < 16; i++) RATIO[i] = i < 10 ? r[i] : 0;
    float c[11][2] = { {0,0},{300,0},{500,0.45f},{1000,0.9f},{1600,1},{2000,0.98f},{2400,0.9f},{2800,0.75f},{3100,0.55f},{3400,0.35f},{3700,0} }; NCURVE = 11;
    memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1800,3400},{800,2000},{1100,2500},{1800,3400} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2700; CDA = 2.0f; LIMIT = 3900; DIFF = 3.31f; RADIUS = 0.413f; NG = 10;
    T.fwd_gears = 10; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    tc_set(2.0f, 2, 2200);
}
static double g_cp_t100;
static void c_full_t100(double t) { if (t <= 0) g_cp_t100 = -1; M.pedal = 1; if (g_cp_t100 < 0 && M.v * 3.6f >= 100) g_cp_t100 = t; }
static float cp_cross(float step, float *peak)   // the test's own copy of the crossover, from the plant's curve
{
    engine_info e; gb_engine(0, &e);
    float best = 0, rp = 0, cap = LIMIT * 0.975f;
    for (float r = 600; r <= LIMIT; r += 25) { float p = curve_factor(&e, r) * r; if (p > best) { best = p; rp = r; } }
    if (peak) *peak = rp;
    for (float r = rp; r < cap; r += 25) { float lo = r / step; if (curve_factor(&e, lo) * lo >= curve_factor(&e, r) * r) return r; }
    return cap;
}
static void run_car_power()
{
    printf("\n=== TC: a light vehicle at the floor shifts at the power crossover ===\n");
    static const struct { const char *n; void (*use)(); float mass; int tc; double secs; } cars[] = {
        { "Crown Victoria", NULL, 1963, 1, 30 }, { "3-speed (Mustang)", use_tc_ford3, 1400, 0, 30 }, { "F-150 10-speed", use_tc_ford10, 2400, 0, 40 }, { "diesel pickup 3900", use_tc_diesel39, 8300, 0, 60 } };
    // 0-100 km/h before this change, the same scenarios on the box as it was (fw12 bd62b31), Normal: filled in below by the baseline run
    static const float t100_before[4] = { 6.70f, 4.23f, 5.48f, 7.68f };
    for (int ci = 0; ci < 4; ci++) {
        sim_reset(RESET_HARD, "sim: car power"); g_idle_sim = 823; set_param("idle_rpm", 823);
        if (ci == 0) use_tc_crownvic_game(true); else cars[ci].use();
        if (ci == 3) { g_idle_sim = 700; set_param("idle_rpm", 700); }
        g_rec_on = 1; g_rec_n = 0;
        run("full pedal launch", {cars[ci].mass, 0, 1, cars[ci].secs, 3.0f}, c_full_t100);
        g_rec_on = 0;
        int worst_lim = 0, n_up = 0, bad_cross = 0; float lo_ratio = 9, hi_ratio = 0;
        printf("        upshifts (engine rpm at the shift / the crossover of that gear's step):");
        for (int i = 0; i < g_rec_n; i++) {
            if (!(g_rec[i].b > g_rec[i].a)) continue;
            float r0 = RATIO[g_rec[i].a - 1], r1 = RATIO[g_rec[i].b - 1], peak, cr = cp_cross(r0 / r1, &peak);
            n_up++;
            printf(" %d>%d %.0f/%.0f", g_rec[i].a, g_rec[i].b, g_rec[i].rpm, cr);
            if (!strncmp(g_rec[i].why, "rev limit", 9)) worst_lim++;
            if (g_rec[i].rpm >= LIMIT * 0.99f) worst_lim++;
            if (g_rec[i].b == g_rec[i].a + 1) { lo_ratio = fminf(lo_ratio, g_rec[i].rpm / cr); hi_ratio = fmaxf(hi_ratio, g_rec[i].rpm / cr); }
            if (g_rec[i].rpm < 0.85f * cr) bad_cross++;
        }
        printf("\n        0-100 km/h %.2f s (before: %.2f s)\n", g_cp_t100, t100_before[ci]);
        tc_check(n_up >= 2 && worst_lim == 0 && bad_cross == 0, "%s: %d upshifts, %d into the limiter or under 85 %% of the crossover (expect none), the shift rpm / crossover %.2f-%.2f", cars[ci].n, n_up, worst_lim + bad_cross, lo_ratio, hi_ratio);
        tc_check(g_cp_t100 > 0 && g_cp_t100 <= t100_before[ci] + 0.05f, "%s: 0-100 km/h in %.2f s (before %.2f s: expect no slower)", cars[ci].n, g_cp_t100, t100_before[ci]);
    }
    g_idle_sim = 600; set_param("idle_rpm", 600);
}
// ---- the car's lug limit (CarShift car_lug): derived from the engine's torque curve and the force the car needs, so it moves with the weight and the
// grade, with the gear and with the engine. Read after a short hold of the speed through the module's own accessor.
extern float cs_lug_rpm(int gear, float v_ms);
static float g_lug_grade;
static void c_lug_hold(double) { M.grade_pct = g_lug_grade; hold(g_tc_hold); }
static float lug_of(void (*use)(), float mass, float grade, int gear, float kmh)
{
    sim_reset(RESET_HARD, "sim: lug"); use();
    g_lug_grade = grade; g_tc_hold = kmh;
    int q = g_quiet; g_quiet = 1;
    run("lug", {mass, kmh, gear, 8}, c_lug_hold);
    g_quiet = q;
    return cs_lug_rpm(gear, kmh / 3.6f);
}
static void run_car_lug()
{
    printf("\n=== TC: the car's lug limit follows the vehicle ===\n");
    // the 10-speed Ford (1.8 t): 100 km/h in 7th (a high gear, little torque asked on the flat)
    float flat = lug_of(use_tc_ford10, 1800, 0, 7, 100), steep = lug_of(use_tc_ford10, 1800, 8, 7, 100);
    float heavy = lug_of(use_tc_ford10, 3400, 8, 7, 100), tall = lug_of(use_tc_ford10, 1800, 8, 10, 100);
    printf("        Ford 10-speed, 7th at 100 km/h: 1.8 t flat %.0f, 1.8 t 8 %% up %.0f, 3.4 t 8 %% up %.0f; 10th, 1.8 t 8 %% up %.0f\n", flat, steep, heavy, tall);
    tc_check(steep > flat + 200, "lug limit: a climb raises it (flat %.0f, 8 %% up %.0f rpm, expect > +200)", flat, steep);
    tc_check(heavy > steep + 50, "lug limit: more weight raises it (8 %% up: 1.8 t %.0f, 3.4 t %.0f rpm, expect > +50)", steep, heavy);
    tc_check(tall >= steep, "lug limit: a taller gear needs at least as much (7th %.0f, 10th %.0f rpm on the same climb)", steep, tall);
    // a torque-rich diesel (500 rpm: 45 % of its torque, 1000 rpm: 90 %) wants far less than a petrol V8 that wakes at 1500
    float diesel = lug_of(use_tc_diesel39, 3000, 0, 10, 100), v8 = lug_of(use_tc_ford10, 3000, 0, 10, 100);
    float diesel_trl = lug_of(use_tc_diesel39, 8300, 8, 10, 100);
    printf("        diesel pickup 10th at 100 km/h: 3 t flat %.0f, 8.3 t on 8 %% %.0f; Ford V8 at 3 t flat %.0f\n", diesel, diesel_trl, v8);
    tc_check(diesel_trl > diesel + 100, "lug limit: a trailer on a climb raises the pickup's (3 t flat %.0f, 8.3 t on 8 %% %.0f rpm, expect > +100)", diesel, diesel_trl);
    tc_check(flat >= 0.99f * (550 + 300) && steep < 0.97f * LIMIT, "lug limit: never under the downshift clearance, never past the plateau's start (%.0f / %.0f rpm)", flat, steep);
}
// ---- the car's launch (CarShift launch_pick): the start gear by physics, with the speed the launch reaches and the engine speed it locks at taken from
// the vehicle (the first gear's calm range, the converter's stall speed or the torque rise), not from two constants.
extern float cs_launch_rpm(float thr);
extern float cs_launch_kmh(int which);
static void c_standing(double) { M.pedal = 0; }
static int car_start_pick(void (*use)(), float mass, float grade, float thr)
{
    sim_reset(RESET_HARD, "sim: car start"); use();
    int q = g_quiet; g_quiet = 1;
    run("start", {mass, 0, 1, 2}, c_standing);
    g_quiet = q;
    M.v = 0; T.speed = 0; T.in_thr = thr; M.pedal = thr; M.grade_pct = grade; M.mass = mass;
    return s_start(SELF);
}
static void use_crownvic_primed() { use_tc_crownvic(true); }
static void run_car_launch()
{
    printf("\n=== TC: the car's launch follows the gearing and the converter ===\n");
    static const struct { const char *n; void (*use)(); float mass; } cars[] = {
        { "Crown Victoria 4-speed", use_crownvic_primed, 1963 }, { "Ford 3-speed", use_tc_ford3, 1400 }, { "Ford 10-speed", use_tc_ford10, 2400 },
        { "diesel pickup 10-speed", use_tc_diesel39, 3000 }, { "6-speed clutch car", use_car, 1800 } };
    float lk[5], le[5], r1[5], r6[5], r10[5]; int pick[5][4];
    for (int ci = 0; ci < 5; ci++) {
        sim_reset(RESET_HARD, "sim: car launch"); cars[ci].use();
        int q = g_quiet; g_quiet = 1; run("start", {cars[ci].mass, 0, 1, 2}, c_standing); g_quiet = q;
        lk[ci] = cs_launch_kmh(0); le[ci] = cs_launch_kmh(1);
        r1[ci] = cs_launch_rpm(0.1f); r6[ci] = cs_launch_rpm(0.6f); r10[ci] = cs_launch_rpm(1.0f);
        // (the sim's stub of the game's start gear says 4: not a gear of the 3-speed)
        const float thr[4] = { 0.15f, 0.4f, 0.8f, 0.4f }, grade[4] = { 0, 0, 0, 8 };
        for (int k = 0; k < 4; k++) pick[ci][k] = ci == 1 ? 0 : car_start_pick(cars[ci].use, cars[ci].mass, grade[k], thr[k]);
        printf("        %-24s launch up to %4.1f km/h (first gear's calm range), over at %4.1f km/h; locks at %4.0f rpm (pedal 0.1), %4.0f (0.6), %4.0f (1.0)\n"
               "        %-24s start gear flat, pedal 0.15 / 0.4 / 0.8: %d / %d / %d; 8 %% up, 0.4: %d\n", cars[ci].n, lk[ci], le[ci], r1[ci], r6[ci], r10[ci], "",
               pick[ci][0], pick[ci][1], pick[ci][2], pick[ci][3]);
    }
    // 8300 kg of diesel pickup and trailer on the same start
    int heavy = car_start_pick(use_tc_diesel39, 8300, 0, 0.8f);
    printf("        diesel pickup 10-speed at 8.3 t flat, pedal 0.8: start gear %d (3 t: %d)\n", heavy, pick[3][2]);
    // 0 Crown, 1 Ford 3, 2 Ford 10, 3 diesel, 4 clutch
    tc_check(lk[2] < lk[0] && lk[0] < lk[1], "launch: the shorter the first gear the shorter the launch (10-speed %.1f < Crown %.1f < 3-speed %.1f km/h)", lk[2], lk[0], lk[1]);
    tc_check(le[0] > lk[0] && le[2] > lk[2] && le[4] > lk[4], "launch: it is over after the second gear's calm range, beyond the first's (Crown %.1f > %.1f, 10-speed %.1f > %.1f km/h)", le[0], lk[0], le[2], lk[2]);
    tc_check(fabsf(r10[0] - 4300) < 1 && r1[0] < r6[0] && r6[0] < r10[0] && r10[2] > r1[2] + 1000, "launch: a converter couples at the engine speed the pedal holds it at, up to the stall speed (Crown %.0f / %.0f / %.0f rpm for pedal 0.1 / 0.6 / 1.0, stall 4300)", r1[0], r6[0], r10[0]);
    tc_check(r1[4] == r10[4] && r1[4] > 700 && r1[4] < 1600, "launch: a clutch car locks where its torque has risen, whatever the pedal (%.0f rpm)", r1[4]);
    bool low = true; for (int ci : { 0, 2, 4 }) for (int k = 0; k < 4; k++) if (pick[ci][k] != 1) low = false;
    tc_check(low, "launch: the Crown, the 10-speed and the clutch car start in 1st at every pedal and on a climb");
    tc_check(pick[3][3] <= pick[3][1] && pick[3][1] <= pick[3][2] && heavy <= pick[3][2], "launch: the pickup starts no higher on a climb (%d, flat %d) or with the trailer (%d, 3 t %d)", pick[3][3], pick[3][1], heavy, pick[3][2]);
}
// ---- a hard-driven car never loops: the floored pedal's upshift and kickdown share one criterion (ride 01-11, ATS RAM 3500 with a petrol engine, 10-speed
// converter box, 2.2 t, Sport, pedal floored: 5>2 kickdown, 2>5 up skip landing at 3094 rpm, 5>3 kickdown, 3>6 up skip at 3408, 6>4 kickdown ... eight of
// them). At the floor a skip may only land where the kickdown would not fire again, and the kickdown only lands where the upshift would not leave.
static void use_tc_ram_petrol()   // the ATS RAM 3500 with the petrol engine: 563 Nm, limit 5900, plateau from 3000, power range 4500-5500, 10 speeds, diff 3.73, 0.441 m
{
    float r[10] = { 4.70f, 2.99f, 2.15f, 1.80f, 1.52f, 1.28f, 1.00f, 0.85f, 0.69f, 0.64f };
    tc_reset();
    for (int i = 0; i < 16; i++) RATIO[i] = i < 10 ? r[i] : 0;
    float c[10][2] = { {0,0},{1000,0.35f},{1500,0.60f},{2000,0.78f},{2500,0.92f},{3000,1.00f},{4000,0.96f},{5000,0.82f},{5500,0.72f},{5900,0.62f} }; NCURVE = 10;
    memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {3500,5500},{1200,3000},{1500,3000},{4500,5500} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 563; CDA = 1.6f; LIMIT = 5900; DIFF = 3.73f; RADIUS = 0.441f; NG = 10;
    T.fwd_gears = 10; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    tc_set(2.0f, 2, 2300);
}
// 0.3 until 4 s, then the floor; the pedal dips to 0.75 for 0.15 s every 2.5 s (a trigger or a foot that lets go for a moment: the ride re-armed the
// kickdown every 2-3 s that way), and each rise is a fast press
static void c_wot_dips(double t)
{
    if (t < 4) { M.pedal = 0.3f; return; }
    M.pedal = fmod(t - 4, 2.5) < 0.15 ? 0.75f : 1.0f;
}
static void run_car_wot()
{
    printf("\n=== TC: a car at the floor does not loop (upshift and kickdown share one criterion) ===\n");
    static const struct { const char *n; float cvar; } modes[] = { { "Sport", 1.67f }, { "Normal", 3.0f } };
    for (int mi = 0; mi < 2; mi++) for (int g0 : { 8, 6 }) {
        sim_reset(RESET_HARD, "sim: car wot"); g_idle_sim = 700; set_param("idle_rpm", 700);
        use_tc_ram_petrol();
        g_rec_on = 1; g_rec_n = 0;
        const double t0 = t_now;
        char nm[96]; snprintf(nm, sizeof nm, "RAM petrol %s: 0.3 at 70 km/h in gear %d, then the floor with dips", modes[mi].n, g0);
        run(nm, {2200, 70, g0, 40, modes[mi].cvar}, c_wot_dips);
        g_rec_on = 0;
        // the loop: a reversal within 5 s of two shifts after the stomp's own response (the first kickdown and the upshift out of it are the driver driving)
        int kd = 0, loops = 0, bad_land = 0, bad_cross = 0, first_kd = -1, wot_up = 0; float lowest_land = 1e9f, pw_lo = RANGES[3][0];
        printf("        shifts (from>to at km/h, the rpm it lands on):");
        for (int i = 0; i < g_rec_n; i++) {
            float land = g_rec[i].kmh / 3.6f / RADIUS * RATIO[g_rec[i].b - 1] * DIFF * 9.5493f;
            printf(" %d>%d(%s) %.0f/%.0f@%.1fs", g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].kmh, land, g_rec[i].t - t0);
            if (!strncmp(g_rec[i].why, "kickdown", 8)) { kd++; if (first_kd < 0) first_kd = i; }
            if (i > 0 && i - 1 != first_kd && (g_rec[i].b > g_rec[i].a) != (g_rec[i - 1].b > g_rec[i - 1].a) && g_rec[i].t - g_rec[i - 1].t < 5 && g_rec[i - 1].t - t0 > 4) loops++;
            if (g_rec[i].b > g_rec[i].a && g_rec[i].t - t0 > 4.5) {   // an upshift of the floored pedal: a skip lands in the power band, a single step at the crossover
                wot_up++;
                lowest_land = fminf(lowest_land, land);
                if (g_rec[i].b > g_rec[i].a + 1 && land < pw_lo + 100) bad_land++;
                if (g_rec[i].b == g_rec[i].a + 1 && g_rec[i].rpm < 0.85f * cp_cross(RATIO[g_rec[i].a - 1] / RATIO[g_rec[i].b - 1], NULL)) bad_cross++;
            }
        }
        printf("\n");
        tc_check(loops == 0 && kd <= 3, "%s: %d shifts, %d kickdowns, %d up-down/down-up pairs after the stomp's own kickdown (the sim's own count, the stomp included: %d); expect no loop", nm, g_rec_n, kd, loops, hunts);
        tc_check(wot_up >= 3 && bad_land == 0 && bad_cross == 0, "%s: %d upshifts at the floor, lowest landing %.0f rpm; %d skips under the power band (%.0f + 100), %d single steps under 85 %% of the crossover (expect none)", nm, wot_up, lowest_land, bad_land, pw_lo, bad_cross);
    }
    g_idle_sim = 600; set_param("idle_rpm", 600);
}
// ---- the car's protections (CarShift's wrong gear, rev limit and stall rules post GP_PROTECT to the gear arbiter, core/gearreq.h): the first shift of
// a car that starts in a gear the rules must take it out of is the protection's, named
static void c_car_prot(double) { M.pedal = 0.5f; }
static void run_car_protect()
{
    printf("\n=== TC: a car's protections take it out of a bad gear (wrong gear, rev limit, stall) ===\n");
    static const struct { const char *n; float kmh; int g0; const char *want; } rows[] = {
        { "6-speed car, 150 km/h in 2nd (8500 rpm on a 6000 limit)", 150, 2, "wrong gear" },
        { "6-speed car, 162 km/h in 3rd (5940 rpm, 99 % of the limit)", 162, 3, "rev limit" },
        { "6-speed car, 25 km/h in 5th (525 rpm, under idle + 120)", 25, 5, "stall" } };
    for (const auto &r : rows) {
        sim_reset(RESET_HARD, "sim: car protect"); use_car();
        g_rec_on = 1; g_rec_n = 0;
        int q = g_quiet; g_quiet = 1;
        run(r.n, {1800, r.kmh, r.g0, 6}, c_car_prot);
        g_quiet = q; g_rec_on = 0;
        tc_check(g_rec_n >= 1 && !strncmp(g_rec[0].why, r.want, strlen(r.want)), "%s: the first shift is '%s' (got %s%s%s, %d > %d)", r.n, r.want, g_rec_n ? "'" : "none", g_rec_n ? g_rec[0].why : "", g_rec_n ? "'" : "", g_rec_n ? g_rec[0].a : 0, g_rec_n ? g_rec[0].b : 0);
    }
}
static float g_cvc_swing_lo, g_cvc_swing_hi, g_cvc_thr_lo, g_cvc_thr_hi;
static void c_cv_flat_cruise(double t)
{
    if (t <= 0) { g_cvc_swing_lo = g_cvc_thr_lo = 1e9f; g_cvc_swing_hi = g_cvc_thr_hi = -1e9f; }
    M.grade_pct = 0; g_cf_set = g_cvc_set; c_cforce(t);
    if (t > 40) { float k = M.v * 3.6f; g_cvc_swing_lo = fminf(g_cvc_swing_lo, k); g_cvc_swing_hi = fmaxf(g_cvc_swing_hi, k); g_cvc_thr_lo = fminf(g_cvc_thr_lo, M.cc); g_cvc_thr_hi = fmaxf(g_cvc_thr_hi, M.cc); }
}
static void run_crownvic_cruise()
{
    printf("\n=== TC: the Crown Victoria on the cruise 75 up a climb (the game's converter) ===\n");
    const float choice0 = param_def("choice");
    static const struct { const char *n; float cvar; } modes[] = { { "Eco", 10.0f }, { "Normal", 3.0f } };
    for (int ch = 1; ch < 2; ch++) {
        set_param("choice", (float)ch);
        for (int mi = 0; mi < 2; mi++) for (float gr : { 5.0f, 8.0f }) for (float v0 : { 52.0f, 30.0f }) for (float weak : { 1.0f, 0.5f }) {
            char cn[60]; snprintf(cn, sizeof cn, "choice %d %s %.0f %% from %.0f%s", ch, modes[mi].n, gr, v0, weak < 1 ? " (the game's weak pull)" : "");
            g_cvc_set = 75;
            const float eta0 = g_plant_eta; g_plant_eta = 0.9f * weak;   // (the game's car pulls half of what the model says on this climb: 2nd at 0.5-0.8 throttle held 52 km/h, accel ~0)
            sim_reset(RESET_HARD, "sim: crown victoria cruise"); g_idle_sim = 823; set_param("idle_rpm", 823);
            use_tc_crownvic_game(true);
            g_cruise_smooth = 1; CF.fm = 1; CF.on = 0; CF.at = -1; g_cvc_grade = gr;
            g_rec_on = 1; g_rec_n = 0;
            const double t_start = t_now;
            run("cruise 75 up a climb", {1963, v0, v0 > 40 ? 2 : 1, 70, modes[mi].cvar, 0.5f, 0, -1, -1, NULL, 1}, c_cv_cruise);
            g_rec_on = 0;
            g_cruise_smooth = 0; CF.fm = 0; CF.on = 0; CF.at = -1; g_plant_eta = eta0;
            int first_fast = 0, ups4 = 0;
            // (the first 3 s are the plant settling into the climb: the cruise is not on yet in the first frames)
            for (int i = 0; i < g_rec_n; i++) { if (g_rec[i].t < t_start + 3) continue; if (g_rec[i].b == 1 && g_rec[i].kmh > 30) first_fast++; if (g_rec[i].b == 4) ups4++; }
            // (the weak pull is a car that cannot hold the speed in the gear above: one failed upshift and its lesson are the box learning, not hunting)
            tc_check(first_fast == 0 && hunts <= (weak < 1 ? 2 : 0) && shifts <= 6, "%s: %d shifts, %d into 1st above 30 km/h, %d hunting pairs, end gear %d at %.0f km/h (expect none into 1st, no hunting%s, at most 6 shifts; the old box: 11-56 shifts, 1st at 52-69 km/h)", cn, shifts, first_fast, hunts, M.gear, M.v * 3.6f, weak < 1 ? " beyond one failed upshift" : "");
            tc_check(M.gear >= 2 && M.gear <= 3, "%s: it ends in gear %d (expect 2 or 3)", cn, M.gear);
        }
    }
    // a descent at a firm pedal: the driveline's rpm in the gear it goes up to stays out of the lug range (the ride took the 4th at 907 rpm)
    for (int mi = 0; mi < 2; mi++) for (int w = 0; w < 2; w++) {
        char cn[60]; snprintf(cn, sizeof cn, "choice 1 %s descent %s", modes[mi].n, w ? "wavering pedal" : "pedal 0.7");
        sim_reset(RESET_HARD, "sim: crown victoria descent"); g_idle_sim = 823; set_param("idle_rpm", 823);
        use_tc_crownvic_game(true);
        g_rec_on = 1; g_rec_n = 0;
        run("descent at a firm pedal", {1963, 25, 1, 40, modes[mi].cvar, 0.5f, 0, -1, -1, NULL, 1}, w ? c_cv_down_wob : c_cv_down);
        g_rec_on = 0;
        float lowest = 1e9f;
        for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a) lowest = fminf(lowest, vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f));
        tc_check(lowest >= 1100, "%s: %d shifts, no upshift under 1100 rpm of driveline (the lowest %.0f; the ride took the 4th at 907)", cn, shifts, lowest);
    }
    // the cruise on the flat (ride 31-10 21:20, the Mustang set 80: the speed swung 71-78 every 6 s, the throttle 0-0.9, for 35 s): a car's plant answers a
    // throttle step several times faster than the truck's the controller's gains were fitted on, so it must settle
    for (int mi = 0; mi < 2; mi++) for (int car = 0; car < 2; car++) {
        char cn[60]; snprintf(cn, sizeof cn, "%s %s flat cruise 75", car ? "3-speed" : "Crown Victoria", modes[mi].n);
        sim_reset(RESET_HARD, "sim: car cruise flat"); g_idle_sim = 823; set_param("idle_rpm", 823);
        if (car) use_tc_ford3(); else use_tc_crownvic_game(true);
        g_cruise_smooth = 1; CF.fm = 1; CF.on = 0; CF.at = -1; g_cvc_set = 75;
        run("cruise 75 on the flat", {car ? 1800.0f : 1963.0f, 70, 3, 100, modes[mi].cvar, 0.5f, 0, -1, -1, NULL, 1}, c_cv_flat_cruise);
        g_cruise_smooth = 0; CF.fm = 0; CF.on = 0; CF.at = -1;
        printf("        after 40 s: speed %.1f-%.1f km/h, throttle %.2f-%.2f\n", g_cvc_swing_lo, g_cvc_swing_hi, g_cvc_thr_lo, g_cvc_thr_hi);
        tc_check(g_cvc_swing_hi - g_cvc_swing_lo < 2.0f && g_cvc_thr_hi - g_cvc_thr_lo < 0.3f, "%s: after 40 s the speed stays within %.1f km/h and the throttle within %.2f (expect < 2 km/h, < 0.3: it settles)", cn, g_cvc_swing_hi - g_cvc_swing_lo, g_cvc_thr_hi - g_cvc_thr_lo);
    }
    // the idle of a car that never stands for the learner's 5 s (the ride: 550 all session, the lug floor 900 with the real idle at 840): the engine's floor
    // while it coasts in gear is learned, the plugin starting from the ini's 550 with the plant's idle at 823 and its in-gear floor at 870
    sim_reset(RESET_HARD, "sim: crown victoria idle"); g_idle_sim = 823; set_param("idle_rpm", 550);
    const float roll_i = param_def("eco_roll");
    set_param("eco_roll", 0);
    use_tc_crownvic_game(true);
    run("pull, then a long coast, the idle unknown", {1963, 0, 1, 150, 3.0f}, c_cv_ride_coast);
    set_param("eco_roll", roll_i);
    tc_check(ts_idle_rpm() > 780 && ts_idle_rpm() < 900, "idle learned from the coast floor: %.0f rpm (the ini said 550, the plant's floor is 870; expect 780-900)", ts_idle_rpm());
    set_param("choice", choice0);
    g_idle_sim = 600; set_param("idle_rpm", 600);
}
static float r_cf3(double s) { return s < 2000 ? 0 : s < 4000 ? 3.0f : 0; }   // 2 km flat, 2 km at 3 %, flat
static double g_cf_lo, g_cf_e, g_cf_end; static int g_cf_n;   // the lowest speed and the speed at the end of the climb, |e| on the flat after 30 s; the shifts go to the recorder

// ---- part 6: the cruise control's approach to the set speed and its hold, with choice 1 (ride 7: 33 t, 70 -> 83 km/h, 11 held 22 s before the 12th)
// The own cruise controller of part 5 in each mode (g_cruise_smooth on: TrueShift knows the throttle is the cruise's). Flat road, route on.
// A minute at 70 km/h first (the model of the resistance learns while the speed is steady), then the set speed goes to 83: the times are
// from that moment. Reports the approach (throttle at full for how long, the highest throttle, the time to the set speed) and when each gear came
static const double AP_PRE = 60;
static double g_ap_set_t, g_ap_wot, g_ap_thr_max, g_ap_t82; static int g_ap_g0; static float g_ap_cut_from = 1e9f, g_ap_cut_s = 0, g_ap_v0 = 70, g_ap_v1 = 83;
static float g_ap_gr; static float r_ap_gr(double) { return g_ap_gr; }   // a constant grade for the comparison table
static float r_ap_cut(double s) { return s < 1300 ? 2.0f : 0.0f; }   // 2 % up to 1.3 km (the pre-roll), then flat
static int sc_first_to(int gear)   // index of the first recorded shift into this gear, -1 = none
{
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b == gear) return i;
    return -1;
}

// ---- part 8: [trueshift] power_shift_crossover. At a firm pedal in Power (0.8 up) and a full pedal in Normal (0.95 up) the window top of each gear is
// the power crossover to the next one where that lies above the top of the power range (core/gearscore.h gs_cross_rpm). Per gear pair the crossover,
// then a full pedal from 30 to 80 km/h at 17 and 33 t with the key 0 and 1: the time, the shifts and the rpm each upshift happened at
static float g_x_ped = 1; static double g_x_t80, g_x_t0;


// ---- part 9: kickdown_by_wish ([trueshift], choice 1 with the wish pedal): the kickdown as the driver's unmet wish, against the flat-pedal rule
// The wish pedal is on as in part 2's wish rows (the best gear's full force at this speed is a full wish), the driver's pedal is T.in_thr.
// c_kw: the pedal holds g_kw_p0 for g_kw_hold s, then rises to g_kw_p1 in g_kw_ramp s. g_kw_r: the ratio of the wished force to this gear's force at
// full throttle at the last tick before a decision; at every kickdown it is checked (never one with the ratio under 1).
static float g_kw_p0 = 0.3f, g_kw_p1 = 1.0f, g_kw_hold = 6, g_kw_ramp = 24; static double g_kw_t0, g_kw_r;
static float g_kw_rmin = 9; static int g_kw_seen, g_kw_kd; static char g_kw_buf[2][16];
static float r_flat0(double) { return 0; }
static float r_two(double) { return 2.0f; }
static float r_minus1(double) { return -1.0f; }
struct kw_res { int shifts, downs, ups, pairs, kd; double first_dn; char why[24]; };

// ---- part 10: [trueshift] skip_land_frac. A skip upshift must land a share of the way up from the choice floor to the
// upshift point, so a moderate pedal ramps through fewer skips and with a smaller rpm drop per upshift; a full-pedal launch
// still skips as before.
static float g_sf_ped; static int g_sf_full;

// ---- part 13: the landing cost (choice_land_weight) and the cost of revs over the upshift point (choice_over_weight). The score, not a
// hard rule, keeps a gear that would land just above the floor under load from being taken (the ride: 7 -> 9 at 36 km/h, 867 rpm, pedal 0.5),
// and keeps a gear from being held far over the mode's upshift point (Eco, ride 8). A pedal ramp to a moderate pedal, 33 t and 17 t, flat and 2 %.
float ts_up_pt(); float ts_choice_floor();
static float g_lc_ped, g_lc_over_s; static double g_lc_t70 = -1, g_lc_t30 = -1;
static float r_2pct(double) { return 2.0f; }
struct lc_res { int shifts, skips, pairs; double drop, land, t3070, over_s, end_kmh; };
struct lc_cfg { const char *name; float frac, land, over; };
static void lc_set(const lc_cfg &c) { set_param("skip_land_frac", c.frac); set_param("choice_land_weight", c.land); set_param("choice_over_weight", c.over); }

// ---- part 12: the own cruise downshifts when the gear is too weak. With [cruise] force_mode 1 the cruise asks a force, and
// kickdown_by_wish reads it the same way as the pedal's wish: from 63 km/h in 11 (a step too tall) it downshifts to 10 within
// seconds and reaches 80 km/h sooner than without the kickdown.
static double g_ck_t80;

// ---- part 13: a held gear may not rev past the upshift point (ride 8: 13 s in 9 at 1510 rpm, up_pt 1230).
// The junction hold (both choices) and the choice-1 'lifting' hold keep the gear only up to up_pt + hysteresis;
// past that the normal choice upshifts. A truck that is actually braking for the junction still holds its gear.
static void c_hc_junction(double t)   // 61 km/h in 9, pedal 0.17, a junction 70 m ahead whose speed is lower
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = 0.17f;
}
static void c_hc_lift(double t)   // 61 km/h in 9, full pedal (the takeover dwell holds 9), then lift 0.99 -> 0.17
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = t < 1.5 ? 0.99f : 0.17f;
}
static void c_hc_brake(double t)   // 61 km/h in 9, actually braking for the junction
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = 0;
    M.brake = 0.25f;
}

#include "sim_sweep.inc"

// ---- part 16: the ride's hunts (30-10, ds-hunt). Four hunts, three causes:
//  1. the cruise's force wish (force_mode 1) and the score disagreed: a set-speed step made the cruise ask more than the gear gives
//     (kickdown_by_wish dropped a gear) while the score's demand stayed f_hold and skipped straight back up (12->11->12->11, 11->12->11->13).
//  2. a coasting upshift under the deceleration assist's retarder, taken back by the assist's downshift a few seconds later
//     (12->11->12->10).
//  3. the fair hunt metric: a brake-then-pull (or a pedal press/lift) that reverses the shifts is the driver, not a hunt (9->7->9->7).
static void c_hcstep(double t)   // Scania (G25CM 14-speed), 44 t, 56 km/h in 12, cruise force_mode 1, set speed 90: the cruise's force wish
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    g_cf_set = 90;
    c_cforce(t);
}
static void c_hc_ret(double t)   // 20 t, 68 km/h in 11, the pedal off, a light retarder (the deceleration assist steps it): the truck slows
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = 0;
    M.ret = 1;
}
static int g_hk_lift; static float g_hk_to;   // 0 = hold the full pedal; > 0 = let go to g_hk_to after that many seconds
static float g_hk_dm_lo, g_hk_dm_hi;   // held flat: the demand share at ~60 and ~80 km/h (a held pedal must be the same demand, not a force that falls)
static float g_hk_dm_pre, g_hk_dm_post;   // lift: the demand share just before and just after the foot comes off (a clear lift must drop it)
static void c_hk_kick(double t)   // Scania, 20 t, 60 km/h in 12, the full wish pedal from the first tick (a kickdown): held flat, or lifted
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; g_hk_dm_lo = g_hk_dm_hi = g_hk_dm_pre = g_hk_dm_post = -1; }
    float p = g_hk_lift > 0 && t >= g_hk_lift ? g_hk_to : 1.0f;
    M.pedal = p; g_pl_ped = p;
    sim_pedal_wish();
    float dm = sim_demand(), kmh = M.v * 3.6f;
    if (g_hk_lift == 0) {
        if (g_hk_dm_lo < 0 && kmh <= 62.0f) g_hk_dm_lo = dm;
        if (g_hk_dm_hi < 0 && kmh >= 80.0f) g_hk_dm_hi = dm;
    } else {
        if (g_hk_dm_pre < 0 && t >= g_hk_lift - 0.5f) g_hk_dm_pre = dm;
        if (g_hk_dm_post < 0 && t >= g_hk_lift + 0.5f) g_hk_dm_post = dm;
    }
}
static int g_hc_first_up, g_hc_first_down;
static void hc_count(double from)   // the first upshift and downshift at or after `from` seconds (from the recorder), -1 = none
{
    g_hc_first_up = g_hc_first_down = -1;
    for (int i = 0; i < g_rec_n; i++) {
        double t = g_rec[i].t - g_pl_t0;
        if (t < from) continue;
        if (g_rec[i].b > g_rec[i].a && g_hc_first_up < 0) g_hc_first_up = (int)t;
        if (g_rec[i].b < g_rec[i].a && g_hc_first_down < 0) g_hc_first_down = (int)t;
    }
}
// part 17: a heavy US engine (2900 Nm, plateau from 1150, 12 speeds) pulling away with the torque pedal on (the wish), Eco, 15.5 t, to 70 km/h:
// the gear after each upshift at a calm (0.2), a middle (0.4) and a firm pedal (0.6). Printed: the shifts, the skips, the landing rpm of every upshift
static float r_pete_grade(double) { return 1.0f; }

// ---- part 18: the layered powertrain ([trueshift] gear_logic 1) against the old choice (0) in the simulator.
// Each scenario runs both, and prints shifts / skips / landing rpm / time to 70 / fuel / seconds of rpm outside the engine
// band / hunting. The old choice is the score choice (choice 1), which the layers replace; both drive the torque pedal's wish.
static double g_ly_out_s;   // seconds in gear with the engine rpm outside [floor, cap]
static void ly_track(double dt)
{
    if (M.gear >= 1 && !M.state) {
        const eng_bands *b = bands_now();
        float r = rpm_now();
        if (b->ok && (r < b->floor || r > b->cap)) g_ly_out_s += dt;
    }
}
struct ly_res { int shifts, skips, hunts, back, back_drv, end_gear; double t70; float land_rpm, mean_rpm; double fuel, out_s; };
static int g_ly_keep;   // 1: ly_row does not reset the session (what the plugin learned stays; part 18e "again")
static ly_res g_cr83[4];   // the cruise stand rows of part 18: 18 t flat, 18 t -1 %, 40 t flat, 40 t -1 %
static void c_ly83(double t) { hold_cc(83, t); }   // (the cruise stand: the game's cruise is on at 83, a controller and no driver's foot; the plant is driven through the pedal like the other rows)
static ly_res ly_row(setup su, void (*ctl)(double), int logic, int wish);
static ly_res ly_row_keep(setup su, void (*ctl)(double), int logic, int wish) { g_ly_keep = 1; ly_res r = ly_row(su, ctl, logic, wish); g_ly_keep = 0; return r; }   // the cruise stand at 83 km/h
static const char *g_ly_nm;   // the row being run (SC_LY=<substring>: only that row prints its shift log)
static ly_res ly_row(setup su, void (*ctl)(double), int logic, int wish)
{
    set_param("gear_logic", (float)logic);
    set_param("choice", 1.0f); set_param("choice_objective", 1.0f); set_param("kickdown_by_wish", 1.0f);
    g_pl_wish = wish;
    setup wash = {20000, 30, 6, 2}; wash.cvar = su.cvar;   // (as sc_do: a short drive first, so a run that ends in a crawl does not leak into the next)
    g_quiet = 1; run("wash", wash, c_hold80); g_quiet = 0;
    if (!g_ly_keep) sim_reset(RESET_HARD, "sim: next layers scenario");
    double fuel0 = g_fuel_raw;
    g_ly_out_s = 0;
    g_rec_on = 1; g_rec_n = 0; g_pl_t0 = t_now; g_pl_t70 = -1;
    g_tick_hook = ly_track;
    g_quiet = getenv("SC_VERBOSE") || (getenv("SC_LY") && g_ly_nm && logic == 1 && strstr(g_ly_nm, getenv("SC_LY"))) ? 0 : 1;
    run("layers", su, ctl);
    g_tick_hook = NULL;
    g_rec_on = 0;
    g_quiet = 0;
    ly_res r = {};
    double t_end = g_pl_t70 >= 0 ? g_pl_t0 + g_pl_t70 : 1e30;
    float land_sum = 0; int land_n = 0;
    for (int i = 0; i < g_rec_n; i++) {
        if (g_rec[i].t > t_end) break;
        r.shifts++;
        if (abs(g_rec[i].b - g_rec[i].a) > 1) r.skips++;
        if (g_rec[i].b > g_rec[i].a) { land_sum += vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f); land_n++; }
    }
    r.land_rpm = land_n ? land_sum / land_n : 0;
    r.hunts = hunts;
    r.back = rec_pairs(&r.back_drv);   // the one hunt definition (hunt_kind): a reversal at an unchanged foot within the window, real gears only; the driver's own reversals are counted apart
    r.t70 = g_pl_t70;
    r.fuel = fuel0 - g_fuel_raw;
    r.out_s = g_ly_out_s;
    r.end_gear = M.gear;
    r.mean_rpm = rpm_t > 0 ? (float)(rpm_sum / rpm_t) : 0;
    return r;
}

// ---- part 18b: the drive modes (Eco / Normal / Power) on the layers. Per truck and load: a calm cruise, a 4 % climb, and
// two pedal pulls; per mode the mean rpm above 50 km/h and the share of that time in each band (lug / eco / mid / power+temporary).
static double g_lm_band_s[8], g_lm_rpm_s, g_lm_t;
static struct { double rpm; int shifts, hunts; } g_lm_p6[3];   // 18b: Pete 35.6 t, cruise 80 on +6 % (no gear holds 80: the speed falls), per mode (18i-7 holds it to no hunt)
static struct { double rpm, fuel; int shifts, hunts; } g_lm_c4[3];   // 18b: DC16 18 t, cruise 80 on +4 %, per mode (18i-7 holds the modes to one economical gear)
static struct { double rpm, fuel; int shifts; } g_lm_c83[3];   // 18b: DC16 40 t, cruise 83 flat, per mode (18i-7 holds it to the economical gear)
static void lm_track(double dt)
{
    if (M.gear >= 1 && !M.state && M.v * 3.6f >= 50) {
        const eng_bands *b = bands_now();
        float r = rpm_now();
        if (b->ok) { g_lm_band_s[pt_band_of(b, r, gc_cross(b, M.gear, NG, M.v))] += dt; g_lm_rpm_s += r * dt; g_lm_t += dt; }
    }
}
static float r_four(double) { return 4.0f; }
static float r_six(double) { return 6.0f; }
static int g_lm_cruise_kmh = 83;
static void c_lm_cruise(double t) { hold_cc((float)g_lm_cruise_kmh, t); }   // (the game's cruise is on: a controller, no driver's foot; the plant is driven through the pedal)
static void lm_row(const char *name, const char *mode, setup su, void (*ctl)(double), int wish)
{
    static const char *only = getenv("SC_LM");   // SC_LM=<scenario substring>[:<mode>]: only these rows, verbose (the shift log)
    char key[160]; snprintf(key, sizeof key, "%s:%s", name, mode);
    if (only && !strstr(key, only)) return;
    set_param("gear_logic", 1.0f);
    set_param("choice", 1.0f); set_param("choice_objective", 1.0f); set_param("kickdown_by_wish", 1.0f);
    g_pl_wish = wish;
    g_fuel_raw = 1000.0; T.fuel = 1000.0f;
    setup wash = {20000, 30, 6, 2}; wash.cvar = su.cvar; wash.road = su.road;
    g_quiet = 1; run("wash", wash, c_hold80); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: next modes scenario");
    memset(g_lm_band_s, 0, sizeof g_lm_band_s); g_lm_rpm_s = g_lm_t = 0;
    g_rec_on = 1; g_rec_n = 0; g_pl_t0 = t_now; g_pl_t70 = -1;
    g_tick_hook = lm_track;
    g_quiet = getenv("SC_VERBOSE") || only ? 0 : 1;
    run("modes", su, ctl);
    g_tick_hook = NULL; g_rec_on = 0; g_quiet = 0;
    double t = g_lm_t;
    if (t < 1) { printf("  %-34s %-6s %8s\n", name, mode, "-"); return; }
    const double *s = g_lm_band_s;
    char t70s[12]; snprintf(t70s, sizeof t70s, g_pl_t70 < 0 ? "-" : "%.1f", g_pl_t70);
    if (!strcmp(name, "DC16 18 t, cruise 80 on +4 %")) { const int mi = !strcmp(mode, "Eco") ? 0 : !strcmp(mode, "Normal") ? 1 : 2; g_lm_c4[mi] = { g_lm_rpm_s / t, 1000.0 - g_fuel_raw, shifts, hunts }; }
    if (!strcmp(name, "Pete 35.6 t, cruise 80 on +6 %")) { const int mi = !strcmp(mode, "Eco") ? 0 : !strcmp(mode, "Normal") ? 1 : 2; g_lm_p6[mi] = { g_lm_rpm_s / t, shifts, hunts }; }
    if (!strcmp(name, "DC16 40 t, cruise 83 flat")) { const int mi = !strcmp(mode, "Eco") ? 0 : !strcmp(mode, "Normal") ? 1 : 2; g_lm_c83[mi] = { g_lm_rpm_s / t, 1000.0 - g_fuel_raw, shifts }; }
    printf("  %-34s %-6s %8.0f   %4.0f %4.0f %4.0f   %6d %6d   %5s %7.2f %7.0f\n", name, mode, g_lm_rpm_s / t,
           100 * s[PT_NOGO_LO + 0] / t + 100 * s[PT_LUG] / t, 100 * s[PT_ECO] / t,
           100 * (s[PT_POW] + s[PT_TMP] + s[PT_NOGO_HI]) / t, shifts, hunts, t70s, 1000.0 - g_fuel_raw, (double)M.s);
}
static void run_layers_modes()
{
    printf("\n=== SCORES part 18b: the drive modes on the layers (rpm and band shares above 50 km/h) ===\n");
    printf("\n  %-34s %-6s %8s   %4s %4s %4s   %6s %6s   %5s %7s %7s\n", "scenario", "mode", "mean rpm", "lug%", "eco%", "pow%", "shifts", "hunts", "t70", "fuel L", "dist m");
    struct Md { const char *n; float cvar; } mds[3] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    struct Tr { const char *n; int dc; float mass; } trs[] = { { "DC16 18 t", 1, 18000 }, { "DC16 40 t", 1, 40000 },
                                                              { "Pete 15.5 t", 0, 15500 }, { "Pete 35.6 t", 0, 35600 } };
    for (const Tr &tr : trs) {
        if (tr.dc) use_dc16(); else use_pete();
        for (int sc = 0; sc < 6; sc++) {
            for (int mi = 0; mi < 3; mi++) {
                char nm[96];
                setup su = {tr.mass, 83, sc_gear_for(83, 1100), 120, mds[mi].cvar, 1.0f};
                if (sc == 0) { snprintf(nm, sizeof nm, "%s, cruise 83 flat", tr.n); g_lm_cruise_kmh = 83; lm_row(nm, mds[mi].n, su, c_lm_cruise, 0); }
                else if (sc == 1) {
                    snprintf(nm, sizeof nm, "%s, cruise 80 on +4 %%", tr.n); g_lm_cruise_kmh = 80; su.road = r_four; su.v0 = 80; su.g0 = sc_gear_for(80, 1100);
                    lm_row(nm, mds[mi].n, su, c_lm_cruise, 0);
                } else if (sc == 4) {
                    snprintf(nm, sizeof nm, "%s, cruise 80 on +6 %%", tr.n); g_lm_cruise_kmh = 80; su.road = r_six; su.v0 = 80; su.g0 = sc_gear_for(80, 1100);
                    lm_row(nm, mds[mi].n, su, c_lm_cruise, 0);
                } else {
                    g_pl_ped = sc == 2 ? 0.3f : sc == 3 ? 0.6f : 1.0f;
                    setup pu = {tr.mass, 1, 1, 120, mds[mi].cvar, 1.0f};
                    snprintf(nm, sizeof nm, "%s, pull pedal %.1f", tr.n, g_pl_ped);
                    lm_row(nm, mds[mi].n, pu, c_pull, 1);
                }
            }
            printf("\n");
        }
    }
    g_pl_wish = 0;
}

// ---- part 18c: hunts from the ride. (A) a light pedal at low speed (ATS 22:44: 4>3>4 in 3.2 s at 14 km/h, pedal 0.13), (B) the cruise
// switching off into a pedal (ETS2 23:15: 6>7>6>7 in 9.5 s after the cruise went off, pedal 0 -> 0.46). Per truck, load and mode: the
// shifts and hunting pairs over the run; the total must be 0.
static float g_hb_kmh = 60, g_hb_ped = 0.46f, g_hb_t = 20;
static void c_hb(double t)
{
    if (t < g_hb_t) { M.wish_on = 0; hold(g_hb_kmh); return; }           // the cruise: the speed held, no wish
    g_pl_ped = g_hb_ped; M.pedal = g_hb_ped;                              // the cruise off, the pedal at 0.46: the torque pedal's wish
    sim_pedal_wish();
}
static int run_layers_hunts()
{
    printf("\n=== SCORES part 18c: hunts from the ride (A: a light pedal at low speed, B: the cruise off into a pedal) ===\n");
    printf("\n  %-40s %-6s %6s %6s\n", "scenario", "mode", "shifts", "a>b>a");
    struct Md { const char *n; float cvar; } mds[3] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    struct Tr { const char *n; int dc; float mass; } trs[] = { { "DC16 18 t", 1, 18000 }, { "DC16 40 t", 1, 40000 },
                                                              { "Pete 15.5 t", 0, 15500 }, { "Pete 35.6 t", 0, 35600 } };
    int total = 0, total_drv = 0;
    const char *only = getenv("SC_HB"), *onlym = getenv("SC_HBM") ? getenv("SC_HBM") : "";   // SC_HB=<name substring> [SC_HBM=<mode>]: only these rows (with SC_VERBOSE=1 the shift log)
    const bool wide = getenv("SC_WIDE") != NULL;   // SC_WIDE=1: the wide grid (many more speeds and pedals), for the hunt hunt
    for (const Tr &tr : trs) {
        if (tr.dc) use_dc16(); else use_pete();
        for (int mi = 0; mi < 3; mi++) {
            // A: pedals 0.08 / 0.13 / 0.2 from 10 and 14 km/h
            std::vector<float> aped = wide ? std::vector<float>{ 0.05f, 0.08f, 0.1f, 0.13f, 0.17f, 0.2f, 0.25f, 0.3f } : std::vector<float>{ 0.08f, 0.13f, 0.2f };
            std::vector<float> akmh = wide ? std::vector<float>{ 6, 8, 10, 12, 14, 17, 20, 25, 30 } : std::vector<float>{ 10, 14 };
            std::vector<float> bkmh = wide ? std::vector<float>{ 40, 50, 55, 65, 75, 85 } : std::vector<float>{ 55, 75 };
            std::vector<float> bped = wide ? std::vector<float>{ 0.2f, 0.3f, 0.4f, 0.46f, 0.55f, 0.7f, 0.85f } : std::vector<float>{ 0.3f, 0.46f, 0.7f };
            for (float ped : aped)
                for (float kmh : akmh) {
                    char nm[96]; snprintf(nm, sizeof nm, "%s, A pedal %.2f from %.0f km/h", tr.n, ped, kmh);
                    g_pl_ped = ped;
                    setup su = {tr.mass, kmh, sc_gear_for(kmh, 1000), 40, mds[mi].cvar, 1.0f};
                    if (only && !(strstr(nm, only) && strstr(mds[mi].n, onlym))) continue;
                    ly_res r = ly_row(su, c_pull, 1, 1);
                    printf("  %-40s %-6s %6d %6d%s\n", nm, mds[mi].n, r.shifts, r.back, r.back_drv ? "  (+ a reversal after the driver's foot moved)" : "");
                    total += r.back; total_drv += r.back_drv;
                }
            // B: the cruise at 55 / 75 km/h for 20 s, then the pedal at 0.3 / 0.46 / 0.7
            for (float kmh : bkmh)
                for (float ped : bped) {
                    char nm[96]; snprintf(nm, sizeof nm, "%s, B cruise %.0f then pedal %.2f", tr.n, kmh, ped);
                    g_hb_kmh = kmh; g_hb_ped = ped;
                    setup su = {tr.mass, kmh, sc_gear_for(kmh, 1100), 60, mds[mi].cvar, 1.0f};
                    if (only && !(strstr(nm, only) && strstr(mds[mi].n, onlym))) continue;
                    ly_res r = ly_row(su, c_hb, 1, 0);
                    printf("  %-40s %-6s %6d %6d%s\n", nm, mds[mi].n, r.shifts, r.back, r.back_drv ? "  (+ a reversal after the driver's foot moved)" : "");
                    if ((r.back || r.back_drv) && getenv("SC_HBP")) for (int i = 0; i < g_rec_n; i++) printf("      %7.1f  %d>%d  %.0f km/h  %.0f rpm  (%s)\n", g_rec[i].t, g_rec[i].a, g_rec[i].b, g_rec[i].kmh, g_rec[i].rpm, g_rec[i].why);
                    total += r.back; total_drv += r.back_drv;
                }
        }
    }
    printf("  hunts in total: %d (reversals after the driver's own foot change, printed apart and not counted: %d)\n", total, total_drv);
    g_pl_wish = 0;
    return total ? 1 : 0;
}

// ---- part 18d: a full-throttle pull into a climb that starts (ETS2 23:21:54: 16.5 t, Power, 1>4 landing at 1006 rpm, 4>7, then kickdown 7>6 at
// 1924 rpm as the grade went 4.2 -> 8.5 %; ATS 22:44:50: 5>7 at 1097 rpm, 1.7 -> 6.5 %, then 7>5>6). The route is known (navigation), the grade rises
// from 2 % to 7 % over the first 80..400 m. Per row: shifts, skips, downshifts (a>b, both real gears, after the start), the lowest rpm an upshift
// landed at (above 8 km/h), the speed and the distance after 40 s.
static float r_ramp27(double s) { return s < 80 ? 2.0f : s < 400 ? 2.0f + 5.0f * (float)(s - 80) / 320.0f : 7.0f; }
static float r_ramp39(double s) { return s < 60 ? 3.0f : s < 300 ? 3.0f + 6.0f * (float)(s - 60) / 240.0f : 9.0f; }
static float r_ramp15(double s) { return s < 150 ? 1.0f : s < 500 ? 1.0f + 4.0f * (float)(s - 150) / 350.0f : 5.0f; }
static void c_full40(double t) { c_pull(t); }
static int run_layers_climb()
{
    printf("\n=== SCORES part 18d: a full-throttle pull into a climb that starts (route known) ===\n");
    printf("\n  %-34s %-6s %6s %6s %6s %9s %8s %7s\n", "scenario", "mode", "shifts", "skips", "downs", "min land", "km/h@40", "m@40");
    struct Md { const char *n; float cvar; } mds[2] = { { "Normal", 3.0f }, { "Power", 1.6667f } };
    struct Tr { const char *n; int dc; float mass; } trs[] = { { "DC16 18 t", 1, 18000 }, { "DC16 40 t", 1, 40000 },
                                                              { "Pete 15.5 t", 0, 15500 }, { "Pete 35.6 t", 0, 35600 } };
    for (const Tr &tr : trs) {
        if (tr.dc) use_dc16(); else use_pete();
        for (int ri = 0; ri < 3; ri++)
            for (int mi = 0; mi < 2; mi++) {
                char nm[96]; snprintf(nm, sizeof nm, "%s, pedal 1.0, grade %s", tr.n, ri == 2 ? "3 -> 9 %" : ri ? "1 -> 5 %" : "2 -> 7 %");
                g_pl_ped = 1.0f;
                setup su = {tr.mass, 1, 1, 40, mds[mi].cvar, 1.0f};
                su.road = ri == 2 ? r_ramp39 : ri ? r_ramp15 : r_ramp27; su.route = 1;
                ly_res r = ly_row(su, c_full40, 1, 1);
                int downs = 0; float minland = 1e9f;
                for (int i = 0; i < g_rec_n; i++) {
                    if (g_rec[i].a > 0 && g_rec[i].b > 0 && g_rec[i].b < g_rec[i].a) downs++;
                    if (g_rec[i].b > g_rec[i].a && g_rec[i].kmh >= 8) { float lr = vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f); if (lr < minland) minland = lr; }
                }
                if (getenv("SC_HBP")) for (int i = 0; i < g_rec_n; i++) printf("      %7.1f  %d>%d  %.0f km/h  %.0f rpm  (%s)\n", g_rec[i].t, g_rec[i].a, g_rec[i].b, g_rec[i].kmh, g_rec[i].rpm, g_rec[i].why);
                printf("  %-34s %-6s %6d %6d %6d %9.0f %8.1f %7.0f\n", nm, mds[mi].n, r.shifts, r.skips, downs, minland > 1e8f ? 0.0f : minland, M.v * 3.6f, M.s);
            }
    }
    g_pl_wish = 0;
    return 0;
}

// ---- part 18e: the landing rpm of a shift at crawl speeds. The plant loses more speed in the shift than the plugin's model says (an extra
// deceleration in the gap and a gap that lasts longer than the engine info: the game's hook, the clutch) -- ETS2 41 t: decided 1>4 for a
// landing at 737 rpm, landed at 651, under the no-go. Per row: the upshifts above 3 km/h, how many landed under the lug floor (the no-go), the lowest
// landing rpm, the shifts and the time to 70 km/h. The first pull of a truck is the plugin learning what its shifts cost; "again" is the second.
static int run_layers_landing()
{
    printf("\n=== SCORES part 18e: landing rpm of upshifts at crawl speeds (plant: extra decel in the gap, gap longer than the plugin knows) ===\n");
    printf("\n  %-34s %-24s %7s %7s %9s %6s %5s  %6s %6s\n", "scenario", "plant", "upshifts", "< lug", "min land", "shifts", "t70", "err avg", "worst");   // err: landed rpm - the layers' estimate
    struct Tr { const char *n; int dc; float mass; } trs[] = { { "DC16 40 t", 1, 40000 }, { "Pete 35.6 t", 0, 35600 }, { "Pete 15.5 t", 0, 15500 } };
    struct Pl { const char *n; float drag, extra; } pls[] = { { "as modelled", 0, 0 }, { "0.15 m/s2 + 0.6 s", 0.15f, 0.6f }, { "0.3 m/s2 + 1.0 s", 0.3f, 1.0f } };
    for (const Tr &tr : trs) {
        if (tr.dc) use_dc16(); else use_pete();
        for (const Pl &pl : pls)
            for (float ped : { 0.5f, 1.0f }) {
                g_plant_shift_drag = pl.drag; g_plant_shift_extra = pl.extra;
                g_pl_ped = ped;
                setup su = {tr.mass, 1, 1, 90, 3, 1.0f};
                char nm[96]; snprintf(nm, sizeof nm, "%s, pull pedal %.1f", tr.n, ped);
                for (int pass = 0; pass < 2; pass++) {   // the second pass in the same session: what the first one learned
                    g_ly_nm = nm;
                    ly_res r = pass ? ly_row_keep(su, c_pull, 1, 1) : ly_row(su, c_pull, 1, 1);
                    const eng_bands *b = bands_now();
                    int ups = 0, under = 0, np = 0; float minl = 1e9f, worst = 0, errsum = 0;
                    for (int i = 0; i < g_rec_n; i++)
                        if (g_rec[i].a > 0 && g_rec[i].b > g_rec[i].a && g_rec[i].kmh >= 3) {
                            float lr = vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f);
                            ups++; if (lr < b->floor) under++; if (lr < minl) minl = lr;
                            if (g_rec[i].pred > 0) { float e = lr - g_rec[i].pred; np++; errsum += e; if (e < worst) worst = e; }
                        }
                    char ta[16]; snprintf(ta, sizeof ta, r.t70 < 0 ? "   -" : "%4.1f", r.t70);
                    printf("  %-34s %-24s %7d %7d %9.0f %6d %5s  %6.0f %6.0f\n", nm, pass ? (std::string(pl.n) + " again").c_str() : pl.n, ups, under, minl > 1e8f ? 0.0f : minl, r.shifts, ta, np ? errsum / np : 0.0f, worst);
                }
            }
    }
    g_plant_shift_drag = g_plant_shift_extra = 0;
    g_pl_wish = 0;
    return 0;
}

// ---- part 18f: release, demand and history. What the box does when the driver LETS GO after a lug pull (a fast and a slow release, on the flat
// and on a known +2 %), before a climb it knows about (the foot 25 s ahead), when a demand arrives out of a calm cruise (the surprise), on a heavy
// truck at a part load, on an empty one, and after a few sprints. These rows are the yardstick for the expected-time term (docs/powerband-change.md,
// round 3): they print what the box does today; the "must" lines of the plan are read off them. All modes, the drive modes as in 18b (cvar 10 / 3 / 1.6667).
// Per row: mean rpm (in gear above 50 km/h), shifts and hunts of the run, rel_shifts (shifts within 10 s after the release mark g_lr_t_rel; known25
// counts from 5 s before the foot), rel_land_band (the band of the gear 3 s after the mark), t70 (s to 70 km/h from a pull-away), t_resp (s from the
// demand mark g_lr_t_dem to +10 km/h), rpm_tail (mean rpm over the last 30 s) and the fuel. Mean rpm: in gear for the 30 s after the mark when the
// row has one, else above 50 km/h. The release mark of the slow release is the start of the ramp. The eco-roll is off in these rows (a lift at speed
// would roll in neutral: the gear logic is what is measured; the roll has its own rows). SC_LR=<name substring>[:<mode>]: only those rows, verbose.
static double g_lr_t0, g_lr_secs, g_lr_t_rel, g_lr_t_dem;   // run start (t_now), run length, the marks in seconds since the start (-1 = none)
static int g_lr_on;                                           // the controller has started the row (tracking runs from its first call)
static float g_lr_foot_m, g_lr_pre_s, g_lr_hold_kmh, g_lr_pull = 0.5f;          // known25: the foot of the climb (0 = none) and the seconds before the mark that count as the release; the flat rows' speed
static int g_lr_calm, g_lr_resp;                              // sprints: 1 = the calm control (no sprints); 1 = print t_resp
static double g_lr_rpm_s, g_lr_rpm_t, g_lr_tail_s, g_lr_tail_t, g_lr_lug_s, g_lr_lug_t, g_lr_t70, g_lr_t_resp;
static float g_lr_v_dem, g_lr_v200, g_lr_rel_kmh; static int g_lr_band3, g_lr_rel_lug;
static float r_step25(double s) { return s < 486 ? 0.0f : 5.0f; }   // flat for 486 m (70 km/h x 25 s), then 5 %

static void lr_start(double t)   // every controller's first call: the marks and the sums start over
{
    if (t > 0) return;
    g_lr_t0 = t_now; g_lr_on = 1; g_lr_t_rel = g_lr_t_dem = -1; g_lr_band3 = -1; g_lr_rel_lug = 0;
    g_lr_rpm_s = g_lr_rpm_t = g_lr_tail_s = g_lr_tail_t = g_lr_lug_s = g_lr_lug_t = 0;
    g_lr_t70 = g_lr_t_resp = -1; g_lr_v_dem = g_lr_v200 = g_lr_rel_kmh = -1;
}
static void lr_track(double dt)
{
    if (!g_lr_on) return;
    const double t = t_now - g_lr_t0;
    const float kmh = M.v * 3.6f;
    const bool in_gear = M.gear >= 1 && !M.state;
    const eng_bands *b = bands_now();
    int band = in_gear && b->ok ? pt_band_of(b, rpm_now(), gc_cross(b, M.gear, NG, M.v)) : -1;
    if (g_lr_t_rel >= 0 ? (in_gear && t >= g_lr_t_rel && t < g_lr_t_rel + 30) : (in_gear && kmh >= 50)) { g_lr_rpm_s += rpm_now() * dt; g_lr_rpm_t += dt; }
    if (in_gear && t >= g_lr_secs - 30) { g_lr_tail_s += rpm_now() * dt; g_lr_tail_t += dt; }
    if (g_lr_t70 < 0 && kmh >= 70) g_lr_t70 = t;
    if (g_lr_t_rel >= 0 && g_lr_band3 < 0 && t >= g_lr_t_rel + 3 && band >= 0) g_lr_band3 = band;
    if (g_lr_t_dem >= 0 && t >= g_lr_t_dem) {
        if (g_lr_v_dem < 0) g_lr_v_dem = M.v;
        else if (g_lr_t_resp < 0 && M.v >= g_lr_v_dem + 10.0f / 3.6f) g_lr_t_resp = t - g_lr_t_dem;
        if (t < g_lr_t_dem + 30 && band >= 0) { g_lr_lug_t += dt; if (band <= PT_LUG) g_lr_lug_s += dt; }   // (lug and the no-go under it)
    }
    if (g_lr_foot_m > 0 && g_lr_v200 < 0 && M.s >= g_lr_foot_m + 200) g_lr_v200 = kmh;
}
static void lr_hold(float kmh) { M.wish_on = 0; hold(kmh); }   // a held speed (the cruise's way, as 18b: no wish)

// A release (K2 step 1, rows audit 7.2: the release rows waited for a lug that came in 5 of 72 rows, a timer fallback let the rest 'release anyway', and they had no verdict): the owner's own situation, set up
// by the row. The pedal is FULL (the torque pedal's wish, from a standstill) until the EVENT, the first upshift of two gears or more whose landing is under P1 (a big skip into the lug band: the shift's
// landing rpm, the plant's, at the moment it completes); 1.0 s after it the pedal goes: at once (fast) or in a ramp to 0 over 1 s (slow: a longer ramp keeps pulling the truck out of the lug band).
// No timer: a row without the event never releases and says so (g_lr_rel_lug stays 0 and the note reads 'no situation').
static int g_lr_ev_seen, g_lr_ev_gear, g_lr_is_rel; static double g_lr_ev_t;   // g_lr_is_rel: the row is a release row (lr_release ran)
static void lr_release(double t, int slow)
{
    lr_start(t); g_lr_is_rel = 1;
    if (t <= 0) { g_lr_ev_seen = 0; g_lr_ev_gear = M.gear; g_lr_ev_t = -1; }
    if (g_lr_t_rel < 0) {
        const eng_bands *b = bands_now();
        for (int i = g_lr_ev_seen; i < g_rec_n; i++) {   // (the recorder logs a shift as it completes: the plant's landing is the rpm of the new gear now)
            if (g_rec[i].a > 0 && g_rec[i].b - g_rec[i].a >= 2 && b->ok && vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f) * 0.975f < b->p1) g_lr_ev_t = t;
        }
        g_lr_ev_seen = g_rec_n;
        if (g_lr_ev_t >= 0 && t >= g_lr_ev_t + 1.0) { g_lr_t_rel = t; g_lr_rel_kmh = M.v * 3.6f; g_lr_rel_lug = 1; }
    }
    float ped = g_lr_pull;
    if (g_lr_t_rel >= 0) ped = slow ? fmaxf(0.0f, g_lr_pull * (1.0f - (float)(t - g_lr_t_rel))) : 0.0f;
    g_pl_ped = ped;
    c_pull(t);
}
static void c_lr_fast(double t) { lr_release(t, 0); }
static void c_lr_slow(double t) { lr_release(t, 1); }
static void c_lr_fast_climb(double t) { lr_release(t, 0); }   // (the same driver; the setup has the +2 % road and the route)
static void c_lr_slow_climb(double t) { lr_release(t, 1); }
// A known climb: hold 70 on the flat; the mark is the foot (486 m, 25 s at 70 km/h). The setup has the route (r_step25).
static void c_lr_known25(double t)
{
    lr_start(t);
    if (g_lr_t_rel < 0 && M.s >= g_lr_foot_m) g_lr_t_rel = t;
    lr_hold(70);
}
// A surprise: hold 60 for 30 s with no route, then the pedal 0.6 (a wish) for 20 s; the demand mark is the 30 s.
static void c_lr_surprise(double t)
{
    lr_start(t);
    if (t < 30) { lr_hold(60); return; }
    g_lr_t_dem = 30;
    g_pl_ped = 0.6f;
    c_pull(t);
}
static void c_lr_40flat(double t) { lr_start(t); lr_hold(g_lr_hold_kmh); }   // a heavy truck at a part load (60 and 83 km/h)
static void c_lr_empty(double t) { lr_start(t); lr_hold(85); }               // an empty truck at 85
// Sprints from 50 km/h: three times the pedal 1.0 for 8 s and 0 for 6 s, then hold 60 for 60 s; the demand mark is the end of the sprints. The calm control
// (g_lr_calm) holds 60 throughout: the same truck with no history.
static void c_lr_sprints(double t)
{
    lr_start(t);
    g_lr_t_dem = 42;
    if (!g_lr_calm && t < 42) { g_pl_ped = fmod(t, 14.0) < 8 ? 1.0f : 0.0f; c_pull(t); }
    else lr_hold(60);
}

static int g_lr_sh_all, g_lr_hu_all, g_lr_sh_spr, g_lr_hu_spr, g_lr_sh_known, g_lr_hu_known, g_lr_downs_foot, g_lr_drv_all, g_lr_drv_spr;   // g_lr_drv_*: the reversals after the driver's own foot change (hunt_kind), which the shift bars of 18i-6 do not count   // the totals of the 18f rows (part 18i checks them against the rewrite's)
static void lr_row(const char *name, const char *mode, setup su, void (*ctl)(double), int wish)
{
    static const char *only = getenv("SC_LR");
    char key[160]; snprintf(key, sizeof key, "%s:%s", name, mode);
    if (only && !strstr(key, only)) return;
    set_param("gear_logic", 1.0f);
    set_param("choice", 1.0f); set_param("choice_objective", 1.0f); set_param("kickdown_by_wish", 1.0f);
    set_param("eco_roll", 0);   // the gear logic only: a lift at speed would roll in neutral (the roll has its own rows)
    g_pl_wish = wish;
    g_fuel_raw = 1000.0; T.fuel = 1000.0f;
    setup wash = {20000, 30, 6, 2}; wash.cvar = su.cvar; wash.road = su.road;
    g_quiet = 1; run("wash", wash, c_hold80); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: next release scenario");
    g_lr_on = 0; g_lr_secs = su.secs; g_lr_is_rel = 0;
    g_rec_on = 1; g_rec_n = 0; g_pl_t0 = t_now; g_pl_t70 = -1;
    g_tick_hook = lr_track;
    g_quiet = getenv("SC_VERBOSE") || only ? 0 : 1;
    run("release", su, ctl);
    g_tick_hook = NULL; g_rec_on = 0; g_quiet = 0; g_lr_on = 0;
    // the shifts around the release mark, from the recorder (absolute times)
    char rel_s[8] = "-", band_s[10] = "-", t70_s[8] = "-", tr_s[8] = "-", note[96] = "";
    if (g_lr_t_rel >= 0) {
        const double a = g_lr_t0 + g_lr_t_rel - g_lr_pre_s, z = g_lr_t0 + g_lr_t_rel + 10;
        int rel = 0, pre_dn = 0;
        for (int i = 0; i < g_rec_n; i++) {
            if (g_rec[i].t >= a && g_rec[i].t <= z) rel++;
            if (g_rec[i].t < a && g_rec[i].a > 0 && g_rec[i].b > 0 && g_rec[i].b < g_rec[i].a) pre_dn++;
        }
        snprintf(rel_s, sizeof rel_s, "%d", rel);
        if (g_lr_foot_m > 0) g_lr_downs_foot += pre_dn;
        if (g_lr_band3 >= 0) snprintf(band_s, sizeof band_s, "%s", gc_band_name(g_lr_band3));
        if (g_lr_foot_m > 0) snprintf(note, sizeof note, "foot %.1f s, downs before it %d, v200 %.1f km/h", g_lr_t_rel, pre_dn, g_lr_v200);
        else snprintf(note, sizeof note, "release %.1f s at %.0f km/h%s", g_lr_t_rel, g_lr_rel_kmh, g_lr_rel_lug ? " (1 s after a skip that landed in lug)" : "");
    } else if (g_lr_is_rel) snprintf(note, sizeof note, "NO SITUATION: no skip landed in lug, no release (the row does not count as a release)");
    if (g_lr_t70 >= 0 && su.v0 < 10) snprintf(t70_s, sizeof t70_s, "%.1f", g_lr_t70);
    if (g_lr_resp) snprintf(tr_s, sizeof tr_s, g_lr_t_resp >= 0 ? "%.1f" : "never", g_lr_t_resp);
    if (g_lr_t_dem >= 0 && g_lr_lug_t > 1 && !g_lr_resp) snprintf(note, sizeof note, "lug %.0f %% of the first 30 s after the sprints", 100 * g_lr_lug_s / g_lr_lug_t);
    g_lr_sh_all += shifts; g_lr_hu_all += hunts; g_lr_drv_all += hunts_drv;
    if (strstr(name, "sprint")) { g_lr_sh_spr += shifts; g_lr_hu_spr += hunts; g_lr_drv_spr += hunts_drv; }
    if (strstr(name, "known")) { g_lr_sh_known += shifts; g_lr_hu_known += hunts; }
    printf("  %-30s %-6s %8.0f %6d %5d %6s %8s %5s %6s %8.0f %7.2f  %s\n", name, mode, g_lr_rpm_t > 1 ? g_lr_rpm_s / g_lr_rpm_t : 0.0, shifts, hunts, rel_s,
           band_s, t70_s, tr_s, g_lr_tail_t > 1 ? g_lr_tail_s / g_lr_tail_t : 0.0, 1000.0 - g_fuel_raw, note);
}
static void run_layers_release()
{
    g_lr_sh_all = g_lr_hu_all = g_lr_sh_spr = g_lr_hu_spr = g_lr_sh_known = g_lr_hu_known = g_lr_downs_foot = g_lr_drv_all = g_lr_drv_spr = 0;
    printf("\n=== SCORES part 18f: release, demand and history (what the box does when the driver lets go, before a known climb, on a surprise) ===\n");
    printf("\n  %-30s %-6s %8s %6s %5s %6s %8s %5s %6s %8s %7s  %s\n", "scenario", "mode", "mean rpm", "shifts", "hunts", "rel_sh", "rel_band", "t70", "t_resp", "rpm_tail", "fuel L", "note");
    struct Md { const char *n; float cvar; } mds[3] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    struct Tr { const char *n; int kind; float mass; };   // kind: 0 DC16, 1 Pete, 2 ATO
    auto use = [](int kind) { if (kind == 0) use_dc16(); else if (kind == 1) use_pete(); else use_ato(); };
    const Tr rel_trs[] = { { "DC16 18 t", 0, 18000 }, { "Pete 15.5 t", 1, 15500 }, { "ATO 18 t", 2, 18000 } };
    const Tr heavy[] = { { "DC16 40 t", 0, 40000 }, { "Pete 35.6 t", 1, 35600 } };
    const Tr all[] = { { "DC16 9 t", 0, 9000 }, { "DC16 18 t", 0, 18000 }, { "DC16 40 t", 0, 40000 }, { "Pete 15.5 t", 1, 15500 }, { "Pete 35.6 t", 1, 35600 },
                       { "ATO 9 t", 2, 9000 }, { "ATO 18 t", 2, 18000 } };
    char nm[96];
    g_lr_foot_m = 0; g_lr_pre_s = 0; g_lr_calm = 0; g_lr_resp = 0;

    // 18f-1..4: the release after a full-pedal skip into lug, fast and slow, flat and on a known +2 % (K2 step 1: one pull, the full pedal; the event fires the release, lr_release)
    for (int var = 0; var < 4; var++)
        for (float pull : { 1.0f }) {
            const int slow = var & 1, climb = var >> 1;
            void (*ctl)(double) = slow ? (climb ? c_lr_slow_climb : c_lr_slow) : (climb ? c_lr_fast_climb : c_lr_fast);
            g_lr_pull = pull;
            for (const Tr &tr : rel_trs) {
                use(tr.kind);
                for (int mi = 0; mi < 3; mi++) {
                    snprintf(nm, sizeof nm, "%s, %s %s, pull %.1f", tr.n, slow ? "slow" : "fast", climb ? "+2 %" : "flat", pull);
                    setup su = {tr.mass, 1, 1, 45, mds[mi].cvar, 1.0f};
                    if (climb) { su.road = r_two; su.route = 1; }
                    lr_row(nm, mds[mi].n, su, ctl, 1);
                }
                printf("\n");
            }
        }
    g_lr_pull = 0.5f;
    // 18f-known25: hold 70 on the flat, a 5 % climb that starts at 486 m (the route is known): the foot is the mark, counted from 5 s before it
    g_lr_foot_m = 486; g_lr_pre_s = 5;
    for (const Tr &tr : heavy) {
        use(tr.kind);
        for (int mi = 0; mi < 3; mi++) {
            snprintf(nm, sizeof nm, "%s, known climb in 25 s", tr.n);
            setup su = {tr.mass, 70, sc_gear_for(70, 1100), 60, mds[mi].cvar, 1.0f};
            su.road = r_step25; su.route = 1;
            lr_row(nm, mds[mi].n, su, c_lr_known25, 0);
        }
        printf("\n");
    }
    g_lr_foot_m = 0; g_lr_pre_s = 0;
    // 18f-surprise: hold 60 for 30 s, no route, then the pedal 0.6 for 20 s: the seconds to +10 km/h
    g_lr_resp = 1;
    for (const Tr &tr : all) {
        use(tr.kind);
        for (int mi = 0; mi < 3; mi++) {
            snprintf(nm, sizeof nm, "%s, surprise 60 then 0.6", tr.n);
            setup su = {tr.mass, 60, sc_gear_for(60, 1100), 50, mds[mi].cvar, 1.0f};
            lr_row(nm, mds[mi].n, su, c_lr_surprise, 1);
        }
        printf("\n");
    }
    g_lr_resp = 0;
    // 18f-40flat: the DC16 at 40 t holding 60 and 83 km/h for 60 s
    use_dc16();
    for (float kmh : { 60.0f, 83.0f }) {
        g_lr_hold_kmh = kmh;
        for (int mi = 0; mi < 3; mi++) {
            snprintf(nm, sizeof nm, "DC16 40 t, hold %.0f", kmh);
            setup su = {40000, kmh, sc_gear_for(kmh, 1100), 60, mds[mi].cvar, 1.0f};
            lr_row(nm, mds[mi].n, su, c_lr_40flat, 0);
        }
        printf("\n");
    }
    // 18f-empty: an empty tractor (9 t) holding 85 km/h for 60 s
    for (const Tr &tr : all) {
        if (tr.mass != 9000) continue;
        use(tr.kind);
        for (int mi = 0; mi < 3; mi++) {
            snprintf(nm, sizeof nm, "%s, hold 85", tr.n);
            setup su = {tr.mass, 85, sc_gear_for(85, 1100), 60, mds[mi].cvar, 1.0f};
            lr_row(nm, mds[mi].n, su, c_lr_empty, 0);
        }
        printf("\n");
    }
    // 18f-sprints: three sprints from 50 km/h, then hold 60 for 60 s; against the same truck holding 60 with no history (calm)
    for (const Tr &tr : rel_trs) {
        use(tr.kind);
        for (int calm = 0; calm < 2; calm++) {
            g_lr_calm = calm;
            for (int mi = 0; mi < 3; mi++) {
                snprintf(nm, sizeof nm, "%s, %s", tr.n, calm ? "hold 60, no sprints" : "3 sprints, hold 60");
                setup su = {tr.mass, 50, sc_gear_for(50, 1100), 102, mds[mi].cvar, 1.0f};
                lr_row(nm, mds[mi].n, su, c_lr_sprints, 1);
            }
        }
        g_lr_calm = 0;
        printf("\n");
    }
    g_pl_wish = 0;
}

// ---- part 18g: the gearbox does not read the route's geometry (WP-R C4; owner 02-11: "the box is its own thing", a junction, a bend or a roundabout
// is the speed choosers' business and reaches the gearbox only through the wish). The same drive on a flat road must give the same gears without a
// route, with a route and no junction, and with a route and a junction ahead. The drives are the ones the junction hold and the braking floor
// of the old rules acted on: a lift towards a junction, a gentle pedal towards one, braking for one. Two comparisons per drive. MUST: a junction
// ahead changes nothing (the rules). WANT: a flat route changes nothing against no route (the layers read the road ahead and its certainty; a
// difference there is the layers' term, package B, and is printed, not failed).
struct gr_shift { double t; int a, b; char why[28]; };
struct gr_seq { int n; gr_shift s[REC_MAX]; };
static int g_gr_failed;
static void gr_save(gr_seq *q)
{
    q->n = g_rec_n;
    for (int i = 0; i < g_rec_n; i++) { q->s[i].t = g_rec[i].t - g_pl_t0; q->s[i].a = g_rec[i].a; q->s[i].b = g_rec[i].b; snprintf(q->s[i].why, sizeof q->s[i].why, "%s", g_rec[i].why); }
}
static bool gr_same(const gr_seq &x, const gr_seq &y)
{
    if (x.n != y.n) return false;
    for (int i = 0; i < x.n; i++) if (x.s[i].a != y.s[i].a || x.s[i].b != y.s[i].b || fabs(x.s[i].t - y.s[i].t) > 0.05) return false;
    return true;
}
static void gr_print(const char *tag, const gr_seq &q)
{
    printf("        %-28s %2d shifts:", tag, q.n);
    for (int i = 0; i < q.n && i < 8; i++) printf(" %d>%d@%.1fs(%s)", q.s[i].a, q.s[i].b, q.s[i].t, q.s[i].why);
    printf("%s\n", q.n > 8 ? " ..." : "");
}
static void c_gr_lift(double t) { M.brake = 0; if (t < 10) hold(70); else if (t < 14) M.pedal = 0; else M.pedal = 1.0f; }
static void c_gr_gentle(double) { M.brake = 0; M.pedal = 0.17f; }
static void c_gr_brake(double) { M.pedal = 0; M.brake = 0.25f; }
static void run_layers_route()
{
    printf("\n=== SCORES part 18g: the gears do not depend on the route's geometry (no route, a route without a junction, a route with a junction ahead; a flat road) ===\n");
    struct { const char *name; void (*use)(); float mass, kmh; int g0; double secs; void (*ctl)(double); double jn; } rows[] = {
        { "DC16 18 t, 70 km/h, lift 10-14 s, pedal 1.0 (junction 254 m)", use_dc16, 18000, 70, 12, 25, c_gr_lift, 254 },
        { "DC16 40 t, 60 km/h, pedal 0.17 (junction 200 m)", use_dc16, 40000, 60, 11, 25, c_gr_gentle, 200 },
        { "DC16 40 t, braking 0.25 from 80 km/h (junction 250 m)", use_dc16, 40000, 80, 12, 25, c_gr_brake, 250 },
        { "Pete 35.6 t, 70 km/h, lift 10-14 s, pedal 1.0 (junction 254 m)", use_pete, 35600, 70, 8, 25, c_gr_lift, 254 },
    };
    static const struct { const char *n; float cvar; } modes[] = { { "Normal", 3.0f }, { "Eco", 10.0f } };
    int same = 0, total = 0, must_ok = 0, must_n = 0;
    for (const auto &r : rows)
        for (const auto &m : modes) {
            static gr_seq q[3];
            for (int v = 0; v < 3; v++) {
                r.use();
                g_fuel_raw = 1000.0; T.fuel = 1000.0f;
                setup su = {r.mass, r.kmh, r.g0, r.secs, m.cvar, 1.0f, 0, -1, -1, r_flat, v ? 1 : 0, v == 2 ? r.jn : 0.0};
                char nm[160]; snprintf(nm, sizeof nm, "%s %s", r.name, m.n); g_ly_nm = nm;
                ly_row(su, r.ctl, 1, 1);
                gr_save(&q[v]);
            }
            printf("    %s, %s\n", r.name, m.n);
            gr_print("no route", q[0]); gr_print("route, no junction", q[1]); gr_print("route, junction ahead", q[2]);
            const bool a = gr_same(q[0], q[1]), b = gr_same(q[1], q[2]);
            printf("        %s: a junction ahead changes no gear (route without junction = route with junction);  %s: a flat route changes no gear (no route = route without junction)\n",
                   b ? "ok  " : "FAIL", a ? "ok  " : "want");
            total += 2; same += a + b; must_ok += b; must_n++;
            if (!b) g_gr_failed = 1;
        }
    printf("    junction ahead changes no gear in %d of %d drives (must: all); %d of %d comparisons give the same gears\n", must_ok, must_n, same, total);
}

// ---- part 18h: a braking downshift must not land where the next tick needs an upshift (owner's ride 02-11: braking downshifts landed at 1840-2142 rpm of a
// 2200 limit and the rev-limit rule shifted back up, the hunts 8>5>2>3>1 at 904 s and 9>6>9 at 1448 s on -7 %). The truck is the DC16 at 41 t.
// A "flip back" is a downshift of ours (braking, engine brake, assist) followed within 3 s by an upshift of the rev limit or the wrong-gear rule.
static float g_bk_brake = 0.4f, g_bk_to = 8.0f;
static double g_bk_ret = -1;   // when the controller gives the pedal back (t_now): everything before it is a lift with the brakes, the engine brake or the descent in charge
static void c_bk_stop(double t)   // 31 km/h, brake down to g_bk_to km/h, then the foot off both pedals for 3 s, then pedal 0.3
{
    if (M.v * 3.6f > g_bk_to && t < 20) { M.pedal = 0; M.brake = g_bk_brake; }
    else if (t < 20) { M.pedal = 0; M.brake = 0; }
    else { if (g_bk_ret < 0) g_bk_ret = t_now; M.pedal = 0.3f; M.brake = 0; }
}
static float r_m7(double) { return -7.0f; }
static void c_bk_descent(double t)   // -7 % at 45 km/h, pedal 0 and the service brake holding the speed for 12 s, then both off and pedal 0.99
{
    if (t < 12) { M.pedal = 0; M.brake = fminf(0.6f, fmaxf(0.0f, (M.v - 45.0f / 3.6f) * 0.5f + 0.12f)); }
    else { if (g_bk_ret < 0) g_bk_ret = t_now; M.brake = 0; M.pedal = 0.99f; }
}
static void c_bk_eb(double t)   // -7 %, the engine brake on and the service brake holding the speed for 20 s, then the engine brake out (the pedal cuts it) and pedal 0.99 (late enough for the braking downshifts to be done: a pedal press that lands during a pending braking downshift is the driver changing his mind, not a landing)
{
    if (t < 20) { M.pedal = 0; M.ebrake = 1; M.brake = fminf(0.5f, fmaxf(0.0f, (M.v - g_bk_to / 3.6f) * 0.1f)); }
    else { if (g_bk_ret < 0) g_bk_ret = t_now; M.ebrake = 0; M.brake = 0; M.pedal = 0.99f; }
}
static void use_bk_truck()   // the DC16 with a game 'low' range that reaches up to 92 % of the limiter (a constructed engine: the landings of the owner's ride, 1840-2142 of a 2200 limit, say his target sits that high)
{
    use_dc16();
    RANGES[1][1] = 1850;
}
static void use_bk_truck_eb()   // the same with the engine brake's range reaching up to 98 % of the limiter
{
    use_bk_truck();
    RANGES[0][1] = 1960;
}
struct bk_res { int shifts, brake_dn, flips, up_lim, layers_off; float land_max, land_frac; };   // layers_off: shifts the layers made while the pedal was off with the brakes in charge
static bk_res bk_count()
{
    bk_res r = {};
    r.shifts = g_rec_n;
    for (int i = 0; i < g_rec_n; i++) {
        const bool dn = g_rec[i].b < g_rec[i].a && g_rec[i].b > 0;
        const bool brk = !strncmp(g_rec[i].why, "braking", 7) || !strncmp(g_rec[i].why, "engine brake", 12) || !strncmp(g_rec[i].why, "braking assist", 14);
        if (dn && brk) {
            r.brake_dn++;
            float land = vehicle_engine_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f);
            if (land > r.land_max) { r.land_max = land; r.land_frac = land / V.eng.rpm_limit; }
            if (i + 1 < g_rec_n && g_rec[i + 1].b > g_rec[i + 1].a && g_rec[i + 1].t - g_rec[i].t < 3.0 &&
                (!strncmp(g_rec[i + 1].why, "rev limit", 9) || !strncmp(g_rec[i + 1].why, "wrong gear", 10))) r.flips++;
        }
        if (g_rec[i].b > g_rec[i].a && (!strncmp(g_rec[i].why, "rev limit", 9) || !strncmp(g_rec[i].why, "wrong gear", 10))) r.up_lim++;
        if (!strncmp(g_rec[i].why, "layers", 6) && (g_bk_ret < 0 || g_rec[i].t < g_bk_ret)) r.layers_off++;
    }
    return r;
}
static int g_bk_failed, g_bk_layers_off, g_bk_rows;   // D4: the layers never shift a lift that the brakes, the engine brake or the descent control own
static void run_layers_braking()
{
    printf("\n=== SCORES part 18h: a braking downshift lands where the next tick does not need an upshift (DC16, 41 t) ===\n");
    printf("  %-44s %-6s %6s %8s %6s %9s %7s\n", "scenario", "mode", "shifts", "brk down", "flips", "limit up", "land %");
    static const struct { const char *n; float kmh; int g0; double secs; void (*ctl)(double); float (*road)(double); float brake, to; } rows[] = {
        { "brake 0.4 from 31 km/h to 8, then 0.3", 31, 9, 30, c_bk_stop, r_flat, 0.4f, 8 },
        { "brake 0.3 from 31 km/h to 14, then 0.3", 31, 9, 30, c_bk_stop, r_flat, 0.3f, 14 },
        { "brake 0.5 from 31 km/h to 8, then 0.3", 31, 9, 30, c_bk_stop, r_flat, 0.5f, 8 },
        { "brake 0.5 from 31 km/h to 14, then 0.3", 31, 9, 30, c_bk_stop, r_flat, 0.5f, 14 },
        { "brake 0.3 from 45 km/h to 14, then 0.3", 45, 9, 30, c_bk_stop, r_flat, 0.3f, 14 },
        { "brake 0.8 from 45 km/h to 20, then 0.3", 45, 9, 30, c_bk_stop, r_flat, 0.8f, 20 },
        { "-7 % at 45 km/h in 13, brake to hold, 0.99 at 12 s", 45, 13, 30, c_bk_descent, r_m7, 0, 0 },
        { "-7 % at 45 km/h in 11, brake to hold, 0.99 at 12 s", 45, 11, 30, c_bk_descent, r_m7, 0, 0 },
    };
    static const struct { const char *n; float cvar; } modes[] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    for (const auto &r : rows)
        for (const auto &m : modes) {
            use_bk_truck();
            g_bk_brake = r.brake; g_bk_to = r.to; g_bk_ret = -1;
            g_fuel_raw = 1000.0; T.fuel = 1000.0f;
            setup su = {41000, r.kmh, r.g0, r.secs, m.cvar, 1.0f, 0, -1, -1, r.road, 0, 0.0};
            char nm[160]; snprintf(nm, sizeof nm, "%s %s", r.n, m.n); g_ly_nm = nm;
            ly_row(su, r.ctl, 1, 1);
            bk_res k = bk_count();
            printf("  %-44s %-6s %6d %8d %6d %9d %6.0f%%\n", r.n, m.n, k.shifts, k.brake_dn, k.flips, k.up_lim, 100 * k.land_frac);
            if (k.flips) g_bk_failed = 1;
            g_bk_layers_off += k.layers_off; g_bk_rows++;
        }
    // the descent control / engine brake path: the rule that lands on the middle of the engine brake's range (up to its top) with the game's toggle on, the
    // pedal then to 0.99 (the engine brake drops out and the rev-limit rule falls from 99 % to 96 % of the limit)
    for (float kmh : { 40.0f, 44.0f, 48.0f, 52.0f, 56.0f, 60.0f }) {
        use_bk_truck_eb();
        g_bk_to = kmh; g_bk_ret = -1;
        g_fuel_raw = 1000.0; T.fuel = 1000.0f;
        setup su = {41000, kmh, 13, 30, 3.0f, 1.0f, 0, 1, -1, r_m7, 0, 0.0};
        char nm[160]; snprintf(nm, sizeof nm, "-7 %% at %.0f km/h in 13, engine brake (full), 0.99 at 20 s", kmh); g_ly_nm = nm;
        ly_row(su, c_bk_eb, 1, 1);
        bk_res k = bk_count();
        printf("  %-44s %-6s %6d %8d %6d %9d %6.0f%%\n", nm, "Normal", k.shifts, k.brake_dn, k.flips, k.up_lim, 100 * k.land_frac);
        if (k.flips) g_bk_failed = 1;
        g_bk_layers_off += k.layers_off; g_bk_rows++;
    }
    // D4 (WP-D, plan D4): a lift with the brakes, the engine brake lever or the descent control in charge never reaches the layers' pedal-off model (gear_layers returns before
    // it: the descent hold, the deceleration assist, the brake channel, f_res <= 0): the gears in those windows are the braking rules', as before the lift model existed.
    printf("  D4: %d shifts by the layers while the pedal was off with the brakes, the engine brake or the descent in charge, in %d rows (must: 0)\n", g_bk_layers_off, g_bk_rows);
    if (g_bk_layers_off) g_bk_failed = 1;
}

// ---- part 18j: pedal off on a descent (WP-E 4, owner point 19.4). The engine's drag is what holds the speed: the box takes the highest gear whose drag is at least the push of the
// grade (inside the bands up to the crossover, never the temporary band), a downshift only where the gear we are in does not hold, and nothing up. Reported, not judged: the speed the
// truck ends at and tops out at over 40 s, the shifts, the rpm it ran at (the noise the owner hears). The brakes are not in the sim row (the truck coasts): the speed that is
// held is the brake use that is saved. The numbers before the pick (the gear of B's hold stays) are in the commit that made it.
static double g_ds_vmax, g_ds_rpmmax;
static float r_m3(double) { return -3.0f; }
static float r_m5(double) { return -5.0f; }
static void c_ds(double t)
{
    if (t < 0.1) { g_ds_vmax = 0; g_ds_rpmmax = 0; }
    M.pedal = 0; M.brake = 0;
    if (M.v > g_ds_vmax) g_ds_vmax = M.v;
    if (!M.state && M.gear > 0 && rpm_now() > g_ds_rpmmax) g_ds_rpmmax = rpm_now();
}
static void run_layers_descent()
{
    printf("\n=== SCORES part 18j: pedal off on a descent, the gear whose engine drag about holds the speed (DC16, coasting 40 s, no brakes in the sim) ===\n");
    printf("  %-30s %6s %9s %8s %8s %9s %9s\n", "scenario", "start", "end km/h", "max km/h", "shifts", "mean rpm", "max rpm");
    use_dc16();
    for (float mass : { 18000.0f, 40000.0f })
        for (int gr = 0; gr < 2; gr++)
            for (float kmh : { 50.0f, 70.0f }) {
                setup su = { mass, kmh, 11, 40, 3.0f, 0.5f };
                su.road = gr ? r_m5 : r_m3;
                char nm[96]; snprintf(nm, sizeof nm, "%.0f t, %s, from %.0f km/h", mass / 1000, gr ? "-5 %" : "-3 %", kmh);
                g_ly_nm = nm;
                ly_res r = ly_row(su, c_ds, 1, 1);
                printf("  %-30s %6.0f %9.1f %8.1f %8d %9.0f %9.0f\n", nm, kmh, M.v * 3.6f, g_ds_vmax * 3.6f, r.shifts, r.mean_rpm, g_ds_rpmmax);
            }
}

// ---- part 18l: the brake channel's wish for engine drag, turned into a gear (WP-BR). The channel posts "N more drag" to the gear arbiter (core/gearreq.h, GSO_BRAKE) when every cheaper
// stage is full and the service brake still helps; the gearbox picks the gear: a lower one whose revs sit inside the bands up to its crossover (the owner's noise rule), only after the
// pedal has been off for a shift and its latency (a lift is no surprise) and only if the shift pays (the drag it adds for as long as the need lasts against the drag the shift's gap
// takes away). Rows, DC16, the pedal off, a flat road, no route (the need lasts "as long again as the wish has stood"):
//   L1 a sustained wish of 20 kN from 3 s on: a downshift by the arbiter's braking road ("braking assist") that lands at or under the crossover of its gear, none before the lift has lasted
//      a shift and its latency, only downshifts, none within 3 s of another (the standing wish wants the next gear down again as the truck slows);
//   L2 the same wish for 0.6 s only: no shift (a need that ends before the shift pays never gets one);
//   L3 no wish: no shift from this rule (the coast of the flat road makes none of its own).
static int g_dr_failed;
static double g_dr_post_t, g_dr_clear_t = 1e9; static float g_dr_n = 20000;
static void c_drag(double t)
{
    M.pedal = 0; M.brake = 0;
    if (t >= g_dr_post_t && t < g_dr_clear_t) { const gear_stand w = { g_dr_n, GSK_SUSTAINED, 0, 0.0 }; gear_stand_post(GSO_BRAKE, &w); }
    else gear_stand_clear(GSO_BRAKE);
}
static void run_layers_drag()
{
    printf("\n=== SCORES part 18l: the brake channel's wish for engine drag becomes a gear (DC16, the pedal off, flat, no route) ===\n");
    use_dc16();
    struct Row { const char *nm; double post, clear; float n; float mass; int kmh; } rows[] = {
        { "L1 20 kN wished from 3 s, 18 t from 60 km/h", 3.0, 1e9, 20000, 18000, 60 },
        { "L1 20 kN wished from 3 s, 40 t from 60 km/h", 3.0, 1e9, 20000, 40000, 60 },
        { "L2 the same wish for 0.6 s only, 18 t", 3.0, 3.6, 20000, 18000, 60 },
        { "L3 no wish, 18 t", 1e9, 1e9, 0, 18000, 60 } };
    for (const Row &rw : rows) {
        g_dr_post_t = rw.post; g_dr_clear_t = rw.clear; g_dr_n = rw.n;
        setup su = { rw.mass, (float)rw.kmh, sc_gear_for(rw.kmh, 1100), 25, 3.0f, 0.5f };
        su.road = r_flat0;
        g_ly_nm = rw.nm;
        ly_res r = ly_row(su, c_drag, 1, 1);
        gear_stand_clear(GSO_BRAKE);
        int by_wish = 0, over = 0; double first = -1; char seq[160] = ""; size_t k = 0;
        for (int i = 0; i < g_rec_n; i++) {
            if (!strncmp(g_rec[i].why, "braking assist", 14)) {
                by_wish++;
                if (first < 0) first = g_rec[i].t - g_pl_t0;
                const eng_bands *b = bands_now();
                if (b->ok && g_rec[i].b > 0 && vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f) > 1.04f * gc_cross(b, g_rec[i].b, (int)T.fwd_gears, g_rec[i].kmh / 3.6f)) {   // (the line's speed is the one of the decision; the truck loses ~2 km/h of 50 in the shift's 1.2 s, ~4 % of the revs, which the pick counts and this check allows)
                    over++;
                    if (getenv("DR_DEBUG")) printf("      over: %d>%d at %.1f km/h, %.0f rpm against the crossover %.0f (the line says %.0f rpm)\n", g_rec[i].a, g_rec[i].b, g_rec[i].kmh, vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f), gc_cross(b, g_rec[i].b, (int)T.fwd_gears, g_rec[i].kmh / 3.6f), g_rec[i].rpm);
                }
            }
            if (k < sizeof seq - 40) k += (size_t)snprintf(seq + k, sizeof seq - k, "%s%d>%d@%.1fs(%s)", k ? " " : "", g_rec[i].a, g_rec[i].b, g_rec[i].t - g_pl_t0, g_rec[i].why);
        }
        const bool l1 = rw.clear > 1e8 && rw.n > 0, l2 = rw.n > 0 && rw.clear < 1e8;
        bool bad = false; const char *why = "";
        // (the wish stands the whole run: as the truck slows and its revs fall the next gear down is wanted again; every shift goes down, none follows another within 3 s = twice a shift and
        // its latency, and each lands under the crossover)
        bool up = false, close = false;
        for (int i = 0; i < g_rec_n; i++) { if (g_rec[i].b > g_rec[i].a) up = true; if (i && g_rec[i].t - g_rec[i - 1].t < 3.0) close = true; }
        if (l1 && by_wish < 1) { bad = true; why = "the wish made no downshift"; }
        else if (l1 && (up || close)) { bad = true; why = up ? "a shift up in a run with a wish for drag" : "two shifts within 3 s"; }
        else if (l1 && first < rw.post + 1.0) { bad = true; why = "a shift before the lift had lasted a shift and its latency"; }
        else if (l1 && over) { bad = true; why = "a downshift landed over the crossover of its gear"; }
        else if ((l2 || rw.n == 0) && by_wish) { bad = true; why = l2 ? "a need that ended before the shift pays got a shift" : "a shift by the wish with no wish"; }
        printf("  %-48s braking-assist shifts %d (first at %.1f s, %d over the crossover); all shifts: %s  %s%s\n", rw.nm, by_wish, first, over, seq[0] ? seq : "none", bad ? "FAIL: " : "ok", why);
        if (bad) g_dr_failed = 1;
        (void)r;
    }
}

#include "sim_layers_b.inc"

// ---- part 18k: the grade assist ([pedal] grade_assist, P6.3). A climb at a fixed pedal: the pedal 0.4 from 50 km/h on +4 % for 60 s, the speed it ends at, the shifts, the gear and the
// rpm, at grade_assist 0 / 0.5 / 0.75 / 1 (keuzes 16: where the throttle saturates and the gear the box takes). The assist is a REMAP of the pedal's range: base = min(c x m g sin(grade), F_env),
// level = base + s x (F_env - base): the bottom of the pedal range goes up on a climb, the top never (a full pedal is the full truck), and on the flat nothing changes. Judged (must): at the
// state the run ends in, a full pedal is gas 1.0 and a pedal of 0 is gas 0 at every assist, on +4 % and on +6 %; and the assist on the flat is the identical run (every assist, one row).
// Reported: the speed, the shifts, the gear, and at the START of the climb (50 km/h in the gear the box starts it in) the throttle at pedal 0.4 and the pedal at which the throttle first reaches 1.
static int g_as_failed;
static float r_p4(double) { return 4.0f; }
static float r_p6(double) { return 6.0f; }
static void set_pedal_param(const char *key, float v) { for (int i = 0; i < m_pedal.n_params; i++) if (!strcmp(m_pedal.params[i].key, key)) *m_pedal.params[i].val = v; }
static bool has_pedal_param(const char *key) { for (int i = 0; i < m_pedal.n_params; i++) if (!strcmp(m_pedal.params[i].key, key)) return true; return false; }
static void c_as_pull(double t) { g_pl_ped = 0.4f; c_pull(t); }
static void c_as_full(double t) { g_pl_ped = 1.0f; c_pull(t); }
// the throttle the pedal module gives for a pedal p at the state the run ended in (the gear and the speed of now, the clutch locked): the pedal held until the rise limit has run out; NAN = the module
// passes the pedal through (a shift is on, the truck stands)
static float as_gas(float p)
{
    m_pedal.reset(RESET_SOFT, "sim: grade assist scan");
    float out = p; int on = 0;
    for (int k = 0; k < 120; k++) { t_now += 1.0 / 60; on = pedal_wish(p, 1.0f / 60, 0, &out); }
    return on ? out : NAN;
}
static void run_layers_assist()
{
    printf("\n=== SCORES part 18k: the grade assist ([pedal] grade_assist): a climb at a fixed pedal, the pedal 0.4 from 50 km/h for 60 s, Normal ===\n");
    if (!has_pedal_param("grade_assist")) printf("  (no grade_assist in this tree: every column is the run without the assist)\n");
    printf("  %-28s %6s %9s %7s %9s %9s | %-22s | %-18s\n", "truck, road, pedal", "assist", "end km/h", "shifts", "end gear", "mean rpm", "at 50 km/h, start gear", "at the end (must)");
    printf("  %-28s %6s %9s %7s %9s %9s | %8s %13s | %9s %9s\n", "", "", "", "", "", "", "gas@0.4", "saturates at", "gas@1.0", "gas@0");
    struct Tr { const char *n; int kind; float mass; } trs[] = { { "DC16 40 t", 0, 40000 }, { "Pete 35.6 t", 2, 35600 } };
    for (const Tr &tr : trs)
        for (int road = 0; road < 3; road++) {   // +4 % at pedal 0.4, flat at pedal 0.4 (the assist must change nothing), +6 % at a full pedal
            const float ped = road == 2 ? 1.0f : 0.4f;
            double ref_v = -1; int ref_sh = -1, ref_g = -1;
            for (float c : { 0.0f, 0.5f, 0.75f, 1.0f, 2.0f }) {
                if (road == 1 && c == 2.0f) continue;
                if (tr.kind == 0) use_dc16(); else use_pete();
                set_pedal_param("grade_assist", c);
                const int g0 = sc_gear_for(50, 1100);
                setup su = { tr.mass, 50, g0, 60, 3, 0.5f };
                su.road = road == 0 ? r_p4 : road == 1 ? NULL : r_p6;
                char nm[96]; snprintf(nm, sizeof nm, "%s, %s, assist %.2f", tr.n, road == 0 ? "+4 %" : road == 1 ? "flat" : "+6 %, full pedal", c); g_ly_nm = nm;
                ly_res r = ly_row(su, road == 2 ? c_as_full : c_as_pull, 1, 1);
                const float kmh = M.v * 3.6f; const int gear = M.gear;
                const float g1 = as_gas(1.0f), g0v = as_gas(0.0f);   // (the state the run ended in)
                float sat = -1, g04 = NAN;
                {   // the start of the climb: 50 km/h in the gear the run started in
                    const float sv = T.speed, sr = T.rpm; const int sg = T.gear;
                    T.speed = 50 / 3.6f; T.gear = g0; T.rpm = vehicle_rpm_at(g0, T.speed);
                    g04 = as_gas(0.4f);
                    for (int i = 0; i <= 100 && sat < 0; i++) { float gv = as_gas(i / 100.0f); if (gv >= 0.999f) sat = i / 100.0f; }
                    T.speed = sv; T.rpm = sr; T.gear = sg;
                }
                char satb[16]; if (sat >= 0) snprintf(satb, sizeof satb, "%.2f", sat); else snprintf(satb, sizeof satb, "-");
                char lab[48]; snprintf(lab, sizeof lab, "%s, %s", tr.n, road == 0 ? "+4 %, pedal 0.4" : road == 1 ? "flat, pedal 0.4" : "+6 %, pedal 1.0");
                printf("  %-28s %6.2f %9.1f %7d %9d %9.0f | %8.3f %13s | %9.4f %9.4f\n", lab, c, kmh, r.shifts, gear, r.mean_rpm, g04, satb, g1, g0v);
                // must: the pedal's ends are the pedal's ends at every assist, and the flat is the flat
                if (g1 == g1 && g1 < 0.9999f) { printf("      FAIL: %s assist %.2f: a full pedal gives gas %.4f\n", lab, c, g1); g_as_failed = 1; }
                if (g0v == g0v && g0v != 0.0f) { printf("      FAIL: %s assist %.2f: a pedal of 0 gives gas %.4f\n", lab, c, g0v); g_as_failed = 1; }
                if (road == 1) {
                    if (ref_v < 0) { ref_v = M.v; ref_sh = r.shifts; ref_g = gear; }
                    else if (fabs(M.v - ref_v) > 0.05 || r.shifts != ref_sh || gear != ref_g) { printf("      FAIL: %s assist %.2f: the flat run differs from the run without the assist\n", lab, c); g_as_failed = 1; }
                }
            }
        }
    set_pedal_param("grade_assist", 0.75f);
}

// SC_PARTS=18f,18i: run only these parts of the layers' scores (a development filter; unset = all). 18i-6 reads the 18f totals, so a run without 18f has none.
static bool sc_part(const char *p)
{
    static const char *e = getenv("SC_PARTS");
    if (!e) return true;
    char want[40], list[160];
    snprintf(want, sizeof want, ",%s,", p);
    snprintf(list, sizeof list, ",%s,", e);
    return strstr(list, want) != NULL;
}
// The context the part-18 rows run in (sim_promises.cpp reuses its rows): the idle of 550, the driver profile fed, the cruise model off, the TrueShift keys at their defaults
static int g_sc_idle0; static float g_sc_saved[256];
static void layers_begin()
{
    g_sc_idle0 = g_idle_sim; g_idle_sim = 550;
    g_driver_feed = getenv("SC_NOFEED") ? 0 : 1;
    g_cruise_smooth = 0; CF.fm = 0; CF.on = 0; CF.at = -1;
    const int np = m_trueshift.n_params < 256 ? m_trueshift.n_params : 256;
    for (int j = 0; j < np; j++) { g_sc_saved[j] = *m_trueshift.params[j].val; *m_trueshift.params[j].val = m_trueshift.params[j].def; }
    // SCP="key=value,..." (the tuning override, as in main) on top of the defaults: the rows below run on the shipped values unless asked otherwise
    if (getenv("SCP")) { char b[1024]; snprintf(b, sizeof b, "%s", getenv("SCP")); for (char *t = strtok(b, ","); t; t = strtok(NULL, ",")) { char *e = strchr(t, '='); if (e) { *e = 0; set_param(t, (float)atof(e + 1)); } } }
}
static void layers_end()
{
    const int np = m_trueshift.n_params < 256 ? m_trueshift.n_params : 256;
    for (int j = 0; j < np; j++) *m_trueshift.params[j].val = g_sc_saved[j];
    g_idle_sim = g_sc_idle0;
    g_pl_wish = 0;
    g_driver_feed = 0; g_driver_inject = NULL; g_driver_pre = NULL;
    use_ato();
}
static int run_scores_layers()
{
    printf("\n=== SCORES part 18: the layered powertrain (the old choice, column l = 0, is gone: its numbers are in tools/sim/results/base_*_p18.txt) ===\n");
    layers_begin();

    printf("\n  %-32s %s %6s %6s %8s %5s %8s %7s %6s\n", "scenario", "l", "shifts", "skips", "land rpm", "t70", "fuel L", "out s", "hunts");
    auto row = [](const char *name, ly_res a) {
        char ta[16];
        snprintf(ta, sizeof ta, a.t70 < 0 ? "   -" : "%4.1f", a.t70);
        printf("  %-32s 1  %6d %6d %8.0f %5s %8.2f %7.1f %6d\n", name, a.shifts, a.skips, a.land_rpm, ta, a.fuel, a.out_s, a.hunts);
    };

    if (sc_part("18")) {
    // the ATS Peterbilt at 15.5 t, torque-pedal pulls from a standstill to 70 km/h, flat and +2 %
    use_pete();
    float peds[4] = { 0.3f, 0.5f, 0.8f, 1.0f };
    for (int gr = 0; gr < 2; gr++)
        for (float ped : peds) {
            g_pl_ped = ped;
            setup su = {15500, 1, 1, 120, 3, 1.0f};
            if (gr) su.road = r_two;
            char nm[96];
            snprintf(nm, sizeof nm, "Pete 15.5 t, pedal %.1f, %s", ped, gr ? "+2 %" : "flat");
            g_ly_nm = nm; row(nm, ly_row(su, c_pull, 1, 1));
        }
    // the DC16 770 at 40 t
    use_dc16();
    for (float ped : { 0.5f, 1.0f }) {
        g_pl_ped = ped;
        setup su = {40000, 1, 1, 120, 3, 1.0f};
        char nm[96];
        snprintf(nm, sizeof nm, "DC16 40 t, pedal %.1f, flat", ped);
        g_ly_nm = nm; row(nm, ly_row(su, c_pull, 1, 1));
    }
    // the DC16 770 at 18 t (solo)
    use_dc16();
    for (float ped : { 0.5f, 1.0f }) {
        g_pl_ped = ped;
        setup su = {18000, 1, 1, 120, 3, 1.0f};
        char nm[96];
        snprintf(nm, sizeof nm, "DC16 18 t, pedal %.1f, flat", ped);
        g_ly_nm = nm; row(nm, ly_row(su, c_pull, 1, 1));
    }
    // the cruise stand at 83 km/h: the DC16 770 at 18 t and 40 t, flat and down -1 % (the box's top is the 0.78 overdrive:
    // the stand should pick it downhill / solo, and not the loaded flat where it lugs)
    use_dc16();
    g_fuel_raw = 1000.0; T.fuel = 1000.0f;   // refuel: the pull-away rows above have drained the 1000 L plant tank
    for (float mass : { 18000.0f, 40000.0f })
        for (int gr = 0; gr < 2; gr++) {
            setup cs = {mass, 83, 14, 120, 3, 1.0f};
            if (gr) cs.road = r_minus1;
            char nm[96];
            snprintf(nm, sizeof nm, "DC16 %.0f t, cruise 83, %s", mass / 1000, gr ? "-1 %" : "flat");
            g_ly_nm = nm;   // (SC_LY=<name substring> shows the log of these rows too)
            ly_res a = ly_row(cs, c_ly83, 1, 0);
            g_cr83[(mass > 20000 ? 2 : 0) + gr] = a;   // (18i-7 holds them to the rewrite's)
            row(nm, a);
            printf("        (cruise: end gear %d, mean rpm %.0f)\n", a.end_gear, a.mean_rpm);
        }
    }

    if (sc_part("18b")) run_layers_modes();
    const int rh = sc_part("18c") ? run_layers_hunts() : 0;   // (18c: any hunt fails the run)
    if (sc_part("18d")) run_layers_climb();
    if (sc_part("18e")) run_layers_landing();
    // (the rows start from 1 km/h in the first gear: the plant slips the clutch there as the game's does, core/vehicle.h; the older plant held the engine at the idle with the clutch "closed", which no gear
    //  but the first can carry and which the start gear of the launch rows no longer is)
    if (sc_part("18f")) { g_slip_k = CLUTCH_SLIP_K; run_layers_release(); g_slip_k = 0.0f; }
    if (sc_part("18g")) run_layers_route();
    if (sc_part("18h")) run_layers_braking();
    if (sc_part("18j")) run_layers_descent();
    if (sc_part("18l")) run_layers_drag();
    if (sc_part("18k")) run_layers_assist();
    const int rb = sc_part("18i") ? run_layers_b() : 0;   // the GAPs of 18i fail the run (its "open" rows are the next package's and are not counted)

    layers_end();
    return g_gr_failed | g_bk_failed | g_as_failed | g_dr_failed | (rb ? 1 : 0) | (rh ? 1 : 0);
}


// drives. Three checks: (a) plant eta 1.0 (11 % stronger than the model's 0.9), level road, varying steady pedals:
// f_scale -> 1.11 +- 0.03, f_bias near 0; (b) model eta but +1.5 kN of plant resistance the model does not know:
// f_bias -> 1.5 +- 0.4 kN, f_scale stays 1; (c) plant == model with a lagged telemetry accel: repeated full-throttle
// pulls and shifts must not move either.
static void c_fbias_var(double t) { static const float L[] = { 0.15f, 0.30f, 0.50f, 0.70f, 0.25f, 0.45f, 0.65f, 0.85f }; M.pedal = L[((int)(t / 12)) % 8]; }
static void c_fbias_pull(double t)
{
    float kmh = M.v * 3.6f;
    if (M.pedal > 0.5f) { if (kmh > 85) { M.pedal = 0; M.brake = 0.15f; } else M.pedal = 1; }
    else { if (kmh < 50) { M.pedal = 1; M.brake = 0; } else { M.pedal = 0; M.brake = 0.15f; } }
}
// the largest |f_scale - 1| and |f_bias| after the first minute (the learner's own settling is no "drift")
static double g_fb_t0; static float g_fb_maxds, g_fb_maxdb;
static void fb_track(double) { if (t_now - g_fb_t0 >= 60) { g_fb_maxds = fmaxf(g_fb_maxds, fabsf(V.f_scale - 1.0f)); g_fb_maxdb = fmaxf(g_fb_maxdb, fabsf(V.f_bias)); } }

static int run_fbias()
{
    int f = 0;
    float save_eta = g_plant_eta, save_res = g_plant_res, save_lag = g_accel_lag;
    use_volvo();

    g_plant_eta = 1.0f; g_plant_res = 0; g_accel_lag = 0;
    sim_reset(RESET_HARD, "sim: force learner a");
    run("FB (a) 40 t, plant 11 % stronger than the model, varying steady pedals", {40000, 80, 12, 620}, c_fbias_var);
    {
        float s = V.f_scale, b = V.f_bias / 1000;
        int ok = fabsf(s - 1.1111f) <= 0.03f && fabsf(b) < 0.5f;
        printf("  %-4s plant factor -> f_scale %.3f (1.11 +- 0.03), f_bias %.2f kN (|.| < 0.5)\n", ok ? "ok" : "FAIL", s, b);
        if (!ok) f = 1;
    }

    g_plant_eta = 0.9f; g_plant_res = 1500; g_accel_lag = 0;
    sim_reset(RESET_HARD, "sim: force learner b");
    run("FB (b) 40 t, +1.5 kN of plant resistance, varying steady pedals", {40000, 80, 12, 620}, c_fbias_var);
    {
        float s = V.f_scale, b = V.f_bias / 1000;
        int ok = fabsf(s - 1.0f) <= 0.03f && fabsf(b - 1.5f) <= 0.4f;
        printf("  %-4s plant resistance -> f_bias %.2f kN (1.5 +- 0.4), f_scale %.3f (1 +- 0.03)\n", ok ? "ok" : "FAIL", b, s);
        if (!ok) f = 1;
    }

    g_plant_eta = 0.9f; g_plant_res = 0; g_accel_lag = 1;
    sim_reset(RESET_HARD, "sim: force learner c");
    g_fb_t0 = t_now; g_fb_maxds = g_fb_maxdb = 0;
    g_tick_hook = fb_track;
    run("FB (c) 40 t, plant == model, lagged accel, repeated full-throttle pulls", {40000, 20, 4, 300}, c_fbias_pull);
    g_tick_hook = NULL;
    {
        int ok = g_fb_maxds <= 0.03f && g_fb_maxdb < 500.0f;
        printf("  %-4s no drift with a lagged telemetry accel: |f_scale - 1| <= %.3f (0.03), |f_bias| <= %.2f kN (0.5), f_scale ends %.3f\n",
               ok ? "ok" : "FAIL", g_fb_maxds, g_fb_maxdb / 1000, V.f_scale);
        if (!ok) f = 1;
    }

    g_plant_eta = save_eta; g_plant_res = save_res; g_accel_lag = save_lag;
    sim_reset(RESET_HARD, "sim: force learner done");
    return f;
}

// ---- the fuel / drag learner (core/vehicle.cpp): a 25 min mixed drive, twice (friction share 0.06 then 0.12), a fuel-scale
// switch 1 -> 6 at 12 min, a refuel at 20 min. k and kd converge to the plant's, a jumps within ~30 s of the switch, no NaN,
// and the refuel does not upset the learning.
static int g_fuel_refueled;
static double g_fuel_t0, g_fuel_coast_rpm_sum, g_fuel_coast_t, g_fuel_a_before, g_fuel_a_after;
static void c_fuel(double t)
{
    double cyc = fmod(t, 180.0);
    if (cyc < 50) { M.pedal = 0.35f; M.brake = 0; }          // steady cruise (a steady pedal: clean pull windows)
    else if (cyc < 64) { M.pedal = 0; M.brake = 0; }         // coast in gear (the drag / kd windows)
    else if (cyc < 84) { M.pedal = 0; M.brake = 0.5f; }      // brake to a stop
    else if (cyc < 104) { M.pedal = 0; M.brake = 0; M.gear = 0; M.target = 0; M.state = 0; }   // idle at the stop (neutral: the k windows)
    else if (cyc < 124) { M.pedal = 0.7f; M.brake = 0; }     // pull away
    else { M.pedal = 0.35f; M.brake = 0; }                   // back to cruise
    if (t > 1200 && !g_fuel_refueled) { g_fuel_refueled = 1; g_fuel_raw += 200.0; }
}
static void fuel_track(double dt)
{
    if (M.gear >= 1 && !M.state && M.pedal < 0.02f && M.cc < 0.02f && M.brake < 0.02f && M.v * 3.6f > 40) {
        g_fuel_coast_rpm_sum += rpm_now() * dt; g_fuel_coast_t += dt;
    }
    double rel = t_now - g_fuel_t0;
    if (rel >= 718 && rel < 720) g_fuel_a_before = fuel_a();   // just before the 1 -> 6 switch
    if (rel >= 748 && rel < 750) g_fuel_a_after = fuel_a();    // ~30 s after it
}

// ---- the fuel / drag learner on a RIDE (WP-E item 0). The row above feeds the learner what it likes (50 s of one pedal, a 14 s coast, 20 s of neutral, nothing braked);
// the owner's ride (02-11) did not: the Scania's line read nk=4 nkd=0 after 15 minutes. What a ride is made of, from its trueshift.csv: cruise 70-87 % of the time, the
// pedal's lifts in gear a median 0.7 s long (63 lifts, 2 of them 3.5 s or more), stops at lights with the brake held, hills, and part-throttle holds (a loaded truck at
// 10-25 % friction share, not the 40 % the learner used to want). Here: 150-190 s cycles of a cruise at 80 on rolling hills, the driver's own pedal with five lifts of
// 0.6 to 4.5 s, a braked stop with 12 s of standing in neutral with the brake on, and a pull away. 15 minutes, twice (18 t / friction share 0.12, and 40 t / 0.06 with the
// fuel scale jumping 1 -> 6 at 7 min). Must: k within 20 % of the plant's, kd within 25 %, and both have taken at least five windows (fuel_learned).
static double g_fr_c0, g_fr_stop; static int g_fr_ph;
static void c_fuelride(double t)
{
    if (t < 0.05) { g_fr_c0 = 0; g_fr_ph = 0; g_fr_stop = 0; M.cc = 0; M.cruise_kmh = 0; }
    static const double LIFT[5][2] = { { 48, 0.7 }, { 54, 1.2 }, { 62, 2.0 }, { 70, 0.6 }, { 76, 4.5 } };
    const double cs = t - g_fr_c0;
    M.brake = 0;
    if (g_fr_ph == 0) {
        if (cs < 45) hold_cc(80, cs);
        else {
            M.cc = 0; M.cruise_kmh = 0;
            int lift = 0;
            for (auto &l : LIFT) if (cs >= l[0] && cs < l[0] + l[1]) lift = 1;
            if (lift) M.pedal = 0; else hold(80);
        }
        if (cs >= 90) g_fr_ph = 1;
    } else if (g_fr_ph == 1) {   // slow to a stop at 1.2 m/s2
        M.pedal = 0; M.cc = 0; M.cruise_kmh = 0; M.brake = 0.3f;
        if (M.v < 0.2f) { g_fr_ph = 2; g_fr_stop = t; }
    } else if (g_fr_ph == 2) {   // standing at the light: the brake held, the engine idling in neutral
        M.pedal = 0; M.brake = 0.3f; M.gear = 0; M.target = 0; M.state = 0;
        if (t - g_fr_stop > 12) g_fr_ph = 3;
    } else {                     // pulling away to 60 km/h, then the next cycle
        M.pedal = 0.7f;
        if (M.v * 3.6f > 60) { g_fr_ph = 0; g_fr_c0 = t; }
    }
    if (t > 420 && !g_fuel_refueled) { g_fuel_refueled = 1; g_fuel_raw += 150.0; }
}
static int run_fuel_ride()
{
    int f = 0;
    use_volvo();
    float save_tf = g_sw_tf, save_drag = g_sw_drag, save_s = g_fuel_scale, save_s2 = g_fuel_scale2, save_sw = g_fuel_switch_s;
    remove("fuel_learn.txt");
    g_tick_hook = fuel_track;
    for (int i = 0; i < 2; i++) {
        const float tf = i == 0 ? 0.12f : 0.06f, mass = i == 0 ? 18000.0f : 40000.0f;
        g_sw_tf = tf; g_sw_drag = 0.12f; g_fuel_scale = 1.0f; g_fuel_scale2 = i == 0 ? 0.0f : 6.0f;
        g_fuel_refueled = 0; g_fuel_coast_rpm_sum = g_fuel_coast_t = 0; g_fuel_a_before = g_fuel_a_after = 0;
        g_fuel_raw = 1000.0; T.fuel = 1000.0f;
        g_fuel_t0 = t_now; g_fuel_switch_s = g_fuel_t0 + 420.0;
        sim_reset(RESET_HARD, "sim: fuel learner ride");
        char nm[160];
        snprintf(nm, sizeof nm, "FUEL RIDE %.0f t, friction share %.2f: 15 min of cruise, lifts of 0.6-4.5 s, braked stops, hills%s", mass / 1000, tf, i ? ", scale 1 -> 6 at 7 min" : "");
        setup su = { mass, 80, 11, 900, 1.6667f };
        su.road = r_hills;   // (+-1.5 % over 1.5 km: the cruise holds with its throttle; a steeper road lets it coast, which gives the drag windows a ride does not have)
        run(nm, su, c_fuelride);
        int nk = 0, nkd = 0, na = 0;
        vehicle_fuel_counts(&nk, &nkd, &na);
        const float k = fuel_k(), kd = fuel_kd();
        const float r_coast = g_fuel_coast_t > 0 ? (float)(g_fuel_coast_rpm_sum / g_fuel_coast_t) : 0;
        const float kd_target = r_coast > 0 ? g_sw_drag * (fuel_kd_rpm() / LIMIT) / V.eta : 0;   // (the plant's drag at the rpm the windows were taken at: kd is a share at that rpm)
        const int ok_k = k == k && fabsf(k - tf) <= 0.20f * tf && nk >= 5;
        const int ok_kd = kd == kd && kd_target > 0 && fabsf(kd - kd_target) <= 0.25f * kd_target && nkd >= 5;
        printf("  %-4s RIDE %2.0f t: k %.3f (%.2f +- 20 %%, %d windows), kd %.3f (%.3f +- 25 %%, %d windows, coast %.0f rpm), a windows %d, learned %d\n",
               ok_k && ok_kd ? "ok" : "FAIL", mass / 1000, k, tf, nk, kd, kd_target, nkd, r_coast, na, fuel_learned());
        if (!(ok_k && ok_kd)) f = 1;
    }
    g_tick_hook = NULL;
    g_sw_tf = save_tf; g_sw_drag = save_drag; g_fuel_scale = save_s; g_fuel_scale2 = save_s2; g_fuel_switch_s = save_sw;
    sim_reset(RESET_HARD, "sim: fuel learner ride done");
    remove("fuel_learn.txt");
    return f;
}

static int run_fuel()
{
    int f = 0;
    use_volvo();
    float save_tf = g_sw_tf, save_drag = g_sw_drag, save_s = g_fuel_scale, save_s2 = g_fuel_scale2, save_sw = g_fuel_switch_s;
    g_fuel_scale = 1.0f; g_fuel_scale2 = 6.0f; g_sw_drag = 0.12f;
    remove("fuel_learn.txt");   // (hermetic: the learner persists per truck in the working directory; as sim_brake does with brake_gain.txt)
    g_tick_hook = fuel_track;
    for (int i = 0; i < 2; i++) {
        float tf = i == 0 ? 0.06f : 0.12f;
        g_sw_tf = tf;
        g_fuel_refueled = 0; g_fuel_coast_rpm_sum = g_fuel_coast_t = 0; g_fuel_a_before = g_fuel_a_after = 0;
        g_fuel_raw = 1000.0; T.fuel = 1000.0f;
        g_fuel_t0 = t_now; g_fuel_switch_s = g_fuel_t0 + 720.0;
        sim_reset(RESET_HARD, "sim: fuel learner");
        char nm[160];
        snprintf(nm, sizeof nm, "FUEL TF %.2f: 25 min cruise/pull/coast/idle, zone 1 -> 6 at 12 min, refuel at 20 min", tf);
        run(nm, {40000, 80, 11, 1500, 1.6667f}, c_fuel);
        float k = fuel_k(), kd = fuel_kd(), a = fuel_a();
        float r_coast = g_fuel_coast_t > 0 ? (float)(g_fuel_coast_rpm_sum / g_fuel_coast_t) : 0;
        float kd_target = r_coast > 0 ? g_sw_drag * (r_coast / LIMIT) / V.eta : 0;
        int ok_k = k == k && fabsf(k - tf) <= 0.15f * tf;
        int ok_kd = kd == kd && kd_target > 0 && fabsf(kd - kd_target) <= 0.20f * kd_target;
        int ok_a = a == a && g_fuel_a_before > 0 && g_fuel_a_after >= 4.0f * g_fuel_a_before;
        int ok_learned = fuel_learned();
        printf("  %-4s TF %.2f: k %.3f (%.2f +- 15 %%), kd %.3f (%.3f +- 20 %%, coast %.0f rpm), a %.3g -> %.3g at 12 min (>= 4x), learned %d, refuel %d\n",
               ok_k && ok_kd && ok_a && ok_learned ? "ok" : "FAIL", tf, k, tf, kd, kd_target, r_coast, g_fuel_a_before, g_fuel_a_after, fuel_learned(), g_fuel_refueled);
        if (!(ok_k && ok_kd && ok_a && ok_learned)) f = 1;
    }
    g_tick_hook = NULL;
    g_sw_tf = save_tf; g_sw_drag = save_drag; g_fuel_scale = save_s; g_fuel_scale2 = save_s2; g_fuel_switch_s = save_sw;
    sim_reset(RESET_HARD, "sim: fuel learner done");
    remove("fuel_learn.txt");
    f |= run_fuel_ride();
    return f;
}

#ifndef SIM_PCC
// The real torque pedal (modules/pedal.cpp) runs in this sim; what it asks of the other modules' state is the idle stand-in of a sim without the cruise and the crawl.
static creep_info CI0; static cruise_info CU0;
void creep_info_get(creep_info *o) { *o = CI0; }
void cruise_info_get(cruise_info *o) { *o = CU0; }
#include "modules/pedal.cpp"
#endif

// The start of main, once (sim_promises.cpp calls it too): the modules' defaults, the Volvo as the file starts, the hooks.
#include "sim_tc1.inc"   // WP-TC1: the converter truck and the mod engine (TC1=1, run_tc1.sh)
#include "sim_tc2.inc"   // WP-TC2: the converter's stall ratio read from the game's transmission object (TC2=1, run_tc2.sh)
#include "sim_allison.inc" // R0611-ALLISON: the MAN Allison of the ride of 06-11, skips that the converter model made look free (ALLISON=1, run_allison.sh)
#include "sim_g8.inc"    // WP-G8: the price of reversing a shift that fades with the situation (the last block of the default run)
#include "sim_r0611.inc"  // R0611: the Scania of the ride of 06-11 (R0611=1, run_r0611.sh; not in the default run)
#include "sim_g1.inc"     // WP-0911-G1: the service brake alone never raises the revs (a feathering pedal), and the route layer off in the gearbox (G1=1, run_g1.sh; not in the default run)

static void sim_boot()
{
    for (int i = 0; i < m_trueshift.n_params; i++) *m_trueshift.params[i].val = m_trueshift.params[i].def;
    for (int i = 0; i < m_carshift.n_params; i++) *m_carshift.params[i].val = m_carshift.params[i].def;
    for (int i = 0; i < m_pedal.n_params; i++) *m_pedal.params[i].val = m_pedal.params[i].def;
    T.fwd_gears = 12; T.diff = 2.85f; T.wheels = 4; T.retarder_steps = 5;
    T.fuel = 1000.0f;
    for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
    save_volvo();
    G.tail_site = 5; G.decide_site = 1; G.n_choose = 1; G.choose_sites[0] = 2; G.n_start = 1; G.start_sites[0] = 4;
    G.window_fn = (uintptr_t)f_window; G.chooser_fn = (uintptr_t)f_choose; G.start_fn = (uintptr_t)f_start;
    m_trueshift.start();
    m_carshift.start();   // (the game hooks are installed once, by whichever starts first)
}
int main(int argc, char **argv)
{
    sim_env_apply("trueshift", m_trueshift.params, m_trueshift.n_params);
    sim_env_apply("carshift", m_carshift.params, m_carshift.n_params);
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    sim_boot();
    if (getenv("BAND0")) set_param("crest_band_kmh", 0);
    if (getenv("NOBOOST")) set_param("eco_roll_boost_kmh", 0);
    if (getenv("NOLOAD")) set_param("load_down_from", 1);
    if (getenv("CHOICE")) set_param("choice", (float)atoi(getenv("CHOICE")));   // CHOICE=1: every scenario below with the score-based gear choice
    if (getenv("SCP")) { char b[1024]; snprintf(b, sizeof b, "%s", getenv("SCP")); for (char *t = strtok(b, ","); t; t = strtok(NULL, ",")) { char *e = strchr(t, '='); if (e) { *e = 0; set_param(t, (float)atof(e + 1)); } } }   // SCP="key=value,..." overrides ini keys (tuning)
    if (getenv("PMP")) { char b[512]; snprintf(b, sizeof b, "%s", getenv("PMP")); for (char *t = strtok(b, ","); t; t = strtok(NULL, ",")) { char *e = strchr(t, '='); if (e) { *e = 0; char k[96]; snprintf(k, sizeof k, "pd_%s", t); set_param(k, (float)atof(e + 1)); } } }   // PMP="key=value,..." the [pedal] keys

    if (getenv("G1")) return run_g1();   // G1=1: only the rows of WP-0911-G1 (g_route_rows stays 0: the key is the row's own)
    if (getenv("R0611")) return run_r0611();   // R0611=1: only the rows of the Scania ride of 06-11
    if (getenv("G8")) return run_g8();   // G8=1: only the WP-G8 rows (they are also the last block of the default run)
    if (getenv("SWEEP")) return run_sweep();   // (the sweep replays the owner's rides, the gearbox as it is: g_route_rows stays 0 there)
    g_route_rows = 1;                          // every other mode: its rows with a route are rows about the route layer
    if (getenv("SCORES")) return run_scores();
    if (getenv("FBIAS")) return run_fbias();
    if (getenv("FUEL")) return run_fuel();
    if (getenv("TC")) return run_tc();
    if (getenv("TC1")) return run_tc1();
    if (getenv("ALLISON")) return run_allison();
    if (getenv("TC2")) return run_tc2();
    if (getenv("RELAX")) {  // skip_relax_rpm study: pull-away at a fixed pedal from a standstill, 12 and 25 km/h, 12/20/36 t, both of his boxes
        // CSV: truck, t, pedal (-1 = wavering), from km/h, s to 30/50/80, shifts, ups, skips, hunting pairs, no-pull lessons, incidents,
        // lowest up-landing rpm (>= 8 km/h), mean rpm below 50 km/h, shifts and hunting pairs >= 8 km/h, lowest skip landing rpm, lowest pull after a skip (m/s2)
        // env: RELAX_R = skip_relax_rpm (0 = the old floor), RELAX_MS = max_skip, RELAX_IDLE = idle_rpm
        const char *ks[] = { "skip_relax_rpm", "max_skip", "idle_rpm" }, *es[] = { "RELAX_R", "RELAX_MS", "RELAX_IDLE" };
        for (int i = 0; i < 3; i++) if (getenv(es[i])) set_param(ks[i], (float)atof(getenv(es[i])));
        struct Tr { const char *name; void (*use)(); int g0; } trucks[2] = { { "ATO", use_ato, 1 }, { "XT", use_renault, 3 } };
        for (auto &tr : trucks) {
            tr.use();
            for (float mass : { 12000.0f, 20000.0f, 36000.0f })
                for (float ped : { 0.1f, 0.2f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.55f, 0.6f, 0.7f, 1.0f, -1.0f })
                    for (float v0 : { 1.0f, 12.0f, 25.0f }) {
                        int g0 = tr.g0;
                        if (v0 > 5) for (int g = 1; g <= NG; g++) { float r = (v0 / 3.6f) / RADIUS * RATIO[g - 1] * DIFF * 9.5493f; if (r >= 1000) g0 = g; }
                        g_ped = ped; int i0 = g_n_incidents, f0 = n_failed;
                        char nm[160]; snprintf(nm, sizeof nm, "RELAX %s %.0f t pedal %.2f from %.0f km/h in %d", tr.name, mass / 1000, ped, v0, g0);
                        run(nm, {mass, v0, g0, 120}, c_ped);
                        printf("CSV,%s,%.0f,%.2f,%.0f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%.0f,%.0f,%d,%d,%.0f,%.2f\n", tr.name, mass / 1000, ped, v0, t_30, t_50, t_80,
                               shifts, ups, skips_up, hunts, n_failed - f0, g_n_incidents - i0, up_rpm_min < 1e8 ? up_rpm_min : 0, rpm_all_t > 0 ? rpm_all_sum / rpm_all_t : 0, shifts_ts, hunts_ts,
                               skip_rpm_min < 1e8 ? skip_rpm_min : 0, skip_pull_min < 1e8 ? skip_pull_min : 0);
                    }
        }
        return 0;
    }
    if (getenv("ACCEL0")) {  // pull-away from a standstill at light pedal, on his two boxes: 0 to 30 and 0 to 50 km/h with 0 or 2 skipped gears
        struct Tr { const char *name; void (*use)(); int g0; } trucks[2] = { { "ATO2612F (12, diff 3.08)", use_ato, 1 }, { "Xtended 12+2 (diff 3.40)", use_renault, 3 } };
        void (*ctls[4])(double) = { c_p20, c_p30, c_p40, c_p50 }; const char *cn[4] = { "0.2", "0.3", "0.4", "0.5" };
        for (auto &tr : trucks) {
            tr.use();
            for (float mass : { 36000.0f, 20000.0f })
                for (int c = 0; c < 4; c++)
                    for (int ms : { 0, 2 }) {
                        set_param("max_skip", (float)ms);
                        char nm[160];
                        snprintf(nm, sizeof nm, "ACCEL0 %s, %.0f t, pedal %s, max_skip %d", tr.name, mass / 1000, cn[c], ms);
                        run(nm, {mass, 1, tr.g0, 90}, ctls[c]);
                    }
        }
        return 0;
    }
    if (getenv("ACCEL")) {  // acceleration study: how fast is a pull-away with 0, 1 or 2 skipped gears (no fuel in it), on his two boxes
        struct Tr { const char *name; void (*use)(); int ng; } trucks[2] = { { "ATO2612F 12 speeds, diff 3.08", use_ato, 12 }, { "Xtended 12+2, diff 3.40", use_renault, 14 } };
        for (auto &tr : trucks) {
            tr.use();
            // the start gear at 12 km/h: the highest gear that still gives >= 1000 rpm there
            int g0 = 1; for (int g = 1; g <= tr.ng; g++) { float r = (12 / 3.6f) / RADIUS * RATIO[g - 1] * DIFF * 9.5493f; if (r >= 1000) g0 = g; }
            for (int ms = 0; ms <= 2; ms += 1) {
                set_param("max_skip", (float)ms);
                char nm[160];
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 36 t, pedal 1.0, from 12 km/h in %d", tr.name, ms, g0); run(nm, {36000, 12, g0, 80}, c_full);
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 36 t, pedal 0.7", tr.name, ms); run(nm, {36000, 12, g0, 80}, c_p70);
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 36 t, pedal 0.5", tr.name, ms); run(nm, {36000, 12, g0, 80}, c_p50);
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 20 t, pedal 0.7", tr.name, ms); run(nm, {20000, 12, g0, 80}, c_p70);
            }
        }
        return 0;
    }
    run("A1 40 t, flat, full throttle from 12 km/h (DCT 0.5 s)", {40000, 12, 3, 70}, c_full);
    run("A2 40 t, flat, full throttle, slow box 1.2 s", {40000, 12, 3, 70, 3, 1.2f}, c_full);
    run("A2b 8 t solo, flat, full throttle (skips?)", {8000, 12, 4, 40}, c_full);
    run("A2c 13.4 t empty trailer, full throttle from 12 km/h in 3 (ts3b: wants 3->5)", {13400, 12, 3, 40}, c_full);
    run("A3 10 t solo, light throttle 0.4 from 12 km/h, Normal", {10000, 12, 4, 60}, c_light);
    run("A4 same, Eco", {10000, 12, 4, 60, 10}, c_light);
    run("A5 same, Power", {10000, 12, 4, 60, 1.6667f}, c_light);
    run("A6 same, Normal, slow box 1.2 s (skips?)", {10000, 12, 4, 60, 3, 1.2f}, c_light);
    run("B1 10 t solo, driver holds 80 flat (low rpm?)", {10000, 80, 10, 90}, c_solo80);
    run("B2 40 t, driver holds 85 on 1%", {40000, 85, 11, 90}, c_cruise85);
    run("B3 BeamNG test: 40 t holds 80, 3.5% from t=30 to 90", {40000, 80, 11, 150}, c_beamng);
    // The roll at the end of the climb (80 km/h, the pedal at 0.05 and falling) was a start for nothing: the model saw a coast that held the speed for 30 s, the driver was back on the
    // pedal a second later (WP-G6c: the neutral way of the roll model ends where the driver would, under the speed he held by the roll band). The gear it left stays.
    if (hunts || short_rolls) { printf("  B3 FAIL: %d hunting pairs, %d rolls over within 5 s, in %d shifts (expect none: a roll the model could see ending that soon was a start for nothing)\n", hunts, short_rolls, shifts); g_x_failed = 1; }
    else printf("  B3 ok: %d shifts, no hunting pairs, no roll for nothing\n", shifts);
    run("B3b BeamNG trap: 14 t solo seen as 10 t, holds 80, 3.5% t=30-90", {14000, 80, 11, 150, 3, 0.5f, 10000}, c_beamng);
    run("B3c same, Eco", {14000, 80, 11, 150, 10, 0.5f, 10000}, c_beamng);
    run("B3d long trap: 25 t seen as 14 t, holds 80 on rolling hills", {25000, 80, 11, 400, 3, 0.5f, 14000}, c_hills);
    // The roll on a descent that steepens (no route): it started where the model's neutral coast still held the speed (16 N of margin) and ended 0.5 s later, when the grade had
    // steepened by 0.1 % (-2.20 -> -2.30 %): 11 -> 0 -> 10 inside a second and a half, a hunt each time. A roll pays for two shifts, so it has to be one the model sees lasting longer.
    if (hunts || short_rolls) { printf("  B3d FAIL: %d hunting pairs, %d rolls over within 5 s, in %d shifts (expect none: every roll the model could see ending within a shift time or two was a start for nothing)\n", hunts, short_rolls, shifts); g_x_failed = 1; }
    else printf("  B3d ok: %d shifts, no hunting pairs, no roll for nothing\n", shifts);
    g_row_extra = 1; run("B4 40 t holds 80 on rolling hills +-3%", {40000, 80, 11, 240}, c_hills); g_row_extra = 0;
    // The rolls at the foot of each descent end on the climb: the engage out of neutral takes the gear for the speed and a light pedal (12 at 917 rpm on 2.6 %) and the climb takes it down again,
    // 12 > 11 > 10 in two seconds (WP-G6c round 2, the owner's "why do you shift so much"). a5988c6 and 629d6cd drive this road in 11 shifts and no roll; a roll may be there, the shifts after it may not.
    // K2 step 1 (rows audit RS-4, manager's decision): the pinned 11 shifts (what a5988c6 and 629d6cd made on this road) becomes the owner's cruise yardstick, the one T6c uses (yard_verdict): not up against the
    // base (K1 + the installed line, 43f2524: 17 shifts), no downshift within the dwell of an engage, at most two shifts on a climb and a gap between two of them of at least the dwell (the 12 > 11 > 10 in two
    // seconds of the owner's 'why do you shift so much'), diesel not up and the distance not down (memory cruise-meer-schakelen-ok: flips, fuel, time; the 12 share is printed).
    { yard_t ref = {}; ref.shifts = 17; ref.fuel = B4_BASE_FUEL; ref.dist = B4_BASE_DIST; g_yard_ref = "the base 43f2524";
      const int ok = yard_verdict("B4", yard_snap(), ref, false); g_yard_ref = "int-1011";
      if (!ok || engage_downs) { printf("  B4 FAIL: %d downshifts within 5 s of an engage (expect none) and the yardstick above\n", engage_downs); g_x_failed = 1; } }
    run("B5 40 t cruise control 80 on 1% (bang-bang throttle)", {40000, 80, 11, 90}, c_cc);
    run("B6 26 t adaptive cruise 80 behind a truck at 50", {26000, 60, 10, 60}, c_acc);
    run("C1 40 t, full throttle, 6% climb from t=10 s", {40000, 80, 11, 60}, c_climb);
    run("C2 40 t, 70 km/h at 0.35, kickdown at t=10 s", {40000, 70, 11, 25}, c_kick);
    // WP-G6c round 2 (2): the layers are told the gear the last shift left while the dwell runs, so that a shift back to it costs its shift time twice (core/gearchoice.h gc_in::prev, "anti-hunting
    // inside the score"). TrueShift passed the gear it was IN (S.last_gear is the gear after the shift), so the price never applied. Rows: the pull of C1 (downshifts and upshifts under a held pedal)
    // and the kickdown above; every decision within 1.5 s of a shift (inside both dwells, 2 s and 3 s) names the gear that shift left.
    g_dw_n = g_dw_bad = 0; g_dw_before = g_dw_last = 0; g_dw_t = -1e9; g_tick_hook = dw_track;
    run("DW1 40 t, 70 km/h at 0.35, kickdown at t=10 s", {40000, 70, 11, 25}, c_kick);
    run("DW2 40 t, full throttle, 6% climb from t=10 s", {40000, 80, 11, 60}, c_climb);
    run("DW3 40 t, full throttle from 12 km/h (the pull through the gears)", {40000, 12, 3, 70}, c_full);
    g_tick_hook = NULL;
    // K2 step 1 (rows audit RS-5, DROPPED as a row): this checks a wiring field (gc_in::prev names the gear the last shift left), nothing a driver feels; the felt effect, no reversal at an unchanged
    // foot, is the one hunt definition's and the rows B3 / B3d / G1c. It stays as a printed line for diagnosis and no longer fails the run.
    printf("  DW (wiring, not a row): %d of %d decisions inside the dwell name a gear other than the one the last shift left\n", g_dw_bad, g_dw_n);
    run("C3 40 t, braking 0.25 from 80 km/h", {40000, 80, 11, 25}, c_brake);
    run("C3b same, a 30 km/h junction 250 m ahead on the route (earlier downshifts)", {40000, 80, 11, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 250}, c_brake);
    run("D1 40 t, 5% down holding 70, engine brake (level unknown = 0.5, toggle off)", {40000, 72, 12, 40}, c_eb);
    run("D2 40 t, 5% down holding 70, retarder 1/5", {40000, 72, 12, 40}, c_ret1);
    run("D3 40 t, 5% down holding 70, retarder 5/5", {40000, 72, 12, 40}, c_ret5);
    run("D4 40 t, 5% down holding 70, engine brake, game toggle on (full)", {40000, 72, 12, 40, 3, 0.5f, 0, 1}, c_eb);
    run("D5 40 t, 5% down holding 70, engine brake stage 1/3, toggle off", {40000, 72, 12, 40, 3, 0.5f, 0, 0, 0.33f}, c_eb);
    run("E1 40 t, 0.6 throttle, hint down at t=15 s", {40000, 50, 9, 35}, c_hint);
    run("E2 40 t low speed: 0.5 pedal from 0, brake, creep, stop", {40000, 0, 1, 70}, c_lowspeed);
    // M with m_style 2 (assisted: rev limit, stall, kickdown, stopping, lugging) here, 0 and 1 at the end: the sim carries
    // its state from run to run, so more runs here would move every result after them
    set_param("m_style", 2);
    printf("\n--- m_style = 2 ---");
    run_m();
    set_param("m_style", 0);

    run("R1 40 t holds 80, 4% climb at 1.5-2.5 km, NO route", {40000, 80, 11, 150, 3, 0.5f, 0, -1, -1, r_climb, 0}, c_hold80);
    run("R1b same WITH route (climb ahead)", {40000, 80, 11, 150, 3, 0.5f, 0, -1, -1, r_climb, 1}, c_hold80);
    run("R2 40 t full pedal 3% to a top at 700 m, NO route", {40000, 55, 10, 70, 3, 0.5f, 0, -1, -1, r_crest, 0}, c_full_road);
    run("R2b same WITH route (top ahead)", {40000, 55, 10, 70, 3, 0.5f, 0, -1, -1, r_crest, 1}, c_full_road);
    run("V1 12 t holds 70, lifts off at 10 s with a junction 60 m ahead, full pedal at 14 s", {12300, 70, 11, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 254}, c_lift_jn);
    run("V3 same as V1 but the game reports throttle 0.08 at pedal 0, starting in 10", {12300, 70, 10, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 254}, c_lift_jn_bug);
    run("V2 same, no junction, pedal stays off (rolls out)", {12300, 70, 11, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 0}, c_lift_jn_free);
    run("R3 26 t 0.5 pedal from 30, junction at 500 m, WITH route", {26000, 30, 6, 40, 3, 0.5f, 0, -1, -1, r_flat, 1, 500}, c_half);
    run("R4 40 t at 80, pedal released t=10-50 on -0.8% (eco-roll, Normal)", {40000, 80, 12, 70, 3, 0.5f, 0, -1, -1, r_down, 0}, c_release);
    run("R4b same in Power (no roll)", {40000, 80, 12, 70, 1.6667f, 0.5f, 0, -1, -1, r_down, 0}, c_release);
    g_cruise_smooth = 1; g_pcc_on = 1;
    run("R5 26 t cruise 80 on hills +-1.5% (roll with cruise, Eco)", {26000, 80, 12, 240, 10, 0.5f, 0, -1, -1, r_hills, 1}, c_cc80);
    run("R5b same, eco-roll off via Power", {26000, 80, 12, 240, 1.6667f, 0.5f, 0, -1, -1, r_hills, 1}, c_cc80);
    g_cruise_smooth = 0; g_pcc_on = 0;
    // PCC band (Damien 11-10): a top within reach is taken in this gear (at most -5 km/h), a long climb still downshifts.
    // The G1c must rows: the game's own cruise on gentle hills must not shift at every hill (WP-G1c). The bar is the 72ad92e count (before G1.0b's
    // cruise throttle read as the pedal and G1b), 0 hunting pairs on each; an extra shift would pass only if the row's own time and fuel show it buys
    // time worth its fuel in the mode's currency (none of the five does). Captured on the shipped crest_band_kmh (5).
    // K2 step 1: the targets are the base's own counts (43f2524: all four at 0 shifts), not up against the base; the pinned 72ad92e count of T0b (2) let two shifts pass that the base does not make.
    struct { const char *name; int target; int shifts, hunts; } must[4] = {
        { "T0b", 0, 0, 0 }, { "T4", 0, 0, 0 }, { "T6", 0, 0, 0 }, { "T6b", 0, 0, 0 },
    };
    // The five must rows are a requirement of the BOX (no shift at every hill under the game's cruise), not of the route layer: they run with the key as it stands (0 by default since WP-0911-G1), so a
    // row that held only thanks to the route's crest band shows it (T6c: 2 shifts with the route, 15 without). The other T rows are about the crest band itself and run with the route on.
    auto gearbox_run = [&](const char *nm, setup su, void (*ctl)(double)) {
        const int was = g_route_rows; g_route_rows = 0;
        g_row_extra = !strncmp(nm, "T6c", 3); if (g_row_extra) g_tick_hook = t6c_trace;   // (T6c also says fuel and the shortest time between two shifts, and can trace the layers: T6C_TR)
        run(nm, su, ctl);
        g_row_extra = 0; g_tick_hook = NULL; g_route_rows = was;
    };
    yard_t t6c = {};
    for (float b : std::initializer_list<float>{0.0f, 5.0f}) {
        set_param("crest_band_kmh", b);
        printf("\n--- crest_band_kmh = %.0f ---", b);
        run("T0 40 t cruise 80, 150 m at 3 % hump, route", {40000, 80, 12, 110, 3, 0.5f, 0, -1, -1, r_hump3, 1}, c_cci80);
        gearbox_run("T0b 40 t cruise 80, 400 m at 1.5 %, route", {40000, 80, 12, 110, 3, 0.5f, 0, -1, -1, r_hump15, 1}, c_cci80);
        if (b > 4) { must[0].shifts = shifts; must[0].hunts = hunts; }
        run("T0c 26 t cruise 80, 400 m at 1.5 %, route", {26000, 80, 12, 110, 3, 0.5f, 0, -1, -1, r_hump15, 1}, c_cci80);
        run("T1 40 t cruise 80, 400 m at 3 % then flat, route", {40000, 80, 12, 120, 3, 0.5f, 0, -1, -1, r_short3, 1}, c_cci80);
        run("T2 40 t cruise 80, 300 m at 5 % then flat, route", {40000, 80, 12, 120, 3, 0.5f, 0, -1, -1, r_short5, 1}, c_cci80);
        run("T3 40 t cruise 80, 2 km at 4 %, route (must downshift)", {40000, 80, 12, 160, 3, 0.5f, 0, -1, -1, r_long4, 1}, c_cci80);
        run("T2b 40 t cruise 80, 800 m at 3 %, route", {40000, 80, 12, 140, 3, 0.5f, 0, -1, -1, r_mid3, 1}, c_cci80);
        gearbox_run("T4 12 t cruise 80, 300 m at 5 %, route", {12000, 80, 12, 120, 3, 0.5f, 0, -1, -1, r_short5, 1}, c_cci80);
        if (b > 4) { must[1].shifts = shifts; must[1].hunts = hunts; }
        gearbox_run("T6 40 t cruise 80 on rolling hills +-1.5 %, route", {40000, 80, 12, 300, 3, 0.5f, 0, -1, -1, r_hills, 1}, c_cci80);
        if (b > 4) { must[2].shifts = shifts; must[2].hunts = hunts; }
        gearbox_run("T6b same, Eco", {40000, 80, 12, 300, 10, 0.5f, 0, -1, -1, r_hills, 1}, c_cci80);
        if (b > 4) { must[3].shifts = shifts; must[3].hunts = hunts; }
        gearbox_run("T6c 40 t cruise 80, hills +-2.5 % every 1 km, route", {40000, 80, 12, 300, 3, 0.5f, 0, -1, -1, r_hills25, 1}, c_cci80);
        if (b > 4) t6c = yard_snap();
        run("T5 40 t full pedal 55, 5 % to a top at 700 m, route", {40000, 55, 10, 70, 3, 0.5f, 0, -1, -1, r_crest, 1}, c_full_road);
        for (float gg : { 5.8f, 6.5f, 7.0f, 7.5f, 8.0f }) {
            g_dipg = gg; char nm[160];
            snprintf(nm, sizeof nm, "H1 30.4 t cruise 73, %.1f %% climb with a %.1f %% dip 60 m long (ride 3: 8 -> 9 -> 8), Eco, NO route", gg, gg - 1.7f);
            run(nm, {30400, 60, 9, 150, 10, 1.0f, 0, -1, -1, r_dip, 0}, c_cci73);
            run("H1b same WITH route", {30400, 60, 9, 150, 10, 1.0f, 0, -1, -1, r_dip, 1}, c_cci73);
        }
    }
    printf("\nG1c rows (the game's own cruise on a gentle hill, shifts not up against the base 43f2524, 0 hunting pairs):\n");
    for (auto &m : must) {
        if (m.shifts <= m.target && m.hunts == 0) printf("  %-4s ok: %d shifts (<= %d), no hunting pairs\n", m.name, m.shifts, m.target);
        else { printf("  %-4s FAIL: %d shifts (<= %d), %d hunting pairs (expect 0)\n", m.name, m.shifts, m.target, m.hunts); g_x_failed = 1; }
    }
    // T6c is judged on the owner's yardstick (yard_verdict): the game's own cruise on hills of +-2.5 % every km, which 12th climbs with a sag of 5 km/h and cannot hold at the top. Against int-1011 (799bae4):
    // 15 shifts, two of them 1.0 s apart on a climb, 42.62 L, 6743 m, 12 at 5 % of the holds. (Before 10-11 the row asked for at most 2 shifts, which only the route's crest band delivered.)
    {
        yard_t ref = {}; ref.fuel = 42.62; ref.dist = 6743; ref.shifts = 14;   // (14 shifts: the base 43f2524's own, K2 step 1: not up against it)
        if (!yard_verdict("T6c", t6c, ref, false)) g_x_failed = 1;
    }
    set_param("crest_band_kmh", getenv("BAND0") ? 0 : 5);
    // PCC momentum: only what the next climb needs (Damien 11-10: solo differs from loaded). Foot of the climb 500 m ahead.
    {
        float (*roads[])(double) = { r_hump15, r_short3, r_short5, r_mid3, r_long4 };
        const char *rn[] = { "400 m 1.5 %", "400 m 3 %", "300 m 5 %", "800 m 3 %", "2 km 4 %" };
        printf("\nPCC need (km/h over set 80 at the foot):\n");
        float *la_need = lookahead_val(); const float la_was = la_need ? *la_need : 0.0f;
        if (la_need) *la_need = 1.0f;   // the climb the cruise is told about is the route layer's (the ticks after run() are the table's own, outside the row's key)
        for (float mass : { 8000.f, 12000.f, 26000.f, 40000.f }) {
            printf("  %2.0f t:", mass / 1000);
            for (int r = 0; r < 5; r++) {
                run("  (need)", {mass, 80, 12, 40, 3, 0.5f, 0, -1, -1, roads[r], 1}, c_cci80);
                M.s = 1000;  // after 40 s on the flat (the force bias has settled): the foot of the climb 500 m ahead
                for (int i = 0; i < 40; i++) tick(1 / 60.0, 0);  // the frame works the need out every 0.5 s
                printf("  %s %.1f", rn[r], g_pcc_need_kmh);
            }
            printf("\n");
        }
        if (la_need) *la_need = la_was;
    }

    // Robustness (polish round): resets in the middle of a drive must not break the drive; unsupported trucks stay hands-off.
    g_rst_every = 7; g_rst_level = RESET_SOFT;
    run("X1 40 t holds 80 on rolling hills, SOFT reset every 7 s (pause menu)", {40000, 80, 11, 240}, c_hills);
    g_rst_every = 11; g_rst_level = RESET_HARD;
    run("X2 same, HARD reset every 11 s (teleport / load: learned values gone)", {40000, 80, 11, 240}, c_hills);
    g_rst_every = 0;
    {
        scs_telemetry_configuration_t cfg = { "truck", NULL };
        T.fwd_gears = 2;
        m_trueshift.config(&cfg);
        run("X3 electric / 2-gear truck: TrueShift must leave the gears to the game", {40000, 80, 2, 60}, c_full);
        T.fwd_gears = 12;
        sim_reset(RESET_HARD, "sim: back to the diesel");
    }
    float st;
    run("L1 13.4 t, Eco, game starts in 4 at 12 km/h, full throttle (ride 12-10 1 > 4 > 5 > 6)", {13400, 12, 4, 50, 10.0f, 1.0f}, c_full);
    run("L2 13.4 t, Eco, from 14 km/h in 3, light throttle 0.4", {13400, 14, 3, 50, 10.0f, 1.0f}, c_light);
    // The start-gear checks are from a standstill (else the engage-at-speed path answers): L1/L2 end at ~70 km/h.
    // s_start reads the pedal from T (set by the physics step only), so set it here too.
    M.v = 0; T.speed = 0; T.in_thr = 0.5f;
    m_trueshift.frame(); M.mass = 26000; M.grade_pct = 0; M.pedal = 0.5f; st = (float)s_start(SELF); printf("\nstart gear 26 t flat: %g (game 1)\n", st);
    M.mass = 40000; M.grade_pct = 8; M.pedal = 0.5f; st = (float)s_start(SELF); printf("\nstart gear 40 t on 8%%: %g (game 1)\n", st);
    M.mass = 10000; M.grade_pct = 0; st = (float)s_start(SELF); printf("start gear 10 t flat: %g (game 4)\n", st);
    M.mass = 10000; M.grade_pct = 10; st = (float)s_start(SELF); printf("start gear 10 t on 10%%: %g (game 4)\n", st);
    // Ride 13-10 (617 s, 2087 s): 25 t, full pedal, downhill: 4->1 (start) at 12 km/h, 2000 rpm, then 1->4 (rev limit skip).
    M.mass = 25000; M.grade_pct = 0; T.in_thr = 1.0f; st = (float)s_start(SELF); printf("\nstart gear 25 t flat, full pedal: %g (game 1)\n", st);
    M.grade_pct = -4; st = (float)s_start(SELF); printf("start gear 25 t on -4%%, full pedal: %g (game 1)\n", st);
    M.v = T.speed = 12 / 3.6f; st = (float)s_start(SELF); printf("engage 25 t on -4%%, full pedal, rolling 12 km/h: %g (game 1; expect a gear near 1000-1300 rpm, not 1)\n", st);
    M.v = 0; T.speed = 0; M.mass = 10000; M.grade_pct = 10; T.in_thr = 0.5f; (void)s_start(SELF);  // back to the state the car runs started from

    // m_style 0 European (no shift of our own) and 1 protected (rev limit, stall only)
    for (float st : std::initializer_list<float>{0.0f, 1.0f}) {
        set_param("m_style", st);
        printf("\n--- m_style = %.0f ---", st);
        run_m();
    }
    set_param("m_style", 0);

    use_renault();
    run("C10 Renault 14 gears 32.5 t, Eco, cruise 46 (ACC behind a slow lead), flat then a 10.5 % climb of 300 m from 43 km/h in 12 (ride 24-10, 545 s)", {32500, 43, 12, 45, 10.0f, 1.0f, 0, -1, -1, r_climb10, 1}, c_cc46);
    run("Y1 Renault 16.4 t yard: 13 km/h in 6, a short brake, then pedal 0.3 (ride 24-10, 3856 s: 6>4>5>3>4>5>6>7>8>9 in 21 s)", {16400, 13, 6, 25, 10.0f, 1.0f, 0, -1, -1, r_flat, 0}, c_yard);
    run("C11 Renault 32.5 t, Eco, cruise 52 (ACC), flat then a 6 % climb of 400 m from 50 km/h in 13: a gear that can just hold it", {32500, 50, 13, 50, 10.0f, 1.0f, 0, -1, -1, r_climb6, 1}, c_cc52);
    run("C12 same on a long 3 % climb of 600 m from 52 km/h in 13", {32500, 52, 13, 60, 10.0f, 1.0f, 0, -1, -1, r_climb3, 1}, c_cc52);
    run("C10b same from 55 km/h in 13", {32500, 55, 13, 45, 10.0f, 1.0f, 0, -1, -1, r_climb10, 1}, c_cc63);
    // start gear of the Renault box by load (24-10: 36.7 t started in 2 and climbed 2>3>4>5 in 7 s)
    {
        M.v = 0; T.speed = 0; M.grade_pct = 0;
        int follows = 0;
        for (float tons : { 16.4f, 25.0f, 34.6f, 36.7f, 42.0f }) {
            int light = 0, full = 0;
            for (float ped : { 0.06f, 0.25f, 0.6f }) {
                M.mass = tons * 1000; M.pedal = ped; T.in_thr = ped; m_trueshift.frame();
                int g = (int)s_start(SELF);
                printf("Renault start gear %4.1f t, pedal %.2f: %d (game 1)\n", tons, ped, g);
                if (ped == 0.06f) light = g;
                if (ped == 0.6f) full = g;
            }
            follows += light < full;
        }
        // the start gear follows the pedal: a light pedal launches in a lower gear (30-10 ride: the Scania started in gear 5-6 at pedal 0.06)
        printf("start gear follows the pedal on %d/5 loads (light pedal lower than a firm one)\n", follows);
        if (follows < 2) { printf("  start gear pedal check FAIL\n"); g_x_failed = 1; }
    }
    run("G1 Renault 36.7 t at a standstill, the game started in 2, pedal 0.3 (24-10, 324 s: 2>3>4>5 in 7 s)", {36700, 0, 2, 25, 10.0f, 1.0f, 0, -1, -1, r_flat, 0}, c_pull03);
    use_car();
    run("CAR1 2.5 t pickup, light 0.35 from 15 km/h", {2500, 15, 2, 40}, c_car_light);
    run("CAR2 2.5 t pickup, full throttle from 15", {2500, 15, 2, 25}, c_car_full);
    run("CAR3 2.5 t pickup holds 100", {2500, 100, 6, 60}, c_car_100);
    // Reset scenarios, last: they move the clock and the module's once-a-second polls, which would shift the runs above a little.
    use_volvo();
    sim_reset(RESET_HARD, "sim: the Volvo again");
    g_tp_done = 0; g_tp_up_at = -1;
    {
    run("X4 40 t full pedal on 6 %, at 20 s a teleport (HARD reset) onto 2 % at the same speed", {40000, 45, 8, 60}, c_teleport);
    // the upshift must come within 10 s (the 6 % peak is gone), and it must hold: demand and f_need start from the pedal of now, not from 0
    // (from 0 the full pedal looked like a light one: a skip to a gear without pull, back down, a hunt incident)
    // (the downshifts from 1.5 s after the teleport on, DEVIATION: the first frame after the teleport takes 8 -> 7 on fresh data (the road view is reset with the
    // session: hold 10660 N, the 2 % resistance): the floored plan finds gear 8 past its crossover with the upshift blocked (the next gear gives less force) and
    // the stronger 7 the better place until 7 -> 9 three seconds later. The old lugging rule took that downshift on the 6 % climb BEFORE the teleport, which
    // is what the zero counted on; R removed the rule. Open: the plan's two terms (the crossover of bands_cross against next_force < cur_force) disagree.)
    int downs_after = shifts - ups - g_tp_downs1;
    // (WP-E: a kickdown no longer lands in the temporary band, so the 8 -> 7 -> 9 of above is gone and gear 8 stays until its own upshift at 50 km/h: 10.3 s of pull at 40 t on
    // 2 % from 40 km/h. The stale 6 % peak the row guards against would have held it ~27 s, so the bound is 12 s: the upshift of the road we are on, not of the one we left.)
    printf("  first upshift %.1f s after the teleport (must be within 12 s); downshifts after it %d, no-pull lessons %d, incidents %d (all 0)\n",
           g_tp_up_at < 0 ? -1.0 : g_tp_up_at - 20, downs_after, n_failed - g_tp_f0, g_n_incidents - g_tp_i0);
    if (!(g_tp_up_at >= 0 && g_tp_up_at - 20 < 12) || downs_after || n_failed != g_tp_f0 || g_n_incidents != g_tp_i0) { printf("  X4 FAIL\n"); g_x_failed = 1; } }
    // X5: the gearbox parked in its gap state (gear 0, a target, clutch open) at 40 km/h, a pause (SOFT reset) 1.0 s into it. The stuck-in-neutral
    // timer must start again at the reset: the engage then comes 0.3 s after the pause (the pause's gap is normal), not at once as 'stuck for 1.0 s'.
    {
        memset(&M, 0, sizeof M); M.mass = 40000; M.shift_time = 0.5f; M.cvar = 3; M.v = 40 / 3.6f; M.gear = 0; M.target = 8; M.parked = 1;
        t_now += 10;
        g_stuck_s = -1; int i0 = g_n_incidents; double t0 = t_now;
        for (int k = 0; k < 120 && M.parked; k++) {
            t_now += 1 / 60.0; T.sim_time = (scs_timestamp_t)(t_now * 1e6);
            T.speed = M.v; T.gear = 0; T.dgear = 0; T.in_thr = 0; T.eff_thr = 0; T.eff_clutch = 1; T.engine_on = 1; T.rpm = 600;
            if (k == 60) sim_reset(RESET_SOFT, "sim: pause menu");
            s_tail(SELF);
        }
        printf("\nX5 gap state at 40 km/h, a pause 1.0 s into it: engaged %.2f s after the pause as 'stuck for %.1f s' (expect ~0.3 and 0.3), incidents %d (expect 0)\n",
               t_now - t0 - 1.0, g_stuck_s, g_n_incidents - i0);
        if (!(g_stuck_s >= 0 && g_stuck_s < 0.5f && g_n_incidents == i0 && !M.parked)) { printf("  X5 FAIL\n"); g_x_failed = 1; }
    }
    g_x_failed |= run_fbias();
    g_x_failed |= run_fuel();
    g_x_failed |= run_g8();
    return g_x_failed;
}
