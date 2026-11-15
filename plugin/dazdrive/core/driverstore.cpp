// The driver profile, the thin part (core/driver.h): the sample from the sense and the model, the file, the save timer, the log lines. Everything that
// decides lives in driver.cpp and demand.cpp, which tools/sim/test_driver.cpp tests alone.
//
// One file per game profile, in the plugin's output folder next to roll_gain.txt and fuel_learn.txt: driver_<profile>.txt, versioned plain text (driver.h).
// Read on the first frame and when the profile changes, written every DRV_SAVE_S while something changed, on a hard reset and at shutdown. A missing, old or
// damaged file is the prior: the plugin starts as a truck with no previous driver, which is what it did before there was a file.
//
// THE PROFILE'S NAME: nothing the plugin can reach names the game's profile. The telemetry SDK has no profile channel (its common parameters are the game,
// its id and version; the truck configuration has a license plate, which belongs to a truck, not to a player), core/game.cpp has no reader for it, and
// the other learned files (roll_gain.txt, brake_gain.txt, fuel_learn.txt, steer_profile.txt) are keyed by truck or wheel, not by player. So the key is "":
// ONE file, driver_profile.txt, and a second player on the same installation shares it. driver_set_profile() is the place a reader would call when one
// exists (the game's profile folder name read from its memory, say): it saves the old profile's file, forgets, and loads the new one's.
#include "driver.h"
#include "demand.h"
#include "suite.h"
#include "vehicle.h"
#include "wish.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Seconds between saves while the profile changes (it changes every second while the driver drives: a note). A crash then loses at most a minute, a tenth
// of the demand memory, and the file is a few kilobytes: 60 writes an hour.
static const double DRV_SAVE_S = 60.0;

static struct {
    char key[48];          // the game profile ("" = unknown)
    int loaded;            // the file was looked at for this profile
    double save_at;        // wall_s() of the last save (or of the load)
    unsigned saved_gen;    // driver_gen() at the last save
} D;

static void file_name(char *out, size_t n)
{
    if (D.key[0]) snprintf(out, n, "driver_%s.txt", D.key);
    else snprintf(out, n, "driver_profile.txt");
}

static void load()
{
    D.loaded = 1;
    D.save_at = wall_s();
    char name[80]; file_name(name, sizeof name);
    FILE *f = open_out(name, "r");
    if (!f) { logv(1, "Driver profile: nothing stored (%s), starting from the priors", name); D.saved_gen = driver_gen(); return; }
    static char buf[16384];
    size_t n = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n] = 0;
    if (driver_parse(buf)) {
        char sum[160]; driver_summary(sum, sizeof sum);
        logv(1, "Driver profile: loaded from %s: %s", name, sum);
    } else logv(1, "Driver profile: %s is old or damaged, ignored; starting from the priors", name);
    D.saved_gen = driver_gen();
}

static void save(const char *why)
{
    if (!D.loaded) return;
    static char buf[16384];
    size_t n = driver_serialize(buf, sizeof buf);
    D.save_at = wall_s();
    if (!n) return;   // (it cannot outgrow the buffer: the rings are bounded)
    char name[80]; file_name(name, sizeof name);
    FILE *f = open_out(name, "w");
    if (!f) { static int warned; if (!warned++) logv(1, "Driver profile: cannot write %s (the profile stays in memory)", name); return; }
    size_t w = fwrite(buf, 1, n, f);
    fclose(f);
    if (w == n) {
        D.saved_gen = driver_gen();
        char sum[160]; driver_summary(sum, sizeof sum);
        logv(2, "Driver profile: saved (%s): %s", why, sum);
    }
}

// The frame the learners see, from the sense and the model: the driver's own controls (the raw pedal and brake, not what the cruise or an owner makes of
// the throttle), the speed, the mass, and whether the driver's pedal is what drives.
static driver_sample sample_now()
{
    driver_sample s;
    s.t = sim_s();
    s.pedal = T.in_thr;
    s.brake = T.in_brake;
    s.v = fabsf(T.speed);
    s.mass = V.mass_ok ? V.mass : 0.0f;
    s.gear = T.gear;
    // (the same tests modules/pedal.cpp passes() makes before it shapes the pedal: engine on, not paused, no cruise, no crawl, no launch control)
    s.own = T.engine_on && !T.paused && !(T.cruise > 0.1f) && !g_lv.crawl_toggle && !g_lv.lcx;
    s.launch_share = wish_launch_share(T.in_thr);
    return s;
}

void driver_frame()
{
    if (!D.loaded) load();
    demand_tick(sim_s(), wish_share_eff());
    // The controller's own memory (core/demand.h, this session only, never in the profile): what the cruise asked while the foot was off, as a share of a full wish: the plugin cruise's wish when it has
    // one, else the throttle the game's cruise puts in. NAN (not recorded) when no controller drives.
    {
        float cs = NAN;
        if (T.engine_on && !T.paused && T.cruise > 0.1f && T.in_thr <= 0.02f) {
            const float w = cruise_wish_share();
            cs = w == w ? w : T.eff_thr;
        }
        demand_ctl_tick(sim_s(), cs);
    }
    const driver_sample s = sample_now();
    const int ev = driver_step(&s);
    if (ev) {
        char line[480];
        driver_describe(ev, line, sizeof line);
        if (line[0]) logv(2, "%s", line);
    }
    if (wall_s() - D.save_at >= DRV_SAVE_S && driver_gen() != D.saved_gen) save("timer");
}

void driver_break(int level)
{
    driver_drop_open();   // (a lift, a press or a launch across a pause, a teleport or a load is no event to learn from)
    if (level >= RESET_HARD) demand_ctl_reset();   // (the controllers' memory is this session's and this truck's)
    if (level >= RESET_HARD && D.loaded && driver_gen() != D.saved_gen) save("reset");
}

void driver_stop()
{
    if (D.loaded && driver_gen() != D.saved_gen) save("stop");
}

void driver_set_profile(const char *key)
{
    if (!key) key = "";
    char want[sizeof D.key];
    snprintf(want, sizeof want, "%s", key);
    for (char *c = want; *c; c++)
        if (!((*c >= '0' && *c <= '9') || (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || *c == '-' || *c == '_')) *c = '_';   // (the key becomes part of a file name)
    if (D.loaded && !strcmp(want, D.key)) return;
    if (D.loaded && driver_gen() != D.saved_gen) save("profile change");
    snprintf(D.key, sizeof D.key, "%s", want);
    driver_reset();
    load();
}
