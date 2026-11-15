// Layer 3 of the powertrain (design: Damien 01-11/12, "Aandrijflijn in lagen"): the gear.
// TrueShift owns the hard limits and passes gc_choose the gears it allows (gc_in.allowed, bit g); here the gear is
// chosen with ONE score over the allowed gears, lowest wins, and EVERY term is seconds of drive time over a common
// horizon (the next ~20 s, or until the next upshift). It knows no TrueShift state and no hard rule beyond the
// engine's own rpm window and the wish: a pure function of the vehicle model, the intent and a few numbers, so a
// simulator can call it directly.
//
//   candidate  allowed (bit g) AND the rpm the gear runs at (after the shift time's speed loss for a target, at now
//              for the current gear) inside [bands.floor, bands.cap] AND the gear delivers the wish within the
//              intent's share (wish_n <= max_share x its full-throttle force; land shares for a target, run shares
//              for the current gear). No candidate at all: the gear with the most force (a kickdown).
//   score(g)   = shift + accel + band + expect + fuel + target - stay, all in seconds:
//                shift  = shift_time for every shift, now and the ones the gear needs before the wished speed (the
//                         gear's range to the next upshift), so a skip earns back the shifts it saves; a shift back
//                         to the gear just left within the dwell costs double (the caller's dwell rules stay hard).
//                         With the pedal off the shift now costs the drive it loses if the pedal returns during it
//                         (lift_scale = gc_lift_return, times the price gc_expect charges a later shift).
//                accel  = the extra time the gear needs to reach the horizon speed vs the fastest gear (a gear that
//                         pulls less, or lugs because its force is under the wish, reaches the speed later).
//                band   = the intent's per-band cost (land for a target, run for the current gear), already in seconds.
//                expect = the seconds the gear is expected to lose to the demands still to come (gc_expect): the wish now,
//                         the road ahead with the time each stretch arrives, and with the pedal off the demand that was lifted.
//                fuel   = the mode's seconds per litre x the litres the gear burns over the time it is expected to be held, less the cheapest
//                         candidate's (gc_fuel_litres); with the pedal off the game cuts the fuel in gear, so only the share of the hold in which the
//                         demand is back burns any.
//                stay   = a tiny bonus for the current gear (tie-breaking; anti-hunting is the double shift + the
//                         caller's hard dwell), and in a bend one shift duration per unit of the lateral load: no shift mid-bend.
//   target     the pedal is an rpm target (WP-0911-K1, intent.cpp tgt_rpm): from calm revs at a light pedal up to the mode's full-pedal point, P3 at the end of the driver's pedal (the kickdown).
//              One more term of the score, tgt = the value of the path the gear takes (gc_path_solve in gearchoice.cpp): the next GC_HORIZON seconds in steps of time and speed, the miss of every second in
//              gear steps (one step off the point costs one second per second), every shift on the way ts, the speed from the force the gear gives at this pedal. There is no separate regime for a full pedal.
//   upshift    the current gear's rpm reaching bands.cap - rpm_rate x latency forces the best qualifying upshift.
#pragma once
#include "powertrain.h"
#include "vehicle.h"

enum { GC_MAX = 32, DEMAND_LEVELS_MAX = 8 };

// The seconds from the moment a limiter wall is seen to the revs being there (the limiter wall is anticipated by rpm_rate x this): the one latency the layers
// and the downshift rules' landing wall (TrueShift landing_wall) share.
static const float GC_LATENCY_S = 0.25f;
// How old a standing decision of the layers may be before the whole score is made again, in game seconds (TrueShift keeps the last choice in between and decides at once on an event: a press, the brake, a shift
// done, the pedal on or off, the mode). The limiter wall is anticipated by GC_LATENCY_S (the rpm rise over it is the margin); a choice at most a quarter of that old costs at most a quarter of the margin, and a
// shift takes longer than this to act on a choice (shift_time 0.3 s for a converter, 0.9 s for a clutch box), so deciding faster gains nothing: the cost of the choice no longer depends on the frame rate.
// (K1 round 14: no floor. A floor of GC_LATENCY_S / 4 halved the hunts by timing alone, a debounce and not a price, and split a near-tie into two states by the frame spacing
// (run_pcc crest Renault 40 t Eco, brake energy 730 or 768 kJ); without it the score is made every frame, as 182607e, unless what a decision costs on this PC asks for room: GC_CPU_BUDGET.)
static const float GC_DECIDE_S = 0.0f;
// The share of the game thread's time the whole score may take, on average. A CPU budget (a frame of a weak PC is not given to the gearbox), not a driving threshold: nothing the truck does is judged against it.
// TrueShift measures what a decision costs on this PC (the median of the last decisions) and, where cost / GC_CPU_BUDGET is longer than GC_DECIDE_S, makes the decisions that much further apart.
static const float GC_CPU_BUDGET = 0.01f;
// The seconds of drive the score's terms cover (the plan horizon). The demand memory of a lift (TrueShift) fades over it too.
static const float GC_HORIZON = 20.0f;

struct gc_in {
    int cur;                    // the gear now
    int prev;                   // the gear we shifted out of to reach cur, while the caller's dwell still runs (0 = none):
                                // a shift back to it costs its shift time twice (anti-hunting inside the score) times prev_w (below)
    int skips;                  // most gears an upshift may skip (TrueShift's max_skip): the shifts a gear needs later
                                // are counted as skips, so a gear that waits to skip farther is not under-valued
    int n;                      // forward gears
    float v;                    // m/s
    unsigned long long allowed; // bit g = the caller's hard limits allow gear g (1..n)
    pt_intent intent;           // layer 2's intent
    eng_bands bands;            // the engine bands
    float shift_time;           // s (the caller passes 0.3 for a torque converter until it is measured)
    float rpm_rate;             // rpm/s now (how fast the revs rise in the gear)
    float latency;              // s, 0.25 default: the limiter wall is anticipated by this much
    float shift_dur = 0;        // s from the decision to the gear being in (the caller learns it; 0 = shift_time): the speed lost in the
                                // shift, and so the rpm a gear lands at, is measured over this, not over the shift time alone
    float shift_decel = 0;      // m/s2 lost in the shift on top of the resistance (the caller learns it; crawl speeds lose more than f_res/m)
    const ahead_view *road = nullptr;   // the stretches ahead with the time each arrives (ahead.h n_samples): the demands gc_expect looks at; null = none known
    // The pedal is off (the layers run on the demand that is still expected): lift_n is that demand (N; the largest wish of the pedal on, fading over the plan
    // horizon), lift_p the probability that it comes back within the horizon (the driver's history: demand_p of its level, the uniform prior until he proves calm),
    // lift_scale the expected drive a shift now loses: the chance that the pedal returns during it, 1 while the lift is shorter than a shift (a lift of that length
    // cannot be used for one, the owner's rule), falling with the shift duration as the time constant after it (the caller keeps the clock). Everything that
    // only happens when the demand is back (the band taste, the shifts of the plan, the reserve) weighs lift_p. lift_n 0 = the pedal is on.
    float lift_n = 0, lift_p = 0, lift_scale = 1;
    // How long the band taste is expected to be paid, as a multiple of the plan horizon: 1 normally; a speed the cruise holds with the driver's foot off goes on for as
    // long as such a hold does, and the scale-free survival of a hold says that is about as long again as it has lasted (1 + age / horizon). The same for the pedal's
    // hold, by the age of the gear (the time since the last shift, WP-G1b); the caller caps the age and ends it at a net descent ahead. The fuel term reads it too.
    float band_scale = 1;
    // The foot is off and a controller holds (the cruise): the history is what the DRIVER may ask next, and the wish is the controller's throttle, which wavers around the
    // holding force; weighting the history by how much of that wish holds the speed (hold_share) made two gears a few hundredths apart trade places with the throttle's
    // noise (T0, the hump, 12 <-> 11 on the flat). With the foot off the whole history counts, as in a lift, scaled by the chance the foot is back (the caller).
    int hist_full = 0;
    // The lateral load (0..2): the share of the truck's own bend allowance the truck uses or, by its steering, is about to use. The gear we are in gets
    // lat_load shift durations of stay bonus: no shift mid-bend, the same for a roundabout as for a bend, with or without a route.
    float lat_load = 0;
    // The time (s) a gear landed in a pull should be held before it is left upward again, or the shift is a hop (WP-G7): the caller's anti-hunt dwell, the time its own shift logic needs to
    // tell a hold from a hunt. A landing that the pull takes to the end of its range sooner pays its shift time for the part it falls short of (gc_choose, the plan's shift term).
    float hold_s = 3.0f;
    // The driver's history (core/demand.h): with the pedal on, the largest demand to come in the horizon is one of these levels (hist_f: N at the wheels at
    // this speed), with these probabilities (hist_p, mutually exclusive: the chance that the largest is that level and not the next one up). A fresh session
    // has the uniform prior, which asks a reserve like the old cruise stand's, and relaxes as the driver proves calm.
    int hist_n = 0; float hist_f[DEMAND_LEVELS_MAX], hist_p[DEMAND_LEVELS_MAX];
    // The torque pedal's phi (core/wish.h wish_phi): the level above (intent.wish_n) is one number for all gears and times the horizon, a gear is asked F_d = level x
    // phi(rpm of the gear). Qualification ("g delivers its ask") and the demands of gc_expect (the demand now exactly, the pedal part of the road ahead and of the
    // history on top of the resistance of its moment) use the gear's ask. The plan's pull (gc_gain, accc) and the horizon stay on the level: the driver asks an
    // acceleration and adapts the pedal, and timing every gear on its own F_d books a 10 % pull edge of a gear at higher revs over the whole horizon, which paid for a
    // downshift that the wall undid two seconds later (tried: 18c 32 hunts, the roundabout and 18f rows reverse). null = phi 1 everywhere: a controller (the cruise,
    // the planner) wants a force, it has no foot.
    float (*phi)(float rpm) = nullptr;
    // The foot a gear would have for the same wish (null = the foot of now in every gear): the torque pedal asks level x phi(rpm), so the same force needs another raw foot in each gear. foot_for(foot, k) is the
    // raw pedal whose shaped share is k times the share of `foot` (the caller owns the pedal's map). A candidate gear reads the pedal's point at the foot it would have, k = phi(rpm now) / phi(rpm of the gear), as far
    // as the decision is a steady hold of the speed (a hold part that grows from the half of hold_share, gc_choose): the force asked of a hold is one fact and a gear must not change it (core value 4); a pull keeps the driver's foot.
    float (*foot_for)(float foot, float k) = nullptr;
    // The wish is a controller's (the cruise, the planner), not a driver's foot. What a controller asks above the force that holds the speed is the throttle of the moment
    // (it climbs from 0 after it is engaged, wavers around the holding force), not a demand that will still be there when the road ahead comes: the road's demands are the
    // force that holds the speed at their moment, nothing added. A driver's pull is his own, and the road adds to it.
    int controller = 0;
    // The wish is the throttle of the GAME's own cruise with the foot off (WP-G1b): a proportional controller whose throttle rises with the speed error, so the speed a shift
    // loses is won back at what the gear has to spare, not at the excess its throttle has over the holding force (that is the price of a shift in gc_choose). The plugin's own
    // cruise and the planner (controller) ask a force they plan, and keep the timing of their own excess.
    int game_cruise = 0;
    // The wish is held under a speed limiter (WP-0911-K1): the foot asks for more than the limiter lets through, and what is left is the force that closes the gap to the limit. That small pull is the
    // limiter's, not the driver's: a shift's lost speed is a dip below the limit (a distance, priced as the later shifts are), not a speed to reach (a duration at the spare of the gear: 5 .. 14 s,
    // which kept 10 at 1795 rpm for 30 s where 11 holds the limit, ride 08-11).
    int capped = 0;
    // How much of the anti-hunting price is due (0..1, WP-G8): the shift back to `prev` costs its shift time twice only while the situation is the one the last shift was made in; the caller
    // says how much of it is still the same (core/shiftcommon.h sc_reversal_weight). 1 = the same situation (a hunt): the full price.
    float prev_w = 1.0f;
};

struct gc_term {
    int g;                      // the gear
    int allowed, in_win, qualifies, candidate;
    float rpm, force;           // engine rpm (after the shift) and full-throttle force (N)
    float pull;                 // what the gear really gives at the foot: the full-throttle force, but the foot's own pull where the converter slips (the torque pedal's throttle passes the pedal through there)
    int band;                   // pt_band_of(rpm)
    float bandc, shiftc, accc, expc, fuelc, tgtc, stay, score;   // the score terms, all seconds (tgtc: the value of the path the gear takes to the pedal's operating point, WP-0911-K1: the miss in gear steps, the band taste and the fuel as rates, every shift on the way)
    float fuel_l;               // the litres the gear burns over the time it is expected to be held (the fuel term is the weight times the excess over the cheapest candidate)
    int shifts;                 // shifts counted (now + the ones the gear needs later)
    float ask_n;                // observation only (tools/sim/sim_promises.cpp, promise P4): what the gear was asked at qualification, gc_ask(level, rpm after the shift); nothing reads it
};

enum { GC_KEEP = 0, GC_UP, GC_DOWN, GC_KICKDOWN, GC_WALL, GC_NEAREST };

struct gc_out {
    int n;                      // terms filled (1..n)
    gc_term t[GC_MAX + 1];      // indexed by gear
    int best;                   // the gear chosen (0 = none)
    int why;                    // GC_*
    int skip;                   // gears skipped on an upshift (0 = single step)
    float level_n;              // observation only: the level (the demand, N) the candidates were asked with ...
    int lift, phi_on;           // ... the pedal was off (the asks are 0) / the asks carry phi of the gear's rpm (the torque pedal) and not the level itself (a controller)
    float hold_share, hold_n, band_scale, urgency, need;   // observation only (WP-G6, the sim's ride replay and the level-2 log line): what the terms were weighted with, from the intent and the caller
    int prev;                   // observation only (WP-G6c round 2): the gear the caller said it had just left (gc_in::prev), 0 = none
    float prev_w;               // observation only (WP-G8): the share of the anti-hunting price the caller said is due (gc_in::prev_w)
    float cell_v;               // the speed (m/s) one cell of the path's grid spans, 0 = no path was solved (K1 round 14): the choice was made around a cell of the speed, and the truck having moved a whole cell is another choice
    float shift_ts, prev_price; // observation only (WP-G8): the shift time the decision was made with, and the seconds the score charged the gear `prev` for going back to it (0 = not charged)
};

int gc_choose(const gc_in *in, gc_out *dbg);
// The landing of the rev-limit rule's upshift at a full pedal: the gear up to `span` above cur that the fastest way to the horizon speed goes to (cur + 1 when none lands inside the window).
// tgt_rpm: the pedal's operating point (intent.tgt_rpm, 0 = none): the landing is priced by its distance from it, as in the score.
int gc_full_upshift(int cur, int n, float v, float shift_time, float shift_dur, float shift_decel, int span, float tgt_rpm = 0.0f, float tgt_w = 1.0f);
// (tgt_w: the share of the target that counts, 1 - the kickdown: at the end of the pedal the plan's time alone picks the landing)
// Observation only (read-only; tools/sim/sim_promises.cpp, promise P4): the last decision of TrueShift's layers (modules/trueshift.cpp) and when it was made (sim_s()); null before the first.
const gc_out *ts_last_choice(double *at);

// A demand the gear is asked to deliver: a force F (N at the wheels) tau seconds from now, against the resistance R then, with probability p.
// share > 0 overrides the gear's own share (the lugging allowance does not apply to a reserve: 1 = full throttle).
// excl 1 = this demand and the ones right after it with excl 1 are mutually exclusive events (the driver's history: the largest demand of a window is one level
// or another): they add up by their probabilities, and only afterwards the shift they may have needed stops the later demands.
struct gc_demand { float tau, f, r, p, share; int excl; };
// The seconds gear g is expected to lose to the demands d[0..nd), taken in the order given. fg = the gear's full-throttle force (N) at the rpm it
// runs at, fbest the strongest force any gear in the window gives at this speed, share the gear's lugging allowance (intent.max_share), dur the
// seconds a shift takes, decel the m/s2 it loses beyond the resistance, m the mass. A demand the gear cannot deliver (F over share x fg, ramped over
// GC_SHARE_BAND so that a demand 1 % over the force costs a tenth of the shift, not all of it) costs the shift it will need then, dur + the speed
// the gap loses (at that moment's resistance) regained at the pull of the strongest gear, if some gear can deliver it; only the first failing
// demand pays (after the shift the later ones are the new gear's business, weighted by the probability that this one came). A demand it delivers
// inside the allowance but at a lower acceleration than the demand asks costs the lag over one shift: dur x (1 - a_gear / a_demand).
// v: the speed (m/s) the shifts are priced at (the distance a dip costs is a time at the speed).
// A gear that fails a demand is left for another one then, so the band taste it is charged for the whole horizon is its own only until then: succ_band[k] is the band cost
// of the cheapest gear that does deliver demand k (>= 1e29: none), own_band this gear's, band_h the time the band cost is charged over (s); band_after gets what the
// taste changes by (the gear's own for the stretch after the shift becomes the successor's: probability x fail x the share of band_h after tau x (own - succ)).
float gc_expect(float fg, float fbest, float share, float dur, float decel, float m, const gc_demand *d, int nd, float v = 20.0f,
                const float *succ_band = nullptr, float own_band = 0.0f, float band_h = 0.0f, float *band_after = nullptr, float (*stay_fn)(void *) = nullptr, void *stay_ctx = nullptr, float v_hi = 0.0f, float stay_q = 1.0f);

// The chance that the drive a shift would lose is needed during it: the lift (the pedal off) that has lasted tau seconds ends in a re-press within the window w
// (a shift and the time from the decision to the clutch opening); shift is the shift's own duration, the unit of the profile's prior. The score's pedal-off shift
// cost is this times the price of a shift (lift_scale = gc_lift_return(tau, dur + GC_LATENCY_S, dur)). Until the lift has lasted the window it is 1: a lift
// no longer than a shift cannot be used for one (the owner's rule, 02-11). After it, core/driver's driver_lift_return_p: the scale-free prior (shift / (shift + t),
// halved for re-press against brake), learned from this driver's lifts. So a held lift is judged at the window: the saving of the shift against about 1/6 of its
// price, and falls due 1 to 1.5 shift durations after the lift for the savings of the rows measured (18i-4).
float gc_lift_return(float tau, float w, float shift);

// The fuel the engine burns while a demand is delivered: the game burns a x omega x (T_net + T_f) (core/vehicle.cpp, the learner), T_net the work torque and T_f = k x
// peak torque the friction. At the wheels the work is the demanded force x the speed / the driveline efficiency, the same in every gear that can deliver the demand;
// what differs between gears is the friction, omega x k x peak: the lower the rpm, the less. hold_s is the time the gear is expected to be held, on the share of
// it in which the engine is in fuel (1; with the pedal off the game cuts the fuel in gear and only the share in which the demand is back counts). The litres are in
// real units, so the seconds per litre of a mode mean the same on every map: the shaft work of a litre is the diesel's lower heating value 35.8 MJ/L x the
// indicated efficiency of the Willans line the game's model follows, 0.45 (the friction is the intercept of the line, so the efficiency is the slope's).
float gc_fuel_litres(float rpm, float k, float peak_nm, float demand_n, float v, float eta, float hold_s, float on);
// The crossover of gear g to g + 1 at the speed v: the end of the gear's power band (bands_cross with the step of the two ratios).
// The top gear gives the cap (its power band runs to the end).
float gc_cross(const eng_bands *b, int g, int n, float v);
// The band's short name, for the log.
const char *gc_band_name(int band);
