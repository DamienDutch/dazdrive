// The engine bands (core/bands.cpp): where the engine is usable, as named rpm ranges, from three anchors of the torque curve and the
// gearbox instead of fixed shares of torque or power (powerband-change, Damien/Claude 02-11). TrueShift and the dashboard read
// bands_now(); the layers (intent, gear choice) read the band of an rpm through pt_band_of.
//
//   P1  start of the plateau: where the curve first reaches 98 % of its peak torque, interpolated between the points
//   P2  end of the plateau: where it drops under 98 % again (interpolated)
//   P3  the power peak (torque x rpm)
//
//   no-go     idle .. floor           floor = the stall line + the hysteresis (the caller passes both, e.g. ioff(120) + hysteresis_rpm)
//   lug       floor .. P1             how deep a gear may sit here is the expected-time term's price, not a share of the torque
//   eco       P1 .. P2                ONE band: full torque at the lowest revs (the cost inside it grows with the depth, see intent.cpp)
//   power     P2 .. crossover         the crossover of gear g: where the power in g equals the power in g + 1 at the rpm it lands at
//   temporary crossover .. cap        cap = 95 % of the rev limit; only when the time score says it pays (pulling on to skip gears)
//   no-go     cap .. limit
//
// How deep a gear may sit in the lug band is no band function: it is the price the expected-time term puts on the demands a gear cannot deliver
// (core/gearchoice.cpp, gc_expect). The lug band carries no taste cost, only that price.
//
// Everything here is a pure function of engine_info (no V, no T), so the sims can check it.
#pragma once
#include "game.h"
#include <math.h>

static const float BANDS_PLATEAU = 0.98f;   // share of the peak torque that counts as the plateau
static const float BANDS_CAP = 0.95f;       // the top of the temporary band: share of the rev limit (as gs_cross_rpm)

// The power table (K2 step 3): the engine's power at BANDS_PW_N rpms from P1 to P3, over its power at P3 (the peak), made once with the bands (bands_compute) so that the operating point a power asks for
// is a table lookup per decision, not a walk of the curve. A running maximum: the lowest rpm that reaches a power is the first crossing.
static const int BANDS_PW_N = 17;

enum pt_band { PT_NOGO_LO = 0, PT_LUG, PT_ECO, PT_POW, PT_TMP, PT_NOGO_HI, PT_BANDS };

struct eng_bands {
    float idle;                 // the idle the rules use (learned)
    float floor;                // the lug floor: below it is no-go (the stall line + the hysteresis)
    float p1, p2, p3;           // the anchors: plateau start, plateau end (held under cap), power peak
    float cap;                  // 95 % of the rev limit: above it is no-go
    float limit;                // the engine's rpm limit
    float game_low_lo;          // the lower end of the game's R_LOW range (a check only, logged)
    float pw[BANDS_PW_N];       // the power table: pw[k] = the engine's power at p1 + k / (N - 1) x (p3 - p1) over its power at p3, as a running maximum (0..1, pw[N - 1] = 1)
    int ok;                     // 1 = the bands are sane; 0 = a missing / flat curve (all fields 0): the caller keeps its old rules
};

// The anchors and the fixed edges. idle = the idle the rules use (learned), floor_off = the distance of the lug floor above idle
// (the stall line + the hysteresis, by the engine's span when rpm_by_engine is on).
void bands_compute(const engine_info *e, float idle, float floor_off, eng_bands *b);

const eng_bands *bands_now();                   // the current engine's bands, recomputed with the engine (see bands_refresh); ok 0 until the engine is read
void bands_refresh(const engine_info *e);       // recompute + cache (vehicle.cpp on a new engine; TrueShift when idle or floor_off moved), using the values held here
void bands_set_idle(float idle);                // TrueShift: the truck's idle (used by the next bands_refresh)
void bands_set_floor_off(float off);            // TrueShift: the floor's distance above idle (stall margin + hysteresis; used by the next bands_refresh)
int bands_stale();                              // 1 = idle or the floor moved since the last bands_refresh: the caller refreshes again (when it has the engine)

// The crossover of a gear to the next one: step = ratio(g) / ratio(g + 1) (> 1). The first rpm from P2 up where the next gear, at the
// rpm it lands at (rpm / step), gives at least the power of this one; the cap if never. (The same sum as gs_cross_rpm, from P2 instead of
// the top of the game's power range, and without the vehicle model, so it can be computed for any gear without a speed.)
static inline float bands_cross(const engine_info *e, const eng_bands *b, float step)
{
    if (!b->ok || !(step > 1.001f)) return b->cap;
    for (float r = b->p2; r <= b->cap; r += 5)
        if (curve_factor(e, r) * r <= curve_factor(e, r / step) * r / step) return r;
    return b->cap;
}

// The lowest rpm in [P1, P3] whose full-throttle power reaches `frac` of the engine's peak power (the table above, interpolated): P1 for a power the plateau start already gives, P3 for the peak or more.
// The target of the pedal reads this (core/intent.cpp): the operating point that delivers the power the foot and the road ask, no higher.
static inline float bands_rpm_for_power(const eng_bands *b, float frac)
{
    if (!b->ok || !(b->p3 > b->p1)) return b->p1;
    if (!(frac > b->pw[0])) return b->p1;
    for (int k = 1; k < BANDS_PW_N; k++)
        if (frac <= b->pw[k]) {
            const float lo = b->pw[k - 1], hi = b->pw[k], f = hi > lo ? (frac - lo) / (hi - lo) : 1.0f;
            return b->p1 + ((k - 1) + f) / (BANDS_PW_N - 1) * (b->p3 - b->p1);
        }
    return b->p3;
}

// The band an rpm lies in (cross = bands_cross of the gear it is in; cross < 0 = unknown: the power band runs up to the cap)
static inline int bands_of(const eng_bands *b, float cross, float rpm)
{
    if (rpm < b->floor) return PT_NOGO_LO;
    if (rpm < b->p1) return PT_LUG;
    if (rpm < b->p2) return PT_ECO;
    if (rpm < (cross >= 0 ? cross : b->cap)) return PT_POW;
    if (rpm < b->cap) return PT_TMP;
    return PT_NOGO_HI;
}
