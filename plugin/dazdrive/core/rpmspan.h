// [trueshift] rpm_by_engine: the fixed rpm offsets of the shift rules ('idle + 350', 'plateau - 100', the stall margin, the hand-off, the
// engine-brake and air-charge revs) as fractions of the engine's own span, so an engine with a high plateau does not get margins fitted on
// one with a low one.
//
// The offsets were fitted on engines with idle 550-600 rpm and a torque plateau at 900-1000 rpm: a span (plateau - idle) of 300-450 rpm. In
// that band they are used as they are (the factor is exactly 1); outside it they follow the span: N x span / 300 under it, N x span / 450
// over it (continuous, at most x0.5 / x2.5). The revs that are absolute numbers (m_lug_rpm, air_charge_rpm) sit on the plateau: they move
// with the plateau beyond 900-1000 and scale their distance above 900 with the factor; the engine-brake light stage is a distance from the
// low end of the brake range in fractions of that range (the reference range is 1500-1900, the engines the numbers were fitted on).
// With the key at 0 every helper returns its input unchanged.
#pragma once
#include "game.h"
#include "vehicle.h"
#include <math.h>

extern float g_rpm_by_engine;   // [trueshift] rpm_by_engine (defined in modules/trueshift.cpp)

static const float RS_SPAN_LO = 300, RS_SPAN_HI = 450;          // the fitted band of (plateau - idle)
static const float RS_PLATEAU_LO = 900, RS_PLATEAU_HI = 1000;   // the plateaus the absolute revs were fitted on
static const float RS_BRAKE_LO = 1500, RS_BRAKE_SPAN = 400;     // the brake range the light stage was fitted on (1500-1900)

static inline int rs_on() { return g_rpm_by_engine > 0 && V.eng_ok && V.plateau > 0; }

// The factor of the fixed offsets for this engine (1 = as fitted).
static inline float rs_k(float idle)
{
    if (!rs_on()) return 1.0f;
    float span = V.plateau - idle;
    if (!(span > 0)) return 1.0f;
    float k = span < RS_SPAN_LO ? span / RS_SPAN_LO : span > RS_SPAN_HI ? span / RS_SPAN_HI : 1.0f;
    return fminf(2.5f, fmaxf(0.5f, k));
}

// A fixed distance from idle or from the plateau (rpm).
static inline float rs_off(float idle, float n) { return n * rs_k(idle); }

// An absolute rev fitted for a plateau of 900-1000: it moves with the plateau outside that band, and its distance above 900 scales.
static inline float rs_abs(float idle, float x)
{
    if (!rs_on()) return x;
    float pb = fminf(RS_PLATEAU_HI, fmaxf(RS_PLATEAU_LO, V.plateau));
    return x + (V.plateau - pb) + (x - RS_PLATEAU_LO) * (rs_k(idle) - 1.0f);
}

// An absolute rev of the engine brake's light stage: the same distance from the low end of the brake range in fractions of that range.
static inline float rs_brake(float x)
{
    if (!rs_on()) return x;
    float lo = V.eng.range[R_BRAKE][0], span = V.eng.range[R_BRAKE][1] - lo;
    if (!(span > 100)) return x;
    return lo + (x - RS_BRAKE_LO) * fminf(2.5f, fmaxf(0.5f, span / RS_BRAKE_SPAN));
}
