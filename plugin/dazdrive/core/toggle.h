// Pressing a toggle control (lift axle, high beam) and checking that it took: a plain state machine, no SDK, so a sim can run it.
//
// The module knows the control's state from telemetry (cur) and what it wants (want). toggle_step() says "press now" at most once per
// attempt, waits verify_s for cur to follow, tries again once, then gives up for backoff_s (the game may use another mix for that
// control, or ignore the device): a toggle that is pressed blindly in a loop would flip the control back and forth.
// toggle_ours() tells whether a change of cur just now is the answer to our own press (else the driver or the game changed it).
#pragma once

struct toggle_ctl {
    double pressed_at;    // time of the last press (0 = none)
    int target;           // the state that press was for (-1 = no press in flight)
    int tries;            // presses for the same target without success
    double backoff_until; // no press before this time
    float verify_s;       // how long the state may take to follow a press (0 = TOGGLE_VERIFY_S)
    unsigned n_ok, n_fail;
};

static const double TOGGLE_VERIFY_S = 1.5;    // the state must follow a press within this time (a few frames at 30 fps is plenty)
static const int TOGGLE_MAX_TRIES = 2;
static const double TOGGLE_BACKOFF_S = 30;

static inline void toggle_clear(toggle_ctl *c) { c->pressed_at = 0; c->target = -1; c->tries = 0; c->backoff_until = 0; }

// cur/want: 0 or 1; cur < 0 = state unknown, want < 0 = no wish. Returns 1 = press now. *event: 0 nothing, 1 the last press worked,
// -1 it did not (given up for TOGGLE_BACKOFF_S).
static inline int toggle_step(toggle_ctl *c, double now, int cur, int want, int *event)
{
    if (event) *event = 0;
    if (cur < 0 || want < 0) { c->target = -1; c->tries = 0; return 0; }
    double verify = c->verify_s > 0 ? c->verify_s : TOGGLE_VERIFY_S;
    if (c->target >= 0 && cur == c->target) {                // the press worked
        c->n_ok++;
        if (event) *event = 1;
        c->target = -1; c->tries = 0;
    }
    if (cur == want) { c->target = -1; c->tries = 0; return 0; }
    if (now < c->backoff_until) return 0;
    if (c->target >= 0 && c->target != want) { c->target = -1; c->tries = 0; }   // the wish changed while waiting
    if (c->target >= 0) {                                     // a press is in flight for this wish
        if (now - c->pressed_at < verify) return 0;
        c->tries++;                                           // it did not take
        c->target = -1;
        if (c->tries >= TOGGLE_MAX_TRIES) {
            c->n_fail++;
            c->tries = 0;
            c->backoff_until = now + TOGGLE_BACKOFF_S;
            if (event) *event = -1;
            return 0;
        }
    }
    c->pressed_at = now;
    c->target = want;
    return 1;
}

// A change of the control's state at `now`: is it the answer to our press (still within the verify time)?
static inline int toggle_ours(const toggle_ctl *c, double now)
{
    double verify = c->verify_s > 0 ? c->verify_s : TOGGLE_VERIFY_S;
    return c->target >= 0 && now - c->pressed_at < verify + 0.5;
}
