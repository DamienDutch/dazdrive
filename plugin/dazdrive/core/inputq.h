// The event queue of the semantic input device (core/input.cpp), as plain logic so a sim can run it: which game mixes the device
// offers, and the order in which presses and held levels reach the game. No SDK types in here.
//
// A PRESS is two events: value 1 in one frame, value 0 in a later one (after hold_ms and at least one frame). Several presses of
// the same mix (two modules in one frame) are kept in order, never merged: the second one starts one frame after the first one
// went low, so the game sees two separate edges. A press that has waited over IQ_STALE_S (the device was not asked for events) is
// dropped: a stale toggle must not fire minutes later. A HELD level is one event per change; it does not mix with presses on the
// same mix (a mix is used either way, not both).
#pragma once
#include <string.h>

// The mixes the device registers (names from notes/semantical-mixes-1.60.txt, the game's own 290; these are the ones we use or
// plan to: the button box, the lights, the axles, the camera, the engine start-stop). The input's name IS the mix name; the game
// maps it by name.
static const char *const IQ_MIXES[] = {
    "lwinopen", "lwinclose", "rwinopen", "rwinclose",
    "frontsuspup", "frontsuspdwn", "rearsuspup", "rearsuspdwn", "trlrsuspup", "trlrsuspdwn", "suspreset",
    "liftaxle", "liftaxlet", "diflock", "beacon", "flasher4way", "horn", "airhorn",
    "light", "lighton", "lightoff", "hblight", "lighthorn",
    "wipers0", "wipers1", "wipers2", "wipers3", "wipers4",
    "retarder0", "retarder1", "retarder2", "retarder3", "retarder4", "retarder5",
    "engbraketog", "parkingbrake", "trailerbrake",
    "cruiectrl", "cruiectrlinc", "cruiectrldec", "cruiectrlres", "accmode", "laneassist",
    "screenshot", "quickpark", "radio",
    "engine", "ignitionoff", "ignitionon", "ignitionstrt",
};
enum { IQ_N = sizeof IQ_MIXES / sizeof IQ_MIXES[0], IQ_RING = 256, IQ_FIFO = 32 };
static const double IQ_STALE_S = 2.0;

static inline int inq_index(const char *mix)
{
    if (!mix) return -1;
    for (int i = 0; i < IQ_N; i++) if (!strcmp(IQ_MIXES[i], mix)) return i;
    return -1;
}

struct inq {
    int active;                       // the game asks for events (device active)
    unsigned frame;                   // frames seen (inq_frame calls)
    int state[IQ_N];                  // 0 idle, 1 high (value 1 sent), 2 gap (value 0 sent this frame)
    double high_at[IQ_N];
    int hold_ms[IQ_N];
    unsigned state_frame[IQ_N];       // the frame the state was entered
    int want[IQ_N], out[IQ_N];        // held level wanted / sent
    struct { int idx, hold_ms; double t; } fifo[IQ_FIFO];
    int n_fifo;
    struct { int idx, val; } ring[IQ_RING];
    int r_head, r_n;
    unsigned n_press, n_refused, n_stale, n_events;   // counters for the log
};

static inline void inq_clear(inq *q) { memset(q, 0, sizeof *q); }

// A press of a mix. 1 = queued; 0 = refused (unknown mix, device not active, queue full). hold_ms 0 = one frame.
static inline int inq_press(inq *q, const char *mix, int hold_ms, double now)
{
    int i = inq_index(mix);
    if (i < 0 || !q->active || q->n_fifo >= IQ_FIFO) { q->n_refused++; return 0; }
    q->fifo[q->n_fifo].idx = i; q->fifo[q->n_fifo].hold_ms = hold_ms > 0 ? hold_ms : 0; q->fifo[q->n_fifo].t = now;
    q->n_fifo++;
    q->n_press++;
    return 1;
}

// A held level: sent as one event at the next frame, and again when the device becomes active. 1 = accepted.
static inline int inq_hold(inq *q, const char *mix, int on)
{
    int i = inq_index(mix);
    if (i < 0 || !q->active) { q->n_refused++; return 0; }
    q->want[i] = on ? 1 : 0;
    return 1;
}

static inline void inq_emit(inq *q, int idx, int val)
{
    if (q->r_n >= IQ_RING) return;   // cannot happen: at most two events per mix per frame
    int at = (q->r_head + q->r_n++) % IQ_RING;
    q->ring[at].idx = idx; q->ring[at].val = val;
    q->n_events++;
}

// The device becomes active (or inactive). Either way the presses in flight are gone and the sent levels start again from 0
// (the game forgets what a device said when it goes inactive); the wanted levels stay, so an active device sends them anew.
static inline void inq_set_active(inq *q, int active)
{
    if (active == q->active) return;
    q->active = active;
    q->n_fifo = 0;
    q->r_head = q->r_n = 0;
    for (int i = 0; i < IQ_N; i++) { q->state[i] = 0; q->out[i] = 0; }
}

// The first event request of a game frame: work out this frame's events (they are then popped one by one).
static inline void inq_frame(inq *q, double now)
{
    q->frame++;
    if (!q->active) return;
    for (int i = 0; i < IQ_N; i++) {
        if (q->state[i] == 2 && q->state_frame[i] < q->frame) q->state[i] = 0;                    // the gap frame is over
        if (q->state[i] == 1 && q->state_frame[i] < q->frame && (now - q->high_at[i]) * 1000.0 >= q->hold_ms[i]) {
            inq_emit(q, i, 0);
            q->state[i] = 2; q->state_frame[i] = q->frame;
        }
        if (q->want[i] != q->out[i] && q->state[i] == 0) { inq_emit(q, i, q->want[i]); q->out[i] = q->want[i]; }
    }
    int keep = 0;
    for (int k = 0; k < q->n_fifo; k++) {
        int i = q->fifo[k].idx;
        if (now - q->fifo[k].t > IQ_STALE_S) { q->n_stale++; continue; }
        if (q->state[i] == 0) {
            inq_emit(q, i, 1);
            q->state[i] = 1; q->state_frame[i] = q->frame; q->high_at[i] = now; q->hold_ms[i] = q->fifo[k].hold_ms;
            continue;
        }
        q->fifo[keep++] = q->fifo[k];     // that mix is busy: this press waits for a later frame, in order
    }
    q->n_fifo = keep;
}

// The next event of this frame: 1 = idx/val set, 0 = none left.
static inline int inq_pop(inq *q, int *idx, int *val)
{
    if (q->r_n <= 0) return 0;
    *idx = q->ring[q->r_head].idx; *val = q->ring[q->r_head].val;
    q->r_head = (q->r_head + 1) % IQ_RING;
    q->r_n--;
    return 1;
}
