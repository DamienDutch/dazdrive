// The sense layer's reading of the game's AI traffic list (WP-S step S3; it was the memory walk of modules/traffic.cpp). Read-only: nothing is ever
// written to the game. Layout: ETS2 1.60 only (notes in modules/traffic.cpp). The module keeps the zones, the csv, the summary and the readers' API
// (core/traffic.h); it reads the entries below.
#pragma once
#include "suite.h"
#include "trafficmath.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

enum { TR_MAX_ENT = 1400 };
enum { L_SP1 = 1, L_SP2 = 2, L_OBJ = 4, L_SLAVE = 8 };   // the lists an entry was found in (bit mask in the csv)
enum { K_AI = 0, K_AIT = 1 };

struct tr_ent {
    uintptr_t p;
    double x, z;          // world metres, middle of the box
    double ox, oz;        // world metres, the placement origin
    float len, wid, speed, rv_l, rv_s, hdg, dx, dy, dist;
    float pspeed;         // the speed of the vehicle it belongs to (a trailer inherits its vehicle's), NAN unknown
    float bz0, bz1;       // the box in the local z (start, end; z is backward)
    int mask, kind;
    tr_pose pose;
};
struct tr_own_box { int ok; float f, r, w, dist; };   // our truck's / trailer's box from the player lists (m ahead / behind the origin, width), and how far its origin sat from the SDK's

struct tr_sense {
    tr_ent ent[TR_MAX_ENT];
    int n, bad;
    int cnt[8];                     // entries per list this tick: sp1, sp2, obj, obj-as-vehicle, slaves, pv, pt, dup
    struct { uintptr_t vt; int n, ai; } ovt[8];   // vtables seen in traffic_objects_1 this tick
    int n_ovt;
    int pv_n, pt_n;
    double pv_dx, pv_dy, pv_d, pt_dx, pt_dy, pt_d;   // nearest player vehicle / trailer origin relative to ours
    tr_own_box own_v, own_t;        // (reset and filled by every walk)
    uintptr_t slot, img_lo, img_hi;
    int hits;
    uintptr_t ai_vt[4];             // vtables of the vehicles in the spawned lists
    int n_ai_vt;
};
extern tr_sense g_trs;

static inline int trs_plausible(double v, double lim) { return v == v && fabs(v) <= lim; }

// Find the game_traffic global (pattern scan, exactly one hit). Returns the module's state: 0 = no layout or the image is unreadable (status says why),
// 1 = no pattern / slot outside the image, 2 = waiting for the instance. Logs what the module logged before.
int trs_discover(char *status, size_t nstatus);
unsigned long long trs_slot_off();      // the global slot, as an offset into the game image (for the status line)
// The instance: 1 = the object at the slot is the game's traffic object, and *vt_off is its vtable as an image offset; 0 = not (yet).
int trs_instance(unsigned long long *vt_off);
// Walk the lists of the instance trs_instance found, into g_trs. me = our pose.
void trs_walk(const tr_pose &me);
void trs_forget_vtables();              // another save: the AI vtables are learned again
