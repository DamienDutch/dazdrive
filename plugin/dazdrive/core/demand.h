#pragma once
// The driver's demand history (model layer, core/model.h): how hard the driver has asked, in the last ten minutes, as a probability that a
// demand of a given size comes within the next few seconds. The gear choice uses it to price the reserve it keeps for what is likely to come
// (an expected-time term, docs/powerband-change.md round 3); Cruise's overtake pull and the planner's comfort are later clients.
//
// The measure is the torque pedal's wish SHARE (wish_share_eff(): 0..1 of a full wish at the speed), not a force: the same driver reads the same
// in a 9 t and a 44 t truck and on the 1:19 map. One entry per second (the second's largest share); a ring of DEMAND_MEM_S of them.

enum { DEMAND_MEM_S = 600,      // the memory, seconds. The one number of this model: a window as long as the 20 s horizon needs about ten of them for a
                                // 5 % quantile to mean anything (10 x 20 = 200 s is the least), and ten minutes is what "how he drove lately" means
       DEMAND_LEVELS = 4 };     // the levels of demand_view
static const float DEMAND_HORIZON_S = 20.0f;   // the window of demand_view: the gear choice's plan horizon (gearchoice.cpp GC_HORIZON)
static const float DEMAND_LEVEL_LIGHT = 0.3f;  // the lowest level of demand_view: the least a share must be to be a demand rather than trimming

struct demand_view {
    float horizon_s;                // the window the p's are over (s)
    float level[DEMAND_LEVELS];     // shares of a full wish: 0.3 0.5 0.7 0.9 (a light, a half, a firm and a hard demand)
    float p[DEMAND_LEVELS];         // the probability that a window of horizon_s contains a demand of at least that share
    float p_seen[DEMAND_LEVELS];    // the same, the foot as it has been seen (demand_p_seen): what is not seen yet is worth one window of the prior
};

// One second of the pedal: the largest wish share in that second (NAN = the pedal gave no wish: not recorded). demand_tick is the way in; call demand_note
// directly only to put a second in by hand (a test, a restored history).
void demand_note(float share);
// The feed, once per frame (model_frame): the wish's EFFECTIVE share (wish_share_eff(): after the limiter and the caps, NAN = the pedal is not the source:
// cruise, standing, neutral, reverse, a pass-through) at the simulation clock now_s (sim_s(): it stops in a pause). Keeps the largest share of the current
// whole second and notes it when the clock moves into another second; a second with no share at all is not recorded, so the memory counts seconds the
// driver's pedal was the demand (a lift in gear is a share of 0 and is recorded). A clock that jumps (a reset, a pause) closes the open second once and
// continues from the new one: it never fills the gap.
void demand_tick(double now_s, float share);
// Seconds held (0..DEMAND_MEM_S).
int demand_seconds();
// Counts every change of the memory (a note, an import, a reset): the persistence saves when it moved.
unsigned demand_gen();
// The profile on disk (core/driver.h): the newest seconds, oldest first (returns how many were written, at most max), and the way back in. A history that
// comes from a file is the previous session's: demand_import puts it in only while the memory is empty (a start; live seconds win), and its seconds weigh
// less than live ones: a loaded second has 1 - age / DEMAND_MEM_S of a vote, age in driving seconds since it was noted (the ring position from the newest),
// the rest of its vote is the prior. So the previous session's last second counts almost as much as a live one, its oldest hardly, and the weight reaches
// 0 at the moment the ring drops the second: the loaded history fades out as the driver drives, with no knob of its own (the decay is the memory length).
// The wall-clock time between sessions is not an age: days off do not make the way he drove less true.
int demand_export(float *out, int max);
int demand_import(const float *shares, int n);   // returns the seconds taken (0 = refused: the memory is not empty, or n <= 0); shares clamped to 0..1, NAN is 0
// The share of horizon-long windows in the memory that contain a share >= level (0..1). Windows the memory has not filled yet count with the prior
// "any share equally likely": p = 1 - level^H for H whole seconds (a window of H independent seconds holds none >= level with probability level^H), so a
// fresh start keeps full reserve and relaxes as the driver proves calm; an empty memory is the prior itself, ten calm minutes are 0.
float demand_p(float level, float horizon_s);
// The same with the foot as it has been SEEN: the windows not filled yet are worth one window of the prior (demand_p: all of the unfilled memory). For the foot that is on the pedal and holds a speed.
float demand_p_seen(float level, float horizon_s);
// The q-quantile of the driver's demand: the level (a share of a full wish, 0..1) he stays under in a fraction q of the windows, read off the view by linear
// interpolation between its levels (a window holds a demand of at least 0 with p 1, and of more than a full wish with p 0). p_scale (0..1) multiplies the
// probabilities first: the chance that the driver is there at all (the cruise holds, the foot is off: the chance the foot comes back). The band taste reads
// q = 0.7 (core/intent.cpp): ten minutes of him, so one stab at the pedal moves it by a window in thirty.
float demand_level_at(const demand_view *v, float p_scale, float q);
// Once per frame, before the modules (model_frame): refreshes demand_now() when the memory changed.
void demand_frame();
const demand_view *demand_now();
// Forgets the history (the caller decides when: a new profile, not a truck change, the demand is scale-free).
void demand_reset();

// The CONTROLLER's memory (WP-0911-K1): the same ring and the same view, fed with the share a controller asked (the game's cruise: its throttle; the plugin's cruise and the planner: their force
// wish as a share of a full wish), one entry per second it was driving. Session only: never exported, never written to the profile; a pause or a reset closes the open second like the driver's.
// The gear choice reads it where the foot is off and a controller holds the speed, as it reads the driver's where the foot is on.
void demand_ctl_tick(double now_s, float share);
const demand_view *demand_ctl_now();
int demand_ctl_seconds();
void demand_ctl_reset();
