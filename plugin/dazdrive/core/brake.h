// The brake channel (core/brake.cpp): one place every automated brake request goes through ([brake] channel = 1), and the
// driver's brake pedal as a deceleration wish ([brake] pedal = 1). A requester asks only a deceleration of the truck (m/s2,
// + = slowing, the total: what coasting gives counts towards it) and an urgency; the channel allocates it to coasting, the
// retarder (fed through the game's retarder demand, a lower gear asked from TrueShift when its force needs the rpm) and the
// service brake (+0x17c through the actuator). The AEB, the rollover guard, Auto Hold, hill start, the stop assist, Stop&Go's
// hold and the ACC's stop behind a standing lead keep their own requests above it (the actuator merges by the biggest).
#pragma once

// The automated requesters (the driver's pedal has its own path: the pedal provider below).
enum { BRQ_DESCENT,    // Cruise: a descent the retarder and the engine brake cannot hold within the descent allowance
       BRQ_LIMIT,      // Cruise: the line at the speed limit + overspeed_cap_kmh (gravity taking the truck over it)
       BRQ_BEND,       // Cruise: too fast for a bend ahead or the bend the truck is in
       BRQ_JUNCTION,   // Cruise: the speed profile to a junction / roundabout
       BRQ_ACC,        // Cruise: the own ACC following a lead (its stop behind a standing lead and its standstill hold stay ACT_ACC)
       BRQ_PLANNER,    // the longitudinal planner (phase 2 of the planner plan; nobody asks yet)
       BRQ_SPEED,      // Cruise in layers ([cruise.general] engine = layers): the speed arbiter's deceleration (docs/cruise-layers.md)
       BRQ_N };
// Urgency = how fast the deceleration may come and whether the service brake waits for the retarder to build up.
enum { BRU_COMFORT,    // jerk_comfort; the retarder first, the service brake only for what the retarder cannot give (or a shortfall over 0.3 m/s2)
       BRU_FIRM,       // jerk_firm; the service brake fills any shortfall over 0.1 m/s2 while the retarder builds up
       BRU_URGENT,     // no jerk limit; the service brake fills everything the retarder does not give at once
       BRU_N };

// The stages a braking event is carried by, in the order of their cost (one unit per m/s2, core/brake.cpp BC_COST): what the truck does by itself (rolling, air, the grade, the
// engine's drag in the gear it is in), a lower gear's engine drag (asked of the gearbox through the arbiter's standing request, core/gearreq.h), the engine brake's stages (on every truck with an
// engine brake, WP-BR3), the retarder, the service brake. BS_* are also bits of brake_view.stages.
enum { BS_COAST, BS_DRAG, BS_EB, BS_RET, BS_SRV, BS_N };
// The term that decided the split of the last decision, for the log line (promise P7): why the service brake and the retarder have what they have.
enum { BT_NONE,         // no event
       BT_COAST,        // coasting (and the stages held) give what is asked: nothing else brakes
       BT_RETARDER,     // the retarder carries it (the service brake not needed or within the tolerance)
       BT_BEYOND,       // the ask is more than the retarder's full force: the service brake fills the rest
       BT_SHORT,        // the retarder is still building up (more than the urgency's tolerance short): the service brake fills the shortfall
       BT_GAP,          // a shift gap: the retarder is gone, the service brake bridges
       BT_FADE,         // the low-speed fade: the retarder hands over to the service brake
       BT_URGENT,       // an urgent ask: the service brake from the first tick
       BT_N };
extern float g_brake_channel;   // [brake] channel
extern float g_brake_pedal;     // [brake] pedal

void brake_attach();                                 // the channel's tick hook in (start does it when the actuator is hooked; sims call it after Cruise's start)
int brake_channel_on();                              // [brake] channel = 1 and the actuator hooked: the requesters ask here, not with act_brake
void brake_ask(int who, float decel, int urgency);   // held until changed; decel <= 0 or NAN = none. Cheap: call it every tick
// The same, but a brake deceleration on top of what the truck does by itself now (coasting, the grade, the game's own overspeed retarder):
// the requester's law was tuned as an extra request (the own ACC's). Its share is open loop (the added ask is not a target of its own),
// but the channel's trim still closes the loop on the measured deceleration so the service brake escalates when the retarder can't deliver.
void brake_ask_add(int who, float brake_decel, int urgency);
// A requester that can wait for the gear (WP-SP3 round 3): the speed in m/s that the truck may owe of its braking while the retarder is gone for a shift (a gap) -- the retarder pays it back when the
// clutch is closed, the service brake bridges only what is owed beyond it. Not for urgent asks. 0 (the default) = the service brake bridges the gap at once. Held until changed, per requester.
void brake_patience(int who, float owe_ms);
float brake_asked(int who);                          // what who asks now (0 = none)
// The engine brake is the first stage of every event ([brake] engine_brake): a requester that closes its own loop on the deceleration (the deceleration assist's lever steps) tells the
// channel how much deceleration it wants of the stages, in m/s2, TOTAL (what the stage it holds already gives is in it: wish = what the reference still lacks of the deceleration without
// its own brake + brake_eb_worth(), so the wish does not change when the stage comes on and the stage does not switch itself off again). Renew it every
// tick (it lives 0.5 s); 0 or nothing = none. The channel sets the engine brake's stage (only its tick may: the game's stage setter is safe from the actuator's hooks and nowhere else).
void brake_eb_wish(float decel);
float brake_eb_credit();                             // m/s2 the engine-brake stage the allocation holds is counted to give now (0 = none, or a shift gap): the need of the retarder and the service brake is less by this
float brake_eb_worth();                              // m/s2 the same stage is worth in the model, in full (the credit is a conservative part of it until the stage is measured); what a requester that measures the TOTAL adds to its wish
float brake_asks_max();                              // the largest deceleration any requester asks now (0 = none): no throttle of ours against it
// Cruise, every tick (whatever [brake] says): where the game's own overspeed braking starts now, A in km/h (its retarder demand is
// clamp(v - A, 0, 1), cruise-re notes § 2); NAN = it does not act (cruise off, smart off, the driver's throttle). The channel counts
// that demand as retarder it already has: it feeds only above it and never fights it.
void brake_game_overspeed(float from_kmh);
// Cruise, every tick: the same point in the strict form the engine-brake stage's control reads (where the game's own overspeed braking starts, and the speed its cruise holds,
// km/h; NAN for both = it does not act: cruise off, the driver's throttle, the overspeed setting off). The engine-brake stage control needs both.
void brake_descent_view(float from_kmh, float target_kmh);
// EBrake ([ebrake] descent keys): the engine-brake stage control the channel runs for a truck without a retarder: on (1/0), the first stage's margin under the game's overspeed
// start (km/h), the least seconds between two stages up, the seconds the gearbox keeps its gear after the stages came off. Pushed by the module every frame; the control
// is off while it has not been pushed for 2 s (the module is not running).
void brake_eb_config(float on, float margin_kmh, float step_s, float hold_s);
// The service brake's deceleration per unit of request at the mass of now: the vehicle model's estimate (the prior derived from the mass, moved
// towards the learned slot by its weight; core/vehicle.h). *learned = 1 when the slot is confident (the pedal provider needs that).
float brake_gain(int *learned);
// The pedal provider (game_brake_pedal_set_provider): the service brake the game applies for the driver's raw pedal. Exposed for the sims.
float brake_pedal_map(float raw, float req, void *ga);

// Read-only view for the dashboard stream and the sims (nothing here steers).
struct brake_view {
    int on;            // the channel runs ([brake] channel 1 and hooked)
    int active;        // a requester asks now
    int who, urgency;  // the requester whose ask is the largest (BRQ_*, -1 = none) and its urgency
    float want;        // m/s2 asked (the largest ask)
    float ref;         // m/s2, the jerk-limited reference toward it
    float nat;         // m/s2 the truck slows by itself now (rolling, air, grade, engine drag in gear; - = it speeds up)
    float trim;        // m/s2, the closed-loop correction on the measured deceleration
    float need;        // m/s2 the retarder and the service brake have to give
    float ret_full;    // m/s2 the retarder gives at full demand now (fade, shift gap, mass)
    float ret_force;   // N, the retarder's force at full demand (the vehicle model's: the start value 54 kN = 3.0 m/s2 at 18 t, times what was learned for this truck)
    float game_demand; // the game's own overspeed retarder demand now (estimate, 0..1)
    float feed;        // our retarder demand (0..1), the automated part
    float pedal_feed;  // the retarder demand of the pedal's blending
    float ret_built;   // the retarder's built-up demand (model)
    float ret_decel;   // m/s2 the retarder gives now (model)
    float service;     // m/s2 asked from the service brake
    float request;     // the service request posted (0..1, ACT_CHAN)
    float gain;        // m/s2 per request (brake_gain)
    int gain_learned;
    float measured;    // m/s2, the measured deceleration (fast filter)
    int lights;        // the brake lights asked on
    int down;          // more engine drag asked of the gearbox (the arbiter's standing request, GSO_BRAKE)
    float drag_n;      // N at the wheels asked of the gearbox (0 = none)
    int stages;        // bit BS_* per stage that carries part of the event now
    int term;          // BT_*: what decided the split
    float cost;        // the event's cost now: sum over the stages of the unit cost x the m/s2 it carries
    int eb_stage, eb_cap;   // the engine-brake stage we hold (0 = none) and the stage count of the engine
    float eb_decel;    // m/s2 the held stage is worth in the gear and at the revs of now (the vehicle model's)
    int eb_alloc;      // the stage the allocation asks (an event of any asker; the descent rule's own stage is eb_stage when it is the larger)
    float eb_credit;   // m/s2 of the event the stage is counted to carry (the retarder's and the service brake's need is less by this)
    float eb_asked;    // m/s2 the stages may carry of the event now (the largest of the channel's need, the lever assist's wish, the pedal's)
    int pedal;         // the pedal provider maps (1) or passes the raw pedal (0)
    float pedal_raw, pedal_out, pedal_want;   // the last provider call: raw pedal, service value returned, deceleration it stood for (m/s2)
    float pedal_top;   // m/s2 the linear part of the pedal reaches
};
void brake_view_get(brake_view *o);
