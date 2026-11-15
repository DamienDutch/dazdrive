#pragma once
// The gear arbiter (arbitrate layer, docs/architecture.md). Whoever has an opinion about the gear of this decision posts it with a priority; the
// request with the highest priority wins, the first post wins within a priority. The brains (modules/trueshift.cpp, modules/carshift.cpp) begin a
// decision, their rules post, the decision resolves. Nothing here knows a rule: it is the one place where the priorities between the kinds of
// request live, so a module that wants the gear (the planner's neutral request, the low-speed launch owner) posts here and never reaches into
// the brain.
//
// A request is the gear asked (0 = leave it to the game, GR_NEUTRAL = the neutral of the eco-roll) and why (the brain's why code, for the log
// and the dashboard). "Keep this gear" is a request like any other (the gear of now), so a hold can outrank a soft choice.

enum gear_prio {        // lower value = higher priority
    GP_PROTECT = 0,     // the limiter, the stall, a wrong gear: physics, nothing outranks it
    GP_OWNER,           // somebody else has the gearbox: the low-speed launch hold, a hint hold, the manual mode
    GP_BRAKING,         // braking, the engine brake and the descent control, the brake air
    GP_PLANNER,         // the neutral request of the eco-roll
    GP_SOFT,            // the soft choice (the layers) and the holds that wait for it
    GP_COUNT
};

enum { GR_NEUTRAL = 100,    // a request for neutral (the eco-roll); never a real gear number
       GR_NONE = -1 };      // gear_req_resolve: nobody posted

struct gear_req {
    int gear;       // the gear asked: 0 = leave it to the game, GR_NEUTRAL = neutral, else a gear number
    int why;        // the brain's why code
    int prio;       // gear_prio
};

void gear_req_begin();                      // a new decision: forget the posts of the last one
void gear_req_post(const gear_req *r);      // one request; it wins when no earlier post has the same or a higher priority
int gear_req_resolve(int *why);             // the winner's gear (and its why in *why); GR_NONE (why untouched) when nobody posted. Starts the next decision

// ---------------------------------------------------------------- standing requests (WP-BR)
// A request that is not a gear and does not belong to one decision: a wish held by its owner until the owner changes it, applied by the brain at each decision
// (the brain turns it into a gear by its own rules: the bands up to the crossover, the landing wall, the lift model's price; the arbiter holds no rule). The
// brake channel posts the engine drag it wants and the engine-brake stage it holds, Decel (the lever assist) the service brake it needs besides the lever's
// retarder and engine brake. These were three globals (g_brake_need, g_decel_need, g_eb_descent) the gearbox read; now the one reader is the brain, through here.
// A post of the same owner replaces its last one; a clear removes it. Written by the owners' frames or ticks, read in the gearbox's decision (maybe another
// thread): each owner's slot is a seqlock, so a read never sees half a post.
enum { GSO_BRAKE, GSO_DECEL, GSO_N };   // who posts (the log and the sims name them)
enum { GSK_SUSTAINED, GSK_LEVER };      // drag_n's kind: the owner's need has lasted and the gearbox weighs the road ahead (does the shift pay?); the driver holds a lever and wants it as asked
struct gear_stand {
    float drag_n;          // N at the wheels: the extra engine drag the owner wants beyond what the gear in gives (0 = none)
    int kind;              // GSK_*
    int eb_held;           // the owner holds an engine-brake stage: the brain keeps the gear in the engine's brake range, whatever the stage
    double hold_until;     // sim_s(): the gear stays until then (the stages just came off at the end of a stretch of descent; they come back); 0 = none
    int eb_alloc;          // the brake channel holds an engine-brake stage it took for a request (the lever's wish, the pedal, an ask), not the descent rule's and not the driver's: information only, nothing in the gearbox reads it yet.
                           // TrueShift follows a held stage with its engine-brake rule (keep the revs up toward the brake range); a rule that means to leave a stage of the channel's to work in the gear of now reads this.
};
void gear_stand_post(int owner, const gear_stand *s);
void gear_stand_clear(int owner);
// What stands now, all owners merged (the largest drag, any held stage, the latest hold); 0 = nobody asks anything. *owner_mask gets a bit per owner that stands.
int gear_stand_get(gear_stand *out, int *owner_mask = nullptr);
