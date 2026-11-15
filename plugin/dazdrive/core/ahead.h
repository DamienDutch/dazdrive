#pragma once
// The road ahead ("vooruitblik"), read-only for the powertrain layers: the force it takes to hold the speed over the next seconds,
// when each stretch of it arrives, and how sure that is. It reads the route (core/route.h: the navigation's elevation profile, route_ok()), the vehicle model
// (core/vehicle.h: mass, rolling and air resistance, vehicle_resist_grade) and, when the planner drives, its plan. Nothing else
// in the powertrain layers looks at the road; later the cruise, bends and eco-roll may read it too.

// The samples a view carries: the horizon (30 s) times the fastest speed it is read at (53 m/s, about 190 km/h) over the grid the route is sampled on
// (25 m): 30 x 53 / 25 = 63.6, so 64. A faster truck or a longer horizon keeps the first AHEAD_MAX (the sums and the conf still cover all of them).
enum { AHEAD_MAX = 64 };

struct ahead_view {
    float resist_now_n;     // the force that holds the speed now (rolling + drag + the grade under the truck) (N)
    float resist_worst_n;   // the largest of it over the horizon (N)
    float resist_mean_n;    // the mean over the horizon (N)
    float horizon_s;        // the seconds looked ahead (at the speed v)
    float conf;             // 0..1: 0 = no route (only the grade now: the worst = now), 1 = a known route over the whole horizon
                            // (the planner driving or the route's profile valid); in between when the route ends inside the horizon
    // The same stretches one by one, in the order they come (appended: the views above did not have them). No route: n_samples 0.
    int n_samples;
    float sample_s[AHEAD_MAX];          // the seconds until the truck is at the middle of sample i, at the speed of now: (25 i + 12.5 m) / v
    float sample_resist_n[AHEAD_MAX];   // the force that holds the speed over sample i (N)
};

// v: m/s; horizon_s: seconds to look ahead (30 by default). Fills out; never fails (no route = conf 0, worst = now).
void ahead_get(float v, float horizon_s, ahead_view *out);

// The 30 s view the powertrain layers read, at the speed of now (|T.speed|), with its conf faded: the certainty of the road ahead can drop at once (a
// junction, the route lost, the planner handing back) and what is priced on it would jump, so conf falls and rises at 0.25 per second, per sim
// second of the callers' asks (a gap of more than 3 s starts over at the raw value). Cached as ahead_get is (0.25 s); asking twice at the same
// moment gives the same view. Valid until the next call.
const ahead_view *ahead_now();
// The route switch of the view: 0 = the road is not read at all (route_ok() and the route's grades are not called): the view is that of a truck with no route, whatever the navigation
// has, which is what ahead_get reports without one (n_samples 0, conf 0, the worst and the mean are the resistance of now). The reader that owns the key (TrueShift, [trueshift]
// lookahead) says it before it asks; default 1. A change takes effect at once (the cache is dropped).
void ahead_use_route(int on);
// A break in the drive (a teleport, a load, another truck: RESET_*): the cache and the certainty fade belong to the road we left. The next ask computes afresh.
void ahead_reset();
