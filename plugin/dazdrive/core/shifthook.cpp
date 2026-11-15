// The shared plumbing of the gear brains: the game's hook sites, one set for all (see shifthook.h), and the ts_* questions the other
// modules ask of the brain of the class.
#include "shifthook.h"
#include "vclass.h"
#include "vehicle.h"
#include <string.h>

static window_fn o_window;
static gear_fn o_choose, o_start;
static int s_ok, s_done, s_want, s_installed;   // the window / chooser / start hooks
static int s_stop_in, s_stop_tried, s_tail_in, s_tail_tried, s_engage;

static const shift_brain *s_reg[4];
static int s_nreg;

void shift_register(const shift_brain *b)
{
    if (b && s_nreg < 4) s_reg[s_nreg++] = b;
}

const shift_brain *shift_active()
{
    unsigned bit = vclass_active() == VC_LIGHT ? VCM_CAR : VCM_TRUCK;
    for (int i = 0; i < s_nreg; i++)
        if (s_reg[i]->classes & bit) return s_reg[i];
    return NULL;
}

// The brain that has the gearbox now: of the class, and on (its ini, its hooks). NULL = the game's own automatic keeps it.
static const shift_brain *live()
{
    if (!s_ok) return NULL;
    const shift_brain *b = shift_active();
    return b && b->is_on && b->is_on() ? b : NULL;
}

static float *hk_window(void *self, float *out, unsigned gear)
{
    float *r = o_window(self, out, gear);
    const shift_brain *b = live();
    if (b && b->on_window) b->on_window(self);
    return r;
}
static int hk_choose(void *self)
{
    int stock = o_choose(self);
    const shift_brain *b = live();
    return b && b->on_choose ? b->on_choose(self, stock) : stock;
}
static int hk_start(void *self)
{
    int stock = o_start(self);
    const shift_brain *b = live();
    return b && b->on_start ? b->on_start(self, stock) : stock;
}
static void hk_tail(void *self)
{
    const shift_brain *b = live();
    if (b && b->on_tail) b->on_tail(self);
}
// The automatic's stop threshold: below it the clutch opens / neutral. Stock 0.8 x idle (+ a brake term the game adds).
static float hk_stop(float idle, float thr, float brake)
{
    const shift_brain *b = live();
    return b && b->stop_threshold ? b->stop_threshold(idle, thr, brake) : 0.8f * idle;
}

int shift_hooks_install(int *done, int *want)
{
    if (!s_installed) {
        s_installed = 1;
        o_window = (window_fn)G.window_fn;
        o_choose = (gear_fn)G.chooser_fn;
        o_start = (gear_fn)G.start_fn;
        int d = game_patch_call(G.decide_site, (void *)hk_window), w = 1 + G.n_choose + G.n_start;
        for (int i = 0; i < G.n_choose; i++) d += game_patch_call(G.choose_sites[i], (void *)hk_choose);
        for (int i = 0; i < G.n_start; i++) d += game_patch_call(G.start_sites[i], (void *)hk_start);
        s_done = d; s_want = w; s_ok = d == w;
    }
    if (done) *done = s_done;
    if (want) *want = s_want;
    return s_ok;
}
int shift_hooks_ok() { return s_ok; }

int shift_hook_stop()
{
    if (!s_ok) return 0;
    if (!s_stop_tried) {
        s_stop_tried = 1;
        s_stop_in = G.stop_site && game_patch_stop(G.stop_site, hk_stop) ? 1 : 0;
    }
    return s_stop_in;
}

// The update's tail hook carries eco-roll AND the neutral watchdog (a shift interrupted by a pause leaves gear 0 with the clutch open):
// both need the engage code.
int shift_hook_tail()
{
    if (!s_ok) return 0;
    if (!s_tail_tried) {
        s_tail_tried = 1;
        s_engage = game_engage_init();
        s_tail_in = s_engage && G.tail_site && game_patch_tail(G.tail_site, (void *)hk_tail) ? 1 : 0;
    }
    return s_tail_in;
}
int shift_tail_engage_ok() { return s_engage; }

// ---------------------------------------------------------------- what the others ask (suite.h)

int ts_start_gear(float thr)
{
    const shift_brain *b = shift_active();
    return b && b->start_gear ? b->start_gear(thr) : 0;
}
float ts_idle_rpm()
{
    const shift_brain *b = shift_active();
    return b && b->idle_rpm ? b->idle_rpm() : 550.0f;
}
int ts_mode()
{
    const shift_brain *b = shift_active();
    return b && b->mode ? b->mode() : -1;
}
void ts_commanded(int gear)
{
    const shift_brain *b = shift_active();
    if (b && b->commanded) b->commanded(gear);
}
int ts_view(ts_view_t *o)
{
    memset(o, 0, sizeof *o);
    o->why = "";
    const shift_brain *b = shift_active();
    return b && b->view ? b->view(o) : 0;
}
int ts_stop_hooked() { return s_stop_in; }
