// The dashboard letter: the one place that writes the transmission's one-letter auto_mode_name for every module. A module
// posts a wish (a letter, a priority, a duration) under its own owner tag, as with the actuator (core/actuate.cpp); the wish is
// held until the owner changes it or its time runs out; one writer picks the letter: a flash (a confirmation for a moment), else
// a status (something holds), else the mode letter, and within a priority the latest post. Until now the modules shared two
// globals (one flash, one status) that only TrueShift's dash_letter() read, and game.cpp kept its own list of the letters a
// buffer may hold: a flash that was not on it (I = ISA) made the buffer look foreign and the letter stayed. Both now come from
// k_letters below, and a letter that is not on it cannot be posted.
//
// The log (promise P7): the writer says in one line which owner's wish is shown, at which priority, each time that changes (log level 1).
#include "suite.h"
#include "game.h"
#include <string.h>

// Every letter the dashboard may show; game.cpp's auto_name() accepts a buffer that holds one of them. A: the game's automatic,
// TrueShift in reverse, ACC on; D, E, P, M: TrueShift's modes (S: car mode's Sport); X: something off; L: lane keeping on; C: crawl on, plain cruise;
// B: hill start, Auto Hold, Stop & Go holding; 1, 2, 3: the ACC distance; I: ISA; T: the lane steering calibration is armed or running.
// N, F, O, H, K: from earlier versions.
static const char k_letters[] = "ADEPMNSFOHXLCBK123IT";
const char *disp_letters() { return k_letters; }

static const char *const k_disp_owner[DISP_N] = { "TrueShift", "Cruise", "Lane", "Creep" };   // DISP_TS, DISP_CRUISE, DISP_LANE, DISP_CREEP
static const char *const k_disp_level[DISP_LEVELS] = { "mode", "status", "flash" };           // DISP_MODE, DISP_STATUS, DISP_FLASH
struct disp_wish { char c; double until; unsigned seq; };   // c 0 = none; until 0 = held until changed, else wall_s(); seq = order of the posts
static disp_wish D[DISP_N][DISP_LEVELS];
static unsigned s_seq;
static int s_ours;               // the buffer holds a letter we wrote (the game's own goes back when nobody posts, dash_messages only)
float g_dash_messages;           // [core] dash_messages

int disp_post(int owner, int level, char c, double secs)
{
    if (owner < 0 || owner >= DISP_N || level < 0 || level >= DISP_LEVELS) return 0;
    if (c && !strchr(k_letters, c)) {
        static int said;
        if (!said) { said = 1; logline("dashboard letter 0x%02x is not in the list of letters: not shown", (unsigned char)c); }
        return 0;
    }
    disp_wish *w = &D[owner][level];
    w->c = c;
    w->until = c && secs > 0 ? wall_s() + secs : 0;
    w->seq = ++s_seq;
    return 1;
}

// The wish to show now: the latest live wish of the highest priority that has one (NULL = nobody posts); *owner / *level say whose and which.
static const disp_wish *disp_pick_wish(int *owner, int *level)
{
    double now = wall_s();
    for (int l = DISP_LEVELS - 1; l >= 0; l--) {
        const disp_wish *best = NULL;
        int bo = -1;
        for (int o = 0; o < DISP_N; o++) {
            const disp_wish *w = &D[o][l];
            if (w->c && (!w->until || now < w->until) && (!best || w->seq > best->seq)) { best = w; bo = o; }
        }
        if (best) { if (owner) *owner = bo; if (level) *level = l; return best; }
    }
    return NULL;
}

// The letter to show now: the latest live wish of the highest priority that has one; 0 = nobody posts.
char disp_pick()
{
    const disp_wish *w = disp_pick_wish(NULL, NULL);
    return w ? w->c : 0;
}

// One line when the shown wish changes (who won, at which priority, what it hides): the answer to "why does the dash show a B".
static void disp_note(const disp_wish *w, int owner, int level)
{
    static int last_o = -2, last_l = -2;
    static char last_c = 1;
    char c = w ? w->c : 0;
    int o = w ? owner : -1, l = w ? level : -1;
    if (c == last_c && o == last_o && l == last_l) return;
    last_c = c; last_o = o; last_l = l;
    if (w) logv(1, "Display: letter %c from %s (%s)", c, k_disp_owner[o], k_disp_level[l]);
    else logv(1, "Display: nobody posts a letter");
}

// The one writer: the picked letter into gb's transmission. 1 written (or already there), 0 the buffer was not found, -1 nothing
// to show. *shown = the letter.
int disp_write(void *gb, char *shown)
{
    int ow = -1, lv = -1;
    const disp_wish *w = disp_pick_wish(&ow, &lv);
    disp_note(w, ow, lv);
    char c = w ? w->c : 0;
    if (shown) *shown = c;
    if (!c) return -1;
    int r = game_letter(gb, c);
    if (r > 0) s_ours = 1;
    return r;
}

// [core] dash_messages: while no mode letter is posted (TrueShift off, its dash_letter 0, a truck it leaves to the game) the
// flashes and statuses are written here, at TrueShift's rate, over the game's own letter, which comes back when they are over.
// With a mode letter posted its owner runs the writer.
void disp_frame()
{
    static double at;
    static int state = -1;       // one log line per change: -1 unknown, 0 not found, 1 written
    double now = wall_s();
    if (g_dash_messages <= 0 || now - at < 0.25) return;   // (0.25 s = TrueShift's own write interval, core/shiftcommon.cpp sc_dash_letter: one rate for both writers)
    for (int o = 0; o < DISP_N; o++) if (D[o][DISP_MODE].c) return;
    at = now;
    if (T.shifter[0] && strcmp(T.shifter, "automatic") && strcmp(T.shifter, "arcade")) return;   // a manual box shows its gear
    void *gb = game_gearbox();
    if (!gb) return;
    char c;
    int r = disp_write(gb, &c);
    if (r < 0) {
        if (!s_ours) return;
        s_ours = 0;
        c = 0;
        r = game_letter(gb, 0);   // the game's own letter back
    }
    if (r != state) {
        if (r > 0) logline("dashboard letter without the mode letter works (%c)", c ? c : '-');
        else logline("dashboard letter without the mode letter: auto_mode_name string not found");
        state = r;
    }
}
