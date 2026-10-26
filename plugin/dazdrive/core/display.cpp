// The dashboard letter: the one place that writes the transmission's one-letter auto_mode_name for every module. A module
// posts a wish (a letter, a priority, a duration) under its own owner tag, as with the actuator (core/actuate.cpp); the wish is
// held until the owner changes it or its time runs out; one writer picks the letter: a flash (a confirmation for a moment), else
// a status (something holds), else the mode letter, and within a priority the latest post. Until now the modules shared two
// globals (one flash, one status) that only TrueShift's dash_letter() read, and game.cpp kept its own list of the letters a
// buffer may hold: a flash that was not on it (I = ISA) made the buffer look foreign and the letter stayed. Both now come from
// k_letters below, and a letter that is not on it cannot be posted.
#include "suite.h"
#include "game.h"
#include <string.h>

// Every letter the dashboard may show; game.cpp's auto_name() accepts a buffer that holds one of them. A: the game's automatic,
// TrueShift in reverse, ACC on; D, E, S, M: TrueShift's modes; X: something off; L: lane keeping on; C: crawl on, plain cruise;
// B: hill start, Auto Hold, Stop & Go holding; 1, 2, 3: the ACC distance; I: ISA. P, N, F, O, H, K: from earlier versions.
static const char k_letters[] = "ADEPMNSFOHXLCBK123I";
const char *disp_letters() { return k_letters; }

struct disp_wish { char c; double until; unsigned seq; };   // c 0 = none; until 0 = held until changed, else wall_s(); seq = order of the posts
static disp_wish D[DISP_N][DISP_LEVELS];
static unsigned s_seq;

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

// The letter to show now: the latest live wish of the highest priority that has one; 0 = nobody posts.
char disp_pick()
{
    double now = wall_s();
    for (int l = DISP_LEVELS - 1; l >= 0; l--) {
        const disp_wish *best = NULL;
        for (int o = 0; o < DISP_N; o++) {
            const disp_wish *w = &D[o][l];
            if (w->c && (!w->until || now < w->until) && (!best || w->seq > best->seq)) best = w;
        }
        if (best) return best->c;
    }
    return 0;
}

// The one writer: the picked letter into gb's transmission. 1 written (or already there), 0 the buffer was not found, -1 nothing
// to show. *shown = the letter.
int disp_write(void *gb, char *shown)
{
    char c = disp_pick();
    if (shown) *shown = c;
    if (!c) return -1;
    return game_letter(gb, c);
}
