// Offline test of the dashboard letter (core/display.cpp): the priority (a flash over a status over the mode letter, the latest
// post within a priority, a flash's time), and that a letter outside the table, which game.cpp's auto_name() would not accept,
// can never be posted or written, and the writer of [core] dash_messages (no mode letter posted: flashes and statuses over the
// game's own letter, which comes back, at 4 Hz; quiet with the key at 0 or a mode letter posted). The layer is included, so its statics are reachable. Build+run: tools/sim/run_display.sh
#include "core/suite.h"
#include "core/game.h"
#include <string.h>

telemetry T;
void logline(const char *, ...) {}
static double t_now;
double wall_s() { return t_now; }
static char letter = 'A';   // the transmission's buffer
static int writes, restores;
int game_letter(void *, char c) { if (!c) { c = 'A'; restores++; } letter = c; writes++; return 1; }   // 0: the game's own (A here)
static int gb;
void *game_gearbox() { return &gb; }
#include "core/display.cpp"

static int bad;
static void check(int ok, const char *what) { if (!ok) { printf("   FAIL: %s\n", what); bad = 1; } }
static char shown() { char c; disp_write(&gb, &c); return c ? letter : 0; }

static void priority()
{
    printf("--- priority\n");
    check(disp_write(&gb, NULL) == -1 && letter == 'A', "nobody posts: nothing written");
    disp_post(DISP_TS, DISP_MODE, 'D', 0);
    char c = shown(); printf("mode D -> %c\n", c); check(c == 'D', "mode letter");
    disp_post(DISP_CRUISE, DISP_STATUS, 'B', 0);
    c = shown(); printf("+ status B -> %c\n", c); check(c == 'B', "status over mode");
    disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5);
    c = shown(); printf("+ flash L for 1.5 s -> %c\n", c); check(c == 'L', "flash over status");
    t_now += 1.0;
    disp_post(DISP_CRUISE, DISP_FLASH, '2', 1.5);
    c = shown(); printf("+ flash 2 at 1.0 s -> %c\n", c); check(c == '2', "latest flash wins");
    t_now += 0.7;
    c = shown(); printf("1.7 s: the first flash is over, the second is not -> %c\n", c); check(c == '2', "second flash still on");
    t_now += 0.9;
    c = shown(); printf("2.6 s: both over -> %c\n", c); check(c == 'B', "back to the status");
    disp_post(DISP_CREEP, DISP_STATUS, 'C', 0);
    c = shown(); printf("+ status C -> %c\n", c); check(c == 'C', "latest status wins");
    disp_post(DISP_CRUISE, DISP_STATUS, 'B', 0);
    c = shown(); printf("status B posted again -> %c\n", c); check(c == 'B', "latest status wins again");
    disp_post(DISP_CRUISE, DISP_STATUS, 0, 0);
    c = shown(); printf("B withdrawn -> %c\n", c); check(c == 'C', "the other status stays");
    disp_post(DISP_CREEP, DISP_STATUS, 0, 0);
    c = shown(); printf("C withdrawn -> %c\n", c); check(c == 'D', "back to the mode");
    disp_post(DISP_TS, DISP_MODE, 'A', 0);
    c = shown(); printf("reverse: mode A -> %c\n", c); check(c == 'A', "mode A");
    disp_post(DISP_TS, DISP_MODE, 0, 0);
    check(disp_write(&gb, NULL) == -1, "all withdrawn: nothing to show");
}

static void table()
{
    printf("--- letters outside the table\n");
    disp_post(DISP_TS, DISP_MODE, 'D', 0);
    int refused = 0, odd = 0, n = 0;
    for (int i = 1; i < 256; i++) {
        char c = (char)i;
        int in = strchr(disp_letters(), c) != NULL;   // what auto_name() accepts
        int ok = disp_post(DISP_CRUISE, DISP_FLASH, c, 1.5);
        char s = shown();
        if (!in && (ok || s != 'D')) odd++;
        if (in && (!ok || s != c)) odd++;
        refused += !ok;
        n++;
        disp_post(DISP_CRUISE, DISP_FLASH, 0, 0);
    }
    printf("%d of %d letters refused, %d accepted (the table has %d); written but not accepted by auto_name: %d (0)\n", refused, n, n - refused,
           (int)strlen(disp_letters()), odd);
    check(odd == 0 && n - refused == (int)strlen(disp_letters()), "only the table's letters can be shown");
    disp_post(DISP_CREEP, DISP_STATUS, 'Q', 0);
    char s = shown(); printf("status Q -> %c (D: refused, the mode stays)\n", s); check(s == 'D', "status Q refused");
    disp_post(DISP_TS, DISP_MODE, 0, 0);
}

// The writer of [core] dash_messages, run every 1/60 s for secs; returns the writes.
static int frames(double secs)
{
    int w0 = writes;
    for (double e = 0; e < secs - 1e-9; e += 1 / 60.0) { disp_frame(); t_now += 1 / 60.0; }
    return writes - w0;
}

static void messages()
{
    printf("--- without a mode letter ([core] dash_messages)\n");
    letter = 'A'; s_ours = 0; t_now = 100;
    g_dash_messages = 0;
    disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5);
    int w = frames(1);
    printf("key 0, flash L: %d writes, letter %c (A)\n", w, letter); check(w == 0 && letter == 'A', "key 0: nothing written");
    g_dash_messages = 1;
    disp_post(DISP_TS, DISP_MODE, 'E', 0);
    w = frames(1);
    printf("key 1, a mode letter posted: %d writes (0: its owner writes)\n", w); check(w == 0, "mode owner writes, not the frame");
    disp_post(DISP_TS, DISP_MODE, 0, 0);
    disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5);
    w = frames(1);
    printf("key 1, flash L: %d writes in 1 s (4 Hz), letter %c\n", w, letter); check(w >= 4 && w <= 5 && letter == 'L', "flash written at 4 Hz");
    w = frames(1);
    printf("the flash is over: letter %c, %d game's-own write(s)\n", letter, restores); check(letter == 'A' && restores == 1, "game's letter back once");
    disp_post(DISP_CRUISE, DISP_STATUS, 'B', 0);
    frames(0.5);
    printf("status B -> %c\n", letter); check(letter == 'B', "status shown");
    disp_post(DISP_CREEP, DISP_FLASH, 'X', 1.5);
    frames(0.5);
    printf("+ flash X -> %c\n", letter); check(letter == 'X', "flash over the status");
    frames(1.5);
    printf("flash over -> %c\n", letter); check(letter == 'B', "back to the status");
    disp_post(DISP_CRUISE, DISP_STATUS, 0, 0);
    w = frames(2);
    printf("status withdrawn -> %c, %d game's-own writes in all, %d writes in the last 2 s (1)\n", letter, restores, w);
    check(letter == 'A' && restores == 2 && w == 1, "game's letter back once, then quiet");
    disp_post(DISP_TS, DISP_MODE, 'D', 0); disp_write(&gb, NULL);   // TrueShift shows D, then stops (an unsupported truck)
    disp_post(DISP_TS, DISP_MODE, 0, 0);
    frames(0.5);
    printf("TrueShift wrote D and withdrew -> %c (A)\n", letter); check(letter == 'A' && restores == 3, "game's letter after TrueShift's");
    g_dash_messages = 0;
}

int main()
{
    priority();
    table();
    messages();
    printf(bad ? "FAILED\n" : "all checks passed\n");
    return bad;
}
