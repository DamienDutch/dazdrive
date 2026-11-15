// Minimal stand-in so the suite's modules compile natively for the simulator.
#pragma once
#include <stdint.h>
#define MAX_PATH 260
typedef int BOOL;
static inline short GetAsyncKeyState(int) { return 0; }

// The clock of the cost of a decision (modules/trueshift.cpp, core/gearchoice.h GC_CPU_BUDGET). The sims must be reproducible, so this one does not tick by itself: it advances SW_QPC_US microseconds
// (default 0) at every read, and a pair of reads around a decision measures exactly that cost. SW_QPC_US=700 replays a PC on which the whole score costs 700 us.
#include <stdlib.h>
struct LARGE_INTEGER { int64_t QuadPart; };
static inline BOOL QueryPerformanceFrequency(LARGE_INTEGER *f) { f->QuadPart = 1000000; return 1; }
static inline BOOL QueryPerformanceCounter(LARGE_INTEGER *c)
{
    static const long long step = getenv("SW_QPC_US") ? atoll(getenv("SW_QPC_US")) : 0;
    static long long t = 0;
    t += step; c->QuadPart = t;
    return 1;
}
