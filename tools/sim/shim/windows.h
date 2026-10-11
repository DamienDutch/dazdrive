// Minimal stand-in so the suite's modules compile natively for the simulator.
#pragma once
#include <stdint.h>
#define MAX_PATH 260
typedef int BOOL;
static inline short GetAsyncKeyState(int) { return 0; }
