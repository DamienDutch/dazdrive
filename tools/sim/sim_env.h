// SIM_SET="cruise.ahead_accel=0.2,decel.jerk_limit=3": override ini defaults in a sim, for sweeps. Call sim_env_apply(section, params, n)
// once per module at the start of main(), before the sim's own "val = def" loop. It sets BOTH def and *val, so every later
// reset-to-defaults in the sim picks the override up. Unset variable: no-op. A key that is never found prints a warning.
#pragma once
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
static inline void sim_env_apply(const char *section, param *ps, int n)
{
    const char *e = getenv("SIM_SET"); if (!e || !*e) return;
    char buf[2048]; snprintf(buf, sizeof buf, "%s", e);
    size_t sl = strlen(section);
    for (char *t = strtok(buf, ","); t; t = strtok(NULL, ",")) {
        char *eq = strchr(t, '='); if (!eq) continue; *eq = 0;
        if (strncmp(t, section, sl) || t[sl] != '.') continue;
        bool hit = false;
        for (int i = 0; i < n; i++) if (!strcmp(ps[i].key, t + sl + 1)) { ps[i].def = (float)atof(eq + 1); *ps[i].val = ps[i].def; hit = true; }
        if (!hit) fprintf(stderr, "SIM_SET: no key %s\n", t);
    }
}
