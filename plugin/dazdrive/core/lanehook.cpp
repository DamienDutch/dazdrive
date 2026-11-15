// The game surface of Lane's hooks: see lanehook.h. Moved from modules/lane.cpp ln_start (WP-LA2) without a change of pattern, guard or order.
#include "suite.h"
#include "game.h"
#include "sense.h"
#include "act.h"
#include "lanehook.h"
#include <stdint.h>

static float *s_minp;    // the pooled constant the law compares the speed with (55.0 in the game), once repointed at ours

int lanehook_law(lane_law_fn stub, lane_law_fn *game_law)
{
    // The LKA call-site pattern carries no struct offsets, so it can match on any build while the fields the law reads and writes (L->a_lka_u, L->i_steer_in, L->i_assist,
    // L->a_lights, L->a_lka) are per-layout: hook only when the layout has them.
    if (!L || !L->a_lka_u || !L->i_steer_in || !L->i_assist || !L->i_steer) return LH_NO_LAYOUT;
    uintptr_t h = game_scan("41 0F 28 DF F3 44 0F 11 64 24 20 41 0F 28 D6 41 0F 28 CD 48 8B CB E8");
    uintptr_t site = h ? h + 0x16 : 0;
    if (!site) return LH_NOT_FOUND;
    *game_law = (lane_law_fn)(site + 5 + (intptr_t) * (const int32_t *)(site + 1));
    if (!game_patch_call(site, (void *)stub)) return LH_FAILED;
    return LH_OK;
}

int lanehook_min_speed_patch(float kmh)
{
    // Per-layout (the game_actor read it anchors on).
    uintptr_t m = game_scan(sense_is_161() ? "F3 0F 59 C7 0F 54 C6 0F 2F 05 ?? ?? ?? ?? 73 5E 8B 83 7C 01 00 00"
                                          : "F3 0F 59 C7 0F 54 C6 0F 2F 05 ?? ?? ?? ?? 73 5E 8B 83 14 01 00 00");
    return m && *(const float *)(m + 14 + *(const int32_t *)(m + 10)) == 55.0f && (s_minp = game_patch_rip_float(m + 7, 3, 7, kmh)) != NULL;
}

int lanehook_min_speed_ok() { return s_minp != NULL; }
void act_lane_min_speed(float kmh) { if (s_minp) *s_minp = kmh; }

int lanehook_assist_store(float (*filter)(float))
{
    uintptr_t st = game_scan(game_is_162() ? "F3 44 0F 11 87 88 14 00 00 84 DB 75 04" : sense_is_161() ? "F3 44 0F 11 87 B0 14 00 00 84 DB 75 04"
                                           : "F3 44 0F 11 87 20 14 00 00 84 DB 75 04");
    return st && game_patch_store_xmm8(st, filter);
}
