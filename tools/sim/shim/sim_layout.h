// The simulator runs the module logic against the 1.60 offsets (the feature offsets the sims write into their fake game_actor
// buffers). Pattern fields stay empty: the sims never scan the game. Included once per sim; each sim then binds L to it.
#pragma once
#include "core/game.h"

static const layout sim_layout = {
    "sim", "eut2",
    "", "", "", "",                                   // pat_window, pat_chooser, pat_lobound, pat_mass
    "", "", "", "",                                   // chk_target, chk_force, chk_decide, chk_torque
    "",                                               // pat_adaptive
    0x10, 0x18, 0x68, 0x119,                          // s_inner, s_state, s_target, s_force
    0x1448, 0x1424, 0x1428, 0xff0, 0xfe8,             // i_gear, i_thr_in, i_thr_eff, i_engine, i_trans
    0x1c0, 0x1d0, 0x1d8, 0x22c, 0x23c,                // e_torque, e_curve, e_curve_n, e_rpm_limit, e_ranges
    0x1c0,                                            // t_shift_time
    0x199,                                            // a_adaptive_block
    0x258, 0x150, 0x358,                              // t_auto_name, i_tx_alt, x_tx_alt
    0x1440, 0x143c, 0x260, 0x264, 0x378,              // i_eb_flag, i_eb_demand, e_eb_downshift, e_eb_cap, a_eb_frac
    0,                                                // a_gearbox (0: the actuator's act_player_inner finds no gearbox in the sim's small actor)
    "", "", "",                                       // pat_gps, pat_tail, pat_stop
    0x1b8,                                            // e_type
    0x150, 0x154, 0x17c, 0x180, 0x190, 0x198, 0x199, 0x19c, 0x1a0, 0x1a8,   // a_state..a_lead_m
    0x1d0, 0x1d1, 0x1d5c, 0x340, 0x1e0, 0x1d00, 0x1cc0, 0x1cc4, 0x142c, 0x1444,   // a_lights..i_retarder
    0x1408, 0x1420, 0x640, 0x100, 0x101, 0x110, 0x228, 8,    // i_steer_in, i_assist, i_steer, s_amode, s_latch, s_drive, e_idle, tail_jmp
    "", "", "",                                       // pat_actuator, pat_hazard, pat_gate
    "", "", "", "",                                   // bp_pat, bp_getter, bp_req, bp_hill
    "", "", "",                                       // bl_pat, bl_on, bl_getter
    "", "", "",                                       // rf_pat, rf_before, rf_after
};
