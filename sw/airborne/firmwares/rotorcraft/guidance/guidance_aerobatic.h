/*
 * Copyright (C) 2015
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file firmwares/rotorcraft/guidance/guidance_module.h
 * Guidance in a module file.
 *
 * Implement a custom controller in a module.
 * Re-use desired modes:
 *
 * e.g.: <tt>#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER</tt>
 * can be used to only define a horizontal control in the module and use normal z_hold
 *
 * The guidance that the module implement must be activated with following defines:
 *
 * a) Implement own Horizontal loops when GUIDANCE_H_MODE_MODULE_SETTING is set to GUIDANCE_H_MODE_MODULE
 * One must then implement:
 * - void guidance_h_module_init(void);
 * - void guidance_h_module_enter(void);
 * - void guidance_h_module_read_rc(void);
 * - void guidance_h_module_run(bool in_flight);
 *
 *
 * b) Implement own Vertical loops when GUIDANCE_V_MODE_MODULE_SETTING is set to GUIDANCE_V_MODE_MODULE
 * - void guidance_v_module_enter(void);
 * - void guidance_v_module_run(bool in_flight);
 *
 * If the module implements both V and H mode, take into account that the H is called first and later V
 *
 */

#ifndef GUIDANCE_AEROBATIC_H_
    #define GUIDANCE_AEROBATIC_H_
    #define GUIDANCE_MODULE_H_
    //#include "generated/modules.h"
    #include "math/pprz_algebra_int.h"
    #include "math/pprz_matrix_decomp_float.h"

    struct AerobaticVariableSettings {
        float max_vel;  //maximum velocity used on trajectory
        float max_acc;  //maximum acceleration used on trajectory
        float p_gain;   //gain used on position control
        float d_gain;   //gain used on velocity control
        float z_p_factor;
        float z_d_factor;
        int32_t mode; 
        uint8_t use_feedforward;
        uint8_t use_alt_gains;  
        uint8_t use_filtered_output;
        uint8_t ce_mode;
        // float min_thrust_eff;   // 0 = hover, 1 = trajectory, 2 = impulse
    };

    struct GuidanceLogStruct {
        float t;
        float mode;
        float max_vel;
        float max_acc;
        float q[4];
        float pos[3];
        float vel[3];
        float acc[3];
        float ref_pos[3];
        float ref_vel[3];
        float ref_acc[3];
        float air[3];
        float gains[2];
        float thrust_in;
        float dyaw;
        float du[3];
    };

    extern int32_t sd_id;
    extern int32_t sd_log[30];

    extern struct GuidanceLogStruct aerolog;
    extern struct AerobaticVariableSettings varset;

    extern float guidance_aerobatic_indi_gain;


    extern void setReferenceHere(uint8_t wp);
    extern void makeWaypointsSquare(uint8_t wp_1, uint8_t wp_2, uint8_t wp_3, uint8_t wp_4);
    extern void getWaypoints(uint8_t wp_1, uint8_t wp_2, uint8_t wp_3, uint8_t wp_4);
    extern void updateWaypoints(void);
    extern void setModeKill(void);

    void guidance_aerobatic_init(void);
    void guidance_aerobatic_enter(void);
    void guidance_aerobatic_run(void);

    void filter_init(void);
    void filter_run(void);

    void YawControl(void);
    void Controller(void);
    void get_states(void);
    void get_G(void);
    void get_du(void);
    void set_states(void);

    void get_mode(void);
    void UpdateTrajectory(void);
    void CalcNewTrajectory(void);
    void TrajectoryScenario(void);

    void update_sd_logger(void);
    void message_log(void);

    bool Mat44Inv(const float m[16], float invOut[16]);
    void Mat44Vect4Mul(float du[4], const float G_inv[16], const float dv[4]);
#endif 
