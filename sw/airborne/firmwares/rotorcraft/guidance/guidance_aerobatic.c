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

/** @file firmwares/rotorcraft/guidance/guidance_aerobatic.c
 *  Reference generation for horizontal guidance.
 *
 */

//Include
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_matrix_decomp_float.c"
#include "math/pprz_simple_matrix.h"
#include "stabilization.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"
#include "subsystems/navigation/waypoints.h"
#include "CEmainmain.h"

#define PPRZ_MSG_ID_AEROBATIC_INDI 158

//Declare variables
float guidance_aerobatic_indi_gain = 0.5;
float debug_value;

struct ctrl_lin_struct {
    // Input Position
    struct NedCoor_f pos; //get from stat.h: stateGetPositionNed_f(void)
    // Input Velocity
    struct NedCoor_f vel; //get from stat.h: stateGetSpeedNed_f(void)
    // Input Acceleration
    struct NedCoor_f acc; //get from stat.h: stateGetAccelNed_f(void)
    // Output Acceleration
    struct NedCoor_f cmdacc;
} ctrl_lin;

struct trajectory {
    float t;
    struct NedCoor_f pos;
    struct NedCoor_f vel;
    struct NedCoor_f acc;
} traj;

struct AerobaticVariableSettings varset = {
    .max_vel = GUIDANCE_AEROBATIC_MAX_VEL,
    .max_acc = GUIDANCE_AEROBATIC_MAX_ACC,
    .p_gain  = GUIDANCE_AEROBATIC_P_GAIN,
    .d_gain  = GUIDANCE_AEROBATIC_D_GAIN,
    .mode = GUIDANCE_AEROBATIC_MODE
};

struct GuidanceLogStruct aerolog = {
    .t = 0.0f,
    .mode = GUIDANCE_AEROBATIC_MODE,
    .max_vel = GUIDANCE_AEROBATIC_MAX_VEL,
    .max_acc = GUIDANCE_AEROBATIC_MAX_ACC,
    .qi = 0,
    .qx = 0,
    .qy = 0,
    .qz = 0,
    .pos_x = 0.0f,
    .pos_y = 0.0f,
    .pos_z = 0.0f,
    .vel_x = 0.0f,
    .vel_y = 0.0f,
    .vel_z = 0.0f,
    .acc_x = 0.0f,
    .acc_y = 0.0f,
    .acc_z = 0.0f,
    .ref_pos_x = 0.0f,
    .ref_pos_y = 0.0f,
    .ref_pos_z = 0.0f,
    .ref_vel_x = 0.0f,
    .ref_vel_y = 0.0f,
    .ref_vel_z = 0.0f,
    .ref_acc_x = 0.0f,
    .ref_acc_y = 0.0f,
    .ref_acc_z = 0.0f,
    .thrust_in = 0.0f,
    .d_yaw = 0.0f,
    .du0 = 0.0f,
    .du1 = 0.0f,
    .du2 = 0.0f
};


struct FloatVect3 V_inf;
struct FloatVect3 V_drone;
struct FloatQuat q;

struct FloatVect3 V_temp;

struct FloatVect3 Yaw_proj;
struct FloatVect3 Yaw_ref;

struct FloatVect3 Yaw_b;
struct FloatVect3 Yaw_bE;

float d_yaw;

struct NedCoor_f meas_acc;  
struct NedCoor_f fix_pos;

float thrust_in = 0;

float V[3]={0.1f,2.0f,1.0f};
float phi = 0;
float alpha = 0;

float t = 0.0;

#define ROWS_GUIDANCE 3 //rows
#define COLS_GUIDANCE 3 //columns

float Weight[COLS_GUIDANCE] = {1, 1 ,1};

float G[ROWS_GUIDANCE][COLS_GUIDANCE] = {0};

struct FloatVect3 dv;
float du[COLS_GUIDANCE] = {0};
struct FloatVect3 dua;


struct trajectory_section {
    int number;
    float t_beg;
    float t_trans_beg;
    float t_trans_end;
    float t_end;
    
    float d_x_trans;

    int type;
    struct NedCoor_f pos_beg;
    struct NedCoor_f pos_end;
    struct NedCoor_f dir;

    float delta_pos;
    float vel_max;
    float acc_max;
} traj_sec;

float x_coords[4] = {0};
float y_coords[4] = {0};


#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  float thrust_in_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
#else
  float thrust_in_specific_force_gain = -500;
#endif

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
  #ifdef STABILIZATION_INDI_FILT_CUTOFF
    #define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
  #else
    #define GUIDANCE_INDI_FILTER_CUTOFF 3.0
  #endif
#endif

#define GUIDANCE_INDI_THRUST_DYNAMICS 0.1

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass filt_vel_ned[3];
Butterworth2LowPass filt_pos_ned[3];
Butterworth2LowPass filt_quat[4];
// Butterworth2LowPass roll_filt;
// Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;


float log_pos[3] = {0};
float log_vel[3] = {0};
float log_acc[3] = {0};
float log_ref_pos[3] = {0};
float log_ref_vel[3] = {0};
float log_ref_acc[3] = {0};
float log_gains[2] = {0};

int32_t sd_id;
int32_t sd_log[30];

//Declare all functions

// void guidance_aerobatic_init(void);
void guidance_aerobatic_enter(void);
void guidance_aerobatic_read_rc(void);
// void guidance_aerobatic_run(void);


void filter_init(void);
static void guidance_indi_filter_thrust(void);
void guidance_indi_propagate_filters(void);

void test_SVD(void);
void add_increment(void);


void print_matrix(float **ptr,int m, int n);
void print_matrix_matlab(float **ptr,int m, int n);
// void print_eq(float **ptrA, float **ptrX, float **ptrB, int m, int n, int l);
void print_2darr(int row, int col, float* Data_00);
void print_row(float *ptr,int m);
void print_row_matlab(float *ptr,int m);

void UpdateTrajectory(void);
void LinAccCon(void);
void get_increment(void);
void SVDw(void);
void get_dv(void);
void get_G(void);
void get_states(void);
void print_states(void);
// void print_matrix(float** ptr,int m, int n);
// void print_row(float* ptr,int m);

void CalcNewTrajectory(void);
void TrajectoryScenario(void);



//Guidance Module
void guidance_h_module_init(void){
    guidance_aerobatic_init();
}

void guidance_h_module_enter(void){
    guidance_aerobatic_enter();
}

void guidance_h_module_read_rc(void){
    // // -MAX_PPRZ to MAX_PPRZ
    // ctrl_windtunnel.rc_throttle = radio_control.values[RADIO_THROTTLE];
    // ctrl_windtunnel.rc_roll = radio_control.values[RADIO_ROLL];
    // ctrl_windtunnel.rc_pitch = radio_control.values[RADIO_PITCH];
    // ctrl_windtunnel.rc_yaw = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(UNUSED bool in_flight){
    // Call full inner-/outerloop / horizontal-/vertical controller:
    guidance_aerobatic_run();
}

void guidance_v_module_init(void){ 
      // initialization of your custom vertical controller goes here
}

    // Implement own Vertical loops
void guidance_v_module_enter(void){
      // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(UNUSED bool in_flight){
      // your vertical controller goes here
}
  

//

//Messages
static void send_aerobatic_indi(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_AEROBATIC_INDI(trans, dev, AC_ID,
                         &varset.max_vel,
                         &varset.max_acc,
                         &varset.p_gain,
                         &varset.d_gain,
                         &varset.mode);
}

static void send_aerobatic_log(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_AEROBATIC_LOG(trans, dev, AC_ID,
                         3,&log_pos[0],
                         3,&log_vel[0],
                         3,&log_acc[0],
                         3,&log_ref_pos[0],
                         3,&log_ref_vel[0],
                         3,&log_ref_acc[0],
                         2,&log_gains[0]);
}
//

// Initialize function
void guidance_aerobatic_init(void){
  debug_value = 0.0;

  // get_dv();
  // get_G();

  t = -5.0;

  filter_init();
  // ////printf("thrust filt: %f", thrust_filt.o[0]);
  
  // test_SVD(); //This function works in init
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_LOG, send_aerobatic_log);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_INDI, send_aerobatic_indi);
 }
// Enter function
void guidance_aerobatic_enter(void){
    t = -5.0;
    
    varset.max_vel = 1.0;
    varset.max_acc = 1.0;

    traj_sec.type = 0;
    traj_sec.number = -1;
    traj_sec.t_beg = 0.0f;
    CalcNewTrajectory();

    traj.pos.x = stateGetPositionNed_f()->x;
    traj.pos.y = stateGetPositionNed_f()->y;
    traj.pos.z = stateGetPositionNed_f()->z-0.05;
    traj.vel.x = 0.0f;
    traj.vel.y = 0.0f;
    traj.vel.z = 0.0f;
    traj.acc.x = 0.0f;
    traj.acc.y = 0.0f;
    traj.acc.z = 0.0f;

    filter_init();
    // thrust_in = stabilization_cmd[COMMAND_THRUST];
    // thrust_act = thrust_in;

    ////printf("pos_est_x\tpos_est_y\tpos_est_z\tpos_ref_x\tpos_ref_y\tpos_ref_z\tvel_est_x\tvel_est_y\tvel_est_z\tvel_ref_x\tvel_ref_y\tvel_ref_z\tacc_est_x\tacc_est_y\tacc_est_z\tacc_ref_x\tacc_ref_y\tacc_ref_z\t");
    ////printf("qi\tqx\tqy\tqz\t");
    ////printf("V_b_x\tV_b_y\tV_b_z\t");
    ////printf("G00\tG01\tG02\tG03\tG10\tG11\tG12\tG13\tG20\tG21\tG22\tG23\tdv_x\tdv_y\tdv_z\tdu_phi\tdu_theta\tdu_psi\tdu_T\n");
}
// Periodic function
void guidance_aerobatic_run(){
  static int32_t counter = 0;
  t = t + 0.001953125;
  if (t > 360000){
    t = 0;
  }

  guidance_indi_propagate_filters();
  guidance_indi_filter_thrust();

  get_states();

  // UpdateTrajectory();
  LinAccCon();

  get_G();
  get_increment();
  add_increment();
  print_states();
  if (counter%2){
    update_sd_logger();
  }
  counter++;
}
// Controller functions
void get_states(void){

    ////printf("\n\nget_states");
    q.qi = filt_quat[0].o[0];
    q.qx = filt_quat[1].o[0];
    q.qy = filt_quat[2].o[0];
    q.qz = filt_quat[3].o[0];

    V_drone.x = stateGetSpeedNed_f()->x;
    V_drone.y = stateGetSpeedNed_f()->y;
    V_drone.z = stateGetSpeedNed_f()->z;

    aerolog.qi = q.qi;
    aerolog.qx = q.qx;
    aerolog.qy = q.qy;
    aerolog.qz = q.qz;

    meas_acc.x = filt_accel_ned[0].o[0];
    meas_acc.y = filt_accel_ned[1].o[0];
    meas_acc.z = filt_accel_ned[2].o[0];

    // ////printf("\n\npos_enu_i:\t%d\t%d\t%d\n", stateGetPositionEnu_i()->x,stateGetPositionEnu_i()->y,stateGetPositionEnu_i()->z);

    ////printf("\n\nmeas_acc:\t%f\t%f\t%f\n", meas_acc.x,meas_acc.y,meas_acc.z);
  
    // // Test Quaternion
    // V_drone.x = 5.0;
    // V_drone.y = 1.0;
    // V_drone.z = 2.0;
    
    // q.qi = sqrtf(2.0)/2;
    // q.qx = 0.0;
    // q.qy = sqrtf(2.0)/2;
    // q.qz = 0.0;

    Yaw_b.x = 0.0f;
    Yaw_b.y = 1.0f;
    Yaw_b.z = 0.0f;

    
    struct FloatQuat q_inv;
    float_quat_invert(&q_inv, &q);
    float_quat_vmult(&Yaw_bE, &q_inv, &Yaw_b);


    float mag;
    mag = sqrt(Yaw_bE.x*Yaw_bE.x+Yaw_bE.y*Yaw_bE.y);

    Yaw_proj.x = Yaw_bE.x/mag;
    Yaw_proj.y = Yaw_bE.y/mag;
    Yaw_proj.z = 0.0f;


    Yaw_ref.x = 1.0f/sqrt(2.0f);
    Yaw_ref.y = 1.0f/sqrt(2.0f);
    Yaw_ref.z = 0.0f;

    d_yaw = atan2f(Yaw_ref.y*Yaw_proj.x - Yaw_proj.y*Yaw_ref.x, Yaw_proj.x*Yaw_ref.x + Yaw_proj.y*Yaw_ref.y);

    if (d_yaw > 5.0f/M_PI){
      d_yaw = 5.0f/M_PI;
    } else if (d_yaw < -5.0f/M_PI){
      d_yaw = -5.0f/M_PI;
    }

    aerolog.d_yaw = d_yaw;

    // q.qi = -q.qi;
    V_drone.x = -V_drone.x;
    V_drone.y = -V_drone.y;
    V_drone.z = -V_drone.z;
    float_quat_vmult(&V_inf, &q, &V_drone);
    V_drone.x = -V_drone.x;
    V_drone.y = -V_drone.y;
    V_drone.z = -V_drone.z;
    // q.qi = -q.qi;

    // //Print Quaternion Check
    // ////printf("V_E: \t%f\t%f\t%f\t [m/s]\n",V_drone.x,V_drone.y,V_drone.z);
    // ////printf("qEb: \t%f\t%f\t%f\t%f\n",q.qi, q.qx, q.qy, q.qz);
    // ////printf("V_b: \t%f\t%f\t%f\t [m/s]\n\n",V_inf.x,V_inf.y,V_inf.z);

    V[0] = V_inf.x;
    V[1] = V_inf.y;
    V[2] = V_inf.z;   

    thrust_in = thrust_filt.o[0];

    // ////printf("\n\nV: %f \t %f \t %f\n",V[0],V[1],V[2]);   
}

void UpdateTrajectory(){
    traj.t = t;
    aerolog.t = t;
    aerolog.mode = varset.mode;

    traj.pos.x =  0.0f;
    traj.pos.y =  0.0f;
    traj.pos.z = -2.0f;

    traj.vel.x = 0.0f;
    traj.vel.y = 0.0f;
    traj.vel.z = 0.0f;

    traj.acc.x = 0.0f;
    traj.acc.y = 0.0f;
    traj.acc.z = 0.0f;





    if (varset.mode == 0) {

      traj.pos.x = traj_sec.pos_beg.x;
      traj.pos.y = traj_sec.pos_beg.y;
      traj.pos.z = traj_sec.pos_beg.z;
      
      traj.vel.x = 0.0f;
      traj.vel.y = 0.0f;
      traj.vel.z = 0.0f;

      traj.acc.x = 0.0f;
      traj.acc.y = 0.0f;
      traj.acc.z = 0.0f;

    } else if (varset.mode == 1){

      float new_pos;
      float new_vel;
      float new_acc;

      float i = 0;

      new_pos = 0.0f;
      new_vel = 0.0f;
      new_acc = 0.0f;
      if(traj_sec.type == 0 ){
        if (t >= traj_sec.t_beg && t <= traj_sec.t_end){
          i = (t-traj_sec.t_beg)/(traj_sec.t_end-traj_sec.t_beg);
          new_pos = traj_sec.delta_pos * ( i - sinf(2*M_PI*i)/(2*M_PI) );
          new_vel = traj_sec.delta_pos * ( 1 - cosf(2*M_PI*i) ) / (traj_sec.t_end-traj_sec.t_beg);             ;
          new_acc = traj_sec.delta_pos * (2*M_PI) * sinf(2*M_PI*i) / pow((traj_sec.t_end-traj_sec.t_beg),2);
        }else if(t > traj_sec.t_end){
          new_pos = traj_sec.delta_pos;
          new_vel = 0.0f;
          new_acc = 0.0f;
        }
      }else if(traj_sec.type == 1 ){
        if (t >= traj_sec.t_beg && t <= traj_sec.t_trans_beg){
          i = (t-traj_sec.t_beg)/(traj_sec.t_trans_beg-traj_sec.t_beg);
          new_pos = -traj_sec.vel_max*(traj_sec.t_trans_beg-traj_sec.t_beg)/(2*M_PI)*sinf(M_PI*i) + traj_sec.vel_max*(t-traj_sec.t_beg)/2.0;
          new_vel = traj_sec.vel_max/2.0*(1.0-cosf(M_PI*i));
          new_acc = M_PI*traj_sec.vel_max/(2.0*(traj_sec.t_trans_beg-traj_sec.t_beg))*sinf(M_PI*i);

        } else if (t >= traj_sec.t_trans_beg && t <= traj_sec.t_trans_end){
          new_pos = (t-traj_sec.t_trans_beg)*traj_sec.vel_max + traj_sec.d_x_trans;
          new_vel = traj_sec.vel_max;
          new_acc = 0.0f;
        } else if (t >= traj_sec.t_trans_end && t <= traj_sec.t_end){
          i = (t-traj_sec.t_trans_end)/(traj_sec.t_end-traj_sec.t_trans_end);
          new_pos = +traj_sec.vel_max*(traj_sec.t_trans_beg-traj_sec.t_beg)/(2*M_PI)*sinf(M_PI*i) - traj_sec.vel_max*(t-traj_sec.t_trans_end)/2.0 + traj_sec.vel_max * (t-traj_sec.t_trans_end) + traj_sec.delta_pos - traj_sec.d_x_trans;
          new_vel = -traj_sec.vel_max/2.0*(1.0-cosf(M_PI*i))+traj_sec.vel_max;
          new_acc = -M_PI*traj_sec.vel_max/(2.0*(traj_sec.t_end-traj_sec.t_trans_end))*sinf(M_PI*i);

        } else if (t >= traj_sec.t_end){
          new_pos = traj_sec.delta_pos;
          new_vel = 0.0f;
          new_acc = 0.0f;
        }

      }
      traj.pos.x = traj_sec.dir.x * new_pos + traj_sec.pos_beg.x;
      traj.pos.y = traj_sec.dir.y * new_pos + traj_sec.pos_beg.y;
      traj.pos.z = traj_sec.dir.z * new_pos + traj_sec.pos_beg.z;
      
      traj.vel.x = traj_sec.dir.x * new_vel;
      traj.vel.y = traj_sec.dir.y * new_vel;
      traj.vel.z = traj_sec.dir.z * new_vel;

      traj.acc.x = traj_sec.dir.x * new_acc;
      traj.acc.y = traj_sec.dir.y * new_acc;
      traj.acc.z = traj_sec.dir.z * new_acc;

    } else if (varset.mode == 2){

      traj.pos.x = traj_sec.pos_end.x;
      traj.pos.y = traj_sec.pos_end.y;
      traj.pos.z = traj_sec.pos_end.z;
      
      traj.vel.x = 0.0f;
      traj.vel.y = 0.0f;
      traj.vel.z = 0.0f;

      traj.acc.x = 0.0f;
      traj.acc.y = 0.0f;
      traj.acc.z = 0.0f;
    }



    aerolog.ref_pos_x = traj.pos.x;
    aerolog.ref_pos_y = traj.pos.y;
    aerolog.ref_pos_z = traj.pos.z;
    aerolog.ref_vel_x = traj.vel.x;
    aerolog.ref_vel_y = traj.vel.y;
    aerolog.ref_vel_z = traj.vel.z;
    aerolog.ref_acc_x = traj.acc.x;
    aerolog.ref_acc_y = traj.acc.y;
    aerolog.ref_acc_z = traj.acc.z;

    log_ref_pos[0] = aerolog.ref_pos_x;
    log_ref_pos[1] = aerolog.ref_pos_y;
    log_ref_pos[2] = aerolog.ref_pos_z;
    log_ref_vel[0] = aerolog.ref_vel_x;
    log_ref_vel[1] = aerolog.ref_vel_y;
    log_ref_vel[2] = aerolog.ref_vel_z;
    log_ref_acc[0] = aerolog.ref_acc_x;
    log_ref_acc[1] = aerolog.ref_acc_y;
    log_ref_acc[2] = aerolog.ref_acc_z;

    // #ifdef SITL
    // printf("\npos: %e %e %e",(int32_t)1000 * aerolog.pos_x, (int32_t)1000 * aerolog.pos_y, (int32_t)1000 * aerolog.pos_z);
    // #endif


    // //printf("\nPos: %f \t %f \t %f",traj.pos.x,traj.pos.y,traj.pos.z);
    if(traj_sec.type == 1){
    //printf("\nPos: %f \tVel: %f \tAcc: %f \ttime: %f \ttimeline: %f -> %f -> %f -> %f",new_pos,new_vel,new_acc,t,traj_sec.t_beg,traj_sec.t_trans_beg,traj_sec.t_trans_end,traj_sec.t_end);
    }
    // //printf("\nVel: %f \t %f \t %f",traj.vel.x,traj.vel.y,traj.vel.z);
    // //printf("\nAcc: %f \t %f \t %f",traj.acc.x,traj.acc.y,traj.acc.z);




    if (varset.mode != 0){
      if (t > (traj_sec.t_end + 10.0)){
        t=0;
        CalcNewTrajectory();
      }
    }

    // if (t <= 20 && t >= 0){    
    //   traj.pos.x = 7.5 * t - 47.7465 * sinf(0.15708*t);
    //   traj.vel.x = 15.0 * (0.5*sinf(t/20.*M_PI - M_PI/2.) + 0.5);
    //   traj.acc.x = 1.1781 * sinf((M_PI*t)/20.);
    // } else if (t > 20){
    //   traj.pos.x = 15. * (t-20.0) + 7.5*20.0;
    //   traj.vel.x = 15. ;
    //   traj.acc.x = 0.0 ;
    // }
    
    // if (t<20){
    //   traj.pos.x = 10.0*(1-(1- (t-5)/15)*(1- (t-5)/15))*sin(t*2*M_PI/20.);
    //   traj.pos.y = 10.0*(1-(1- (t-5)/15)*(1- (t-5)/15))*cos(t*2*M_PI/20.);
    //   traj.pos.z = -40.0*(1-(1- t/20)*(1-t/20));
    // }else{
    //   traj.pos.x = 10.0*sin(t*2*M_PI/20.);
    //   traj.pos.y = 10.0*cos(t*2*M_PI/20.);
    //   traj.pos.z = -40.0;

    // }

}

void LinAccCon(void){
    float PGain = varset.p_gain;
    float VGain = varset.d_gain;
    //PGain = PGain * VGain;

    ctrl_lin.pos.x = stateGetPositionNed_f()->x;
    ctrl_lin.pos.y = stateGetPositionNed_f()->y;
    ctrl_lin.pos.z = stateGetPositionNed_f()->z;
    ctrl_lin.vel.x = stateGetSpeedNed_f()->x;
    ctrl_lin.vel.y = stateGetSpeedNed_f()->y;
    ctrl_lin.vel.z = stateGetSpeedNed_f()->z;
    ctrl_lin.acc.x = meas_acc.x;
    ctrl_lin.acc.y = meas_acc.y;
    ctrl_lin.acc.z = meas_acc.z;

    aerolog.pos_x = ctrl_lin.pos.x;
    aerolog.pos_y = ctrl_lin.pos.y;
    aerolog.pos_z = ctrl_lin.pos.z;
    aerolog.vel_x = ctrl_lin.vel.x;
    aerolog.vel_y = ctrl_lin.vel.y;
    aerolog.vel_z = ctrl_lin.vel.z;
    aerolog.acc_x = ctrl_lin.acc.x;
    aerolog.acc_y = ctrl_lin.acc.y;
    aerolog.acc_z = ctrl_lin.acc.z;

    log_pos[0] = ctrl_lin.pos.x;
    log_pos[1] = ctrl_lin.pos.y;
    log_pos[2] = ctrl_lin.pos.z;
    log_vel[0] = ctrl_lin.vel.x;
    log_vel[1] = ctrl_lin.vel.y;
    log_vel[2] = ctrl_lin.vel.z;
    log_acc[0] = ctrl_lin.acc.x;
    log_acc[1] = ctrl_lin.acc.y;
    log_acc[2] = ctrl_lin.acc.z;

    log_gains[0] = varset.p_gain;
    log_gains[1] = varset.d_gain;

    ////printf("\nPos: %f \t %f \t %f",ctrl_lin.pos.x,ctrl_lin.pos.y,ctrl_lin.pos.z);
    ////printf("\nVel: %f \t %f \t %f",ctrl_lin.vel.x,ctrl_lin.vel.y,ctrl_lin.vel.z);
    ////printf("\nAcc: %f \t %f \t %f",ctrl_lin.acc.x,ctrl_lin.acc.y,ctrl_lin.acc.z);

    ctrl_lin.cmdacc.x = PGain * (traj.pos.x-ctrl_lin.pos.x) + VGain * (traj.vel.x-ctrl_lin.vel.x) + traj.acc.x;
    ctrl_lin.cmdacc.y = PGain * (traj.pos.y-ctrl_lin.pos.y) + VGain * (traj.vel.y-ctrl_lin.vel.y) + traj.acc.y;
    ctrl_lin.cmdacc.z = PGain * (traj.pos.z-ctrl_lin.pos.z) + VGain * (traj.vel.z-ctrl_lin.vel.z) + traj.acc.z;

    ////printf("\n\nacc_com:\t%f\t%f\t%f\n", ctrl_lin.cmdacc.x,ctrl_lin.cmdacc.y,ctrl_lin.cmdacc.z);

    ctrl_lin.cmdacc.x = ctrl_lin.cmdacc.x - ctrl_lin.acc.x;
    ctrl_lin.cmdacc.y = ctrl_lin.cmdacc.y - ctrl_lin.acc.y;
    ctrl_lin.cmdacc.z = ctrl_lin.cmdacc.z - ctrl_lin.acc.z;

   
    // #ifdef SITL
    //   printf("\nREF Pos: %f, %f, %f",(traj.pos.x),(traj.pos.y),(traj.pos.z));
    //   printf("\nREF Vel: %f, %f, %f",(traj.vel.x),(traj.vel.y),(traj.vel.z));
    //   printf("\nREF Acc: %f, %f, %f",(traj.acc.x),(traj.acc.y),(traj.acc.z));
    //   printf("\nMEAS Pos: %f, %f, %f",(ctrl_lin.pos.x),(ctrl_lin.pos.y),(ctrl_lin.pos.z));
    //   printf("\nMEAS Vel: %f, %f, %f",(ctrl_lin.vel.x),(ctrl_lin.vel.y),(ctrl_lin.vel.z));

    //   printf("\nTOT  Acc: %f, %f, %f",(ctrl_lin.cmdacc.x+meas_acc.x),(ctrl_lin.cmdacc.y+meas_acc.y),(ctrl_lin.cmdacc.z+meas_acc.z));
    //   printf("\nMEAS Acc: %f, %f, %f",(meas_acc.x),(meas_acc.y),(meas_acc.z));
  
    //   printf("\nCMD Pos: %f, %f, %f",(traj.pos.x-ctrl_lin.pos.x),(traj.pos.y-ctrl_lin.pos.y),(traj.pos.z-ctrl_lin.pos.z));
    //   printf("\nCMD Vel: %f, %f, %f",(traj.vel.x-ctrl_lin.vel.x),(traj.vel.y-ctrl_lin.vel.y),(traj.vel.z-ctrl_lin.vel.z));
    //   printf("\nCMD  Acc: %f, %f, %f",ctrl_lin.cmdacc.x,ctrl_lin.cmdacc.y,ctrl_lin.cmdacc.z);
    // #endif

    ////printf("\n\nacc_err:\t%f\t%f\t%f\n", ctrl_lin.cmdacc.x,ctrl_lin.cmdacc.y,ctrl_lin.cmdacc.z);

    ////printf("\n\nq:\t%f\t%f\t%f\t%f\n", q.qi,q.qx,q.qy,q.qz);
    V_temp.x = ctrl_lin.cmdacc.x;
    V_temp.y = ctrl_lin.cmdacc.y;
    V_temp.z = ctrl_lin.cmdacc.z;
    float_quat_vmult(&dv, &q, &V_temp);

    ////printf("\n\ndv:\t%f\t%f\t%f\n", dv.x,dv.y,dv.z);

    // if (dv.x > 0.1){
    //   dv.x = 0.1;
    // } else if (dv.x < -0.1){
    //   dv.x = -0.1;
    // }

    // if (dv.y > 0.1){
    //   dv.y = 0.1;
    // } else if (dv.y < -0.1){
    //   dv.y = -0.1;
    // }

    // if (dv.z > 2){
    //   dv.z = 2;
    // } else if (dv.z < -2){
    //   dv.z = -2;
    // }

    // ////printf("\n\nCommanded Acceleration: %f \t %f \t %f \t [m/s^2]\n", ctrl_lin.cmdacc.x, ctrl_lin.cmdacc.y, ctrl_lin.cmdacc.z);
}

void get_G(void){
    if (ROWS_GUIDANCE > COLS_GUIDANCE){
      G[0][0] =  da_wing_x_dphii(V); 
      G[0][1] =  da_wing_y_dphii(V) + 20.0 * thrust_filt.o[0]/MAX_PPRZ; 
      G[0][2] =  da_wing_z_dphii(V); 
      G[1][0] =  da_wing_x_dthetai(V) - 20.0 * thrust_filt.o[0]/MAX_PPRZ; 
      G[1][1] =  da_wing_y_dthetai(); 
      G[1][2] =  da_wing_z_dthetai(V); 
      G[2][0] =  0.0f; 
      G[2][1] =  0.0f; 
      // G[2][2] =  -20.0* thrust_in/MAX_PPRZ; 
      G[2][2] =  -20.0*480.0; 
    } else {
      G[0][0] =  da_wing_x_dphii(V); 
      G[1][0] =  da_wing_y_dphii(V)+ 20.0 * thrust_in/MAX_PPRZ; 
      G[2][0] =  da_wing_z_dphii(V); 
      G[0][1] =  da_wing_x_dthetai(V) - 20.0* thrust_in/MAX_PPRZ; 
      G[1][1] =  da_wing_y_dthetai(); 
      G[2][1] =  da_wing_z_dthetai(V); 
      G[0][2] =  0.0f; 
      G[1][2] =  0.0f; 
      // G[2][2] =  -20.0* thrust_in/MAX_PPRZ; 
      G[2][2] =  -20.0*480.0; 
    }
}

void get_increment(void){

    struct FloatMat33 Ga = {0.0f};
    struct FloatMat33 Ga_inv = {0.0f};

    for (int r = 0; r < ROWS_GUIDANCE; r++){
      for (int c = 0; c < COLS_GUIDANCE; c++){
        Ga.m[3*r+c] = G[r][c];
      }
    }    
    MAT33_INV(Ga_inv, Ga); 
    MAT33_VECT3_MUL(dua, Ga_inv, dv);

    du[0] = dua.x;
    du[1] = dua.y;
    du[2] = dua.z;

    du[2] = du[2] * 480.;

    float du_min =  960.-thrust_in; // min = 0    (defined by thrust_in = CMD_Thrust / 9600 * 20)
    float du_max = 8640.-thrust_in; // max = 20
    
    if (du[2] < du_min){
      du[2] = du_min;
    }else if (du[2] > du_max){
      du[2] = du_max;
    }

}

void add_increment(void){
    AbiSendMsgTHRUST(THRUST_INCREMENT_ID, du[2]);
    // ////printf("thrust filt: %f", thrust_filt.o[0]);
    thrust_in = thrust_filt.o[0] + du[2];
    if (thrust_in > 9600){
      thrust_in = 9600;
    } 
    if (thrust_in < 0 ){
      thrust_in = 0;
    }
    stabilization_cmd[COMMAND_THRUST] = thrust_in;

    struct FloatEulers rpy;
    // struct Int32Eulers rpyi;
    struct FloatQuat dq;
    struct FloatQuat q_res = {0};
    struct Int32Quat q_i_res = {0};


    aerolog.du0 = du[0];
    aerolog.du1 = du[1];
    aerolog.du1 = du[2];
    aerolog.thrust_in = thrust_in;

    float magdu;
    magdu = sqrt(du[0]*du[0]+du[1]*du[1]+d_yaw*d_yaw);

    dq.qi = cos(magdu);
    dq.qx = sin(magdu)*du[0]/magdu;
    dq.qy = sin(magdu)*du[1]/magdu;
    dq.qz = sin(magdu)*d_yaw/magdu;
    float_quat_comp(&q_res, &q, &dq);

    magdu = sqrt(q_res.qi*q_res.qi+q_res.qx*q_res.qx+q_res.qy*q_res.qy+q_res.qz*q_res.qz);

    q_res.qi = q_res.qi/magdu;//printf
    q_res.qx = q_res.qx/magdu;
    q_res.qy = q_res.qy/magdu;
    q_res.qz = q_res.qz/magdu;

    // if (magdu < 0.1){
    //   q_res.qi = q_res.qi/magdu;
    //   q_res.qx = q_res.qx/magdu;
    //   q_res.qy = q_res.qy/magdu;
    //   q_res.qz = q_res.qz/magdu;
    // } else {
    //   q_res.qi = 1.0;
    //   q_res.qx = 0.0;
    //   q_res.qy = 0.0;
    //   q_res.qz = 0.0;
    // }
    // ////printf("\n\nQ:  \t%f\t%f\t%f\t%f" , q.qi      , q.qx      , q.qy      , q.qz    );
    // ////printf("\ndQ:  \t%f\t%f\t%f\t%f", dq.qi     , dq.qx     , dq.qy     , dq.qz   );
    ////printf("\nQres:\t%f\t%f\t%f\t%f\n", q_res.qi  , q_res.qx  , q_res.qy  , q_res.qz);


    // q_res.qi = cosf(10.0/180*M_PI);
    // q_res.qx = 0.0;
    // q_res.qy = sinf(10.0/180*M_PI);
    // q_res.qz = 0.0;

    float_eulers_of_quat(&rpy, &q_res);

    // EULERS_BFP_OF_REAL(rpyi,rpy);

    QUAT_BFP_OF_REAL(q_i_res, q_res);
    QUAT_BFP_OF_REAL(stab_att_sp_quat, q_res);
    // stabilization_attitude_set_rpy_setpoint_i(&rpyi);
    stabilization_indi_attitude_run(q_i_res,true);

}

void update_sd_logger(void){
  sd_id = sd_id + 2;

  sd_log[0]  = *(int*)&ctrl_lin.pos.x;
  sd_log[1]  = *(int*)&ctrl_lin.pos.y;
  sd_log[2]  = *(int*)&ctrl_lin.pos.z;
  sd_log[3]  = *(int*)&ctrl_lin.vel.x;
  sd_log[4]  = *(int*)&ctrl_lin.vel.y;
  sd_log[5]  = *(int*)&ctrl_lin.vel.z;
  sd_log[6]  = *(int*)&ctrl_lin.acc.x;
  sd_log[7]  = *(int*)&ctrl_lin.acc.y;
  sd_log[8]  = *(int*)&ctrl_lin.acc.z;

  sd_log[9]  = *(int*)&traj.pos.x;
  sd_log[10] = *(int*)&traj.pos.y;
  sd_log[11] = *(int*)&traj.pos.z;

  sd_log[12] = *(int*)&q.qi;
  sd_log[13] = *(int*)&q.qx;
  sd_log[14] = *(int*)&q.qy;
  sd_log[15] = *(int*)&q.qz;

  sd_log[16] = *(int*)&du[0];
  sd_log[17] = *(int*)&du[1];
  sd_log[18] = *(int*)&d_yaw;
  sd_log[19] = *(int*)&du[2];
  sd_log[20] = *(int*)&thrust_in;

  sd_log[21] = stabilization_cmd[COMMAND_ROLL];
  sd_log[22] = stabilization_cmd[COMMAND_PITCH];
  sd_log[23] = stabilization_cmd[COMMAND_YAW];
  sd_log[24] = stabilization_cmd[COMMAND_THRUST];


  sd_log[25] = (int32_t) 10000 * 420;
  sd_log[26] = (int32_t) 1000 * 420;
  sd_log[27] = (int32_t) 100 * 420;
  sd_log[28] = (int32_t) 10 * 420;
  sd_log[29] = (int32_t) 420;
}

void print_states(void){

    ////printf("%f\t%f\t%f", ctrl_lin.pos.x, ctrl_lin.pos.y, ctrl_lin.pos.z);
    ////printf("\t%f\t%f\t%f", traj.pos.x, traj.pos.y, traj.pos.z);
    ////printf("\t%f\t%f\t%f", ctrl_lin.vel.x, ctrl_lin.vel.y, ctrl_lin.vel.z);
    ////printf("\t%f\t%f\t%f", traj.vel.x, traj.vel.y, traj.vel.z);
    ////printf("\t%f\t%f\t%f", ctrl_lin.acc.x, ctrl_lin.acc.y, ctrl_lin.acc.z);
    ////printf("\t%f\t%f\t%f", traj.acc.x, traj.acc.y, traj.acc.z);
    // ////printf("\npos_est [  m  ]: \t%f\t%f\t%f", ctrl_lin.pos->x, ctrl_lin.pos->y, ctrl_lin.pos->z);
    // ////printf("\npos_ref [  m  ]: \t%f\t%f\t%f", traj.pos.x, traj.pos.y, traj.pos.z);
    // ////printf("\nvel_est [ m/s ]: \t%f\t%f\t%f", ctrl_lin.vel->x, ctrl_lin.vel->y, ctrl_lin.vel->z);
    // ////printf("\nvel_ref [ m/s ]: \t%f\t%f\t%f", traj.vel.x, traj.vel.y, traj.vel.z);
    // ////printf("\nacc_est [m/s^2]: \t%f\t%f\t%f", ctrl_lin.acc->x, ctrl_lin.acc->y, ctrl_lin.acc->z);
    // ////printf("\nacc_ref [m/s^2]: \t%f\t%f\t%f", traj.acc.x, traj.acc.y, traj.acc.z);


    ////printf("\t%f\t%f\t%f\t%f",q.qi,q.qx,q.qy,q.qz);
    ////printf("\t%f\t%f\t%f",V[0],V[1],V[2]);

    ////printf("\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f",
    //G[0][0],G[0][1],G[0][2],G[0][3],
    //G[1][0],G[1][1],G[1][2],G[1][3],
    //G[2][0],G[2][1],G[2][2],G[2][3]);
    ////printf("\t%f\t%f\t%f", dv.x, dv.y, dv.z);
    ////printf("\t%f\t%f\t%f\t%f\n", du[0], du[1], du[2], du[3]);
    // ////printf("\nG: \t%f\t%f\t%f\t%f\n \t%f\t%f\t%f\t%f\n \t%f\t%f\t%f\t%f",
    // G[0][0],G[0][1],G[0][2],G[0][3],
    // G[1][0],G[1][1],G[1][2],G[1][3],
    // G[2][0],G[2][1],G[2][2],G[2][3]);
    // ////printf("\ndv [m/s^2]: \t%f\t%f\t%f", dv.x, dv.y, dv.z);
    // ////printf("\ndu [ 3xrad | N ]: \t%f\t%f\t%f\t%f", du[0], du[1], du[2], du[3]);
}

void CalcNewTrajectory(){
    
    // traj_sec.pos_beg.x = traj.pos.x;
    // traj_sec.pos_beg.y = traj.pos.y;
    // traj_sec.pos_beg.z = traj.pos.z;

    traj_sec.pos_beg.x = traj_sec.pos_end.x;
    traj_sec.pos_beg.y = traj_sec.pos_end.y;
    traj_sec.pos_beg.z = traj_sec.pos_end.z;
    
    TrajectoryScenario(); //get pos_end, v_max, a_max based on scenario

    // #ifdef SITL
    // printf("\nPos_beg: %f \t %f \t %f",traj_sec.pos_beg.x,traj_sec.pos_beg.y,traj_sec.pos_beg.z);
    // printf("\nPos_end: %f \t %f \t %f",traj_sec.pos_end.x,traj_sec.pos_end.y,traj_sec.pos_end.z);
    // printf("\nv_lim: %f\na_lim: %f",traj_sec.vel_max, traj_sec.acc_max); 
    // printf("\nv_lim: %f\na_lim: %f",varset.max_vel, varset.max_acc);
    // #endif
    
    traj_sec.delta_pos = sqrtf(pow(traj_sec.pos_end.x-traj_sec.pos_beg.x,2) +  pow(traj_sec.pos_end.y-traj_sec.pos_beg.y,2) +  pow(traj_sec.pos_end.z-traj_sec.pos_beg.z,2));
    traj_sec.dir.x = (traj_sec.pos_end.x-traj_sec.pos_beg.x)/traj_sec.delta_pos;
    traj_sec.dir.y = (traj_sec.pos_end.y-traj_sec.pos_beg.y)/traj_sec.delta_pos;
    traj_sec.dir.z = (traj_sec.pos_end.z-traj_sec.pos_beg.z)/traj_sec.delta_pos;

    float time_v_constraint = 2.0*traj_sec.delta_pos/traj_sec.vel_max;
    float time_a_constraint = sqrtf(2*M_PI*traj_sec.delta_pos/traj_sec.acc_max);

    if(time_a_constraint>time_v_constraint){
      traj_sec.type = 0;
      traj_sec.t_end = time_a_constraint;
      //printf("\ntimeline: %f -> %f",traj_sec.t_beg,traj_sec.t_end);
    } else {
      traj_sec.type = 1;
      traj_sec.t_trans_beg =  M_PI * traj_sec.vel_max/traj_sec.acc_max/2.0;
      traj_sec.d_x_trans = traj_sec.vel_max*traj_sec.t_trans_beg/2;
      traj_sec.t_end = (traj_sec.delta_pos-2*traj_sec.d_x_trans)/traj_sec.vel_max + 2* traj_sec.t_trans_beg;
      traj_sec.t_trans_end =  traj_sec.t_end-traj_sec.t_trans_beg;

      //printf("\ntimeline: %f -> %f -> %f -> %f",traj_sec.t_beg,traj_sec.t_trans_beg,traj_sec.t_trans_end,traj_sec.t_end);
      //printf("\ntransient distance: %f ",traj_sec.d_x_trans);
    }



}

void TrajectoryScenario(){
    traj_sec.number += 1;
    if (traj_sec.number >= 4){
      traj_sec.number = 0;
    } 

    traj_sec.pos_end.x =  y_coords[traj_sec.number];
    traj_sec.pos_end.y =  x_coords[traj_sec.number];

    traj_sec.vel_max = varset.max_vel; 
    traj_sec.acc_max = varset.max_acc;

    aerolog.max_vel = varset.max_vel;
    aerolog.max_acc = varset.max_acc;

}

// FlightPlan
void setReferenceHere(uint8_t wp){
    fix_pos.x = waypoint_get_x(wp);
    fix_pos.y = waypoint_get_y(wp);
    // fix_pos.z = waypoint_get_alt(wp);
    fix_pos.z = stateGetPositionNed_f()->z;

    traj.pos.x = fix_pos.x;
    traj.pos.y = fix_pos.y;
    traj.pos.z = fix_pos.z;

    traj_sec.pos_end.x = traj.pos.x;
    traj_sec.pos_end.y = traj.pos.y;
    traj_sec.pos_end.z = traj.pos.z;
    traj_sec.pos_beg.x = traj.pos.x;
    traj_sec.pos_beg.y = traj.pos.y;
    traj_sec.pos_beg.z = traj.pos.z;


    // printf("\nWP_REF: \t%f \t%f \t%f \nPOS: \t%f \t%f \t%f" , fix_pos.x, fix_pos.y, fix_pos.z, stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z);
  }

void makeWaypointsSquare(uint8_t wp_1, uint8_t wp_2, uint8_t wp_3, uint8_t wp_4){

    x_coords[0] = waypoint_get_x(wp_1);
    x_coords[1] = waypoint_get_x(wp_2);
    x_coords[2] = waypoint_get_x(wp_3);
    x_coords[3] = waypoint_get_x(wp_4);

    y_coords[0] = waypoint_get_y(wp_1);
    y_coords[1] = waypoint_get_y(wp_2);
    y_coords[2] = waypoint_get_y(wp_3);
    y_coords[3] = waypoint_get_y(wp_4);

    // printf("\nWPs: \t%f \t %f\n \t%f \t%f\n \t%f \t%f\n \t%f \t%f" , x_coords[0], y_coords[0], x_coords[1], y_coords[1], x_coords[2], y_coords[2], x_coords[3], y_coords[3]);
}

void getWaypoints(uint8_t wp_1, uint8_t wp_2, uint8_t wp_3, uint8_t wp_4){

    x_coords[0] = waypoint_get_x(wp_1);
    x_coords[1] = waypoint_get_x(wp_2);
    x_coords[2] = waypoint_get_x(wp_3);
    x_coords[3] = waypoint_get_x(wp_4);

    y_coords[0] = waypoint_get_y(wp_1);
    y_coords[1] = waypoint_get_y(wp_2);
    y_coords[2] = waypoint_get_y(wp_3);
    y_coords[3] = waypoint_get_y(wp_4);
    // printf("\nWPs: \t%f \t %f\n \t%f \t%f\n \t%f \t%f\n \t%f \t%f" , x_coords[0], y_coords[0], x_coords[1], y_coords[1], x_coords[2], y_coords[2], x_coords[3], y_coords[3]);

  }

// Filters
void filter_init(void){
    thrust_in = stabilization_cmd[COMMAND_THRUST];
    thrust_act = thrust_in;

    float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    for (int8_t i = 0; i < 3; i++) {
      init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
      init_butterworth_2_low_pass(&filt_vel_ned[i], tau, sample_time, 0.0);
      init_butterworth_2_low_pass(&filt_pos_ned[i], tau, sample_time, 0.0);
    }
    for (int8_t i = 0; i < 4; i++) {
      init_butterworth_2_low_pass(&filt_quat[i], tau, sample_time, 0.0);
    }
    // init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
    // init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
    init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
}

void guidance_indi_filter_thrust(void){
    // Actuator dynamics
    thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);
    // ////printf('thrust act: %f',thrust_act);
    // same filter as for the acceleration
    
    // ////printf("Thrust_act (input for filter): \t%f ",thrust_act);
    update_butterworth_2_low_pass(&thrust_filt, thrust_act);
}

void guidance_indi_propagate_filters(void){
    struct NedCoor_f *accel = stateGetAccelNed_f();
    update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
    update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
    update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

    struct NedCoor_f *vel = stateGetSpeedNed_f();
    update_butterworth_2_low_pass(&filt_vel_ned[0], vel->x);
    update_butterworth_2_low_pass(&filt_vel_ned[1], vel->y);
    update_butterworth_2_low_pass(&filt_vel_ned[2], vel->z);

    struct NedCoor_f *pos = stateGetPositionNed_f();
    update_butterworth_2_low_pass(&filt_pos_ned[0], pos->x);
    update_butterworth_2_low_pass(&filt_pos_ned[1], pos->y);
    update_butterworth_2_low_pass(&filt_pos_ned[2], pos->z);

    struct FloatQuat *quat = stateGetNedToBodyQuat_f();
    update_butterworth_2_low_pass(&filt_quat[0], quat->qi);
    update_butterworth_2_low_pass(&filt_quat[1], quat->qx);
    update_butterworth_2_low_pass(&filt_quat[2], quat->qy);
    update_butterworth_2_low_pass(&filt_quat[3], quat->qz);

    // ////printf("\nquat\n%f\t%f\t%f\t%f\n\n",quat->qi,quat->qx,quat->qy,quat->qz);
    ////printf("\nquat\n%f\t%f\t%f\t%f\n\n",stateGetNedToBodyQuat_f()->qi,stateGetNedToBodyQuat_f()->qx,stateGetNedToBodyQuat_f()->qy,stateGetNedToBodyQuat_f()->qz);


    // update_butterworth_2_low_pass(&roll_filt, eulers->phi);
    // update_butterworth_2_low_pass(&pitch_filt, eulers->theta);
}
