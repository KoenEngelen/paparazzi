/** @file firmwares/rotorcraft/guidance/guidance_aerobatic.c
 *  Reference generation for horizontal guidance.
 *
 */

//Include
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "firmwares/rotorcraft/guidance/guidance_aerobatic.h"
#include "math/pprz_algebra_float.h"
#include "filters/low_pass_filter.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "subsystems/radio_control.h"
#include "stabilization.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

//Define
#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 512
#endif
#define GUIDANCE_INDI_FILTER_CUTOFF 5.0
#define THRUST_DYNAMICS 0.03
// #define CMD_TO_THRUST 0.00240416666//(23.08/((float)(MAX_PPRZ)))
// #define MIN_THRUST 4.616 //(23.08 * 0.2)
// #define MAX_THRUST 20.772 //(23.08 * 0.9)
#define CMD_TO_THRUST 0.002718 //0.0019
#define MIN_THRUST 4.0 
#define MAX_THRUST 14.5 

struct filter_struct{
  Butterworth2LowPass acc_filter[3];
  Butterworth2LowPass q_filter[4];
  Butterworth2LowPass thrust_filter;
  struct NedCoor_f acc;
  struct FloatQuat q;
  float thrust;
} filt;
struct input_states {
    struct NedCoor_f pos;
    struct NedCoor_f vel;
    struct NedCoor_f acc;
    struct FloatQuat q;
    float thrust;
} in;
struct reference_states {
  struct NedCoor_f pos;
  struct NedCoor_f vel;
  struct NedCoor_f acc;
} ref;
struct output_states {
  struct FloatQuat q;
  float thrust;
} out;
struct ctrl_struct {
  float Gx;
  float Gy;
  float Gz;
  float ddx;
  float ddy;
  float ddz;
  float dphi;
  float dtheta;
  float dpsi;
  float dT;
  float dyaw;
} ctrl;

int32_t aerobatic_active;
float t = 0.0f;
struct NedCoor_f origin;

struct MySettings my_settings = {
  .offset_x  = GUIDANCE_AEROBATIC_X_OFF,
  .offset_y  = GUIDANCE_AEROBATIC_Y_OFF,
  .offset_z  = GUIDANCE_AEROBATIC_Z_OFF,
  .p_gain_x  = GUIDANCE_AEROBATIC_P_GAIN_X,
  .v_gain_x  = GUIDANCE_AEROBATIC_V_GAIN_X,
  .a_gain_x  = GUIDANCE_AEROBATIC_A_GAIN_X,
  .p_gain_y  = GUIDANCE_AEROBATIC_P_GAIN_Y,
  .v_gain_y  = GUIDANCE_AEROBATIC_V_GAIN_Y,
  .a_gain_y  = GUIDANCE_AEROBATIC_A_GAIN_Y,
  .p_gain_z  = GUIDANCE_AEROBATIC_P_GAIN_Z,
  .v_gain_z  = GUIDANCE_AEROBATIC_V_GAIN_Z,
  .a_gain_z  = GUIDANCE_AEROBATIC_A_GAIN_Z,
  .ctrl_mode  = GUIDANCE_AEROBATIC_CTRL_MODE,
  .ctrl_axis  = GUIDANCE_AEROBATIC_CTRL_AXIS,
};

struct GuidanceAerobaticStruct GAStates;

// Guidance V Module: UNUSED
void guidance_v_module_init(void){}
void guidance_v_module_enter(void){}
void guidance_v_module_run(UNUSED bool in_flight){}
//Guidance H Module
void guidance_h_module_read_rc(void){}
void guidance_h_module_init(void){
    guidance_aerobatic_init();
}
void guidance_h_module_enter(void){
    guidance_aerobatic_enter();
}
void guidance_h_module_run(UNUSED bool in_flight){
    guidance_aerobatic_run();
}

// Main functions

void guidance_aerobatic_init(void){
  filter_init();
}

void guidance_aerobatic_enter(void){
  // struct NedCoor_f ref = *stateGetPositionNed_f();
  origin.x = stateGetPositionNed_f()->x;
  origin.y = stateGetPositionNed_f()->y;
  origin.z = stateGetPositionNed_f()->z;
  ref.pos.x = origin.x;
  ref.pos.y = origin.y;
  ref.pos.z = origin.z;  
  ref.vel.x = 0.0;
  ref.vel.y = 0.0;
  ref.vel.z = 0.0;
  ref.acc.x = 0.0;
  ref.acc.y = 0.0;
  ref.acc.z = 0.0;
  filter_init();
  t = 0.0f;
}

void guidance_aerobatic_run(){
  aerobatic_active = 1;
  get_states();
  get_dyaw();
  get_dv();
  get_G();
  get_du();
  set_states();
  t += 1.0f/(float) PERIODIC_FREQUENCY;
}

// Controller functions

void get_states(void){

  // Input States
  in.q   = *stateGetNedToBodyQuat_f();
  in.pos = *stateGetPositionNed_f();
  in.vel = *stateGetSpeedNed_f();
  in.acc = *stateGetAccelNed_f();
  // Actuator Dynamics
  in.thrust = (1.0-THRUST_DYNAMICS)*in.thrust + THRUST_DYNAMICS*out.thrust;
  // Filter States
  filter_run();   
}
void get_dyaw(void){
  struct FloatVect3 Yaw_bE;
  struct FloatVect3 Yaw_b;
  struct FloatVect3 Yaw_proj;
  struct FloatVect3 Yaw_ref;

  Yaw_b.x = 1.0f;
  Yaw_b.y = 0.0f;
  Yaw_b.z = 0.0f;
  
  struct FloatQuat q_inv;
  float_quat_invert(&q_inv, &filt.q);
  float_quat_vmult(&Yaw_bE, &q_inv, &Yaw_b);

  float mag = sqrt(Yaw_bE.x*Yaw_bE.x+Yaw_bE.y*Yaw_bE.y);
  Yaw_proj.x = Yaw_bE.x/mag;
  Yaw_proj.y = Yaw_bE.y/mag;
  Yaw_proj.z = 0.0f;

  Yaw_ref.x = 1.0f;
  Yaw_ref.y = 0.0f;
  Yaw_ref.z = 0.0f;

  ctrl.dyaw = atan2f(Yaw_ref.y*Yaw_proj.x - Yaw_proj.y*Yaw_ref.x, Yaw_proj.x*Yaw_ref.x + Yaw_proj.y*Yaw_ref.y);

  float max_yaw = 90.0*M_PI/180.0;
  if (ctrl.dyaw > max_yaw){
      ctrl.dyaw = max_yaw;
  } else if (ctrl.dyaw < -max_yaw){
      ctrl.dyaw = -max_yaw;
  }

  switch (my_settings.ctrl_mode)
  {
  case 1:
    ctrl.dyaw = 0.0;
    break;
  }

}
void get_dv(void){  
  
  // Load Gain Settings
  float Kax = my_settings.a_gain_x;
  float Kay = my_settings.a_gain_y;
  float Kaz = my_settings.a_gain_z;
  float Kvx = my_settings.v_gain_x * Kax;
  float Kvy = my_settings.v_gain_y * Kay;
  float Kvz = my_settings.v_gain_z * Kaz;
  float Kpx = my_settings.p_gain_x * Kvx;
  float Kpy = my_settings.p_gain_y * Kvy;
  float Kpz = my_settings.p_gain_z * Kvz;

  switch (my_settings.ctrl_axis)
  {
  case 0:
    Kay = 0.0;
    Kvy = 0.0;
    Kpy = 0.0;
    Kaz = 0.0;
    Kvz = 0.0;
    Kpz = 0.0;
    break;
  
  case 1:
    Kax = 0.0;
    Kvx = 0.0;
    Kpx = 0.0;
    Kaz = 0.0;
    Kvz = 0.0;
    Kpz = 0.0;
    break;

  case 2:
    Kax = 0.0;
    Kvx = 0.0;
    Kpx = 0.0;
    Kay = 0.0;
    Kvy = 0.0;
    Kpy = 0.0;
    break;

  case 3:
    Kay = 0.0;
    Kvy = 0.0;
    Kpy = 0.0;
    break;

  case 4:
    Kax = 0.0;
    Kvx = 0.0;
    Kpx = 0.0;
    break;
  
  default:
    break;
  }

  switch (my_settings.ctrl_mode)
  {
  case 0:
    break;
  
  case 1:
    ref.pos.x = origin.x;
    ref.pos.y = origin.y + 20.0 + 20.0*sinf(0.314159*(t-5.0));
    ref.pos.z = origin.z - 25.0 - 25.0*sinf( 2.0*M_PI*(t-10.0)/20.0 + 1.2*sinf(2.0*M_PI*(t-10.0)/20.0) + 0.5*M_PI );  

    ref.vel.x = 0.0;
    ref.vel.y = 6.28319*cosf(0.314159*(t-5.0));
    ref.vel.z = -2.0*M_PI*25.0*(1.2*cosf(2.0*M_PI*(t-10.0)/20.0)+1.0)*cosf( 2.0*M_PI*(t-10.0)/20.0 + 1.2*sinf(2.0*M_PI*(t-10.0)/20.0) + 0.5*M_PI )/20.0;  

    ref.acc.x = 0.0;
    ref.acc.y = -1.973920*sinf(0.314159*(t-5.0));
    ref.acc.z = -(-2.0*M_PI*M_PI/(400.0)*25.0*(powf(1.2*cosf(2.0*M_PI*(t-10.0)/20.0)+1,2)*sinf(2.0*M_PI*(t-10.0)/20.0 + 1.2*sinf(2.0*M_PI*(t-10.0)/20.0) + 0.5*M_PI ))+1.2*sinf(2.0*M_PI*(t-10.0)/20.0)*cosf(1.2*sinf(2.0*M_PI*(t-10.0)/20.0)+2.0*M_PI*(t-10.0)/20.0 + 0.5*M_PI ));
    break;


  case 2:
    Kvx = 0.0;
    Kvy = 0.0;
    Kvz = 0.0;
    Kpx = 0.0;
    Kpy = 0.0;
    Kpz = 0.0;
    break;

  default:
    Kax = 0.0;
    Kay = 0.0;
    Kaz = 0.0;
    Kvx = 0.0;
    Kvy = 0.0;
    Kvz = 0.0;
    Kpx = 0.0;
    Kpy = 0.0;
    Kpz = 0.0;
    break;
  }


  // Incremental Acceleration in NED (da) and Body (da_b) Reference Frame 
  struct FloatVect3 da_p;
  struct FloatVect3 da_v;
  struct FloatVect3 da_a;
  struct FloatVect3 da_b_p;
  struct FloatVect3 da_b_v;
  struct FloatVect3 da_b_a;
  da_p.x = (my_settings.offset_x+ref.pos.x-in.pos.x);
  da_p.y = (my_settings.offset_y+ref.pos.y-in.pos.y);
  da_p.z = (my_settings.offset_z+ref.pos.z-in.pos.z);
  da_v.x = (ref.vel.x-in.vel.x);
  da_v.y = (ref.vel.y-in.vel.y);
  da_v.z = (ref.vel.z-in.vel.z);
  da_a.x = (ref.acc.x - filt.acc.x);
  da_a.y = (ref.acc.y - filt.acc.y);
  da_a.z = (ref.acc.z - filt.acc.z);
  float_quat_vmult(&da_b_p, &filt.q, &da_p);
  float_quat_vmult(&da_b_v, &filt.q, &da_v);
  float_quat_vmult(&da_b_a, &filt.q, &da_a);

  struct FloatVect3 da_tot;
  struct FloatVect3 da_b_tot;
  da_tot.x = Kpx * da_p.x + Kvx * da_v.x + Kax * da_a.x;
  da_tot.y = Kpy * da_p.y + Kvy * da_v.y + Kay * da_a.y;
  da_tot.z = Kpz * da_p.z + Kvz * da_v.z + Kaz * da_a.z;
  float_quat_vmult(&da_b_tot, &filt.q, &da_tot);

  // Output Incremental Acceleration
  ctrl.ddx = da_b_tot.x;
  ctrl.ddy = da_b_tot.y;
  ctrl.ddz = da_b_tot.z;


  // ctrl.ddx = Kpx * da_b_p.x + Kvx * da_b_v.x + Kax * da_b_a.x;
  // ctrl.ddy = Kpy * da_b_p.y + Kvy * da_b_v.y + Kay * da_b_a.y;
  // ctrl.ddz = Kpz * da_b_p.z + Kvz * da_b_v.z + Kaz * da_b_a.z;

}

void get_G(void){

  //Reduced Control Effectiveness
  ctrl.Gx = -filt.thrust;
  ctrl.Gy =  filt.thrust;
  ctrl.Gz = -1.0f;  
}

void get_du(void){

  // Output Incremental Orientation, Thrust
  ctrl.dphi   = ctrl.ddy/ctrl.Gy;
  ctrl.dtheta = ctrl.ddx/ctrl.Gx;
  ctrl.dpsi   = ctrl.dyaw;
  ctrl.dT     = ctrl.ddz/ctrl.Gz;
  
  // Clamp Incremental Thrust
  float MIN_DT = (MIN_THRUST) - filt.thrust; 
  float MAX_DT = (MAX_THRUST) - filt.thrust; 
  ctrl.dT = fmaxf(MIN_DT,fminf(MAX_DT,ctrl.dT)); // clamp
}

void set_states(void){
  float sum;
  float length;

  //Set Thrust State
  out.thrust = filt.thrust + ctrl.dT;
  out.thrust = fmaxf(MIN_THRUST,fminf(MAX_THRUST,out.thrust)); // clamp 
  stabilization_cmd[COMMAND_THRUST] = (int)(out.thrust/(CMD_TO_THRUST));

  // Set Quaternion State
  struct FloatQuat dq;
  sum = powf(ctrl.dphi,2) 
      + powf(ctrl.dtheta,2)
      + powf(ctrl.dpsi,2);
  length = sqrtf(sum);
  // Test Theta only
  dq.qi = cosf(length/2.0);
  dq.qx = sinf(length/2.0)*ctrl.dphi/length;
  dq.qy = sinf(length/2.0)*ctrl.dtheta/length;
  dq.qz = sinf(length/2.0)*ctrl.dpsi/length;
  // Add dQ to Quaternion
  struct FloatQuat q_res = {0};
  float_quat_comp(&q_res, &filt.q, &dq);
  // Normalize Resultant Quaternion
  sum = powf(q_res.qi,2) 
      + powf(q_res.qx,2)
      + powf(q_res.qy,2)
      + powf(q_res.qz,2);
  length = sqrtf(sum);
  q_res.qi = q_res.qi/length;
  q_res.qx = q_res.qx/length;
  q_res.qy = q_res.qy/length;
  q_res.qz = q_res.qz/length;
  // Write Quaterniojn to Stabilization
  struct Int32Quat q_i_res = {0};
  QUAT_BFP_OF_REAL(q_i_res, q_res);
  stabilization_indi_attitude_run(q_i_res,true);

  GAStates.pos_x = in.pos.x;
  GAStates.pos_y = in.pos.y;
  GAStates.pos_z = in.pos.z;
  GAStates.ref_pos_x = ref.pos.x + my_settings.offset_x;
  GAStates.ref_pos_y = ref.pos.y + my_settings.offset_y;
  GAStates.ref_pos_z = ref.pos.z + my_settings.offset_z;
  GAStates.vel_x = in.vel.x;
  GAStates.vel_y = in.vel.y;
  GAStates.vel_z = in.vel.z;
  GAStates.ref_vel_x = ref.vel.x;
  GAStates.ref_vel_y = ref.vel.y;
  GAStates.ref_vel_z = ref.vel.z;
  GAStates.acc_x = in.acc.x;
  GAStates.acc_y = in.acc.y;
  GAStates.acc_z = in.acc.z;
  GAStates.filt_acc_x = filt.acc.x;
  GAStates.filt_acc_y = filt.acc.y;
  GAStates.filt_acc_z = filt.acc.z;
  GAStates.ref_acc_x = ref.acc.x;
  GAStates.ref_acc_y = ref.acc.y;
  GAStates.ref_acc_z = ref.acc.z;
  GAStates.quat_qi = in.q.qi;
  GAStates.quat_qx = in.q.qx;
  GAStates.quat_qy = in.q.qy;
  GAStates.quat_qz = in.q.qz;
  GAStates.filt_quat_qi = filt.q.qi;
  GAStates.filt_quat_qx = filt.q.qx;
  GAStates.filt_quat_qy = filt.q.qy;
  GAStates.filt_quat_qz = filt.q.qz;
}

// Filters

void filter_init(void){
  
  // Initial Values: Thrust, Quaterion
  in.thrust = (CMD_TO_THRUST)*stabilization_cmd[COMMAND_THRUST];
  in.thrust = fmaxf(MIN_THRUST,fminf(MAX_THRUST,in.thrust)); // clamp 
  out.thrust = in.thrust;
  in.q = *stateGetNedToBodyQuat_f();

  // Filter Settings
  float tau = 1.0 / (2.0 * M_PI * GUIDANCE_INDI_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  
  // Initialize Filters: Thrust, Acceleration, Quaternion
  init_butterworth_2_low_pass(&filt.thrust_filter, tau, sample_time, in.thrust);
  init_butterworth_2_low_pass(&filt.acc_filter[0], tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&filt.acc_filter[1], tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&filt.acc_filter[2], tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&filt.q_filter[0], tau, sample_time, in.q.qi);
  init_butterworth_2_low_pass(&filt.q_filter[1], tau, sample_time, in.q.qx);
  init_butterworth_2_low_pass(&filt.q_filter[2], tau, sample_time, in.q.qy);
  init_butterworth_2_low_pass(&filt.q_filter[3], tau, sample_time, in.q.qz);
}

void filter_run(void){

  // Update Filters: Thrust, Acceleration, Quaternion
  update_butterworth_2_low_pass(&filt.thrust_filter, in.thrust);
  update_butterworth_2_low_pass(&filt.acc_filter[0], in.acc.x);
  update_butterworth_2_low_pass(&filt.acc_filter[1], in.acc.y);
  update_butterworth_2_low_pass(&filt.acc_filter[2], in.acc.z);
  update_butterworth_2_low_pass(&filt.q_filter[0], in.q.qi);
  update_butterworth_2_low_pass(&filt.q_filter[1], in.q.qx);
  update_butterworth_2_low_pass(&filt.q_filter[2], in.q.qy);
  update_butterworth_2_low_pass(&filt.q_filter[3], in.q.qz);
  
  // Write Filter Outputs to Structs: Thrust, Acceleration, Quaternion
  filt.thrust = filt.thrust_filter.o[0];
  filt.acc.x = filt.acc_filter[0].o[0];
  filt.acc.y = filt.acc_filter[1].o[0];
  filt.acc.z = filt.acc_filter[2].o[0];
  float sum = 0.0;
  for (int i = 0; i < 4; i++){
    sum += powf(filt.q_filter[i].o[0],2);
  }
  float length = sqrtf(sum);
  filt.q.qi = filt.q_filter[0].o[0]/length;
  filt.q.qx = filt.q_filter[1].o[0]/length;
  filt.q.qy = filt.q_filter[2].o[0]/length;
  filt.q.qz = filt.q_filter[3].o[0]/length;
}