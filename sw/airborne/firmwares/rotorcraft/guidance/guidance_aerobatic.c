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
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "modules/sensors/airspeed_ms45xx_i2c.h"
#include "modules/sensors/aoa_adc.h"
#include "subsystems/imu.h"
// #include "SF1eFilter.h"
// #include "SF1eFilter.c"
//Define
#ifndef PPRZ_MSG_ID_AEROBATIC_INDI
#define PPRZ_MSG_ID_AEROBATIC_INDI 158
#endif

#ifndef PPRZ_MSG_ID_AEROBATIC_LOG
#define PPRZ_MSG_ID_AEROBATIC_LOG 172
#endif

#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 512
#endif

// #ifndef GUIDANCE_AEROBATIC_USE_FEEDFORWARD
// #define GUIDANCE_AEROBATIC_USE_FEEDFORWARD FALSE
// #endif

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
  #ifdef STABILIZATION_INDI_FILT_CUTOFF
    #define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
  #else
    #define GUIDANCE_INDI_FILTER_CUTOFF 3.0
  #endif
#endif

#define MAX_PPRZ_f 9600.0
#define THRUST_MIN 1920 //0.20*MAX_PPRZ //TODO lower min thrust (Check)
#define THRUST_MAX 8640 //0.90*MAX_PPRZ
#define THRUST_DYNAMICS 0.03
#define MIN_THRUST_EFFECTIVNESS 4.0

#define ROWS_GUIDANCE 3 //Control Effectiveness Rows
#define COLS_GUIDANCE 3 //Control Effectiveness Columns

struct filter_struct{
  Butterworth2LowPass acc_filter[3];
  Butterworth2LowPass vel_filter[3];
  Butterworth2LowPass pos_filter[3];
  Butterworth2LowPass q_filter[4];
  Butterworth2LowPass qout_filter[4];
  Butterworth2LowPass thrust_filter;
  Butterworth2LowPass AoA_filter;
  Butterworth2LowPass ay_filter;
  Butterworth2LowPass V_filter;
  struct NedCoor_f pos;
  struct NedCoor_f vel;
  struct NedCoor_f acc;
  struct FloatQuat q;
  struct FloatQuat qout;
  float thrust;
  float V;
  float ay;
  float AoA;
} filt;
struct input_states {
    struct NedCoor_f pos;
    struct NedCoor_f vel;
    struct NedCoor_f acc;
    struct NedCoor_f air;
    struct FloatQuat q;
    float thrust;
    float AoA;
    float V;
    float ay;
    int32_t thrust_cmd;
} in;
struct reference_states {
    struct NedCoor_f pos;
    struct NedCoor_f vel;
    struct NedCoor_f acc;
} ref;
struct output_states {
  struct FloatQuat q;
  float thrust;
  int32_t thrust_cmd;
} out;
struct ctrl_struct3 {
  float G[3][3];
  struct FloatVect3 dv; 
  struct FloatVect3 du;
  float dyaw;
} ctrl3;

struct ctrl_struct4 {
  float G[16];
  float dv[4]; 
  float du[4];
} ctrl4;

struct ctrl_struct {
  float dphi;
  float dtheta;
  float dpsi;
  float dT;
} ctrl;

struct wp_struct {
  uint8_t stay_wp;
  uint8_t circle;
  uint8_t start;
  uint8_t dir;
} wps;


struct trajectory {
    float t;
    struct NedCoor_f pos;
    struct NedCoor_f vel;
    struct NedCoor_f acc;
} traj;
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
struct AerobaticVariableSettings varset = {
    .max_vel = GUIDANCE_AEROBATIC_MAX_VEL,
    .max_acc = GUIDANCE_AEROBATIC_MAX_ACC,
    .p_gain  = GUIDANCE_AEROBATIC_P_GAIN,
    .d_gain  = GUIDANCE_AEROBATIC_D_GAIN,
    .z_p_factor  = GUIDANCE_AEROBATIC_Z_P_FACTOR,
    .z_d_factor  = GUIDANCE_AEROBATIC_Z_D_FACTOR,
    .mode = GUIDANCE_AEROBATIC_MODE,
    .use_feedforward = GUIDANCE_AEROBATIC_USE_FEEDFORWARD,
    .use_alt_gains = GUIDANCE_AEROBATIC_USE_ALT_GAINS,
    .use_filtered_output = GUIDANCE_AEROBATIC_USE_FILTERED_OUTPUT,
    .ce_mode = GUIDANCE_AEROBATIC_CE_MODE
    // .min_thrust_eff = GUIDANCE_AEROBATIC_MIN_THRUST_EFFECT
};

struct GuidanceLogStruct aerolog = {
    .t = 0.0f,
    .mode = GUIDANCE_AEROBATIC_MODE,
    .max_vel = GUIDANCE_AEROBATIC_MAX_VEL,
    .max_acc = GUIDANCE_AEROBATIC_MAX_ACC,
    .q[0] = 0,
    .q[1] = 0,
    .q[2] = 0,
    .q[3] = 0,
    .pos[0] = 0.0f,
    .pos[1] = 0.0f,
    .pos[2] = 0.0f,
    .vel[0] = 0.0f,
    .vel[1] = 0.0f,
    .vel[2] = 0.0f,
    .acc[0] = 0.0f,
    .acc[1] = 0.0f,
    .acc[2] = 0.0f,
    .ref_pos[0] = 0.0f,
    .ref_pos[1] = 0.0f,
    .ref_pos[2] = 0.0f,
    .ref_vel[0] = 0.0f,
    .ref_vel[1] = 0.0f,
    .ref_vel[2] = 0.0f,
    .ref_acc[0] = 0.0f,
    .ref_acc[1] = 0.0f,
    .ref_acc[2] = 0.0f,
    .air[0] = 0.0f,
    .air[1] = 0.0f,
    .air[2] = 0.0f,
    .thrust_in = 0.0f,
    .dyaw = 0.0f,
    .du[0] = 0.0f,
    .du[1] = 0.0f,
    .du[2] = 0.0f,
    .gains[0] = 0.0f,
    .gains[1] = 0.0f
};

float t = 0.0;
int wp_id = 0;

struct NedCoor_f ref_pos;
struct NedCoor_f cur_pos;
struct NedCoor_f circ_pos;

struct NedCoor_f Home;

bool is_killed = false;

bool mode_disabled_altitude_gains = false;
bool mode_disabled_Kp = false;
bool mode_disabled_Kd = false;
int8_t active_mode = 0;

float mode_2_radius = 0.0f;
float mode_2_angle = 0.0f;

float THRUST_TO_CMD = MAX_PPRZ_f/20.0;
float CMD_TO_THRUST = 20.0/MAX_PPRZ_f;

float x_coords[4] = {0};
float y_coords[4] = {0};

int32_t sd_id = 0;
int32_t sd_log[30];

// SF1eFilter f_acc_x;
// SF1eFilter f_acc_y;
// SF1eFilter f_acc_z;
// SF1eFilter f2_acc_x;
// SF1eFilter f2_acc_y;
// SF1eFilter f2_acc_z;

// struct FloatVect3 f_acc;
// struct FloatVect3 f2_acc;

// Guidance V Module: UNUSED
void guidance_v_module_init(void){}
void guidance_v_module_enter(void){}
void guidance_v_module_run(UNUSED bool in_flight){}
//Guidance H Module
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
    guidance_aerobatic_run();
}
//Messages
static void send_aerobatic_INDI(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_AEROBATIC_INDI(trans, dev, AC_ID,
                         &varset.max_vel,
                         &varset.max_acc,
                         &varset.p_gain,
                         &varset.d_gain,
                         &varset.mode,
                         &varset.ce_mode,
                         3,&aerolog.air[0]);
}
static void send_aerobatic_log(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_AEROBATIC_LOG(trans, dev, AC_ID,
                         3,&aerolog.pos[0],
                         3,&aerolog.vel[0],
                         3,&aerolog.acc[0],
                         3,&aerolog.ref_pos[0],
                         3,&aerolog.ref_vel[0],
                         3,&aerolog.ref_acc[0],
                         3,&aerolog.air[0],
                         2,&aerolog.gains[0]);
}
// Main functions
void guidance_aerobatic_init(void){
  t = -5.0;

  filter_init();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_LOG, send_aerobatic_log);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_INDI, send_aerobatic_INDI);
 }
void guidance_aerobatic_enter(void){
    t = 0.0;
    // getWaypoints()

    ref_pos = *stateGetPositionNed_f();
    in.thrust = CMD_TO_THRUST*stabilization_cmd[COMMAND_THRUST];
    out.thrust = in.thrust;

    // varset.max_vel = 1.0;
    // varset.max_acc = 1.0;

    traj_sec.type = 0;
    traj_sec.number = -1;
    traj_sec.t_beg = 0.0f;
    CalcNewTrajectory();

    ref.pos.x = stateGetPositionNed_f()->x;
    ref.pos.y = stateGetPositionNed_f()->y;
    ref.pos.z = stateGetPositionNed_f()->z;
    ref.vel.x = 0.0f;
    ref.vel.y = 0.0f;
    ref.vel.z = 0.0f;
    ref.acc.x = 0.0f;
    ref.acc.y = 0.0f;
    ref.acc.z = 0.0f;

    filter_init();

}
void guidance_aerobatic_run(){
  static int32_t counter = 0;
  t = t + 0.001953125;
  if (t > 360000){
    t = 0;
  }
  get_states();
  get_mode();
  if (varset.ce_mode == 1){
    YawControl();
  }
  Controller();
  get_G();
  get_du();
  set_states();

 

  message_log();
  if (counter%2){
    update_sd_logger();
  }
  counter++;
}
// Controller functions
void get_states(void){
  in.q   = *stateGetNedToBodyQuat_f();
  in.pos = *stateGetPositionNed_f();
  in.vel = *stateGetSpeedNed_f();
  in.acc = *stateGetAccelNed_f();

  in.AoA = 0.0;
  if (!isnan(aoa_adc.angle)){
    in.AoA = -aoa_adc.angle;
  }
  in.V = 0.0;
  if (!isnan(ms45xx.airspeed)){
    in.V = ms45xx.airspeed;
  }
  in.ay = ACCEL_FLOAT_OF_BFP(imu.accel.y);

  // Actuator Dynamics 
  // float dthrust = ((float) out.thrust_cmd)*CMD_TO_THRUST -  filt.thrust;   
  // in.thrust = filt.thrust + THRUST_DYNAMICS * dthrust;
  in.thrust = (1.0-THRUST_DYNAMICS)*in.thrust + THRUST_DYNAMICS*out.thrust;

  filter_run();



  struct FloatVect3 ab;
  struct FloatVect3 ag;
  ag.x = 0.0;
  ag.y = 0.0;
  ag.z = 9.81;
  float_quat_vmult(&ab, &filt.q, &ag);
  float aymg = filt.ay - ab.y;



  float_quat_vmult(&ab, &filt.q, &filt.acc);
  float Vy = sqrtf(fabsf(5.0*aymg));
  float Vxz = filt.V * powf(cosf(filt.AoA),2.0) + powf(sinf(filt.AoA),2.0) * sqrtf(5.0*sqrtf(fabsf(ab.x*ab.x + ab.z*ab.z)));

  float Vz = cosf(filt.AoA)*Vxz;
  float Vx = - sinf(filt.AoA)*Vxz;

  in.air.x = Vx ;
  in.air.y = Vy ;
  in.air.z = Vz ;

  aerolog.air[0] =  in.air.x;
  aerolog.air[1] =  in.air.y;
  aerolog.air[2] =  in.air.z;



  struct FloatVect3 V_drone;
  V_drone.x = in.vel.x;
  V_drone.y = in.vel.y;
  V_drone.z = in.vel.z;

  V_drone.x = -V_drone.x;
  V_drone.y = -V_drone.y;
  V_drone.z = -V_drone.z;

  struct FloatVect3 V_inf;
  float_quat_vmult(&V_inf, &filt.q, &V_drone);
  V_drone.x = -V_drone.x;
  V_drone.y = -V_drone.y;
  V_drone.z = -V_drone.z;

  // in.air.x = V_inf.x;
  // in.air.y = V_inf.y;
  // in.air.z = V_inf.z;   
}
void YawControl(void){
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

  float mag;
  mag = sqrt(Yaw_bE.x*Yaw_bE.x+Yaw_bE.y*Yaw_bE.y);

  Yaw_proj.x = Yaw_bE.x/mag;
  Yaw_proj.y = Yaw_bE.y/mag;
  Yaw_proj.z = 0.0f;


  Yaw_ref.x = 1.0f/sqrt(2.0f);
  Yaw_ref.y = -1.0f/sqrt(2.0f);
  Yaw_ref.z = 0.0f;

  // Yaw_ref.x = 0.0f;
  // Yaw_ref.y = 1.0f;
  // Yaw_ref.z = 0.0f;

  float dposx = Home.x - in.pos.x;
  float dposy = Home.y - in.pos.y;

  float dmag = sqrtf(powf(dposx,2.0) +  powf(dposx,2));

  // Yaw_ref.x = dposx/dmag;
  // Yaw_ref.y = dposy/dmag;

  ctrl3.dyaw = atan2f(Yaw_ref.y*Yaw_proj.x - Yaw_proj.y*Yaw_ref.x, Yaw_proj.x*Yaw_ref.x + Yaw_proj.y*Yaw_ref.y);

  if (ctrl3.dyaw > 5.0f/M_PI){
      ctrl3.dyaw = 5.0f/M_PI;
  } else if (ctrl3.dyaw < -5.0f/M_PI){
      ctrl3.dyaw = -5.0f/M_PI;
  }
  ctrl3.dyaw = 0.0;
}
void Controller(void){
  // float PGain = varset.p_gain;
  // if (mode_disabled_Kp){
  //   float PGain = 0.0;
  // }
  // float VGain = varset.d_gain;
  // if (mode_disabled_Kd){
  //   float VGain = 0.0;
  // }
  // float zPFactor = GUIDANCE_AEROBATIC_Z_P_FACTOR * PGain;
  // float zVFactor = GUIDANCE_AEROBATIC_Z_D_FACTOR * VGain;
  // if (!mode_disabled_altitude_gains){
  //   if (mode_disabled_Kp){
  //     float zPFactor = GUIDANCE_AEROBATIC_Z_P_FACTOR;
  //   }
  //   if (mode_disabled_Kd){
  //     float zVFactor = GUIDANCE_AEROBATIC_Z_D_FACTOR;
  //   }
  // }
  float PGain = varset.p_gain;
  float VGain = varset.d_gain;

  float zPFactor = GUIDANCE_AEROBATIC_Z_P_FACTOR * PGain;
  float zVFactor = GUIDANCE_AEROBATIC_Z_D_FACTOR * VGain;

  struct FloatVect3 da_p;
  struct FloatVect3 da_d;
  struct FloatVect3 da;
  
  float ddx = 0.0;
  float ddy = 0.0;
  float dx = 0.0;
  float dy = 0.0;
  float dz = 0.0;

  // if(varset.mode == 1){
  //   dx = varset.max_acc * (radio_control.values[1]-radio_control.values[2])/9600.0;
  //   dy = varset.max_acc * (radio_control.values[2]-radio_control.values[1])/9600.0;
  // }else if(varset.mode == 1.5){
  //   dz = -radio_control.values[2]/19200.0;
  // }else if(varset.mode == 2){
  //   ddx =  varset.max_vel * (radio_control.values[1]-radio_control.values[2])/9600.0;
  //   ddy =  varset.max_vel * (radio_control.values[2]-radio_control.values[1])/9600.0;
  //   PGain = 0.0;
  // }
  
  da_p.x = PGain * (ref.pos.x-in.pos.x);
  da_p.y = PGain * (ref.pos.y-in.pos.y);
  da_p.z = zPFactor * (ref.pos.z-in.pos.z);

  da_d.x = VGain * (ref.vel.x-in.vel.x);
  da_d.y = VGain * (ref.vel.y-in.vel.y);
  da_d.z = zVFactor * (ref.vel.z-in.vel.z);

  da.x = da_p.x + da_d.x + ref.acc.x - filt.acc.x;
  da.y = da_p.y + da_d.y + ref.acc.y - filt.acc.y;
  da.z = da_p.z + da_d.z + ref.acc.z - filt.acc.z;

  float_quat_vmult(&ctrl3.dv, &filt.q, &da);
}
void get_G(void){ 
  float V[3];
  V[0] = in.air.x;
  V[1] = in.air.y;
  V[2] = in.air.z;

  float dbdt = -in.air.y*in.air.x;
  float dbdp = -in.air.x*in.air.z;
  float V2yz = powf(in.air.y,2.0)+powf(in.air.z,2.0);
  if (V2yz > 1.0f){
    dbdt = dbdt/V2yz;
    dbdp = dbdp/V2yz;
  }
  if (isnan(dbdt)){
    dbdt = 0.0f;
  }
  if (isnan(dbdp)){
    dbdp = 0.0f;
  }

  switch (varset.ce_mode)
  {
  case 1:
    ctrl3.G[0][0] =  0.0f * varset.max_acc * da_wing_x_dphii(V)  ; 
    ctrl3.G[1][0] =  0.0f * varset.max_acc * da_wing_y_dphii(V)   + varset.max_acc * fmaxf(filt.thrust,MIN_THRUST_EFFECTIVNESS); 
    ctrl3.G[2][0] =  0.0f * varset.max_acc * da_wing_z_dphii(V)  ; 
    ctrl3.G[0][1] =  0.0f * varset.max_acc * da_wing_x_dthetai(V) - varset.max_acc * fmaxf(filt.thrust,MIN_THRUST_EFFECTIVNESS); 
    ctrl3.G[1][1] =  0.0f * varset.max_acc * da_wing_y_dthetai() ; 
    ctrl3.G[2][1] =  0.0f * varset.max_acc * da_wing_z_dthetai(V); 
    ctrl3.G[0][2] =  0.0f; 
    ctrl3.G[1][2] =  0.0f; 
    ctrl3.G[2][2] = -1.0f * varset.max_acc; 
    break;
  
  case 2:
    ctrl4.G[ 0] =  da_wing_x_dphii(V)  ; 
    ctrl4.G[ 1] =  da_wing_y_dphii(V)  + fmaxf(filt.thrust,MIN_THRUST_EFFECTIVNESS); 
    ctrl4.G[ 2] =  da_wing_z_dphii(V)  ; 
    ctrl4.G[ 3] =  1.0;
    ctrl4.G[ 4] =  da_wing_x_dthetai(V)- fmaxf(filt.thrust,MIN_THRUST_EFFECTIVNESS); 
    ctrl4.G[ 5] =  da_wing_y_dthetai() ; 
    ctrl4.G[ 6] =  da_wing_z_dthetai(V); 
    ctrl4.G[ 7] =  dbdt;
    ctrl4.G[ 8] =  da_wing_x_dpsii(V)  ;  
    ctrl4.G[ 9] =  da_wing_y_dpsii(V)  ; 
    ctrl4.G[10] =  da_wing_z_dpsii(V)  ; 
    ctrl4.G[11] =  dbdp;
    ctrl4.G[12] =  0.0f;
    ctrl4.G[13] =  0.0f;
    ctrl4.G[14] = -1.0f;
    ctrl4.G[15] =  0.0f;
    break;
  
  default:
    ctrl4.G[ 0] =  da_wing_x_dphii(V)  ; 
    ctrl4.G[ 1] =  da_wing_y_dphii(V)  + fmaxf(filt.thrust,MIN_THRUST_EFFECTIVNESS); 
    ctrl4.G[ 2] =  da_wing_z_dphii(V)  ; 
    ctrl4.G[ 3] =  1.0;
    ctrl4.G[ 4] =  da_wing_x_dthetai(V)- fmaxf(filt.thrust,MIN_THRUST_EFFECTIVNESS); 
    ctrl4.G[ 5] =  da_wing_y_dthetai() ; 
    ctrl4.G[ 6] =  da_wing_z_dthetai(V); 
    ctrl4.G[ 7] =  dbdt;
    ctrl4.G[ 8] =  da_wing_x_dpsii(V)  ;  
    ctrl4.G[ 9] =  da_wing_y_dpsii(V)  ; 
    ctrl4.G[10] =  da_wing_z_dpsii(V)  ; 
    ctrl4.G[11] =  dbdp;
    ctrl4.G[12] =  0.0f;
    ctrl4.G[13] =  0.0f;
    ctrl4.G[14] = -1.0f;
    ctrl4.G[15] =  0.0f;
    break;
  }
  
}
void get_du(void){
  // in.thrust = 20*CMD_Thrust/9600
  // min = 0 
  // max = 20
  float du_min = THRUST_MIN*CMD_TO_THRUST - filt.thrust; 
  float du_max = THRUST_MAX*CMD_TO_THRUST - filt.thrust; 
  switch (varset.ce_mode)
  {
  case 1: ; 
    struct FloatMat33 Ga;
    struct FloatMat33 Ga_inv;
    
    for (int r = 0; r < ROWS_GUIDANCE; r++){
      for (int c = 0; c < COLS_GUIDANCE; c++){
        Ga.m[3*r+c] = ctrl3.G[r][c];
        Ga_inv.m[3*r+c] = 0.0f;
      }
    }    
    
    MAT33_INV(Ga_inv, Ga); 
    MAT33_VECT3_MUL(ctrl3.du, Ga_inv, ctrl3.dv);

    if (ctrl3.du.z < du_min){
      ctrl3.du.z = du_min;
    }else if (ctrl3.du.z > du_max){
      ctrl3.du.z = du_max;
    }

    ctrl.dphi   = ctrl3.du.x;
    ctrl.dtheta = ctrl3.du.y;
    ctrl.dpsi   = ctrl3.dyaw;
    ctrl.dT     = ctrl3.du.z;

    break;
  
  case 2: ;
    float G_inv[16];
    if (Mat44Inv(ctrl4.G, G_inv)){
        ctrl4.dv[0] = ctrl3.dv.x;
        ctrl4.dv[1] = ctrl3.dv.y;
        ctrl4.dv[2] = ctrl3.dv.z;
        ctrl4.dv[3] = 0.0;
        Mat44Vect4Mul(ctrl4.du,G_inv,ctrl4.dv);
    } else {
        ctrl4.du[0] = 0.0f;
        ctrl4.du[1] = 0.0f;
        ctrl4.du[2] = 0.0f;
        ctrl4.du[3] = 0.0f;
    }

    if (ctrl4.du[3] < du_min){
      ctrl4.du[3] = du_min;
    }else if (ctrl4.du[3] > du_max){
      ctrl4.du[3] = du_max;
    }
    ctrl.dphi   = ctrl4.du[0];
    ctrl.dtheta = ctrl4.du[1];
    ctrl.dpsi   = ctrl4.du[2];
    ctrl.dT     = ctrl4.du[3];

    break;

  default:
    break;
  }
}
void set_states(void){
  out.thrust = filt.thrust + ctrl.dT;
  out.thrust_cmd = (int) (out.thrust*THRUST_TO_CMD);
  if (out.thrust_cmd > THRUST_MAX){
      out.thrust_cmd = THRUST_MAX;
      out.thrust = THRUST_MAX*CMD_TO_THRUST;
  } 
  if (out.thrust_cmd < THRUST_MIN){
      out.thrust_cmd = THRUST_MIN;
      out.thrust = THRUST_MIN*CMD_TO_THRUST;
  }
    if (!is_killed){
      stabilization_cmd[COMMAND_THRUST] = out.thrust_cmd;
    } else {
      stabilization_cmd[COMMAND_THRUST] = out.thrust_cmd;
    }

    struct FloatQuat dq;
    struct FloatQuat q_res = {0};
    struct Int32Quat q_i_res = {0};


    float magdu;
    magdu = sqrt(ctrl.dphi*ctrl.dphi+ctrl.dtheta*ctrl.dtheta+ctrl.dpsi*ctrl.dpsi);
    if (magdu < 1e-10){
      dq.qi = 1.0;
      dq.qx = 0.0;
      dq.qy = 0.0;
      dq.qz = 0.0;
    }else{
      dq.qi = cos(magdu/2.0);
      dq.qx = sin(magdu/2.0)*ctrl.dphi/magdu;
      dq.qy = sin(magdu/2.0)*ctrl.dtheta/magdu;
      dq.qz = sin(magdu/2.0)*ctrl.dpsi/magdu;
    }
    float_quat_comp(&q_res, &filt.q, &dq);

    magdu = sqrt(q_res.qi*q_res.qi+q_res.qx*q_res.qx+q_res.qy*q_res.qy+q_res.qz*q_res.qz);

    q_res.qi = q_res.qi/magdu;
    q_res.qx = q_res.qx/magdu;
    q_res.qy = q_res.qy/magdu;
    q_res.qz = q_res.qz/magdu;

    update_butterworth_2_low_pass(&filt.qout_filter[0], q_res.qi);
    update_butterworth_2_low_pass(&filt.qout_filter[1], q_res.qx);
    update_butterworth_2_low_pass(&filt.qout_filter[2], q_res.qy);
    update_butterworth_2_low_pass(&filt.qout_filter[3], q_res.qz);

    float sum = 0.0;
    for (int i = 0; i < 4; i++){
      sum += powf(filt.qout_filter[i].o[0],2);
    }
    float length = sqrtf(sum);
    filt.qout.qi = filt.qout_filter[0].o[0]/length;
    filt.qout.qx = filt.qout_filter[1].o[0]/length;
    filt.qout.qy = filt.qout_filter[2].o[0]/length;
    filt.qout.qz = filt.qout_filter[3].o[0]/length;

    if (varset.use_filtered_output){
      q_res = filt.qout;
    }
    QUAT_BFP_OF_REAL(q_i_res, q_res);
    if (!is_killed){
      stabilization_indi_attitude_run(q_i_res,true);
    } else{
      q_res.qi = 1.0f;
      q_res.qx = 0.0f;
      q_res.qy = 0.0f;
      q_res.qz = 0.0f;
      QUAT_BFP_OF_REAL(q_i_res, q_res);
      stabilization_indi_attitude_run(q_i_res,true);
    }
}
// Filters
void filter_init(void){
    in.q   = *stateGetNedToBodyQuat_f();
    in.thrust_cmd = stabilization_cmd[COMMAND_THRUST];
    in.thrust = in.thrust_cmd*CMD_TO_THRUST;
    if (in.thrust_cmd > THRUST_MAX){
      in.thrust_cmd = THRUST_MAX;
      in.thrust = THRUST_MAX*CMD_TO_THRUST;
    } 
    if (in.thrust_cmd < THRUST_MIN){
        in.thrust_cmd = THRUST_MIN;
        in.thrust = THRUST_MIN*CMD_TO_THRUST;
    }

    float tau = 1.0 / (2.0 * M_PI * GUIDANCE_INDI_FILTER_CUTOFF);
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    for (int8_t i = 0; i < 3; i++) {
      init_butterworth_2_low_pass(&filt.acc_filter[i], tau, sample_time, 0.0);
      init_butterworth_2_low_pass(&filt.vel_filter[i], tau, sample_time, 0.0);
    }
    init_butterworth_2_low_pass(&filt.pos_filter[0], tau, sample_time, in.pos.x);
    init_butterworth_2_low_pass(&filt.pos_filter[1], tau, sample_time, in.pos.y);
    init_butterworth_2_low_pass(&filt.pos_filter[2], tau, sample_time, in.pos.z);   
    
    init_butterworth_2_low_pass(&filt.q_filter[0], tau, sample_time, in.q.qi);
    init_butterworth_2_low_pass(&filt.q_filter[1], tau, sample_time, in.q.qx);
    init_butterworth_2_low_pass(&filt.q_filter[2], tau, sample_time, in.q.qy);
    init_butterworth_2_low_pass(&filt.q_filter[3], tau, sample_time, in.q.qz);

    init_butterworth_2_low_pass(&filt.qout_filter[0], 1.0 / (2.0 * M_PI), sample_time, in.q.qi);
    init_butterworth_2_low_pass(&filt.qout_filter[1], 1.0 / (2.0 * M_PI), sample_time, in.q.qx);
    init_butterworth_2_low_pass(&filt.qout_filter[2], 1.0 / (2.0 * M_PI), sample_time, in.q.qy);
    init_butterworth_2_low_pass(&filt.qout_filter[3], 1.0 / (2.0 * M_PI), sample_time, in.q.qz);
    
    init_butterworth_2_low_pass(&filt.thrust_filter, tau, sample_time, in.thrust);


    init_butterworth_2_low_pass(&filt.V_filter, tau, sample_time, in.V);
    init_butterworth_2_low_pass(&filt.AoA_filter, tau, sample_time, in.AoA);
    init_butterworth_2_low_pass(&filt.ay_filter, tau, sample_time, in.ay);

    // float freq = 256;
    // float beta = 1.000000;
    // float mincutoff = 8.000000;
    // float dcutoff = 0.1;


    // f_acc_x.config.frequency = freq;
    // f_acc_x.config.cutoffSlope = beta;
    // f_acc_x.config.derivativeCutoffFrequency = dcutoff;
    // f_acc_x.config.minCutoffFrequency = mincutoff;    
    // SF1eFilterInit(&f_acc_x);
    // f_acc_y.config.frequency = freq;
    // f_acc_y.config.cutoffSlope = beta;
    // f_acc_y.config.derivativeCutoffFrequency = dcutoff;
    // f_acc_y.config.minCutoffFrequency = mincutoff;    
    // SF1eFilterInit(&f_acc_y);
    // f_acc_z.config.frequency = freq;
    // f_acc_z.config.cutoffSlope = beta;
    // f_acc_z.config.derivativeCutoffFrequency = dcutoff;
    // f_acc_z.config.minCutoffFrequency = mincutoff;    
    // SF1eFilterInit(&f_acc_z);
    // f2_acc_x.config.frequency = freq;
    // f2_acc_x.config.cutoffSlope = beta;
    // f2_acc_x.config.derivativeCutoffFrequency = dcutoff;
    // f2_acc_x.config.minCutoffFrequency = mincutoff;    
    // SF1eFilterInit(&f2_acc_x);
    // f2_acc_y.config.frequency = freq;
    // f2_acc_y.config.cutoffSlope = beta;
    // f2_acc_y.config.derivativeCutoffFrequency = dcutoff;
    // f2_acc_y.config.minCutoffFrequency = mincutoff;    
    // SF1eFilterInit(&f2_acc_y);
    // f2_acc_z.config.frequency = freq;
    // f2_acc_z.config.cutoffSlope = beta;
    // f2_acc_z.config.derivativeCutoffFrequency = dcutoff;
    // f2_acc_z.config.minCutoffFrequency = mincutoff;    
    // SF1eFilterInit(&f2_acc_z);

}

void filter_run(void){
  // same filter as for the acceleration
  update_butterworth_2_low_pass(&filt.thrust_filter, in.thrust);

  update_butterworth_2_low_pass(&filt.pos_filter[0], in.pos.x);
  update_butterworth_2_low_pass(&filt.pos_filter[1], in.pos.y);
  update_butterworth_2_low_pass(&filt.pos_filter[2], in.pos.z);

  update_butterworth_2_low_pass(&filt.vel_filter[0], in.vel.x);
  update_butterworth_2_low_pass(&filt.vel_filter[1], in.vel.y);
  update_butterworth_2_low_pass(&filt.vel_filter[2], in.vel.z);

  update_butterworth_2_low_pass(&filt.acc_filter[0], in.acc.x);
  update_butterworth_2_low_pass(&filt.acc_filter[1], in.acc.y);
  update_butterworth_2_low_pass(&filt.acc_filter[2], in.acc.z);

  update_butterworth_2_low_pass(&filt.q_filter[0], in.q.qi);
  update_butterworth_2_low_pass(&filt.q_filter[1], in.q.qx);
  update_butterworth_2_low_pass(&filt.q_filter[2], in.q.qy);
  update_butterworth_2_low_pass(&filt.q_filter[3], in.q.qz);


  update_butterworth_2_low_pass(&filt.AoA_filter, in.AoA);
  update_butterworth_2_low_pass(&filt.ay_filter, in.ay);
  update_butterworth_2_low_pass(&filt.V_filter, in.V);

  filt.thrust = filt.thrust_filter.o[0];

  filt.AoA = filt.AoA_filter.o[0];
  filt.ay = filt.ay_filter.o[0];
  filt.V = filt.V_filter.o[0];
  
  filt.pos.x = filt.pos_filter[0].o[0];
  filt.pos.y = filt.pos_filter[1].o[0];
  filt.pos.z = filt.pos_filter[2].o[0];

  filt.vel.x = filt.vel_filter[0].o[0];
  filt.vel.y = filt.vel_filter[1].o[0];
  filt.vel.z = filt.vel_filter[2].o[0];

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


  // float time = 1.0 / PERIODIC_FREQUENCY;
  // f_acc.x = SF1eFilterDoAtTime(&f_acc_x, in.acc.x, time);
  // f_acc.y = SF1eFilterDoAtTime(&f_acc_y, in.acc.y, time);
  // f_acc.z = SF1eFilterDoAtTime(&f_acc_z, in.acc.z, time);
  // f2_acc.x = SF1eFilterDoAtTime(&f2_acc_x, f_acc.x, time);
  // f2_acc.y = SF1eFilterDoAtTime(&f2_acc_y, f_acc.y, time);
  // f2_acc.z = SF1eFilterDoAtTime(&f2_acc_z, f_acc.z, time);

}
// Loggers
void message_log(void){
    aerolog.pos[0] = in.pos.x;
    aerolog.pos[1] = in.pos.y;
    aerolog.pos[2] = in.pos.z;
    aerolog.vel[0] = in.vel.x;
    aerolog.vel[1] = in.vel.y;
    aerolog.vel[2] = in.vel.z;
    aerolog.acc[0] = stateGetAccelNed_f()->x;
    aerolog.acc[1] = stateGetAccelNed_f()->y;
    aerolog.acc[2] = stateGetAccelNed_f()->z;

    aerolog.ref_pos[0] = ref.pos.x;
    aerolog.ref_pos[1] = ref.pos.y;
    aerolog.ref_pos[2] = ref.pos.z;
    aerolog.ref_vel[0] = ref.vel.x;
    aerolog.ref_vel[1] = ref.vel.y;
    aerolog.ref_vel[2] = ref.vel.z;
    aerolog.ref_acc[0] = ref.acc.x;
    aerolog.ref_acc[1] = ref.acc.y;
    aerolog.ref_acc[2] = ref.acc.z;

    aerolog.q[0] = filt.q.qi;
    aerolog.q[1] = filt.q.qx;
    aerolog.q[2] = filt.q.qy;
    aerolog.q[3] = filt.q.qz;

    aerolog.dyaw = ctrl.dpsi;

    aerolog.t = t;
    aerolog.mode = varset.mode;

    aerolog.max_vel = varset.max_vel;
    aerolog.max_acc = varset.max_acc;

    aerolog.du[0] = ctrl.dphi;
    aerolog.du[1] = ctrl.dtheta;
    aerolog.du[2] = ctrl.dT;
    aerolog.thrust_in = in.thrust;

    aerolog.gains[0] = varset.p_gain;
    aerolog.gains[1] = varset.d_gain;
}
void update_sd_logger(void){
  sd_id = sd_id + 2;

  sd_log[0]  = *(int32_t*)&in.pos.x;
  sd_log[1]  = *(int32_t*)&in.pos.y;
  sd_log[2]  = *(int32_t*)&in.pos.z;
  sd_log[3]  = *(int32_t*)&in.vel.x;
  sd_log[4]  = *(int32_t*)&in.vel.y;
  sd_log[5]  = *(int32_t*)&in.vel.z;
  sd_log[6]  = *(int32_t*)&in.acc.x;
  sd_log[7]  = *(int32_t*)&in.acc.y;
  sd_log[8]  = *(int32_t*)&in.acc.z;

  sd_log[9]  = *(int32_t*)&ref.pos.x;
  sd_log[10] = *(int32_t*)&ref.pos.y;
  sd_log[11] = *(int32_t*)&ref.pos.z;

  sd_log[12] = *(int32_t*)&in.q.qi;
  sd_log[13] = *(int32_t*)&in.q.qx;
  sd_log[14] = *(int32_t*)&in.q.qy;
  sd_log[15] = *(int32_t*)&in.q.qz;

  sd_log[16] = *(int32_t*)&ctrl.dphi;
  sd_log[17] = *(int32_t*)&ctrl.dtheta;
  sd_log[18] = *(int32_t*)&ctrl.dpsi;
  sd_log[19] = *(int32_t*)&ctrl.dT;
  sd_log[20] = *(int32_t*)&in.thrust;

  sd_log[21] = stabilization_cmd[COMMAND_ROLL];
  sd_log[22] = stabilization_cmd[COMMAND_PITCH];
  sd_log[23] = stabilization_cmd[COMMAND_YAW];
  sd_log[24] = stabilization_cmd[COMMAND_THRUST];

  sd_log[25] = varset.p_gain;
  sd_log[26] = varset.d_gain;
  // sd_log[27] = (int32_t) 100 * 420;
  // sd_log[28] = (int32_t) 10 * 420;
  // sd_log[29] = (int32_t) 420;

  // sd_log[25] = *(int32_t*)&f2_acc.x;
  // sd_log[26] = *(int32_t*)&f2_acc.y;
  // sd_log[27] = *(int32_t*)&f2_acc.z;
  // sd_log[28] = *(int32_t*)&LogFiltRate.p;
  // sd_log[29] = *(int32_t*)&LogFiltRate.q;
  sd_log[27] = *(int32_t*)&in.AoA;
  sd_log[28] = *(int32_t*)&in.ay;
  sd_log[29] = *(int32_t*)&in.V;
}
// Trajectory
void get_mode(void){
  bool switched_mode = false;
  if(active_mode != varset.mode){
    t = 0.0;
    switched_mode = true;
    active_mode = varset.mode;
  }
  //Precaution set all references to zero to prevent values from 'sticking'
  cur_pos = *stateGetPositionNed_f();
  
  ref.pos.x = cur_pos.x;
  ref.pos.y = cur_pos.y;
  ref.pos.z = ref_pos.z;
  ref.vel.x = 0.0;
  ref.vel.y = 0.0;
  ref.vel.z = 0.0;
  ref.acc.x = 0.0;
  ref.acc.y = 0.0;
  ref.acc.y = 0.0;
  //Precaution enable Kp and Kd
  mode_disabled_Kp = false;
  mode_disabled_Kd = false;
  mode_disabled_altitude_gains = !varset.use_alt_gains;
  
  

  switch (varset.mode)
  {
  // Stay at reference position
  case 0: ;
    ref.pos.x = ref_pos.x;
    ref.pos.y = ref_pos.y;
    break;
  // Fly in square
  case 1: ;
    ref.pos.x = ref_pos.x - 10.0;
    ref.pos.y = ref_pos.y + 10.0;
    break;
  // Fly in circle
  case 2: ;
    if (switched_mode){
      float x0 = cur_pos.x - 14.0;
      float y0 = cur_pos.y + 14.0;
      circ_pos.x = cur_pos.x - 14.0;
      circ_pos.y = cur_pos.y + 14.0;
      circ_pos.x = ref_pos.z;
      mode_2_angle = atan2f(14.0,-14.0);
      mode_2_radius = sqrtf(powf(14.0,2.0) + powf(14.0,2.0));
    }
    ref.pos.x =  mode_2_radius * sinf(mode_2_angle) + circ_pos.x;
    ref.pos.y =  mode_2_radius * cosf(mode_2_angle) + circ_pos.y;
    ref.vel.x =  mode_2_radius * cosf(mode_2_angle) * varset.max_vel/(2.0*M_PI*mode_2_radius);
    ref.vel.y = -mode_2_radius * sinf(mode_2_angle) * varset.max_vel/(2.0*M_PI*mode_2_radius);
    if (varset.use_feedforward){
      ref.acc.x = -mode_2_radius * sinf(mode_2_angle) * powf(varset.max_vel/(2.0*M_PI*mode_2_radius),2.0);
      ref.acc.y = -mode_2_radius * cosf(mode_2_angle) * powf(varset.max_vel/(2.0*M_PI*mode_2_radius),2.0);
    }
    mode_2_angle += varset.max_vel/(2.0*M_PI*mode_2_radius*512.0);
    break;
  // Acceleration tracking test
  case 3: ;
    ref.pos.x = ref_pos.x - 30.0;
    ref.pos.y = ref_pos.y - 30.0;
    break;
  // Acceleration tracking test
  case 4: ;
    ref.pos.x = cur_pos.x;
    ref.pos.y = cur_pos.y;
    ref.pos.z = ref_pos.z;
    break;
  // Default stay at reference position
  default: ;
    ref.pos.x = ref_pos.x;
    ref.pos.y = ref_pos.y;
    break;
  }
}

void UpdateTrajectory(void){
    traj.t = t;

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

void CalcNewTrajectory(void){
    
    // traj_sec.pos_beg.x = traj.pos.x;
    // traj_sec.pos_beg.y = traj.pos.y;
    // traj_sec.pos_beg.z = traj.pos.z;

    traj_sec.pos_beg.x = traj_sec.pos_end.x;
    traj_sec.pos_beg.y = traj_sec.pos_end.y;
    traj_sec.pos_beg.z = traj_sec.pos_end.z;
    
    TrajectoryScenario(); //get pos_end, v_max, a_max based on scenario
    
    traj_sec.delta_pos = sqrtf(pow(traj_sec.pos_end.x-traj_sec.pos_beg.x,2) +  pow(traj_sec.pos_end.y-traj_sec.pos_beg.y,2) +  pow(traj_sec.pos_end.z-traj_sec.pos_beg.z,2));
    traj_sec.dir.x = (traj_sec.pos_end.x-traj_sec.pos_beg.x)/traj_sec.delta_pos;
    traj_sec.dir.y = (traj_sec.pos_end.y-traj_sec.pos_beg.y)/traj_sec.delta_pos;
    traj_sec.dir.z = (traj_sec.pos_end.z-traj_sec.pos_beg.z)/traj_sec.delta_pos;

    float time_v_constraint = 2.0*traj_sec.delta_pos/traj_sec.vel_max;
    float time_a_constraint = sqrtf(2*M_PI*traj_sec.delta_pos/traj_sec.acc_max);

    if(time_a_constraint>time_v_constraint){
      traj_sec.type = 0;
      traj_sec.t_end = time_a_constraint;
    } else {
      traj_sec.type = 1;
      traj_sec.t_trans_beg =  M_PI * traj_sec.vel_max/traj_sec.acc_max/2.0;
      traj_sec.d_x_trans = traj_sec.vel_max*traj_sec.t_trans_beg/2;
      traj_sec.t_end = (traj_sec.delta_pos-2*traj_sec.d_x_trans)/traj_sec.vel_max + 2* traj_sec.t_trans_beg;
      traj_sec.t_trans_end =  traj_sec.t_end-traj_sec.t_trans_beg;
    }



}
void TrajectoryScenario(void){
    traj_sec.number += 1;
    if (traj_sec.number >= 4){
      traj_sec.number = 0;
    } 

    traj_sec.pos_end.x =  y_coords[traj_sec.number];
    traj_sec.pos_end.y =  x_coords[traj_sec.number];

    traj_sec.vel_max = varset.max_vel; 
    traj_sec.acc_max = varset.max_acc;


}
// FlightPlan
void setReferenceHere(uint8_t wp){
    struct NedCoor_f fix_pos;
    fix_pos.x = waypoint_get_x(wp);
    fix_pos.y = waypoint_get_y(wp);
    Home.x = waypoint_get_x(wp);
    Home.y = waypoint_get_y(wp);
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

}
void getWaypoints(uint8_t wp_1, uint8_t wp_2, uint8_t wp_3, uint8_t wp_4){
    wps.stay_wp = wp_1;
    wps.circle = wp_2;
    wps.start = wp_3;
    wps.dir = wp_4;

    x_coords[0] = waypoint_get_x(wp_1);
    x_coords[1] = waypoint_get_x(wp_2);
    x_coords[2] = waypoint_get_x(wp_3);
    x_coords[3] = waypoint_get_x(wp_4);

    y_coords[0] = waypoint_get_y(wp_1);
    y_coords[1] = waypoint_get_y(wp_2);
    y_coords[2] = waypoint_get_y(wp_3);
    y_coords[3] = waypoint_get_y(wp_4);
}
void updateWaypoints(void){
  switch (varset.mode)
  {
  case 0: ; // Hover
    float x0 = waypoint_get_x(wps.start);
    float y0 = waypoint_get_y(wps.start);
    float x1 = waypoint_get_x(wps.dir);
    float y1 = waypoint_get_y(wps.dir);

    float head = atan2f(y1-y0,x1-x0);
    waypoint_set_here_2d(wps.start);

    waypoints[wps.dir].enu_f.x = x0 + sinf(head) * 150.0;
    waypoints[wps.dir].enu_f.y = y0 + cosf(head) * 150.0;
    waypoint_globalize(wps.dir);
    break;
  
  case 1: ; // Waypoint
    break;

  case 2: ; // Circle
    break;

  case 3: ; // Transition straight, pause, back.
    break;

  case 4: ; // Trajectory extreme
    break;

  default:
    break;
  }
}
void setModeKill(void){
  is_killed = true;
}
bool Mat44Inv(const float m[16], float invOut[16])
{
    float inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
}
void Mat44Vect4Mul(float du[4], const float G_inv[16], const float dv[4]){
  du[0] = G_inv[0]  * dv[0] + G_inv[1]  * dv[1] + G_inv[2]  * dv[2] + G_inv[3]  * dv[3];
  du[1] = G_inv[4]  * dv[0] + G_inv[5]  * dv[1] + G_inv[6]  * dv[2] + G_inv[7]  * dv[3];
  du[2] = G_inv[8]  * dv[0] + G_inv[9]  * dv[1] + G_inv[10] * dv[2] + G_inv[11] * dv[3];
  du[3] = G_inv[12] * dv[0] + G_inv[13] * dv[1] + G_inv[14] * dv[2] + G_inv[15] * dv[3];
}