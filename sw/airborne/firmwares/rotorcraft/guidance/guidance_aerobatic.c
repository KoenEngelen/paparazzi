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
  #include "subsystems/datalink/telemetry.h"
  #include "firmwares/rotorcraft/stabilization.h"
  #include "filters/low_pass_filter.h"
  #include "subsystems/abi.h"
  // #include 

  #define PPRZ_MSG_ID_AEROBATIC_INDI 158
//

//Declare variables
  float guidance_aerobatic_indi_gain = 0.5;
  float debug_value;

  struct ctrl_lin_struct {
    // Input Position
    struct NedCoor_f *pos; //get from stat.h: stateGetPositionNed_f(void)
    // Input Velocity
    struct NedCoor_f *vel; //get from stat.h: stateGetSpeedNed_f(void)
    // Input Acceleration
    struct NedCoor_f *acc; //get from stat.h: stateGetAccelNed_f(void)
    // Output Acceleration
    struct FloatVect3 cmdacc;
  } ctrl_lin;

  struct trajectory {
    float t;
    struct NedCoor_f pos;
    struct NedCoor_f vel;
    struct NedCoor_f acc;
  } traj;

  struct FloatVect3 V_inf;
  struct FloatVect3 V_drone;
  struct FloatQuat q;

  float thrust_in = 0;

  float V[3]={0.1f,2.0f,1.0f};
  float phi = 0;
  float alpha = 0;

  float t = 0.0;

  #define ROWS_GUIDANCE 3 //rows
  #define COLS_GUIDANCE 4 //columns

  float Weight[COLS_GUIDANCE] = {1, 1 , 1 , 1};

  float G[ROWS_GUIDANCE][COLS_GUIDANCE] = {0};

  struct FloatVect3 dv;
  float du[COLS_GUIDANCE] = {0};

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
  Butterworth2LowPass filt_quat[4];
  Butterworth2LowPass roll_filt;
  Butterworth2LowPass pitch_filt;
  Butterworth2LowPass thrust_filt;

  float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;


//

//Declare all functions
  extern void guidance_h_module_init(void);
  extern void guidance_h_module_enter(void);
  extern void guidance_h_module_read_rc(void);
  extern void guidance_h_module_run(bool in_flight);
  extern void guidance_v_module_init(void);
  extern void guidance_v_module_enter(void);
  extern void guidance_v_module_run(bool in_flight);

  // void guidance_h_module_init(void);
  // void guidance_h_module_enter(void);
  // void guidance_h_module_read_rc(void);
  // void guidance_h_module_run(bool in_flight);



  void guidance_aerobatic_init(void);
  void guidance_aerobatic_enter(void);
  void guidance_aerobatic_read_rc(void);
  void guidance_aerobatic_run(void);


  void filter_init(void);
  static void guidance_indi_filter_thrust(void);
  void guidance_indi_propagate_filters(struct FloatEulers *eulers);

  void test_SVD(void);
  void add_increment(void);
  void test_a_L_wing(void);
  float a_D_wing_x(void);
  float a_D_wing_z(void);
  float a_L_wing_x(void);
  float a_L_wing_z(void);
  float a_wing_x(void);
  float a_wing_z(void);
  float AoA(void);
  float AoD(void);
  float AoS(void);
  float Cd(void);
  float Cl(void);
  float dCd(void);
  float dCl(void);
  float Vxz_2(void);
  float da_wing_x_dphii(void);
  float da_wing_x_dpsii(void);
  float da_wing_x_dthetai(void);
  float da_wing_y_dphii(void);
  float da_wing_y_dpsii(void);
  float da_wing_y_dthetai(void);
  float da_wing_z_dphii(void);
  float da_wing_z_dpsii(void);
  float da_wing_z_dthetai(void);
  float dalphai_dphii(void);
  float dalphai_dpsii(void);
  float dalphai_dthetai(void);
  float dbetai_dphii(void);
  float dbetai_dpsii(void);
  float dbetai_dthetai(void);
  float dCd_dphii(void);
  float dCd_dpsii(void);
  float dCd_dthetai(void);
  float dCl_dphii(void);
  float dCl_dpsii(void);
  float dCl_dthetai(void);
  float dVx_dphii(void);
  float dVx_dpsii(void);
  float dVx_dthetai(void);
  float dVxz_2_dphii(void);
  float dVxz_2_dpsii(void);
  float dVxz_2_dthetai(void);
  float dVy_dphii(void);
  float dVy_dthetai(void);
  float dVz_dphii(void);
  float dVz_dpsii(void);
  float dVz_dthetai(void);
  float Vxz_2_dalphai_dphii(void);
  float Vxz_2_dalphai_dpsii(void);
  float Vxz_2_dCd_dphii(void);
  float Vxz_2_dCd_dpsii(void);
  float Vxz_2_dCl_dphii(void);
  float Vxz_2_dCl_dpsii(void);

  void print_matrix(float **ptr,int m, int n);
  void print_matrix_matlab(float **ptr,int m, int n);
  // void print_eq(float **ptrA, float **ptrX, float **ptrB, int m, int n, int l);
  void print_2darr(int row, int col, float* Data_00);
  void print_row(float *ptr,int m);
  void print_row_matlab(float *ptr,int m);

  void UpdateTrajectory(float t);
  void LinAccCon(void);
  void get_increment(void);
  void SVDw(void);
  void get_dv(void);
  void get_G(void);
  // void print_matrix(float** ptr,int m, int n);
  // void print_row(float* ptr,int m);
//

//Guidance Module
  void guidance_h_module_init(void)
  {
    guidance_aerobatic_init();
  }

  void guidance_h_module_enter(void)
  {
    guidance_aerobatic_enter();
  }

  void guidance_h_module_read_rc(void)
  {
    // // -MAX_PPRZ to MAX_PPRZ
    // ctrl_windtunnel.rc_throttle = radio_control.values[RADIO_THROTTLE];
    // ctrl_windtunnel.rc_roll = radio_control.values[RADIO_ROLL];
    // ctrl_windtunnel.rc_pitch = radio_control.values[RADIO_PITCH];
    // ctrl_windtunnel.rc_yaw = radio_control.values[RADIO_YAW];
  }

  void guidance_h_module_run(UNUSED bool in_flight)
  {
    // Call full inner-/outerloop / horizontal-/vertical controller:
    guidance_aerobatic_run();
  }

  void guidance_v_module_init(void)
  {
    // initialization of your custom vertical controller goes here
  }

  // Implement own Vertical loops
  void guidance_v_module_enter(void)
  {
    // your code that should be executed when entering this vertical mode goes here
  }

  void guidance_v_module_run(UNUSED bool in_flight)
  {
    // your vertical controller goes here
  }
//

//Messages
 static void send_aerobatic_indi(struct transport_tx *trans, struct link_device *dev){
   pprz_msg_send_AEROBATIC_INDI(trans, dev, AC_ID,
                         &debug_value, 
                         &guidance_aerobatic_indi_gain);
 }
//

// Initialize function
 void guidance_aerobatic_init(void) {
  debug_value = 0.0;

  get_dv();
  get_G();

  t = 0.0;
  filter_init();
  // printf("thrust filt: %f", thrust_filt.o[0]);
  
  // test_SVD(); //This function works in init
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_INDI, send_aerobatic_indi);
 }
//

// Enter function
  void guidance_aerobatic_enter(void) {
    t = -5.0;
    filter_init();
    thrust_in = stabilization_cmd[COMMAND_THRUST];
    thrust_act = thrust_in;
    printf("pos_est_x\tpos_est_y\tpos_est_z\tpos_ref_x\tpos_ref_y\tpos_ref_z\tvel_est_x\tvel_est_y\tvel_est_z\tvel_ref_x\tvel_ref_y\tvel_ref_z\tacc_est_x\tacc_est_y\tacc_est_z\tacc_ref_x\tacc_ref_y\tacc_ref_z\t");

    printf("qi\tqx\tqy\tqz\t");
    printf("V_b_x\tV_b_y\tV_b_z\t");
    printf("G00\tG01\tG02\tG03\tG10\tG11\tG12\tG13\tG20\tG21\tG22\tG23\tdv_x\tdv_y\tdv_z\tdu_phi\tdu_theta\tdu_psi\tdu_T\n");
  }
//

// Periodic function
 void guidance_aerobatic_run() {
  // printf("\naerobatic running\n");
  t = t + 0.001953125;
  if (t > 360000){
    t = 0;
  }

  UpdateTrajectory(t);
  LinAccCon();
  // // printf("state_accel: %f" , stateGetSpeedNed_f()->x);

  q.qi = stateGetNedToBodyQuat_f()->qi;
  q.qx = -stateGetNedToBodyQuat_f()->qx;
  q.qy = -stateGetNedToBodyQuat_f()->qy;
  q.qz = -stateGetNedToBodyQuat_f()->qz;

  q.qi = filt_quat[0].o[0];
  q.qx = filt_quat[1].o[0];
  q.qy = filt_quat[2].o[0];
  q.qz = filt_quat[3].o[0];

  struct FloatEulers eulers_yxz;
  struct FloatQuat  * statequat = stateGetNedToBodyQuat_f();
  // struct FloatQuat * statequat = &q;
  // statequat.qi = q.qi;
  // statequat.qx = q.qx;
  // statequat.qy = q.qy;
  // statequat.qz = q.qz;
  // printf("\n\n%f\t%f\t%f\t%f\n\n",statequat->qi,statequat->qx,statequat->qy,statequat->qz);
  // printf("\n\n%f\t%f\t%f\t%f\n\n",q.qi,q.qx,q.qy,q.qz);
  float_eulers_of_quat_yxz(&eulers_yxz, &q);

  

  guidance_indi_propagate_filters(&eulers_yxz);
  guidance_indi_filter_thrust();
  // printf("\n\nthrust filt: %f", thrust_filt.o[0]);
  // printf("\n\tfilt.a: \t%f\t%f", thrust_filt.a[0], thrust_filt.a[1]);
  // printf("\n\tfilt.b: \t%f\t%f", thrust_filt.b[0], thrust_filt.b[1]);
  // printf("\n\tfilt.i: \t%f\t%f", thrust_filt.i[0], thrust_filt.i[1]);
  // printf("\n\tfilt.o: \t%f\t%f\n\n", thrust_filt.o[0], thrust_filt.o[1]);
  V_drone.x = stateGetSpeedNed_f()->x;
  V_drone.y = stateGetSpeedNed_f()->y;
  V_drone.z = stateGetSpeedNed_f()->z;

  q.qi = -q.qi;
  float_quat_vmult(&V_inf, &q, &V_drone);
  q.qi = -q.qi;

  V[0] = - V_inf.x;
  V[1] = - V_inf.y;
  V[2] = - V_inf.z;   

  // printf("\n\nV: %f \t %f \t %f\n",V[0],V[1],V[2]);   

  SVDw(); 
  //  stabilization_cmd[COMMAND_THRUST] = 0;
 }
//
 
// Print functions
  // // Print_matix_function
  // void print_matrix(float **ptr,int m, int n){
  //     for (int r = 0; r < m; r++) {
  //       for (int c = 0; c < n; c++) {
  //         // printf("\t%e\t",ptr[r][c]);
  //       }
  //       // printf("\n");
  //     }
  //     // printf("\n");
  // }

  // // Print_matix_function
  // void print_matrix_matlab(float **ptr,int m, int n){
  //     // printf("\n[");
  //     for (int r = 0; r < m; r++) {
  //       for (int c = 0; c < n; c++) {
  //         // printf("\t%e",ptr[r][c]);
  //         if (c<(n-1)){
  //           // printf(",");
  //         }
  //       }
  //       if (r<(m-1)){
  //         // printf(";\n");
  //       }else{
  //         // printf("];\n\n");
  //       }
  //     }
  // }


  // // Print_equation_function
  //   // void print_eq(float **ptrA, float **ptrX, float **ptrB, int m, int n, int l){
  //   //     for (int r = 0; r < m; r++) {
          
  //   //       // printf("|");
  //   //       for (int c = 0; c < n; c++) {
  //   //         // printf("\t%e\t",ptrA[r][c]);
  //   //       }
  //   //       // printf("|");

  //   //       if(r == 1){
  //   //         // printf("\t*\t");
  //   //       }else{
  //   //         // printf("\t\t");
  //   //       }

  //   //       // printf("|");
  //   //       // printf("\t%e\t",ptrX[r][0]);
  //   //       // printf("|");

  //   //       if(r == 1){
  //   //         // printf("\t=\t");
  //   //       }else{
  //   //         // printf("\t\t");
  //   //       }
          
  //   //       // printf("|");
  //   //       // printf("\t%e\t",ptrB[r][0]);
  //   //       // printf("|");

  //   //       // printf("\n");
  //   //     }
  //   //     // printf("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t|");
  //   //     // printf("\t%e\t",ptrX[3][0]);
  //   //     // printf("|");


  //   //     // printf("\n");
  //   // }
  // //


  // void print_2darr(int row, int col, float* Data_00){

  //   // printf("[");
  //   // print 2D array
  //   for (int i = 0; i < row; i++){
  //       for (int j = 0; j < col; j++) {
  //           float* MatData_val = Data_00 + (i * col + j);
  //           if (j==col-1){
  //             // printf("\t%e", *MatData_val);
  //           }else{      
  //             // printf("\t%e,", *MatData_val);
  //           }

  //       }
  //       if (i==row-1){
  //         // printf("]\n");
  //       }else{      
  //         // printf(";\n");
  //       }
        
  //   }
  //   // printf("\n");
  // }

  // void print_row(float *ptr,int m){
  //     for (int r = 0; r < m; r++) {
  //         // printf("\t%e\t",ptr[r]);
  //     }
  //     // printf("\n\n");
  // }

  // void print_row_matlab(float *ptr,int m){
  //     // printf("\n[");
  //     for (int r = 0; r < m; r++) {
  //         // printf("\t%e",ptr[r]);
  //         if(r!=m-1){
  //           // printf(",");
  //         }
  //     }
  //     // printf("];\n\n");
  // }
//

// Controller functions
  void UpdateTrajectory(float t){
    traj.t = t;


    traj.pos.x = 00.;
    traj.pos.y = 00.;
    traj.pos.z = -50.;

    traj.vel.x = 0.;
    traj.vel.y = 0.;
    traj.vel.z = 0.;

    traj.acc.x = 0.;
    traj.acc.y = 0.;
    traj.acc.z = 0.;

    if (t <= 20 && t >= 0){    
      traj.pos.y = 7.5 * t - 47.7465 * sinf(0.15708*t);
      traj.vel.y = 15.0 * (0.5*sinf(t/20.*M_PI - M_PI/2.) + 0.5);
      traj.acc.y = 1.1781 * sinf((M_PI*t)/20.);
    } else if (t > 20){
      traj.pos.y = 7.5 * t;
      traj.vel.y = 15. ;
      traj.acc.y = 0.0 ;
    }
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
    float PGain = -1.0f;
    float VGain = -1.0f;

    ctrl_lin.pos = stateGetPositionNed_f();
    ctrl_lin.vel = stateGetSpeedNed_f();
    ctrl_lin.acc = stateGetAccelNed_f();

    // printf("\nPos: %f \t %f \t %f",ctrl_lin.pos->x,ctrl_lin.pos->y,ctrl_lin.pos->z);
    // printf("\nVel: %f \t %f \t %f",ctrl_lin.vel->x,ctrl_lin.vel->y,ctrl_lin.vel->z);
    // printf("\nAcc: %f \t %f \t %f",ctrl_lin.acc->x,ctrl_lin.acc->y,ctrl_lin.acc->z);

    ctrl_lin.cmdacc.x = PGain * (ctrl_lin.pos->x-traj.pos.x) + VGain * (ctrl_lin.vel->x-traj.vel.x) + traj.acc.x;
    ctrl_lin.cmdacc.y = PGain * (ctrl_lin.pos->y-traj.pos.y) + VGain * (ctrl_lin.vel->y-traj.vel.y) + traj.acc.y;
    ctrl_lin.cmdacc.z = PGain * (ctrl_lin.pos->z-traj.pos.z) + VGain * (ctrl_lin.vel->z-traj.vel.z) + traj.acc.z;


    float_quat_vmult(&dv, &q, &ctrl_lin.cmdacc);
    dv.x = -dv.x ;
    dv.y = -dv.y ;
    dv.z = -dv.z ;


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

    // printf("\n\nCommanded Acceleration: %f \t %f \t %f \t [m/s^2]\n", ctrl_lin.cmdacc.x, ctrl_lin.cmdacc.y, ctrl_lin.cmdacc.z);
  }

  void get_increment(void){
    float WGt[COLS_GUIDANCE][ROWS_GUIDANCE] = {0};
    float GWGt[ROWS_GUIDANCE][ROWS_GUIDANCE] = {0}; 
    float WiUv[ROWS_GUIDANCE] = {0};
    float VWiUv[ROWS_GUIDANCE] = {0};

    float sum = 0;


    // tol used for zero division avoidance
    int max = (ROWS_GUIDANCE < COLS_GUIDANCE) ? COLS_GUIDANCE : ROWS_GUIDANCE;
    float tol = max * FLT_EPSILON;

    // get: WGt = Weight * G_transposed
    int i,j;
    for (i=0; i<COLS_GUIDANCE; i++){
      for (j=0; j<ROWS_GUIDANCE; j++){
        WGt[i][j] = Weight[i] * G[j][i];
      }
    }

    // get: GWGt = G * Weight * G_transposed
    sum = 0;
    for (int i = 0; i < ROWS_GUIDANCE; i++){
      for (int j = 0; j < ROWS_GUIDANCE; j++){
        for (int k = 0; k < COLS_GUIDANCE; k++){
          sum =  sum + G[i][k] * WGt[k][j];
        }
        GWGt[i][j] = sum;
        sum = 0;
      }
    }

    // get pseudoinverse of G * Weight * G_transposed = GWGt
    float _svd_W[ROWS_GUIDANCE] = {0};
    float _svd_V[ROWS_GUIDANCE][ROWS_GUIDANCE] = {0};
    MAKE_MATRIX_PTR(svd_U, GWGt, ROWS_GUIDANCE);
    MAKE_MATRIX_PTR(svd_V, _svd_V, ROWS_GUIDANCE);
    pprz_svd_float(svd_U,_svd_W, svd_V, ROWS_GUIDANCE, ROWS_GUIDANCE); 

    // get: WiUv =  W_inverse * U * dv
    sum = 0;
    float tmp_dv;
    for (int i = 0; i < ROWS_GUIDANCE; i++){
      for (int j = 0; j < ROWS_GUIDANCE; j++){
        if (j == 0){
            tmp_dv = dv.x;
        } else if (j == 1){
            tmp_dv = dv.y;
        } else {
            tmp_dv = dv.z;
        }
        sum = sum + svd_U[j][i]  * tmp_dv;  // <-- trasnpose of U
      }

      if (_svd_W[i] > tol){
        WiUv[i] = sum / _svd_W[i];
      }else{
        WiUv[i] = 0;
      }
      sum = 0;
    }


    // get: VWiUv = V * (W_inverse * U * dv) = V * WiUv
    sum = 0;
    for (int i = 0; i < ROWS_GUIDANCE; i++){
      for (int j = 0; j < ROWS_GUIDANCE; j++){
        sum = sum + svd_V[i][j] * WiUv[j];
      }
      VWiUv[i] = sum;
      sum = 0;
    }
    // get: du = (Weight * G_transposed) * (V * (W_inverse * U * dv)) = WGt * VWiUv
    sum = 0;
    for (int i = 0; i < COLS_GUIDANCE; i++){
      for (int j = 0; j < ROWS_GUIDANCE; j++){
        sum = sum + WGt[i][j] * VWiUv[j];
      }
      du[i] = sum;
      sum = 0;
    } 

    du[3] = du[3] * 480.;

    float du_min =  960.-thrust_in; // min = 0    (defined by thrust_in = CMD_Thrust / 9600 * 20)
    float du_max = 8640.-thrust_in; // max = 20
    // printf("\ndu_min:\t%e\n",du_min);
    // printf("\ndu_max:\t%e\n",du_max);
    // printf("\ndu[3]:\t%e\n",du[3]);
    
    if (du[3] < du_min){
      du[3] = du_min;
    }else if (du[3] > du_max){
      du[3] = du_max;
    }


  }

  void get_G(void){
    if (ROWS_GUIDANCE > COLS_GUIDANCE){
      G[0][0] =  da_wing_x_dphii(); 
      G[0][1] =  da_wing_y_dphii() + 20.0 * thrust_in/MAX_PPRZ; 
      G[0][2] =  da_wing_z_dphii(); 
      G[1][0] =  da_wing_x_dthetai() - 20.0 * thrust_in/MAX_PPRZ; 
      G[1][1] =  da_wing_y_dthetai(); 
      G[1][2] =  da_wing_z_dthetai(); 
      G[2][0] =  da_wing_x_dpsii(); 
      G[2][1] =  da_wing_y_dpsii(); 
      G[2][2] =  da_wing_z_dpsii(); 
      G[3][0] =  0.0f; 
      G[3][1] =  0.0f; 
      G[3][2] =  -20.0* thrust_in/MAX_PPRZ; 
    } else {
      G[0][0] =  da_wing_x_dphii(); 
      G[1][0] =  da_wing_y_dphii()+ 20.0 * thrust_in/MAX_PPRZ; 
      G[2][0] =  da_wing_z_dphii(); 
      G[0][1] =  da_wing_x_dthetai() - 20.0* thrust_in/MAX_PPRZ; 
      G[1][1] =  da_wing_y_dthetai(); 
      G[2][1] =  da_wing_z_dthetai(); 
      G[0][2] =  da_wing_x_dpsii(); 
      G[1][2] =  da_wing_y_dpsii(); 
      G[2][2] =  da_wing_z_dpsii(); 
      G[0][3] =  0.0f; 
      G[1][3] =  0.0f; 
      G[2][3] =  -20.0* thrust_in/MAX_PPRZ; 
    }
  }

  void get_dv(void){
    // dv.x =  0.0;
    // dv.y =  0.0;
    // dv.z = -1;
    LinAccCon();

  }

  void add_increment(void){
    AbiSendMsgTHRUST(THRUST_INCREMENT_ID, du[3]);
    // printf("thrust filt: %f", thrust_filt.o[0]);
    thrust_in = thrust_filt.o[0] + du[3];
    if (thrust_in > 9600){
      thrust_in = 9600;
    } 
    if (thrust_in < 0 ){
      thrust_in = 0;
    }
    stabilization_cmd[COMMAND_THRUST] = thrust_in;

    struct FloatEulers rpy;
    struct Int32Eulers rpyi;
    struct FloatQuat dq;
    struct FloatQuat q_res = {0};
    struct Int32Quat q_i_res = {0};

    float magdu;
    magdu = sqrt(du[0]*du[0]+du[1]*du[1]+du[2]*du[2]);

    dq.qi = cos(magdu);
    dq.qx = sin(magdu)*du[0]/magdu;
    dq.qy = sin(magdu)*du[1]/magdu;
    dq.qz = sin(magdu)*du[2]/magdu;
    float_quat_comp(&q_res, &q, &dq);
    // printf("\n\nQ:  \t%f\t%f\t%f\t%f" , q.qi      , q.qx      , q.qy      , q.qz    );
    // printf("\ndQ:  \t%f\t%f\t%f\t%f", dq.qi     , dq.qx     , dq.qy     , dq.qz   );
    // printf("\nQres:\t%f\t%f\t%f\t%f\n", q_res.qi  , q_res.qx  , q_res.qy  , q_res.qz);




    float_eulers_of_quat(&rpy, &q_res);
    // guidance_euler_cmd.theta
    // guidance_euler_cmd.phi
    // guidance_euler_cmd.psi
    // printf("\nrpy:\t%f\t%f\t%f", rpy.phi  , rpy.theta  , rpy.psi);

    rpyi.phi = (int)rpy.phi;
    rpyi.theta = (int)rpy.theta;
    rpyi.psi = (int)rpy.psi;

    // printf("\nrpyi:\t%d\t%d\t%d\n", rpyi.phi  , rpyi.theta  , rpyi.psi);

    QUAT_BFP_OF_REAL(q_i_res, q_res);
    // QUAT_BFP_OF_REAL(stab_att_sp_quat, q_res);
    // stabilization_attitude_set_rpy_setpoint_i(&rpyi);
    stabilization_indi_attitude_run(q_i_res,true);

  }

  void SVDw(void){
    get_dv();
    get_G();
    get_increment();
    // printf("\ndv:\t%f\t%f\t%f",dv.x,dv.y,dv.z);
    // printf("\ndu:\t%f\t%f\t%f\t%f",du[0],du[1],du[2],du[3]);
    // printf("\nT: %f", thrust_in);
    // printf("\nT:\t%e\n",thrust_in);
    add_increment();





    printf("%f\t%f\t%f", ctrl_lin.pos->x, ctrl_lin.pos->y, ctrl_lin.pos->z);
    printf("\t%f\t%f\t%f", traj.pos.x, traj.pos.y, traj.pos.z);
    printf("\t%f\t%f\t%f", ctrl_lin.vel->x, ctrl_lin.vel->y, ctrl_lin.vel->z);
    printf("\t%f\t%f\t%f", traj.vel.x, traj.vel.y, traj.vel.z);
    printf("\t%f\t%f\t%f", ctrl_lin.acc->x, ctrl_lin.acc->y, ctrl_lin.acc->z);
    printf("\t%f\t%f\t%f", traj.acc.x, traj.acc.y, traj.acc.z);
    // printf("\npos_est [  m  ]: \t%f\t%f\t%f", ctrl_lin.pos->x, ctrl_lin.pos->y, ctrl_lin.pos->z);
    // printf("\npos_ref [  m  ]: \t%f\t%f\t%f", traj.pos.x, traj.pos.y, traj.pos.z);
    // printf("\nvel_est [ m/s ]: \t%f\t%f\t%f", ctrl_lin.vel->x, ctrl_lin.vel->y, ctrl_lin.vel->z);
    // printf("\nvel_ref [ m/s ]: \t%f\t%f\t%f", traj.vel.x, traj.vel.y, traj.vel.z);
    // printf("\nacc_est [m/s^2]: \t%f\t%f\t%f", ctrl_lin.acc->x, ctrl_lin.acc->y, ctrl_lin.acc->z);
    // printf("\nacc_ref [m/s^2]: \t%f\t%f\t%f", traj.acc.x, traj.acc.y, traj.acc.z);


    printf("\t%f\t%f\t%f\t%f",q.qi,q.qx,q.qy,q.qz);
    printf("\t%f\t%f\t%f",V[0],V[1],V[2]);

    printf("\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f",
    G[0][0],G[0][1],G[0][2],G[0][3],
    G[1][0],G[1][1],G[1][2],G[1][3],
    G[2][0],G[2][1],G[2][2],G[2][3]);
    printf("\t%f\t%f\t%f", dv.x, dv.y, dv.z);
    printf("\t%f\t%f\t%f\t%f\n", du[0], du[1], du[2], du[3]);
    // printf("\nG: \t%f\t%f\t%f\t%f\n \t%f\t%f\t%f\t%f\n \t%f\t%f\t%f\t%f",
    // G[0][0],G[0][1],G[0][2],G[0][3],
    // G[1][0],G[1][1],G[1][2],G[1][3],
    // G[2][0],G[2][1],G[2][2],G[2][3]);
    // printf("\ndv [m/s^2]: \t%f\t%f\t%f", dv.x, dv.y, dv.z);
    // printf("\ndu [ 3xrad | N ]: \t%f\t%f\t%f\t%f", du[0], du[1], du[2], du[3]);
  }
//

// Filters

  
  /**
  * Filter the thrust, such that it corresponds to the filtered acceleration
  */
  void filter_init(void){
    thrust_in = stabilization_cmd[COMMAND_THRUST];
    thrust_act = thrust_in;

    float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
    float sample_time = 1.0 / PERIODIC_FREQUENCY;
    for (int8_t i = 0; i < 3; i++) {
      init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
    }
    for (int8_t i = 0; i < 4; i++) {
      init_butterworth_2_low_pass(&filt_quat[i], tau, sample_time, 0.0);
    }
    init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
    init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
    init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
  }

  void guidance_indi_filter_thrust(void){
    // Actuator dynamics
    thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);
    // printf('thrust act: %f',thrust_act);
    // same filter as for the acceleration
    
    // printf("Thrust_act (input for filter): \t%f ",thrust_act);
    update_butterworth_2_low_pass(&thrust_filt, thrust_act);
  }

  /**
  * Low pass the accelerometer measurements to remove noise from vibrations.
  * The roll and pitch also need to be filtered to synchronize them with the
  * acceleration
  */
  void guidance_indi_propagate_filters(struct FloatEulers *eulers){
    struct NedCoor_f *accel = stateGetAccelNed_f();
    update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
    update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
    update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

    // struct FloatQuat *quat = stateGetNedToBodyQuat_f();
    // update_butterworth_2_low_pass(&filt_quat[0], quat->qi);
    // update_butterworth_2_low_pass(&filt_quat[1], quat->qx);
    // update_butterworth_2_low_pass(&filt_quat[2], quat->qy);
    // update_butterworth_2_low_pass(&filt_quat[3], quat->qz);

    // printf("\nquat\n%f\t%f\t%f\t%f\n\n",quat->qi,quat->qx,quat->qy,quat->qz);
    printf("\nquat\n%f\t%f\t%f\t%f\n\n",stateGetNedToBodyQuat_f()->qi,stateGetNedToBodyQuat_f()->qx,stateGetNedToBodyQuat_f()->qy,stateGetNedToBodyQuat_f()->qz);


    update_butterworth_2_low_pass(&roll_filt, eulers->phi);
    update_butterworth_2_low_pass(&pitch_filt, eulers->theta);
  }

//

// CELib (matlab functions)

  /*
  * function res = a_D_wing_x(V)
  */
  float a_D_wing_x(void)
  {
    float a;
    float alpha_rad;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;

    /* 'a_D_wing_x:2' Vx = V(1); */
    /* 'a_D_wing_x:3' Vz = V(3); */
    /* 'a_D_wing_x:5' rho = 1.225; */
    /* 'a_D_wing_x:6' S = 0.2; */
    /* 'a_D_wing_x:7' m= 0.8; */
    /* 'a_D_wing_x:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    a = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'a_D_wing_x:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'a_D_wing_x:12' alpha = alpha_rad * 180 / pi; */
    /* 'a_D_wing_x:14' res = rho*S/(2*m)* ... */
    /* 'a_D_wing_x:15'                 ( sin(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
    x = alpha_rad * 180.0F / M_PI;

    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:86' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = x * 0.01675F;
    b_CDa_tmp = 2.0F * x * 0.01675F;
    c_CDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    return 0.153125F * (sinf(alpha_rad) * ((((((((0.1004F * cosf(CDa_tmp) +
      0.8371F) + 0.01822F * sinf(CDa_tmp)) + -0.6466F * cosf(b_CDa_tmp)) +
      -0.005324F * sinf(b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp)) + 0.008351F *
      sinf(c_CDa_tmp)) + -0.05651F * cosf(x)) + 0.0007461F * sinf(x)) * (a * a));
  }
  /* End of code generation (a_D_wing_x.c) */

  /*
  * function res = a_D_wing_z(V,phi)
  */
  float a_D_wing_z(void)
  {
    float phi_rad;
    float R_tmp;
    float fv[9];
    int i;
    float fv1[3];
    float alpha_rad;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;

    /* 'a_D_wing_z:2' phi_rad = -phi * pi/180.0; */
    phi_rad = -phi * M_PI / 180.0F;

    /* 'a_D_wing_z:3' R = [1 0 0; 0 cos(phi_rad) -sin(phi_rad); 0 sin(phi_rad) cos(phi_rad)]; */
    R_tmp = sinf(phi_rad);
    phi_rad = cosf(phi_rad);

    /* 'a_D_wing_z:5' V = R*V; */
    fv[0] = 1.0F;
    fv[3] = 0.0F;
    fv[6] = 0.0F;
    fv[1] = 0.0F;
    fv[4] = phi_rad;
    fv[7] = -R_tmp;
    fv[2] = 0.0F;
    fv[5] = R_tmp;
    fv[8] = phi_rad;
    for (i = 0; i < 3; i++) {
      fv1[i] = (fv[i] * V[0] + fv[i + 3] * V[1]) + fv[i + 6] * V[2];
    }

    /* 'a_D_wing_z:7' Vx = V(1); */
    /* 'a_D_wing_z:8' Vz = V(3); */
    /* 'a_D_wing_z:10' rho = 1.225; */
    /* 'a_D_wing_z:11' S = 0.2; */
    /* 'a_D_wing_z:12' m = 0.8; */
    /* 'a_D_wing_z:14' Vxz = (Vx^2 + Vz^2)^0.5; */
    R_tmp = sqrtf(fv1[0] * fv1[0] + fv1[2] * fv1[2]);

    /* 'a_D_wing_z:16' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(fv1[0], fv1[2]);

    /* 'a_D_wing_z:17' alpha = alpha_rad * 180 / pi; */
    /* 'a_D_wing_z:19' res = rho*S/(2*m)* cos(phi*pi/180) *... */
    /* 'a_D_wing_z:20'                 ( cos(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
    phi_rad = alpha_rad * 180.0F / M_PI;

    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (phi_rad > 180.0F) {
      /* 'CE:86' x = x - 360  */
      phi_rad -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = phi_rad * 0.01675F;
    b_CDa_tmp = 2.0F * phi_rad * 0.01675F;
    c_CDa_tmp = 3.0F * phi_rad * 0.01675F;
    phi_rad = 4.0F * phi_rad * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    return 0.153125F * cosf(phi * M_PI / 180.0F) * (cosf(alpha_rad) *
      ((((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) +
            -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
          -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) + -0.05651F *
        cosf(phi_rad)) + 0.0007461F * sinf(phi_rad)) * (R_tmp * R_tmp));
  }
  /* End of code generation (a_D_wing_z.c) */

  /*
  * function res = a_L_wing_x(V)
  */
  float a_L_wing_x(void)
  {
    float a;
    float alpha_rad;
    float x;

    /* 'a_L_wing_x:2' Vx = V(1); */
    /* 'a_L_wing_x:3' Vz = V(3); */
    /* 'a_L_wing_x:5' rho = 1.225; */
    /* 'a_L_wing_x:6' S = 0.2; */
    /* 'a_L_wing_x:7' m = 0.8; */
    /* 'a_L_wing_x:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    a = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'a_L_wing_x:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'a_L_wing_x:12' alpha = alpha_rad * 180 / pi; */
    /* 'a_L_wing_x:14' res = rho*S/(2*m)* ... */
    /* 'a_L_wing_x:15'                 ( cos(alpha_rad)* CE.Cl(alpha) * Vxz^2 ); */
    x = alpha_rad * 180.0F / M_PI;

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    /* 'CE:40' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    return 0.153125F * (cosf(alpha_rad) * (((((((1.233F * sinf(0.04068F * x +
      0.09493F) + 141.1F * sinf(0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F *
      x + 0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf
      (0.01427F * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) +
      0.03084F * sinf(0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x +
      3.305F)) * (a * a));
  }
  /* End of code generation (a_L_wing_x.c) */

  /*
  * function res = a_L_wing_z(V,phi)
  */
  float a_L_wing_z(void)
  {
    float phi_rad;
    float alpha_rad;
    float fv[9];
    int i;
    float fv1[3];
    float x;

    /* 'a_L_wing_z:2' phi_rad = -phi * pi/180.0; */
    phi_rad = -phi * M_PI / 180.0F;

    /* 'a_L_wing_z:3' R = [1 0 0; 0 cos(phi_rad) -sin(phi_rad); 0 sin(phi_rad) cos(phi_rad)]; */
    alpha_rad = sinf(phi_rad);
    phi_rad = cosf(phi_rad);

    /* 'a_L_wing_z:5' V = R*V; */
    fv[0] = 1.0F;
    fv[3] = 0.0F;
    fv[6] = 0.0F;
    fv[1] = 0.0F;
    fv[4] = phi_rad;
    fv[7] = -alpha_rad;
    fv[2] = 0.0F;
    fv[5] = alpha_rad;
    fv[8] = phi_rad;
    for (i = 0; i < 3; i++) {
      fv1[i] = (fv[i] * V[0] + fv[i + 3] * V[1]) + fv[i + 6] * V[2];
    }

    /* 'a_L_wing_z:7' Vx = V(1); */
    /* 'a_L_wing_z:8' Vz = V(3); */
    /* 'a_L_wing_z:10' rho = 1.225; */
    /* 'a_L_wing_z:11' S = 0.2; */
    /* 'a_L_wing_z:12' m = 0.8; */
    /* 'a_L_wing_z:14' Vxz = (Vx^2 + Vz^2)^0.5; */
    phi_rad = sqrtf(fv1[0] * fv1[0] + fv1[2] * fv1[2]);

    /* 'a_L_wing_z:16' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(fv1[0], fv1[2]);

    /* 'a_L_wing_z:17' alpha = alpha_rad * 180 / pi; */
    /* 'a_L_wing_z:19' res = rho*S/(2*m)* cos(phi*pi/180) *... */
    /* 'a_L_wing_z:20'                 ( -sin(alpha_rad)* CE.Cl(alpha) * Vxz^2 ); */
    x = alpha_rad * 180.0F / M_PI;

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    /* 'CE:40' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    return 0.153125F * cosf(phi * M_PI / 180.0F) * (-sinf(alpha_rad) *
      (((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf(0.05424F * x +
      0.1637F)) + 0.1126F * sinf(0.101F * x + 0.2612F)) + 0.08224F * sinf(0.136F *
      x + 0.2642F)) + 0.1197F * sinf(0.01427F * x + 0.6238F)) + 0.05624F * sinf
        (0.1704F * x + 0.2682F)) + 0.03084F * sinf(0.2046F * x + 0.4082F)) +
      141.6F * sinf(0.05419F * x + 3.305F)) * (phi_rad * phi_rad));
  }
  /* End of code generation (a_L_wing_z.c) */

  /*
  * function res = a_wing_x(V)
  */
  float a_wing_x(void)
  {
    float Vxz;
    float alpha_rad;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float d_CDa_tmp;

    /* 'a_wing_x:2' Vx = V(1); */
    /* 'a_wing_x:3' Vz = V(3); */
    /* 'a_wing_x:5' rho = 1.225; */
    /* 'a_wing_x:6' S = 0.2; */
    /* 'a_wing_x:7' m = 0.8; */
    /* 'a_wing_x:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    Vxz = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'a_wing_x:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'a_wing_x:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'a_wing_x:14' res = rho*S/(2*m)* ... */
    /* 'a_wing_x:15'                 ( cos(alpha_rad)* CE.Cl(alpha) * Vxz^2 ... */
    /* 'a_wing_x:16'                  +sin(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    d_CDa_tmp = 4.0F * alpha * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    alpha = Vxz * Vxz;
    return 0.153125F * (cosf(alpha_rad) * (((((((1.233F * sinf(0.04068F * x +
      0.09493F) + 141.1F * sinf(0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F *
      x + 0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf
      (0.01427F * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) +
      0.03084F * sinf(0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x +
      3.305F)) * alpha + sinf(alpha_rad) * ((((((((0.1004F * cosf(CDa_tmp) +
      0.8371F) + 0.01822F * sinf(CDa_tmp)) + -0.6466F * cosf(b_CDa_tmp)) +
      -0.005324F * sinf(b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp)) + 0.008351F *
      sinf(c_CDa_tmp)) + -0.05651F * cosf(d_CDa_tmp)) + 0.0007461F * sinf
      (d_CDa_tmp)) * alpha);
  }
  /* End of code generation (a_wing_x.c) */

  /*
  * function res = a_wing_z(V,phi)
  */
  float a_wing_z(void)
  {
    float phi_rad;
    float Vxz;
    float fv[9];
    int i;
    float fv1[3];
    float alpha_rad;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float d_CDa_tmp;

    /* 'a_wing_z:2' phi_rad = -phi * pi/180.0; */
    phi_rad = -phi * M_PI / 180.0F;

    /* 'a_wing_z:3' R = [1 0 0; 0 cos(phi_rad) -sin(phi_rad); 0 sin(phi_rad) cos(phi_rad)]; */
    Vxz = sinf(phi_rad);
    phi_rad = cosf(phi_rad);

    /* 'a_wing_z:5' V = R*V; */
    fv[0] = 1.0F;
    fv[3] = 0.0F;
    fv[6] = 0.0F;
    fv[1] = 0.0F;
    fv[4] = phi_rad;
    fv[7] = -Vxz;
    fv[2] = 0.0F;
    fv[5] = Vxz;
    fv[8] = phi_rad;
    for (i = 0; i < 3; i++) {
      fv1[i] = (fv[i] * V[0] + fv[i + 3] * V[1]) + fv[i + 6] * V[2];
    }

    /* 'a_wing_z:7' Vx = V(1); */
    /* 'a_wing_z:8' Vz = V(3); */
    /* 'a_wing_z:10' rho = 1.225; */
    /* 'a_wing_z:11' S = 0.2; */
    /* 'a_wing_z:12' m = 0.8; */
    /* 'a_wing_z:14' Vxz = (Vx^2 + Vz^2)^0.5; */
    Vxz = sqrtf(fv1[0] * fv1[0] + fv1[2] * fv1[2]);

    /* 'a_wing_z:16' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(fv1[0], fv1[2]);

    /* 'a_wing_z:17' alpha = alpha_rad * 180 / pi; */
    phi_rad = alpha_rad * 180.0F / M_PI;

    /* 'a_wing_z:19' res = rho*S/(2*m)* cos(phi*pi/180) * ... */
    /* 'a_wing_z:20'                 ( -sin(alpha_rad)* CE.Cl(alpha) * Vxz^2 ... */
    /* 'a_wing_z:21'                  +cos(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = phi_rad;

    /* 'CE:40' if x > 180 */
    if (phi_rad > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = phi_rad - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (phi_rad > 180.0F) {
      /* 'CE:86' x = x - 360  */
      phi_rad -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = phi_rad * 0.01675F;
    b_CDa_tmp = 2.0F * phi_rad * 0.01675F;
    c_CDa_tmp = 3.0F * phi_rad * 0.01675F;
    d_CDa_tmp = 4.0F * phi_rad * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    phi_rad = Vxz * Vxz;
    return 0.153125F * cosf(phi * M_PI / 180.0F) * (-sinf(alpha_rad) *
      (((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf(0.05424F * x +
      0.1637F)) + 0.1126F * sinf(0.101F * x + 0.2612F)) + 0.08224F * sinf(0.136F *
      x + 0.2642F)) + 0.1197F * sinf(0.01427F * x + 0.6238F)) + 0.05624F * sinf
        (0.1704F * x + 0.2682F)) + 0.03084F * sinf(0.2046F * x + 0.4082F)) +
      141.6F * sinf(0.05419F * x + 3.305F)) * phi_rad + cosf(alpha_rad) *
      ((((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) +
            -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
          -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) + -0.05651F *
        cosf(d_CDa_tmp)) + 0.0007461F * sinf(d_CDa_tmp)) * phi_rad);
  }
  /* End of code generation (a_wing_z.c) */

  /*
  * function res = AoA(V)
  */
  float AoA(void)
  {
    float res;

    /* 'AoA:2' Vx = V(1); */
    /* 'AoA:3' Vz = V(3); */
    /* 'AoA:5' alpha_rad = atan2(Vx,Vz); */
    /* 'AoA:6' alpha = alpha_rad * 180 / pi; */
    res = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'AoA:8' res = alpha; */
    return res;
  }
  /* End of code generation (AoA.c) */

  /*
  * function res = AoD(V)
  */
  float AoD(void)
  {
    float res;

    /* 'AoD:2' Vx = V(1); */
    /* 'AoD:3' Vy = V(2); */
    /* sign of gamma not sure yet, remove comment if checked */
    /* 'AoD:6' gamma_rad = atan2(Vx,Vy); */
    /* 'AoD:7' gamma = gamma_rad * 180 / pi; */
    res = atan2f(V[0], V[1]) * 180.0F / M_PI;

    /* 'AoD:9' res = gamma; */
    return res;
  }
  /* End of code generation (AoD.c) */

  /*
  * function res = AoS(V)
  */
  float AoS(void)
  {
    float res;

    /* 'AoS:2' Vy = V(2); */
    /* 'AoS:3' Vz = V(3); */
    /* 'AoS:5' alpha_rad = atan2(Vy,Vz); */
    /* 'AoS:6' alpha = alpha_rad * 180 / pi; */
    res = atan2f(V[1], V[2]) * 180.0F / M_PI;

    /* 'AoS:8' res = alpha; */
    return res;
  }
  /* End of code generation (AoS.c) */

  /*
  * function res = Cd(alpha)
  */
  float Cd(void)
  {
    float res;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;

    /* 'Cd:2' x  =      alpha     ; */
    x = alpha;

    /* 'Cd:3' if x > 180 */
    if (alpha > 180.0F) {
      /* 'Cd:4' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'Cd:6' a0 =      0.8371  ; */
    /* 'Cd:7' a1 =      0.1004 ; */
    /* 'Cd:8' b1 =     0.01822  ; */
    /* 'Cd:9' a2 =     -0.6466  ; */
    /* 'Cd:10' b2 =   -0.005324 ; */
    /* 'Cd:11' a3 =    -0.02505 ; */
    /* 'Cd:12' b3 =    0.008351 ; */
    /* 'Cd:13' a4 =    -0.05651 ; */
    /* 'Cd:14' b4 =   0.0007461  ; */
    /* 'Cd:15' w =     0.01675 ; */
    /* 'Cd:16' CDa =  a0 + ... */
    /* 'Cd:17'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'Cd:18'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'Cd:19'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'Cd:20'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = x * 0.01675F;
    b_CDa_tmp = 2.0F * x * 0.01675F;
    c_CDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;
    res = (((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) +
              -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
            -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) +
          -0.05651F * cosf(x)) + 0.0007461F * sinf(x);

    /* 'Cd:21' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    return res;
  }
  /* End of code generation (Cd.c) */

  /*
  * function res = Cl(alpha)
  */
  float Cl(void)
  {
    float res;
    float x;

    /* 'Cl:2' x  =      alpha     ; */
    x = alpha;

    /* 'Cl:3' if x > 180 */
    if (alpha > 180.0F) {
      /* 'Cl:4' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'Cl:6' a1 =      1.233     ; */
    /* 'Cl:7' b1 =      0.04068   ; */
    /* 'Cl:8' c1 =      0.09493   ; */
    /* 'Cl:9' a2 =    141.1       ; */
    /* 'Cl:10' b2 =      0.05424   ; */
    /* 'Cl:11' c2 =      0.1637    ; */
    /* 'Cl:12' a3 =      0.1126    ; */
    /* 'Cl:13' b3 =      0.101     ; */
    /* 'Cl:14' c3 =      0.2612    ; */
    /* 'Cl:15' a4 =      0.08224   ; */
    /* 'Cl:16' b4 =      0.136     ; */
    /* 'Cl:17' c4 =      0.2642    ; */
    /* 'Cl:18' a5 =      0.1197    ; */
    /* 'Cl:19' b5 =      0.01427   ; */
    /* 'Cl:20' c5 =      0.6238    ; */
    /* 'Cl:21' a6 =      0.05624   ; */
    /* 'Cl:22' b6 =      0.1704    ; */
    /* 'Cl:23' c6 =      0.2682    ; */
    /* 'Cl:24' a7 =      0.03084   ; */
    /* 'Cl:25' b7 =      0.2046    ; */
    /* 'Cl:26' c7 =      0.4082    ; */
    /* 'Cl:27' a8 =    141.6       ; */
    /* 'Cl:28' b8 =      0.05419   ; */
    /* 'Cl:29' c8 =      3.305     ; */
    /* 'Cl:30' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'Cl:31'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'Cl:32'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    res = ((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf(0.05424F *
      x + 0.1637F)) + 0.1126F * sinf(0.101F * x + 0.2612F)) + 0.08224F * sinf
              (0.136F * x + 0.2642F)) + 0.1197F * sinf(0.01427F * x + 0.6238F)) +
            0.05624F * sinf(0.1704F * x + 0.2682F)) + 0.03084F * sinf(0.2046F * x
            + 0.4082F)) + 141.6F * sinf(0.05419F * x + 3.305F);

    /* 'Cl:33' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    return res;
  }
  /* End of code generation (Cl.c) */

  /*
  * function res = dCd(alpha)
  */
  float dCd(void)
  {
    float res;
    float x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'dCd:2' x  =      alpha   ; */
    x = alpha;

    /* 'dCd:3' if x > 180 */
    if (alpha > 180.0F) {
      /* 'dCd:4' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'dCd:6' a1 =      0.1004  ; */
    /* 'dCd:7' b1 =     0.01822  ; */
    /* 'dCd:8' a2 =     -0.6466  ; */
    /* 'dCd:9' b2 =   -0.005324 ; */
    /* 'dCd:10' a3 =    -0.02505 ; */
    /* 'dCd:11' b3 =    0.008351 ; */
    /* 'dCd:12' a4 =    -0.05651 ; */
    /* 'dCd:13' b4 =   0.0007461  ; */
    /* 'dCd:14' w =     0.01675 ; */
    /* 'dCd:15' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'dCd:16'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'dCd:17'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'dCd:18'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = x * 0.01675F;
    b_dCDa_tmp = 2.0F * x * 0.01675F;
    c_dCDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'dCd:19' res =  180/pi*dCDa; */
    res = 57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) + 0.000305185F * cosf
      (dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) + -0.000178354F * cosf
      (b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) + 0.000419637741F * cosf
      (c_dCDa_tmp)) - -0.00378617F * sinf(x)) + 4.99887E-5F * cosf(x));

    /*              res =  2*sin(2*alpha*pi/180); */
    return res;
  }
  /* End of code generation (dCd.c) */

  /*
  * function res = dCl(alpha)
  */
  float dCl(void)
  {
    float res;
    float x;

    /* 'dCl:2' x  =      alpha     ; */
    x = alpha;

    /* 'dCl:3' if x > 180 */
    if (alpha > 180.0F) {
      /* 'dCl:4' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'dCl:6' a1 =      1.233     ; */
    /* 'dCl:7' b1 =      0.04068   ; */
    /* 'dCl:8' c1 =      0.09493   ; */
    /* 'dCl:9' a2 =    141.1       ; */
    /* 'dCl:10' b2 =      0.05424   ; */
    /* 'dCl:11' c2 =      0.1637    ; */
    /* 'dCl:12' a3 =      0.1126    ; */
    /* 'dCl:13' b3 =      0.101     ; */
    /* 'dCl:14' c3 =      0.2612    ; */
    /* 'dCl:15' a4 =      0.08224   ; */
    /* 'dCl:16' b4 =      0.136     ; */
    /* 'dCl:17' c4 =      0.2642    ; */
    /* 'dCl:18' a5 =      0.1197    ; */
    /* 'dCl:19' b5 =      0.01427   ; */
    /* 'dCl:20' c5 =      0.6238    ; */
    /* 'dCl:21' a6 =      0.05624   ; */
    /* 'dCl:22' b6 =      0.1704    ; */
    /* 'dCl:23' c6 =      0.2682    ; */
    /* 'dCl:24' a7 =      0.03084   ; */
    /* 'dCl:25' b7 =      0.2046    ; */
    /* 'dCl:26' c7 =      0.4082    ; */
    /* 'dCl:27' a8 =    141.6       ; */
    /* 'dCl:28' b8 =      0.05419   ; */
    /* 'dCl:29' c8 =      3.305     ; */
    /* 'dCl:30' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'dCl:31'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'dCl:32'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'dCl:33' res =  180/pi*dCLa; */
    res = 57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
      cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf(0.1704F * x + 0.2682F))
                          + 0.00630986411F * cosf(0.2046F * x + 0.4082F)) +
                        7.67330408F * cosf(0.05419F * x + 3.305F));

    /*              res = cos(alpha*pi/180); */
    return res;
  }
  /* End of code generation (dCl.c) */

  /*
  * function res = Vxz_2(V)
  */
  float Vxz_2(void)
  {
    /* 'Vxz_2:2' Vx = V(1); */
    /* 'Vxz_2:3' Vz = V(3); */
    /* 'Vxz_2:4' res = Vx^2+Vz^2; */
    return V[0] * V[0] + V[2] * V[2];
  }
  /* End of code generation (Vxz_2.c) */

  //New Functions
  /*
  * function res = da_wing_x_dphii(V)
  */
  float da_wing_x_dphii(void)
  {
    float alpha_rad;
    float res_tmp;
    float x;
    float b_res_tmp;
    float res_tmp_tmp;
    float c_res_tmp;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float dCDa_tmp;

    /* 'da_wing_x_dphii:2' Vx = V(1); */
    /* 'da_wing_x_dphii:3' Vz = V(3); */
    /* 'da_wing_x_dphii:5' rho = 1.225; */
    /* 'da_wing_x_dphii:6' S = 0.2; */
    /* 'da_wing_x_dphii:7' m = 0.8; */
    /* 'da_wing_x_dphii:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    /* 'da_wing_x_dphii:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_x_dphii:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_x_dphii:14' res = rho*S/(2*m)* ... */
    /* 'da_wing_x_dphii:15' 		(-sin(alpha_rad)* CE.Cl(alpha)          * CE.Vxz_2_dalphai_dphii(V) ... */
    /* 'da_wing_x_dphii:16' 		 +cos(alpha_rad)* CE.Vxz_2_dCl_dphii(V)                             ... */
    /* 'da_wing_x_dphii:17' 		 -cos(alpha_rad)* CE.Cl(alpha)          * CE.dVxz_2_dphii(V)        ... */
    /* 'da_wing_x_dphii:18' 		 +cos(alpha_rad)* CE.Cd(alpha)          * CE.Vxz_2_dalphai_dphii(V) ... */
    /* 'da_wing_x_dphii:19' 		 +sin(alpha_rad)* CE.Vxz_2_dCd_dphii(V)                             ... */
    /* 'da_wing_x_dphii:20' 		 -sin(alpha_rad)* CE.Cd(alpha)          * CE.dVxz_2_dphii(V)        ); */
    res_tmp = cosf(alpha_rad);

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    b_res_tmp = ((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf
                      (0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F * x +
      0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf(0.01427F
      * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) + 0.03084F * sinf
                (0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x + 3.305F);

    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    res_tmp_tmp = V[0] * V[1];
    c_res_tmp = sinf(alpha_rad);

    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    alpha_rad = 4.0F * alpha * 0.01675F;
    b_CDa_tmp = (((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf
                      (CDa_tmp)) + -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf
                    (b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf
                  (c_CDa_tmp)) + -0.05651F * cosf(alpha_rad)) + 0.0007461F * sinf
      (alpha_rad);

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    /* 'CE:312' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:313' res = CE.dCl(alpha) * CE.Vxz_2_dalphai_dphii(V); */
    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    /* 'CE:339' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    alpha_rad = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:340' res = CE.dCd(alpha) * CE.Vxz_2_dalphai_dphii(V); */
    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:157' x = x - 360  */
      alpha_rad -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    c_CDa_tmp = alpha_rad * 0.01675F;
    alpha = 2.0F * alpha_rad * 0.01675F;
    dCDa_tmp = 3.0F * alpha_rad * 0.01675F;
    alpha_rad = 4.0F * alpha_rad * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    /* dVxz^2 */
    /* 'CE:463' Vy = V(2); */
    /* 'CE:464' Vz = V(3); */
    /* 'CE:465' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'CE:466' res = 2*Vz*sin(beta_rad)*sqrt(Vy^2+Vz^2); */
    /* dVxz^2 */
    /* 'CE:463' Vy = V(2); */
    /* 'CE:464' Vz = V(3); */
    /* 'CE:465' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'CE:466' res = 2*Vz*sin(beta_rad)*sqrt(Vy^2+Vz^2); */
    CDa_tmp = 2.0F * V[2] * sinf(atan2f(V[1], V[2])) * sqrtf(V[1] * V[1] + V[2] * V[2]);
    return 0.153125F * (((((-c_res_tmp * b_res_tmp * res_tmp_tmp + res_tmp *
      (57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
                        cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf
                        (0.1704F * x + 0.2682F)) + 0.00630986411F * cosf(0.2046F *
      x + 0.4082F)) + 7.67330408F * cosf(0.05419F * x + 3.305F)) * res_tmp_tmp)) -
      res_tmp * b_res_tmp * CDa_tmp) + res_tmp * b_CDa_tmp * res_tmp_tmp) +
                        c_res_tmp * (57.2957802F * (((((((-0.0016817F * sinf
      (c_CDa_tmp) + 0.000305185F * cosf(c_CDa_tmp)) - -0.0216611F * sinf(alpha)) +
      -0.000178354F * cosf(alpha)) - -0.00125876255F * sinf(dCDa_tmp)) +
      0.000419637741F * cosf(dCDa_tmp)) - -0.00378617F * sinf(alpha_rad)) +
      4.99887E-5F * cosf(alpha_rad)) * res_tmp_tmp)) - c_res_tmp * b_CDa_tmp *
                        CDa_tmp);
  }
  /* End of code generation (da_wing_x_dphii.c) */

  /*
  * function res = da_wing_x_dpsii(V)
  */
  float da_wing_x_dpsii(void)
  {
    float alpha_rad;
    float res_tmp;
    float x;
    float b_res_tmp;
    float res_tmp_tmp;
    float c_res_tmp;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float dCDa_tmp;

    /* 'da_wing_x_dpsii:2' Vx = V(1); */
    /* 'da_wing_x_dpsii:3' Vz = V(3); */
    /* 'da_wing_x_dpsii:5' rho = 1.225; */
    /* 'da_wing_x_dpsii:6' S = 0.2; */
    /* 'da_wing_x_dpsii:7' m = 0.8; */
    /* 'da_wing_x_dpsii:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    /* 'da_wing_x_dpsii:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_x_dpsii:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_x_dpsii:14' res = rho*S/(2*m)* ... */
    /* 'da_wing_x_dpsii:15' 		(+sin(alpha_rad)* CE.Cl(alpha)          * CE.Vxz_2_dalphai_dpsii(V)... */
    /* 'da_wing_x_dpsii:16' 		 -cos(alpha_rad)* CE.Vxz_2_dCl_dpsii(V)                                                ... */
    /* 'da_wing_x_dpsii:17' 		 -cos(alpha_rad)* CE.Cl(alpha)          * CE.dVxz_2_dpsii(V)                             ... */
    /* 'da_wing_x_dpsii:18' 		 -cos(alpha_rad)* CE.Cd(alpha)          * CE.Vxz_2_dalphai_dpsii(V)... */
    /* 'da_wing_x_dpsii:19' 		 -sin(alpha_rad)* CE.Vxz_2_dCd_dpsii(V)                                                  ... */
    /* 'da_wing_x_dpsii:20' 		 -sin(alpha_rad)* CE.Cd(alpha)          * CE.dVxz_2_dpsii(V)                             ); */
    res_tmp = cosf(alpha_rad);

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    b_res_tmp = ((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf
                      (0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F * x +
      0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf(0.01427F
      * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) + 0.03084F * sinf
                (0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x + 3.305F);

    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    res_tmp_tmp = V[1] * V[2];
    c_res_tmp = sinf(alpha_rad);

    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    alpha_rad = 4.0F * alpha * 0.01675F;
    CDa_tmp = (((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp))
                  + -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
                -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) +
              -0.05651F * cosf(alpha_rad)) + 0.0007461F * sinf(alpha_rad);

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    /* 'CE:474' Vx = V(1); */
    /* 'CE:475' Vy = V(2); */
    /* 'CE:476' gamma_rad = CE.AoD(V)*pi/180; */
    /*              res = 2*Vx*sin(gamma_rad)*sqrt(Vx^2+Vy^2); */
    /* 'CE:478' res = 2*Vx*Vy; */
    b_CDa_tmp = 2.0F * V[0] * V[1];

    /* 'CE:328' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:329' res = CE.dCl(alpha) * CE.Vxz_2_dalphai_dpsii(V); */
    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    /* 'CE:354' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    alpha_rad = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:355' res = CE.dCd(alpha) * CE.Vxz_2_dalphai_dpsii(V); */
    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:157' x = x - 360  */
      alpha_rad -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    c_CDa_tmp = alpha_rad * 0.01675F;
    alpha = 2.0F * alpha_rad * 0.01675F;
    dCDa_tmp = 3.0F * alpha_rad * 0.01675F;
    alpha_rad = 4.0F * alpha_rad * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    return 0.153125F * (((((c_res_tmp * b_res_tmp * res_tmp_tmp - res_tmp *
      (57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
                        cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf
                        (0.1704F * x + 0.2682F)) + 0.00630986411F * cosf(0.2046F *
      x + 0.4082F)) + 7.67330408F * cosf(0.05419F * x + 3.305F)) * res_tmp_tmp)) -
      res_tmp * b_res_tmp * b_CDa_tmp) - res_tmp * CDa_tmp * res_tmp_tmp) -
                        c_res_tmp * (57.2957802F * (((((((-0.0016817F * sinf
      (c_CDa_tmp) + 0.000305185F * cosf(c_CDa_tmp)) - -0.0216611F * sinf(alpha)) +
      -0.000178354F * cosf(alpha)) - -0.00125876255F * sinf(dCDa_tmp)) +
      0.000419637741F * cosf(dCDa_tmp)) - -0.00378617F * sinf(alpha_rad)) +
      4.99887E-5F * cosf(alpha_rad)) * res_tmp_tmp)) - c_res_tmp * CDa_tmp *
                        b_CDa_tmp);
  }
  /* End of code generation (da_wing_x_dpsii.c) */

  /*
  * function res = da_wing_x_dthetai(V)
  */
  float da_wing_x_dthetai(void)
  {
    float Vxz;
    float alpha_rad;
    float res_tmp;
    float b_res_tmp;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float d_CDa_tmp;
    float b_x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'da_wing_x_dthetai:2' Vx = V(1); */
    /* 'da_wing_x_dthetai:3' Vz = V(3); */
    /* 'da_wing_x_dthetai:5' rho = 1.225; */
    /* 'da_wing_x_dthetai:6' S = 0.2; */
    /* 'da_wing_x_dthetai:7' m = 0.8; */
    /* 'da_wing_x_dthetai:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    Vxz = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'da_wing_x_dthetai:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_x_dthetai:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_x_dthetai:14' res = -rho*S/(2*m)* ... */
    /* 'da_wing_x_dthetai:15' 		(-cos(alpha_rad)* CE.dCl_dthetai(V)* Vxz^2                                                   ... */
    /* 'da_wing_x_dthetai:16' 		 -cos(alpha_rad)* CE.Cl(alpha)            * CE.dVxz_2_dthetai()                   * CE.dalphai_dthetai()... */
    /* 'da_wing_x_dthetai:17' 		 -sin(alpha_rad)* CE.dCd_dthetai(V)* Vxz^2                                                   ... */
    /* 'da_wing_x_dthetai:18' 		 -sin(alpha_rad)* CE.Cd(alpha)            * CE.dVxz_2_dthetai()                             ); */
    res_tmp = cosf(alpha_rad);
    b_res_tmp = sinf(alpha_rad);

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    d_CDa_tmp = 4.0F * alpha * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    /* 'CE:318' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    b_x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:319' res = CE.dCl(alpha) * CE.dalphai_dthetai(); */
    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (b_x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      b_x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* 'CE:344' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    alpha_rad = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:345' res = CE.dCd(alpha) * CE.dalphai_dthetai(); */
    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:157' x = x - 360  */
      alpha_rad -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = alpha_rad * 0.01675F;
    b_dCDa_tmp = 2.0F * alpha_rad * 0.01675F;
    c_dCDa_tmp = 3.0F * alpha_rad * 0.01675F;
    alpha = 4.0F * alpha_rad * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    alpha_rad = Vxz * Vxz;
    return -0.153125F * (((-res_tmp * -(57.2957802F * (((((((0.0501584411F * cosf
      (0.04068F * b_x + 0.09493F) + 7.65326405F * cosf(0.05424F * b_x + 0.1637F))
      + 0.0113726F * cosf(0.101F * b_x + 0.2612F)) + 0.0111846402F * cosf(0.136F *
      b_x + 0.2642F)) + 0.00170811894F * cosf(0.01427F * b_x + 0.6238F)) +
      0.00958329625F * cosf(0.1704F * b_x + 0.2682F)) + 0.00630986411F * cosf
      (0.2046F * b_x + 0.4082F)) + 7.67330408F * cosf(0.05419F * b_x + 3.305F))) *
      alpha_rad - (-(res_tmp * (((((((1.233F * sinf(0.04068F * x + 0.09493F) +
      141.1F * sinf(0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F * x + 0.2612F))
      + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf(0.01427F * x +
      0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) + 0.03084F * sinf
      (0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x + 3.305F)) * 0.0F))) -
                          b_res_tmp * -(57.2957802F * (((((((-0.0016817F * sinf
      (dCDa_tmp) + 0.000305185F * cosf(dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp))
      + -0.000178354F * cosf(b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) +
      0.000419637741F * cosf(c_dCDa_tmp)) - -0.00378617F * sinf(alpha)) +
      4.99887E-5F * cosf(alpha))) * alpha_rad) - b_res_tmp * ((((((((0.1004F *
      cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) + -0.6466F * cosf
      (b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp))
      + 0.008351F * sinf(c_CDa_tmp)) + -0.05651F * cosf(d_CDa_tmp)) + 0.0007461F *
      sinf(d_CDa_tmp)) * 0.0F);
  }
  /* End of code generation (da_wing_x_dthetai.c) */

  /*
  * function res = da_wing_y_dphii(V)
  */
  float da_wing_y_dphii(void)
  {
    float Vxz;
    float alpha_rad;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float d_CDa_tmp;

    /* 'da_wing_y_dphii:2' Vx = V(1); */
    /* 'da_wing_y_dphii:3' Vz = V(3); */
    /* 'da_wing_y_dphii:5' rho = 1.225; */
    /* 'da_wing_y_dphii:6' S = 0.2; */
    /* 'da_wing_y_dphii:7' m = 0.8; */
    /* 'da_wing_y_dphii:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    Vxz = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'da_wing_y_dphii:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_y_dphii:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_y_dphii:14' res = rho*S/(2*m)* ... */
    /* 'da_wing_y_dphii:15' 		(+sin(alpha_rad)* CE.Cl(alpha) * Vxz^2 ... */
    /* 'da_wing_y_dphii:16' 		 -cos(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    d_CDa_tmp = 4.0F * alpha * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    alpha = Vxz * Vxz;
    return 0.153125F * (sinf(alpha_rad) * (((((((1.233F * sinf(0.04068F * x +
      0.09493F) + 141.1F * sinf(0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F *
      x + 0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf
      (0.01427F * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) +
      0.03084F * sinf(0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x +
      3.305F)) * alpha - cosf(alpha_rad) * ((((((((0.1004F * cosf(CDa_tmp) +
      0.8371F) + 0.01822F * sinf(CDa_tmp)) + -0.6466F * cosf(b_CDa_tmp)) +
      -0.005324F * sinf(b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp)) + 0.008351F *
      sinf(c_CDa_tmp)) + -0.05651F * cosf(d_CDa_tmp)) + 0.0007461F * sinf
      (d_CDa_tmp)) * alpha);
  }
  /* End of code generation (da_wing_y_dphii.c) */

  /*
  * function res = da_wing_y_dpsii(V)
  */
  float da_wing_y_dpsii(void)
  {
    float Vxz;
    float alpha_rad;
    float x;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float d_CDa_tmp;

    /* 'da_wing_y_dpsii:2' Vx = V(1); */
    /* 'da_wing_y_dpsii:3' Vz = V(3); */
    /* 'da_wing_y_dpsii:5' rho = 1.225; */
    /* 'da_wing_y_dpsii:6' S = 0.2; */
    /* 'da_wing_y_dpsii:7' m = 0.8; */
    /* 'da_wing_y_dpsii:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    Vxz = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'da_wing_y_dpsii:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_y_dpsii:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_y_dpsii:14' res = rho*S/(2*m)* ... */
    /* 'da_wing_y_dpsii:15' 		(-cos(alpha_rad)* CE.Cl(alpha) * Vxz^2 ... */
    /* 'da_wing_y_dpsii:16' 		 -sin(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    d_CDa_tmp = 4.0F * alpha * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    alpha = Vxz * Vxz;
    return 0.153125F * (-cosf(alpha_rad) * (((((((1.233F * sinf(0.04068F * x +
      0.09493F) + 141.1F * sinf(0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F *
      x + 0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf
      (0.01427F * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) +
      0.03084F * sinf(0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x +
      3.305F)) * alpha - sinf(alpha_rad) * ((((((((0.1004F * cosf(CDa_tmp) +
      0.8371F) + 0.01822F * sinf(CDa_tmp)) + -0.6466F * cosf(b_CDa_tmp)) +
      -0.005324F * sinf(b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp)) + 0.008351F *
      sinf(c_CDa_tmp)) + -0.05651F * cosf(d_CDa_tmp)) + 0.0007461F * sinf
      (d_CDa_tmp)) * alpha);
  }
  /* End of code generation (da_wing_y_dpsii.c) */

  /*
  * function res = da_wing_y_dthetai
  */
  float da_wing_y_dthetai(void)
  {
    /* 'da_wing_y_dthetai:2' res = 0; */
    return 0.0;
  }
  /* End of code generation (da_wing_y_dthetai.c) */

  /*
  * function res = da_wing_z_dphii(V)
  */
  float da_wing_z_dphii(void)
  {
    float alpha_rad;
    float res_tmp;
    float x;
    float b_res_tmp;
    float res_tmp_tmp;
    float c_res_tmp;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float dCDa_tmp;
    /* 'da_wing_z_dphii:2' Vx = V(1); */
    /* 'da_wing_z_dphii:3' Vz = V(3); */
    /* 'da_wing_z_dphii:5' rho = 1.225; */
    /* 'da_wing_z_dphii:6' S = 0.2; */
    /* 'da_wing_z_dphii:7' m = 0.8; */
    /* 'da_wing_z_dphii:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    /* 'da_wing_z_dphii:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_z_dphii:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_z_dphii:14' res = rho*S/(2*m)* ... */
    /* 'da_wing_z_dphii:15' 		(-cos(alpha_rad)* CE.Cl(alpha)          * CE.Vxz_2_dalphai_dphii(V)... */
    /* 'da_wing_z_dphii:16' 		 -sin(alpha_rad)* CE.Vxz_2_dCl_dphii(V)                                                 ... */
    /* 'da_wing_z_dphii:17' 		 -sin(alpha_rad)* CE.Cl(alpha)          * CE.dVxz_2_dphii(V)                             ... */
    /* 'da_wing_z_dphii:18' 		 -sin(alpha_rad)* CE.Cd(alpha)          * CE.Vxz_2_dalphai_dphii(V)... */
    /* 'da_wing_z_dphii:19' 		 +cos(alpha_rad)* CE.Vxz_2_dCd_dphii(V)                                               ... */
    /* 'da_wing_z_dphii:20' 		 +cos(alpha_rad)* CE.Cd(alpha)          * CE.dVxz_2_dphii(V)                             ); */
    res_tmp = sinf(alpha_rad);

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    b_res_tmp = ((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf
                      (0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F * x +
      0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf(0.01427F
      * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) + 0.03084F * sinf
                (0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x + 3.305F);

    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    res_tmp_tmp = V[0] * V[1];
    c_res_tmp = cosf(alpha_rad);

    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    alpha_rad = 4.0F * alpha * 0.01675F;
    b_CDa_tmp = (((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf
                      (CDa_tmp)) + -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf
                    (b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf
                  (c_CDa_tmp)) + -0.05651F * cosf(alpha_rad)) + 0.0007461F * sinf
      (alpha_rad);

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    /* 'CE:312' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:313' res = CE.dCl(alpha) * CE.Vxz_2_dalphai_dphii(V); */
    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    /* 'CE:339' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    alpha_rad = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:13' res = alpha; */
    /* 'CE:340' res = CE.dCd(alpha) * CE.Vxz_2_dalphai_dphii(V); */
    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:157' x = x - 360  */
      alpha_rad -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    c_CDa_tmp = alpha_rad * 0.01675F;
    alpha = 2.0F * alpha_rad * 0.01675F;
    dCDa_tmp = 3.0F * alpha_rad * 0.01675F;
    alpha_rad = 4.0F * alpha_rad * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    /* dVxz^2 */
    /* 'CE:463' Vy = V(2); */
    /* 'CE:464' Vz = V(3); */
    /* 'CE:465' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'CE:466' res = 2*Vz*sin(beta_rad)*sqrt(Vy^2+Vz^2); */
    /* dVxz^2 */
    /* 'CE:463' Vy = V(2); */
    /* 'CE:464' Vz = V(3); */
    /* 'CE:465' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'CE:466' res = 2*Vz*sin(beta_rad)*sqrt(Vy^2+Vz^2); */
    CDa_tmp = 2.0F * V[2] * sinf(atan2f(V[1], V[2])) * sqrtf(V[1] * V[1] + V[2] * V[2]);
    return 0.153125F * (((((-c_res_tmp * b_res_tmp * res_tmp_tmp - res_tmp *
      (57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
                        cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf
                        (0.1704F * x + 0.2682F)) + 0.00630986411F * cosf(0.2046F *
      x + 0.4082F)) + 7.67330408F * cosf(0.05419F * x + 3.305F)) * res_tmp_tmp)) -
      res_tmp * b_res_tmp * CDa_tmp) - res_tmp * b_CDa_tmp * res_tmp_tmp) +
                        c_res_tmp * (57.2957802F * (((((((-0.0016817F * sinf
      (c_CDa_tmp) + 0.000305185F * cosf(c_CDa_tmp)) - -0.0216611F * sinf(alpha)) +
      -0.000178354F * cosf(alpha)) - -0.00125876255F * sinf(dCDa_tmp)) +
      0.000419637741F * cosf(dCDa_tmp)) - -0.00378617F * sinf(alpha_rad)) +
      4.99887E-5F * cosf(alpha_rad)) * res_tmp_tmp)) + c_res_tmp * b_CDa_tmp *
                        CDa_tmp);
  }
  /* End of code generation (da_wing_z_dphii.c) */

  /*
  * function res = da_wing_z_dpsii(V)
  */
  float da_wing_z_dpsii(void)
  {
    float alpha_rad;
    float res_tmp;
    float x;
    float b_res_tmp;
    float res_tmp_tmp;
    float c_res_tmp;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;
    float dCDa_tmp;

    /* 'da_wing_z_dpsii:2' Vx = V(1); */
    /* 'da_wing_z_dpsii:3' Vz = V(3); */
    /* 'da_wing_z_dpsii:5' rho = 1.225; */
    /* 'da_wing_z_dpsii:6' S = 0.2; */
    /* 'da_wing_z_dpsii:7' m = 0.8; */
    /* 'da_wing_z_dpsii:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    /* 'da_wing_z_dpsii:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_z_dpsii:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_z_dpsii:14' res = rho*S/(2*m)* ... */
    /* 'da_wing_z_dpsii:15' 		(-cos(alpha_rad)* CE.Cl(alpha)          * CE.Vxz_2_dalphai_dpsii(V)... */
    /* 'da_wing_z_dpsii:16' 		 -sin(alpha_rad)* CE.Vxz_2_dCl_dpsii(V)                                               ... */
    /* 'da_wing_z_dpsii:17' 		 -sin(alpha_rad)* CE.Cl(alpha)          * CE.dVxz_2_dpsii(V)                             ... */
    /* 'da_wing_z_dpsii:18' 		 -sin(alpha_rad)* CE.Cd(alpha)          * CE.Vxz_2_dalphai_dpsii(V)... */
    /* 'da_wing_z_dpsii:19' 		 +cos(alpha_rad)* CE.Vxz_2_dCd_dpsii(V)                                                 ... */
    /* 'da_wing_z_dpsii:20' 		 +cos(alpha_rad)* CE.Cd(alpha)          * CE.dVxz_2_dpsii(V)                             ); */
    res_tmp = sinf(alpha_rad);

    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    b_res_tmp = ((((((1.233F * sinf(0.04068F * x + 0.09493F) + 141.1F * sinf
                      (0.05424F * x + 0.1637F)) + 0.1126F * sinf(0.101F * x +
      0.2612F)) + 0.08224F * sinf(0.136F * x + 0.2642F)) + 0.1197F * sinf(0.01427F
      * x + 0.6238F)) + 0.05624F * sinf(0.1704F * x + 0.2682F)) + 0.03084F * sinf
                (0.2046F * x + 0.4082F)) + 141.6F * sinf(0.05419F * x + 3.305F);

    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    res_tmp_tmp = V[1] * V[2];
    c_res_tmp = cosf(alpha_rad);

    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    alpha_rad = 4.0F * alpha * 0.01675F;
    CDa_tmp = (((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp))
                  + -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
                -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) +
              -0.05651F * cosf(alpha_rad)) + 0.0007461F * sinf(alpha_rad);

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    /* 'CE:474' Vx = V(1); */
    /* 'CE:475' Vy = V(2); */
    /* 'CE:476' gamma_rad = CE.AoD(V)*pi/180; */
    /*              res = 2*Vx*sin(gamma_rad)*sqrt(Vx^2+Vy^2); */
    /* 'CE:478' res = 2*Vx*Vy; */
    b_CDa_tmp = 2.0F * V[0] * V[1];

    /* 'CE:328' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'CE:329' res = CE.dCl(alpha) * CE.Vxz_2_dalphai_dpsii(V); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    /* 'CE:354' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'CE:355' res = CE.dCd(alpha) * CE.Vxz_2_dalphai_dpsii(V); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    alpha_rad = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:157' x = x - 360  */
      alpha_rad -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    c_CDa_tmp = alpha_rad * 0.01675F;
    alpha = 2.0F * alpha_rad * 0.01675F;
    dCDa_tmp = 3.0F * alpha_rad * 0.01675F;
    alpha_rad = 4.0F * alpha_rad * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    return 0.153125F * (((((-c_res_tmp * b_res_tmp * res_tmp_tmp - res_tmp *
      (57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
                        cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf
                        (0.1704F * x + 0.2682F)) + 0.00630986411F * cosf(0.2046F *
      x + 0.4082F)) + 7.67330408F * cosf(0.05419F * x + 3.305F)) * res_tmp_tmp)) -
      res_tmp * b_res_tmp * b_CDa_tmp) - res_tmp * CDa_tmp * res_tmp_tmp) +
                        c_res_tmp * (57.2957802F * (((((((-0.0016817F * sinf
      (c_CDa_tmp) + 0.000305185F * cosf(c_CDa_tmp)) - -0.0216611F * sinf(alpha)) +
      -0.000178354F * cosf(alpha)) - -0.00125876255F * sinf(dCDa_tmp)) +
      0.000419637741F * cosf(dCDa_tmp)) - -0.00378617F * sinf(alpha_rad)) +
      4.99887E-5F * cosf(alpha_rad)) * res_tmp_tmp)) + c_res_tmp * CDa_tmp *
                        b_CDa_tmp);
  }
  /* End of code generation (da_wing_z_dpsii.c) */

  /*
  * function res = da_wing_z_dthetai(V)
  */
  float da_wing_z_dthetai(void)
  {
    float Vxz;
    float alpha_rad;
    float res_tmp;
    float b_res_tmp;
    float x;
    float b_x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;
    float d_dCDa_tmp;
    float CDa_tmp;
    float b_CDa_tmp;
    float c_CDa_tmp;

    /* 'da_wing_z_dthetai:2' Vx = V(1); */
    /* 'da_wing_z_dthetai:3' Vz = V(3); */
    /* 'da_wing_z_dthetai:5' rho = 1.225; */
    /* 'da_wing_z_dthetai:6' S = 0.2; */
    /* 'da_wing_z_dthetai:7' m = 0.8; */
    /* 'da_wing_z_dthetai:9' Vxz = (Vx^2 + Vz^2)^0.5; */
    Vxz = sqrtf(V[0] * V[0] + V[2] * V[2]);

    /* 'da_wing_z_dthetai:11' alpha_rad = atan2(Vx,Vz); */
    alpha_rad = atan2f(V[0], V[2]);

    /* 'da_wing_z_dthetai:12' alpha = alpha_rad * 180 / pi; */
    alpha = alpha_rad * 180.0F / M_PI;

    /* 'da_wing_z_dthetai:14' res =  - rho*S/(2*m)* ... */
    /* 'da_wing_z_dthetai:15' 		( -sin(alpha_rad)* CE.dCl_dthetai(V)* Vxz^2                                                   ... */
    /* 'da_wing_z_dthetai:16' 		 -sin(alpha_rad)* CE.Cl(alpha)            * CE.dVxz_2_dthetai()                             ... */
    /* 'da_wing_z_dthetai:17' 		 +cos(alpha_rad)* CE.dCd_dthetai(V)* Vxz^2                                                   ... */
    /* 'da_wing_z_dthetai:18' 		 +cos(alpha_rad)* CE.Cd(alpha)            * CE.dVxz_2_dthetai()                             ); */
    res_tmp = sinf(alpha_rad);
    b_res_tmp = cosf(alpha_rad);

    /* 'CE:318' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'CE:319' res = CE.dCl(alpha) * CE.dalphai_dthetai(); */
    /* 'CE:344' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'CE:345' res = CE.dCd(alpha) * CE.dalphai_dthetai(); */
    alpha_rad = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    x = alpha_rad;

    /* 'CE:120' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x = alpha_rad - 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* Aerodynamic Coefficients */
    /* 'CE:39' x  =      alpha     ; */
    b_x = alpha;

    /* 'CE:40' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:41' x = x - 360  */
      b_x = alpha - 360.0F;
    }

    /* 'CE:43' a1 =      1.233     ; */
    /* 'CE:44' b1 =      0.04068   ; */
    /* 'CE:45' c1 =      0.09493   ; */
    /* 'CE:46' a2 =    141.1       ; */
    /* 'CE:47' b2 =      0.05424   ; */
    /* 'CE:48' c2 =      0.1637    ; */
    /* 'CE:49' a3 =      0.1126    ; */
    /* 'CE:50' b3 =      0.101     ; */
    /* 'CE:51' c3 =      0.2612    ; */
    /* 'CE:52' a4 =      0.08224   ; */
    /* 'CE:53' b4 =      0.136     ; */
    /* 'CE:54' c4 =      0.2642    ; */
    /* 'CE:55' a5 =      0.1197    ; */
    /* 'CE:56' b5 =      0.01427   ; */
    /* 'CE:57' c5 =      0.6238    ; */
    /* 'CE:58' a6 =      0.05624   ; */
    /* 'CE:59' b6 =      0.1704    ; */
    /* 'CE:60' c6 =      0.2682    ; */
    /* 'CE:61' a7 =      0.03084   ; */
    /* 'CE:62' b7 =      0.2046    ; */
    /* 'CE:63' c7 =      0.4082    ; */
    /* 'CE:64' a8 =    141.6       ; */
    /* 'CE:65' b8 =      0.05419   ; */
    /* 'CE:66' c8 =      3.305     ; */
    /* 'CE:67' CLa = a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + ... */
    /* 'CE:68'                  a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + ... */
    /* 'CE:69'                  a7*sin(b7*x+c7) + a8*sin(b8*x+c8); */
    /* 'CE:70' res =  CLa; */
    /*              CLa_y = [   0,      1,     1.2,      1,    0.6,    0.8,     0.85,      0,  -0.3, -0.3,  0];  %Lookup_y: C_L */
    /*              CLa_x = [   0,      6,      10,     20,     21,     30,       50,     90,   110,  150,   180];  %Lookup_x: alpha */
    /*              CLa_y = [CLa_y -fliplr(CLa_y(1:end-1))]; */
    /*              CLa_x = [CLa_x 360-fliplr(CLa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CLa = interp1(CLa_x,CLa_y,alpha); */
    /*              res =  sin(alpha*pi/180); */
    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (alpha_rad > 180.0F) {
      /* 'CE:157' x = x - 360  */
      alpha_rad -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = alpha_rad * 0.01675F;
    b_dCDa_tmp = 2.0F * alpha_rad * 0.01675F;
    c_dCDa_tmp = 3.0F * alpha_rad * 0.01675F;
    d_dCDa_tmp = 4.0F * alpha_rad * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    /* 'CE:84' x  =      alpha     ; */
    /* 'CE:85' if x > 180 */
    if (alpha > 180.0F) {
      /* 'CE:86' x = x - 360  */
      alpha -= 360.0F;
    }

    /* 'CE:88' a0 =      0.8371  ; */
    /* 'CE:89' a1 =      0.1004 ; */
    /* 'CE:90' b1 =     0.01822  ; */
    /* 'CE:91' a2 =     -0.6466  ; */
    /* 'CE:92' b2 =   -0.005324 ; */
    /* 'CE:93' a3 =    -0.02505 ; */
    /* 'CE:94' b3 =    0.008351 ; */
    /* 'CE:95' a4 =    -0.05651 ; */
    /* 'CE:96' b4 =   0.0007461  ; */
    /* 'CE:97' w =     0.01675 ; */
    /* 'CE:98' CDa =  a0 + ... */
    /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
    /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
    /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
    /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
    CDa_tmp = alpha * 0.01675F;
    b_CDa_tmp = 2.0F * alpha * 0.01675F;
    c_CDa_tmp = 3.0F * alpha * 0.01675F;
    alpha = 4.0F * alpha * 0.01675F;

    /* 'CE:103' res =  CDa; */
    /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
    /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
    /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
    /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
    /*               */
    /*              if alpha < 0 */
    /*                  alpha= 360; */
    /*              end */
    /*              CDa = interp1(CDa_x,CDa_y,alpha); */
    /*              res =  CDa; */
    /*              res =  1.02-cos(2*alpha*pi/180); */
    alpha_rad = Vxz * Vxz;
    return -0.153125F * (((-res_tmp * -(57.2957802F * (((((((0.0501584411F * cosf
      (0.04068F * x + 0.09493F) + 7.65326405F * cosf(0.05424F * x + 0.1637F)) +
      0.0113726F * cosf(0.101F * x + 0.2612F)) + 0.0111846402F * cosf(0.136F * x +
      0.2642F)) + 0.00170811894F * cosf(0.01427F * x + 0.6238F)) + 0.00958329625F *
      cosf(0.1704F * x + 0.2682F)) + 0.00630986411F * cosf(0.2046F * x + 0.4082F))
      + 7.67330408F * cosf(0.05419F * x + 3.305F))) * alpha_rad - res_tmp *
      (((((((1.233F * sinf(0.04068F * b_x + 0.09493F) + 141.1F * sinf(0.05424F *
      b_x + 0.1637F)) + 0.1126F * sinf(0.101F * b_x + 0.2612F)) + 0.08224F * sinf
          (0.136F * b_x + 0.2642F)) + 0.1197F * sinf(0.01427F * b_x + 0.6238F)) +
        0.05624F * sinf(0.1704F * b_x + 0.2682F)) + 0.03084F * sinf(0.2046F * b_x
      + 0.4082F)) + 141.6F * sinf(0.05419F * b_x + 3.305F)) * 0.0F) + b_res_tmp *
                          -(57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) +
      0.000305185F * cosf(dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) +
      -0.000178354F * cosf(b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) +
      0.000419637741F * cosf(c_dCDa_tmp)) - -0.00378617F * sinf(d_dCDa_tmp)) +
      4.99887E-5F * cosf(d_dCDa_tmp))) * alpha_rad) + b_res_tmp * ((((((((0.1004F *
      cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) + -0.6466F * cosf
      (b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) + -0.02505F * cosf(c_CDa_tmp))
      + 0.008351F * sinf(c_CDa_tmp)) + -0.05651F * cosf(alpha)) + 0.0007461F *
      sinf(alpha)) * 0.0F);
  }
  /* End of code generation (da_wing_z_dthetai.c) */

  /*
  * function res = dalphai_dphii(V)
  */
  float dalphai_dphii(void)
  {

    /* 'dalphai_dphii:2' Vx = V(1); */
    /* 'dalphai_dphii:3' Vy = V(2); */
    /* 'dalphai_dphii:4' Vz = V(3); */
    /* 'dalphai_dphii:5' res = Vx * Vy / (Vx^2 + Vz^2); */
    return V[0] * V[1] / (V[0] * V[0] + V[2] * V[2]);
  }
  /* End of code generation (dalphai_dphii.c) */

  /*
  * function res = dalphai_dpsii(V)
  */
  float dalphai_dpsii(void)
  {

    /* 'dalphai_dpsii:2' Vx = V(1); */
    /* 'dalphai_dpsii:3' Vy = V(2); */
    /* 'dalphai_dpsii:4' Vz = V(3); */
    /* 'dalphai_dpsii:5' res = Vy * Vz / (Vx^2 + Vz^2); */
    return V[1] * V[2] / (V[0] * V[0] + V[2] * V[2]);
  }
  /* End of code generation (dalphai_dpsii.c) */

  /*
  * function res = dalphai_dthetai
  */
  float dalphai_dthetai(void)
  {
    /* 'dalphai_dthetai:2' res = -1; */
    return -1.0;
  }
  /* End of code generation (dalphai_dthetai.c) */

  /*
  * function res = dbetai_dphii
  */
  float dbetai_dphii(void)
  { 
    /* 'dbetai_dphii:2' res = 1; */
    return 1.0;
  }
  /* End of code generation (dbetai_dphii.c) */

  /*
  * function res = dbetai_dpsii(V)
  */
  float dbetai_dpsii(void)
  {

    /* 'dbetai_dpsii:2' Vx = V(1); */
    /* 'dbetai_dpsii:3' Vy = V(2); */
    /* 'dbetai_dpsii:4' Vz = V(3); */
    /* 'dbetai_dpsii:5' res = - Vx * Vz / (Vy^2 + Vz^2); */
    return -V[0] * V[2] / (V[1] * V[1] + V[2] * V[2]);
  }
  /* End of code generation (dbetai_dpsii.c) */

  /*
  * function res = dbetai_dthetai(V)
  */
  float dbetai_dthetai(void)
  {

    /* 'dbetai_dthetai:2' Vx = V(1); */
    /* 'dbetai_dthetai:3' Vy = V(2); */
    /* 'dbetai_dthetai:4' Vz = V(3); */
    /* 'dbetai_dthetai:5' res = - Vx * Vy / (Vy^2 + Vz^2); */
    return -V[0] * V[1] / (V[1] * V[1] + V[2] * V[2]);
  }
  /* End of code generation (dbetai_dthetai.c) */

  /*
  * function res = dCd_dphii(V)
  */
  float dCd_dphii(void)
  {
    float x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'dCd_dphii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dCd_dphii:3' res = CE.dCd(alpha) * CE.dalphai_dphii(V); */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:157' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = x * 0.01675F;
    b_dCDa_tmp = 2.0F * x * 0.01675F;
    c_dCDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    /* dalpha */
    /* 'CE:360' Vx = V(1); */
    /* 'CE:361' Vy = V(2); */
    /* 'CE:362' Vz = V(3); */
    /* 'CE:363' res = Vx * Vy / (Vx^2 + Vz^2); */
    return 57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) + 0.000305185F * cosf
      (dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) + -0.000178354F * cosf
      (b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) + 0.000419637741F * cosf
      (c_dCDa_tmp)) - -0.00378617F * sinf(x)) + 4.99887E-5F * cosf(x)) * (V[0] *
      V[1] / (V[0] * V[0] + V[2] * V[2]));
  }
  /* End of code generation (dCd_dphii.c) */

  /*
  * function res = dCd_dpsii(V)
  */
  float dCd_dpsii(void)
  {
    float x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'dCd_dpsii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dCd_dpsii:3' res = CE.dCd(alpha) * CE.dalphai_dpsii(V); */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:157' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = x * 0.01675F;
    b_dCDa_tmp = 2.0F * x * 0.01675F;
    c_dCDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    /* 'CE:371' Vx = V(1); */
    /* 'CE:372' Vy = V(2); */
    /* 'CE:373' Vz = V(3); */
    /* 'CE:374' res = Vy * Vz / (Vx^2 + Vz^2); */
    return 57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) + 0.000305185F * cosf
      (dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) + -0.000178354F * cosf
      (b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) + 0.000419637741F * cosf
      (c_dCDa_tmp)) - -0.00378617F * sinf(x)) + 4.99887E-5F * cosf(x)) * (V[1] *
      V[2] / (V[0] * V[0] + V[2] * V[2]));
  }
  /* End of code generation (dCd_dpsii.c) */

  /*
  * function res = dCd_dthetai(V)
  */
  float dCd_dthetai(void)
  {
    float x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'dCd_dthetai:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dCd_dthetai:3' res = CE.dCd(alpha) * CE.dalphai_dthetai(); */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:157' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = x * 0.01675F;
    b_dCDa_tmp = 2.0F * x * 0.01675F;
    c_dCDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    return -(57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) + 0.000305185F *
      cosf(dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) + -0.000178354F * cosf
      (b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) + 0.000419637741F * cosf
                (c_dCDa_tmp)) - -0.00378617F * sinf(x)) + 4.99887E-5F * cosf(x)));
  }
  /* End of code generation (dCd_dthetai.c) */

  /*
  * function res = dCl_dphii(V)
  */
  float dCl_dphii(void)
  {
    float x;

    /* 'dCl_dphii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dCl_dphii:3' res = CE.dCl(alpha) * CE.dalphai_dphii(V); */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* dalpha */
    /* 'CE:360' Vx = V(1); */
    /* 'CE:361' Vy = V(2); */
    /* 'CE:362' Vz = V(3); */
    /* 'CE:363' res = Vx * Vy / (Vx^2 + Vz^2); */
    return 57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
      cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf(0.1704F * x + 0.2682F))
      + 0.00630986411F * cosf(0.2046F * x + 0.4082F)) + 7.67330408F * cosf
                          (0.05419F * x + 3.305F)) * (V[0] * V[1] / (V[0] * V[0] +
      V[2] * V[2]));
  }
  /* End of code generation (dCl_dphii.c) */

  /*
  * function res = dCl_dpsii(V)
  */
  float dCl_dpsii(void)
  {
    float x;

    /* 'dCl_dpsii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dCl_dpsii:3' res = CE.dCl(alpha) * CE.dalphai_dpsii(V); */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    /* 'CE:371' Vx = V(1); */
    /* 'CE:372' Vy = V(2); */
    /* 'CE:373' Vz = V(3); */
    /* 'CE:374' res = Vy * Vz / (Vx^2 + Vz^2); */
    return 57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
      cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf(0.1704F * x + 0.2682F))
      + 0.00630986411F * cosf(0.2046F * x + 0.4082F)) + 7.67330408F * cosf
                          (0.05419F * x + 3.305F)) * (V[1] * V[2] / (V[0] * V[0] +
      V[2] * V[2]));
  }
  /* End of code generation (dCl_dpsii.c) */

  /*
  * function res = dCl_dthetai(V)
  */
  float dCl_dthetai(void)
  {
    float x;

    /* 'dCl_dthetai:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dCl_dthetai:3' res = CE.dCl(alpha) * CE.dalphai_dthetai(); */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    return -(57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
      cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf(0.1704F * x + 0.2682F))
              + 0.00630986411F * cosf(0.2046F * x + 0.4082F)) + 7.67330408F *
              cosf(0.05419F * x + 3.305F)));
  }
  /* End of code generation (dCl_dthetai.c) */

  /*
  * function res = dVx_dphii
  */
  float dVx_dphii(void)
  {
    /* 'dVx_dphii:2' res = 0; */
    return 0.0;
  }
  /* End of code generation (dVx_dphii.c) */

  /*
  * function res = dVx_dpsii(V)
  */
  float dVx_dpsii(void)
  {
    float res;

    /* 'dVx_dpsii:2' Vx = V(1); */
    /* 'dVx_dpsii:3' Vy = V(2); */
    res = V[1];

    /* 'dVx_dpsii:4' gamma_rad = CE.AoD(V)*pi/180; */
    /*              res = sin(gamma_rad)*sqrt(Vx^2+Vy^2); */
    /* 'dVx_dpsii:6' res = Vy; */
    return res;
  }
  /* End of code generation (dVx_dpsii.c) */

  /*
  * function res = dVx_dthetai(V)
  */
  float dVx_dthetai(void)
  {
    /* 'dVx_dthetai:2' Vx = V(1); */
    /* 'dVx_dthetai:3' Vz = V(3); */
    /* 'dVx_dthetai:4' alpha_rad = CE.AoA(V)*pi/180; */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dVx_dthetai:5' res = cos(alpha_rad)*sqrt(Vx^2+Vz^2); */
    return cosf(atan2f(V[0], V[2])) * sqrtf(V[0] * V[0] + V[2] * V[2]);
  }
  /* End of code generation (dVx_dthetai.c) */

  /*
  * function res = dVxz_2_dphii(V)
  */
  float dVxz_2_dphii(void)
  {

    /* 'dVxz_2_dphii:2' Vy = V(2); */
    /* 'dVxz_2_dphii:3' Vz = V(3); */
    /* 'dVxz_2_dphii:4' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'dVxz_2_dphii:5' res = 2*Vz*sin(beta_rad)*sqrt(Vy^2+Vz^2); */
    return 2.0F * V[2] * sinf(atan2f(V[1], V[2])) * sqrtf(V[1] * V[1] + V[2] * V[2]);
  }
  /* End of code generation (dVxz_2_dphii.c) */

  /*
  * function res = dVxz_2_dpsii(V)
  */
  float dVxz_2_dpsii(void)
  {

    /* 'dVxz_2_dpsii:2' Vx = V(1); */
    /* 'dVxz_2_dpsii:3' Vy = V(2); */
    /* 'dVxz_2_dpsii:4' gamma_rad = CE.AoD(V)*pi/180; */
    /* 'dVxz_2_dpsii:5' res = 2*Vx*Vy; */
    return 2.0F * V[0] * V[1];
  }
  /* End of code generation (dVxz_2_dpsii.c) */

  /*
  * function res = dVxz_2_dthetai
  */
  float dVxz_2_dthetai(void)
  {

    /* 'dVxz_2_dthetai:2' res = 0; */
    return 0.0F;
  }
  /* End of code generation (dVxz_2_dthetai.c) */

  /*
  * function res = dVy_dphii(V)
  */
  float dVy_dphii(void)
  {

    /* 'dVy_dphii:2' Vy = V(2); */
    /* 'dVy_dphii:3' Vz = V(3); */
    /* 'dVy_dphii:4' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'dVy_dphii:5' res = -cos(beta_rad)*sqrt(Vy^2+Vz^2); */
    return -cosf(atan2f(V[1], V[2])) * sqrtf(V[1] * V[1] + V[2] * V[2]);
  }
  /* End of code generation (dVy_dphii.c) */

  /*
  * function res = dVy_dthetai
  */
  float dVy_dthetai(void)
  {

    /* 'dVy_dthetai:2' res = 0; */
    return 0.0F;
  }
  /* End of code generation (dVy_dthetai.c) */

  /*
  * function res = dVz_dphii(V)
  */
  float dVz_dphii(void)
  {

    /* 'dVz_dphii:2' Vy = V(2); */
    /* 'dVz_dphii:3' Vz = V(3); */
    /* 'dVz_dphii:4' beta_rad = CE.AoS(V)*pi/180; */
    /* 'CE:17' Vy = V(2); */
    /* 'CE:18' Vz = V(3); */
    /* 'CE:20' alpha_rad = atan2(Vy,Vz); */
    /* 'CE:21' alpha = alpha_rad * 180 / pi; */
    /* 'CE:23' res = alpha; */
    /* 'dVz_dphii:5' res = sin(beta_rad)*sqrt(Vy^2+Vz^2); */
    return sinf(atan2f(V[1], V[2])) * sqrtf(V[1] * V[1] + V[2] * V[2]);
  }
  /* End of code generation (dVz_dphii.c) */

  /*
  * function res = dVz_dpsii
  */
  float dVz_dpsii(void)
  {

    /* 'dVz_dpsii:2' res = 0; */
    return 0.0F;
  }
  /* End of code generation (dVz_dpsii.c) */

  /*
  * function res = dVz_dthetai(V)
  */
  float dVz_dthetai(void)
  {

    /* 'dVz_dthetai:2' Vx = V(1); */
    /* 'dVz_dthetai:3' Vz = V(3); */
    /* 'dVz_dthetai:4' alpha_rad = CE.AoA(V)*pi/180; */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'dVz_dthetai:5' res = -sin(alpha_rad)*sqrt(Vx^2+Vz^2); */
    return -sinf(atan2f(V[0], V[2])) * sqrtf(V[0] * V[0] + V[2] * V[2]);
  }
  /* End of code generation (dVz_dthetai.c) */

  /*
  * function res = Vxz_2_dalphai_dphii(V)
  */
  float Vxz_2_dalphai_dphii(void)
  {

    /* 'Vxz_2_dalphai_dphii:2' Vx = V(1); */
    /* 'Vxz_2_dalphai_dphii:3' Vy = V(2); */
    /* 'Vxz_2_dalphai_dphii:4' Vz = V(3); */
    /* 'Vxz_2_dalphai_dphii:5' res = Vx * Vy; */
    return V[0] * V[1];
  }
  /* End of code generation (Vxz_2_dalphai_dphii.c) */

  /*
  * function res = Vxz_2_dalphai_dpsii(V)
  */
  float Vxz_2_dalphai_dpsii(void)
  {

    /* 'Vxz_2_dalphai_dpsii:2' Vy = V(2); */
    /* 'Vxz_2_dalphai_dpsii:3' Vz = V(3); */
    /* 'Vxz_2_dalphai_dpsii:4' res = Vy * Vz; */
    return V[1] * V[2];
  }
  /* End of code generation (Vxz_2_dalphai_dpsii.c) */

  /*
  * function res = Vxz_2_dCd_dphii(V)
  */
  float Vxz_2_dCd_dphii(void)
  {
    float x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'Vxz_2_dCd_dphii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'Vxz_2_dCd_dphii:3' res = CE.dCd(alpha) * CE.Vxz_2_dalphai_dphii(V); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:157' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = x * 0.01675F;
    b_dCDa_tmp = 2.0F * x * 0.01675F;
    c_dCDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    return 57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) + 0.000305185F * cosf
      (dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) + -0.000178354F * cosf
      (b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) + 0.000419637741F * cosf
      (c_dCDa_tmp)) - -0.00378617F * sinf(x)) + 4.99887E-5F * cosf(x)) * (V[0] *
      V[1]);
  }
  /* End of code generation (Vxz_2_dCd_dphii.c) */

  /*
  * function res = Vxz_2_dCd_dpsii(V)
  */
  float Vxz_2_dCd_dpsii(void)
  {
    float x;
    float dCDa_tmp;
    float b_dCDa_tmp;
    float c_dCDa_tmp;

    /* 'Vxz_2_dCd_dpsii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'Vxz_2_dCd_dpsii:3' res = CE.dCd(alpha) * CE.Vxz_2_dalphai_dpsii(V); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:155' x  =      alpha   ; */
    /* 'CE:156' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:157' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:159' a1 =      0.1004  ; */
    /* 'CE:160' b1 =     0.01822  ; */
    /* 'CE:161' a2 =     -0.6466  ; */
    /* 'CE:162' b2 =   -0.005324 ; */
    /* 'CE:163' a3 =    -0.02505 ; */
    /* 'CE:164' b3 =    0.008351 ; */
    /* 'CE:165' a4 =    -0.05651 ; */
    /* 'CE:166' b4 =   0.0007461  ; */
    /* 'CE:167' w =     0.01675 ; */
    /* 'CE:168' dCDa = -1*w*a1*sin(1*x*w) + 1*b1*w*cos(1*x*w) - ... */
    /* 'CE:169'                    2*w*a2*sin(2*x*w) + 2*b2*w*cos(2*x*w) - ... */
    /* 'CE:170'                    3*w*a3*sin(3*x*w) + 3*b3*w*cos(3*x*w) - ... */
    /* 'CE:171'                    4*w*a4*sin(4*x*w) + 4*b4*w*cos(4*x*w); */
    dCDa_tmp = x * 0.01675F;
    b_dCDa_tmp = 2.0F * x * 0.01675F;
    c_dCDa_tmp = 3.0F * x * 0.01675F;
    x = 4.0F * x * 0.01675F;

    /* 'CE:172' res =  180/pi*dCDa; */
    /*              res =  2*sin(2*alpha*pi/180); */
    return 57.2957802F * (((((((-0.0016817F * sinf(dCDa_tmp) + 0.000305185F * cosf
      (dCDa_tmp)) - -0.0216611F * sinf(b_dCDa_tmp)) + -0.000178354F * cosf
      (b_dCDa_tmp)) - -0.00125876255F * sinf(c_dCDa_tmp)) + 0.000419637741F * cosf
      (c_dCDa_tmp)) - -0.00378617F * sinf(x)) + 4.99887E-5F * cosf(x)) * (V[1] *
      V[2]);
  }
  /* End of code generation (Vxz_2_dCd_dpsii.c) */

  /*
  * function res = Vxz_2_dCl_dphii(V)
  */
  float Vxz_2_dCl_dphii(void)
  {
    float x;

    /* 'Vxz_2_dCl_dphii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'Vxz_2_dCl_dphii:3' res = CE.dCl(alpha) * CE.Vxz_2_dalphai_dphii(V); */
    /* 'CE:378' Vx = V(1); */
    /* 'CE:379' Vy = V(2); */
    /* 'CE:380' Vz = V(3); */
    /* 'CE:381' res = Vx * Vy; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    return 57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
      cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf(0.1704F * x + 0.2682F))
      + 0.00630986411F * cosf(0.2046F * x + 0.4082F)) + 7.67330408F * cosf
                          (0.05419F * x + 3.305F)) * (V[0] * V[1]);
  }
  /* End of code generation (Vxz_2_dCl_dphii.c) */

  /*
  * function res = Vxz_2_dCl_dpsii(V)
  */
  float Vxz_2_dCl_dpsii(void)
  {
    float x;

    /* 'Vxz_2_dCl_dpsii:2' alpha = CE.AoA(V); */
    /*         %% Basic parameters */
    /* Aerodynamic angles */
    /* 'CE:7' Vx = V(1); */
    /* 'CE:8' Vz = V(3); */
    /* 'CE:10' alpha_rad = atan2(Vx,Vz); */
    /* 'CE:11' alpha = alpha_rad * 180 / pi; */
    /* 'CE:13' res = alpha; */
    /* 'Vxz_2_dCl_dpsii:3' res = CE.dCl(alpha) * CE.Vxz_2_dalphai_dpsii(V); */
    /* 'CE:385' Vy = V(2); */
    /* 'CE:386' Vz = V(3); */
    /* 'CE:387' res = Vy * Vz; */
    x = atan2f(V[0], V[2]) * 180.0F / M_PI;

    /* 'CE:119' x  =      alpha     ; */
    /* 'CE:120' if x > 180 */
    if (x > 180.0F) {
      /* 'CE:121' x = x - 360  */
      x -= 360.0F;
    }

    /* 'CE:123' a1 =      1.233     ; */
    /* 'CE:124' b1 =      0.04068   ; */
    /* 'CE:125' c1 =      0.09493   ; */
    /* 'CE:126' a2 =    141.1       ; */
    /* 'CE:127' b2 =      0.05424   ; */
    /* 'CE:128' c2 =      0.1637    ; */
    /* 'CE:129' a3 =      0.1126    ; */
    /* 'CE:130' b3 =      0.101     ; */
    /* 'CE:131' c3 =      0.2612    ; */
    /* 'CE:132' a4 =      0.08224   ; */
    /* 'CE:133' b4 =      0.136     ; */
    /* 'CE:134' c4 =      0.2642    ; */
    /* 'CE:135' a5 =      0.1197    ; */
    /* 'CE:136' b5 =      0.01427   ; */
    /* 'CE:137' c5 =      0.6238    ; */
    /* 'CE:138' a6 =      0.05624   ; */
    /* 'CE:139' b6 =      0.1704    ; */
    /* 'CE:140' c6 =      0.2682    ; */
    /* 'CE:141' a7 =      0.03084   ; */
    /* 'CE:142' b7 =      0.2046    ; */
    /* 'CE:143' c7 =      0.4082    ; */
    /* 'CE:144' a8 =    141.6       ; */
    /* 'CE:145' b8 =      0.05419   ; */
    /* 'CE:146' c8 =      3.305     ; */
    /* 'CE:147' dCLa = b1*a1*cos(b1*x+c1) + b2*a2*cos(b2*x+c2) + b3*a3*cos(b3*x+c3) + ... */
    /* 'CE:148'                  b4*a4*cos(b4*x+c4) + b5*a5*cos(b5*x+c5) + b6*a6*cos(b6*x+c6) + ... */
    /* 'CE:149'                  b7*a7*cos(b7*x+c7) + b8*a8*cos(b8*x+c8); */
    /* 'CE:150' res =  180/pi*dCLa; */
    /*              res = cos(alpha*pi/180); */
    return 57.2957802F * (((((((0.0501584411F * cosf(0.04068F * x + 0.09493F) +
      7.65326405F * cosf(0.05424F * x + 0.1637F)) + 0.0113726F * cosf(0.101F * x +
      0.2612F)) + 0.0111846402F * cosf(0.136F * x + 0.2642F)) + 0.00170811894F *
      cosf(0.01427F * x + 0.6238F)) + 0.00958329625F * cosf(0.1704F * x + 0.2682F))
      + 0.00630986411F * cosf(0.2046F * x + 0.4082F)) + 7.67330408F * cosf
                          (0.05419F * x + 3.305F)) * (V[1] * V[2]);
  }
  /* End of code generation (Vxz_2_dCl_dpsii.c) */

  void test_a_L_wing(void)
  {
  // printf("\nRunning test_a_L_wing func in guidance_aerobatic\n\n");
  int Vinf;
  int theta;
  int psi;

  //  float temp_res[3];

  Vinf = 10;
  theta = 4;
  psi = 60;

  // printf("Vinf\tphi\ttheta\tpsi\tVx\tVy\tVz\ta_wing_x\ta_wing_z\ta_L_wing_x\ta_L_wing_z\ta_D_wing_x\ta_D_wing_z\tAoA\tAoS\tAoD\tCl\tCd\tdCl\tdCd\tVxz_2\t");

  // printf("da_wing_x_dphii\tda_wing_x_dpsii\tda_wing_x_dthetai\tda_wing_y_dphii\tda_wing_y_dpsii\tda_wing_y_dthetai\tda_wing_z_dphii\tda_wing_z_dpsii\tda_wing_z_dthetai\t");

  // printf("dalphai_dphii\tdalphai_dpsii\tdalphai_dthetai\tdbetai_dphii\tdbetai_dpsii\tdbetai_dthetai\tdCd_dphii\tdCd_dpsii\tdCd_dthetai\tdCl_dphii\tdCl_dpsii\tdCl_dthetai\t");

  // printf("dVx_dphii\tdVx_dpsii\tdVx_dthetai\tdVxz_2_dphii\tdVxz_2_dpsii\tdVxz_2_dthetai\tdVy_dphii\tdVy_dthetai\tdVz_dphii\tdVz_dpsii\tdVz_dthetai\tVxz_2_dalphai_dphii\tVxz_2_dalphai_dpsii\t");

  // printf("Vxz_2_dCd_dphii\tVxz_2_dCd_dpsii\tVxz_2_dCl_dphii\tVxz_2_dCl_dpsii\n");

  for (Vinf=0;Vinf<10;Vinf++)
  {
      for (theta = -90; theta <= 90; theta = theta + 10)
      {
        for (psi = -180; psi < 180; psi = psi + 10)
        {
          V[0] = (float)Vinf*cosf((float)psi/180.0f*M_PI)*sinf((float)theta/180.0f*M_PI);
          V[1] = (float)Vinf*cosf((float)psi/180.0f*M_PI)*cosf((float)theta/180.0f*M_PI);
          V[2] = (float)Vinf*sinf((float)psi/180.0f*M_PI);


        
          // printf("%i"  ,Vinf );
          
          // printf("\t%.4f",phi  );
          
          // printf("\t%i"  ,theta);
          // printf("\t%i"  ,psi  );
          
          // printf("\t%.4f",V[0]  );
          // printf("\t%.4f",V[1]  );
          // printf("\t%.4f",V[2]  );

          // printf("\t%.4f",a_wing_x());
          // printf("\t%.4f",a_wing_z());

          // printf("\t%.4f",a_L_wing_x());
          // printf("\t%.4f",a_L_wing_z());

          // printf("\t%.4f",a_D_wing_x());
          // printf("\t%.4f",a_D_wing_z());

          // printf("\t%.4f",AoA());
          // printf("\t%.4f",AoS());
          // printf("\t%.4f",AoD());

          // printf("\t%.4f",Cl());
          // printf("\t%.4f",Cd());
          // printf("\t%.4f",dCl());
          // printf("\t%.4f",dCd());

          // printf("\t%.4f",Vxz_2());

          // printf("\t%.4f",da_wing_x_dphii());
          // printf("\t%.4f",da_wing_x_dpsii());
          // printf("\t%.4f",da_wing_x_dthetai());


          // printf("\t%.4f",da_wing_y_dphii());
          // printf("\t%.4f",da_wing_y_dpsii());
          // printf("\t%.4f",da_wing_y_dthetai());


          // printf("\t%.4f",da_wing_z_dphii());
          // printf("\t%.4f",da_wing_z_dpsii());
          // printf("\t%.4f",da_wing_z_dthetai());

          // printf("\t%.4f",dalphai_dphii());
          // printf("\t%.4f",dalphai_dpsii());
          // printf("\t%.4f",dalphai_dthetai());

          // printf("\t%.4f",dbetai_dphii());
          // printf("\t%.4f",dbetai_dpsii());
          // printf("\t%.4f",dbetai_dthetai());

          // printf("\t%.4f",dCd_dphii());
          // printf("\t%.4f",dCd_dpsii());
          // printf("\t%.4f",dCd_dthetai());
          
          // printf("\t%.4f",dCl_dphii());
          // printf("\t%.4f",dCl_dpsii());
          // printf("\t%.4f",dCl_dthetai());

          // printf("\t%.4f",dVx_dphii());
          // printf("\t%.4f",dVx_dpsii());
          // printf("\t%.4f",dVx_dthetai());

          // printf("\t%.4f",dVxz_2_dphii());
          // printf("\t%.4f",dVxz_2_dpsii());
          // printf("\t%.4f",dVxz_2_dthetai());

          // printf("\t%.4f",dVy_dphii());
          // printf("\t%.4f",dVy_dthetai());
          // printf("\t%.4f",dVz_dphii());

          // printf("\t%.4f",dVz_dpsii());
          // printf("\t%.4f",dVz_dthetai());
          // printf("\t%.4f",Vxz_2_dalphai_dphii());

          // printf("\t%.4f",Vxz_2_dalphai_dpsii());
          // printf("\t%.4f",Vxz_2_dCd_dphii());
          // printf("\t%.4f",Vxz_2_dCd_dpsii());


          // printf("\t%.4f",Vxz_2_dCl_dphii());
          // printf("\t%.4f",Vxz_2_dCl_dpsii());

          // printf("\n");


        }
      }

  }
  // printf("\n");
  }
//