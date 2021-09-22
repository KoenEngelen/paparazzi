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


float guidance_aerobatic_indi_gain = 0.5;
float debug_value;

/*
#include "subsystems/datalink/telemetry.h"

 #define PPRZ_MSG_ID_AEROBATIC_INDI 158

 static void send_aerobatic_indi(struct transport_tx *trans, struct link_device *dev)
 {
   pprz_msg_send_AEROBATIC_INDI(trans, dev, AC_ID,
                         &debug_value, 
                         &guidance_aerobatic_indi_gain);
 }
*/

 void guidance_aerobatic_init(void) {

   debug_value = 0.0;


// register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_INDI, send_aerobatic_indi);]
 }


 void guidance_aerobatic_run(void) {
   debug_value += 0.00195;

   if (debug_value > 60){
     debug_value = 0;
   }

 }

// CELib

/*
 * function res = a_D_wing_x(V)
 */
float a_D_wing_x(const float V[3])
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
  /* 'a_D_wing_x:7' m = 0.8; */
  /* 'a_D_wing_x:9' Vxz = (Vx^2 + Vz^2)^0.5; */
  a = sqrtf(V[0] * V[0] + V[2] * V[2]);

  /* 'a_D_wing_x:11' alpha_rad = atan2(Vx,Vz); */
  alpha_rad = atan2f(V[0], V[2]);

  /* 'a_D_wing_x:12' alpha = alpha_rad * 180 / pi; */
  /* 'a_D_wing_x:14' res = rho*S/(2*m)* ... */
  /* 'a_D_wing_x:15'                 ( sin(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
  x = alpha_rad * 180.0F / 3.14159274F;

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
float a_D_wing_z(const float V[3], float phi)
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
  phi_rad = -phi * 3.14159274F / 180.0F;

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
  phi_rad = alpha_rad * 180.0F / 3.14159274F;

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
  return 0.153125F * cosf(phi * 3.14159274F / 180.0F) * (cosf(alpha_rad) *
    ((((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) +
          -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
        -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) + -0.05651F *
      cosf(phi_rad)) + 0.0007461F * sinf(phi_rad)) * (R_tmp * R_tmp));
}
/* End of code generation (a_D_wing_z.c) */

/*
 * function res = a_L_wing_x(V)
 */
float a_L_wing_x(const float V[3])
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
  x = alpha_rad * 180.0F / 3.14159274F;

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
float a_L_wing_z(const float V[3], float phi)
{
  float phi_rad;
  float alpha_rad;
  float fv[9];
  int i;
  float fv1[3];
  float x;

  /* 'a_L_wing_z:2' phi_rad = -phi * pi/180.0; */
  phi_rad = -phi * 3.14159274F / 180.0F;

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
  x = alpha_rad * 180.0F / 3.14159274F;

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
  return 0.153125F * cosf(phi * 3.14159274F / 180.0F) * (-sinf(alpha_rad) *
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
float a_wing_x(const float V[3])
{
  float Vxz;
  float alpha_rad;
  float alpha;
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
  alpha = alpha_rad * 180.0F / 3.14159274F;

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
float a_wing_z(const float V[3], float phi)
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
  phi_rad = -phi * 3.14159274F / 180.0F;

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
  phi_rad = alpha_rad * 180.0F / 3.14159274F;

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
  return 0.153125F * cosf(phi * 3.14159274F / 180.0F) * (-sinf(alpha_rad) *
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
float AoA(const float V[3])
{
  float res;

  /* 'AoA:2' Vx = V(1); */
  /* 'AoA:3' Vz = V(3); */
  /* 'AoA:5' alpha_rad = atan2(Vx,Vz); */
  /* 'AoA:6' alpha = alpha_rad * 180 / pi; */
  res = atan2f(V[0], V[2]) * 180.0F / 3.14159274F;

  /* 'AoA:8' res = alpha; */
  return res;
}
/* End of code generation (AoA.c) */

/*
 * function res = AoD(V)
 */
float AoD(const float V[3])
{
  float res;

  /* 'AoD:2' Vx = V(1); */
  /* 'AoD:3' Vy = V(2); */
  /* sign of gamma not sure yet, remove comment if checked */
  /* 'AoD:6' gamma_rad = atan2(Vx,Vy); */
  /* 'AoD:7' gamma = gamma_rad * 180 / pi; */
  res = atan2f(V[0], V[1]) * 180.0F / 3.14159274F;

  /* 'AoD:9' res = gamma; */
  return res;
}
/* End of code generation (AoD.c) */

/*
 * function res = AoS(V)
 */
float AoS(const float V[3])
{
  float res;

  /* 'AoS:2' Vy = V(2); */
  /* 'AoS:3' Vz = V(3); */
  /* 'AoS:5' alpha_rad = atan2(Vy,Vz); */
  /* 'AoS:6' alpha = alpha_rad * 180 / pi; */
  res = atan2f(V[1], V[2]) * 180.0F / 3.14159274F;

  /* 'AoS:8' res = alpha; */
  return res;
}
/* End of code generation (AoS.c) */

/*
 * function res = Cd(alpha)
 */
float Cd(float alpha)
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
float Cl(float alpha)
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
float dCd(float alpha)
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
float dCl(float alpha)
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
float Vxz_2(const float V[3])
{
  /* 'Vxz_2:2' Vx = V(1); */
  /* 'Vxz_2:3' Vz = V(3); */
  /* 'Vxz_2:4' res = Vx^2+Vz^2; */
  return V[0] * V[0] + V[2] * V[2];
}
/* End of code generation (Vxz_2.c) */


// QLib

/*
 * function VectorParallel = getParallelComponent(VectorA,VectorB)
 */
void getParallelComponent(const float VectorA[3], const float VectorB[3], float VectorParallel[3])
{
  float scale;
  float absxk;
  float t;
  float a;
  float b;

  /* 'getParallelComponent:2' VectorParallel = VectorB*dot(VectorA,VectorB)/norm(VectorB)^2; */
  scale = 1.29246971E-26F;
  absxk = fabsf(VectorB[0]);
  if (absxk > 1.29246971E-26F) {
    a = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    a = t * t;
  }

  absxk = fabsf(VectorB[1]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  b = (VectorA[0] * VectorB[0] + VectorA[1] * VectorB[1]) + VectorA[2] *
    VectorB[2];
  absxk = fabsf(VectorB[2]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  a = scale * sqrtf(a);
  scale = a * a;
  VectorParallel[0] = VectorB[0] * b / scale;
  VectorParallel[1] = VectorB[1] * b / scale;
  VectorParallel[2] = VectorB[2] * b / scale;
}
/* End of code generation (getParallelComponent.c) */

/*
 * function VectorPerpendicular = getPerpendicularComponent(VectorA,VectorB)
 */
void getPerpendicularComponent(const float VectorA[3], const float VectorB[3], float VectorPerpendicular[3])
{
  float scale;
  float absxk;
  float t;
  float a;
  float b;

  /* 'getPerpendicularComponent:2' VectorPerpendicular = VectorA - QLib.getParallelComponent(VectorA,VectorB); */
  /* 'QLib:40' VectorParallel = VectorB*dot(VectorA,VectorB)/norm(VectorB)^2; */
  scale = 1.29246971E-26F;
  absxk = fabsf(VectorB[0]);
  if (absxk > 1.29246971E-26F) {
    a = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    a = t * t;
  }

  absxk = fabsf(VectorB[1]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  b = (VectorA[0] * VectorB[0] + VectorA[1] * VectorB[1]) + VectorA[2] *
    VectorB[2];
  absxk = fabsf(VectorB[2]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  a = scale * sqrtf(a);
  scale = a * a;
  VectorPerpendicular[0] = VectorA[0] - VectorB[0] * b / scale;
  VectorPerpendicular[1] = VectorA[1] - VectorB[1] * b / scale;
  VectorPerpendicular[2] = VectorA[2] - VectorB[2] * b / scale;
}
/* End of code generation (getPerpendicularComponent.c) */

/*
 * function q = getQuaternion(Angle,Vector)
 */
void getQuaternion(float Angle, const float Vector[3], float q[4]) /* replace with *q if not working in paparazzi*/
{
  float s_tmp;
  float a;
  float scale;
  float absxk;
  float t;
  float y;

  /* 'getQuaternion:2' s = cos(Angle/2); */
  s_tmp = Angle / 2.0F;

  /* 'getQuaternion:3' v = sin(Angle/2)*Vector/norm(Vector); */
  a = sinf(s_tmp);
  scale = 1.29246971E-26F;
  absxk = fabsf(Vector[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(Vector[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(Vector[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrtf(y);

  /* 'getQuaternion:4' q = [s;v]; */
  q[0] = cosf(s_tmp);
  q[1] = a * Vector[0] / y;
  q[2] = a * Vector[1] / y;
  q[3] = a * Vector[2] / y;
}
/* End of code generation (getQuaternion.c) */

/*
 * function qtot = getQuaternionAllignAxes(AxisA,AxisB,AxisAd,AxisBd)
 */
void getQuaternionAllignAxes(const float AxisA[3], const float AxisB[3], const float AxisAd[3], const float AxisBd[3], float qtot[4])
{
  float BperA[3];
  float scale;
  float absxk;
  float t;
  float y;
  float eAd[3];
  float b_y;
  float q1[4];
  float B1[3];
  float q2[4];

  /* 'getQuaternionAllignAxes:3' BperA = getPerpendicularComponent(AxisBd,AxisAd); */
  getPerpendicularComponent(AxisBd, AxisAd, BperA);

  /* 'getQuaternionAllignAxes:5' eAd = AxisAd/norm(AxisAd); */
  scale = 1.29246971E-26F;
  absxk = fabsf(AxisAd[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(AxisAd[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(AxisAd[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrtf(y);

  /* 'getQuaternionAllignAxes:6' eBd = BperA/norm(BperA); */
  scale = 1.29246971E-26F;
  eAd[0] = AxisAd[0] / y;
  absxk = fabsf(BperA[0]);
  if (absxk > 1.29246971E-26F) {
    b_y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    b_y = t * t;
  }

  eAd[1] = AxisAd[1] / y;
  absxk = fabsf(BperA[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  eAd[2] = AxisAd[2] / y;
  absxk = fabsf(BperA[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  b_y = scale * sqrtf(b_y);
  BperA[0] /= b_y;
  BperA[1] /= b_y;
  BperA[2] /= b_y;

  /* 'getQuaternionAllignAxes:7' eCd = cross(eAd,eBd); */
  /* 'getQuaternionAllignAxes:10' q1 = getQuaternionDifference(AxisA,AxisAd); */
  getQuaternionDifference(AxisA, AxisAd, q1);

  /* 'getQuaternionAllignAxes:11' B1 = QuaternionRotate(AxisB,q1); */
  QuaternionRotate(AxisB, q1, B1);

  /* 'getQuaternionAllignAxes:13' ex = -eBd; */
  /* 'getQuaternionAllignAxes:14' ey = cross(eAd,-eBd); */
  scale = -BperA[0];
  BperA[0] = -BperA[0];
  absxk = -BperA[1];
  BperA[1] = -BperA[1];
  t = -BperA[2];
  BperA[2] = -BperA[2];

  /* 'getQuaternionAllignAxes:15' ez = eAd; */
  /* 'getQuaternionAllignAxes:17' xex = dot(B1,ex); */
  /* 'getQuaternionAllignAxes:18' xey = dot(B1,ey); */
  /* 'getQuaternionAllignAxes:20' theta = atan2(xey,xex); */
  /* 'getQuaternionAllignAxes:22' q2 = getQuaternion(theta,ez); */
  getQuaternion(atan2f((B1[0] * (eAd[1] * BperA[2] - eAd[2] * BperA[1]) +
    B1[1] * (eAd[2] * BperA[0] - eAd[0] * BperA[2])) + B1[2] * (eAd[0] * BperA[1]
    - eAd[1] * BperA[0]), (B1[0] * scale + B1[1] * absxk) + B1[2] * t), eAd, q2);

  /* 'getQuaternionAllignAxes:24' qtot = getQuaternionProduct(q2,q1); */
  getQuaternionProduct(q2, q1, qtot);

  /* 'getQuaternionAllignAxes:27' AxisC = cross(AxisA,AxisB); */
  /* 'getQuaternionAllignAxes:28' eA = QuaternionRotate(AxisA,qtot)/norm(AxisA); */
  /* 'getQuaternionAllignAxes:29' eB = QuaternionRotate(AxisB,qtot)/norm(AxisB); */
  /* 'getQuaternionAllignAxes:30' eC = QuaternionRotate(AxisC,qtot)/norm(AxisC); */
  /*              disp([dot(eA,eAd);dot(eB,eBd);dot(eC,eCd)]); */
}
/* End of code generation (getQuaternionAllignAxes.c) */

/*
 * function alpha = getQuaternionAngle(q)
 */
float getQuaternionAngle(const float q[4])
{
  /* 'getQuaternionAngle:2' alpha = 2*atan2( (q(2)^2+q(3)^2+q(4)^2)^0.5, q(1) ); */
  return 2.0F * atan2f(sqrtf((q[1] * q[1] + q[2] * q[2]) + q[3] * q[3]),
    q[0]);
}
/* End of code generation (getQuaternionAngle.c) */

/*
 * function alpha = getQuaternionAngleDegrees(q)
 */
float getQuaternionAngleDegrees(const float q[4])
{
  float a;

  /* 'getQuaternionAngleDegrees:2' alpha = getQuaternionAngle(q)*180/pi; */
  a = getQuaternionAngle(q);
  return a * 180.0F / 3.14159274F;
}
/* End of code generation (getQuaternionAngleDegrees.c) */

/*
 * function q = getQuaternionDifference(VectorA, VectorB)
 */
void getQuaternionDifference(const float VectorA[3], const float VectorB[3], float q[4])
{
  float scale;
  float absxk;
  float t;
  float c1_tmp;
  float b_c1_tmp;
  float y;
  
  /* 'getQuaternionDifference:2' c1 = norm(VectorA)*norm(VectorB)+dot(VectorA.',VectorB); */
  scale = 1.29246971E-26F;
  absxk = fabsf(VectorB[0]);
  if (absxk > 1.29246971E-26F) {
    c1_tmp = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    c1_tmp = t * t;
  }

  absxk = fabsf(VectorB[1]);
  if (absxk > scale) {
    t = scale / absxk;
    c1_tmp = c1_tmp * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    c1_tmp += t * t;
  }

  absxk = fabsf(VectorB[2]);
  if (absxk > scale) {
    t = scale / absxk;
    c1_tmp = c1_tmp * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    c1_tmp += t * t;
  }

  c1_tmp = scale * sqrtf(c1_tmp);
  scale = 1.29246971E-26F;
  absxk = fabsf(VectorA[0]);
  if (absxk > 1.29246971E-26F) {
    b_c1_tmp = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    b_c1_tmp = t * t;
  }

  absxk = fabsf(VectorA[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_c1_tmp = b_c1_tmp * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_c1_tmp += t * t;
  }

  absxk = fabsf(VectorA[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_c1_tmp = b_c1_tmp * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_c1_tmp += t * t;
  }

  b_c1_tmp = scale * sqrtf(b_c1_tmp);

  /* 'getQuaternionDifference:3' c2 = cross(VectorA,VectorB); */
  /* 'getQuaternionDifference:4' c3 = norm(norm(VectorB)*VectorA + norm(VectorA)*VectorB); */
  scale = 1.29246971E-26F;
  absxk = fabsf(c1_tmp * VectorA[0] + b_c1_tmp * VectorB[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(c1_tmp * VectorA[1] + b_c1_tmp * VectorB[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(c1_tmp * VectorA[2] + b_c1_tmp * VectorB[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrtf(y);

  /* 'getQuaternionDifference:6' q = [c1 ;c2] / c3; */
  q[0] = (b_c1_tmp * c1_tmp + ((VectorA[0] * VectorB[0] + VectorA[1] * VectorB[1])
           + VectorA[2] * VectorB[2])) / y;
  q[1] = (VectorA[1] * VectorB[2] - VectorA[2] * VectorB[1]) / y;
  q[2] = (VectorA[2] * VectorB[0] - VectorA[0] * VectorB[2]) / y;
  q[3] = (VectorA[0] * VectorB[1] - VectorA[1] * VectorB[0]) / y;
}
/* End of code generation (getQuaternionDifference.c) */

/*
 * function qi = getQuaternionInverse(q)
 */
void getQuaternionInverse(const float q[4], float qi[4])
{
  float scale;
  float absxk;
  float t;
  float y;

  /* 'getQuaternionInverse:2' s = [q(1);0;0;0]; */
  /* 'getQuaternionInverse:3' v = [0;q(2:4)]; */
  /* 'getQuaternionInverse:5' qi = (s-v)/norm(s+v); */
  scale = 1.29246971E-26F;
  absxk = fabsf(q[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(q[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(q[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(q[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrtf(y);
  qi[0] = q[0] / y;
  qi[1] = (0.0F - q[1]) / y;
  qi[2] = (0.0F - q[2]) / y;
  qi[3] = (0.0F - q[3]) / y;
}
/* End of code generation (getQuaternionInverse.c) */

/*
 * function q = getQuaternionProduct(Q1,Q2)
 */
void getQuaternionProduct(const float Q1[4], const float Q2[4], float q[4])
{

  /* 'getQuaternionProduct:2' q = [Q1(1)*Q2(1)-Q1(2)*Q2(2)-Q1(3)*Q2(3)-Q1(4)*Q2(4) */
  /* 'getQuaternionProduct:3'                  Q1(1)*Q2(2)+Q1(2)*Q2(1)+Q1(3)*Q2(4)-Q1(4)*Q2(3) */
  /* 'getQuaternionProduct:4'                  Q1(1)*Q2(3)-Q1(2)*Q2(4)+Q1(3)*Q2(1)+Q1(4)*Q2(2) */
  /* 'getQuaternionProduct:5'                  Q1(1)*Q2(4)+Q1(2)*Q2(3)-Q1(3)*Q2(2)+Q1(4)*Q2(1)]; */
  q[0] = ((Q1[0] * Q2[0] - Q1[1] * Q2[1]) - Q1[2] * Q2[2]) - Q1[3] * Q2[3];
  q[1] = ((Q1[0] * Q2[1] + Q1[1] * Q2[0]) + Q1[2] * Q2[3]) - Q1[3] * Q2[2];
  q[2] = ((Q1[0] * Q2[2] - Q1[1] * Q2[3]) + Q1[2] * Q2[0]) + Q1[3] * Q2[1];
  q[3] = ((Q1[0] * Q2[3] + Q1[1] * Q2[2]) - Q1[2] * Q2[1]) + Q1[3] * Q2[0];

  /*              disp(['q1:    ' num2str(Q1.')]) */
  /*              disp(['q2:    ' num2str(Q2.')]) */
  /*              disp(['q1*q2: ' num2str( q.')]) */
}
/* End of code generation (getQuaternionProduct.c) */

/*
 * function q = matrixToQuaternion(m)
 */
void matrixToQuaternion(const float m[9], float q[4])
{
  float qw;

  /* 'matrixToQuaternion:2' m00 = m(1,1); */
  /* 'matrixToQuaternion:3' m01 = m(1,2); */
  /* 'matrixToQuaternion:4' m02 = m(1,3); */
  /* 'matrixToQuaternion:5' m10 = m(2,1); */
  /* 'matrixToQuaternion:6' m11 = m(2,2); */
  /* 'matrixToQuaternion:7' m12 = m(2,3); */
  /* 'matrixToQuaternion:8' m20 = m(3,1); */
  /* 'matrixToQuaternion:9' m21 = m(3,2); */
  /* 'matrixToQuaternion:10' m22 = m(3,3); */
  /* 'matrixToQuaternion:12' qw = (1+m00+m11+m22)^0.5/2; */
  qw = sqrtf(((m[0] + 1.0F) + m[4]) + m[8]) / 2.0F;

  /* 'matrixToQuaternion:13' qx = (m12-m21)/(4*qw); */
  /* 'matrixToQuaternion:14' qy = (m20-m02)/(4*qw); */
  /* 'matrixToQuaternion:15' qz = (m01-m10)/(4*qw); */
  /* 'matrixToQuaternion:16' q = [qw;qx;qy;qz] */
  q[0] = qw;
  q[1] = (m[7] - m[5]) / (4.0F * qw);
  q[2] = (m[2] - m[6]) / (4.0F * qw);
  q[3] = (m[3] - m[1]) / (4.0F * qw);
}
/* End of code generation (matrixToQuaternion.c) */

/*
 * function Vector = QuaternionRotate(VectorA,q)
 */
void QuaternionRotate(const float VectorA[3], const float q[4], float Vector[3])
{
  float qi[4];
  float fv[4];
  float fv1[4];

  /* 'QuaternionRotate:2' qi = getQuaternionInverse(q); */
  getQuaternionInverse(q, qi);

  /* 'QuaternionRotate:3' Vector = getQuaternionProduct(q,getQuaternionProduct([0;VectorA],qi)); */
  fv[0] = 0.0F;
  fv[1] = VectorA[0];
  fv[2] = VectorA[1];
  fv[3] = VectorA[2];
  getQuaternionProduct(fv, qi, fv1);
  getQuaternionProduct(q, fv1, qi);

  /*                  disp(['A:   ' num2str(VectorA.')]) */
  /*                  disp(['q:   ' num2str(q.')]) */
  /*                  disp(['qi:  ' num2str(qi.')]) */
  /*                  disp(['QA:  ' num2str(getQuaternion(0,VectorA).')]) */
  /*                  disp(['Aqi: ' num2str(getQuaternionProduct([0;VectorA],qi).')]) */
  /*                  disp(['qV:  ' num2str(Vector.')]) */
  /* 'QuaternionRotate:10' Vector = Vector(2:4); */
  Vector[0] = qi[1];
  Vector[1] = qi[2];
  Vector[2] = qi[3];
}
/* End of code generation (QuaternionRotate.c) */

void test(){
	float Angle; 
	float Vector[3];
	float q[4];
	
	Angle = M_PI/6.0F;
	Vector[0] = 1.0F;
	Vector[1] = 0.0F;
	Vector[2] = 0.0F;
	q[0] = 1.0F;
	q[1] = 0.0F;
	q[2] = 0.0F;
	q[3] = 0.0F;
	
	
	getQuaternion(Angle, Vector, q);
	printf("q1: %.6f\nq2: %.6f\nq3: %.6f\nq4: %.6f\n", q[0], q[1], q[2], q[3]);
	
	
	Angle = M_PI/2.0F;
	Vector[0] = 0.0F;
	Vector[1] = 1.0F;
	Vector[2] = 0.0F;
	q[0] = 1.0F;
	q[1] = 0.0F;
	q[2] = 0.0F;
	q[3] = 0.0F;
	
	
	getQuaternion(Angle, Vector, q);
	printf("q1: %.6f\nq2: %.6f\nq3: %.6f\nq4: %.6f\n", q[0], q[1], q[2], q[3]);
	
	
	float VectorA[3];
	float VectorB[3];
	
	VectorA[0] = 1.0F;
	VectorA[1] = 0.0F;
	VectorA[2] = 0.0F;
	
	VectorB[0] = 0.0F;
	VectorB[1] = 0.0F;
	VectorB[2] = 0.0F;
	
	QuaternionRotate(VectorA, q, VectorB);
	
	printf("v1: %.6f\nv2: %.6f\nv3: %.6f\n", VectorB[0], VectorB[1], VectorB[2]);
}