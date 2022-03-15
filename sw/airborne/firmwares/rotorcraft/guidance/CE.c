#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "math/pprz_algebra_float.h"

// CE Lib Matlab
  
  
 
  
  /*
  * function res = a_D_wing_x(V)
  */
  float a_D_wing_x(float V[])
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
  // float a_D_wing_z(float V[],float phi)
  // {
  //   float phi_rad;
  //   float R_tmp;
  //   float fv[9];
  //   int i;
  //   float fv1[3];
  //   float alpha_rad;
  //   float CDa_tmp;
  //   float b_CDa_tmp;
  //   float c_CDa_tmp;

  //   /* 'a_D_wing_z:2' phi_rad = -phi * pi/180.0; */
  //   phi_rad = -da_wing_x_dphii(V) * M_PI / 180.0F;

  //   /* 'a_D_wing_z:3' R = [1 0 0; 0 cos(phi_rad) -sin(phi_rad); 0 sin(phi_rad) cos(phi_rad)]; */
  //   R_tmp = sinf(phi_rad);
  //   phi_rad = cosf(phi_rad);

  //   /* 'a_D_wing_z:5' V = R*V; */
  //   fv[0] = 1.0F;
  //   fv[3] = 0.0F;
  //   fv[6] = 0.0F;
  //   fv[1] = 0.0F;
  //   fv[4] = phi_rad;
  //   fv[7] = -R_tmp;
  //   fv[2] = 0.0F;
  //   fv[5] = R_tmp;
  //   fv[8] = phi_rad;
  //   for (i = 0; i < 3; i++) {
  //     fv1[i] = (fv[i] * V[0] + fv[i + 3] * V[1]) + fv[i + 6] * V[2];
  //   }

  //   /* 'a_D_wing_z:7' Vx = V(1); */
  //   /* 'a_D_wing_z:8' Vz = V(3); */
  //   /* 'a_D_wing_z:10' rho = 1.225; */
  //   /* 'a_D_wing_z:11' S = 0.2; */
  //   /* 'a_D_wing_z:12' m = 0.8; */
  //   /* 'a_D_wing_z:14' Vxz = (Vx^2 + Vz^2)^0.5; */
  //   R_tmp = sqrtf(fv1[0] * fv1[0] + fv1[2] * fv1[2]);

  //   /* 'a_D_wing_z:16' alpha_rad = atan2(Vx,Vz); */
  //   alpha_rad = atan2f(fv1[0], fv1[2]);

  //   /* 'a_D_wing_z:17' alpha = alpha_rad * 180 / pi; */
  //   /* 'a_D_wing_z:19' res = rho*S/(2*m)* cos(phi*pi/180) *... */
  //   /* 'a_D_wing_z:20'                 ( cos(alpha_rad)* CE.Cd(alpha) * Vxz^2 ); */
  //   phi_rad = alpha_rad * 180.0F / M_PI;

  //   /* 'CE:84' x  =      alpha     ; */
  //   /* 'CE:85' if x > 180 */
  //   if (phi_rad > 180.0F) {
  //     /* 'CE:86' x = x - 360  */
  //     phi_rad -= 360.0F;
  //   }

  //   /* 'CE:88' a0 =      0.8371  ; */
  //   /* 'CE:89' a1 =      0.1004 ; */
  //   /* 'CE:90' b1 =     0.01822  ; */
  //   /* 'CE:91' a2 =     -0.6466  ; */
  //   /* 'CE:92' b2 =   -0.005324 ; */
  //   /* 'CE:93' a3 =    -0.02505 ; */
  //   /* 'CE:94' b3 =    0.008351 ; */
  //   /* 'CE:95' a4 =    -0.05651 ; */
  //   /* 'CE:96' b4 =   0.0007461  ; */
  //   /* 'CE:97' w =     0.01675 ; */
  //   /* 'CE:98' CDa =  a0 + ... */
  //   /* 'CE:99'                    a1*cos(x*w) + b1*sin(x*w) + ... */
  //   /* 'CE:100'                    a2*cos(2*x*w) + b2*sin(2*x*w) + ... */
  //   /* 'CE:101'                    a3*cos(3*x*w) + b3*sin(3*x*w) + ... */
  //   /* 'CE:102'                    a4*cos(4*x*w) + b4*sin(4*x*w); */
  //   CDa_tmp = phi_rad * 0.01675F;
  //   b_CDa_tmp = 2.0F * phi_rad * 0.01675F;
  //   c_CDa_tmp = 3.0F * phi_rad * 0.01675F;
  //   phi_rad = 4.0F * phi_rad * 0.01675F;

  //   /* 'CE:103' res =  CDa; */
  //   /*              CDa_y = [0.2,  0.4,   0.7,   1.9,     2,     1.9,   0.5, 0.4]; %Lookup_y: C_D */
  //   /*              CDa_x = [   0,   7.5,     15,    80,    90,     100,    170,  180];      %Lookup_x: alpha */
  //   /*              CDa_y = [CDa_y fliplr(CDa_y(1:end-1))]; */
  //   /*              CDa_x = [CDa_x 360-fliplr(CDa_x(1:end-1))]; */
  //   /*               */
  //   /*              if alpha < 0 */
  //   /*                  alpha= 360; */
  //   /*              end */
  //   /*              CDa = interp1(CDa_x,CDa_y,alpha); */
  //   /*              res =  CDa; */
  //   /*              res =  1.02-cos(2*alpha*pi/180); */
  //   return 0.153125F * cosf(phi * M_PI / 180.0F) * (cosf(alpha_rad) *
  //     ((((((((0.1004F * cosf(CDa_tmp) + 0.8371F) + 0.01822F * sinf(CDa_tmp)) +
  //           -0.6466F * cosf(b_CDa_tmp)) + -0.005324F * sinf(b_CDa_tmp)) +
  //         -0.02505F * cosf(c_CDa_tmp)) + 0.008351F * sinf(c_CDa_tmp)) + -0.05651F *
  //       cosf(phi_rad)) + 0.0007461F * sinf(phi_rad)) * (R_tmp * R_tmp));
  // }
  /* End of code generation (a_D_wing_z.c) */

  /*
  * function res = a_L_wing_x(V)
  */
  float a_L_wing_x(float V[])
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
  float a_L_wing_z(float V[], float phi)
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
  float a_wing_x(float V[])
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
  float a_wing_z(float V[], float phi)
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
  float AoA(float V[])
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
  float AoD(float V[])
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
  float AoS(float V[])
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
  float Vxz_2(float V[])
  {
    /* 'Vxz_2:2' Vx = V(1); */
    /* 'Vxz_2:3' Vz = V(3); */
    /* 'Vxz_2:4' res = Vx^2+Vz^2; */
    return V[0] * V[0] + V[2] * V[2];
  }
  /* End of code generation (Vxz_2.c) */

 
  /*
  * function res = dalphai_dphii(V)
  */
  float dalphai_dphii(float V[])
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
  float dalphai_dpsii(float V[])
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
  float dbetai_dpsii(float V[])
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
  float dbetai_dthetai(float V[])
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
  float dCd_dphii(float V[])
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
  float dCd_dpsii(float V[])
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
  float dCd_dthetai(float V[])
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
  float dCl_dphii(float V[])
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
  float dCl_dpsii(float V[])
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
  float dCl_dthetai(float V[])
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
  float dVx_dpsii(float V[])
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
  float dVx_dthetai(float V[])
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
  float dVxz_2_dphii(float V[])
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
  float dVxz_2_dpsii(float V[])
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
  float dVy_dphii(float V[])
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
  float dVz_dphii(float V[])
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
  float dVz_dthetai(float V[])
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
  float Vxz_2_dalphai_dphii(float V[])
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
  float Vxz_2_dalphai_dpsii(float V[])
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
  float Vxz_2_dCd_dphii(float V[])
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
  float Vxz_2_dCd_dpsii(float V[])
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
  float Vxz_2_dCl_dphii(float V[])
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
  float Vxz_2_dCl_dpsii(float V[])
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
