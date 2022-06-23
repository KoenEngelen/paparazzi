#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "math/pprz_algebra_float.h"

 

static inline float thrust2cmd(float thrust)
{
  float x[20];
  float y[20];

  x[0]  = 0.0;
  x[1]  = 0.0526315789473684;	
  x[2]  = 0.105263157894737;	
  x[3]  = 0.157894736842105;	
  x[4]  = 0.210526315789474;	
  x[5]  = 0.263157894736842;	
  x[6]  = 0.315789473684211;	
  x[7]  = 0.368421052631579;	
  x[8]  = 0.421052631578947;	
  x[9]  = 0.473684210526316;	
  x[10] = 0.526315789473684;	
  x[11] = 0.578947368421053;	
  x[12] = 0.631578947368421;	
  x[13] = 0.684210526315790;	
  x[14] = 0.736842105263158;	
  x[15] = 0.789473684210526;	
  x[16] = 0.842105263157895;	
  x[17] = 0.894736842105263;	
  x[18] = 0.947368421052632;	
  x[19] = 1.0;
  
  y[0]  = 0.00018366013071; 
  y[1]  = 0.05073244315826; 	
  y[2]  = 0.16345421649358; 	
  y[3]  = 0.92652640909841; 	
  y[4]  = 1.87456099802201; 	
  y[5]  = 2.62738722228120; 	
  y[6]  = 3.16092679228087; 	
  y[7]  = 3.54847057386879; 	
  y[8]  = 3.84445853420274; 	
  y[9]  = 4.07913232027519; 	
  y[10] = 4.26854898019188; 	
  y[11] = 4.42138188867095; 	
  y[12] = 4.54247867820420; 	
  y[13] = 4.63465002715227; 	
  y[14] = 4.69958280369314; 	
  y[15] = 4.73832115551302; 	
  y[16] = 4.75152908989299; 	
  y[17] = 4.75152908989299; 	
  y[18] = 4.75152908989299; 	
  y[19] = 4.75152908989299;

  float cmd;

  for (int i = 0; i < 20; i++)
  {
    if (thrust <= y[i]){
      cmd = (x[i]- x[i-1])*(thrust - y[i-1])/(y[i] - y[i-1]) + x[i-1];
      return cmd;
    }
  }
return -1.0;
}

 
 //New Functions
  /*
  * function res = da_wing_x_dphii(V)
  */
     static inline float da_wing_x_dphii(float V[3])
  {
    float alpha_rad;
    float alpha;
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
  static inline float da_wing_x_dpsii(float V[3])
  {
    float alpha_rad;
    float alpha;
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
  static inline float da_wing_x_dthetai(float V[3])
  {
    float Vxz;
    float alpha_rad;
    float alpha;
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
  static inline float da_wing_y_dphii(float V[3])
  {
    float Vxz;
    float alpha_rad;
    float alpha;
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
  static inline float da_wing_y_dpsii(float V[3])
  {
    float Vxz;
    float alpha_rad;
    float alpha;
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
  static inline float da_wing_y_dthetai(void)
  {
    /* 'da_wing_y_dthetai:2' res = 0; */
    return 0.0;
  }
  /* End of code generation (da_wing_y_dthetai.c) */



  /*
  * function res = da_wing_z_dphii(V)
  */
  static inline float da_wing_z_dphii(float V[3])
  {
    float alpha_rad;
    float alpha;
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
static inline float da_wing_z_dpsii(float V[3])
  {
    float alpha_rad;
    float alpha;
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
  static inline float da_wing_z_dthetai(float V[3])
  {
    float Vxz;
    float alpha_rad;
    float alpha;
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