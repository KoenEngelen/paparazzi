#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
// #include "dep.h"
float rho = 1.225;
float S = 0.2;
float m = 0.8; 

// Sub CE functions 
float AoA(float V[]){
    return atan2f(V[0], V[2]);
}
float AoS(float V[]){
    return atan2f(V[1], V[2]);
}
float Cl(float alpha_rad){
    float alpha = alpha_rad * 180.0/M_PI;
    float res;
    if (alpha > 180.0){
        alpha = alpha - 360.0;
    }

    float a1 =      1.233     ;
    float b1 =      0.04068   ;
    float c1 =      0.09493   ;
    float a2 =    141.1       ;
    float b2 =      0.05424   ;
    float c2 =      0.1637    ;
    float a3 =      0.1126    ;
    float b3 =      0.101     ;
    float c3 =      0.2612    ;
    float a4 =      0.08224   ;
    float b4 =      0.136     ;
    float c4 =      0.2642    ;
    float a5 =      0.1197    ;
    float b5 =      0.01427   ;
    float c5 =      0.6238    ;
    float a6 =      0.05624   ;
    float b6 =      0.1704    ;
    float c6 =      0.2682    ;
    float a7 =      0.03084   ;
    float b7 =      0.2046    ;
    float c7 =      0.4082    ;
    float a8 =    141.6       ;
    float b8 =      0.05419   ;
    float c8 =      3.305     ;
    res = a1*sin(b1*alpha+c1) + a2*sin(b2*alpha+c2) + a3*sin(b3*alpha+c3)
        + a4*sin(b4*alpha+c4) + a5*sin(b5*alpha+c5) + a6*sin(b6*alpha+c6)
        + a7*sin(b7*alpha+c7) + a8*sin(b8*alpha+c8);
    return res;
}
float Cd(float alpha_rad){
    float alpha = alpha_rad * 180.0/M_PI;
    float res;
    if (alpha > 180.0){
        alpha = alpha - 360.0;
    }

    float a0 =      0.8371 ;
    float a1 =      0.1004 ;
    float b1 =     0.01822 ;
    float a2 =     -0.6466 ;
    float b2 =   -0.005324 ;
    float a3 =    -0.02505 ;
    float b3 =    0.008351 ;
    float a4 =    -0.05651 ;
    float b4 =   0.0007461 ;
    float w  =     0.01675 ;
    res = a0
        + a1*cos(alpha*w)   + b1*sin(alpha*w)
        + a2*cos(2*alpha*w) + b2*sin(2*alpha*w)
        + a3*cos(3*alpha*w) + b3*sin(3*alpha*w)
        + a4*cos(4*alpha*w) + b4*sin(4*alpha*w);
    return res;
}
float dCl(float alpha_rad){
    float res;
    float alpha = alpha_rad *180.0/M_PI;
    if (alpha > 180.0){
        alpha = alpha - 360.0;
    }

    float a1 =    1.233     ;
    float b1 =    0.04068   ;
    float c1 =    0.09493   ;
    float a2 =  141.1       ;
    float b2 =    0.05424   ;
    float c2 =    0.1637    ;
    float a3 =    0.1126    ;
    float b3 =    0.101     ;
    float c3 =    0.2612    ;
    float a4 =    0.08224   ;
    float b4 =    0.136     ;
    float c4 =    0.2642    ;
    float a5 =    0.1197    ;
    float b5 =    0.01427   ;
    float c5 =    0.6238    ;
    float a6 =    0.05624   ;
    float b6 =    0.1704    ;
    float c6 =    0.2682    ;
    float a7 =    0.03084   ;
    float b7 =    0.2046    ;
    float c7 =    0.4082    ;
    float a8 =  141.6       ;
    float b8 =    0.05419   ;
    float c8 =    3.305     ;
    res = b1*a1*cos(b1*alpha+c1) + b2*a2*cos(b2*alpha+c2) + b3*a3*cos(b3*alpha+c3)
         + b4*a4*cos(b4*alpha+c4) + b5*a5*cos(b5*alpha+c5) + b6*a6*cos(b6*alpha+c6)
         + b7*a7*cos(b7*alpha+c7) + b8*a8*cos(b8*alpha+c8);
    res =  180.0/M_PI*res;
    return res;
}
float dCd(float alpha_rad){
    float res;
    float alpha = alpha_rad *180.0/M_PI;
    if (alpha > 180.0){
        alpha = alpha - 360.0;
    }
    float a1 =   0.1004   ;
    float b1 =   0.01822  ;
    float a2 =  -0.6466   ;
    float b2 =  -0.005324 ;
    float a3 =  -0.02505  ;
    float b3 =   0.008351 ;
    float a4 =  -0.05651  ;
    float b4 =   0.0007461;
    float w  =   0.01675  ;
    res = -1*w*a1*sin(1*alpha*w) + 1*b1*w*cos(1*alpha*w)
        -  2*w*a2*sin(2*alpha*w) + 2*b2*w*cos(2*alpha*w)
        -  3*w*a3*sin(3*alpha*w) + 3*b3*w*cos(3*alpha*w)
        -  4*w*a4*sin(4*alpha*w) + 4*b4*w*cos(4*alpha*w);
    res =  180.0/M_PI*res;
    return res;
}

float Vxz_2_dalphai_dphii(float V[]){
    float res;
    res = V[0] * V[1];
    return res;
}
float Vxz_2_dCl_dphii(float V[]){
    float res;
    float alpha_rad = AoA(V);
    res = dCl(alpha_rad) * Vxz_2_dalphai_dphii(V);
    return res;
}
float Vxz_2_dCd_dphii(float V[]){
    float res;
    float alpha_rad = AoA(V);
    res = dCd(alpha_rad) * Vxz_2_dalphai_dphii(V);
    return res;
}
float dVxz_2_dphii(float V[]){
    float res;
    float beta_rad = AoS(V);
    res = 2.0*V[2]*sinf(beta_rad)*sqrtf(powf(V[1],2.0)+powf(V[2],2.0));
    return res;
}
float dalphai_dthetai(){
    return -1.0;
}
float dCl_dthetai(float V[]){
    float res;
    float alpha_rad = AoA(V);
    res = dCl(alpha_rad) * dalphai_dthetai();
    return res;
}
float dVxz_2_dthetai(){
    return 0.0;
}
float dCd_dthetai(float V[]){
    float res;
    float alpha_rad = AoA(V);
    res = dCd(alpha_rad) * dalphai_dthetai();
    return res;
}
float dVxz_2_dpsii(float V[]){
    float res;
    res = 2.0*V[0]*V[1];
    return res;
}
float Vxz_2_dalphai_dpsii(float V[]){
    float res;
    res = V[1] * V[2];
    return res;
}
float Vxz_2_dCl_dpsii(float V[]){
    float res;
    float alpha_rad = AoA(V);
    res = dCl(alpha_rad) * Vxz_2_dalphai_dpsii(V);
    return res;
}
float Vxz_2_dCd_dpsii(float V[]){
    float res;
    float alpha_rad = AoA(V);
    res = dCd(alpha_rad) * Vxz_2_dalphai_dpsii(V);
    return res;
}

// Main CE functions
float da_wing_x_dphii(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    res = rho*S/(2.0*m)*
        (-sinf(alpha_rad)* Cl(alpha_rad)          * Vxz_2_dalphai_dphii(V)
         +cosf(alpha_rad)* Vxz_2_dCl_dphii(V)                            
         -cosf(alpha_rad)* Cl(alpha_rad)          * dVxz_2_dphii(V)       
         +cosf(alpha_rad)* Cd(alpha_rad)          * Vxz_2_dalphai_dphii(V)
         +sinf(alpha_rad)* Vxz_2_dCd_dphii(V)                          
         -sinf(alpha_rad)* Cd(alpha_rad)          * dVxz_2_dphii(V)       );

    return res;
}
float da_wing_y_dphii(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    float Vxz_2 = powf(V[0],2.0) + powf(V[2],2.0);

    res = rho*S/(2*m)*
        (+sinf(alpha_rad)* Cl(alpha_rad) * Vxz_2
         -cosf(alpha_rad)* Cd(alpha_rad) * Vxz_2 );
    return res;
}
float da_wing_z_dphii(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    float Vxz_2 = powf(V[0],2.0) + powf(V[2],2.0);

    res = rho*S/(2*m)* 
        (-cosf(alpha_rad)* Cl(alpha_rad)          * Vxz_2_dalphai_dphii(V)
         -sinf(alpha_rad)* Vxz_2_dCl_dphii(V)                                                 
         -sinf(alpha_rad)* Cl(alpha_rad)          * dVxz_2_dphii(V)                             
         -sinf(alpha_rad)* Cd(alpha_rad)          * Vxz_2_dalphai_dphii(V)
         +cosf(alpha_rad)* Vxz_2_dCd_dphii(V)                                               
         +cosf(alpha_rad)* Cd(alpha_rad)          * dVxz_2_dphii(V)            );
    return res;
}
float da_wing_x_dthetai(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    float Vxz_2 = powf(V[0],2.0) + powf(V[2],2.0);

    res = -rho*S/(2*m)* 
        (-cosf(alpha_rad)* dCl_dthetai(V)* Vxz_2                                                   
         -cosf(alpha_rad)* Cl(alpha_rad)          * dVxz_2_dthetai() * dalphai_dthetai()
         -sinf(alpha_rad)* dCd_dthetai(V)* Vxz_2                                                   
         -sinf(alpha_rad)* Cd(alpha_rad)          * dVxz_2_dthetai()                     );
    return res;
}
float da_wing_y_dthetai(){
    return 0.0;
}
float da_wing_z_dthetai(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    float Vxz_2 = powf(V[0],2.0) + powf(V[2],2.0);

    res =  - rho*S/(2*m)* 
        ( -sinf(alpha_rad)* dCl_dthetai(V)* Vxz_2                                                   
          -sinf(alpha_rad)* Cl(alpha_rad)            * dVxz_2_dthetai()                             
          +cosf(alpha_rad)* dCd_dthetai(V)* Vxz_2                                                   
          +cosf(alpha_rad)* Cd(alpha_rad)            * dVxz_2_dthetai() );
    return res;
}
float da_wing_x_dpsii(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    res = rho*S/(2*m)* 
        (+sinf(alpha_rad)* Cl(alpha_rad)          * Vxz_2_dalphai_dpsii(V)
         -cosf(alpha_rad)* Vxz_2_dCl_dpsii(V)                                                
         -cosf(alpha_rad)* Cl(alpha_rad)          * dVxz_2_dpsii(V)                             
         -cosf(alpha_rad)* Cd(alpha_rad)          * Vxz_2_dalphai_dpsii(V)
         -sinf(alpha_rad)* Vxz_2_dCd_dpsii(V)                                                  
         -sinf(alpha_rad)* Cd(alpha_rad)          * dVxz_2_dpsii(V)         );
    return res;
}
float da_wing_y_dpsii(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    float Vxz_2 = powf(V[0],2.0) + powf(V[2],2.0);

    res = rho*S/(2*m)* 
        (-cosf(alpha_rad)* Cl(alpha_rad) * Vxz_2 
         -sinf(alpha_rad)* Cd(alpha_rad) * Vxz_2 );
}
float da_wing_z_dpsii(float V[]){
    float alpha_rad;
    float res;


    alpha_rad = AoA(V);

    res = rho*S/(2*m)* 
        (-cosf(alpha_rad)* Cl(alpha_rad)          * Vxz_2_dalphai_dpsii(V)
         -sinf(alpha_rad)* Vxz_2_dCl_dpsii(V)                                               
         -sinf(alpha_rad)* Cl(alpha_rad)          * dVxz_2_dpsii(V)                             
         -sinf(alpha_rad)* Cd(alpha_rad)          * Vxz_2_dalphai_dpsii(V)
         +cosf(alpha_rad)* Vxz_2_dCd_dpsii(V)                                                 
         +cosf(alpha_rad)* Cd(alpha_rad)          * dVxz_2_dpsii(V)         );
    return res;
}
