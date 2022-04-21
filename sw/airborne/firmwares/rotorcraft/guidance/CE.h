#ifndef CONTROL_EFFECTIVENESS_KOEN
#define CONTROL_EFFECTIVENESS_KOEN


// Sub CE functions 
extern float AoA(float V[]);
extern float AoS(float V[]);
extern float Cl(float alpha_rad);
extern float Cd(float alpha_rad);
extern float dCl(float alpha_rad);
extern float dCd(float alpha_rad);

extern float Vxz_2_dalphai_dphii(float V[]);
extern float Vxz_2_dCl_dphii(float V[]);
extern float Vxz_2_dCd_dphii(float V[]);
extern float dVxz_2_dphii(float V[]);
extern float dalphai_dthetai(void);
extern float dCl_dthetai(float V[]);
extern float dVxz_2_dthetai(void);
extern float dCd_dthetai(float V[]);
extern float dVxz_2_dpsii(float V[]);
extern float Vxz_2_dalphai_dpsii(float V[]);
extern float Vxz_2_dCl_dpsii(float V[]);
extern float Vxz_2_dCd_dpsii(float V[]);

// Main CE functions
extern float da_wing_x_dphii(float V[]);
extern float da_wing_y_dphii(float V[]);
extern float da_wing_z_dphii(float V[]);
extern float da_wing_x_dthetai(float V[]);
extern float da_wing_y_dthetai(void);
extern float da_wing_z_dthetai(float V[]); 
extern float da_wing_x_dpsii(float V[]);
extern float da_wing_y_dpsii(float V[]);
extern float da_wing_z_dpsii(float V[]);

#endif