#ifndef PROJECT__SPHERICAL__H
#define PROJECT__SPHERICAL__H

void spherical1(double x, double y, double z, double *out_4922184406212571723);
void spherical2(double x, double y, double z, double *out_5156308728579745157);

void error_derivatives_1(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_8361550651882002035);
void error_derivatives_2(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_5375331181930566001);

void spherical_2_vec_1(double rho_1, double theta_1, double *out_7173870751326202183);
void spherical_2_vec_2(double rho_1, double theta_1, double *out_7884124045604159789);

#endif
