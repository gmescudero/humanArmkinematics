#ifndef PROJECT__ERROR_DERIVATIVES__H
#define PROJECT__ERROR_DERIVATIVES__H
double error_dt1(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2);

double error_dr1(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2);

double error_dt2(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2);

double error_dr2(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2);
#endif
