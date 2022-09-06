#include "error_derivatives.h"
#include <math.h>
double error_dt1(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_dt1_result;
   error_dt1_result = omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_2) - cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) + sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1));
   return error_dt1_result;
}

double error_dr1(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_dr1_result;
   error_dr1_result = omega_x*sin(theta_1)*cos(rho_1)*cos(theta_2) + omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) + omega_z*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
   return error_dr1_result;
}

double error_dt2(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_dt2_result;
   error_dt2_result = omega_x*(-sin(rho_1)*sin(theta_1)*sin(theta_2) - sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_1) + cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) + sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2));
   return error_dt2_result;
}

double error_dr2(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_dr2_result;
   error_dr2_result = -omega_x*sin(theta_2)*cos(rho_2)*cos(theta_1) - omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
   return error_dr2_result;
}
