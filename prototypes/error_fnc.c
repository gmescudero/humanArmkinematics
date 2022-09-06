#include "error_fnc.h"
#include <math.h>
double error(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_result;
   error_result = omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1));
   return error_result;
}
