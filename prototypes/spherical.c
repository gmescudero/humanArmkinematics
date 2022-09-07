#include "spherical.h"
#include <math.h>


void spherical1(double x, double y, double z, double *out_4922184406212571723) {
   out_4922184406212571723[0] = atan2(sqrt(x*x + y*y), z);
   out_4922184406212571723[1] = atan2(y, x);
}
void spherical2(double x, double y, double z, double *out_5156308728579745157) {
   out_5156308728579745157[0] = atan2(sqrt(y*y + z*z), x);
   out_5156308728579745157[1] = atan2(y, z);
}


void error_derivatives_1(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_8361550651882002035) {
   out_8361550651882002035[0] = omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_2) - cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) + sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1));
   out_8361550651882002035[1] = omega_x*sin(theta_1)*cos(rho_1)*cos(theta_2) + omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) + omega_z*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
   out_8361550651882002035[2] = omega_x*(-sin(rho_1)*sin(theta_1)*sin(theta_2) - sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_1) + cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) + sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_8361550651882002035[3] = -omega_x*sin(theta_2)*cos(rho_2)*cos(theta_1) - omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
}
void error_derivatives_2(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_5375331181930566001) {
   out_5375331181930566001[0] = omega_x*(sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) - sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_2) + cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*cos(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_1)*sin(theta_2));
   out_5375331181930566001[1] = omega_x*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2)) - omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) - omega_z*sin(theta_1)*cos(rho_1)*cos(theta_2);
   out_5375331181930566001[2] = omega_x*(sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) - sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_1) - cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(sin(rho_1)*sin(theta_1)*sin(theta_2) + sin(rho_2)*cos(theta_1)*cos(theta_2));
   out_5375331181930566001[3] = omega_x*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2)) + omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*sin(theta_2)*cos(rho_2)*cos(theta_1);
}



void spherical_2_vec_1(double rho_1, double theta_1, double *out_7173870751326202183) {
   out_7173870751326202183[0] = sin(theta_1)*cos(rho_1);
   out_7173870751326202183[1] = sin(rho_1)*sin(theta_1);
   out_7173870751326202183[2] = cos(theta_1);
}
void spherical_2_vec_2(double rho_1, double theta_1, double *out_7884124045604159789) {
   out_7884124045604159789[0] = cos(theta_1);
   out_7884124045604159789[1] = sin(rho_1)*sin(theta_1);
   out_7884124045604159789[2] = sin(theta_1)*cos(rho_1);
}
