#include <math.h>

double error_sph(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_result;
   error_result = omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1));
   return error_result;
}
double error_xyz(double j_x1, double j_x2, double j_y1, double j_y2, double j_z1, double j_z2, double omega_x, double omega_y, double omega_z) {
   double error_result;
   error_result = omega_x*(j_y1*j_z2 - j_y2*j_z1) + omega_y*(-j_x1*j_z2 + j_x2*j_z1) + omega_z*(j_x1*j_y2 - j_x2*j_y1);
   return error_result;
}


void spherical(double x, double y, double z, double *out_546617460871897354) {
   out_546617460871897354[0] = atan2(sqrt(pow(y, 2) + pow(z, 2)), x);
   out_546617460871897354[1] = atan2(y, z);
}
void spherical_2_vec_2(double rho_1, double theta_1, double *out_1345109177662629090) {
   out_1345109177662629090[0] = cos(theta_1);
   out_1345109177662629090[1] = sin(rho_1)*sin(theta_1);
   out_1345109177662629090[2] = sin(theta_1)*cos(rho_1);
}


void error_derivatives_11(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_1851644233079195652) {
   out_1851644233079195652[0] = omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_2) - cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) + sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1));
   out_1851644233079195652[1] = omega_x*sin(theta_1)*cos(rho_1)*cos(theta_2) + omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) + omega_z*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
   out_1851644233079195652[2] = omega_x*(-sin(rho_1)*sin(theta_1)*sin(theta_2) - sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_1) + cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) + sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_1851644233079195652[3] = -omega_x*sin(theta_2)*cos(rho_2)*cos(theta_1) - omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
}
void error_derivatives_21(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_2771013456807691716) {
   out_2771013456807691716[0] = omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1)) + omega_y*(sin(theta_1)*cos(theta_2) + sin(theta_2)*cos(rho_1)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) - sin(rho_2)*sin(theta_1)*sin(theta_2));
   out_2771013456807691716[1] = omega_x*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*cos(rho_1)*cos(theta_2)) - omega_y*sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) - omega_z*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2);
   out_2771013456807691716[2] = omega_x*(-sin(rho_1)*sin(theta_1)*sin(theta_2) - sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2)) + omega_y*(sin(theta_1)*cos(rho_1)*cos(rho_2)*cos(theta_2) + sin(theta_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) + sin(rho_2)*cos(theta_1)*cos(theta_2));
   out_2771013456807691716[3] = -omega_x*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2) - omega_y*sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1) + omega_z*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1));
}
void error_derivatives_12(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_114860670380986926) {
   out_114860670380986926[0] = omega_x*(sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) + sin(rho_2)*sin(theta_1)*sin(theta_2)) + omega_y*(-sin(theta_1)*cos(theta_2) - sin(theta_2)*cos(rho_1)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1));
   out_114860670380986926[1] = omega_x*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2) + omega_y*sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + omega_z*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_114860670380986926[2] = omega_x*(sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) - sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(rho_2)*cos(theta_2) - sin(theta_2)*cos(theta_1)) + omega_z*(sin(rho_1)*sin(theta_1)*sin(theta_2) + sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_114860670380986926[3] = omega_x*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_y*sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1) + omega_z*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2);
}
void error_derivatives_22(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_235208720016758055) {
   out_235208720016758055[0] = omega_x*(sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) - sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_2) + cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*cos(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_1)*sin(theta_2));
   out_235208720016758055[1] = omega_x*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2)) - omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) - omega_z*sin(theta_1)*cos(rho_1)*cos(theta_2);
   out_235208720016758055[2] = omega_x*(sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) - sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_1) - cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(sin(rho_1)*sin(theta_1)*sin(theta_2) + sin(rho_2)*cos(theta_1)*cos(theta_2));
   out_235208720016758055[3] = omega_x*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2)) + omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*sin(theta_2)*cos(rho_2)*cos(theta_1);
}
#include "code_gen.h"
#include <math.h>
double error_sph(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_sph_result;
   error_sph_result = omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1));
   return error_sph_result;
}
