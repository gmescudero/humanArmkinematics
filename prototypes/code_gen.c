#include <math.h>


double error_sph_11(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_sph_result;
   error_sph_result = omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1)) + (pow(omega_x, 2) + pow(omega_y, 2) + pow(omega_z, 2))*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2) + cos(theta_1)*cos(theta_2));
   return error_sph_result;
}
double error_xyz(double j_x1, double j_x2, double j_y1, double j_y2, double j_z1, double j_z2, double omega_x, double omega_y, double omega_z) {
   double error_xyz_result;
   error_xyz_result = omega_x*(j_y1*j_z2 - j_y2*j_z1) + omega_y*(-j_x1*j_z2 + j_x2*j_z1) + omega_z*(j_x1*j_y2 - j_x2*j_y1);
   return error_xyz_result;
}



void spherical_1(double x, double y, double z, double *out_6370209996326608495) {
   out_6370209996326608495[0] = atan2(sqrt(pow(x, 2) + pow(y, 2)), z);
   out_6370209996326608495[1] = atan2(y, x);
}
void spherical_2(double x, double y, double z, double *out_4955452728107676566) {
   out_4955452728107676566[0] = atan2(sqrt(pow(y, 2) + pow(z, 2)), x);
   out_4955452728107676566[1] = atan2(y, z);
}



void spherical_2_vec_1(double rho_1, double theta_1, double *out_5770186284105068230) {
   out_5770186284105068230[0] = sin(theta_1)*cos(rho_1);
   out_5770186284105068230[1] = sin(rho_1)*sin(theta_1);
   out_5770186284105068230[2] = cos(theta_1);
}
void spherical_2_vec_2(double rho_1, double theta_1, double *out_5640871096743714671) {
   out_5640871096743714671[0] = cos(theta_1);
   out_5640871096743714671[1] = sin(rho_1)*sin(theta_1);
   out_5640871096743714671[2] = sin(theta_1)*cos(rho_1);
}



void error_derivatives_11(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_7676746111336239405) {
   out_7676746111336239405[0] = omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_2) - cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) + sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1));
   out_7676746111336239405[1] = omega_x*sin(theta_1)*cos(rho_1)*cos(theta_2) + omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) + omega_z*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
   out_7676746111336239405[2] = omega_x*(-sin(rho_1)*sin(theta_1)*sin(theta_2) - sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_1) + cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) + sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_7676746111336239405[3] = -omega_x*sin(theta_2)*cos(rho_2)*cos(theta_1) - omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2));
}
void error_derivatives_21(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_2016783461154155129) {
   out_2016783461154155129[0] = omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1)) + omega_y*(sin(theta_1)*cos(theta_2) + sin(theta_2)*cos(rho_1)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) - sin(rho_2)*sin(theta_1)*sin(theta_2));
   out_2016783461154155129[1] = omega_x*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*cos(rho_1)*cos(theta_2)) - omega_y*sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) - omega_z*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2);
   out_2016783461154155129[2] = omega_x*(-sin(rho_1)*sin(theta_1)*sin(theta_2) - sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2)) + omega_y*(sin(theta_1)*cos(rho_1)*cos(rho_2)*cos(theta_2) + sin(theta_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) + sin(rho_2)*cos(theta_1)*cos(theta_2));
   out_2016783461154155129[3] = -omega_x*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2) - omega_y*sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1) + omega_z*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_2)*cos(rho_2)*cos(theta_1));
}
void error_derivatives_12(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_7298908495412623268) {
   out_7298908495412623268[0] = omega_x*(sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) + sin(rho_2)*sin(theta_1)*sin(theta_2)) + omega_y*(-sin(theta_1)*cos(theta_2) - sin(theta_2)*cos(rho_1)*cos(rho_2)*cos(theta_1)) + omega_z*(-sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1));
   out_7298908495412623268[1] = omega_x*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2) + omega_y*sin(rho_1)*sin(theta_1)*sin(theta_2)*cos(rho_2) + omega_z*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_7298908495412623268[2] = omega_x*(sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) - sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(-sin(theta_1)*cos(rho_1)*cos(rho_2)*cos(theta_2) - sin(theta_2)*cos(theta_1)) + omega_z*(sin(rho_1)*sin(theta_1)*sin(theta_2) + sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2));
   out_7298908495412623268[3] = omega_x*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_y*sin(rho_2)*sin(theta_1)*sin(theta_2)*cos(rho_1) + omega_z*sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2);
}
void error_derivatives_22(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_4554644332477482966) {
   out_4554644332477482966[0] = omega_x*(sin(rho_1)*sin(theta_2)*cos(rho_2)*cos(theta_1) - sin(rho_2)*sin(theta_2)*cos(rho_1)*cos(theta_1)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_2) + cos(rho_1)*cos(theta_1)*cos(theta_2)) + omega_z*(-sin(rho_1)*cos(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_1)*sin(theta_2));
   out_4554644332477482966[1] = omega_x*(sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) + sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2)) - omega_y*sin(rho_1)*sin(theta_1)*cos(theta_2) - omega_z*sin(theta_1)*cos(rho_1)*cos(theta_2);
   out_4554644332477482966[2] = omega_x*(sin(rho_1)*sin(theta_1)*cos(rho_2)*cos(theta_2) - sin(rho_2)*sin(theta_1)*cos(rho_1)*cos(theta_2)) + omega_y*(-sin(theta_1)*sin(theta_2)*cos(rho_1) - cos(rho_2)*cos(theta_1)*cos(theta_2)) + omega_z*(sin(rho_1)*sin(theta_1)*sin(theta_2) + sin(rho_2)*cos(theta_1)*cos(theta_2));
   out_4554644332477482966[3] = omega_x*(-sin(rho_1)*sin(rho_2)*sin(theta_1)*sin(theta_2) - sin(theta_1)*sin(theta_2)*cos(rho_1)*cos(rho_2)) + omega_y*sin(rho_2)*sin(theta_2)*cos(theta_1) + omega_z*sin(theta_2)*cos(rho_2)*cos(theta_1);
}
