#include "spherical.h"
#include <math.h>
void spherical1(double x, double y, double z, double *out_4922184406212571723) {
   out_4922184406212571723[0] = atan2(sqrt(pow(x, 2) + pow(y, 2)), z);
   out_4922184406212571723[1] = atan2(y, x);
}


void spherical2(double x, double y, double z, double *out_5156308728579745157) {
   out_5156308728579745157[0] = atan2(sqrt(pow(y, 2) + pow(z, 2)), x);
   out_5156308728579745157[1] = atan2(y, z);
}
