
double minv1(double *M) {
   double minv_result;
   minv_result[0] = (M[5]*M[10]*M[15] - M[5]*M[11]*M[14] - M[6]*M[9]*M[15] + M[6]*M[11]*M[13] + M[7]*M[9]*M[14] - M[7]*M[10]*M[13])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[1] = (-M[1]*M[10]*M[15] + M[1]*M[11]*M[14] + M[2]*M[9]*M[15] - M[2]*M[11]*M[13] - M[3]*M[9]*M[14] + M[3]*M[10]*M[13])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[2] = (M[1]*M[6]*M[15] - M[1]*M[7]*M[14] - M[2]*M[5]*M[15] + M[2]*M[7]*M[13] + M[3]*M[5]*M[14] - M[3]*M[6]*M[13])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[3] = (-M[1]*M[6]*M[11] + M[1]*M[7]*M[10] + M[2]*M[5]*M[11] - M[2]*M[7]*M[9] - M[3]*M[5]*M[10] + M[3]*M[6]*M[9])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[4] = (-M[4]*M[10]*M[15] + M[4]*M[11]*M[14] + M[6]*M[8]*M[15] - M[6]*M[11]*M[12] - M[7]*M[8]*M[14] + M[7]*M[10]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[5] = (M[0]*M[10]*M[15] - M[0]*M[11]*M[14] - M[2]*M[8]*M[15] + M[2]*M[11]*M[12] + M[3]*M[8]*M[14] - M[3]*M[10]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[6] = (-M[0]*M[6]*M[15] + M[0]*M[7]*M[14] + M[2]*M[4]*M[15] - M[2]*M[7]*M[12] - M[3]*M[4]*M[14] + M[3]*M[6]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[7] = (M[0]*M[6]*M[11] - M[0]*M[7]*M[10] - M[2]*M[4]*M[11] + M[2]*M[7]*M[8] + M[3]*M[4]*M[10] - M[3]*M[6]*M[8])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[8] = (M[4]*M[9]*M[15] - M[4]*M[11]*M[13] - M[5]*M[8]*M[15] + M[5]*M[11]*M[12] + M[7]*M[8]*M[13] - M[7]*M[9]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[9] = (-M[0]*M[9]*M[15] + M[0]*M[11]*M[13] + M[1]*M[8]*M[15] - M[1]*M[11]*M[12] - M[3]*M[8]*M[13] + M[3]*M[9]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[10] = (M[0]*M[5]*M[15] - M[0]*M[7]*M[13] - M[1]*M[4]*M[15] + M[1]*M[7]*M[12] + M[3]*M[4]*M[13] - M[3]*M[5]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[11] = (-M[0]*M[5]*M[11] + M[0]*M[7]*M[9] + M[1]*M[4]*M[11] - M[1]*M[7]*M[8] - M[3]*M[4]*M[9] + M[3]*M[5]*M[8])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[12] = (-M[4]*M[9]*M[14] + M[4]*M[10]*M[13] + M[5]*M[8]*M[14] - M[5]*M[10]*M[12] - M[6]*M[8]*M[13] + M[6]*M[9]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[13] = (M[0]*M[9]*M[14] - M[0]*M[10]*M[13] - M[1]*M[8]*M[14] + M[1]*M[10]*M[12] + M[2]*M[8]*M[13] - M[2]*M[9]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[14] = (-M[0]*M[5]*M[14] + M[0]*M[6]*M[13] + M[1]*M[4]*M[14] - M[1]*M[6]*M[12] - M[2]*M[4]*M[13] + M[2]*M[5]*M[12])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   minv_result[15] = (M[0]*M[5]*M[10] - M[0]*M[6]*M[9] - M[1]*M[4]*M[10] + M[1]*M[6]*M[8] + M[2]*M[4]*M[9] - M[2]*M[5]*M[8])/(M[0]*M[5]*M[10]*M[15] - M[0]*M[5]*M[11]*M[14] - M[0]*M[6]*M[9]*M[15] + M[0]*M[6]*M[11]*M[13] + M[0]*M[7]*M[9]*M[14] - M[0]*M[7]*M[10]*M[13] - M[1]*M[4]*M[10]*M[15] + M[1]*M[4]*M[11]*M[14] + M[1]*M[6]*M[8]*M[15] - M[1]*M[6]*M[11]*M[12] - M[1]*M[7]*M[8]*M[14] + M[1]*M[7]*M[10]*M[12] + M[2]*M[4]*M[9]*M[15] - M[2]*M[4]*M[11]*M[13] - M[2]*M[5]*M[8]*M[15] + M[2]*M[5]*M[11]*M[12] + M[2]*M[7]*M[8]*M[13] - M[2]*M[7]*M[9]*M[12] - M[3]*M[4]*M[9]*M[14] + M[3]*M[4]*M[10]*M[13] + M[3]*M[5]*M[8]*M[14] - M[3]*M[5]*M[10]*M[12] - M[3]*M[6]*M[8]*M[13] + M[3]*M[6]*M[9]*M[12]);
   return minv_result;
}
