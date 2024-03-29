/**
 * @file tests.c
 * @author German Moreno Escudero
 * @brief Test file
 * @version 0.1
 * @date 2022-05-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Quaternion.h"
#include "constants.h"
#include "arm_kin.h"
#include "imu.h"
#include "vector3.h"
#include "matrix.h"
#include "database.h"
#include "calib.h"
#include "comms.h"

#include "tst_lib.h"


#define IMUS_CONNECTED (0)


bool tst_math_001() 
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[] = {1.0, 2.0, 3.0};
    double v2[] = {4.0, 5.0, 6.0};
    double v_result[3];
    double v_expected[] = {1.0+4.0, 2.0+5.0, 3.0+6.0};

    testDescription(__FUNCTION__, "Add two vectors ");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = vector3_add(v1, v2, v_result);
    ok &= assert_OK(ret, "vector3_add");
    ok &= assert_vector3Equal(v_result,v_expected,"vector3_add result");

    ret = vector3_add(NULL, v2, v_result);
    ok &= assert_ERROR(ret,"vector3_add invalid arg0");

    ret = vector3_add(v1, NULL, v_result);
    ok &= assert_ERROR(ret,"vector3_add invalid arg1");

    ret = vector3_add(v1, v2, NULL);
    ok &= assert_ERROR(ret,"vector3_add invalid arg2");

    testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_math_002() 
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[] = {1.0, 2.0, 3.0};
    double v2[] = {4.0, 5.0, 6.0};
    double v_result[3];
    double v_expected[] = {1.0-4.0, 2.0-5.0, 3.0-6.0};

    testDescription(__FUNCTION__, "Substract two vectors ");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = vector3_substract(v1, v2, v_result);
    ok &= assert_OK(ret,"vector3_substract");
    ok &= assert_vector3Equal(v_result,v_expected,"vector3_substract result");

    ret = vector3_substract(NULL, v2, v_result);
    ok &= assert_ERROR(ret,"vector3_substract invalid arg0");

    ret = vector3_substract(v1, NULL, v_result);
    ok &= assert_ERROR(ret,"vector3_substract invalid arg1");

    ret = vector3_substract(v1, v2, NULL);
    ok &= assert_ERROR(ret,"vector3_substract invalid arg2");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_003() 
{
    bool ok = true;
    ERROR_CODE ret;
    double v[] = {1.0, 2.0, 3.0};
    double v_result[3];
    double v_expected[] = {-3.0, 2.0, 1.0};

    testDescription(__FUNCTION__, "Rotate a vector 90 degrees in Y axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = vector3_rotate90y(v, v_result);
    ok &= assert_OK(ret, "vector3_rotate90y");
    ok &= assert_vector3Equal(v_result,v_expected,"vector3_rotate90y result");

    ret = vector3_rotate90y(NULL, v_result);
    ok &= assert_ERROR(ret,"vector3_rotate90y invalid arg0");

    ret = vector3_rotate90y(v, NULL);
    ok &= assert_ERROR(ret,"vector3_rotate90y invalid arg1");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_004() 
{
    bool ok = true;
    ERROR_CODE ret;
    double v[] = {1.0, 2.0, 3.0};
    double v_result[3];
    double v_expected[] = {3.0, 2.0, -1.0};

    testDescription(__FUNCTION__, "Rotate a vector -90 degrees in Y axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = vector3_rotateMinus90y(v, v_result);
    ok &= assert_OK(ret,"vector3_rotateMinus90y");
    ok &= assert_vector3Equal(v_result,v_expected,"vector3_rotateMinus90y result");

    ret = vector3_rotateMinus90y(NULL, v_result);
    ok &= assert_ERROR(ret,"vector3_rotateMinus90y invalid arg0");

    ret = vector3_rotateMinus90y(v, NULL);
    ok &= assert_ERROR(ret,"vector3_rotateMinus90y invalid arg1");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_005() 
{
    bool ok = true;
    ERROR_CODE ret;
    Quaternion q_result;

    Quaternion q1 = {.w = 1.0, .v = {0.0,0.0,0.0}};
    Quaternion q1_exp_expected = {.w = E, .v = {0.0,0.0,0.0}};

    Quaternion q2 = { .w = M_SQRT1_2, .v = {M_SQRT1_2, 0.0, 0.0} };
    Quaternion q2_exp_expected = {.w = 1.541863, .v = {1.317538,0.0,0.0}};

    Quaternion q3 = { .w = 0.0, .v = {0.0, 0.0, M_PI} };
    Quaternion q3_exp_expected = {.w = -1.0, .v = {0.0,0.0,0.0}};

    Quaternion q4 = { .w = 0.0, .v = {0.0,0.0,0.0}};
    Quaternion q4_exp_expected = {.w = 1.0, .v = {0.0,0.0,0.0}};

    testDescription(__FUNCTION__, "Compute the exponential of a quaternion");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = quaternion_exponential(q1, &q_result);
    ok &= assert_OK(ret, "quaternion_exponential");
    ok &= assert_quaternion(q_result,q1_exp_expected,"quaternion_exponential result");
    ok &= assert_double(Quaternion_norm(&q_result), E, EPSI, "quaternion_exponential result norm");

    ret = quaternion_exponential(q2, &q_result);
    ok &= assert_OK(ret, "quaternion_exponential");
    ok &= assert_quaternion(q_result,q2_exp_expected,"quaternion_exponential result");
    ok &= assert_double(Quaternion_norm(&q_result), pow(E,M_SQRT1_2), EPSI, "quaternion_exponential result norm");

    ret = quaternion_exponential(q3, &q_result);
    ok &= assert_OK(ret, "quaternion_exponential");
    ok &= assert_quaternion(q_result,q3_exp_expected,"quaternion_exponential result");
    ok &= assert_double(Quaternion_norm(&q_result), 1.0, EPSI, "quaternion_exponential result norm");

    ret = quaternion_exponential(q4, &q_result);
    ok &= assert_OK(ret, "quaternion_exponential");
    ok &= assert_quaternion(q_result,q4_exp_expected,"quaternion_exponential result");
    ok &= assert_double(Quaternion_norm(&q_result), 1.0, EPSI, "quaternion_exponential result norm");

    ret = quaternion_exponential(q1, NULL);
    ok &= assert_ERROR(ret, "quaternion_exponential");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_006()
{
    bool ok = true;
    ERROR_CODE ret;
    Quaternion q_result;
    
    Quaternion q = {.w = 1.0, .v = {0.0,0.0,0.0}};
    double w[] = {0.0,0.0,M_PI};

    double T1 = 1.0;
    Quaternion q1_exp_expected = {.w = 0.0, .v = {0.0,0.0,1.0}};

    double T2 = 0.0;
    Quaternion q2_exp_expected = {.w = 1.0, .v = {0.0,0.0,0.0}};

    double T3 = 0.5;
    Quaternion q3_exp_expected = {.w = M_SQRT1_2, .v = {0.0,0.0,M_SQRT1_2}};

    double T4 = 2.0;
    Quaternion q4_exp_expected = {.w = -1.0, .v = {0.0,0.0,0.0}};


    testDescription(__FUNCTION__, "Apply an angular velocity in Z axis to a quaternion");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    ret = quaternion_ang_vel_apply(q, T1, w, &q_result);
    ok &= assert_quaternion(q_result, q1_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T2, w, &q_result);
    ok &= assert_quaternion(q_result, q2_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T3, w, &q_result);
    ok &= assert_quaternion(q_result, q3_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T4, w, &q_result);
    ok &= assert_quaternion(q_result, q4_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, -1.0, w, &q_result);
    ok &= assert_ERROR(ret, "quaternion_ang_vel_apply");

    ret = quaternion_ang_vel_apply(q, T4, NULL, &q_result);
    ok &= assert_ERROR(ret, "quaternion_ang_vel_apply");

    ret = quaternion_ang_vel_apply(q, T4, w, NULL);
    ok &= assert_ERROR(ret, "quaternion_ang_vel_apply");

    testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_math_007()
{
    bool ok = true;
    ERROR_CODE ret;
    Quaternion q_result;
    
    Quaternion q = {.w = 1.0, .v = {0.0,0.0,0.0}};
    double w[] = {0.0,M_PI,0.0};

    double T1 = 1.0;
    Quaternion q1_exp_expected = {.w = 0.0, .v = {0.0,1.0,0.0}};

    double T2 = 0.0;
    Quaternion q2_exp_expected = {.w = 1.0, .v = {0.0,0.0,0.0}};

    double T3 = 0.5;
    Quaternion q3_exp_expected = {.w = M_SQRT1_2, .v = {0.0,M_SQRT1_2,0.0}};

    double T4 = 2.0;
    Quaternion q4_exp_expected = {.w = -1.0, .v = {0.0,0.0,0.0}};


    testDescription(__FUNCTION__, "Apply an angular velocity in Y axis to a quaternion");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    ret = quaternion_ang_vel_apply(q, T1, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 1");
    ok &= assert_quaternion(q_result, q1_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T2, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 2");
    ok &= assert_quaternion(q_result, q2_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T3, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 3");
    ok &= assert_quaternion(q_result, q3_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T4, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 4");
    ok &= assert_quaternion(q_result, q4_exp_expected,"quaternion_ang_vel_apply result");

    testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_math_008()
{
    bool ok = true;
    ERROR_CODE ret;
    Quaternion q_result;
    
    Quaternion q = {.w = 1.0, .v = {0.0,0.0,0.0}};
    double w[] = {M_PI,0.0,0.0};

    double T1 = 1.0;
    Quaternion q1_exp_expected = {.w = 0.0, .v = {1.0,0.0,0.0}};

    double T2 = 0.0;
    Quaternion q2_exp_expected = {.w = 1.0, .v = {0.0,0.0,0.0}};

    double T3 = 0.5;
    Quaternion q3_exp_expected = {.w = M_SQRT1_2, .v = {M_SQRT1_2,0.0,0.0}};

    double T4 = 2.0;
    Quaternion q4_exp_expected = {.w = -1.0, .v = {0.0,0.0,0.0}};


    testDescription(__FUNCTION__, "Apply an angular velocity in X axis to a quaternion");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    ret = quaternion_ang_vel_apply(q, T1, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 1");
    ok &= assert_quaternion(q_result, q1_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T2, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 2");
    ok &= assert_quaternion(q_result, q2_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T3, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 3");
    ok &= assert_quaternion(q_result, q3_exp_expected,"quaternion_ang_vel_apply result");

    ret = quaternion_ang_vel_apply(q, T4, w, &q_result);
    ok &= assert_OK(ret,"quaternion_ang_vel_apply 4");
    ok &= assert_quaternion(q_result, q4_exp_expected,"quaternion_ang_vel_apply result");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_009()
{
    bool ok = true;
    ERROR_CODE ret;
    Quaternion q_result;
    
    Quaternion q = {.w = 1.0, .v = {0.0,0.0,0.0}};
    double w[] = {0.0,0.0,M_PI};
    double T = 1.0;
    Quaternion q_exp_expected = {.w = 0.0, .v = {0.0,0.0,1.0}};

    double w_zero [] = {0.0,0.0,0.0};


    testDescription(__FUNCTION__, "Check extreme cases of angular velocity aplication for quaternions");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    ret = quaternion_ang_vel_apply(q, T, w, &q_result);
    ok &= assert_quaternion(q_result, q_exp_expected,"quaternion_ang_vel_apply result");
    ok &= assert_double(Quaternion_norm(&q_result), 1.0, EPSI, "quaternion_exponential result norm");

    ret = quaternion_ang_vel_apply(q, T, w_zero, &q_result);
    ok &= assert_quaternion(q_result, q,"quaternion_ang_vel_apply result");
    ok &= assert_double(Quaternion_norm(&q_result), 1.0, EPSI, "quaternion_exponential result norm");

    ret = quaternion_ang_vel_apply(q, -1.0, w, &q_result);
    ok &= assert_ERROR(ret, "quaternion_ang_vel_apply");

    ret = quaternion_ang_vel_apply(q, T, NULL, &q_result);
    ok &= assert_ERROR(ret, "quaternion_ang_vel_apply");

    ret = quaternion_ang_vel_apply(q, T, w, NULL);
    ok &= assert_ERROR(ret, "quaternion_ang_vel_apply");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_010()
{
    bool ok = true;
    MATRIX m, m1, m2;

    testDescription(__FUNCTION__, "Check matrix allocation");
    ok = preconditions_init(__FUNCTION__);

    m = matrix_allocate(4,3);
    ok &= assert_int(m.rows, 4, "rows");
    ok &= assert_int(m.cols, 3, "cols");

    m1 = matrix_identity_allocate(2);
    ok &= assert_int(m1.rows, 2, "rows");
    ok &= assert_int(m1.cols, 2, "cols");

    m2 = matrix_allocate(2,2);
    ok &= assert_int(m2.rows, 2, "rows");
    ok &= assert_int(m2.cols, 2, "cols");
    m2.data[0][0] = 1.0; 
    m2.data[0][1] = 0.0; 
    m2.data[1][0] = 0.0; 
    m2.data[1][1] = 1.0; 

    ok &= assert_matrix(m1,m2,"identity");

    matrix_free(m);
    matrix_free(m1);
    matrix_free(m2);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_011()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m1, m2;

    testDescription(__FUNCTION__, "Check matrix copy");
    ok = preconditions_init(__FUNCTION__);

    m1 = matrix_identity_allocate(3);
    m2 = matrix_allocate(3,3);

    ok &= assert_matrixIdentity(m1, "identity 1");

    ret = matrix_copy(m1, &m2);
    ok &= assert_OK(ret, "matrix_copy");

    ok &= assert_matrixIdentity(m2, "identity 2");

    matrix_free(m1);
    matrix_free(m2);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_012()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m, mt;

    testDescription(__FUNCTION__, "Check matrix transposition");
    ok = preconditions_init(__FUNCTION__);

    m = matrix_identity_allocate(3);
    mt = matrix_allocate(3,3);

    m.data[0][2] = 3.5;
    ok &= assert_double(m.data[2][0], 0.0, EPSI, "before transposing");

    ret = matrix_transpose(m, &mt);
    ok &= assert_OK(ret, "matrix_transpose");

    ok &= assert_double(mt.data[0][2], 0.0, EPSI, "Transposed 0,2");
    ok &= assert_double(mt.data[2][0], 3.5, EPSI, "Transposed 2,0");

    matrix_free(m);
    matrix_free(mt);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_013()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m1, m2, m;
    MATRIX m_expected;

    testDescription(__FUNCTION__, "Check matrix addition");
    ok = preconditions_init(__FUNCTION__);

    m1 = matrix_identity_allocate(3);
    m2 = matrix_identity_allocate(3);
    m  = matrix_allocate(3,3);
    m_expected = matrix_identity_allocate(3);

    m_expected.data[0][0] = 2.0;
    m_expected.data[1][1] = 2.0;
    m_expected.data[2][2] = 2.0;

    ret = matrix_add(m1, m2, &m);
    ok &= assert_OK(ret, "matrix_add");
    ok &= assert_matrix(m, m_expected, "matrix_add result");

    matrix_free(m);
    matrix_free(m1);
    matrix_free(m2);
    matrix_free(m_expected);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_014()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m1, m2, m3;
    MATRIX m_result1, m_result2;
    MATRIX m_expected1, m_expected2;

    testDescription(__FUNCTION__, "Check matrix multiplication");
    ok = preconditions_init(__FUNCTION__);

    m1 = matrix_identity_allocate(2); m1.data[0][1] = 2.0;
    m2 = matrix_identity_allocate(2); m2.data[0][1] = 3.0;
    m3 = matrix_allocate(2,1); 
    m3.data[0][0] = 3.0;
    m3.data[1][0] = 4.0;
    m_result1  = matrix_allocate(2,2);
    m_result2  = matrix_allocate(2,1);
    m_expected1 = matrix_identity_allocate(2);
    m_expected1.data[0][1] = 5.0;
    m_expected2 = matrix_allocate(2,1);
    m_expected2.data[0][0] = 11.0;
    m_expected2.data[1][0] = 4.0;


    ret = matrix_multiply(m1, m2, &m_result1);
    ok &= assert_OK(ret, "matrix_multiply 1");
    ok &= assert_matrix(m_result1, m_expected1, "matrix_multiply 1 result");

    ret = matrix_multiply(m1, m3, &m_result2);
    ok &= assert_OK(ret, "matrix_multiply 2");
    ok &= assert_matrix(m_result2, m_expected2, "matrix_multiply 2 result");

    matrix_free(m1);
    matrix_free(m2);
    matrix_free(m3);
    matrix_free(m_result1);
    matrix_free(m_result2);
    matrix_free(m_expected1);
    matrix_free(m_expected2);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_015()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m, minv;
    MATRIX m_expected;

    testDescription(__FUNCTION__, "Check matrix inversion");
    ok = preconditions_init(__FUNCTION__);

    m    = matrix_allocate(3,3);
    minv = matrix_allocate(3,3);
    m_expected = matrix_allocate(3,3);

    m.data[0][0] = 1.0;    m.data[0][1] = 1.0;    m.data[0][2] = 3.0;
    m.data[1][0] = 1.0;    m.data[1][1] = 3.0;    m.data[1][2] =-3.0;
    m.data[2][0] =-2.0;    m.data[2][1] =-4.0;    m.data[2][2] =-4.0;

    m_expected.data[0][0] = 3.0;    m_expected.data[0][1] = 1.0;    m_expected.data[0][2] = 1.5;
    m_expected.data[1][0] =-1.25;   m_expected.data[1][1] =-0.25;   m_expected.data[1][2] =-0.75;
    m_expected.data[2][0] =-0.25;   m_expected.data[2][1] =-0.25;   m_expected.data[2][2] =-0.25;

    ret = matrix_inverse(m, &minv);
    ok &= assert_OK(ret, "matrix_inverse");
    ok &= assert_matrix(minv, m_expected, "matrix_inverse result");

    matrix_free(m);
    matrix_free(minv);
    matrix_free(m_expected);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_016()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m, mpinv;
    MATRIX m_expected1;
    MATRIX m_expected2;

    testDescription(__FUNCTION__, "Check matrix pseudo inversion with svd");
    ok = preconditions_init(__FUNCTION__);

    m     = matrix_allocate(2,3);
    mpinv = matrix_allocate(3,2);

    m.data[0][0] = 1.0;    m.data[0][1] = 0.0;    m.data[0][2] = 0.0;
    m.data[1][0] = 0.0;    m.data[1][1] = 1.0;    m.data[1][2] =-3.0;

    m_expected1 = matrix_allocate(3,2);
    m_expected1.data[0][0] = 1.0;   m_expected1.data[0][1] = 0.0;
    m_expected1.data[1][0] = 0.0;   m_expected1.data[1][1] = 0.1;
    m_expected1.data[2][0] = 0.0;   m_expected1.data[2][1] =-0.3;

    ret = matrix_pseudoinverse(m, &mpinv);
    ok &= assert_OK(ret, "matrix_pseudoinverse 1");
    ok &= assert_matrix(mpinv, m_expected1, "matrix_pseudoinverse result 1");

    m.data[0][0] = 3.0; m.data[0][1] = 2.0; m.data[0][2] = 2.0;
    m.data[1][0] = 2.0; m.data[1][1] = 3.0; m.data[1][2] =-2.0;

    m_expected2 = matrix_allocate(3,2);
    m_expected2.data[0][0] = 0.15555556;   m_expected2.data[0][1] = 0.04444444;
    m_expected2.data[1][0] = 0.04444444;   m_expected2.data[1][1] = 0.15555556;
    m_expected2.data[2][0] = 0.22222222;   m_expected2.data[2][1] =-0.22222222;

    ret = matrix_pseudoinverse(m, &mpinv);
    ok &= assert_OK(ret, "matrix_pseudoinverse 2");
    ok &= assert_matrix(mpinv, m_expected2, "matrix_pseudoinverse result 2");

    matrix_free(m);
    matrix_free(mpinv);
    matrix_free(m_expected1);
    matrix_free(m_expected2);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_017()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m, mminor;
    MATRIX m_expected;

    testDescription(__FUNCTION__, "Check matrix minor");
    ok = preconditions_init(__FUNCTION__);

    m          = matrix_allocate(3,3);
    mminor     = matrix_allocate(2,2);
    m_expected = matrix_allocate(2,2);

    m.data[0][0] = 1.0;    m.data[0][1] = 0.0;    m.data[0][2] = 0.0;
    m.data[1][0] = 0.0;    m.data[1][1] = 1.0;    m.data[1][2] =-3.0;
    m.data[2][0] = 2.0;    m.data[2][1] = 1.0;    m.data[2][2] = 5.0;

    ret = matrix_minor(m, 0, 0, &mminor);
    ok &= assert_OK(ret, "matrix_minor 1");
    m_expected.data[0][0] = 1.0;   m_expected.data[0][1] =-3.0;
    m_expected.data[1][0] = 1.0;   m_expected.data[1][1] = 5.0;
    ok &= assert_matrix(mminor, m_expected, "matrix_minor result 1");

    ret = matrix_minor(m, 1, 2, &mminor);
    ok &= assert_OK(ret, "matrix_minor 2");
    m_expected.data[0][0] = 1.0;   m_expected.data[0][1] = 0.0;
    m_expected.data[1][0] = 2.0;   m_expected.data[1][1] = 1.0;
    ok &= assert_matrix(mminor, m_expected, "matrix_minor result 2");

    ret = matrix_minor(m, 2, 2, &mminor);
    ok &= assert_OK(ret, "matrix_minor 3");
    m_expected.data[0][0] = 1.0;   m_expected.data[0][1] = 0.0;
    m_expected.data[1][0] = 0.0;   m_expected.data[1][1] = 1.0;
    ok &= assert_matrix(mminor, m_expected, "matrix_minor result 3");

    matrix_free(m);
    matrix_free(mminor);
    matrix_free(m_expected);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_018()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m1, m2, m3, m4, m5;
    double det;
    
    testDescription(__FUNCTION__, "Check matrix determinant");
    ok = preconditions_init(__FUNCTION__);

    m1    = matrix_identity_allocate(1);
    m2    = matrix_identity_allocate(2);
    m3    = matrix_identity_allocate(3);
    m4    = matrix_identity_allocate(4);
    m5    = matrix_identity_allocate(5);

    ret = matrix_determinant(m1, &det);
    ok &= assert_OK(ret, "matrix_determinant 1");
    ok &= assert_double(det, 1.0, EPSI, "matrix_determinant result 1");

    m2.data[0][0] = 1.0; m2.data[0][1] = 2.0;
    m2.data[1][0] = 3.0; m2.data[1][1] = 4.0;   
    ret = matrix_determinant(m2, &det);
    ok &= assert_OK(ret, "matrix_determinant 2");
    ok &= assert_double(det,-2.0, EPSI, "matrix_determinant result 2");

    m3.data[0][0] = 1.0; m3.data[0][1] = 2.0; m3.data[0][2] = 3.0;
    m3.data[1][0] = 4.0; m3.data[1][1] = 5.0; m3.data[1][2] = 6.0;
    m3.data[2][0] = 7.0; m3.data[2][1] = 8.0; m3.data[2][2] = 10.0;
    ret = matrix_determinant(m3, &det);
    ok &= assert_OK(ret, "matrix_determinant 3");
    ok &= assert_double(det, -3.0, EPSI, "matrix_determinant result 3");

    m4.data[0][0] = 2.0;  m4.data[0][1] = 2.0;  m4.data[0][2] = 3.0;  m4.data[0][3] = 0.0;  
    m4.data[1][0] = 6.0;  m4.data[1][1] = 8.0;  m4.data[1][2] = 8.0;  m4.data[1][3] = 9.0;  
    m4.data[2][0] = 14.0; m4.data[2][1] = 13.0; m4.data[2][2] = 13.0; m4.data[2][3] = 11.0; 
    m4.data[3][0] = 15.0; m4.data[3][1] = 16.0; m4.data[3][2] = 17.0; m4.data[3][3] = 19.0; 
    ret = matrix_determinant(m4, &det); 
    ok &= assert_OK(ret, "matrix_determinant 4");
    ok &= assert_double(det,-183.0, EPSI, "matrix_determinant result 4");

    m5.data[0][0] = 2.0;  m5.data[0][1] = 2.0;  m5.data[0][2] = 3.0;  m5.data[0][3] = 0.0;  m5.data[0][4] = 5.0; 
    m5.data[1][0] = 6.0;  m5.data[1][1] = 8.0;  m5.data[1][2] = 8.0;  m5.data[1][3] = 9.0;  m5.data[1][4] = 10.0; 
    m5.data[2][0] = 14.0; m5.data[2][1] = 13.0; m5.data[2][2] = 13.0; m5.data[2][3] = 11.0; m5.data[2][4] = 11.0; 
    m5.data[3][0] = 15.0; m5.data[3][1] = 16.0; m5.data[3][2] = 17.0; m5.data[3][3] = 19.0; m5.data[3][4] = 19.0; 
    m5.data[4][0] = 24.0; m5.data[4][1] = 23.0; m5.data[4][2] = 22.0; m5.data[4][3] = 21.0; m5.data[4][4] = 20.0; 
    ret = matrix_determinant(m5, &det); 
    ok &= assert_OK(ret, "matrix_determinant 5");
    ok &= assert_double(det,-482.0, EPSI, "matrix_determinant result 5");

    matrix_free(m1);
    matrix_free(m2);
    matrix_free(m3);
    matrix_free(m4);
    matrix_free(m5);
    testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_math_019()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX m1, m2;
    MATRIX m1adj, m2adj;
    MATRIX m1exp, m2exp;
    
    testDescription(__FUNCTION__, "Check matrix adjoint");
    ok = preconditions_init(__FUNCTION__);

    m1    = matrix_identity_allocate(3);
    m2    = matrix_identity_allocate(5);

    m1adj = matrix_from_matrix_allocate(m1);
    m2adj = matrix_from_matrix_allocate(m2);

    m1exp = matrix_from_matrix_allocate(m1);
    m2exp = matrix_from_matrix_allocate(m2);

    m1.data[0][0] = 1.0; m1.data[0][1] = 2.0; m1.data[0][2] = 3.0;
    m1.data[1][0] = 4.0; m1.data[1][1] = 5.0; m1.data[1][2] = 6.0;
    m1.data[2][0] = 7.0; m1.data[2][1] = 8.0; m1.data[2][2] = 10.0;
    ret = matrix_adjoint(m1, &m1adj);
    ok &= assert_OK(ret, "matrix_adjoint 1");
    m1exp.data[0][0] = 2.0; m1exp.data[0][1] = 4.0;  m1exp.data[0][2] =-3.0;
    m1exp.data[1][0] = 2.0; m1exp.data[1][1] =-11.0; m1exp.data[1][2] = 6.0;
    m1exp.data[2][0] =-3.0; m1exp.data[2][1] = 6.0;  m1exp.data[2][2] =-3.0;
    ok &= assert_matrix(m1adj, m1exp, "matrix_adjoint result 1");

    m2.data[0][0] = 2.0;  m2.data[0][1] = 2.0;  m2.data[0][2] = 3.0;  m2.data[0][3] = 0.0;  m2.data[0][4] = 5.0; 
    m2.data[1][0] = 6.0;  m2.data[1][1] = 8.0;  m2.data[1][2] = 8.0;  m2.data[1][3] = 9.0;  m2.data[1][4] = 10.0; 
    m2.data[2][0] = 14.0; m2.data[2][1] = 13.0; m2.data[2][2] = 13.0; m2.data[2][3] = 11.0; m2.data[2][4] = 11.0; 
    m2.data[3][0] = 15.0; m2.data[3][1] = 16.0; m2.data[3][2] = 17.0; m2.data[3][3] = 19.0; m2.data[3][4] = 19.0; 
    m2.data[4][0] = 24.0; m2.data[4][1] = 23.0; m2.data[4][2] = 22.0; m2.data[4][3] = 21.0; m2.data[4][4] = 20.0; 
    ret = matrix_adjoint(m2, &m2adj); 
    ok &= assert_OK(ret, "matrix_adjoint 2");
    m2exp.data[0][0] =-120.0; m2exp.data[0][1] = 362.0; m2exp.data[0][2] = 362.0; m2exp.data[0][3] =-118.0; m2exp.data[0][4] =-238.0; 
    m2exp.data[1][0] = 60.0;  m2exp.data[1][1] =-422.0; m2exp.data[1][2] = 60.0;  m2exp.data[1][3] = 300.0; m2exp.data[1][4] =-122.0; 
    m2exp.data[2][0] = 131.0; m2exp.data[2][1] =-110.0; m2exp.data[2][2] =-833.0; m2exp.data[2][3] =-68.0;  m2exp.data[2][4] = 545.0; 
    m2exp.data[3][0] = 78.0;  m2exp.data[3][1] = 78.0;  m2exp.data[3][2] = 78.0;  m2exp.data[3][3] =-92.0;  m2exp.data[3][4] =-14.0; 
    m2exp.data[4][0] =-151.0; m2exp.data[4][1] = 90.0;  m2exp.data[4][2] = 331.0; m2exp.data[4][3] =-32.0;  m2exp.data[4][4] =-183.0; 
    ok &= assert_matrix(m2adj, m2exp, "matrix_adjoint result 2");

    matrix_free(m1);
    matrix_free(m2);
    matrix_free(m1adj);
    matrix_free(m2adj);
    matrix_free(m1exp);
    matrix_free(m2exp);
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_020() 
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[] = {1.0, 0.0, 0.0};
    double v2[] = {0.0, 0.0, 1.0};
    double v3[] = {M_SQRT1_2, M_SQRT1_2, 0.0};
    double v4[] = {M_SQRT1_2, -M_SQRT1_2, 0.0};
    double angle;
    double angle_expected1 = M_PI_2;
    double angle_expected2 = M_PI_4;
    double angle_expected3 = M_PI_4;

    testDescription(__FUNCTION__, "Check the angle calcualtion between two vectors");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = vector3_angle_between_vectors_calculate(v1, v2, &angle);
    ok &= assert_OK(ret,"vector3_angle_between_vectors_calculate 1");
    ok &= assert_double(angle,angle_expected1, EPSI, "vector3_angle_between_vectors_calculate result 1");

    ret = vector3_angle_between_vectors_calculate(v1, v3, &angle);
    ok &= assert_OK(ret,"vector3_angle_between_vectors_calculate 2");
    ok &= assert_double(angle,angle_expected2, EPSI, "vector3_angle_between_vectors_calculate result 2");

    ret = vector3_angle_between_vectors_calculate(v1, v4, &angle);
    ok &= assert_OK(ret,"vector3_angle_between_vectors_calculate 3");
    ok &= assert_double(angle,angle_expected3, EPSI, "vector3_angle_between_vectors_calculate result 3");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_021()
{
    bool ok = true;
    double eulerAngles[3];
    double zyx_result[3];
    Quaternion q_result;
    Quaternion expected1 = {.w = 0.8535534, .v = {0.3535534, 0.3535534,-0.1464466}};
    Quaternion expected2 = {.w = 0.5, .v = {0.5, 0.5, 0.5}};

    testDescription(__FUNCTION__, "Check conversions between euler ZYX and quaternion");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    eulerAngles[0] = M_PI_4; eulerAngles[1] = M_PI_4; eulerAngles[2] = 0.0; 
    Quaternion_fromEulerZYX(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected1,"Quaternion_fromEulerZYX result 1");
    Quaternion_toEulerZYX(&q_result,zyx_result);
    ok &= assert_vector3Equal(zyx_result,eulerAngles,"Quaternion_toEulerZYX result 1");

    eulerAngles[0] = M_PI_2; eulerAngles[1] = 0.0; eulerAngles[2] = M_PI_2; 
    Quaternion_fromEulerZYX(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected2,"Quaternion_fromEulerZYX result 2");
    Quaternion_toEulerZYX(&q_result,zyx_result);
    ok &= assert_vector3Equal(zyx_result,eulerAngles,"Quaternion_toEulerZYX result 2");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_022()
{
    bool ok = true;
    double eulerAngles[3];
    double zxy_result[3];
    Quaternion q_result;
    Quaternion expected0 = {.w = 1.0, .v = {0.0, 0.0, 0.0}};
    Quaternion expected1 = {.w = 0.8535534, .v = {0.3535534, 0.3535534, 0.1464466}};
    Quaternion expected2 = {.w = 0.5, .v = {-0.5, 0.5, 0.5}};
    Quaternion expected3 = {.w = M_SQRT1_2, .v = {0.0,M_SQRT1_2,0.0}};
    Quaternion expected4 = {.w = M_SQRT1_2, .v = {M_SQRT1_2,0.0,0.0}};
    Quaternion expected5 = {.w = M_SQRT1_2, .v = {0.0,0.0,M_SQRT1_2}};

    testDescription(__FUNCTION__, "Check conversions between euler ZXY and quaternion");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    eulerAngles[0] = 0.0; eulerAngles[1] = 0.0; eulerAngles[2] = 0.0; 
    quaternion_fromEulerZXY(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected0,"quaternion_fromEulerZXY result 0");
    quaternion_toEulerZXY(&q_result,zxy_result);
    ok &= assert_vector3Equal(zxy_result,eulerAngles,"quaternion_toEulerZXY result 0");

    eulerAngles[0] = M_PI_4; eulerAngles[1] = M_PI_4; eulerAngles[2] = 0.0; 
    quaternion_fromEulerZXY(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected1,"quaternion_fromEulerZXY result 1");
    quaternion_toEulerZXY(&q_result,zxy_result);
    ok &= assert_vector3Equal(zxy_result,eulerAngles,"quaternion_toEulerZXY result 1");

    eulerAngles[0] = M_PI_2; eulerAngles[1] = 0.0; eulerAngles[2] = M_PI_2; 
    quaternion_fromEulerZXY(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected2,"quaternion_fromEulerZXY result 2");
    quaternion_toEulerZXY(&q_result,zxy_result);
    ok &= assert_vector3Equal(zxy_result,eulerAngles,"quaternion_toEulerZXY result 2");

    eulerAngles[0] = M_PI_2; eulerAngles[1] = 0.0; eulerAngles[2] = 0.0; 
    quaternion_fromEulerZXY(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected3,"quaternion_fromEulerZXY result 3");
    quaternion_toEulerZXY(&q_result,zxy_result);
    ok &= assert_vector3Equal(zxy_result,eulerAngles,"quaternion_toEulerZXY result 3");

    eulerAngles[0] = 0.0; eulerAngles[1] = M_PI_2; eulerAngles[2] = 0.0; 
    quaternion_fromEulerZXY(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected4,"quaternion_fromEulerZXY result 4");
    quaternion_toEulerZXY(&q_result,zxy_result);
    ok &= assert_vector3Equal(zxy_result,eulerAngles,"quaternion_toEulerZXY result 4");

    eulerAngles[0] = 0.0; eulerAngles[1] = 0.0; eulerAngles[2] = M_PI_2; 
    quaternion_fromEulerZXY(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected5,"quaternion_fromEulerZXY result 5");
    quaternion_toEulerZXY(&q_result,zxy_result);
    ok &= assert_vector3Equal(zxy_result,eulerAngles,"quaternion_toEulerZXY result 5");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_023()
{
    bool ok = true;
    double eulerAngles[3];
    double yxz_result[3];
    Quaternion q_result;
    Quaternion expected0 = {.w = 1.0, .v = {0.0, 0.0, 0.0}};
    Quaternion expected1 = {.w = 0.8535534, .v = {0.3535534, -0.1464466, 0.3535534}};
    Quaternion expected2 = {.w = 0.5, .v = {0.5, 0.5, 0.5}};
    Quaternion expected3 = {.w = M_SQRT1_2, .v = {0.0,0.0,M_SQRT1_2}};
    Quaternion expected4 = {.w = M_SQRT1_2, .v = {M_SQRT1_2,0.0,0.0}};
    Quaternion expected5 = {.w = M_SQRT1_2, .v = {0.0,M_SQRT1_2,0.0}};
    Quaternion expected6 = {.w = 0.89239910083252272, .v = {0.099045760541287622,0.23911761839433451,0.36964381061438617}};

    testDescription(__FUNCTION__, "Check conversions between euler YXZ and quaternion");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    eulerAngles[0] = 0.0; eulerAngles[1] = 0.0; eulerAngles[2] = 0.0; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected0,"quaternion_fromEulerYXZ result 0");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 0");

    eulerAngles[0] = M_PI_4; eulerAngles[1] = M_PI_4; eulerAngles[2] = 0.0; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected1,"quaternion_fromEulerYXZ result 1");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 1");

    eulerAngles[0] = M_PI_2; eulerAngles[1] = 0.0; eulerAngles[2] = M_PI_2; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected2,"quaternion_fromEulerYXZ result 2");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 2");

    eulerAngles[0] = M_PI_2; eulerAngles[1] = 0.0; eulerAngles[2] = 0.0; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected3,"quaternion_fromEulerYXZ result 3");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 3");

    eulerAngles[0] = 0.0; eulerAngles[1] = M_PI_2; eulerAngles[2] = 0.0; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected4,"quaternion_fromEulerYXZ result 4");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 4");

    eulerAngles[0] = 0.0; eulerAngles[1] = 0.0; eulerAngles[2] = M_PI_2; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected5,"quaternion_fromEulerYXZ result 5");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 5");

    eulerAngles[0] = M_PI_4; eulerAngles[1] = 0.0; eulerAngles[2] = PI/6; 
    quaternion_fromEulerYXZ(eulerAngles,&q_result);
    ok &= assert_quaternion(q_result,expected6,"quaternion_fromEulerYXZ result 6");
    quaternion_toEulerYXZ(&q_result,yxz_result);
    ok &= assert_vector3Equal(yxz_result,eulerAngles,"quaternion_toEulerYXZ result 6");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_024()
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[] = {1.0, 0.0, 0.0};
    double v2[] = {0.0, 1.0, 0.0};
    double v3[] = {0.0, 0.0, 1.0};
    double dotVal;
    double expected123 = 1.0;
    double expected456 = 0.0;

    testDescription(__FUNCTION__, "Check the vector dot product");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    ret = vector3_dot(v1,v1,&dotVal);
    ok &= assert_OK(ret, "vector3_dot 1");
    ok &= assert_double(dotVal,expected123,EPSI,"vector3_dot result 1");

    ret = vector3_dot(v2,v2,&dotVal);
    ok &= assert_OK(ret, "vector3_dot 2");
    ok &= assert_double(dotVal,expected123,EPSI,"vector3_dot result 2");

    ret = vector3_dot(v3,v3,&dotVal);
    ok &= assert_OK(ret, "vector3_dot 3");
    ok &= assert_double(dotVal,expected123,EPSI,"vector3_dot result 3");

    ret = vector3_dot(v1,v2,&dotVal);
    ok &= assert_OK(ret, "vector3_dot 4");
    ok &= assert_double(dotVal,expected456,EPSI,"vector3_dot result 4");

    ret = vector3_dot(v2,v3,&dotVal);
    ok &= assert_OK(ret, "vector3_dot 5");
    ok &= assert_double(dotVal,expected456,EPSI,"vector3_dot result 5");

    ret = vector3_dot(v3,v1,&dotVal);
    ok &= assert_OK(ret, "vector3_dot 6");
    ok &= assert_double(dotVal,expected456,EPSI,"vector3_dot result 6");

    for (double ang = 0; ok && (ang < 2.0*M_PI); ang += M_PI/6) {
        Quaternion q;
        double rotV[3];
        Quaternion_fromXRotation(ang,&q);
        Quaternion_rotate(&q,v3,rotV);

        ret = vector3_dot(v3,rotV,&dotVal);
        ok &= assert_OK(ret, "vector3_dot 7");
        ok &= assert_double(dotVal,cos(ang),EPSI,"vector3_dot result 7");
        if (!ok) tst_str("Failed for angle %f, with dot val %d, when expected was %d",ang,dotVal,cos(ang));
    }

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_025()
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[] = {1.0, 0.0, 0.0};
    double v2[] = {0.0, 1.0, 0.0};
    double v3[] = {0.0, 0.0, 1.0};
    double crossVal[3];
    double expected123 = 0.0;
    double expected456 = 1.0;

    testDescription(__FUNCTION__, "Check the vector cross product");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    ret = vector3_cross(v1,v1,crossVal);
    ok &= assert_OK(ret, "vector3_cross 1");
    ok &= assert_vector3Norm(crossVal,expected123,EPSI,"vector3_cross result 1");

    ret = vector3_cross(v2,v2,crossVal);
    ok &= assert_OK(ret, "vector3_cross 2");
    ok &= assert_vector3Norm(crossVal,expected123,EPSI,"vector3_cross result 2");

    ret = vector3_cross(v3,v3,crossVal);
    ok &= assert_OK(ret, "vector3_cross 3");
    ok &= assert_vector3Norm(crossVal,expected123,EPSI,"vector3_cross result 3");

    ret = vector3_cross(v1,v2,crossVal);
    ok &= assert_OK(ret, "vector3_cross 4");
    ok &= assert_vector3Norm(crossVal,expected456,EPSI,"vector3_cross result 4");

    ret = vector3_cross(v2,v3,crossVal);
    ok &= assert_OK(ret, "vector3_cross 5");
    ok &= assert_vector3Norm(crossVal,expected456,EPSI,"vector3_cross result 5");

    ret = vector3_cross(v3,v1,crossVal);
    ok &= assert_OK(ret, "vector3_cross 6");
    ok &= assert_vector3Norm(crossVal,expected456,EPSI,"vector3_cross result 6");

    for (double ang = 0; ok && (ang < 2.0*M_PI); ang += M_PI/6) {
        Quaternion q;
        double rotV[3];
        Quaternion_fromXRotation(ang,&q);
        Quaternion_rotate(&q,v3,rotV);

        ret = vector3_cross(v3,rotV,crossVal);
        ok &= assert_OK(ret, "vector3_cross 7");
        ok &= assert_vector3Norm(crossVal,fabs(sin(ang)),EPSI,"vector3_cross result 7");
        if (!ok) tst_str("Failed for angle %f",ang);
    }

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_026()
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[3] = {1.0, 0.0, 0.0};
    double v2[3] = {0.0, 1.0, 0.0};
    double v_aux[3];
    Quaternion result;
    Quaternion expected;

    testDescription(__FUNCTION__, "Check the quaternion between two vectors computation");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    ret = quaternion_between_two_vectors_compute(v1,v2,&result);
    ok &= assert_OK(ret,"quaternion_between_two_vectors_compute 1");
    Quaternion_fromZRotation(M_PI_2,&expected);
    ok &= assert_quaternion(result,expected,"quaternion_between_two_vectors_compute result 1");

    ret = quaternion_between_two_vectors_compute(v2,v1,&result);
    ok &= assert_OK(ret,"quaternion_between_two_vectors_compute 2");
    Quaternion_fromZRotation(-M_PI_2,&expected);
    ok &= assert_quaternion(result,expected,"quaternion_between_two_vectors_compute result 2");

    v2[0] = cos(M_PI_2+M_PI_4); v2[1] = sin(M_PI_2+M_PI_4); v2[2] = 0;
    ret = quaternion_between_two_vectors_compute(v1,v2,&result);
    ok &= assert_OK(ret,"quaternion_between_two_vectors_compute 3");
    Quaternion_fromZRotation(M_PI_2+M_PI_4,&expected);
    ok &= assert_quaternion(result,expected,"quaternion_between_two_vectors_compute result 3");

    for (int i = 0; ok && i < 10; i++) {
        tstRandomUnitVector3Generate(v1);
        for (int j = 0; ok && j < 10; j++) {
            tstRandomUnitVector3Generate(v2);
            ret = quaternion_between_two_vectors_compute(v1,v2,&result);
            ok &= assert_OK(ret,"quaternion_between_two_vectors_compute 4");
            Quaternion_rotate(&result,v1,v_aux);
            ok &= assert_vector3Equal(v_aux,v2,"quaternion_between_two_vectors_compute result 4");
        }
    }

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_027()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX A = matrix_allocate(2,2);
    MATRIX b = matrix_allocate(2,1);
    MATRIX guess = matrix_allocate(2,1);
    double tolerance = 1e-9;
    MATRIX expected = matrix_from_matrix_allocate(guess);

    testDescription(__FUNCTION__, "Check the linear equations system solver");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    A.data[0][0] = 2.0; A.data[0][1] = 1.0; 
    A.data[1][0] = 5.0; A.data[1][1] = 7.0;
    b.data[0][0] = 11.0; b.data[1][0] = 13.0;
    guess.data[0][0] = 1.0; guess.data[1][0] = 1.0;

    ret = matrix_linear_system_solve(A,b,tolerance,&guess);
    ok &= assert_OK(ret,"matrix_linear_system_solve");

    expected.data[0][0] = 7.1111111111; expected.data[1][0] = -3.2222222222222;
    ok &= assert_matrix(guess,expected,"matrix_linear_system_solve result");

    matrix_free(A);
    matrix_free(b);
    matrix_free(guess);
    matrix_free(expected);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_028()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX A = matrix_allocate(4,4);
    double eigenvalues[4] = {0};
    MATRIX eigenvectors = matrix_allocate(4,4);

    double expected_eigenvalues[4] = {2585.25381032832231, 37.1014913651276582, 1.4780548441181369, 0.1666428611718905};

    testDescription(__FUNCTION__, "Check the eigen values calculation");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    A.data[0][0] =   4.0; A.data[0][1] =  -30.0; A.data[0][2] =    60.0; A.data[0][3] =   -35.0;
    A.data[1][0] = -30.0; A.data[1][1] =  300.0; A.data[1][2] =  -675.0; A.data[1][3] =   420.0;
    A.data[2][0] =  60.0; A.data[2][1] = -675.0; A.data[2][2] =  1620.0; A.data[2][3] = -1050.0;
    A.data[3][0] = -35.0; A.data[3][1] =  420.0; A.data[3][2] = -1050.0; A.data[3][3] =   700.0;

    ret = matrix_eigen(A,eigenvalues,&eigenvectors);
    ok &= assert_OK(ret,"matrix_eigen");
    for (int i=0; i<4; i++) {
        ok &= assert_double(eigenvalues[i],expected_eigenvalues[i],1e-5,"matrix_eigen result");
    }
    matrix_print(eigenvectors,"eigenvectors matrix");

    matrix_free(A);
    matrix_free(eigenvectors);

    testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_math_029()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX A = matrix_allocate(2,3);
    MATRIX expected_S = matrix_allocate(2,3);

    testDescription(__FUNCTION__, "Check the Singular Value Decomposition calculation");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    A.data[0][0] = 3.0; A.data[0][1] = 2.0; A.data[0][2] = 2.0;
    A.data[1][0] = 2.0; A.data[1][1] = 3.0; A.data[1][2] =-2.0;
    MATRIX_SVD svd = matrix_svd_allocate_and_set(A);
    ok &= assert_bool(svd.set, true, "matrix_svd_allocate_and_set result.set");
    matrix_print(svd.Sigma,"matrix_svd_allocate_and_set result.Sigma");
    matrix_print(svd.U,"matrix_svd_allocate_and_set result.U");
    matrix_print(svd.V,"matrix_svd_allocate_and_set result.V");

    expected_S.data[0][0] = 5.0; expected_S.data[0][1] = 0.0; expected_S.data[0][2] = 0.0;
    expected_S.data[1][0] = 0.0; expected_S.data[1][1] = 3.0; expected_S.data[1][2] = 0.0;
    ok &= assert_matrix(svd.Sigma,expected_S,"matrix_svd_allocate_and_set svd.Sigma");

    MATRIX Vt = matrix_allocate(3,3);
    MATRIX S_Vt = matrix_allocate(2,3);
    MATRIX U_S_Vt = matrix_allocate(2,3);

    ret = matrix_transpose(svd.V,&Vt);
    ok &= assert_OK(ret, "matrix_transpose Vt");
    ret = matrix_multiply(svd.Sigma,Vt, &S_Vt);
    ok &= assert_OK(ret, "matrix_multiply S_Vt");
    ret = matrix_multiply(svd.U,S_Vt, &U_S_Vt);
    ok &= assert_OK(ret, "matrix_multiply U_S_Vt");

    ok &= assert_matrix(U_S_Vt,A,"matrix_svd_allocate_and_set U_S_Vt");

    matrix_free(Vt);
    matrix_free(S_Vt);
    matrix_free(U_S_Vt);
    matrix_svd_free(svd);
    matrix_free(expected_S);
    matrix_free(A);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_030()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX A1 = matrix_allocate(3,3);
    MATRIX expected_A1 = matrix_allocate(3,3);
    MATRIX A2 = matrix_allocate(3,2);
    MATRIX expected_A2 = matrix_allocate(3,2);
    MATRIX A3 = matrix_allocate(3,3);
    MATRIX expected_A3 = matrix_allocate(3,3);
    MATRIX Q3 = matrix_allocate(3,3);
    MATRIX expected_Q3 = matrix_allocate(3,3);

    testDescription(__FUNCTION__, "Check the reduction to upper triangular form");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    A1.data[0][0] = 12.0; A1.data[0][1] = -51.0; A1.data[0][2] =   4.0;
    A1.data[1][0] =  6.0; A1.data[1][1] = 167.0; A1.data[1][2] = -68.0;
    A1.data[2][0] = -4.0; A1.data[2][1] =  24.0; A1.data[2][2] = -41.0;
    ret = matrix_upper_triangular(A1,&A1,NULL);
    ok &= assert_OK(ret,"matrix_upper_triangular 1");
    expected_A1.data[0][0] = 14.0; expected_A1.data[0][1] =  21.0; expected_A1.data[0][2] = -14.0;
    expected_A1.data[1][0] =  0.0; expected_A1.data[1][1] = 175.0; expected_A1.data[1][2] = -70.0;
    expected_A1.data[2][0] =  0.0; expected_A1.data[2][1] =   0.0; expected_A1.data[2][2] = -35.0;
    ok &= assert_matrix(A1,expected_A1,"matrix_upper_triangular result 1");

    A2.data[0][0] = 12.0; A2.data[0][1] = -51.0; 
    A2.data[1][0] =  6.0; A2.data[1][1] = 167.0; 
    A2.data[2][0] = -4.0; A2.data[2][1] =  24.0; 
    ret = matrix_upper_triangular(A2,&A2,NULL);
    ok &= assert_OK(ret,"matrix_upper_triangular 2");
    expected_A2.data[0][0] = 14.0; expected_A2.data[0][1] =  21.0;
    expected_A2.data[1][0] =  0.0; expected_A2.data[1][1] = 175.0;
    expected_A2.data[2][0] =  0.0; expected_A2.data[2][1] =   0.0;
    ok &= assert_matrix(A2,expected_A2,"matrix_upper_triangular result 2");

    A3.data[0][0] = 6.0; A3.data[0][1] = 5.0; A3.data[0][2] = 0.0;
    A3.data[1][0] = 5.0; A3.data[1][1] = 1.0; A3.data[1][2] = 4.0;
    A3.data[2][0] = 0.0; A3.data[2][1] = 4.0; A3.data[2][2] = 3.0;
    ret = matrix_upper_triangular(A3,&A3,&Q3);
    ok &= assert_OK(ret,"matrix_upper_triangular 3");
    expected_A3.data[0][0] = 7.810250; expected_A3.data[0][1] = 4.481291; expected_A3.data[0][2] = 2.560738;
    expected_A3.data[1][0] =      0.0; expected_A3.data[1][1] = 4.681670; expected_A3.data[1][2] = 0.966448;
    expected_A3.data[2][0] =      0.0; expected_A3.data[2][1] =      0.0; expected_A3.data[2][2] =-4.184328;
    ok &= assert_matrix(A3,expected_A3,"matrix_upper_triangular result 3");
    expected_Q3.data[0][0] = 0.768221; expected_Q3.data[0][1] = 0.640184; expected_Q3.data[0][2] = 0.0;
    expected_Q3.data[1][0] = 0.332654; expected_Q3.data[1][1] =-0.399185; expected_Q3.data[1][2] = 0.854396;
    expected_Q3.data[2][0] = 0.546971; expected_Q3.data[2][1] =-0.656365; expected_Q3.data[2][2] =-0.519622;
    ok &= assert_matrix(Q3,expected_Q3,"matrix_upper_triangular transform 3");

    matrix_free(expected_A1);
    matrix_free(A1);
    matrix_free(expected_A2);
    matrix_free(A2);
    matrix_free(expected_A3);
    matrix_free(expected_Q3);
    matrix_free(A3);
    matrix_free(Q3);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_math_031()
{
    bool ok = true;
    ERROR_CODE ret;
    MATRIX A1 = matrix_allocate(3,3);
    MATRIX expected_A1 = matrix_allocate(3,3);

    testDescription(__FUNCTION__, "Check the reduction to bidiagonal form");
    ok = preconditions_init(__FUNCTION__); 

    // Test steps
    A1.data[0][0] = 1.0; A1.data[0][1] = 5.0; A1.data[0][2] =  3.0;
    A1.data[1][0] = 1.0; A1.data[1][1] = 0.0; A1.data[1][2] = -7.0;
    A1.data[2][0] = 3.0; A1.data[2][1] = 8.0; A1.data[2][2] =  9.0;
    ret = matrix_upper_bidiagonal(A1,&A1,NULL,NULL);
    ok &= assert_OK(ret,"matrix_upper_bidiagonal 1");
    expected_A1.data[0][0] = 3.316625; expected_A1.data[0][1] = 11.159993; expected_A1.data[0][2] =  0.0;
    expected_A1.data[1][0] = 0.0     ; expected_A1.data[1][1] =  8.274961; expected_A1.data[1][2] =  5.336122;
    expected_A1.data[2][0] = 0.0     ; expected_A1.data[2][1] =  0.0     ; expected_A1.data[2][2] = -2.550561;
    ok &= assert_matrix(A1,expected_A1,"matrix_upper_bidiagonal result 1");

    matrix_free(expected_A1);
    matrix_free(A1);

    testCleanUp();
    testReport(ok);
    return ok;
}



bool tst_db_001()
{
    bool ok = true;
    ERROR_CODE ret;

    testDescription(__FUNCTION__, "Initialize database");
    ok = preconditions_initTraces(__FUNCTION__); 

    // Test Steps
    ret = db_initialize();
    ok &= assert_OK(ret,"db_initialize");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_002()
{
    bool ok = true;
    ERROR_CODE ret;

    testDescription(__FUNCTION__, "Terminate database");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_terminate();
    ok &= assert_OK(ret,"db_terminate");

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_003()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_ACCELEROMETER;
    int instance = 0;
    double buff[3];
    double buff_expected[] = {0.0, 0.0, 0.0};

    testDescription(__FUNCTION__, "Read default value from the database");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_read(field,instance,buff);
    ok &= assert_OK(ret,"db_read");
    ok &= assert_vector3Equal(buff, buff_expected,"db_read result");

    ret = db_read(DB_NUMBER_OF_ENTRIES,instance,buff);
    ok &= assert_ERROR(ret,"db_read invalid arg0");

    ret = db_read(field,10000,buff);
    ok &= assert_ERROR(ret,"db_read invalid arg1");

    ret = db_read(field,instance,NULL);
    ok &= assert_ERROR(ret,"db_read invalid arg2");

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_004()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
    int instance = 0;
    double buff[4] = {1.0, 2.0, 3.0, 4.0};
    double buff_expected[] = {1.0, 2.0, 3.0, 4.0};

    testDescription(__FUNCTION__, "Write values to a field of the database");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_write(field,instance,buff);
    ok &= assert_OK(ret,"db_write");
    ok &= assert_dbFieldDouble(field, instance, buff_expected, "db_write result");

    ret = db_write(DB_NUMBER_OF_ENTRIES, instance, buff);
    ok &= assert_ERROR(ret,"db_write invalid arg0");

    ret = db_write(field, 10000, buff);
    ok &= assert_ERROR(ret,"db_write invalid arg1");

    ret = db_write(field, instance, NULL);
    ok &= assert_ERROR(ret,"db_write invalid arg2");

    // testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_db_005()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
    int instance = 0;
    double buff[4] = {1.0, 2.0, 3.0, 4.0};
    double value;
    int ind;
    double buff_expected[] = {1.0, 2.0, 3.0, 4.0};

    testDescription(__FUNCTION__, "Read a single index of a database field");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_write(field, instance, buff);
    ok &= assert_OK(ret,"db_write");
    ok &= assert_dbFieldDouble(field, instance, buff_expected, "db_write result");

    for(ind = 0; ind < 4; ind++) {
        ret = db_index_read(field, instance, ind, &value);
        ok &= assert_OK(ret,"db_index_read");
        ok &= (EPSI > fabs(buff_expected[ind]-value));
    }

    ret = db_index_read(DB_NUMBER_OF_ENTRIES,instance,ind,buff);
    ok &= assert_ERROR(ret,"db_index_read invalid arg0");

    ret = db_index_read(field,10000,ind,buff);
    ok &= assert_ERROR(ret,"db_index_read invalid arg1");

    ret = db_index_read(field,instance,100,buff);
    ok &= assert_ERROR(ret,"db_index_read invalid arg2");

    ret = db_index_read(field,instance,ind,NULL);
    ok &= assert_ERROR(ret,"db_index_read invalid arg3");

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_006()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
    int instance = 0;
    double buff[4] = {1.0, 2.0, 3.0, 4.0};
    double value;
    int ind;
    double buff_expected1[4] = {0.0, 0.0, 0.0, 0.0};
    double buff_expected2[][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 2.0, 0.0, 0.0},
        {1.0, 2.0, 3.0, 0.0},
        {1.0, 2.0, 3.0, 4.0},
    };
    double buff_expected3[4] = {1.0, 2.0, 5.0, 4.0};


    testDescription(__FUNCTION__, "Write a single index into a database field");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ok &= assert_dbFieldDouble(field, instance, buff_expected1, "default field values");
    for(ind = 0; ind < 4; ind++) {
        value = buff[ind];
        ret = db_index_write(field, instance, ind, &value);
        ok &= assert_OK(ret,"db_index_write");
        ok &= assert_dbFieldDouble(field, instance, buff_expected2[ind], "db_index_write result");
    }
    value = 5.0;
    ret = db_index_write(field, instance, 2, &value);
    ok &= assert_OK(ret,"db_index_write");
    ok &= assert_dbFieldDouble(field, instance, buff_expected3, "db_index_write result");

    ret = db_index_write(DB_NUMBER_OF_ENTRIES,instance,ind,&value);
    ok &= assert_ERROR(ret,"db_index_write invalid arg0");

    ret = db_index_write(field,10000,ind,&value);
    ok &= assert_ERROR(ret,"db_index_write invalid arg1");

    ret = db_index_write(field,instance,100,buff);
    ok &= assert_ERROR(ret,"db_index_write invalid arg2");

    ret = db_index_write(field,instance,ind,NULL);
    ok &= assert_ERROR(ret,"db_index_write invalid arg3");

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_007()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
    int instance = 0;
    double non_default[4] = {-4.0,-3.0,-2.0,-1.0};
    double buff[4] = {1.0, 2.0, 3.0, 4.0};
    double value;
    int ind;
    double buff_expected1[4] = {-4.0,-3.0,-2.0,-1.0};
    double buff_expected2[][4] = {
        {1.0,-3.0,-2.0,-1.0},
        {1.0, 2.0,-2.0,-1.0},
        {1.0, 2.0, 3.0,-1.0},
        {1.0, 2.0, 3.0, 4.0},
    };
    double buff_expected3[4] = {1.0, 2.0, 5.0, 4.0};

    testDescription(__FUNCTION__, "Write a single index into a database field starting from non default values");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_write(field, instance, non_default);
    ok &= assert_OK(ret,"db_write");
    ok &= assert_dbFieldDouble(field, instance, buff_expected1, "default field values");
    for(ind = 0; ind < 4; ind++) {
        value = buff[ind];
        ret = db_index_write(field, instance, ind, &value);
        ok &= assert_OK(ret,"db_index_write");
        ok &= assert_dbFieldDouble(field, instance, buff_expected2[ind], "db_index_write result");
    }
    value = 5.0;
    ret = db_index_write(field, instance, 2, &value);
    ok &= assert_OK(ret,"db_index_write");
    ok &= assert_dbFieldDouble(field, instance, buff_expected3, "db_index_write result");

    ret = db_index_write(DB_NUMBER_OF_ENTRIES,instance,ind,&value);
    ok &= assert_ERROR(ret,"db_index_write invalid arg0");

    ret = db_index_write(field,10000,ind,&value);
    ok &= assert_ERROR(ret,"db_index_write invalid arg1");

    ret = db_index_write(field,instance,100,buff);
    ok &= assert_ERROR(ret,"db_index_write invalid arg2");

    ret = db_index_write(field,instance,ind,NULL);
    ok &= assert_ERROR(ret,"db_index_write invalid arg3");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_008()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_TIMESTAMP;
    double value;

    testDescription(__FUNCTION__, "Check reading and writing different instances");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    for (int instance = 0; instance < IMU_MAX_NUMBER; instance++) {
        value = 1.0 + (double)instance;
        ret = db_write(field, instance, &value);
        ok &= assert_OK(ret,"db_write");
        ok &= assert_dbFieldDouble(field,instance,&value,"db_write result");
    }

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_009()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_GYROSCOPE;
    int size = 30;
    double gyrDataSingle[3] = {1.0,2.0,3.0};
    double buffer1[3] = {0.0,0.0,0.0};
    double buffer2[3] = {0.0,0.0,0.0};

    testDescription(__FUNCTION__, "Check simple buffering of database fields and ");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_field_buffer_setup(field, 0, size);
    ok &= assert_OK(ret, "db_field_buffer_setup");

    // Write a single data instance in the buffer and check tail and head match
    ret = db_write(field, 0, gyrDataSingle);
    ok &= assert_OK(ret, "db_write");
    ret = db_field_buffer_from_tail_data_get(field, 0, 0, buffer1);
    ok &= assert_OK(ret, "db_field_buffer_from_tail_data_get");
    ret = db_field_buffer_from_head_data_get(field, 0, 0, buffer2);
    ok &= assert_OK(ret, "db_field_buffer_from_head_data_get");
    ok &= assert_vector3Equal(buffer1,buffer2,"single data in buffer");

    // Clear the buffer and attempt to read form it
    ret = db_field_buffer_clear(field, 0);
    ok &= assert_OK(ret, "db_field_buffer_clear");
    ret = db_field_buffer_from_tail_data_get(field, 0, 0, buffer1);
    ok &= assert_ERROR(ret, "db_field_buffer_from_tail_data_get");
    ret = db_field_buffer_from_head_data_get(field, 0, 0, buffer2);
    ok &= assert_ERROR(ret, "db_field_buffer_from_head_data_get");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_010()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_GYROSCOPE;
    int size = 30;
    double gyrData[3] = {0.0,0.0,0.0};
    double gyrData_expected1[3] = {0.0,0.0,0.0};
    double gyrData_expected2[3] = {0.0,0.0,0.0};
    double buffer1[3] = {0.0,0.0,0.0};
    double buffer2[3] = {0.0,0.0,0.0};

    testDescription(__FUNCTION__, "Fill and read half of a database field buffer");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_field_buffer_setup(field, 0, size);
    ok &= assert_OK(ret, "db_field_buffer_setup");

    // Fill and read half of the buffer
    for (int i = 0; ok && i < size/2; i++) {
        gyrData[0] += 0.1;
        ret = db_write(field, 0, gyrData);
        ok &= assert_OK(ret, "db_write");
    }
    for (int i = 0; ok && i < size/2; i++) {
        gyrData_expected1[0] = 0.1*(i+1);
        ret = db_field_buffer_from_tail_data_get(field, 0, i, buffer1);
        ok &= assert_OK(ret, "db_field_buffer_from_tail_data_get");
        ok &= assert_vector3Equal(buffer1, gyrData_expected1, "db_field_buffer_from_tail_data_get result");

        gyrData_expected2[0] = 0.1*(size/2-i);
        ret = db_field_buffer_from_head_data_get(field, 0, i, buffer2);
        ok &= assert_OK(ret, "db_field_buffer_from_head_data_get");
        ok &= assert_vector3Equal(buffer2, gyrData_expected2, "db_field_buffer_from_head_data_get result");
    }
    ok &= assert_int(db_field_buffer_current_size_get(field, 0),size/2,"db_field_buffer_current_size_get");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_011()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_GYROSCOPE;
    int size = 30;
    double gyrData[3] = {0.0,0.0,0.0};
    double gyrData_expected1[3] = {0.0,0.0,0.0};
    double gyrData_expected2[3] = {0.0,0.0,0.0};
    double buffer1[3] = {0.0,0.0,0.0};
    double buffer2[3] = {0.0,0.0,0.0};

    testDescription(__FUNCTION__, "Fill and read a full database field buffer");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_field_buffer_setup(field, 0, size);
    ok &= assert_OK(ret, "db_field_buffer_setup");

    // Fill and read the Full buffer
    for (int i = 0; ok && i < size; i++) {
        gyrData[0] += 0.1;
        ret = db_write(field, 0, gyrData);
        ok &= assert_OK(ret, "db_write");
    }
    for (int i = size; ok && i < size; i++) {
        gyrData_expected1[0] = 0.1*(i+1);
        ret = db_field_buffer_from_tail_data_get(field, 0, i, buffer1);
        ok &= assert_OK(ret, "db_field_buffer_from_tail_data_get");
        ok &= assert_vector3Equal(buffer1, gyrData_expected1, "db_field_buffer_from_tail_data_get result");

        gyrData_expected2[0] = 0.1*(size-i);
        ret = db_field_buffer_from_head_data_get(field, 0, i, buffer2);
        ok &= assert_OK(ret, "db_field_buffer_from_head_data_get");
        ok &= assert_vector3Equal(buffer2, gyrData_expected2, "db_field_buffer_from_head_data_get result");
    }
    ok &= assert_int(db_field_buffer_current_size_get(field, 0),size,"db_field_buffer_current_size_get");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_012()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_GYROSCOPE;
    int size = 30;
    double gyrData[3] = {0.0,0.0,0.0};
    double gyrData_expected1[3] = {0.0,0.0,0.0};
    double gyrData_expected2[3] = {0.0,0.0,0.0};
    double buffer1[3] = {0.0,0.0,0.0};
    double buffer2[3] = {0.0,0.0,0.0};

    testDescription(__FUNCTION__, "Check filling and reading in and form the buffer after overwriting some data on it");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_field_buffer_setup(field, 0, size);
    ok &= assert_OK(ret, "db_field_buffer_setup");

    for (int i = 0; ok && i < 1.5*size; i++) {
        gyrData[0] += 0.1;
        ret = db_write(field, 0, gyrData);
        ok &= assert_OK(ret, "db_write");
    }
    for (int i = 0; ok && i < size; i++) {
        gyrData_expected1[0] = 0.1*(size/2+i+1);
        ret = db_field_buffer_from_tail_data_get(field, 0, i, buffer1);
        ok &= assert_OK(ret, "db_field_buffer_from_tail_data_get");
        ok &= assert_vector3Equal(buffer1, gyrData_expected1, "db_field_buffer_from_tail_data_get result");

        gyrData_expected2[0] = 0.1*(1.5*size-i);
        ret = db_field_buffer_from_head_data_get(field, 0, i, buffer2);
        ok &= assert_OK(ret, "db_field_buffer_from_head_data_get");
        ok &= assert_vector3Equal(buffer2, gyrData_expected2, "db_field_buffer_from_head_data_get result");
    }
    ok &= assert_int(db_field_buffer_current_size_get(field, 0),size,"db_field_buffer_current_size_get");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_013()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_GYROSCOPE;
    int size = 5;
    double gyrData[3] = {0.0,0.0,0.0};

    testDescription(__FUNCTION__, "Check filling the buffer and computing statistic parameters");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = db_field_buffer_setup(field, 0, size);
    ok &= assert_OK(ret, "db_field_buffer_setup");

    for (int i = 0; ok && i < size; i++) {
        gyrData[0] += 0.1;
        gyrData[1] -= 1.0;
        gyrData[2] += 10.0;
        ret = db_write(field, 0, gyrData);
        ok &= assert_OK(ret, "db_write");
    }

    DB_BUFFER_STATS stats = db_field_buffer_stats_compute(field,0);
    // Assert max
    ok &= assert_double(stats.max[0],size*0.1,EPSI,"max gyr x");
    ok &= assert_double(stats.max[1],-1.0,EPSI,"max gyr y");
    ok &= assert_double(stats.max[2],size*10.0,EPSI,"max gyr z");
    // Assert min
    ok &= assert_double(stats.min[0],0.1,EPSI,"min gyr x");
    ok &= assert_double(stats.min[1],size*-1.0,EPSI,"min gyr y");
    ok &= assert_double(stats.min[2],10.0,EPSI,"min gyr z");
    // Assert mean
    ok &= assert_double(stats.mean[0],0.3,EPSI,"mean gyr x");
    ok &= assert_double(stats.mean[1],-3.0,EPSI,"mean gyr y");
    ok &= assert_double(stats.mean[2],30.0,EPSI,"mean gyr z");
    // Assert var
    ok &= assert_double(stats.var[0],0.025,EPSI,"var gyr x");
    ok &= assert_double(stats.var[1],2.5,EPSI,"var gyr y");
    ok &= assert_double(stats.var[2],250.0,EPSI,"var gyr z");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_001()
{
    bool ok = true;
    ARM_POSE arm;
    const ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    arm = arm_pose_get();
    ok &= assert_armEqual(arm, expected, "default arm pos");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_002()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotations");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected, "no rot arm position");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_003()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
        .wrist.position       = {0.0, 5.0, -10.0},
        .wrist.orientation    = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for shoulder and 90 degrees rotation in X axis (global frame) for elbow");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromXRotation(M_PI_2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected, "arm position after rot");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_004()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected1 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    const ARM_POSE expected2 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
        .elbow.position       = {0.0, 10.0,  0.0},
        .elbow.orientation    = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
        .wrist.position       = {0.0, 15.0, 0.0},
        .wrist.orientation    = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in X axis (global frame) for shoulder");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    arm = arm_pose_get();
    ok &= assert_armEqual(arm, expected1, "arm position before rot");

    Quaternion_fromXRotation(M_PI_2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected2, "arm position after rot");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_005()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected1 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    const ARM_POSE expected2 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.5, .v = {-0.5, 0.5, 0.5}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.5, .v = {-0.5, 0.5, 0.5}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = 0.5, .v = {-0.5, 0.5, 0.5}},
    };
    const ARM_POSE expected3 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.5, .v = {-0.5, 0.5, 0.5}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.0, .v = {-M_SQRT1_2, 0.0, M_SQRT1_2}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = 0.0, .v = {-M_SQRT1_2, 0.0, M_SQRT1_2}},
    };

    testDescription(__FUNCTION__, "Rotate joints in Z axis (global frame) to see nothing happens");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    arm = arm_pose_get();
    ok &= assert_armEqual(arm, expected1, "arm position before first rot");

    Quaternion_fromZRotation(M_PI_2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected2, "arm position after first rot");

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromZRotation(M_PI_2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected3, "arm position after second rot");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_006()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected1 = { // After elbow rotation in X
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
        .wrist.position       = {0.0, 5.0, -10.0},
        .wrist.orientation    = {.w = 0.5, .v = {0.5, 0.5, 0.5}},
    };
    const ARM_POSE expected2 = { // Afrer shoulder rotation in Z
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.5, .v = {-0.5, 0.5, 0.5}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.0, .v = {0.0, M_SQRT1_2, M_SQRT1_2}},
        .wrist.position       = {-5.0, 0.0, -10.0},
        .wrist.orientation    = {.w = 0.0, .v = {0.0, M_SQRT1_2, M_SQRT1_2}},
    };

    testDescription(__FUNCTION__, "Rotate joints in Z axis (global frame) to see nothing happens");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromXRotation(M_PI_2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected1, "arm position after first rot");

    Quaternion_fromZRotation(M_PI_2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected2, "arm position after second rot");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_007()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.0, .v = {0.0, 1.0, 0.0}},
        .elbow.position       = {-10.0, 0.0, 0.0},
        .elbow.orientation    = {.w = 0.0, .v = {0.0, 1.0, 0.0}},
        .wrist.position       = {-15.0, 0.0, 0.0},
        .wrist.orientation    = {.w = 0.0, .v = {0.0, 1.0, 0.0}},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in Y axis (global frame) for shoulder");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    Quaternion_fromYRotation(M_PI_2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected, "arm position after rot");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_008()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    double eulerAngles[3] = { M_PI_4, M_PI_2, 0.0};
    const ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.0, .v = {0.382683, 0.923880, 0.0}},
        .elbow.position       = {-10.0*M_SQRT1_2, 10.0*M_SQRT1_2, 0.0},
        .elbow.orientation    = {.w = 0.0, .v = {0.382683, 0.923880, 0.0}},
        .wrist.position       = {-15.0*M_SQRT1_2, 15.0*M_SQRT1_2, 0.0},
        .wrist.orientation    = {.w = 0.0, .v = {0.382683, 0.923880, 0.0}},
    };

    testDescription(__FUNCTION__, "Rotate shoulder for a set of ZYX euler angles (45,90,0)");
    ok = preconditions_init(__FUNCTION__);

    // Test Steps
    Quaternion_fromEulerZYX(eulerAngles, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected, "arm position after rot");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_009()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    ARM_FRAME frame = {
        .position    = {0.0, 0.0, 0.0},
        .orientation = {.w = 1.0, .v = {0.0,0.0,0.0}},
    };
    ARM_FRAME transform = {
        .position    = {10.0, 0.0, 0.0},
        .orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    ARM_FRAME expected1 = {
        .position    = {10.0, 0.0, 0.0},
        .orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    ARM_FRAME expected2 = {
        .position    = {10.0, 0.0,-10.0},
        .orientation = {.w = 0.0, .v = {0.0, 1.0, 0.0}},
    };
    ARM_FRAME expected3 = {
        .position    = {0.0, 0.0, -10.0},
        .orientation = {.w = -M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    ARM_FRAME expected4 = {
        .position    = {0.0, 0.0, 0.0},
        .orientation = {.w =-1.0, .v = {0.0, 0.0, 0.0}},
    };

    testDescription(__FUNCTION__, "Perform consecutive rotations in the Y axis and translations");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_homogeneous_transform(frame, transform, &frame);
    ok &= assert_OK(ret, "arm_homogeneous_transform 1");
    ok &= assert_frame(frame, expected1, "arm_homogeneous_transform result 1");

    ret = arm_homogeneous_transform(frame, transform, &frame);
    ok &= assert_OK(ret, "arm_homogeneous_transform 2");
    ok &= assert_frame(frame, expected2, "arm_homogeneous_transform result 2");

    ret = arm_homogeneous_transform(frame, transform, &frame);
    ok &= assert_OK(ret, "arm_homogeneous_transform 3");
    ok &= assert_frame(frame, expected3, "arm_homogeneous_transform result 3");

    ret = arm_homogeneous_transform(frame, transform, &frame);
    ok &= assert_OK(ret, "arm_homogeneous_transform 4");
    ok &= assert_frame(frame, expected4, "arm_homogeneous_transform result 4");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_010()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    Quaternion joints[ARM_NUMBER_OF_JOINTS] = {
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    ARM_POSE result;
    const ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        .elbow.position       = {-10.0, 0.0, 0.0},
        .elbow.orientation    = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        .wrist.position       = {-15.0, 0.0, 0.0},
        .wrist.orientation    = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    double expected_quat_buff[4];

    testDescription(__FUNCTION__, "Set the zero position of the arm (T-pose)");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_direct_kinematics_compute(joints, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected, "arm_direct_kinematics_compute result");

    ok &= assert_dbFieldDouble(DB_ARM_SHOULDER_POSITION, 0, expected.shoulder.position, "arm_direct_kinematics_compute db_sh_pos");
    quaternion_buffer_build(expected.shoulder.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_SHOULDER_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_sh_ori");

    ok &= assert_dbFieldDouble(DB_ARM_ELBOW_POSITION, 0, expected.elbow.position, "arm_direct_kinematics_compute db_el_pos");
    quaternion_buffer_build(expected.elbow.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_ELBOW_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_el_ori");

    ok &= assert_dbFieldDouble(DB_ARM_WRIST_POSITION, 0, expected.wrist.position, "arm_direct_kinematics_compute db_wr_pos");
    quaternion_buffer_build(expected.wrist.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_WRIST_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_wr_ori");

    result = arm_pose_get();
    ok &= assert_armEqual(result, expected, "arm_pose_get result");

    testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_arm_011()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    Quaternion joints[ARM_NUMBER_OF_JOINTS] = {
        {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    ARM_POSE result;
    ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
    };
    double expected_quat_buff[4];

    testDescription(__FUNCTION__, "Apply direct kinematics for a rotation of 90 in Y axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_direct_kinematics_compute(joints, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected, "arm_direct_kinematics_compute result");

    ok &= assert_dbFieldDouble(DB_ARM_SHOULDER_POSITION, 0, expected.shoulder.position, "arm_direct_kinematics_compute db_sh_pos");
    quaternion_buffer_build(expected.shoulder.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_SHOULDER_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_sh_ori");

    ok &= assert_dbFieldDouble(DB_ARM_ELBOW_POSITION, 0, expected.elbow.position, "arm_direct_kinematics_compute db_el_pos");
    quaternion_buffer_build(expected.elbow.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_ELBOW_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_el_ori");

    ok &= assert_dbFieldDouble(DB_ARM_WRIST_POSITION, 0, expected.wrist.position, "arm_direct_kinematics_compute db_wr_pos");
    quaternion_buffer_build(expected.wrist.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_WRIST_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_wr_ori");

    result = arm_pose_get();
    ok &= assert_armEqual(result, expected, "arm_pose_get result");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_012()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    Quaternion joints[ARM_NUMBER_OF_JOINTS] = {
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    ARM_POSE result;
    double upperarm_len = 11.0;
    double forearm_len  = 6.0;
    ARM_POSE expected1 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -11.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -17.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    ARM_POSE expected2 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        .elbow.position       = {-11.0, 0.0, 0.0},
        .elbow.orientation    = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        .wrist.position       = {-17.0, 0.0, 0.0},
        .wrist.orientation    = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    double expected_quat_buff[4];

    testDescription(__FUNCTION__, "Set a different set of arm lengths and then apply direct kinematics for a rotation of 90 in Y axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_segments_length_set(upperarm_len, forearm_len);
    ok &= assert_OK(ret, "arm_segments_length_set");

    result = arm_pose_get();
    ok &= assert_armEqual(result, expected1, "arm_pose_get result 1");

    ret = arm_direct_kinematics_compute(joints, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected2, "arm_direct_kinematics_compute result");

    ok &= assert_dbFieldDouble(DB_ARM_SHOULDER_POSITION, 0, expected2.shoulder.position, "arm_direct_kinematics_compute db_sh_pos");
    quaternion_buffer_build(expected2.shoulder.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_SHOULDER_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_sh_ori");

    ok &= assert_dbFieldDouble(DB_ARM_ELBOW_POSITION, 0, expected2.elbow.position, "arm_direct_kinematics_compute db_el_pos");
    quaternion_buffer_build(expected2.elbow.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_ELBOW_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_el_ori");

    ok &= assert_dbFieldDouble(DB_ARM_WRIST_POSITION, 0, expected2.wrist.position, "arm_direct_kinematics_compute db_wr_pos");
    quaternion_buffer_build(expected2.wrist.orientation,expected_quat_buff);
    ok &= assert_dbFieldDouble(DB_ARM_WRIST_ORIENTATION, 0, expected_quat_buff, "arm_direct_kinematics_compute db_wr_ori");

    result = arm_pose_get();
    ok &= assert_armEqual(result, expected2, "arm_pose_get result 2");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_013()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    Quaternion joints1[ARM_NUMBER_OF_JOINTS] = {
        {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    Quaternion joints2[ARM_NUMBER_OF_JOINTS] = {
        {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
    };
    Quaternion joints3[ARM_NUMBER_OF_JOINTS] = {
        {.w = 0.5, .v = {0.5, -0.5, 0.5}},
        {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
    };    
    ARM_POSE result;
    ARM_POSE expected1 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
    };
    ARM_POSE expected2 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.0, .v = {0.0, -1.0, 0.0}},
        .wrist.position       = {5.0, 0.0, -10.0},
        .wrist.orientation    = {.w = 0.0, .v = {0.0, -1.0, 0.0}},
    };
    ARM_POSE expected3 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.5, .v = {0.5, -0.5, 0.5}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.0, .v = {M_SQRT1_2, -M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 5.0, -10.0},
        .wrist.orientation    = {.w = 0.0, .v = {M_SQRT1_2, -M_SQRT1_2, 0.0}},
    };

    testDescription(__FUNCTION__, "Apply direct kinematics for different sets of joint values for shoulder and elbow");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_direct_kinematics_compute(joints1, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected1, "arm_direct_kinematics_compute result 1");

    ret = arm_direct_kinematics_compute(joints2, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected2, "arm_direct_kinematics_compute result 2");

    ret = arm_direct_kinematics_compute(joints3, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected3, "arm_direct_kinematics_compute result 3");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_014()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    // Quaternion q1 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };
    // Quaternion q2 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };

    Quaternion q1 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };
    Quaternion q2 = { .w = 0.5, .v = {0.5,-0.5, 0.5} };

    double omega1[3] = {0.0, 0.0, M_PI_2};
    double omega2[3] = {M_PI_2, 0.0, M_PI_2};
    double omegaR[3] = {0.0};

    double omegaR_expected[3] = {0.0,-M_PI_2,0.0};

    testDescription(__FUNCTION__, "Compute the angular velocity between two frames");
    ok = preconditions_init(__FUNCTION__); 

    ret = arm_relative_angular_vel_compute(q1, q2, omega1, omega2, omegaR);
    ok &= assert_OK(ret, "arm_relative_angular_vel_compute");
    ok &= assert_vector3Equal(omegaR, omegaR_expected, "arm_relative_angular_vel_compute result");
    // printf("omegaR: %f, %f, %f\n",omegaR[0],omegaR[1],omegaR[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_015()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    Quaternion joints1[ARM_NUMBER_OF_JOINTS] = {
        {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    Quaternion joints2[ARM_NUMBER_OF_JOINTS] = {
        {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    Quaternion joints3[ARM_NUMBER_OF_JOINTS] = {
        {.w = 0.5, .v = {0.5, 0.5,-0.5}},
        {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };  
    ARM_POSE dk_result;
    Quaternion ik_result[ARM_NUMBER_OF_JOINTS];

    testDescription(__FUNCTION__, "Execute the inverse kinematics of the result of the direct kinemtics to see if the starting point matches the result");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_direct_kinematics_compute(joints1, &dk_result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ret = arm_inverse_kinematics_compute(dk_result.shoulder.orientation, dk_result.elbow.orientation, ik_result);
    ok &= assert_OK(ret, "arm_inverse_kinematics_compute");
    ok &= assert_quaternion(ik_result[SHOULDER], joints1[SHOULDER], "arm_inverse_kinematics_compute result1 sh");
    ok &= assert_quaternion(ik_result[ELBOW], joints1[ELBOW], "arm_inverse_kinematics_compute result1 el");

    ret = arm_direct_kinematics_compute(joints2, &dk_result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ret = arm_inverse_kinematics_compute(dk_result.shoulder.orientation, dk_result.elbow.orientation, ik_result);
    ok &= assert_OK(ret, "arm_inverse_kinematics_compute");
    ok &= assert_quaternion(ik_result[SHOULDER], joints2[SHOULDER], "arm_inverse_kinematics_compute result2 sh");
    ok &= assert_quaternion(ik_result[ELBOW], joints2[ELBOW], "arm_inverse_kinematics_compute result2 el");

    ret = arm_direct_kinematics_compute(joints3, &dk_result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ret = arm_inverse_kinematics_compute(dk_result.shoulder.orientation, dk_result.elbow.orientation, ik_result);
    ok &= assert_OK(ret, "arm_inverse_kinematics_compute");
    ok &= assert_quaternion(ik_result[SHOULDER], joints3[SHOULDER], "arm_inverse_kinematics_compute result3 sh");
    // ok &= assert_quaternion(ik_result[ELBOW], joints3[ELBOW], "arm_inverse_kinematics_compute result3 el");
    // TODO: Check this assert

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_016() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    double rotVector1[3] = {0.0,0.0,1.0};
    double rotVector2[3] = {1.0,0.0,0.0};
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER];
    Quaternion q1 = {1,{0,0,0}}, q2 = {1,{0,0,0}};

    testDescription(__FUNCTION__, "Check zeroing of elbow angles from rotation vectors and quaternion");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    for (double ang1 = -M_PI; ok && (ang1 < M_PI); ang1 += M_PI/6) {
        for (double ang2 = -M_PI; ok && (ang2 < M_PI); ang2 += M_PI/6) {
            Quaternion q_r1, q_r2;
            Quaternion_fromAxisAngle(rotVector1, ang1, &q_r1);
            Quaternion_fromAxisAngle(rotVector2, ang2, &q_r2);
            Quaternion_multiply(&q_r2,&q_r1,&q2);

            // tst_str("Angles: %f, %f",ang1,ang2);

            ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
            ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get pre zero");
            // tst_str("[%f,%f] Angles: fe <%f>, ps <%f>, beta <%f>",ang1,ang2,
            //     anglesFE_B_PS[ALPHA_FE],anglesFE_B_PS[GAMMA_PS],anglesFE_B_PS[BETA_CARRYING]);

            ret = arm_elbow_angles_zero(0.0,0.0,q1,q2,rotVector1,rotVector2);
            if (RET_NO_EXEC != ret) {
                ok &= assert_OK(ret, "arm_elbow_angles_zero");

                ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
                ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get zero");
                ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get zeroed alpha");
                ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get zeroed gamma");
            }

            /* Test with random quats */
            Quaternion_fromAxisAngle(rotVector1, 2*M_PI*tstRandomDoubleGenerate(), &q_r1);
            Quaternion_fromAxisAngle(rotVector2, 2*M_PI*tstRandomDoubleGenerate(), &q_r2);
            Quaternion_multiply(&q_r2,&q_r1,&q2);

            ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
            ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get pre zero random");
            // tst_str("[%f,%f] Angles: fe <%f>, ps <%f>, beta <%f>",ang1,ang2,fe,ps,carryingAngle);

            ret = arm_elbow_angles_zero(0.0,0.0,q1,q2,rotVector1,rotVector2);
            if (RET_NO_EXEC != ret) {
                ok &= assert_OK(ret, "arm_elbow_angles_zero random");

                ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
                ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get zero random");
                ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get zeroed alpha");
                ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get zeroed gamma");
            }
        }
    }
    
    testCleanUp();
    testReport(ok);
    return ok;
} 

bool tst_arm_017() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    double rotVector1[3] = {0.0,0.0,1.0};
    double rotVector2[3] = {1.0,0.0,0.0};
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER];
    Quaternion q1 = {1.0,{0.0,0.0,0.0}}, q2 = {1.0,{0.0,0.0,0.0}};

    testDescription(__FUNCTION__, "Compute elbow angles from rotation vectors and quaternion after beeing zeroed");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_elbow_angles_zero(0.0,0.0,q1,q2,rotVector1,rotVector2);
    ok &= assert_OK(ret, "arm_elbow_angles_zero");
    
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 1");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 1");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 1");

    Quaternion_fromXRotation(M_PI_4,&q1);
    Quaternion_fromXRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 2");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 2");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 2");

    Quaternion_fromYRotation(M_PI_4,&q1);
    Quaternion_fromYRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 3");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 3");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 3");

    Quaternion_fromZRotation(M_PI_4,&q1);
    Quaternion_fromZRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 4");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 4");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 4");

    Quaternion_set(1,0,0,0,&q1);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 5");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],M_PI_4,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 5");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,   EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 5");

    Quaternion_fromXRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 6");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,   EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 6");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],M_PI_4,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 6");

    Quaternion_fromYRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 7");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 7");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 7");

    for (int i = 0; ok && i < 10; i++) {
        q2 = tstRandomQuaternionGenerate();
        ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
        ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 8");
        Quaternion q_aux1, q_aux2, q_aux21;
        Quaternion_fromZRotation(-M_PI_2,&q_aux1);
        Quaternion_multiply(&q1,&q_aux1,&q_aux1);
        Quaternion_fromZRotation(-M_PI_2,&q_aux2);
        Quaternion_multiply(&q2,&q_aux2,&q_aux2);
        q_aux21 = arm_quaternion_between_two_get(q_aux1,q_aux2);
        ok &= assert_dbFieldQuaternion(DB_ARM_ELBOW_QUATERNION,0,q_aux21,"arm_elbow_angles_from_rotation_vectors_get db result 8");
    }

    testCleanUp();
    testReport(ok);
    return ok;
} 


bool tst_arm_018() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    double angle = M_PI/6;
    double rotVector1[3] = {0.0,sin(angle),cos(angle)};
    double rotVector2[3] = {1.0,0.0,0.0};
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER];
    Quaternion q_align;
    Quaternion q1 = {1.0,{0.0,0.0,0.0}}, q2 = {1.0,{0.0,0.0,0.0}};

    testDescription(__FUNCTION__, "Compute elbow angles from rotation vectors and quaternion after beeing zeroed");
    ok = preconditions_init(__FUNCTION__); 

    Quaternion_fromXRotation(-angle,&q_align);

    // Test Steps
    ret = arm_elbow_angles_zero(0.0,0.0,q1,q2,rotVector1,rotVector2);
    ok &= assert_OK(ret, "arm_elbow_angles_zero");
    
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 1");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 1");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 1");

    Quaternion_fromXRotation(M_PI_4,&q1);
    Quaternion_fromXRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 2");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 2");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 2");

    Quaternion_fromYRotation(M_PI_4,&q1);
    Quaternion_fromYRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 3");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 3");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 3");

    Quaternion_fromZRotation(M_PI_4,&q1);
    Quaternion_fromZRotation(M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 4");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 4");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 4");

    Quaternion_set(1,0,0,0,&q1);
    Quaternion_fromAxisAngle(rotVector1,M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 5");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],M_PI_4,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 5");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],0.0,   EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 5");

    Quaternion_fromAxisAngle(rotVector2,M_PI_4,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 6");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],0.0,   EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 6");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],M_PI_4,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 6");

    Quaternion q_aux1, q_aux2;
    Quaternion_fromAxisAngle(rotVector1,M_PI_4,&q_aux1);
    Quaternion_fromAxisAngle(rotVector2,M_PI/6,&q_aux2);
    Quaternion_multiply(&q_aux2,&q_aux1,&q2);
    ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
    ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 7");
    ok &= assert_double(anglesFE_B_PS[ALPHA_FE],M_PI_4,EPSI,"arm_elbow_angles_from_rotation_vectors_get alpha 7");
    ok &= assert_double(anglesFE_B_PS[GAMMA_PS],M_PI_4,EPSI,"arm_elbow_angles_from_rotation_vectors_get gamma 7");
    ok &= assert_double(anglesFE_B_PS[BETA_CARRYING],0.0,EPSI,"arm_elbow_angles_from_rotation_vectors_get beta 7");

    for (int i = 0; ok && i < 10; i++) {
        q2 = tstRandomQuaternionGenerate();
        ret = arm_elbow_angles_from_rotation_vectors_get(q1, q2, rotVector1, rotVector2, anglesFE_B_PS);
        ok &= assert_OK(ret, "arm_elbow_angles_from_rotation_vectors_get 8");
        Quaternion q_aux1, q_aux2, q_aux21;
        Quaternion_fromZRotation(-M_PI_2,&q_aux1);
        Quaternion_multiply(&q1,&q_aux1,&q_aux1);
        Quaternion_fromZRotation(-M_PI_2,&q_aux2);
        Quaternion_multiply(&q2,&q_aux2,&q_aux2);
        q_aux21 = arm_quaternion_between_two_get(q_aux1,q_aux2);
        ok &= assert_dbFieldQuaternion(DB_ARM_ELBOW_QUATERNION,0,q_aux21,"arm_elbow_angles_from_rotation_vectors_get db result 8");
    }

    testCleanUp();
    testReport(ok);
    return ok;
} 

bool tst_arm_019() 
{
    bool ok = true;
    Quaternion q_zero = {.w=1,.v={0,0,0}};
    Quaternion q_90z  = {.w=cos(M_PI_2/2),.v={0,0,sin(M_PI_2/2)}};

    ARM_POSE result;
    ARM_POSE expected1 = {
        .shoulder.position = {0,0,0},
        .shoulder.orientation = q_zero,
        .elbow.position = {-10,0,0},
        .elbow.orientation    = q_zero,
        .wrist.position = {-15,0,0},
        .wrist.orientation    = q_zero,
    };
    ARM_POSE expected2 = {
        .shoulder.position = {0,0,0},
        .shoulder.orientation = q_zero,
        .elbow.position = {-10,0,0},
        .elbow.orientation    = q_90z,
        .wrist.position = {-10,-5,0},
        .wrist.orientation    = q_90z,
    };
    ARM_POSE expected3 = {
        .shoulder.position = {0,0,0},
        .shoulder.orientation = q_90z,
        .elbow.position = {0,-10,0},
        .elbow.orientation    = q_zero,
        .wrist.position = {-5,-10,0},
        .wrist.orientation    = q_zero,
    };
    ARM_POSE expected4 = {
        .shoulder.position = {0,0,0},
        .shoulder.orientation = q_90z,
        .elbow.position = {0,-10,0},
        .elbow.orientation    = q_90z,
        .wrist.position = {0,-15,0},
        .wrist.orientation    = q_90z,
    };

    testDescription(__FUNCTION__, "Set orientation of arm and forearm");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    result = arm_orientations_set(q_zero,q_zero,q_zero);
    ok &= assert_armEqual(result,expected1,"arm_orientations_set result 1");
    
    result = arm_orientations_set(q_zero,q_90z,q_90z);
    ok &= assert_armEqual(result,expected2,"arm_orientations_set result 2");
    
    result = arm_orientations_set(q_90z,q_zero,q_zero);
    ok &= assert_armEqual(result,expected3,"arm_orientations_set result 3");
    
    result = arm_orientations_set(q_90z,q_90z,q_90z);
    ok &= assert_armEqual(result,expected4,"arm_orientations_set result 4");

    testCleanUp();
    testReport(ok);
    return ok;
} 

bool tst_arm_020() 
{
    bool ok = true;
    Quaternion q_dflt  = {.w=cos(M_PI_2/2),.v={0,-sin(M_PI_2/2),0}};
    Quaternion q_zero  = {.w=1,.v={0,0,0}};
    Quaternion q_90x   = {.w=cos(M_PI_2/2),.v={sin(M_PI_2/2),0,0}};
    Quaternion q_90xy  = {.w=0.5,.v={0.5,-0.5, 0.5}};
    Quaternion q_45xyz = {.w=0.8446232,.v={0.1913417, 0.4619398, 0.1913417}};

    double sh_angles[ARM_SHOULDER_ANGLES_NUMBER];
    double sh_expected[ARM_SHOULDER_ANGLES_NUMBER];

    testDescription(__FUNCTION__, "Test the shoulder angles computing from quaternion");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    sh_expected[SH_ROTATION] = 0.0;
    sh_expected[SH_FLEXION]  = -M_PI_2; // Singular
    sh_expected[SH_ABDUCTION]= 0.0;
    arm_shoulder_angles_compute(q_dflt,sh_angles);
    ok &= assert_vector3Equal(sh_angles,sh_expected,"arm_shoulder_angles_compute result 0");

    sh_expected[SH_ROTATION] = M_PI_2;
    sh_expected[SH_FLEXION]  = -M_PI_2; // Singular
    sh_expected[SH_ABDUCTION]= 0.0;
    arm_shoulder_angles_compute(q_90xy,sh_angles);
    ok &= assert_vector3Equal(sh_angles,sh_expected,"arm_shoulder_angles_compute result 1");

    sh_expected[SH_ROTATION] = 0.0;
    sh_expected[SH_FLEXION]  = 0.0;
    sh_expected[SH_ABDUCTION]= 0.0;
    arm_shoulder_angles_compute(q_zero,sh_angles);
    ok &= assert_vector3Equal(sh_angles,sh_expected,"arm_shoulder_angles_compute result 2");

    sh_expected[SH_ROTATION] = M_PI_2;
    sh_expected[SH_FLEXION]  = 0.0;
    sh_expected[SH_ABDUCTION]= 0.0;
    arm_shoulder_angles_compute(q_90x,sh_angles);
    ok &= assert_vector3Equal(sh_angles,sh_expected,"arm_shoulder_angles_compute result 3");

    sh_expected[SH_ROTATION] = M_PI_4;
    sh_expected[SH_FLEXION]  = M_PI_4;
    sh_expected[SH_ABDUCTION]= M_PI_4;
    arm_shoulder_angles_compute(q_45xyz,sh_angles);
    ok &= assert_vector3Equal(sh_angles,sh_expected,"arm_shoulder_angles_compute result 4");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_001() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omega1[] = {500.0,0.0,0.0};
    double omega2[] = {1000.0,0.0,0.0};
    double omega1_noise[3];
    double omega2_noise[3];
    Quaternion dummy = {.w = 1.0, .v = {0.0, 0.0, 0.0}};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over X axis");
    ok = preconditions_init_databaseCalib(__FUNCTION__,0,0);

    // Test Steps

    while (ok && time<timeout)
    {
        // Set timestamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        time += timeInc;
        // Add noise to angular velocities
        tstVector3RandomNoiseAdd(omega1, 100.0, omega1_noise);
        tstVector3RandomNoiseAdd(omega2, 100.0, omega2_noise);
        // Execute arm calibration of a single rotation axis
        ret = cal_automatic_rotation_axis_calibrate(omega1_noise,omega2_noise,dummy,dummy,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_substract(omega2,omega1,v_expected);
    ok &= assert_OK(ret, "vector3_substract");
    ret = vector3_normalize(v_expected,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,1e-1,"cal_automatic_rotation_axis_calibrate result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_002() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omega1[] = {0.0,500.0,0.0};
    double omega2[] = {0.0,1000.0,0.0};
    double omega1_noise[3];
    double omega2_noise[3];
    Quaternion dummy = {.w = 1.0, .v = {0.0, 0.0, 0.0}};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over Y axis");
    ok = preconditions_init_databaseCalib(__FUNCTION__,0,0);

    // Test Steps

    while (ok && time<timeout)
    {
        // Set timestamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        time += timeInc;
        // Add noise to angular velocities
        tstVector3RandomNoiseAdd(omega1, 100.0, omega1_noise);
        tstVector3RandomNoiseAdd(omega2, 100.0, omega2_noise);
        // Execute arm calibration of a single rotation axis
        ret = cal_automatic_rotation_axis_calibrate(omega1_noise,omega2_noise,dummy,dummy,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_substract(omega2,omega1,v_expected);
    ok &= assert_OK(ret, "vector3_substract");
    ret = vector3_normalize(v_expected,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,1e-1,"cal_automatic_rotation_axis_calibrate result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_003() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omega1[] = {0.0,0.0,500.0};
    double omega2[] = {0.0,0.0,1000.0};
    double omega1_noise[3];
    double omega2_noise[3];
    Quaternion dummy = {.w = 1.0, .v = {0.0, 0.0, 0.0}};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over Z axis");
    ok = preconditions_init_databaseCalib(__FUNCTION__,0,0);

    // Test Steps

    while (ok && time<timeout)
    {
        // Set timestamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        time += timeInc;
        // Add noise to angular velocities
        tstVector3RandomNoiseAdd(omega1, 100.0, omega1_noise);
        tstVector3RandomNoiseAdd(omega2, 100.0, omega2_noise);
        // Execute arm calibration of a single rotation axis
        ret = cal_automatic_rotation_axis_calibrate(omega1_noise,omega2_noise,dummy,dummy,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_substract(omega2,omega1,v_expected);
    ok &= assert_OK(ret, "vector3_substract");
    ret = vector3_normalize(v_expected,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,1e-1,"cal_automatic_rotation_axis_calibrate result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_004() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    // const char csvFile[] = "test/tst_data/data5_tst_cal_004_onArmArbitraryMotions.csv";
    // const char csvFile[] = "test/tst_data/data6_tst_cal_004_onArmArbitraryMotions.csv";
    const char csvFile[] = "test/tst_data/data7_tst_cal_004_onArmArbitraryMotions.csv";
    double buff[TST_MAX_CSV_DATA_VALUES];
    TST_CSV_IMU_DATA *data = (TST_CSV_IMU_DATA*)buff;

    int iterations = 10;
    double initVector1[7][3] = {
        {1,0,0},
        {0,1,0},
        {1,1,0},
        {0,0,1},
        {1,0,1},
        {0,1,1},
        {1,1,1},
    };
    double initVector2[7][3] = {
        {1,1,1},
        {0,1,1},
        {1,0,1},
        {0,1,1},
        {1,1,0},
        {0,1,0},
        {1,0,0},
    };

    double rotVector1[3];
    double rotVector2[3];

    double prevRotV1[3];
    double prevRotV2[3];

    Quaternion q_sensor1 = {.w = 1.0, .v={0.0, 0.0, 0.0}};
    Quaternion q_sensor2 = {.w = 1.0, .v={0.0, 0.0, 0.0}};

    int imu_data_window = 50;
    int observations_window = CALIB_TWO_ROT_AXES_WINDOW;

    testDescription(__FUNCTION__, "Test two rotation axis calibration by using real data with different set of initialization vectors");
    ok = preconditions_init_databaseCalib(__FUNCTION__,imu_data_window,observations_window);

    // Test Steps
    ok &= tstCsvLoad(csvFile);
    
    for (int i = 0; i < iterations; i++) {
        // Set initial vector
        if (7 < i) {
            tstRandomUnitVector3Generate(rotVector1);
            tstRandomUnitVector3Generate(rotVector2);
        }else{
            ret = vector3_normalize(initVector1[i],rotVector1);
            ok &= assert_OK(ret, "vector3_normalize");
            ret = vector3_normalize(initVector2[i],rotVector2);
            ok &= assert_OK(ret, "vector3_normalize");
        }

        int line = 1000;
        while (tstCsvDataLineGet(line++, buff) && CALIB_TWO_ROT_AXES_WINDOW >= db_field_buffer_current_size_get(DB_CALIB_OMEGA,0)) {
            // Set gyroscope readings
            ret = db_write(DB_IMU_GYROSCOPE,0,data->gyr0);
            ok &= assert_OK(ret, "db_write DB_IMU_GYROSCOPE_0");
            ret = db_write(DB_IMU_GYROSCOPE,1,data->gyr1);
            ok &= assert_OK(ret, "db_write DB_IMU_GYROSCOPE_1");
            // Set quaternion values
            quaternion_from_buffer_build(data->quat0, &q_sensor1);
            ret = db_write(DB_IMU_QUATERNION,0,data->quat0);
            ok &= assert_OK(ret, "db_write DB_IMU_QUATERNION_0");
            quaternion_from_buffer_build(data->quat1, &q_sensor2);
            ret = db_write(DB_IMU_QUATERNION,1,data->quat1);
            ok &= assert_OK(ret, "db_write DB_IMU_QUATERNION_1");
            // Set timestamp
            ret = db_write(DB_IMU_TIMESTAMP,0,&data->timestamp);
            ok &= assert_OK(ret, "db_write");

            if (imu_data_window <= db_field_buffer_current_size_get(DB_IMU_GYROSCOPE,0)) {
                // Set observations
                ret = cal_gn2_observations_from_database_update(0.0);
                ok &= assert_OK(ret, "cal_gn2_observations_from_database_update");
            }

            // Dump database data
            ret = db_csv_dump();
            ok &= assert_OK(ret, "db_csv_dump");
        }
        // Call autocalib procedure
        ret = cal_gn2_two_rot_axes_calib(rotVector1,rotVector2);
        tst_str("[%d] -> V1: <%f, %f, %f>, V2: <%f, %f, %f>", i,
            rotVector1[0],rotVector1[1], rotVector1[2],
            rotVector2[0],rotVector2[1], rotVector2[2]);
        ok &= assert_OK(ret, "cal_gn2_two_rot_axes_calib");

        if (0 < i) {
            ok &= assert_vector3EqualNoSignThreshold(rotVector1,prevRotV1,5e-2,"cal_automatic_rotation_axis_calibrate result1");
            ok &= assert_vector3EqualNoSignThreshold(rotVector2,prevRotV2,5e-2,"cal_automatic_rotation_axis_calibrate result2");
        }
        ret = vector3_copy(rotVector1,prevRotV1);
        ok &= assert_OK(ret, "vector3_copy");
        ret = vector3_copy(rotVector2,prevRotV2);
        ok &= assert_OK(ret, "vector3_copy");
    }

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_005() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double x[3]={1,0,0};
    double z[3]={0,0,1};

    double angle1 = M_PI/12;
    double angle2 = M_PI/6;
    double rotVector1[3] = {0.0, sin(angle1), cos(angle1)};
    double rotVector2[3] = {cos(angle2), sin(angle2), 0.0};

    Quaternion q_sensor1;
    Quaternion q_sensor2;

    Quaternion q_body1;
    Quaternion q_body2;

    Quaternion q_zeroAndBody1, q_zeroAndBody2;

    Quaternion q_zab1_expected;
    Quaternion q_zab2_expected;

    Quaternion q_expected1, q_expected2;
    double q_buff[4] = {1.0,0.0,0.0,0.0};
    Quaternion q_arm, q_forearm;

    testDescription(__FUNCTION__, "Test zeroing of calibration procedure with rotation vectors to define the sensor to segment alignment");
    ok = preconditions_init(__FUNCTION__);

    // Set simulated IMU readings
    Quaternion_fromAxisAngle(x, -M_PI_4,  &q_sensor1);
    Quaternion_fromAxisAngle(z, -M_PI/12, &q_sensor2);
    // Set expected body positions at zero 
    Quaternion_fromAxisAngle(x, 0,       &q_body1);
    Quaternion_fromAxisAngle(z, -M_PI_2, &q_body2);
    // Set zero to body conversion to be expected
    Quaternion_fromAxisAngle(x, M_PI_4,     &q_zab1_expected);
    Quaternion_fromAxisAngle(z, -5*M_PI/12, &q_zab2_expected);

    // Test Steps
    ret = cal_gn2_zero_pose_calibrate(
        rotVector1,rotVector2,
        q_sensor1, q_sensor2,
        q_body1, q_body2,
        &q_zeroAndBody1,&q_zeroAndBody2);
    ok &= assert_OK(ret, "cal_gn2_zero_pose_calibrate");
    ok &= assert_quaternion(q_zeroAndBody1,q_zab1_expected,"cal_gn2_zero_pose_calibrate result 1");
    ok &= assert_quaternion(q_zeroAndBody2,q_zab2_expected,"cal_gn2_zero_pose_calibrate result 2");

    ret = db_write(DB_IMU_QUATERNION,0,q_buff);
    ok &= assert_OK(ret, "db_write");
    ret = db_write(DB_IMU_QUATERNION,1,q_buff);
    ok &= assert_OK(ret, "db_write");

    Quaternion_fromAxisAngle(x,M_PI_4,    &q_expected1);
    Quaternion_fromAxisAngle(z,-5*M_PI/12,&q_expected2);

    ret = cal_gn2_calibrated_orientations_from_database_get(&q_arm, &q_forearm);
    ok &= assert_OK(ret, "cal_gn2_calibrated_orientations_from_database_get 1");
    ok &= assert_quaternion(q_arm,    q_expected1,"cal_gn2_calibrated_orientations_from_database_get 1 result 1");
    ok &= assert_quaternion(q_forearm,q_expected2,"cal_gn2_calibrated_orientations_from_database_get 1 result 2");

    Quaternion_fromAxisAngle(x, M_PI/6, &q_sensor1);
    Quaternion_fromAxisAngle(z, M_PI_4, &q_sensor2);

    quaternion_buffer_build(q_sensor1,q_buff);
    ret = db_write(DB_IMU_QUATERNION,0,q_buff);
    ok &= assert_OK(ret, "db_write");
    quaternion_buffer_build(q_sensor2,q_buff);
    ret = db_write(DB_IMU_QUATERNION,1,q_buff);
    ok &= assert_OK(ret, "db_write");

    Quaternion_fromAxisAngle(x,5*M_PI/12,&q_expected1);
    Quaternion_fromAxisAngle(z,-M_PI/6, &q_expected2);
    
    ret = cal_gn2_calibrated_orientations_from_database_get(&q_arm, &q_forearm);
    ok &= assert_OK(ret, "cal_gn2_calibrated_orientations_from_database_get 2");
    ok &= assert_quaternion(q_arm,    q_expected1,"cal_gn2_calibrated_orientations_from_database_get 2 result 1");
    ok &= assert_quaternion(q_forearm,q_expected2,"cal_gn2_calibrated_orientations_from_database_get 2 result 2");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_006()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double x[3]={1,0,0};
    double z[3]={0,0,1};

    double angle1 = M_PI/12;
    double angle2 = M_PI/6;
    double rotVector1[3] = {0.0, sin(angle1), cos(angle1)};
    double rotVector2[3] = {cos(angle2), sin(angle2), 0.0};

    Quaternion q_sensor1;
    Quaternion q_sensor2;

    Quaternion q_body1;
    Quaternion q_body2;

    Quaternion q_zeroAndBody1, q_zeroAndBody2;

    Quaternion q_zab1_expected;
    Quaternion q_zab2_expected;

    double angle1x = 5*M_PI/12;
    double angle2z = -M_PI/6;
    double q_buff1[4] = {cos(angle1x/2),sin(angle1x/2),0.0,0.0};
    double q_buff2[4] = {cos(angle2z/2),0.0,0.0,sin(angle2z/2)};

    Quaternion q_relative;
    double angles[ARM_ELBOW_ANGLES_NUMBER];

    Quaternion q_expected = {.w = 0.3043807, .v={0.5272029, 0.6870641, 0.3966767}};;
    double angles_expected[ARM_ELBOW_ANGLES_NUMBER] = {DEG_2_RAD(120), 0, DEG_2_RAD(105)};

    testDescription(__FUNCTION__, "Check the computation of the relative orientation between the sensors orientations");
    ok = preconditions_init(__FUNCTION__);

    // Set simulated IMU readings
    Quaternion_fromAxisAngle(x, -M_PI_4,  &q_sensor1);
    Quaternion_fromAxisAngle(z, -M_PI/12, &q_sensor2);
    // Set expected body positions at zero 
    Quaternion_fromAxisAngle(x, 0,       &q_body1);
    Quaternion_fromAxisAngle(z, -M_PI_2, &q_body2);
    // Set zero to body conversion to be expected
    Quaternion_fromAxisAngle(x, M_PI_4,     &q_zab1_expected);
    Quaternion_fromAxisAngle(z, -5*M_PI/12, &q_zab2_expected);

    // Test Steps
    ret = cal_gn2_zero_pose_calibrate(
        rotVector1,rotVector2,
        q_sensor1, q_sensor2,
        q_body1, q_body2,
        &q_zeroAndBody1,&q_zeroAndBody2);
    ok &= assert_OK(ret, "cal_gn2_zero_pose_calibrate");
    ok &= assert_quaternion(q_zeroAndBody1,q_zab1_expected,"cal_gn2_zero_pose_calibrate result");
    ok &= assert_quaternion(q_zeroAndBody2,q_zab2_expected,"cal_gn2_zero_pose_calibrate result");    

    ret = db_write(DB_IMU_QUATERNION,0,q_buff1);
    ok &= assert_OK(ret, "db_write 0");
    ret = db_write(DB_IMU_QUATERNION,1,q_buff2);
    ok &= assert_OK(ret, "db_write 1");

    ret = cal_gn2_calibrated_relative_orientation_get(&q_relative,angles);
    ok &= assert_OK(ret, "cal_gn2_zero_pose_calibrate");
    ok &= assert_quaternion(q_relative,q_expected,"cal_gn2_calibrated_relative_orientation_get result");
    ok &= assert_vector3Equal(angles,angles_expected,"cal_gn2_calibrated_relative_orientation_get result");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_com_001()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    const char *ip      = COM_DEFAULT_IP;
    unsigned short port = COM_DEFAULT_PORT;

    testDescription(__FUNCTION__, "Create a UDP server");
    ok = preconditions_init(__FUNCTION__);

    tst_str("Intitialize UDP server in ip %s and port %u",ip,port);
    ret = com_server_initialize(ip,port,0);
    ok &= assert_OK(ret, "com_server_initialize");

    for (int i = 0; i < 5; i++) {
        sleep_ms(100);
        ret = com_send("Testing com_send");
        ok &= assert_OK(ret, "com_send");
    }

    for (int i = 0; i < 5; i++) {
        sleep_ms(100);
        ret = com_string_build_send("Testing com_string_build_send %d",i);
        ok &= assert_OK(ret, "com_string_build_send");
    }

    for (int i = 0; i < 5; i++) {
        sleep_ms(100);
        ret = com_string_build_send("Short str %d",i);
        ok &= assert_OK(ret, "com_string_build_send");
    }
    
    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_com_002()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    const char *ip      = COM_DEFAULT_IP;
    unsigned short port = COM_DEFAULT_PORT;

    testDescription(__FUNCTION__, "Create a UDP client");
    ok = preconditions_init(__FUNCTION__);

    tst_str("Intitialize UDP client for ip %s and port %u",ip,port);
    ret = com_client_initialize(ip,port,0);
    ok &= assert_OK(ret, "com_client_initialize");

    for (int i = 0; i < 5; i++) {
        sleep_ms(100);
        ret = com_send("Testing com_send");
        ok &= assert_OK(ret, "com_send");
    }

    for (int i = 0; i < 5; i++) {
        sleep_ms(100);
        ret = com_string_build_send("Testing com_string_build_send %d",i);
        ok &= assert_OK(ret, "com_string_build_send");
    }

    for (int i = 0; i < 5; i++) {
        sleep_ms(100);
        ret = com_string_build_send("Short str %d",i);
        ok &= assert_OK(ret, "com_string_build_send");
    }

    testCleanUp();
    testReport(ok);
    return ok;
}


#if 1 <= IMUS_CONNECTED
bool tst_imu_single_001() 
{
    bool ok = true;
    ERROR_CODE ret;
    COM_PORTS discoveredPorts;
    int expected_imusNum = 1;

    testDescription(__FUNCTION__, "Initialize a single IMU sensor");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = com_ports_list(&discoveredPorts);
    ok &= assert_OK(ret, "com_ports_list");
    ok &= assert_int_greater_or_equal(discoveredPorts.ports_number, 1, "com_ports_list result");
    ok &= assert_string_not_empty(discoveredPorts.ports_names[0], "com_ports_list result");

    if (ok) ret = imu_initialize(discoveredPorts.ports_names[0]);
    ok &= assert_OK(ret, "imu_initialize");
    ok &= assert_dbFieldIntGreaterEqual(DB_IMU_NUMBER, 0, &expected_imusNum, "db imus num");

    // ret = imu_initialize(NULL);
    // ok &= assert_ERROR(ret,"imu_initialize NULL arg0");

    // ret = imu_initialize("invalid");
    // ok &= assert_ERROR(ret,"imu_initialize invalid arg0");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_imu_single_002() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    ImuData data;
    double expected_quat[4] = {1.0, 0.0, 0.0, 0.0};

    testDescription(__FUNCTION__, "Attach the IMU data retrieve callback to generate a csv file for a set amount of time");
    ok = preconditions_init_imus(__FUNCTION__); 

    // Test Steps
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ret += db_csv_field_add(DB_IMU_QUATERNION,0);
    ret += db_csv_field_add(DB_IMU_GYROSCOPE,0);
    ok &= assert_OK(ret, "db_csv_field_add");

    ret = imu_read_callback_attach(0, true);
    ok &= assert_OK(ret, "imu_read_callback_attach");

    sleep_s(5);

    for (int i = 0; ok && i < 10; i++) {
        // tst_str("Check set and reset offset %d seconds in", i);
        ret = db_field_print(DB_IMU_QUATERNION, 0);
        ok &= assert_OK(ret, "db_field_print");
        
        if (i % 2 == 0) {
            if (i != 0) ok &= assert_dbFieldDoubleThreshold(DB_IMU_QUATERNION, 0, expected_quat, 5e-2,"quaternion");
            ret = imu_orientation_offset_reset();
            ok &= assert_OK(ret, "imu_orientation_offset_reset");
        }
        else
        {
            ret = imu_orientation_offset_set(1);
            ok &= assert_OK(ret, "imu_orientation_offset_set");
        }
        sleep_s(1);
    }

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_battery_imu_single()
{
    bool ok = true;

    ok &= tst_imu_single_001();
    ok &= tst_imu_single_002();

    testBatteryReport(__FUNCTION__, "SINGLE IMU", ok);
    return ok;
}
#endif

#if 2 <= IMUS_CONNECTED
bool tst_imu_two_001()
{
    bool ok = true;
    ERROR_CODE ret;
    COM_PORTS discoveredPorts;
    int expected_imusNum = 2;

    testDescription(__FUNCTION__, "Initialize two IMU sensors");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = com_ports_list(&discoveredPorts);
    ok &= assert_OK(ret, "com_ports_list");
    ok &= assert_int_greater_or_equal(discoveredPorts.ports_number, 2, "com_ports_list result");
    ok &= assert_string_not_empty(discoveredPorts.ports_names[0], "com_ports_list result");
    ok &= assert_string_not_empty(discoveredPorts.ports_names[1], "com_ports_list result");

    if (ok) ret = imu_batch_initialize(discoveredPorts, 2);
    ok &= assert_OK(ret, "imu_batch_initialize");
    ok &= assert_dbFieldIntGreaterEqual(DB_IMU_NUMBER, 0, &expected_imusNum, "db imus num");

    // ret = imu_initialize(NULL);
    // ok &= assert_ERROR(ret,"imu_initialize NULL arg0");

    // ret = imu_initialize("invalid");
    // ok &= assert_ERROR(ret,"imu_initialize invalid arg0");

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_imu_two_002() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;
    ImuData data;

    testDescription(__FUNCTION__, "Attach the IMU data retrieve callbacks to both IMUs and generate a csv file for a set amount of time");
    ok = preconditions_init_imus(__FUNCTION__); 

    // Test Steps
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ret += db_csv_field_add(DB_IMU_GYROSCOPE,0);
    ret += db_csv_field_add(DB_IMU_GYROSCOPE,1);
    ok &= assert_OK(ret, "db_csv_field_add");

    ret = imu_read_callback_attach(0, true);
    ok &= assert_OK(ret, "imu_read_callback_attach");
    ret = imu_read_callback_attach(1, false);
    ok &= assert_OK(ret, "imu_read_callback_attach");

    sleep_s(10);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_battery_imu_two()
{
    bool ok = true;

    ok &= tst_imu_two_001();
    ok &= tst_imu_two_002();

    testBatteryReport(__FUNCTION__, "TWO IMUS", ok);
    return ok;
}
#endif

bool tst_battery_all()
{
    bool ok = true;

    ok &= tst_math_001();
    ok &= tst_math_002();
    ok &= tst_math_003();
    ok &= tst_math_004();
    ok &= tst_math_005();
    ok &= tst_math_006();
    ok &= tst_math_007();
    ok &= tst_math_008();
    ok &= tst_math_009();
    ok &= tst_math_010();
    ok &= tst_math_011();
    ok &= tst_math_012();
    ok &= tst_math_013();
    ok &= tst_math_014();
    ok &= tst_math_015();
    ok &= tst_math_016();
    ok &= tst_math_017();
    ok &= tst_math_018();
    ok &= tst_math_019();
    ok &= tst_math_020();
    ok &= tst_math_021();
    ok &= tst_math_022();
    ok &= tst_math_023();
    ok &= tst_math_024();
    ok &= tst_math_025();
    ok &= tst_math_026();
    ok &= tst_math_027();
    ok &= tst_math_028();
    // ok &= tst_math_029(); // svd not being used r8 now
    ok &= tst_math_030();
    ok &= tst_math_031();

    ok &= tst_db_001();
    ok &= tst_db_002();
    ok &= tst_db_003();
    ok &= tst_db_004();
    ok &= tst_db_005();
    ok &= tst_db_006();
    ok &= tst_db_007();
    ok &= tst_db_008();
    ok &= tst_db_009();
    ok &= tst_db_010();
    ok &= tst_db_011();
    ok &= tst_db_012();
    ok &= tst_db_013();

    ok &= tst_arm_001();
    ok &= tst_arm_002();
    ok &= tst_arm_003();
    ok &= tst_arm_004();
    ok &= tst_arm_005();
    ok &= tst_arm_006();
    ok &= tst_arm_007();
    ok &= tst_arm_008();
    ok &= tst_arm_009();
    ok &= tst_arm_010();
    ok &= tst_arm_011();
    ok &= tst_arm_012();
    ok &= tst_arm_013();
    ok &= tst_arm_014();
    // ok &= tst_arm_015();
    ok &= tst_arm_016();
    ok &= tst_arm_017();
    // ok &= tst_arm_018();
    ok &= tst_arm_019();
    ok &= tst_arm_020();


    ok &= tst_cal_001();
    ok &= tst_cal_002();
    ok &= tst_cal_003();
    // ok &= tst_cal_004(); // TODO: currently working but very slow
    ok &= tst_cal_005();
    ok &= tst_cal_006();

    ok &= tst_com_001();
    ok &= tst_com_002();

#if 1 <= IMUS_CONNECTED
    ok &= tst_battery_imu_single();
#endif

#if 2 <= IMUS_CONNECTED
    ok &= tst_battery_imu_two();
#endif

    testBatteryReport(__FUNCTION__, "ALL TESTS", ok);
    return ok;
}


int main(int argc, char **argv)
{
    bool ok = true;

    testSetTraceLevel(SILENT_NO_ERROR);
    // testSetTraceLevel(ALL_TRACES);

    ok &= tst_battery_all();
    // ok &= tst_battery_imu_single();

    // ok &= tst_com_002();
    // ok &= tst_cal_xxx();
    // ok &= tst_arm_018();
    // ok &= tst_arm_016();
    // ok &= tst_cal_005();
    // ok &= tst_cal_006();
    // ok &= tst_arm_018();
    // ok &= tst_arm_016();
    // ok &= tst_math_030();
    // ok &= tst_math_031();
    // ok &= tst_math_016();
    // ok &= tst_math_029();

    return (ok)? RET_OK : RET_ERROR;
}