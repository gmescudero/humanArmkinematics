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
#include "arm.h"
#include "imu.h"
#include "vector3.h"
#include "database.h"
#include "calib.h"

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

    Quaternion q2 = { .w = SQRT_2/2, .v = {SQRT_2/2, 0.0, 0.0} };
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
    ok &= assert_double(Quaternion_norm(&q_result), pow(E,SQRT_2/2), EPSI, "quaternion_exponential result norm");

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
    Quaternion q3_exp_expected = {.w = SQRT_2/2, .v = {0.0,0.0,SQRT_2/2}};

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
    Quaternion q3_exp_expected = {.w = SQRT_2/2, .v = {0.0,SQRT_2/2,0.0}};

    double T4 = 2.0;
    Quaternion q4_exp_expected = {.w = -1.0, .v = {0.0,0.0,0.0}};


    testDescription(__FUNCTION__, "Apply an angular velocity in Y axis to a quaternion");
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
    Quaternion q3_exp_expected = {.w = SQRT_2/2, .v = {SQRT_2/2,0.0,0.0}};

    double T4 = 2.0;
    Quaternion q4_exp_expected = {.w = -1.0, .v = {0.0,0.0,0.0}};


    testDescription(__FUNCTION__, "Apply an angular velocity in X axis to a quaternion");
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
    Quaternion_fromXRotation(PI/2, &q2);
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

    Quaternion_fromXRotation(PI/2, &q1);
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

    Quaternion_fromZRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected2, "arm position after first rot");

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromZRotation(PI/2, &q2);
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
    Quaternion_fromXRotation(PI/2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected1, "arm position after first rot");

    Quaternion_fromZRotation(PI/2, &q1);
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
    Quaternion_fromYRotation(PI/2, &q1);
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
    double eulerAngles[3] = {PI/4, PI/2, 0.0};
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
        .elbow.position       = {10.0, 0.0, 0.0},
        .elbow.orientation    = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        .wrist.position       = {15.0, 0.0, 0.0},
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
        {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        {.w = 1.0, .v = {0.0, 0.0, 0.0}},
    };
    ARM_POSE result;
    ARM_POSE expected = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
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
        .elbow.position       = {11.0, 0.0, 0.0},
        .elbow.orientation    = {.w = 1.0, .v = {0.0, 0.0, 0.0}},
        .wrist.position       = {17.0, 0.0, 0.0},
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
    ARM_POSE result;
    ARM_POSE expected1 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    ARM_POSE expected2 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.0, .v = {0.0, 1.0, 0.0}},
        .wrist.position       = {-5.0, 0.0, -10.0},
        .wrist.orientation    = {.w = 0.0, .v = {0.0, 1.0, 0.0}},
    };
    ARM_POSE expected3 = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = 0.5, .v = {0.5, 0.5,-0.5}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = 0.0, .v = {M_SQRT1_2, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 5.0, -10.0},
        .wrist.orientation    = {.w = 0.0, .v = {M_SQRT1_2, M_SQRT1_2, 0.0}},
    };

    testDescription(__FUNCTION__, "Apply direct kinematics for different sets of joint values for shoulder and elbow");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret = arm_direct_kinematics_compute(joints1, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected1, "arm_direct_kinematics_compute result");

    ret = arm_direct_kinematics_compute(joints2, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected2, "arm_direct_kinematics_compute result");

    ret = arm_direct_kinematics_compute(joints3, &result);
    ok &= assert_OK(ret, "arm_direct_kinematics_compute");
    ok &= assert_armEqual(result, expected3, "arm_direct_kinematics_compute result");

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

bool tst_cal_001() 
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omegaR[] = {1000.0,0.0,0.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over X axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA_NORM,0);
    ret += db_csv_field_add(DB_CALIB_ERROR,0);
    ret += db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
    ret += db_csv_field_add(DB_CALIB_SPHERICAL_COORDS,0);
    ret += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
    ok &= assert_OK(ret, "db csv fields add");

    while (ok && time<timeout)
    {
        // Set timestamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        time += timeInc;
        // Execute arm calibration of a single rotation axis
        ret = cal_automatic_rotation_axis_calibrate(omegaR,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2,"cal_automatic_rotation_axis_calibrate result");

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

    double omegaR[] = {0.0,1000.0,0.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over Y axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA_NORM,0);
    ret += db_csv_field_add(DB_CALIB_ERROR,0);
    ret += db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
    ret += db_csv_field_add(DB_CALIB_SPHERICAL_COORDS,0);
    ret += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
    ok &= assert_OK(ret, "db csv fields add");

    while (ok && time<timeout)
    {
        // Set timestamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        time += timeInc;
        // Execute arm calibration of a single rotation axis
        ret = cal_automatic_rotation_axis_calibrate(omegaR,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2,"cal_automatic_rotation_axis_calibrate result");

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
    double timeout = 5.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omegaR[] = {0.0,0.0,1000.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over Z axis");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA_NORM,0);
    ret += db_csv_field_add(DB_CALIB_ERROR,0);
    ret += db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
    ret += db_csv_field_add(DB_CALIB_SPHERICAL_COORDS,0);
    ret += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
    ok &= assert_OK(ret, "db csv fields add");

    while (ok && time<timeout)
    {
        // Set timestamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        time += timeInc;
        // Execute arm calibration of a single rotation axis
        ret = cal_automatic_rotation_axis_calibrate(omegaR,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2,"cal_automatic_rotation_axis_calibrate result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_cal_xxx()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    double timeout = 100.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    Quaternion q1 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };
    Quaternion q2 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };
    Quaternion q1_,q21;
    double quat_buff[4];

    double omega1[3] = {M_PI_2, 0.0, 0.0};
    double omega2[3] = {M_PI_2, 0.0, M_PI_2};
    double omega2_1[3], omegaR[3];

    double rotVector[3]  = {0.5, 0.5, 0.5};
    double v_expected[3] = {0.0, 0.0, 1.0};
    double rotNorm;

    testDescription(__FUNCTION__, "Emulate 2 different IMU sensors rotating");
    ok = preconditions_init(__FUNCTION__); 

    // Test Steps
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ret += db_csv_field_add(DB_IMU_TIMESTAMP,1);
    ret += db_csv_field_add(DB_IMU_QUATERNION,0);
    ret += db_csv_field_add(DB_IMU_QUATERNION,1);
    ret += db_csv_field_add(DB_CALIB_OMEGA,0);
    ret += db_csv_field_add(DB_CALIB_OMEGA_NORM,0);
    ret += db_csv_field_add(DB_CALIB_ERROR,0);
    ret += db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
    ret += db_csv_field_add(DB_CALIB_SPHERICAL_COORDS,0);
    ret += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
    ok &= assert_OK(ret, "db csv fields add");

    do {
        /* Compute q21 */
        Quaternion_conjugate(&q1,&q1_);
        Quaternion_multiply(&q1_,&q2,&q21);
        /* Compute relative w */
        Quaternion_rotate(&q21, omega2, omega2_1);
        ret = vector3_substract(omega2_1, omega1, omegaR);
        ok &= assert_OK(ret, "vector3_substract");
        /* Calibrate rotation axis */
        ret = cal_automatic_rotation_axis_calibrate(omegaR,rotVector);
        ok &= assert_OK(ret, "cal_automatic_rotation_axis_calibrate");
        /* Set timestamp */
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        ret = db_index_write(DB_IMU_TIMESTAMP,1,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        /* Set quaternions */
        quaternion_buffer_build(q1, quat_buff);
        ret = db_write(DB_IMU_QUATERNION,0,quat_buff);
        dbg_str("Q1: %f, %f, %f, %f",quat_buff[0],quat_buff[1],quat_buff[2],quat_buff[3]);
        quaternion_buffer_build(q2, quat_buff);
        ret = db_write(DB_IMU_QUATERNION,1,quat_buff);
        /* Dump database data */
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");

        // printf("q1: %f, %f, %f, %f\n",q1.w,q1.v[0],q1.v[1],q1.v[2]);
        // printf("q2: %f, %f, %f, %f\n",q2.w,q2.v[0],q2.v[1],q2.v[2]);
        // printf("q21: %f, %f, %f, %f\n",q21.w,q21.v[0],q21.v[1],q21.v[2]);

        /* Set new time */
        time += timeInc;
        /* Compute next quaternions */
        ret = quaternion_ang_vel_apply(q1, timeInc, omega1, &q1);
        ok &= assert_OK(ret, "quaternion_ang_vel_apply");
        ret = quaternion_ang_vel_apply(q2, timeInc, omega2, &q2);
        ok &= assert_OK(ret, "quaternion_ang_vel_apply");

    } while (ok && time<timeout);

    ret = vector3_norm(rotVector,&rotNorm);
    ok &= assert_OK(ret, "vector3_norm");
    ok &= assert_double(rotNorm, 1.0, EPSI, "vector3_norm result");
    ok &= assert_vector3EqualThreshold(rotVector, v_expected, 1e-1, "cal_automatic_rotation_axis_calibrate result");
    printf("rotVector: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

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

    ok &= tst_db_001();
    ok &= tst_db_002();
    ok &= tst_db_003();
    ok &= tst_db_004();
    ok &= tst_db_005();
    ok &= tst_db_006();
    ok &= tst_db_007();
    ok &= tst_db_008();

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
    ok &= tst_arm_015();

    ok &= tst_cal_001();
    ok &= tst_cal_002();
    ok &= tst_cal_003();

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

    // ok &= tst_arm_014();
    // ok &= tst_cal_xxx();
    // ok &= tst_cal_005();
    // ok &= tst_arm_015();

    return (ok)? RET_OK : RET_ERROR;
}