
#include "Quaternion.h"
#include "constants.h"
#include "arm.h"
#include "imu.h"
#include "vector3.h"
#include "database.h"

#include "tst_lib.h"


bool tst_math_001() 
{
    bool ok = true;
    ERROR_CODE ret;
    double v1[] = {1.0, 2.0, 3.0};
    double v2[] = {4.0, 5.0, 6.0};
    double v_result[3];
    double v_expected[] = {1.0+4.0, 2.0+5.0, 3.0+6.0};

    testDescription(__FUNCTION__, "Add two vectors ");
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init();

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
    ok = preconditions_init();

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
    ok = preconditions_init();

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
    ok = preconditions_init();

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
    ok = preconditions_initTraces(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

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
    ok = preconditions_init(); 

    // Test Steps
    for (int instance = 0; instance < IMU_MAX_NUMBER; instance++) {
        value = 1.0 + (double)instance;
        ret = db_write(field, instance, &value);
        ok &= assert_OK(ret,"db_write");
        ok &= assert_dbFieldDouble(field,instance,&value,"db_wrtie result");
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
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions");
    preconditions_init();

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
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotations");
    preconditions_init();

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
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 5.0, -10.0},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for shoulder and 90 degrees rotation in X axis (global frame) for elbow");
    preconditions_init();

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
        .shoulderPosition = {0.0, 0.0, 0.0 },
        .elbowPosition    = {0.0, 0.0,-10.0},
        .wristPosition    = {0.0, 0.0,-15.0},
    };
    const ARM_POSE expected2 = {
        .shoulderPosition = {0.0, 0.0,  0.0},
        .elbowPosition    = {0.0, 10.0,  0.0},
        .wristPosition    = {0.0, 15.0, 0.0},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in X axis (global frame) for shoulder");
    preconditions_init();

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
    const ARM_POSE expected = { // With the arm extended rotations in Z
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    testDescription(__FUNCTION__, "Rotate joints in Z axis (global frame) to see nothing happens");
    preconditions_init();

    // Test Steps
    Quaternion_fromZRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected, "arm position after first rot");

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromZRotation(PI/2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected, "arm position after second rot");

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
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 5.0, -10.0},
    };

    const ARM_POSE expected2 = { // Afrer shoulder rotation in Z
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {-5.0, 0.0, -10.0},
    };

    testDescription(__FUNCTION__, "Rotate joints in Z axis (global frame) to see nothing happens");
    preconditions_init();

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
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {-10.0, 0.0, 0.0},
        .wristPosition    = {-15.0, 0.0, 0.0},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in Y axis (global frame) for shoulder");
    preconditions_init();

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
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {-10.0*SQRT_2/2, 10.0*SQRT_2/2, 0.0},
        .wristPosition    = {-15.0*SQRT_2/2, 15.0*SQRT_2/2, 0.0},
    };

    testDescription(__FUNCTION__, "Rotate shoulder for a set of ZYX euler angles (45,90,0)");
    preconditions_init();

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

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omegaR[] = {1000.0,0.0,0.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over X axis");
    ok = preconditions_init(); 

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
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret, "arm_calibrate_rotation_axis");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2,"arm_calibrate_rotation_axis result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_010() 
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
    ok = preconditions_init(); 

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
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret, "arm_calibrate_rotation_axis");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2,"arm_calibrate_rotation_axis result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_011() 
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
    ok = preconditions_init(); 

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
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret, "arm_calibrate_rotation_axis");
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret, "db_csv_dump");
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret, "vector3_normalize");
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2,"arm_calibrate_rotation_axis result");

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_012()
{
    bool ok = true;
    ERROR_CODE ret = RET_OK;

    // Quaternion q1 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };
    // Quaternion q2 = { .w = 1.0, .v = {0.0, 0.0, 0.0} };

    Quaternion q1 = { .w = SQRT_2/2, .v = {SQRT_2/2, 0.0, 0.0} };
    Quaternion q2 = { .w = 0.5, .v = {0.5,-0.5, 0.5} };

    double omega1[3] = {M_PI_2, 0.0, 0.0};
    double omega2[3] = {M_PI_2, 0.0, M_PI_2};
    double omegaR[3] = {0.0};

    testDescription(__FUNCTION__, "Compute the angular velocity between two frames");
    ok = preconditions_init(); 

    ret = arm_relative_angular_vel_compute(q1, q2, omega1, omega2, omegaR);
    ok &= assert_OK(ret, "arm_relative_angular_vel_compute");

    printf("omegaR: %f, %f, %f\n",omegaR[0],omegaR[1],omegaR[2]);
    Quaternion q1_conj;
    double omegaR_R[3] = {0.0};
    Quaternion_conjugate(&q1,&q1_conj);
    Quaternion_rotate(&q1_conj,omegaR,omegaR_R);
    printf("omegaR_R: %f, %f, %f\n",omegaR_R[0],omegaR_R[1],omegaR_R[2]);


    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_013()
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
    ok = preconditions_init(); 

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
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret, "arm_calibrate_rotation_axis");
        /* Set timestamp */
        ret = db_index_write(DB_IMU_TIMESTAMP,0,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        ret = db_index_write(DB_IMU_TIMESTAMP,1,0,&time);
        ok &= assert_OK(ret, "db_index_write timestamp");
        /* Set quaternions */
        quaternion_buffer_build(q1, quat_buff);
        ret = db_write(DB_IMU_QUATERNION,0,quat_buff);
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
    ok &= assert_vector3EqualThreshold(rotVector, v_expected, 1e-1, "arm_calibrate_rotation_axis result");
    printf("rotVector: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}


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

    testBatteryReport(__FUNCTION__, "ALL TESTS", ok);
    return ok;
}

int main(int argc, char **argv)
{
    bool ok = true;

    testSetTraceLevel(SILENT_NO_ERROR);

    ok &= tst_battery_all();
    ok &= tst_arm_013();

    return (int)ok;
}