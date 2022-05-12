
#include "Quaternion.h"
#include "constants.h"
#include "arm.h"
#include "vector3.h"
#include "database.h"

#include "tst_lib.h"

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
    preconditions_initArm();

    // Test Steps
    arm = arm_pose_get();
    ok &= assert_armEqual(arm, expected);

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
    preconditions_initArm();

    // Test Steps
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

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
    preconditions_initArm();

    // Test Steps
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromXRotation(PI/2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_004()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 10.0, 0.0},
        .wristPosition    = {0.0, 15.0, 0.0},
    };

    testDescription(__FUNCTION__, "Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in X axis (global frame) for shoulder");
    preconditions_initArm();

    // Test Steps
    Quaternion_fromXRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

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
    preconditions_initArm();

    // Test Steps
    Quaternion_fromZRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromZRotation(PI/2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

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
    preconditions_initArm();

    // Test Steps
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromXRotation(PI/2, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected1);

    Quaternion_fromZRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected2);

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
    preconditions_initArm();

    // Test Steps
    Quaternion_fromYRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

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
    preconditions_initArm();

    // Test Steps
    Quaternion_fromEulerZYX(eulerAngles, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = arm_rotate(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_009() 
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER fields_monitored[] = {
        DB_IMU_TIMESTAMP,
        DB_CALIB_OMEGA,
        DB_CALIB_OMEGA_NORM,
        DB_CALIB_ERROR,
        DB_CALIB_ROT_VECTOR,
        DB_CALIB_SPHERICAL_COORDS,
        DB_CALIB_COST_DERIVATIVE
    };
    int num_fields = sizeof(fields_monitored)/sizeof(DB_FIELD_IDENTIFIER);  

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omegaR[] = {1000.0,0.0,0.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over X axis");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_csv_setup(fields_monitored,num_fields);
    ok &= assert_OK(ret);

    while (ok && time<timeout)
    {
        // Set timesetamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,&time);
        ok &= assert_OK(ret);
        time += timeInc;
        // Execute arm calibration of a single rotation axis
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret);
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret);
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret);
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2);

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_010() 
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER fields_monitored[] = {
        DB_IMU_TIMESTAMP,
        DB_CALIB_OMEGA,
        DB_CALIB_OMEGA_NORM,
        DB_CALIB_ERROR,
        DB_CALIB_ROT_VECTOR,
        DB_CALIB_SPHERICAL_COORDS,
        DB_CALIB_COST_DERIVATIVE
    };
    int num_fields = sizeof(fields_monitored)/sizeof(DB_FIELD_IDENTIFIER);  

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 20.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omegaR[] = {0.0,1000.0,0.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over Y axis");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_csv_setup(fields_monitored,num_fields);
    ok &= assert_OK(ret);

    while (ok && time<timeout)
    {
        // Set timesetamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,&time);
        ok &= assert_OK(ret);
        time += timeInc;
        // Execute arm calibration of a single rotation axis
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret);
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret);
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret);
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2);

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_011() 
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER fields_monitored[] = {
        DB_IMU_TIMESTAMP,
        DB_CALIB_OMEGA,
        DB_CALIB_OMEGA_NORM,
        DB_CALIB_ERROR,
        DB_CALIB_ROT_VECTOR,
        DB_CALIB_SPHERICAL_COORDS,
        DB_CALIB_COST_DERIVATIVE
    };
    int num_fields = sizeof(fields_monitored)/sizeof(DB_FIELD_IDENTIFIER);  

    double rotVector[3]    = {0.5,0.5,0.5};
    double timeout = 5.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    double omegaR[] = {0.0,0.0,1000.0};
    double v_expected[3];

    testDescription(__FUNCTION__, "Test one rotation axis calibration over Z axis");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_csv_setup(fields_monitored,num_fields);
    ok &= assert_OK(ret);

    while (ok && time<timeout)
    {
        // Set timesetamp
        ret = db_index_write(DB_IMU_TIMESTAMP,0,&time);
        ok &= assert_OK(ret);
        time += timeInc;
        // Execute arm calibration of a single rotation axis
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret);
        // Dump database data
        ret = db_csv_dump();
        ok &= assert_OK(ret);
    }
    ret = vector3_normalize(omegaR,v_expected);
    ok &= assert_OK(ret);
    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2);

    // printf("rotv: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_arm_012()
{
    bool ok = true;
    ERROR_CODE ret;
    
    DB_FIELD_IDENTIFIER fields_monitored[] = {
        DB_IMU_TIMESTAMP,
        DB_CALIB_OMEGA,
        DB_CALIB_OMEGA_NORM,
        DB_CALIB_ERROR,
        DB_CALIB_ROT_VECTOR,
        DB_CALIB_SPHERICAL_COORDS,
        DB_CALIB_COST_DERIVATIVE
    };
    int num_fields = sizeof(fields_monitored)/sizeof(DB_FIELD_IDENTIFIER);  

    double timeout = 5.0;/*(seconds)*/
    double timeInc = 0.02;/*(seconds)*/
    double time = 0.0;

    Quaternion q1, q2, q1_, q21;
    double omega1[] = {100.0,0.0,0.0};
    double omega2[] = {100.0,0.0,1000.0};
    Quaternion dq1, dq2;
    double w1_norm, w2_norm;
    double et2v1, et2v2;
    double omega2_1[3];
    double omegaR[3];
    double rotVector[3] = {0.5,0.5,0.5};
    double v_expected[3] = {0.0,0.0,1.0};

    testDescription(__FUNCTION__, "Emulate 2 different IMU sensors rotating");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_csv_setup(fields_monitored,num_fields);
    ok &= assert_OK(ret);

    /* Set initial quaternions */
    Quaternion_set(1,0,0,0,&q1);
    Quaternion_set(1,0,0,0,&q2);

    /* Set quaternion derivatives */
    ret = vector3_norm(omega1,&w1_norm);
    ok &= assert_OK(ret);
    if (EPSI < w1_norm) {
        et2v1 = (timeInc*0.5*sin(w1_norm))/w1_norm;
        Quaternion_set(0, et2v1*omega1[0], et2v1*omega1[1], et2v1*omega1[2],&dq1);
    } else {
        Quaternion_set(1,0,0,0,&dq1);
    }
    ret = vector3_norm(omega2,&w2_norm);
    ok &= assert_OK(ret);
    if (EPSI < w1_norm) {    
        et2v2 = (timeInc*0.5*sin(w2_norm))/w2_norm;
        Quaternion_set(0, et2v2*omega2[0], et2v2*omega2[1], et2v2*omega1[2],&dq2);
    } else {
        Quaternion_set(1,0,0,0,&dq2);
    }
    printf("dq1: %f, %f, %f, %f\n",dq1.w,dq1.v[0],dq1.v[1],dq1.v[2]);
    printf("dq2: %f, %f, %f, %f\n",dq2.w,dq2.v[0],dq2.v[1],dq2.v[2]);

    do {
        /* Compute q21 */
        Quaternion_conjugate(&q1,&q1_);
        Quaternion_multiply(&q1_,&q2,&q21);
        /* Compute relative w */
        Quaternion_rotate(&q21, omega2, omega2_1);
        ret = vector3_substract(omega2_1, omega1, omegaR);
        ok &= assert_OK(ret);
        /* Calibrate rotation axis */
        ret = arm_calibrate_rotation_axis(omegaR,rotVector);
        ok &= assert_OK(ret);
        /* Dump database data */
        ret = db_csv_dump();
        ok &= assert_OK(ret);
        /* Set new quaternion */
        Quaternion_multiply(&q1,&dq1,&q1);
        Quaternion_normalize(&q1,&q1);
        Quaternion_multiply(&q2,&dq2,&q2);
        Quaternion_normalize(&q2,&q2);
        /* Set new time */
        time += timeInc;
        // printf("omegaR: %f, %f, %f\t| ",omegaR[0],omegaR[1],omegaR[2]);
        // printf("rotVector: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);
    printf("q1: %f, %f, %f, %f\n",q1.w,q1.v[0],q1.v[1],q1.v[2]);
    printf("q2: %f, %f, %f, %f\n",q2.w,q2.v[0],q2.v[1],q2.v[2]);

    } while (ok && time<timeout);

    ok &= assert_vector3EqualThreshold(rotVector,v_expected,5e-2);
    printf("rotVector: %f, %f, %f\n",rotVector[0],rotVector[1],rotVector[2]);

    // exp(q) = e^w(cos(|v|) + (v/|v|)*sin(|v|))
    // qtp1 = qt * exp(T/2*w)
    // printf("aux: %f, %f, %f, %f\n",aux2.w,aux2.v[0],aux2.v[1],aux2.v[2]);

    testCleanUp();
    testReport(ok);
    return ok;
}

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
    ok &= assert_OK(ret);
    ok &= assert_vector3Equal(v_result,v_expected);

    ret = vector3_add(NULL, v2, v_result);
    ok &= assert_ERROR(ret);

    ret = vector3_add(v1, NULL, v_result);
    ok &= assert_ERROR(ret);

    ret = vector3_add(v1, v2, NULL);
    ok &= assert_ERROR(ret);

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
    ok &= assert_OK(ret);
    ok &= assert_vector3Equal(v_result,v_expected);

    ret = vector3_substract(NULL, v2, v_result);
    ok &= assert_ERROR(ret);

    ret = vector3_substract(v1, NULL, v_result);
    ok &= assert_ERROR(ret);

    ret = vector3_substract(v1, v2, NULL);
    ok &= assert_ERROR(ret);

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
    ok &= assert_OK(ret);
    ok &= assert_vector3Equal(v_result,v_expected);

    ret = vector3_rotate90y(NULL, v_result);
    ok &= assert_ERROR(ret);

    ret = vector3_rotate90y(v, NULL);
    ok &= assert_ERROR(ret);

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
    ok &= assert_OK(ret);
    ok &= assert_vector3Equal(v_result,v_expected);

    ret = vector3_rotateMinus90y(NULL, v_result);
    ok &= assert_ERROR(ret);

    ret = vector3_rotateMinus90y(v, NULL);
    ok &= assert_ERROR(ret);

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
    ok &= assert_OK(ret);

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
    ok &= assert_OK(ret);

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_003()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_ACCELEROMETER;
    double buff[3];
    double buff_expected[] = {0.0, 0.0, 0.0};

    testDescription(__FUNCTION__, "Read default value from the database");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_read(field,buff);
    ok &= assert_OK(ret);
    ok &= assert_vector3Equal(buff, buff_expected);

    ret = db_read(DB_NUMBER_OF_ENTRIES,buff);
    ok &= assert_ERROR(ret);

    ret = db_read(field,NULL);
    ok &= assert_ERROR(ret);

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_004()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
    double buff[4] = {1.0, 2.0, 3.0, 4.0};
    double buff_expected[] = {1.0, 2.0, 3.0, 4.0};

    testDescription(__FUNCTION__, "Write values to a field of the database");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_write(field,buff);
    ok &= assert_OK(ret);
    ok &= assert_dbFieldDouble(field, buff_expected);

    ret = db_write(DB_NUMBER_OF_ENTRIES,buff);
    ok &= assert_ERROR(ret);

    ret = db_write(field,NULL);
    ok &= assert_ERROR(ret);

    // testCleanUp();
    testReport(ok);
    return ok;
}


bool tst_db_005()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
    double buff[4] = {1.0, 2.0, 3.0, 4.0};
    double value;
    int ind;
    double buff_expected[] = {1.0, 2.0, 3.0, 4.0};

    testDescription(__FUNCTION__, "Read a single index of a database field");
    ok = preconditions_init(); 

    // Test Steps
    ret = db_write(field, buff);
    ok &= assert_OK(ret);
    ok &= assert_dbFieldDouble(field, buff_expected);

    for(ind = 0; ind < 4; ind++) {
        ret = db_index_read(field, ind, &value);
        ok &= assert_OK(ret);
        ok &= (EPSI > fabs(buff_expected[ind]-value));
    }

    ret = db_index_read(DB_NUMBER_OF_ENTRIES,ind,buff);
    ok &= assert_ERROR(ret);

    ret = db_index_read(field,100,buff);
    ok &= assert_ERROR(ret);

    ret = db_index_read(field,ind,NULL);
    ok &= assert_ERROR(ret);

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_db_006()
{
    bool ok = true;
    ERROR_CODE ret;
    DB_FIELD_IDENTIFIER field = DB_IMU_QUATERNION;
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
    ok &= assert_dbFieldDouble(field, buff_expected1);
    for(ind = 0; ind < 4; ind++) {
        value = buff[ind];
        ret = db_index_write(field, ind, &value);
        ok &= assert_OK(ret);
        ok &= assert_dbFieldDouble(field, buff_expected2[ind]);
    }
    value = 5.0;
    ret = db_index_write(field, 2, &value);
    ok &= assert_OK(ret);
    ok &= assert_dbFieldDouble(field, buff_expected3);

    ret = db_index_write(DB_NUMBER_OF_ENTRIES,ind,&value);
    ok &= assert_ERROR(ret);

    ret = db_index_write(field,100,buff);
    ok &= assert_ERROR(ret);

    ret = db_index_write(field,ind,NULL);
    ok &= assert_ERROR(ret);

    // testCleanUp();
    testReport(ok);
    return ok;
}

bool tst_battery_all()
{
    bool ok = true;

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

    ok &= tst_math_001();
    ok &= tst_math_002();
    ok &= tst_math_003();
    ok &= tst_math_004();

    ok &= tst_db_001();
    ok &= tst_db_002();
    ok &= tst_db_003();
    ok &= tst_db_004();
    ok &= tst_db_005();
    ok &= tst_db_006();

    testBatteryReport(__FUNCTION__, "ALL TESTS", ok);
    return ok;
}

int main(int argc, char **argv)
{
    bool ok = true;

    testSetTraceLevel(SILENT_NO_ERROR);

    // ok &= tst_battery_all();
    ok &= tst_arm_012();

    return (int)ok;
}