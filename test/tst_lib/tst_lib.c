#include <stdio.h>
#include "tst_lib.h"
#include "arm.h"
#include "vector3.h"
#include "general.h"
#include "errors.h"

LOG_LEVEL testTraceLevel = SILENT;

#define WILL_PRINT(ok) ((ALL_TRACES == testTraceLevel) || ((SILENT_NO_ERROR == testTraceLevel) && (true != ok)))

// * TEST FUNCTIONS ***********************************************************

void testSetTraceLevel(LOG_LEVEL testTraceLvl){testTraceLevel = testTraceLvl;}

void testDescription(const char *name, const char *description) {
    printf("Test %s: %s \n", name, description);
}

void testReport(bool result)
{
    printf("\t -> RESULT: %s \n",(true == result)?"PASSED":"FAILED");
}

void testBatteryReport(const char *name, const char *description, bool result)
{
    printf("\n");
    printf("******************************************************************************** \n");
    printf("***** Test battery %s: %s -> RESULT: %s \n", name, description, (true == result)?"PASSED":"FAILED");
    printf("******************************************************************************** \n");
}

void testCleanUp()
{
    db_terminate();
}

// * PRECONDITIONS ************************************************************

bool preconditions_initArm()
{
    static ARM_POSE currentPose = {
        .shoulderPosition = {0.0, 0.0, 0.0},
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };
    arm_joint_positions_set(currentPose);

    return true;
}

bool preconditions_initTraces() {
    bool ok = true;
    ERROR_CODE ret;

    ret = trace_level_set(NONE,DEBUG);
    ok &= assert_OK(ret);

    ret = log_file_initalize();
    ok &= assert_OK(ret);

    return ok;
}

bool preconditions_init()
{
    bool ok = true;
    ERROR_CODE ret;

    ok = preconditions_initTraces();

    ret = db_initialize();
    ok &= assert_OK(ret);

    return ok;
}

// * ASSERTS ******************************************************************

bool assert_OK(ERROR_CODE status) {
    bool ok = true;

    if (RET_OK != status) {
        ok = false;
    }
    if WILL_PRINT(ok) {
        printf("\t -> RESULT: %s | NOT EXPECTED: %d, ACTUAL: %d\n",
            (true == ok)?"PASSED":"FAILED", RET_OK, status);
    }
    return ok;
}

bool assert_ERROR(ERROR_CODE status) {
    bool ok = true;
    if (RET_OK == status) {
        ok = false;
    }
    if WILL_PRINT(ok) {
        printf("\t -> RESULT: %s | EXPECTED: %d, ACTUAL: %d\n",
            (true == ok)?"PASSED":"FAILED", RET_OK, status);
    }
    return ok;
}

bool assert_vector3EqualThreshold(const double actual[3], const double expected[3], const double threshold)
{
    bool ok = true;

    ok &= fabs(actual[0] - expected[0]) < threshold;
    ok &= fabs(actual[1] - expected[1]) < threshold;
    ok &= fabs(actual[2] - expected[2]) < threshold;
    if WILL_PRINT(ok) {
        printf("\t -> RESULT: %s | EXPECTED: [%f,%f,%f], ACTUAL: [%f,%f,%f] \n",
            (true == ok)?"PASSED":"FAILED",expected[0],expected[1],expected[2],actual[0],actual[1],actual[2]);
    }
    return ok;
}

bool assert_vector3Equal(const double actual[3], const double expected[3])
{
    return assert_vector3EqualThreshold(actual,expected,EPSI);
}

bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected)
{
    bool ok = true;

    ok &= assert_vector3Equal(actual.shoulderPosition,  expected.shoulderPosition);
    ok &= assert_vector3Equal(actual.elbowPosition,     expected.elbowPosition);
    ok &= assert_vector3Equal(actual.wristPosition,     expected.wristPosition);

    if WILL_PRINT(ok) {
        printf("\t -> RESULT: %s \n",(true == ok)?"PASSED":"FAILED");
        printf("EXPECTED: \n");
        arm_pose_print(expected);
        printf("ACTUAL: \n");
        arm_pose_print(actual);
    }

    return ok;
}

bool assert_dbFieldDouble(DB_FIELD_IDENTIFIER field, int instance, double expected[]){
    bool ok = true;
    ERROR_CODE ret;
    int size;
    double buff[100];
    double diff;

    ret = db_field_parameters_get(field, &size, NULL, NULL);
    ok &= assert_OK(ret);

    ret = db_read(field, instance, buff);
    ok &= assert_OK(ret);
    
    for (int i = 0; i < size; i++) {
        diff = fabs(buff[0]-expected[0]);
        ok &= (EPSI > diff);
    }
    if WILL_PRINT(ok) {
        printf("\t -> RESULT: %s | EXPECTED: [", (true == ok)?"PASSED":"FAILED");
        for (int i = 0; i < size; i++) {
            printf("%f",expected[i]);
            if (size-1 > i)printf(", ");
        }
        printf("], ACTUAL: [");
        for (int i = 0; i < size; i++) {
            printf("%f",buff[i]);
            if (size-1 > i)printf(", ");
        }
        printf("]\n");
    }

    return ok;
}