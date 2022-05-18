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
    preconditions_initArm();
}

// * PRECONDITIONS ************************************************************

bool preconditions_initArm()
{
    static ARM_POSE currentPose = {
        .shoulder.position    = {0.0, 0.0, 0.0},
        .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .elbow.position       = {0.0, 0.0, -10.0},
        .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
        .wrist.position       = {0.0, 0.0, -15.0},
        .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    };
    arm_joint_positions_set(currentPose);

    return true;
}

bool preconditions_initTraces() {
    bool ok = true;
    ERROR_CODE ret;

    ret = trace_level_set(NONE,DEBUG);
    ok &= assert_OK(ret,"trace_level_set");

    ret = log_file_initalize();
    ok &= assert_OK(ret,"log_file_initalize");

    return ok;
}

bool preconditions_init()
{
    bool ok = true;
    ERROR_CODE ret;

    ok = preconditions_initTraces();

    ret = db_initialize();
    ok &= assert_OK(ret,"db_initialize");

    return ok;
}

// * ASSERTS ******************************************************************

bool assert_OK(ERROR_CODE status, const char *description) {
    bool ok = true;

    if (RET_OK != status) {
        ok = false;
    }
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | NOT EXPECTED: %d, ACTUAL: %d\n",
            (true == ok)?"PASSED":"FAILED", description, RET_OK, status);
    }
    return ok;
}

bool assert_ERROR(ERROR_CODE status, const char *description) {
    bool ok = true;
    if (RET_OK == status) {
        ok = false;
    }
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %d, ACTUAL: %d\n",
            (true == ok)?"PASSED":"FAILED",description, RET_OK, status);
    }
    return ok;
}

bool assert_double(double actual, double expected, double threshold, const char *description) {
    bool ok = true;
    ERROR_CODE ret;

    ok = fabs(actual - expected) < threshold;
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %f, ACTUAL: %f\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, actual);
    }
    return ok;
}

bool assert_vector3EqualThreshold(const double actual[3], const double expected[3], const double threshold, const char *description)
{
    bool ok = true;

    ok &= assert_double(actual[0],expected[0],threshold,NULL);
    ok &= assert_double(actual[1],expected[1],threshold,NULL);
    ok &= assert_double(actual[2],expected[2],threshold,NULL);
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: [%f,%f,%f], ACTUAL: [%f,%f,%f] \n",
            (true == ok)?"PASSED":"FAILED", description, expected[0],expected[1],expected[2],actual[0],actual[1],actual[2]);
    }
    return ok;
}

bool assert_vector3Equal(const double actual[3], const double expected[3], const char *description)
{
    return assert_vector3EqualThreshold(actual,expected,EPSI,description);
}

bool assert_quaternionThreshold(Quaternion actual, Quaternion expected, double threshold, const char *description){
    bool ok = true;

    ok &= assert_double(actual.w, expected.w, threshold, NULL);
    ok &= assert_vector3EqualThreshold(actual.v, expected.v, threshold, NULL);

    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: [%f,%f,%f,%f], ACTUAL: [%f,%f,%f,%f] \n",
            (true == ok)?"PASSED":"FAILED", description, 
            expected.w, expected.v[0],expected.v[1],expected.v[2],
            actual.w, actual.v[0],actual.v[1],actual.v[2]);
    }

    return ok;
}

bool assert_quaternion(Quaternion actual, Quaternion expected, const char *description){
    return assert_quaternionThreshold(actual, expected, EPSI, description);
}

bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected, const char *description)
{
    bool ok = true;

    ok &= assert_vector3Equal(actual.shoulder.position, expected.shoulder.position, "arm assert -> shoulder pos");
    ok &= assert_vector3Equal(actual.elbow.position,    expected.elbow.position,    "arm assert -> elbow pos   ");
    ok &= assert_vector3Equal(actual.wrist.position,    expected.wrist.position,    "arm assert -> wrist pos   ");

    ok &= assert_quaternion(actual.shoulder.orientation, expected.shoulder.orientation, "arm assert -> shoulder ori");
    ok &= assert_quaternion(actual.elbow.orientation,    expected.elbow.orientation,    "arm assert -> elbow ori   ");
    ok &= assert_quaternion(actual.wrist.orientation,    expected.wrist.orientation,    "arm assert -> wrist ori   ");

    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | ",(true == ok)?"PASSED":"FAILED", description);
        printf("EXPECTED: EQUAL, ACTUAL: %s\n",(ok)?"EQUAL":"NOT EQUAL");
    }

    return ok;
}

bool assert_dbFieldDouble(DB_FIELD_IDENTIFIER field, int instance, double expected[], const char *description){
    bool ok = true;
    ERROR_CODE ret;
    int size;
    double buff[100];

    ret = db_field_parameters_get(field, &size, NULL, NULL);
    ok &= assert_OK(ret, "db_field_parameters_get");

    ret = db_read(field, instance, buff);
    ok &= assert_OK(ret, "db_read");
    
    for (int i = 0; i < size; i++) {
        ok &= assert_double(buff[0],expected[0],EPSI/*threshold*/,NULL);
    }
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: [", (true == ok)?"PASSED":"FAILED", description);
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