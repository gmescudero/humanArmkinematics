#include <stdio.h>
#include "tst_lib.h"
#include "directKinematics.h"

LOG_LEVEL testTraceLevel = SILENT;

#define WILL_PRINT(ok) ((ALL_TRACES == testTraceLevel) || ((SILENT_NO_ERROR == testTraceLevel) && (true != ok)))

void testSetTraceLevel(LOG_LEVEL testTraceLvl){testTraceLevel = testTraceLvl;}

void testReport(const char *name, bool result)
{
    printf(" -> Test: %s, result: %s \n",name,(true == result)?"PASSED":"FAILED");
}

bool assert_vector3Equal(const double actual[3], const double expected[3])
{
    bool ok = true;

    ok &= fabs(actual[0] - expected[0]) < EPSI;
    ok &= fabs(actual[1] - expected[1]) < EPSI;
    ok &= fabs(actual[2] - expected[2]) < EPSI;
    if WILL_PRINT(ok)
    {
        printf("\t -> RESULT: %s | EXPECTED: [%f,%f,%f], ACTUAL: [%f,%f,%f] \n",
            (true == ok)?"PASSED":"FAILED",expected[0],expected[1],expected[2],actual[0],actual[1],actual[2]);
    }
    return ok;
}

bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected)
{
    bool ok = true;

    ok &= assert_vector3Equal(actual.shoulderPosition,  expected.shoulderPosition);
    ok &= assert_vector3Equal(actual.elbowPosition,     expected.elbowPosition);
    ok &= assert_vector3Equal(actual.wristPosition,     expected.wristPosition);

    if WILL_PRINT(ok)
    {
        printf("\t -> RESULT: %s \n",(true == ok)?"PASSED":"FAILED");
        printf("EXPECTED: \n");
        printArmPose(expected);
        printf("ACTUAL: \n");
        printArmPose(actual);
    }

    return ok;
}