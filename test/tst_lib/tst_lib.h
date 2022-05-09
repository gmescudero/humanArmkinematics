
#ifndef __tst_lib_h__
#define __tst_lib_h__

#include "arm.h"
#include "database.h"

#ifndef EPSI
#define EPSI (1e-6)
#endif

typedef enum LOG_LEVEL_ENUM{
    ALL_TRACES,
    SILENT_NO_ERROR,
    SILENT
} LOG_LEVEL;

// * TEST FUNCTIONS ***********************************************************
void testSetTraceLevel(LOG_LEVEL testTraceLvl);
void testDescription(const char *name, const char *description);
void testReport(bool result);
void testBatteryReport(const char *name, const char *description, bool result);
void testCleanUp();
// * PRECONDITIONS ************************************************************
bool preconditions_initArm();
bool preconditions_initTraces();
bool preconditions_init();
// * ASSERTS ******************************************************************
bool assert_OK(ERROR_CODE status);
bool assert_ERROR(ERROR_CODE status);
bool assert_vector3Equal(const double actual[3], const double expected[3]);
bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected);
bool assert_dbFieldDouble(DB_FIELD_IDENTIFIER field, double expected[]);


#endif