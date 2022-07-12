
#ifndef __tst_lib_h__
#define __tst_lib_h__

#include "arm.h"
#include "database.h"
#include "matrix.h"

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
void tst_str(const char *text, ...);
double tstRandomDoubleGenerate();
Quaternion tstRandomQuaternionGenerate();
// * PRECONDITIONS ************************************************************
bool preconditions_initArm();
bool preconditions_initTraces(const char *test_name);
bool preconditions_init(const char *test_name);
bool preconditions_init_imus(const char *test_name);
// * ASSERTS ******************************************************************
bool assert_OK(ERROR_CODE status, const char *description);
bool assert_ERROR(ERROR_CODE status, const char *description);
bool assert_int(int actual, int expected, const char *description);
bool assert_int_greater_or_equal(int actual, int expected, const char *description);
bool assert_double(double actual, double expected, double threshold, const char *description);
bool assert_string(const char *actual, const char *expected, const char *description);
bool assert_string_not_equal(const char *actual, const char *expected, const char *description);
bool assert_string_empty(const char *actual, const char *description);
bool assert_string_not_empty(const char *actual, const char *description);
bool assert_vector3EqualThreshold(const double actual[3], const double expected[3], const double threshold, const char *description);
bool assert_vector3Equal(const double actual[3], const double expected[3], const char *description);
bool assert_quaternionThreshold(Quaternion actual, Quaternion expected, double threshold, const char *description);
bool assert_quaternion(Quaternion actual, Quaternion expected, const char *description);
bool assert_frame(ARM_FRAME actual, ARM_FRAME expected, const char *description);
bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected, const char *description);
bool assert_dbFieldDoubleThreshold(DB_FIELD_IDENTIFIER field, int instance, const double expected[], double threshold, const char *description);
bool assert_dbFieldDouble(DB_FIELD_IDENTIFIER field, int instance, const double expected[], const char *description);
bool assert_dbFieldInt(DB_FIELD_IDENTIFIER field, int instance, const int expected[], const char *description);
bool assert_dbFieldIntGreaterEqual(DB_FIELD_IDENTIFIER field, int instance, const int expected[], const char *description);
bool assert_matrix(MATRIX actual, MATRIX expected, const char *description);
bool assert_matrix_identity(MATRIX actual, const char *description);

#endif