
#ifndef __tst_lib_h__
#define __tst_lib_h__

#include "arm.h"
#include "database.h"
#include "matrix.h"
#include "general.h"

#ifndef EPSI
#define EPSI (1e-6)
#endif

#define TST_MAX_CSV_LINES (20000)
#define TST_MAX_CSV_LINE_LENGTH (2048)
#define TST_MAX_CSV_DATA_VALUES (CSV_FILE_VALUES_NUMBER)
#define TST_MAX_CSV_HEADER_LENGTH (CSV_HEADER_MAX_LENGTH)

typedef enum LOG_LEVEL_ENUM {
    ALL_TRACES,
    SILENT_NO_ERROR,
    SILENT
} LOG_LEVEL;

typedef struct TST_CSV_IMU_DATA_STRUCT {
    double timestamp;
    double gyr0[3];
    double gyr1[3];
    double acc0[3];
    double acc1[3];
    double mag0[3];
    double mag1[3];
    double omega0[3];
    double omega1[3];
    double linAcc0[3];
    double linAcc1[3];
    double quat0[4];
    double quat1[4];
} TST_CSV_IMU_DATA;


// * TEST FUNCTIONS ***********************************************************
void testSetTraceLevel(LOG_LEVEL testTraceLvl);
void testDescription(const char *name, const char *description);
void testReport(bool result);
void testBatteryReport(const char *name, const char *description, bool result);
void testCleanUp();
void tst_str(const char *text, ...);
double tstRandomDoubleGenerate();
Quaternion tstRandomQuaternionGenerate();
void tstRandomUnitVector3Generate(double vector[3]);
MATRIX tstRandomMatrixAllocateAndGenerate(unsigned rows, unsigned columns);
void tstVector3RandomNoiseAdd(double vector[3], double scale, double output[3]);
bool tstCsvLoad(const char* csvPath);
int tstCsvColumnsGet();
bool tstCsvRawLineGet(int line, char line_str[TST_MAX_CSV_LINE_LENGTH]);
void tstCsvHeadersGet(char headers[TST_MAX_CSV_DATA_VALUES][TST_MAX_CSV_HEADER_LENGTH]);
bool tstCsvDataLineGet(int line, double data[TST_MAX_CSV_DATA_VALUES]);
// * PRECONDITIONS ************************************************************
bool preconditions_initArm();
bool preconditions_initTraces(const char *test_name);
bool preconditions_init(const char *test_name);
bool preconditions_init_imus(const char *test_name);
bool preconditions_init_databaseCalib(const char *test_name, int imu_data_window, int observations_window);
// * ASSERTS ******************************************************************
bool assert_OK(ERROR_CODE status, const char *description);
bool assert_ERROR(ERROR_CODE status, const char *description);
bool assert_int(int actual, int expected, const char *description);
bool assert_int_greater_or_equal(int actual, int expected, const char *description);
bool assert_double(double actual, double expected, double threshold, const char *description);
bool assert_bool(bool actual, bool expected, const char *description);
bool assert_string(const char *actual, const char *expected, const char *description);
bool assert_string_not_equal(const char *actual, const char *expected, const char *description);
bool assert_string_empty(const char *actual, const char *description);
bool assert_string_not_empty(const char *actual, const char *description);
bool assert_vector3EqualThreshold(const double actual[3], const double expected[3], const double threshold, const char *description);
bool assert_vector3Equal(const double actual[3], const double expected[3], const char *description);
bool assert_vector3EqualNoSignThreshold(const double actual[3], const double expected[3], const double threshold, const char *description);
bool assert_vector3EqualNoSign(const double actual[3], const double expected[3], const char *description);
bool assert_vector3Norm(double actual[3], double expected, const double threshold, const char *description);
bool assert_quaternionThreshold(Quaternion actual, Quaternion expected, double threshold, const char *description);
bool assert_quaternion(Quaternion actual, Quaternion expected, const char *description);
bool assert_frame(ARM_FRAME actual, ARM_FRAME expected, const char *description);
bool assert_armEqualThreshold(const ARM_POSE actual, const ARM_POSE expected, double threshold, const char *description);
bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected, const char *description);
bool assert_dbFieldDoubleThreshold(DB_FIELD_IDENTIFIER field, int instance, const double expected[], double threshold, const char *description);
bool assert_dbFieldDouble(DB_FIELD_IDENTIFIER field, int instance, const double expected[], const char *description);
bool assert_dbFieldInt(DB_FIELD_IDENTIFIER field, int instance, const int expected[], const char *description);
bool assert_dbFieldIntGreaterEqual(DB_FIELD_IDENTIFIER field, int instance, const int expected[], const char *description);
bool assert_dbFieldQuaternionThreshold(DB_FIELD_IDENTIFIER field, int instance, const Quaternion expected, double threshold, const char *description);
bool assert_dbFieldQuaternion(DB_FIELD_IDENTIFIER field, int instance, const Quaternion expected, const char *description);
bool assert_matrix(MATRIX actual, MATRIX expected, const char *description);
bool assert_matrixThreshold(MATRIX actual, MATRIX expected, const double threshold, const char *description);
bool assert_matrixIdentity(MATRIX actual, const char *description);

#endif