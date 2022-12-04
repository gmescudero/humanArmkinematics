#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <signal.h>
#include "tst_lib.h"
#include "arm.h"
#include "vector3.h"
#include "errors.h"
#include "imu.h"
#include "calib.h"
#include "comms.h"

LOG_LEVEL testTraceLevel = SILENT;

#define WILL_PRINT(ok) ((ALL_TRACES == testTraceLevel) || ((SILENT_NO_ERROR == testTraceLevel) && (true != ok)))

// * TEST FUNCTIONS ***********************************************************

void testSetTraceLevel(LOG_LEVEL testTraceLvl){testTraceLevel = testTraceLvl;}

void testDescription(const char *name, const char *description) {
    printf("Test %s: %s \n", name, description);
}

void testReport(bool result) {
    printf("\t -> RESULT: %s \n",(true == result)?"PASSED":"FAILED");
}

void testBatteryReport(const char *name, const char *description, bool result) {
    printf("\n");
    printf("******************************************************************************** \n");
    printf("***** Test battery %s: %s -> RESULT: %s \n", name, description, (true == result)?"PASSED":"FAILED");
    printf("******************************************************************************** \n");
}

void testCleanUp() {
    if (0 != imu_number_get()) imu_all_sensors_remove();
    db_terminate();
    com_terminate();
    preconditions_initArm();
}

void tst_str(const char *text, ...) {
    va_list args;

    if (SILENT != testTraceLevel) {
        va_start(args, text);
        printf("[TEST   ] ");
        vprintf(text, args);
        printf("\n");
        va_end(args);
    }
}

double tstRandomDoubleGenerate() {
    return ((double)rand()/(double)RAND_MAX - 0.5);
}

Quaternion tstRandomQuaternionGenerate() {
    Quaternion gen_quat;
    double w  = tstRandomDoubleGenerate();
    double v1 = tstRandomDoubleGenerate();
    double v2 = tstRandomDoubleGenerate();
    double v3 = tstRandomDoubleGenerate();
    
    Quaternion_set(w, v1, v2, v3, &gen_quat);
    Quaternion_normalize(&gen_quat, &gen_quat);

    return gen_quat;
}

void tstRandomUnitVector3Generate(double vector[3]) {
    vector[0] = tstRandomDoubleGenerate();
    vector[1] = tstRandomDoubleGenerate();
    vector[2] = tstRandomDoubleGenerate();
    vector3_normalize(vector, vector);
}

MATRIX tstRandomMatrixAllocateAndGenerate(unsigned rows, unsigned columns) {
    MATRIX gen = matrix_allocate(rows,columns);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
            gen.data[r][c] = tstRandomDoubleGenerate();
        }
    }
    return gen;
}

void tstVector3RandomNoiseAdd(double vector[3], double scale, double output[3]) {
    output[0] = vector[0] + scale*tstRandomDoubleGenerate();
    output[1] = vector[1] + scale*tstRandomDoubleGenerate();
    output[2] = vector[2] + scale*tstRandomDoubleGenerate();
}

char csv_file_path[256] = {'\0'};
char csv_columns = 0;
char csv_headers[TST_MAX_CSV_DATA_VALUES][TST_MAX_CSV_HEADER_LENGTH] = {{'\0'}};

bool tstCsvLoad(const char* csvPath) {
    bool ok = true;

    char singleChar = '\0';
    char singleHeader[TST_MAX_CSV_HEADER_LENGTH] = {'\0'};
    int lastIndex = 0;
    int headerIndex = 0;
    char line_str[TST_MAX_CSV_LINE_LENGTH];

    strcpy(csv_file_path, csvPath);

    ok &= tstCsvRawLineGet(0, line_str);
    
    for (int i = 0; ok && i < TST_MAX_CSV_LINE_LENGTH; i++) {
        singleChar = line_str[i];
        if ('\n' == singleChar || '\0' == singleChar || ',' == singleChar) {
            singleHeader[i] = '\0';
            strcpy(csv_headers[headerIndex], singleHeader);
            lastIndex = i+1;
            headerIndex++;
            if (',' != singleChar) {
                break;
            }
            else {
                memset(singleHeader,'\0',sizeof(char)*TST_MAX_CSV_HEADER_LENGTH);
            }
        }
        else {
            singleHeader[i-lastIndex] = singleChar;
        }
    }
    csv_columns = headerIndex;
    
    return ok;
}

bool tstCsvRawLineGet(int line, char line_str[TST_MAX_CSV_LINE_LENGTH]) {
    bool ok = true;
    FILE *fd = fopen(csv_file_path,"r");
    if (NULL == fd) {
        tst_str("Failed to read file: %s",csv_file_path);
        ok = false;
    }
    else {
        for (int ln = 0; ok && ln < line+1; ln++) {
            if (NULL == fgets(line_str, TST_MAX_CSV_LINE_LENGTH,fd)) {
                if (ln != line) tst_str("Could not read line %d, file ends in line %d",line,ln-1);
                ok = false;
            }
        }
        fclose(fd);
    }

    return ok;
}

int tstCsvColumnsGet() {
    return csv_columns;
}

void tstCsvHeadersGet(char headers[TST_MAX_CSV_DATA_VALUES][TST_MAX_CSV_HEADER_LENGTH]) {
    memcpy(headers, csv_headers, sizeof(char)*TST_MAX_CSV_DATA_VALUES*TST_MAX_CSV_HEADER_LENGTH);
}

bool tstCsvDataLineGet(int line, double data[TST_MAX_CSV_DATA_VALUES]) {
    bool ok = true;
    char singleChar = '\0';
    char singleDataStr[CSV_HEADER_MAX_LENGTH] = {'\0'};
    int lastIndex = 0;
    int dataIndex = 0;
    char line_str[TST_MAX_CSV_LINE_LENGTH];

    if (0 == line) {
        tst_str("Line 0 is for headers, no data can be found");
        ok = false;
    }
    ok &= tstCsvRawLineGet(line, line_str);

    for (int i = 0; ok && i < TST_MAX_CSV_LINE_LENGTH; i++) {
        singleChar = line_str[i];
        if ('\n' == singleChar || '\0' == singleChar || ',' == singleChar) {
            singleDataStr[i] = '\0';
            data[dataIndex] = atof(singleDataStr);
            lastIndex = i+1;
            dataIndex++;
            if (',' != singleChar) {
                break;
            }
            else {
                memset(singleDataStr,'\0',sizeof(char)*TST_MAX_CSV_HEADER_LENGTH);
            }
        }
        else {
            singleDataStr[i-lastIndex] = singleChar;
        }
    }
    if (ok && dataIndex < csv_columns) {
        ok = false;
        tst_str("Expected to read %d columns but only read %d instead",csv_columns, dataIndex);
    }
    return ok;
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
    arm_pose_set(currentPose);

    return true;
}

bool preconditions_initTraces(const char *test_name) {
    bool ok = true;
    ERROR_CODE ret;
    char log_file_name[LOG_FILE_NAME_LENGTH];
    char csv_file_name[LOG_FILE_NAME_LENGTH];

    ret = trace_level_set(NONE,DEBUG);
    ok &= assert_OK(ret,"trace_level_set");

    sprintf(log_file_name, "log/%s.log",test_name);
    ret = log_file_name_set(log_file_name);
    ok &= assert_OK(ret,"log_file_name_set");

    sprintf(csv_file_name, "data/%s.csv",test_name);
    ret = csv_file_name_set(csv_file_name);
    ok &= assert_OK(ret,"csv_file_name_set");

    ret = log_file_initalize(true,true);
    ok &= assert_OK(ret,"log_file_initalize");

    return ok;
}

static void int_handler(int sig) {
    // tst_str("Testing interrupted by CTRL-C (signal %d)",sig);
    testCleanUp();
    exit(0);
}

bool preconditions_init(const char *test_name)
{
    bool ok = true;
    ERROR_CODE ret;

    struct sigaction act;
    act.sa_handler = int_handler;
    sigaction(SIGINT, &act, NULL);

    ok = preconditions_initTraces(test_name);

    ret = db_initialize();
    ok &= assert_OK(ret,"db_initialize");

    ret = arm_segments_length_set(10.0, 5.0);
    ok &= assert_OK(ret,"arm_segments_length_set");

    return ok;
}

bool preconditions_init_imus(const char *test_name)
{
    bool ok = true;
    ERROR_CODE ret;
    COM_PORTS discoveredPorts;

    ok = preconditions_init(test_name);

    ret = com_ports_list(&discoveredPorts);
    ok &= assert_OK(ret, "com_ports_list");
    ok &= assert_int_greater_or_equal(discoveredPorts.ports_number, 1, "com_ports_list result");
    ok &= assert_string_not_empty(discoveredPorts.ports_names[0], "com_ports_list result");

    ret = imu_batch_initialize(discoveredPorts, discoveredPorts.ports_number);
    ok &= assert_OK(ret, "imu_batch_initialize");

    return ok;
}


bool preconditions_init_databaseCalib(const char *test_name, int imu_data_window, int observations_window) {
    bool ok = true;
    ERROR_CODE ret;

    ok &= preconditions_init(test_name);
    /* Set CSV logging */
    ret = db_csv_field_add(DB_IMU_TIMESTAMP,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_IMU_TIMESTAMP_0");
    ret = db_csv_field_add(DB_CALIB_ITERATIONS,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_ITERATION_0");
    ret = db_csv_field_add(DB_CALIB_OMEGA,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_OMEGA_0");
    ret = db_csv_field_add(DB_CALIB_OMEGA_NORM,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_OMEGA_NORM_0");
    ret = db_csv_field_add(DB_CALIB_ERROR,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_ERROR_0");
    ret = db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_ROT_VECTOR_0");
    ret = db_csv_field_add(DB_CALIB_ROT_VECTOR,1);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_ROT_VECTOR_1");
    ret = db_csv_field_add(DB_CALIB_SPHERICAL_COORDS,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_SPHERICAL_COORDS_0");
    ret = db_csv_field_add(DB_CALIB_SPHERICAL_COORDS,1);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_SPHERICAL_COORDS_1");
    ret = db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_COST_DERIVATIVE_0");
    ret = db_csv_field_add(DB_CALIB_COST_DERIVATIVE,1);
    ok &= assert_OK(ret, "(preconditions_init_databaseCalib) db_csv_field_add DB_CALIB_COST_DERIVATIVE_1");
    /* Initialize calibration package */
    ret = cal_gn2_initialize(imu_data_window, observations_window);
    ok &= assert_OK(ret, "cal_gn2_initialize");

    return ok;
}

// * ASSERTS ******************************************************************

bool assert_OK(ERROR_CODE status, const char *description) {
    bool ok = true;

    if (RET_OK != status) {
        ok = false;
    }
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %d, ACTUAL: %d\n",
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

bool assert_int(int actual, int expected, const char *description) {
    bool ok = true;

    ok = (actual == expected);
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %d, ACTUAL: %d\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, actual);
    }
    return ok;
}

bool assert_int_greater_or_equal(int actual, int expected, const char *description) {
    bool ok = true;

    ok = (actual >= expected);
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: >=%d, ACTUAL: %d\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, actual);
    }
    return ok;
}

bool assert_double(double actual, double expected, double threshold, const char *description) {
    bool ok = true;

    ok = fabs(actual - expected) < threshold;
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %f, ACTUAL: %f\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, actual);
    }
    return ok;
}

bool assert_bool(bool actual, bool expected, const char *description) {
    bool ok = true;

    ok = (actual == expected);
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %s, ACTUAL: %s\n", 
            (true == ok)?"PASSED":"FAILED", description, (expected)?"TRUE":"FALSE", (actual)?"TRUE":"FALSE");
    }
    return ok;
}

bool assert_string(const char *actual, const char *expected, const char *description) {
    bool ok = true;

    ok = (0 == strcmp(actual, expected));
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: \"%s\", ACTUAL: \"%s\"\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, actual);
    }
    return ok;
}

bool assert_string_not_equal(const char *actual, const char *expected, const char *description) {
    bool ok = true;

    ok = (0 != strcmp(actual, expected));
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: not \"%s\", ACTUAL: \"%s\"\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, actual);
    }
    return ok;
}

bool assert_string_empty(const char *actual, const char *description) {
    bool ok = true;

    ok = (0 == strcmp(actual, ""));
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: EMPTY, ACTUAL: %s\n", 
            (true == ok)?"PASSED":"FAILED", description, (true == ok)?"EMPTY":actual);
    }
    return ok;
}

bool assert_string_not_empty(const char *actual, const char *description) {
    bool ok = true;

    ok = (0 != strcmp(actual, ""));
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: NOT EMPTY, ACTUAL: %s\n", 
            (true == ok)?"PASSED":"FAILED", description, (true == ok)?"NOT EMPTY":actual);
    }
    return ok;
}

bool assert_vector3EqualThreshold(const double actual[3], const double expected[3], const double threshold, const char *description) {
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

bool assert_vector3Equal(const double actual[3], const double expected[3], const char *description) {
    return assert_vector3EqualThreshold(actual,expected,EPSI,description);
}

bool assert_vector3EqualNoSignThreshold(const double actual[3], const double expected[3], const double threshold, const char *description) {
    bool ok = true;
    double dotVal = actual[0]*expected[0] + actual[1]*expected[1] + actual[2]*expected[2];

    ok &= assert_double(fabs(dotVal), 1.0, threshold, NULL);
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED(+-): [%f,%f,%f], ACTUAL: [%f,%f,%f] \n",
            (true == ok)?"PASSED":"FAILED", description, expected[0],expected[1],expected[2],actual[0],actual[1],actual[2]);
    }
    return ok;
}

bool assert_vector3EqualNoSign(const double actual[3], const double expected[3], const char *description) {
    return assert_vector3EqualNoSignThreshold(actual,expected,EPSI,description);
}

bool assert_vector3Norm(double actual[3], double expected, const double threshold, const char *description) {
    bool ok = true;
    ERROR_CODE ret;
    double norm;

    ret = vector3_norm(actual, &norm);
    ok &= assert_OK(ret, NULL);
    ok &= assert_double(norm,expected,threshold,NULL);

    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: %f, ACTUAL: %f\n", 
            (true == ok)?"PASSED":"FAILED", description, expected, norm);
    }
    return ok;
}

bool assert_quaternionThreshold(Quaternion actual, Quaternion expected, double threshold, const char *description){
    bool ok = true;

    ok &= assert_double(actual.w, expected.w, threshold, NULL);
    if (ok) {
        ok &= assert_vector3EqualThreshold(actual.v, expected.v, threshold, NULL);
    }
    else {
        ok = assert_double(-actual.w, expected.w, threshold, NULL);
        ok &= assert_vector3EqualNoSignThreshold(actual.v, expected.v, threshold, NULL);
    }

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

bool assert_frame(ARM_FRAME actual, ARM_FRAME expected, const char *description){
    bool ok = true;

    ok &= assert_vector3Equal(actual.position, expected.position, "arm frame -> pos");
    ok &= assert_quaternion(actual.orientation, expected.orientation, "arm frame -> ori");

    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | ",(true == ok)?"PASSED":"FAILED", description);
        printf("EXPECTED: EQUAL, ACTUAL: %s\n",(ok)?"EQUAL":"NOT EQUAL");
    }

    return ok;
}

bool assert_armEqualThreshold(const ARM_POSE actual, const ARM_POSE expected, double threshold, const char *description)
{
    bool ok = true;

    ok &= assert_vector3EqualThreshold(actual.shoulder.position, expected.shoulder.position, threshold, "arm assert -> shoulder pos");
    ok &= assert_vector3EqualThreshold(actual.elbow.position,    expected.elbow.position,    threshold, "arm assert -> elbow pos   ");
    ok &= assert_vector3EqualThreshold(actual.wrist.position,    expected.wrist.position,    threshold, "arm assert -> wrist pos   ");

    ok &= assert_quaternionThreshold(actual.shoulder.orientation, expected.shoulder.orientation, threshold, "arm assert -> shoulder ori");
    ok &= assert_quaternionThreshold(actual.elbow.orientation,    expected.elbow.orientation,    threshold, "arm assert -> elbow ori   ");
    ok &= assert_quaternionThreshold(actual.wrist.orientation,    expected.wrist.orientation,    threshold, "arm assert -> wrist ori   ");

    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | ",(true == ok)?"PASSED":"FAILED", description);
        printf("EXPECTED: EQUAL, ACTUAL: %s\n",(ok)?"EQUAL":"NOT EQUAL");
    }

    return ok;
}

bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected, const char *description)
{
    return assert_armEqualThreshold(actual, expected, EPSI, description);
}

bool assert_dbFieldDoubleThreshold(DB_FIELD_IDENTIFIER field, int instance, const double expected[], double threshold, const char *description) {
    bool ok = true;
    ERROR_CODE ret;
    int size;
    DB_FIELD_TYPE type;
    double buff[100];

    ret = db_field_parameters_get(field, &size, &type, NULL);
    ok &= assert_OK(ret, "db_field_parameters_get");
    if (DB_REAL != type) {
        printf("\t -> RESULT: FAILED (%s) | The requested field does not match type, should be real", description);
        return false;
    }

    ret = db_read(field, instance, buff);
    ok &= assert_OK(ret, "db_read");
    
    for (int i = 0; i < size; i++) {
        ok &= assert_double(buff[i],expected[i],threshold,NULL);
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

bool assert_dbFieldDouble(DB_FIELD_IDENTIFIER field, int instance, const double expected[], const char *description){
    return assert_dbFieldDoubleThreshold(field, instance, expected, EPSI, description);
}

bool assert_dbFieldInt(DB_FIELD_IDENTIFIER field, int instance, const int expected[], const char *description){
    bool ok = true;
    ERROR_CODE ret;
    int size;
    DB_FIELD_TYPE type;
    int buff[100];

    ret = db_field_parameters_get(field, &size, &type, NULL);
    ok &= assert_OK(ret, "db_field_parameters_get");
    if (DB_INTEGER != type) {
        printf("\t -> RESULT: FAILED (%s) | The requested field does not match type, should be integer", description);
        return false;
    }

    ret = db_read(field, instance, buff);
    ok &= assert_OK(ret, "db_read");
    
    for (int i = 0; i < size; i++) {
        ok &= (buff[i] == expected[i]);
    }
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: [", (true == ok)?"PASSED":"FAILED", description);
        for (int i = 0; i < size; i++) {
            printf("%d",expected[i]);
            if (size-1 > i)printf(", ");
        }
        printf("], ACTUAL: [");
        for (int i = 0; i < size; i++) {
            printf("%d",buff[i]);
            if (size-1 > i)printf(", ");
        }
        printf("]\n");
    }

    return ok;
}

bool assert_dbFieldIntGreaterEqual(DB_FIELD_IDENTIFIER field, int instance, const int expected[], const char *description){
    bool ok = true;
    ERROR_CODE ret;
    int size;
    DB_FIELD_TYPE type;
    int buff[100];

    ret = db_field_parameters_get(field, &size, &type, NULL);
    ok &= assert_OK(ret, "db_field_parameters_get");
    if (DB_INTEGER != type) {
        printf("\t -> RESULT: FAILED (%s) | The requested field does not match type, should be integer", description);
        return false;
    }

    ret = db_read(field, instance, buff);
    ok &= assert_OK(ret, "db_read");
    
    for (int i = 0; i < size; i++) {
        ok &= (buff[i] >= expected[i]);
    }
    if (WILL_PRINT(ok) && (NULL != description)) {
        printf("\t -> RESULT: %s (%s) | EXPECTED: >=[", (true == ok)?"PASSED":"FAILED", description);
        for (int i = 0; i < size; i++) {
            printf("%d",expected[i]);
            if (size-1 > i)printf(", ");
        }
        printf("], ACTUAL: [");
        for (int i = 0; i < size; i++) {
            printf("%d",buff[i]);
            if (size-1 > i)printf(", ");
        }
        printf("]\n");
    }

    return ok;
}

bool assert_dbFieldQuaternionThreshold(DB_FIELD_IDENTIFIER field, int instance, const Quaternion expected, double threshold, const char *description){
    bool ok = true;
    ERROR_CODE ret;
    int size;
    DB_FIELD_TYPE type;
    double buff[4];
    Quaternion actual;

    ret = db_field_parameters_get(field, &size, &type, NULL);
    ok &= assert_OK(ret, "db_field_parameters_get");
    if (DB_REAL != type) {
        printf("\t -> RESULT: FAILED (%s) | The requested field does not match type, should be double", description);
        return false;
    }
    if (4 != size) {
        printf("\t -> RESULT: FAILED (%s) | The requested field does is not a quaternion", description);
        return false;
    }

    ret = db_read(field, instance, buff);
    ok &= assert_OK(ret, "db_read");

    quaternion_from_buffer_build(buff,&actual);
    ok &= assert_quaternionThreshold(actual,expected,threshold,description);

    return ok;
}

bool assert_dbFieldQuaternion(DB_FIELD_IDENTIFIER field, int instance, const Quaternion expected, const char *description){
    return assert_dbFieldQuaternionThreshold(field,instance,expected,EPSI,description);
}


bool assert_matrixThreshold(MATRIX actual, MATRIX expected, const double threshold, const char *description) {
    bool ok = true;

    ok &= assert_int(actual.rows, expected.rows, NULL);
    ok &= assert_int(actual.cols, expected.cols, NULL);

    for (int r = 0; ok && r < expected.rows; r ++) {
        for (int c = 0; ok && c < expected.cols; c ++) {
            ok &= assert_double(actual.data[r][c], expected.data[r][c], threshold, NULL);
            if (!ok) tst_str("Messed up in <%d,%d>",r,c);
        }
    }

    if (WILL_PRINT(ok) && (NULL != description)) {        
        printf("\t -> RESULT: %s (%s) | EXPECTED: ", (true == ok)?"PASSED":"FAILED", description);
        for (int r = 0; r < expected.rows; r ++) {
            printf("\n\t\t");
            for (int c = 0; c < expected.cols; c ++) {
                printf("%f\t",expected.data[r][c]);
            }
        }
        printf("\n\n\t\tACTUAL: ");
        for (int r = 0; r < actual.rows; r ++) {
            printf("\n\t\t");
            for (int c = 0; c < actual.cols; c ++) {
                printf("%f\t",actual.data[r][c]);
            }
        }
        printf("\n");
    }
    return ok;
}

bool assert_matrix(MATRIX actual, MATRIX expected, const char *description) {
    return assert_matrixThreshold(actual, expected, EPSI, description);
}

bool assert_matrixIdentity(MATRIX actual, const char *description) {
    bool ok = true;
    MATRIX I;

    ok &= assert_int(actual.rows, actual.cols, NULL);

    I = matrix_identity_allocate(actual.rows);
    ok &= assert_matrix(actual, I, NULL);
    matrix_free(I);

    if (WILL_PRINT(ok) && (NULL != description)) {        
        printf("\t -> RESULT: %s (%s) | EXPECTED: IDENTITY, ACTUAL: ", (true == ok)?"PASSED":"FAILED", description);
        for (int r = 0; r < actual.rows; r ++) {
            printf("\n\t\t");
            for (int c = 0; c < actual.cols; c ++) {
                printf("%f\t",actual.data[r][c]);
            }
        }
        printf("\n");
    }

    return ok;
}