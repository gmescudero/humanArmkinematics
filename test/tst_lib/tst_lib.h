#include "arm.h"

#define EPSI (1e-6)

typedef enum LOG_LEVEL_ENUM{
    ALL_TRACES,
    SILENT_NO_ERROR,
    SILENT
} LOG_LEVEL;

// * TEST FUNCTIONS ***********************************************************
void testSetTraceLevel(LOG_LEVEL testTraceLvl);
void testReport(const char *name, bool result);
void testCleanUp();
// * PRECONDITIONS ************************************************************
void preconditions_initArm();
// * ASSERTS ******************************************************************
bool assert_vector3Equal(const double actual[3], const double expected[3]);
bool assert_armEqual(const ARM_POSE actual, const ARM_POSE expected);