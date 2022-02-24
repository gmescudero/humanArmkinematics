

#include <stdio.h>
#include "Quaternion.h"
#include "constants.h"
#include "arm.h"

#include "tst_lib.h"

bool tst_001()
{
    bool ok = true;
    ARM_POSE arm;
    const ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    preconditions_initArm();
    // Get the Arm Joints positions
    arm = getArmPositions();
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

bool tst_002()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    preconditions_initArm();
    // Get the Arm Joints positions with identity quaternion rotations
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

bool tst_003()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 5.0, -10.0},
    };

    preconditions_initArm();
    // Get the Arm Joints positions with identity quaternion rotation for shoulder and 90 degrees rotation in X axis (global frame) for elbow
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromXRotation(PI/2, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

bool tst_004()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 10.0, 0.0},
        .wristPosition    = {0.0, 15.0, 0.0},
    };

    preconditions_initArm();
    // Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in X axis (global frame) for shoulder
    Quaternion_fromXRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

int main(int argc, char **argv)
{
    bool ok = true;

    testSetTraceLevel(SILENT_NO_ERROR);
    ok &= tst_001();
    ok &= tst_002();
    ok &= tst_003();
    ok &= tst_004();
}