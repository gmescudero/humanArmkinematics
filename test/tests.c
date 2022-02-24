

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

bool tst_005()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = { // With the arm extended rotations in Z
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    preconditions_initArm();
    // Rotate joints in Z axis (global frame) to see nothing happens
    Quaternion_fromZRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected);

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromZRotation(PI/2, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

bool tst_006()
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

    preconditions_initArm();
    // Set the elbow at a 90 degree angle and then rotate in Z axis (global frame)
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    Quaternion_fromXRotation(PI/2, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected1);

    Quaternion_fromZRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected2);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

bool tst_007()
{
    bool ok = true;
    ARM_POSE arm;
    Quaternion q1, q2;
    const ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {-10.0, 0.0, 0.0},
        .wristPosition    = {-15.0, 0.0, 0.0},
    };

    preconditions_initArm();
    // Get the Arm Joints positions with identity quaternion rotation for elbow and 90 degrees rotation in Y axis (global frame) for shoulder
    Quaternion_fromYRotation(PI/2, &q1);
    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    arm = rotateArm(q1,q2);
    ok &= assert_armEqual(arm, expected);

    testCleanUp();
    testReport(__FUNCTION__, ok);
    return ok;
}

bool tst_008()
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

    preconditions_initArm();
    // Rotate shoulder for a set of ZYX euler angles (45,90,0)
    Quaternion_fromEulerZYX(eulerAngles, &q1);
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
    ok &= tst_005();
    ok &= tst_006();
    ok &= tst_007();
    ok &= tst_008();
    testReport(__FUNCTION__, ok);

}