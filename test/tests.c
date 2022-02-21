

#include <stdio.h>
#include "Quaternion.h"
#include "constants.h"
#include "directKinematics.h"

#include "tst_lib.h"

bool tst_001()
{
    bool ok = true;
    Quaternion q1, q2;
    ARM_POSE arm;
    ARM_POSE expected = {
        .shoulderPosition = {0.0, 0.0,  0.0 },
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };

    // Get the Arm Joints positions with identity quaternion rotations
    Quaternion_setIdentity(&q1);
    Quaternion_setIdentity(&q2);
    getArmPositions(q1,q2, &arm);

    ok &= assert_armEqual(arm, expected);

    testReport(__FUNCTION__, ok);
    return ok;
}


int main(int argc, char **argv)
{
    bool ok = true;

    testSetTraceLevel(ALL_TRACES);
    ok &= tst_001();

    
}