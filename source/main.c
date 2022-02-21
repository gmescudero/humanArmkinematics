#include <stdio.h>
#include "Quaternion.h"
#include "constants.h"
#include "directKinematics.h"

int main(int argc, char **argv)
{
    Quaternion q1, q2;
    ARM_POSE arm;

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q1);
    // Quaternion_fromXRotation(PI/2, &q1);

    Quaternion_set(1.0, 0.0, 0.0, 0.0, &q2);
    // Quaternion_fromAxisAngle(rotAxis1, PI/2, &q2);

    getArmPositions(q1,q2, &arm);
    printArmPose(arm);
}