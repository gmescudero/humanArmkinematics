/**
 * @file arm.c
 * @author German Moreno Escudero
 * @brief Human Arm descriptor package
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "arm.h"
#include "Quaternion.h"
#include "vector3.h"
#include "general.h"
#include "database.h"
#include <string.h>

#define DEFAULT_SHOULDER_2_ELBOW_LENGTH (10.0)
#define DEFAULT_ELBOW_2_WRIST_LENGTH    (5.0)
#define DEFAULT_TOTAL_ARM_LENGTH (DEFAULT_SHOULDER_2_ELBOW_LENGTH+DEFAULT_ELBOW_2_WRIST_LENGTH)

#define DEFAULT_ROT_AXIS_CALIB_WINDOW  (3)
#define DEFAULT_ROT_AXIS_CALIB_STEP_SZ (0.3)
#define DEFAULT_ROT_AXIS_CALIB_MIN_VEL (3e-1)

static void sarm_buffer_shift_and_insert(double array[], double value, int size);

static ARM_POSE currentPose = {
    .shoulder.position = {0.0, 0.0, 0.0},
    .elbow.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    .elbow.position = {0.0, 0.0, -DEFAULT_SHOULDER_2_ELBOW_LENGTH},
    .elbow.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
    .wrist.position = {0.0, 0.0, -DEFAULT_TOTAL_ARM_LENGTH},
    .wrist.orientation = {.w = M_SQRT1_2, .v = {0.0, M_SQRT1_2, 0.0}},
};

static ARM_ROT_AXIS_CALIB_CONFIG calibration_config = { 
    .window   = DEFAULT_ROT_AXIS_CALIB_WINDOW,
    .stepSize = DEFAULT_ROT_AXIS_CALIB_STEP_SZ,
    .minVel   = DEFAULT_ROT_AXIS_CALIB_MIN_VEL
};

void arm_joint_positions_set(ARM_POSE initial_arm_pose)
{
    memcpy(&currentPose, &initial_arm_pose, sizeof(ARM_POSE));
}

/**
 * Print the arm pose
 */
void arm_pose_print(const ARM_POSE pose)
{
    char pose_str[ARM_POSE_STRING_MAX_LENGTH] = {'\0'};
    arm_pose_to_string(pose, pose_str);
    log_str("%s",pose_str);
}

void arm_pose_to_string(const ARM_POSE pose, char pose_str[])
{
    sprintf(pose_str, "Arm pose:\n"
        "\t sh: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n"
        "\t sh: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n"
        "\t sh: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n", 
        pose.shoulder.position[0],pose.shoulder.position[1],pose.shoulder.position[2],
        pose.elbow.position[0],pose.elbow.position[1],pose.elbow.position[2],
        pose.wrist.position[0],pose.wrist.position[1],pose.wrist.position[2]);
}

ERROR_CODE arm_homogeneous_transform(ARM_FRAME origin, ARM_FRAME transform, ARM_FRAME *output) {
    ERROR_CODE status;
    double translation[3];
    ARM_FRAME result;

    /* Get the rotated tcp translation vector */
    Quaternion_rotate(&origin.orientation, transform.position, translation);
    /* Set the position part of the converted point */
    status = vector3_add(origin.position, translation, result.position);
    /* Set the rotation part of the converted point */
    if (RET_OK == status) {
        Quaternion_multiply(&origin.orientation, &transform.orientation, &(output->orientation));
    }
    return status;
}

/**
 * Rotate the arm segments by two given quaternions
 */
ARM_POSE arm_rotate(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation)
{
    double sh2el_vector[3];
    double sh2el_vector_rot[3];
    double el2wr_vector[3];
    double el2wr_vector_rot[3];
    Quaternion aux;

    // Compute orientations
    Quaternion_multiply(&sh2el_orientation, &currentPose.shoulder.orientation, &currentPose.shoulder.orientation);

    Quaternion_multiply(&el2wr_orientation, &sh2el_orientation, &aux);
    Quaternion_multiply(&aux, &currentPose.elbow.orientation, &currentPose.elbow.orientation);

    Quaternion_copy(&currentPose.elbow.orientation,&currentPose.wrist.orientation);

    // Compute arm vectors from joints positions positions
    vector3_substract(currentPose.elbow.position, currentPose.shoulder.position, sh2el_vector);
    vector3_substract(currentPose.wrist.position, currentPose.elbow.position, el2wr_vector);

    // Rotate the arm vectors, applying the shoulder rotation to both shoulder and elbow
    Quaternion_rotate(&sh2el_orientation, sh2el_vector, sh2el_vector_rot);
    Quaternion_rotate(&sh2el_orientation, el2wr_vector, el2wr_vector);
    Quaternion_rotate(&el2wr_orientation, el2wr_vector, el2wr_vector_rot);

    // Compute the resulting elbow and wrist position
    vector3_add(currentPose.shoulder.position, sh2el_vector_rot, currentPose.elbow.position);
    vector3_add(currentPose.elbow.position, el2wr_vector_rot, currentPose.wrist.position);

    return currentPose;
}

/**
 * Get the arm pose given the orientation of each segment
 */
ARM_POSE arm_pose_get()
{
    return currentPose;
}

ERROR_CODE arm_relative_angular_vel_compute(
    Quaternion q1, Quaternion q2, 
    double angVel1[3], double angVel2[3], double angVelR[3]) 
{
    ERROR_CODE status = RET_OK;
    Quaternion q1_conj;
    Quaternion q2_to1;
    double angVel2_from1[3];
    
    /* Compute q21 */
    Quaternion_conjugate(&q1,&q1_conj);
    Quaternion_multiply(&q1_conj,&q2,&q2_to1);

    /* Compute relative w */
    Quaternion_rotate(&q2_to1, angVel2, angVel2_from1);
    status = vector3_substract(angVel2_from1, angVel1, angVelR);

    return status;
}

ERROR_CODE arm_calibrate_rotation_axis(
    double omegaR[3],
    double rotationV[3])
{
    ERROR_CODE status = RET_OK;

    static double dJk_t[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};
    static double dJk_r[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};

    int m           = calibration_config.window;
    double lambda   = calibration_config.stepSize;
    double minVel   = calibration_config.minVel;

    double t,r;
    double tempV[3];

    double alpha;
    double aux1,aux2;

    double omegaRnorm;
    double omegaR2;
    double err[3];

    double ct,cr,st,sr;
    double partRotT[3];
    double partRotR[3];

    double djt, djr;
    double sumDjt = 0.0;
    double sumDjr = 0.0;
    double dJk_t_current,dJk_r_current;

    double newT,newR;

    double error = 0.0;

    // Check arguments
    if (NULL == rotationV) return RET_ARG_ERROR;
    if (NULL == omegaR)    return RET_ARG_ERROR;

    // Check if moving
    status = vector3_norm(omegaR, &omegaRnorm);
    if (RET_OK == status && omegaRnorm < minVel) {
        // No movement
        sarm_buffer_shift_and_insert(dJk_t, 0.0, m);
        sarm_buffer_shift_and_insert(dJk_r, 0.0, m);
    }
    else {
        // Convert to spheric coordinates
        if (RET_OK == status) {
            t = atan2(sqrt(rotationV[0]*rotationV[0]+rotationV[1]*rotationV[1]) , rotationV[2]);
            r = atan2(rotationV[1],rotationV[0]);
        }

        // Calculate alpha to satisfy given angular speed
        if (RET_OK == status) {
            status = vector3_dot(rotationV,rotationV,&aux1);
        }
        if (RET_OK == status) {
            status = vector3_dot(rotationV,omegaR,&aux2);
        }
        if (RET_OK == status) {
            alpha = aux1*aux2;
        }

        // Calculate the error value
        if (RET_OK == status) {
            err[0] = alpha*rotationV[0] - omegaR[0];
            err[1] = alpha*rotationV[1] - omegaR[1];
            err[2] = alpha*rotationV[2] - omegaR[2];
        }

        // Calculate partials of each angle
        if (RET_OK == status) {
            ct = cos(t); st = sin(t); cr = cos(r); sr = sin(r);
            partRotT[0] =  ct*cr; partRotT[1] = ct*sr; partRotT[2] = -st;
            partRotR[0] = -st*sr; partRotR[1] = st*cr; partRotT[2] = 0.0;
        }

        // Calculate the partials of the cost index
        if (RET_OK == status) {
            status = vector3_dot(err,partRotT,&aux1);
        }
        if (RET_OK == status) {
            status = vector3_dot(err,partRotR,&aux2);
        }
        if (RET_OK == status) {
            status = vector3_dot(omegaR,omegaR,&omegaR2);
        }
        if (RET_OK == status) {
            djt = alpha*aux1/omegaR2;
            djr = alpha*aux2/omegaR2;

            sarm_buffer_shift_and_insert(dJk_t, djt, m);
            sarm_buffer_shift_and_insert(dJk_r, djr, m);
        }
        for (int i = 0; (i < m) && (RET_OK == status); i++) {
            sumDjt += dJk_t[i];
            sumDjr += dJk_r[i];
        }
        if (RET_OK == status) {
            dJk_t_current = sumDjt/m;
            dJk_r_current = sumDjr/m;
        }

        // Gradient descent 
        if (RET_OK == status) {
            newT = t - lambda*dJk_t_current;
            newR = r - lambda*dJk_r_current;
        }

        // Set the output vector
        if (RET_OK == status) {
            ct = cos(newT); st = sin(newT); cr = cos(newR); sr = sin(newR);
            tempV[0] = st*cr; tempV[1] = st*sr; tempV[2] = ct;

            status = vector3_normalize(tempV, rotationV);
        }

        // Compute the error
        if (RET_OK == status) {
            status = vector3_dot(err,err, &error);
            if (RET_OK == status) {
                error /= omegaR2;
            }
        }

        // Update database  
        if (RET_OK == status) {
            double spherical[] = {t,r};
            status = db_write(DB_CALIB_SPHERICAL_COORDS, 0, spherical);
        }
        if (RET_OK == status) {
            double d_cost[]    = {dJk_t_current,dJk_r_current};
            status = db_write(DB_CALIB_COST_DERIVATIVE, 0, d_cost);
        }
    }

    // Update database
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ERROR, 0, &error);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 0, rotationV);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA, 0, omegaR);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA_NORM, 0, &omegaRnorm);
    }

    return status;
}

/**
 * @brief Insert a new value in the position 0 of a buffer and shift everything else forward
 * 
 * @param array (input/output) Buffer to be used
 * @param value (input) Value to insert in position 0
 * @param size (input) The size of the buffer
 */
static void sarm_buffer_shift_and_insert(double array[], double value, int size) {
    int i;
    if (2 <= size) {
        for (i = size-2; i >= 0; i--) {
            array[i+1] = array[i];
        }
    }
    array[0] = value;
}
