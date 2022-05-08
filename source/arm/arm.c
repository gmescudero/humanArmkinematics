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
    .shoulderPosition = {0.0, 0.0, 0.0},
    .elbowPosition    = {0.0, 0.0, -DEFAULT_SHOULDER_2_ELBOW_LENGTH},
    .wristPosition    = {0.0, 0.0, -DEFAULT_TOTAL_ARM_LENGTH},
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
    log_str("Arm pose:");
    log_str("\t sh: \t<%0.4f>\t<%0.4f>\t<%0.4f> ",pose.shoulderPosition[0],pose.shoulderPosition[1],pose.shoulderPosition[2]);
    log_str("\t el: \t<%0.4f>\t<%0.4f>\t<%0.4f> ",pose.elbowPosition[0],pose.elbowPosition[1],pose.elbowPosition[2]);
    log_str("\t wr: \t<%0.4f>\t<%0.4f>\t<%0.4f> ",pose.wristPosition[0],pose.wristPosition[1],pose.wristPosition[2]);
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

    // Compute arm vectors from joints positions positions
    vector3_substract(currentPose.elbowPosition, currentPose.shoulderPosition, sh2el_vector);
    vector3_substract(currentPose.wristPosition, currentPose.elbowPosition, el2wr_vector);

    // Rotate the arm vectors, applying the shoulder rotation to both shoulder and elbow
    Quaternion_rotate(&sh2el_orientation, sh2el_vector, sh2el_vector_rot);
    Quaternion_rotate(&sh2el_orientation, el2wr_vector, el2wr_vector);
    Quaternion_rotate(&el2wr_orientation, el2wr_vector, el2wr_vector_rot);

    // Compute the resulting elbow and wrist position
    vector3_add(currentPose.shoulderPosition, sh2el_vector_rot, currentPose.elbowPosition);
    vector3_add(currentPose.elbowPosition, el2wr_vector_rot, currentPose.wristPosition);

    return currentPose;
}

/**
 * Get the arm pose given the orientation of each segment
 */
ARM_POSE arm_pose_get()
{
    return currentPose;
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

    int sphericAlternative = 0;
    double t,r;
    double tempV[3], tempV2[3];

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

    double error;

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
            t = atan2(sqrt(1-(rotationV[2]*rotationV[2])),rotationV[2]);
            if (t < M_PI/4 || t > 3*M_PI/4) {
                // Near singularity. Apply rotation to avoid it
                sphericAlternative = 1;
                status = vector3_rotate90y(rotationV, tempV);
                if (RET_OK == status) {
                    t = atan2(sqrt(1-tempV[2]*tempV[2]),tempV[2]);
                }
            }
            else {
                memcpy(tempV, rotationV, sizeof(tempV));
            }
            if (RET_OK == status) {
                r = atan2(tempV[1],tempV[0]);
            }
        }

        // Calculate alpha to satisfy given angular speed
        if (RET_OK == status) {
            status = vector3_dot(tempV,tempV,&aux1);
        }
        if (RET_OK == status) {
            status = vector3_dot(tempV,omegaR,&aux2);
        }
        if (RET_OK == status) {
            alpha = aux1*aux2;
        }

        // Calculate the error value
        if (RET_OK == status) {
            err[0] = alpha*tempV[0] - omegaR[0];
            err[1] = alpha*tempV[1] - omegaR[1];
            err[2] = alpha*tempV[2] - omegaR[2];
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

            status = vector3_normalize(tempV, tempV2);
        }
        if (RET_OK == status) {
            if (1 == sphericAlternative) {
                status = vector3_rotateMinus90y(tempV2,rotationV);
            }
            else {
                memcpy(rotationV,tempV2,sizeof(tempV2));
            }
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
            status = db_write(DB_CALIB_SPHERICAL_ALTERNATIVE, &sphericAlternative);
        }
        if (RET_OK == status) {
            double spherical[] = {t,r};
            status = db_write(DB_CALIB_SPHERICAL_COORDS, spherical);
        }
        if (RET_OK == status) {
            double d_cost[]    = {dJk_t_current,dJk_r_current};
            status = db_write(DB_CALIB_COST_DERIVATIVE, d_cost);
        }
    }

    // Update database
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ERROR, &error);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, rotationV);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA, omegaR);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA_NORM, &omegaRnorm);
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
