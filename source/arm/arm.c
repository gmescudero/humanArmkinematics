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
#include "constants.h"
#include "arm.h"
#include "Quaternion.h"
#include "vector3.h"
#include "general.h"
#include "database.h"
#include <string.h>

#define DEFAULT_SHOULDER_2_ELBOW_LENGTH (10.0)
#define DEFAULT_ELBOW_2_WRIST_LENGTH    (5.0)
#define DEFAULT_TOTAL_ARM_LENGTH (DEFAULT_SHOULDER_2_ELBOW_LENGTH+DEFAULT_ELBOW_2_WRIST_LENGTH)

static ERROR_CODE sarm_current_position_update(ARM_POSE pose);

static ARM_FRAME arm_kinematic_table[NUMBER_OF_NODES] = {
    {.position = {0.0, 0.0, 0.0},                              .orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}}}, // Base to Shoulder
    {.position = {-DEFAULT_SHOULDER_2_ELBOW_LENGTH, 0.0, 0.0}, .orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}}}, // Shoulder to elbow
    {.position = {-DEFAULT_ELBOW_2_WRIST_LENGTH   , 0.0, 0.0}, .orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}}}, // Elbow to wrist
};

static ARM_POSE arm_current_pose = {
    .shoulder.position    = {0.0, 0.0, 0.0},
    .shoulder.orientation = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
    .elbow.position       = {0.0, 0.0, -DEFAULT_SHOULDER_2_ELBOW_LENGTH},
    .elbow.orientation    = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
    .wrist.position       = {0.0, 0.0, -DEFAULT_TOTAL_ARM_LENGTH},
    .wrist.orientation    = {.w = M_SQRT1_2, .v = {0.0, -M_SQRT1_2, 0.0}},
};

ERROR_CODE arm_segments_length_set(double upper_arm, double forearm) {
    ERROR_CODE status;
    double sh2el[3];
    double sh2el_scaled[3];
    double current_upper_length;
    double el2wr[3];
    double el2wr_scaled[3];
    double current_forearm_length;

    // Check arguments
    if (EPSI > upper_arm) return RET_ARG_ERROR;
    if (EPSI > forearm) return RET_ARG_ERROR;

    // Rescale shoulder to elbow
    status = vector3_substract(arm_current_pose.elbow.position, arm_current_pose.shoulder.position, sh2el);
    if (RET_OK == status) {
        status = vector3_norm(sh2el, &current_upper_length);
    }
    if (RET_OK == status) {
        status = vector3_scale(sh2el, upper_arm/current_upper_length, sh2el_scaled);
    }
    if (RET_OK == status) {
        status = vector3_add(arm_current_pose.shoulder.position, sh2el_scaled, arm_current_pose.elbow.position);
    }

    // Rescale elbow to wrist
    if (RET_OK == status) {
        status = vector3_substract(arm_current_pose.wrist.position, arm_current_pose.elbow.position, el2wr);
    }
    if (RET_OK == status) {
        status = vector3_norm(el2wr, &current_forearm_length);
    }
    if (RET_OK == status) {
        status = vector3_scale(el2wr, forearm/current_forearm_length, el2wr_scaled);
    }
    if (RET_OK == status) {
        status = vector3_add(arm_current_pose.elbow.position, el2wr_scaled, arm_current_pose.wrist.position);
    }

    // Adapt kinematic table
    if (RET_OK == status) {
        arm_kinematic_table[ELBOW].position[0] = -upper_arm;
        arm_kinematic_table[WRIST].position[0] = -forearm;
    }

    return status;
}

void arm_pose_set(ARM_POSE initial_arm_pose)
{
    memcpy(&arm_current_pose, &initial_arm_pose, sizeof(ARM_POSE));
}

ARM_POSE arm_orientations_set(Quaternion q_arm, Quaternion q_forearm, Quaternion q_wrist)
{
    ERROR_CODE status = RET_OK;
    ARM_POSE pose;
    double v[3];

    // SHOULDER
    if (RET_OK == status) status = vector3_copy(arm_kinematic_table[SHOULDER].position, pose.shoulder.position);
    if (RET_OK == status) Quaternion_copy(&q_arm, &pose.shoulder.orientation);
    // ELBOW
    if (RET_OK == status) Quaternion_rotate(&q_arm, arm_kinematic_table[ELBOW].position, v);
    if (RET_OK == status) status = vector3_add(pose.shoulder.position, v, pose.elbow.position);
    if (RET_OK == status) Quaternion_copy(&q_forearm, &pose.elbow.orientation);
    // WRIST
    if (RET_OK == status) Quaternion_rotate(&q_forearm, arm_kinematic_table[WRIST].position, v);
    if (RET_OK == status) status = vector3_add(pose.elbow.position, v, pose.wrist.position);
    if (RET_OK == status) Quaternion_copy(&q_wrist, &pose.wrist.orientation);

    if (RET_OK == status) {
        arm_pose_set(pose);
        status = sarm_current_position_update(pose);
        if (RET_OK != status) wrn_str("Failed to update new pose into database (Error code: %d)", status);
    }
    else {
        wrn_str("Failed to set new orientation (Error code: %d)", status);
    }
    return arm_current_pose;
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
        "\tsh: <%0.4f, %0.4f, %0.4f>\t| <%0.4f, %0.4f, %0.4f, %0.4f> \n"
        "\tel: <%0.4f, %0.4f, %0.4f>\t| <%0.4f, %0.4f, %0.4f, %0.4f> \n"
        "\twr: <%0.4f, %0.4f, %0.4f>\t| <%0.4f, %0.4f, %0.4f, %0.4f> \n",
        pose.shoulder.position[0],pose.shoulder.position[1],pose.shoulder.position[2],
        pose.shoulder.orientation.w, pose.shoulder.orientation.v[0],pose.shoulder.orientation.v[1],pose.shoulder.orientation.v[2],
        pose.elbow.position[0],pose.elbow.position[1],pose.elbow.position[2],
        pose.elbow.orientation.w, pose.elbow.orientation.v[0],pose.elbow.orientation.v[1],pose.elbow.orientation.v[2],
        pose.wrist.position[0],pose.wrist.position[1],pose.wrist.position[2],
        pose.wrist.orientation.w, pose.wrist.orientation.v[0],pose.wrist.orientation.v[1],pose.wrist.orientation.v[2]);
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
        Quaternion_multiply(&origin.orientation, &transform.orientation, &result.orientation);
    }
    /* Set output parameter */
    if (RET_OK == status) {
        memcpy(output, &result, sizeof(ARM_FRAME));
    }

    return status;
}

ERROR_CODE arm_direct_kinematics_compute(Quaternion joints[ARM_NUMBER_OF_JOINTS], ARM_POSE *output) {
    ERROR_CODE status = RET_OK;
    ARM_FRAME transform;
    ARM_FRAME poses[NUMBER_OF_NODES] = {
        {.position = {0.0, 0.0, 0.0}, .orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
        {.position = {0.0, 0.0, 0.0}, .orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
        {.position = {0.0, 0.0, 0.0}, .orientation = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    };
    ARM_POSE result;
    int i;

    // Check arguments
    if (NULL == joints) return RET_ARG_ERROR;

    for (i = 0; (RET_OK == status) && (i < NUMBER_OF_NODES); i++) {
        // Set transform
        memcpy(&transform, &arm_kinematic_table[i], sizeof(ARM_FRAME));
        if (i < ARM_NUMBER_OF_JOINTS){
            // Add rotational joint value to orientation
            Quaternion_multiply(&(joints[i]),&(transform.orientation), &(transform.orientation));
        }
        // Compute next node position and orientation
        status = arm_homogeneous_transform(poses[MAX(i-1, 0)], transform, &(poses[i]));
    }

    // Set current position
    if (RET_OK == status) {
        Quaternion_copy(&poses[SHOULDER].orientation, &result.shoulder.orientation);
        status = vector3_copy(poses[SHOULDER].position, result.shoulder.position);
    }
    if (RET_OK == status) {
        Quaternion_copy(&poses[ELBOW].orientation, &result.elbow.orientation);
        status = vector3_copy(poses[ELBOW].position, result.elbow.position);
    }
    if (RET_OK == status) {
        Quaternion_copy(&poses[WRIST].orientation, &result.wrist.orientation);
        status = vector3_copy(poses[WRIST].position, result.wrist.position);
    }

    // Update current position
    if (RET_OK == status) {
        status = sarm_current_position_update(result);
    }
    // Set output parameter
    if (RET_OK == status && NULL != output) {
        memcpy(output, &result, sizeof(ARM_POSE));
    }
    return status;
}

ERROR_CODE arm_inverse_kinematics_compute(Quaternion upper_arm, Quaternion forearm, Quaternion joints[ARM_NUMBER_OF_JOINTS]) {
    Quaternion sh_joint_conj;
    
    Quaternion_copy(&upper_arm, &joints[SHOULDER]);
    Quaternion_conjugate(&joints[SHOULDER], &sh_joint_conj);
    Quaternion_multiply(&forearm, &sh_joint_conj, &joints[ELBOW]);

    Quaternion_conjugate(&joints[SHOULDER], &joints[SHOULDER]);
    Quaternion_conjugate(&joints[ELBOW], &joints[ELBOW]);

    dbg_str("%s -> Inverse kinematics resulted in joints: ",__FUNCTION__);
    dbg_str("\t -> Shoulder: %f, %f, %f, %f", joints[SHOULDER].w, joints[SHOULDER].v[0],joints[SHOULDER].v[1],joints[SHOULDER].v[2]);
    dbg_str("\t -> Elbow:    %f, %f, %f, %f", joints[ELBOW].w, joints[ELBOW].v[0],joints[ELBOW].v[1],joints[ELBOW].v[2]);

    return RET_OK;
}

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
    Quaternion_multiply(&sh2el_orientation, &arm_current_pose.shoulder.orientation, &arm_current_pose.shoulder.orientation);

    Quaternion_multiply(&el2wr_orientation, &sh2el_orientation, &aux);
    Quaternion_multiply(&aux, &arm_current_pose.elbow.orientation, &arm_current_pose.elbow.orientation);

    Quaternion_copy(&arm_current_pose.elbow.orientation,&arm_current_pose.wrist.orientation);

    // Compute arm vectors from joints positions positions
    vector3_substract(arm_current_pose.elbow.position, arm_current_pose.shoulder.position, sh2el_vector);
    vector3_substract(arm_current_pose.wrist.position, arm_current_pose.elbow.position, el2wr_vector);

    // Rotate the arm vectors, applying the shoulder rotation to both shoulder and elbow
    Quaternion_rotate(&sh2el_orientation, sh2el_vector, sh2el_vector_rot);
    Quaternion_rotate(&sh2el_orientation, el2wr_vector, el2wr_vector);
    Quaternion_rotate(&el2wr_orientation, el2wr_vector, el2wr_vector_rot);

    // Compute the resulting elbow and wrist position
    vector3_add(arm_current_pose.shoulder.position, sh2el_vector_rot, arm_current_pose.elbow.position);
    vector3_add(arm_current_pose.elbow.position, el2wr_vector_rot, arm_current_pose.wrist.position);


    // Update current position
    sarm_current_position_update(arm_current_pose);

    return arm_current_pose;
}

/**
 * Get the arm pose given the orientation of each segment
 */
ARM_POSE arm_pose_get()
{
    return arm_current_pose;
}

Quaternion arm_quaternion_between_two_get(Quaternion q1, Quaternion q2) {
    Quaternion q1_conj;
    Quaternion q2_to1;

    Quaternion_conjugate(&q1,&q1_conj);
    Quaternion_multiply(&q1_conj,&q2,&q2_to1);

    return q2_to1;
}

ERROR_CODE arm_relative_angular_vel_compute(
    Quaternion q1, Quaternion q2, 
    double angVel1[3], double angVel2[3], double angVelR[3]) 
{
    ERROR_CODE status = RET_OK;
    Quaternion q2_to1;
    double angVel2_from1[3];
    
    /* Compute q21 */
    q2_to1 = arm_quaternion_between_two_get(q1,q2);

    /* Compute relative w */
    Quaternion_rotate(&q2_to1, angVel2, angVel2_from1);
    status = vector3_substract(angVel2_from1, angVel1, angVelR);

    return status;
}


static ERROR_CODE sarm_current_position_update(ARM_POSE pose) {
    ERROR_CODE status = RET_OK;
    double quat_ori[4];

    dbg_str("%s -> Updating arm pose into database",__FUNCTION__);

    // Update database
    if (RET_OK == status) {
        status = db_write(DB_ARM_SHOULDER_POSITION, 0, pose.shoulder.position);
    }
    if (RET_OK == status) {
        quaternion_buffer_build(pose.shoulder.orientation, quat_ori);
        status = db_write(DB_ARM_SHOULDER_ORIENTATION, 0, quat_ori);
    }
    if (RET_OK == status) {
        status = db_write(DB_ARM_ELBOW_POSITION, 0, pose.elbow.position);
    }
    if (RET_OK == status) {
        quaternion_buffer_build(pose.elbow.orientation, quat_ori);
        status = db_write(DB_ARM_ELBOW_ORIENTATION, 0, quat_ori);
    }
    if (RET_OK == status) {
        status = db_write(DB_ARM_WRIST_POSITION, 0, pose.wrist.position);
    }
    if (RET_OK == status) {
        quaternion_buffer_build(pose.wrist.orientation, quat_ori);
        status = db_write(DB_ARM_WRIST_ORIENTATION, 0, quat_ori);
    }
    // Set current arm pose
    if (RET_OK == status) {
        memcpy(&arm_current_pose, &pose, sizeof(ARM_POSE));
    }
    return status;
}

Quaternion zeroAlpha = {.w=1,.v={0,0,0}};
Quaternion zeroGamma = {.w=1,.v={0,0,0}};

ERROR_CODE arm_elbow_angles_zero(
    double alphaR, 
    double gammaR,     
    Quaternion q_sensor1, 
    Quaternion q_sensor2, 
    double rotationV1[3], 
    double rotationV2[3])
{
    ERROR_CODE status = RET_OK;
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER];
    double x_vector[] = {-1.0,0.0,0.0};
    // double y_vector[] = {0.0,1.0,0.0};
    double z_vector[] = {0.0,0.0,1.0};

    // Check arguments
    if (NULL == rotationV1)    return RET_ARG_ERROR;
    if (NULL == rotationV2)    return RET_ARG_ERROR;

    // Reset zeroing quaternions
    Quaternion_set(1,0,0,0,&zeroAlpha);
    Quaternion_set(1,0,0,0,&zeroGamma);

    // Get current angles
    status = arm_elbow_angles_from_rotation_vectors_get(q_sensor1,q_sensor2,rotationV1,rotationV2,anglesFE_B_PS);
    if (RET_OK == status) {
        // Compute zeroing quaternions
        Quaternion_fromAxisAngle(z_vector,anglesFE_B_PS[ALPHA_FE]-alphaR,&zeroAlpha);
        Quaternion_fromAxisAngle(x_vector,gammaR-anglesFE_B_PS[GAMMA_PS],&zeroGamma);

        dbg_str("%s -> Raw angles at zero position: <%f,%f,%f>; zeroAlpha: <%f,%f,%f,%f>; zeroGamma: <%f,%f,%f,%f>",__FUNCTION__,
            anglesFE_B_PS[ALPHA_FE],anglesFE_B_PS[BETA_CARRYING],anglesFE_B_PS[GAMMA_PS],
            zeroAlpha.w,zeroAlpha.v[0],zeroAlpha.v[1],zeroAlpha.v[2],
            zeroGamma.w,zeroGamma.v[0],zeroGamma.v[1],zeroGamma.v[2]);
    }
    /* Compute the new angles after the zero */
    if (RET_OK == status) status = arm_elbow_angles_from_rotation_vectors_get(q_sensor1,q_sensor2,rotationV1,rotationV2,anglesFE_B_PS);

    return status;
}

ERROR_CODE arm_elbow_angles_from_rotation_vectors_get(
    Quaternion q_sensor1, 
    Quaternion q_sensor2, 
    double rotationV1[3], 
    double rotationV2[3],
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER]) 
{
    ERROR_CODE status = RET_OK;

    // Check arguments
    if (NULL == rotationV1)    return RET_ARG_ERROR;
    if (NULL == rotationV2)    return RET_ARG_ERROR;
    if (NULL == anglesFE_B_PS) return RET_ARG_ERROR;

    Quaternion q1bs, q2bs;
    Quaternion q1bs_zeroed, q2bs_zeroed;
    Quaternion q_relative;
    double x_vector[] = {-1.0,0.0,0.0};
    // double y_vector[] = {0.0,1.0,0.0};
    double z_vector[] = {0.0,0.0,1.0};
    
    // Segment to sensor 1 compute  
    if (RET_OK == status) status = quaternion_between_two_vectors_compute(z_vector,rotationV1,&q1bs);

    // Segment to sensor 2 compute 
    if (RET_OK == status) status = quaternion_between_two_vectors_compute(x_vector,rotationV2,&q2bs);

    if (RET_OK == status) {
        // Caluclate zeroed sensor to segment 
        Quaternion_multiply(&q1bs,&zeroAlpha,&q1bs_zeroed);
        Quaternion_multiply(&q2bs,&zeroGamma,&q2bs_zeroed);
        // Calculate the zeroed relative orientation 
        Quaternion q_aux1, q_aux2, q_aux3;
        Quaternion_multiply(&q_sensor1, &q1bs_zeroed, &q_aux1);
        Quaternion_conjugate(&q_aux1, &q_aux2);
        Quaternion_multiply(&q_aux2, &q_sensor2, &q_aux3);
        Quaternion_multiply(&q_aux3, &q2bs_zeroed, &q_relative);
    }

    // Compute Euler angles ZXY to get non zero FE and PS angles
    if (RET_OK == status) {
        Quaternion_toEulerZYX(&q_relative, anglesFE_B_PS);
        dbg_str("%s -> quat <%f,%f,%f,%f>, eulerZXY <%f,%f,%f>",__FUNCTION__,
            q_relative.w,q_relative.v[0],q_relative.v[1],q_relative.v[2],
            anglesFE_B_PS[ALPHA_FE],anglesFE_B_PS[BETA_CARRYING],anglesFE_B_PS[GAMMA_PS]);
    }

    // Update database
    if (RET_OK == status) {
        double q_buff[4] = {q_relative.w,q_relative.v[0],q_relative.v[1],q_relative.v[2]};
        status = db_write(DB_ARM_ELBOW_QUATERNION,0,q_buff);
    }
    if (RET_OK == status) status = db_write(DB_ARM_ELBOW_ANGLES,0,anglesFE_B_PS);
    
    return status;
}

void arm_shoulder_angles_compute(double shoulderAngles[ARM_SHOULDER_ANGLES_NUMBER])
{
    Quaternion_toEulerZYX(&arm_current_pose.shoulder.orientation, shoulderAngles);
    if (M_PI/3 < fabs(shoulderAngles[SH_FLEXION])) {
        // Avoid Euler angles singularity at 90º in Y axis
        double angle = -copysign(M_PI_2,shoulderAngles[SH_FLEXION]);
        Quaternion q_45y = {.w = cos(angle/2), .v = {0.0, sin(angle/2), 0.0}};
        Quaternion q_aux;
        Quaternion_multiply(&q_45y,&arm_current_pose.shoulder.orientation,&q_aux);
        Quaternion_toEulerZYX(&q_aux, shoulderAngles);
        shoulderAngles[SH_FLEXION] += angle;
    }

    if (RET_OK != db_write(DB_ARM_SHOULDER_ANGLES,0,shoulderAngles)) {
        wrn_str("Failed to update database shoulder angles");
    }
}