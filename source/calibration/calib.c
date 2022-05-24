
#include "calib.h"
#include "database.h"
#include "vector3.h"
#include "general.h"
#include "constants.h"
#include "arm.h"

typedef struct CAL_ROT_AXIS_CALIB_CONFIG_STRUCT {
    int    window;
    double stepSize;
    double minVel;
} CAL_ROT_AXIS_CALIB_CONFIG;

typedef struct CAL_STATIC_CALIBRATION_STRUCT {
    bool        calibration_done;
    Quaternion  raw_to_calib;
} CAL_STATIC_CALIBRATION;

static void scal_buffer_shift_and_insert(double array[], double value, int size);


static CAL_STATIC_CALIBRATION cal_imus_calibration_data[IMU_MAX_NUMBER] = {
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
};

static CAL_ROT_AXIS_CALIB_CONFIG cal_rot_axis_autocalib_config = { 
    .window   = DEFAULT_ROT_AXIS_CALIB_WINDOW,
    .stepSize = DEFAULT_ROT_AXIS_CALIB_STEP_SZ,
    .minVel   = DEFAULT_ROT_AXIS_CALIB_MIN_VEL
};

void cal_static_imu_quat_calibration_set(
    Quaternion known_quat[IMU_MAX_NUMBER],
    ImuData imus_data[IMU_MAX_NUMBER],
    int number_of_imus)
{
    Quaternion imu_quat;
    Quaternion imu_quat_conj;
    int i;
    
    if (number_of_imus > 0) log_str("Calibrating quaternion data of %d IMU sensors", number_of_imus);

    for (i = 0; i < number_of_imus; i++) {
        // Set the quaternion struct from the IMU data
        quaternion_from_float_buffer_build(imus_data[i].q, &imu_quat);
        // Conjugate the imu reading
        Quaternion_conjugate(&imu_quat, &imu_quat_conj);
        // Compute the raw to calibrated quaternion
        Quaternion_multiply(&imu_quat_conj, &(known_quat[i]), &(cal_imus_calibration_data[i].raw_to_calib));
        // Set the imu calibration as "done"
        cal_imus_calibration_data[i].calibration_done = true;
    }
}

ERROR_CODE cal_static_imu_quat_calibration_apply(
    ImuData imus_data[IMU_MAX_NUMBER],
    int number_of_imus,
    Quaternion calibrated_data[IMU_MAX_NUMBER])
{
    ERROR_CODE status = RET_OK;
    Quaternion imu_quat;
    int i;

    for (i = 0; RET_OK == status && i < number_of_imus; i++) {
        if (false == cal_imus_calibration_data[i].calibration_done) {
            err_str("Imu %d not calibrated!", i);
            status = RET_ERROR;
        }
        else {
            // Set the quaternion struct from the IMU data  
            quaternion_from_float_buffer_build(imus_data[i].q, &imu_quat);
            // Apply the calibration
            Quaternion_multiply(&(cal_imus_calibration_data[i].raw_to_calib), &imu_quat, &(calibrated_data[i]));
        }
    }

    return status;
}


ERROR_CODE cal_automatic_rotation_axis_calibrate(
    double omegaR[3],
    double rotationV[3])
{
    ERROR_CODE status = RET_OK;

    static double dJk_t[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};
    static double dJk_r[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};

    int m           = cal_rot_axis_autocalib_config.window;
    double lambda   = cal_rot_axis_autocalib_config.stepSize;
    double minVel   = cal_rot_axis_autocalib_config.minVel;

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
        scal_buffer_shift_and_insert(dJk_t, 0.0, m);
        scal_buffer_shift_and_insert(dJk_r, 0.0, m);
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

            scal_buffer_shift_and_insert(dJk_t, djt, m);
            scal_buffer_shift_and_insert(dJk_r, djr, m);
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
static void scal_buffer_shift_and_insert(double array[], double value, int size) {
    int i;
    if (2 <= size) {
        for (i = size-2; i >= 0; i--) {
            array[i+1] = array[i];
        }
    }
    array[0] = value;
}