
#include "calib.h"
#include "database.h"
#include "vector3.h"
#include "matrix.h"
#include "general.h"
#include "constants.h"
#include "arm.h"
#include "imu.h"

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
static void scal_vector3_to_spherical_coordinates_convert(double vector[3], double *theta, double *rho, int *shperical_convention);
static void scal_spherical_coordinates_to_vector3_convert(double theta, double rho, int shperical_convention, double vector[3]);

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
    Quaternion imus_quat[IMU_MAX_NUMBER],
    int number_of_imus)
{
    Quaternion imu_quat_conj;
    int i;
    
    if (number_of_imus > 0) log_str("Calibrating quaternion data of %d IMU sensors", number_of_imus);

    for (i = 0; i < number_of_imus; i++) {
        // Conjugate the imu reading
        Quaternion_conjugate(&imus_quat[i], &imu_quat_conj);
        // Compute the raw to calibrated quaternion
        Quaternion_multiply(&imu_quat_conj, &(known_quat[i]), &(cal_imus_calibration_data[i].raw_to_calib));
        // Set the imu calibration as "done"
        cal_imus_calibration_data[i].calibration_done = true;
    }
}

ERROR_CODE cal_static_imu_quat_calibration_apply(
    Quaternion imus_quat[IMU_MAX_NUMBER],
    int number_of_imus,
    Quaternion calibrated_data[IMU_MAX_NUMBER])
{
    ERROR_CODE status = RET_OK;
    int i;

    for (i = 0; RET_OK == status && i < number_of_imus; i++) {
        if (false == cal_imus_calibration_data[i].calibration_done) {
            err_str("Imu %d not calibrated!", i);
            status = RET_ERROR;
        }
        else {
            // Apply the calibration
            Quaternion_multiply(&imus_quat[i], &(cal_imus_calibration_data[i].raw_to_calib), &(calibrated_data[i]));
        }
    }

    return status;
}

ERROR_CODE cal_static_imu_quat_calibrated_data_get(Quaternion calib_quat[IMU_MAX_NUMBER]) {
    ERROR_CODE status = RET_OK;
    int number_of_imus = imu_number_get();
    double q_buff[4];
    Quaternion q_tmp;

    for (int i = 0; RET_OK == status && i < number_of_imus; i++) {
        if (false == cal_imus_calibration_data[i].calibration_done) {
            err_str("Imu %d not calibrated!", i);
            status = RET_ERROR;
        }
        else {
            // Retrieve Imu quaternion data
            status = db_read(DB_IMU_QUATERNION, i, q_buff);
            if (RET_OK == status) {
                quaternion_from_buffer_build(q_buff, &q_tmp);
                // Apply the calibration
                Quaternion_multiply(&q_tmp, &(cal_imus_calibration_data[i].raw_to_calib), &calib_quat[i]);
            }
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
            status = vector3_dot(rotationV,omegaR,&alpha);
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
            partRotT[0] =  ct*cr;   partRotT[1] = ct*sr;    partRotT[2] = -st;
            partRotR[0] = -st*sr;   partRotR[1] = st*cr;    partRotT[2] = 0.0;
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


ERROR_CODE cal_automatic_two_rotation_axis_calibrate(
    double omega1_from1[3],
    double omega2_from2[3],
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV1[3],
    double rotationV2[3])
{
    ERROR_CODE status = RET_OK;

    // Check arguments
    if (NULL == omega1_from1)       return RET_ARG_ERROR;
    if (NULL == omega2_from2)       return RET_ARG_ERROR;
    if (NULL == rotationV1)         return RET_ARG_ERROR;
    if (NULL == rotationV2)         return RET_ARG_ERROR;

    // Get the relative agular velocity
    double omegaR[3];
    status = arm_relative_angular_vel_compute(q_sensor1,q_sensor2, omega1_from1, omega2_from2, omegaR);

    // Check the moving status
    double omegaR_norm;
    if (RET_OK == status) {
        status = vector3_norm(omegaR, &omegaR_norm);
    }
    if (RET_OK == status) {
        if (omegaR_norm < cal_rot_axis_autocalib_config.minVel) {
            dbg_str("%s -> Not moving. OmegaR: %f",__FUNCTION__,omegaR_norm);
            return RET_OK;
        }
    }

    // Spherical coordinates
    double theta1, rho1, theta2, rho2;
    int sph_alt1 = 0, sph_alt2 = 0;
    if (RET_OK == status) {
        // First vector
        scal_vector3_to_spherical_coordinates_convert(rotationV1,&theta1,&rho1,&sph_alt1);
        // Second Vector
        scal_vector3_to_spherical_coordinates_convert(rotationV2,&theta2,&rho2,&sph_alt2);
    }

    // Calculate error
    double error;
    static double errorV[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};
    double rotationVn[3]; 
    if (RET_OK == status) {
        status = vector3_cross(rotationV1, rotationV2, rotationVn);
    }
    if (RET_OK == status) {
        status = vector3_dot(omegaR, rotationVn, &error);
    }
    if (RET_OK == status) {
        scal_buffer_shift_and_insert(errorV, error, DEFAULT_ROT_AXIS_CALIB_WINDOW);
    }

    // Calculate partials of each angle
    double dpart_th1[3], dpart_rh1[3], dpart_th2[3], dpart_rh2[3];
    double ct, st, cr, sr;
    if (RET_OK == status) {
        ct = cos(theta1); st = sin(theta1); cr = cos(rho1); sr = sin(rho1);
        if (0 == sph_alt1) {
            dpart_th1[0] =  ct*cr;   dpart_th1[1] = ct*sr;    dpart_th1[2] = -st;
            dpart_rh1[0] = -st*sr;   dpart_rh1[1] = st*cr;    dpart_rh1[2] = 0.0;
        }
        else { // Alternative
            dpart_th1[0] = -st;      dpart_th1[1] = ct*sr;    dpart_th1[2] =  ct*cr;  
            dpart_rh1[0] = 0.0;      dpart_rh1[1] = st*cr;    dpart_rh1[2] = -st*sr;
        }

        ct = cos(theta2); st = sin(theta2); cr = cos(rho2); sr = sin(rho2);
        if (0 == sph_alt2) {
            dpart_th2[0] =  ct*cr;   dpart_th2[1] = ct*sr;    dpart_th2[2] = -st;
            dpart_rh2[0] = -st*sr;   dpart_rh2[1] = st*cr;    dpart_rh2[2] = 0.0;    
        }
        else { // Alternative
            dpart_th2[0] = -st;      dpart_th2[1] = ct*sr;    dpart_th2[2] =  ct*cr;  
            dpart_rh2[0] = 0.0;      dpart_rh2[1] = st*cr;    dpart_rh2[2] = -st*sr;
        }
    }

    // Calculate partials of the error
    double dpart_error[4];
    double aux_vector[3];
    if (RET_OK == status) {
        // d(err)/d(theta1)
        status = vector3_cross(dpart_th1,rotationV2, aux_vector);
        if (RET_OK == status) {
            status = vector3_dot(omegaR, aux_vector, &dpart_error[0]);
        }
    }
    if (RET_OK == status) {
        // d(err)/d(rho1)
        status = vector3_cross(dpart_rh1,rotationV2, aux_vector);
        if (RET_OK == status) {
            status = vector3_dot(omegaR, aux_vector, &dpart_error[1]);
        }
    }
    if (RET_OK == status) {
        // d(err)/d(theta2)
        status = vector3_cross(rotationV1,dpart_th2, aux_vector);
        if (RET_OK == status) {
            status = vector3_dot(omegaR, aux_vector, &dpart_error[2]);
        }
    }
    if (RET_OK == status) {
        // d(err)/d(rho2)
        status = vector3_cross(rotationV1,dpart_rh2, aux_vector);
        if (RET_OK == status) {
            status = vector3_dot(omegaR, aux_vector, &dpart_error[3]);
        }
    }

    // Set up Phi matrix
    MATRIX phi = matrix_allocate(4, 1);
    phi.data[0][0] = theta1;    
    phi.data[1][0] = rho1;      
    phi.data[2][0] = theta2;    
    phi.data[3][0] = rho2;

    // Set up Error matrix
    MATRIX e   = matrix_allocate(DEFAULT_ROT_AXIS_CALIB_WINDOW, 1);
    for (int i=0; i<DEFAULT_ROT_AXIS_CALIB_WINDOW; i++) {
        e.data[i][0]   = errorV[i]; 
    }

    // Calculate and set Jacobian Matrix
    static int it_counter = DEFAULT_ROT_AXIS_CALIB_WINDOW;
    static double Jacobian[DEFAULT_ROT_AXIS_CALIB_WINDOW][4] = {{0.0}};
    MATRIX J = matrix_allocate(DEFAULT_ROT_AXIS_CALIB_WINDOW, 4);

    if (RET_OK == status) {
        for (int i = DEFAULT_ROT_AXIS_CALIB_WINDOW-2; i >= 0; i--) {
            for (int j = 0; j < 4; j++) {
                Jacobian[i+1][j] = Jacobian[i][j];
            }
        }
        for (int c = 0; c < 4; c++) {
            Jacobian[0][c] = dpart_error[c];
            for (int r = 0; r < DEFAULT_ROT_AXIS_CALIB_WINDOW; r++) {
                J.data[r][c]   = Jacobian[r][c];
            }
        }
    }

    if (0 < it_counter) {
        // Allow the Jacobian to reach full range before starting calibration
        it_counter--;
        matrix_free(J);
        matrix_free(e);
        matrix_free(phi);
    }
    else {
        // Update vector of parameters with Gauss-Newton method
        // phi := phi + Jt*(J*Jt)^-1 * e = phi + pinv(J) * e
        MATRIX Jpinv = matrix_allocate(4,DEFAULT_ROT_AXIS_CALIB_WINDOW);
        if (RET_OK == status) {
            status = matrix_pseudoinverse(J,&Jpinv);
        }
        matrix_free(J);
        MATRIX phi_correction = matrix_allocate(4,1);
        if (RET_OK == status) {
            status = matrix_multiply(Jpinv,e, &phi_correction);
        }
        matrix_free(e); matrix_free(Jpinv);
        if (RET_OK == status) {
            status = matrix_scale(phi_correction, cal_rot_axis_autocalib_config.stepSize, &phi_correction);
        }
        if (RET_OK == status) {
            status = matrix_substract(phi, phi_correction, &phi);
        }
        matrix_free(phi_correction); 
        if (RET_OK == status) {
            theta1 = phi.data[0][0];
            rho1   = phi.data[0][1];
            theta2 = phi.data[0][2];
            rho2   = phi.data[0][3];
        }
        matrix_free(phi);
        
        // Set new vectors
        if (RET_OK == status) {
            scal_spherical_coordinates_to_vector3_convert(theta1, rho1, sph_alt1, rotationV1);
            scal_spherical_coordinates_to_vector3_convert(theta2, rho2, sph_alt2, rotationV2);
        }
    }
    // Update database
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ERROR, 0, &error);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 0, rotationV1);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 1, rotationV2);
    }
    if (RET_OK == status) {
        double spherical[] = {theta1,rho1};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 0, spherical);
    }
    if (RET_OK == status) {
        double spherical[] = {theta2,rho2};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 1, spherical);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA, 0, omegaR);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA_NORM, 0, &omegaR_norm);
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

/**
 * @brief Convert a 3D vector to spherical coordinates
 * 
 * @param vector (input) Vector to convert
 * @param theta (output) Theta angle of spherical coordinates
 * @param rho (output) Rho angle of spherical coordinates
 * @param shperical_convention (output) Convention used to get shperical coordinates 
 */
static void scal_vector3_to_spherical_coordinates_convert(double vector[3], double *theta, double *rho, int *shperical_convention) {
    *theta = atan2(sqrt(vector[0]*vector[0]+vector[1]*vector[1]) , vector[2]);
    if (fabs(sin(*theta)) < 0.5) {
        // Avoid singularity
        *shperical_convention = 1;
        *theta = atan2(sqrt(vector[2]*vector[2]+vector[1]*vector[1]) , vector[0]);
        *rho   = atan2(vector[1],vector[2]);
    }
    else {
        *shperical_convention = 0;
        *rho   = atan2(vector[1],vector[0]);
    }
}

/**
 * @brief Convert a set of spherical coordinates to its 3D vector
 * 
 * @param theta (input) Theta angle of spherical coordinates
 * @param rho (input) Rho angle of spherical coordinates
 * @param shperical_convention (input) Convention used to get shperical coordinates 
 * @param vector (output) Converted vector
 */
static void scal_spherical_coordinates_to_vector3_convert(double theta, double rho, int shperical_convention, double vector[3]) {
    double ct = cos(theta);
    double st = sin(theta);
    double cr = cos(rho); 
    double sr = sin(rho);

    if (0 == shperical_convention) {
        vector[0] = st*cr; vector[1] = st*sr; vector[2] = ct;
    }
    else {
        vector[0] = ct; vector[1] = st*sr; vector[2] = st*cr;
    }
}