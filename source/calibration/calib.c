
#include "calib.h"
#include "database.h"
#include "vector3.h"
#include "matrix.h"
#include "general.h"
#include "constants.h"
#include "arm.h"
#include "imu.h"
#include "ga.h"

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
static ERROR_CODE scal_observations_update();

static CAL_STATIC_CALIBRATION cal_imus_calibration_data[IMU_MAX_NUMBER] = {
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    // {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    // {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
    // {.calibration_done = false, .raw_to_calib = {.w = 1.0, .v = {0.0, 0.0, 0.0}}},
};

static CAL_ROT_AXIS_CALIB_CONFIG cal_rot_axis_autocalib_config = { 
    .window   = DEFAULT_ROT_AXIS_CALIB_WINDOW,
    .stepSize = DEFAULT_ROT_AXIS_CALIB_STEP_SZ,
    .minVel   = CALIB_MIN_VEL
};

static double scal_min_velocity_norm = CALIB_MIN_VEL;

void cal_min_velocity_set(double min_vel) {
    log_str("Minimum velocity to consider movement set to %f",min_vel);
    scal_min_velocity_norm = min_vel;
}

void cal_static_imu_quat_calibration_set(
    Quaternion known_quat[IMU_MAX_NUMBER],
    Quaternion imus_quat[IMU_MAX_NUMBER])
{
    Quaternion imu_quat_conj;
    int number_of_imus = imu_number_get();
    
    if (number_of_imus > 0) log_str("Calibrating quaternion data of %d IMU sensors", number_of_imus);

    for (int i = 0; i < number_of_imus; i++) {
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
    Quaternion calibrated_data[IMU_MAX_NUMBER])
{
    ERROR_CODE status = RET_OK;
    int number_of_imus = imu_number_get();

    for (int i = 0; RET_OK == status && i < number_of_imus; i++) {
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
    double omega1_from1[3],
    double omega2_from2[3],
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV[3])
{
    ERROR_CODE status = RET_OK;

    static double dJk_t[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};
    static double dJk_r[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};

    int m           = cal_rot_axis_autocalib_config.window;
    double lambda   = cal_rot_axis_autocalib_config.stepSize;
    double minVel   = cal_rot_axis_autocalib_config.minVel;

    int sph_alt = 0;
    double t,r;

    double alpha;
    double aux1,aux2;

    double omegaRnorm;
    double omegaR2;
    double err[3];

    double partRotT[3];
    double partRotR[3];

    double djt, djr;
    double sumDjt = 0.0;
    double sumDjr = 0.0;
    double dJk_t_current,dJk_r_current;

    double newT,newR;

    double error = 0.0;

    // Check arguments
    if (NULL == rotationV)      return RET_ARG_ERROR;
    if (NULL == omega1_from1)   return RET_ARG_ERROR;
    if (NULL == omega2_from2)   return RET_ARG_ERROR;

    // Get the relative agular velocity
    double omegaR[3];
    status = arm_relative_angular_vel_compute(q_sensor1, q_sensor2, omega1_from1, omega2_from2, omegaR);

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
            vector3_to_spherical_coordinates_convert(rotationV,&t,&r,&sph_alt);
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
            vector3_spherical_coordinates_derivatives_compute(t, r, sph_alt, partRotT, partRotR);
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
            vector3_from_spherical_coordinates_convert(newT, newR, sph_alt, rotationV);
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

#if 0  // Old implementation 
ERROR_CODE cal_automatic_two_rotation_axes_calibrate_old(
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
    status = arm_relative_angular_vel_compute(q_sensor1, q_sensor2, omega1_from1, omega2_from2, omegaR);

    // Check the moving status
    double omegaR_norm;
    if (RET_OK == status) {
        status = vector3_norm(omegaR, &omegaR_norm);
    }
    if (RET_OK == status && omegaR_norm < cal_rot_axis_autocalib_config.minVel) {
        dbg_str("%s -> Not moving. OmegaR: %f",__FUNCTION__,omegaR_norm);
        return RET_OK;
    }

    // Calculate error
    double error;
    static double errorV[DEFAULT_ROT_AXIS_CALIB_WINDOW] = {0.0};
    double rotationV2_from1[3];
    double rotationVn[3]; 
    Quaternion q2_1 = arm_quaternion_between_two_get(q_sensor1,q_sensor2);

    Quaternion_rotate(& q2_1, rotationV2, rotationV2_from1);
    if (RET_OK == status) {
        status = vector3_cross(rotationV1, rotationV2_from1, rotationVn);
    }
    if (RET_OK == status) {
        status = vector3_dot(omegaR, rotationVn, &error);
    }
    if (RET_OK == status) {
        scal_buffer_shift_and_insert(errorV, error, DEFAULT_ROT_AXIS_CALIB_WINDOW);
    }
    if (RET_OK == status && 5e-5 > fabs(error)) {
        dbg_str("%s -> Error below threshold: %f",__FUNCTION__,error);
        return RET_OK;
    }

    // Spherical coordinates
    double theta1, rho1, theta2, rho2;
    int sph_alt1 = 0, sph_alt2 = 0;
    if (RET_OK == status) {
        vector3_to_spherical_coordinates_convert(rotationV1, &theta1, &rho1, &sph_alt1);
        vector3_to_spherical_coordinates_convert(rotationV2_from1, &theta2, &rho2, &sph_alt2);
    }

    // Calculate partials of each angle
    double dpart_th1[3], dpart_rh1[3], dpart_th2[3], dpart_rh2[3];
    if (RET_OK == status) {
        vector3_spherical_coordinates_derivatives_compute(theta1, rho1, sph_alt1, dpart_th1, dpart_rh1);
        vector3_spherical_coordinates_derivatives_compute(theta2, rho2, sph_alt2, dpart_th2, dpart_rh2);
    }

    // Calculate partials of the error
    double dpart_error[4];
    double aux_vector[3];
    if (RET_OK == status) {
        // d(err)/d(theta1)
        status = vector3_cross(dpart_th1,rotationV2_from1, aux_vector);
        if (RET_OK == status) {
            status = vector3_dot(omegaR, aux_vector, &dpart_error[0]);
        }
    }
    if (RET_OK == status) {
        // d(err)/d(rho1)
        status = vector3_cross(dpart_rh1,rotationV2_from1, aux_vector);
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
                J.data[r][c] = Jacobian[r][c];
            }
        }
    }

    if (0 < it_counter) {
        // Allow the Jacobian to reach full range before starting calibration
        it_counter--;
    }
    else {
        // Update vector of parameters with Gauss-Newton method
        // phi := phi + Jt*(J*Jt)^-1 * e = phi + pinv(J) * e
        MATRIX Jpinv = matrix_allocate(4,DEFAULT_ROT_AXIS_CALIB_WINDOW);
        if (RET_OK == status) {
            status = matrix_pseudoinverse(J,&Jpinv);
        }
        MATRIX phi_correction = matrix_allocate(4,1);
        if (RET_OK == status) {
            status = matrix_multiply(Jpinv,e, &phi_correction);
        }
        if (RET_OK == status) {
            status = matrix_scale(phi_correction, cal_rot_axis_autocalib_config.stepSize, &phi_correction);
        }
        if (RET_OK == status) {
            status = matrix_substract(phi, phi_correction, &phi);
        }
        if (RET_OK == status) {
            theta1 = phi.data[0][0];
            rho1   = phi.data[1][0];
            theta2 = phi.data[2][0];
            rho2   = phi.data[3][0];
        }
        matrix_free(Jpinv);
        matrix_free(phi_correction); 
        
        // Set new vectors
        if (RET_OK == status) {
            Quaternion q1_2;
            vector3_from_spherical_coordinates_convert(theta1, rho1, sph_alt1, rotationV1);
            vector3_from_spherical_coordinates_convert(theta2, rho2, sph_alt2, rotationV2_from1);
            Quaternion_conjugate(&q2_1, &q1_2);
            Quaternion_rotate(&q1_2, rotationV2_from1, rotationV2);
        }
    }
    matrix_free(J);
    matrix_free(e); 
    matrix_free(phi);

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
    if (RET_OK == status) {
        double d_cost[]    = {Jacobian[0][0],Jacobian[0][1]};
        status = db_write(DB_CALIB_COST_DERIVATIVE, 0, d_cost);
    }
    if (RET_OK == status) {
        double d_cost[]    = {Jacobian[0][2],Jacobian[0][3]};
        status = db_write(DB_CALIB_COST_DERIVATIVE, 1, d_cost);
    }
    return status;
}
#endif // Old implementation
#if 0
ERROR_CODE cal_automatic_rotation_axis_calibrate_new(
    double omega1_from1[3],
    double omega2_from2[3],
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV[3])
{
    ERROR_CODE status = RET_OK;

    // Check arguments
    if (NULL == omega1_from1)       return RET_ARG_ERROR;
    if (NULL == omega2_from2)       return RET_ARG_ERROR;
    if (NULL == rotationV)          return RET_ARG_ERROR;

    // Get the relative agular velocity and quaternion
    double omegaR[3];
    status = arm_relative_angular_vel_compute(q_sensor1, q_sensor2, omega1_from1, omega2_from2, omegaR);

    // Shift observation buffers and add new measures
    static double w_observations[CALIB_ONE_ROT_AXIS_WINDOW][3] = {{0.0}};
    for (int i = CALIB_ONE_ROT_AXIS_WINDOW-2; RET_OK == status && i >= 0; i--) {
        w_observations[i+1][0] = w_observations[i][0];
        w_observations[i+1][1] = w_observations[i][1];
        w_observations[i+1][2] = w_observations[i][2];
    }
    if (RET_OK == status) {
        w_observations[0][0] = omegaR[0];
        w_observations[0][1] = omegaR[1];
        w_observations[0][2] = omegaR[2];
    }

    // Check the moving status
    double omegaR_norm;
    if (RET_OK == status) {
        status = vector3_norm(omegaR, &omegaR_norm);
    }
    if (RET_OK == status && omegaR_norm < scal_min_velocity_norm) {
        dbg_str("%s -> Not moving. OmegaR: %f",__FUNCTION__,omegaR_norm);
        return RET_OK;
    }

    // Check iterations
    static int observations_num = 0;
    if (RET_OK == status && observations_num < CALIB_ONE_ROT_AXIS_WINDOW) {
        observations_num++;
        dbg_str("%s -> Adjusting calibration window. %d/%d observations",
            __FUNCTION__,observations_num, CALIB_ONE_ROT_AXIS_WINDOW);
    }

    // GAUSS-NEWTON
    double error = 1e9;
    int iterations = CALIB_ONE_ROT_AXIS_MAX_ITERATIONS;

    MATRIX errorV   = matrix_allocate(observations_num,1);
    MATRIX Jacobian = matrix_allocate(observations_num,2);
    MATRIX phi      = matrix_allocate(2, 1); // theta, rho

    while (RET_OK == status && CALIB_ONE_ROT_AXIS_MAX_ERROR < error && 0 < iterations) {
        iterations--;

        // Spherical coordinates
        int sph_alt = 0;
        if (RET_OK == status) {
            vector3_to_spherical_coordinates_convert(rotationV, &phi.data[0][0], &phi.data[1][0], &sph_alt);
        }

        // Calculate partials of each angle
        double dpart_th[3], dpart_rh[3];
        if (RET_OK == status) {
            vector3_spherical_coordinates_derivatives_compute(phi.data[0][0], phi.data[1][0], sph_alt, dpart_th, dpart_rh);
        }

        // Calculate error and Jacobian
        double squared_error = 0.0;
        for (int i = 0; RET_OK == status && i < observations_num; i++) {
            double     wi[3] = {w_observations[i][0],w_observations[i][1],w_observations[i][2]};
            // dbg_str("%s -> w%d: [%f, %f, %f]",__FUNCTION__,i, wi[0],wi[1],wi[2]);
            // Compute error
            double aux[3]; 
            if (RET_OK == status) {
                status = vector3_norm(wi, &omegaR_norm);
            }
            if (RET_OK == status) {
                status = vector3_scale(rotationV, omegaR_norm, aux);
            }
            if (RET_OK == status) {
                status = vector3_substract(wi, aux, aux);
            }
            if (RET_OK == status) {
                status = vector3_norm(aux, &error);
            }
            if (RET_OK == status) {
                errorV.data[i][0] = error;
                squared_error += error*error;
            }

            // Calculate partials of the error
            double dpart_error[2];
            double aux_vector[3];
            if (RET_OK == status) { // d(err)/d(theta)
                status = vector3_scale(dpart_th, omegaR_norm, aux_vector);
                if (RET_OK == status) {
                    status = vector3_norm(dpart_th, &dpart_error[0]);
                }
            }
            if (RET_OK == status) { // d(err)/d(rho)
                status = vector3_scale(dpart_rh, omegaR_norm, aux_vector);
                if (RET_OK == status) {
                    status = vector3_norm(dpart_th, &dpart_error[1]);
                }
            }

            // Build Jacobian matrix row
            for (int c = 0; RET_OK == status && c < 2; c++) {
                Jacobian.data[i][c] = dpart_error[c];
            }
        }
        error = squared_error/observations_num;
        dbg_str("%s -> Current calib error: %f",__FUNCTION__, error);
        if (CALIB_ONE_ROT_AXIS_MAX_ERROR < error) {
            MATRIX Jpinv            = matrix_allocate(2,observations_num);
            MATRIX phi_correction   = matrix_allocate(2,1);
            if (RET_OK == status) {
                status = matrix_pseudoinverse(Jacobian,&Jpinv);
            }
            if (RET_OK == status) {
                status = matrix_multiply(Jpinv, errorV, &phi_correction);
            }
            if (RET_OK == status) {
                status = matrix_scale(phi_correction, CALIB_ONE_ROT_AXIS_STEP_SZ, &phi_correction);
            }
            if (RET_OK == status) {
                status = matrix_add(phi, phi_correction, &phi);
            }
            matrix_print(Jacobian, "Jacobian");
            matrix_print(errorV, "errorV");
            matrix_print(phi, "phi");
            matrix_print(Jpinv, "Jpinv");
            matrix_print(phi_correction, "phi_correction");
            matrix_free(Jpinv);
            matrix_free(phi_correction); 
            // Set new vector
            if (RET_OK == status) {
                vector3_from_spherical_coordinates_convert(phi.data[0][0], phi.data[1][0], sph_alt, rotationV);
            }
        }
    }

    if (RET_OK == status && (0 >= iterations || CALIB_ONE_ROT_AXIS_MAX_ERROR < error)) {
        wrn_str("Could not achieve target error. Achieved error %f > Target error %f",error, CALIB_ONE_ROT_AXIS_MAX_ERROR);
    }

    // Update database
    if (RET_OK == status) {
        double spherical[] = {phi.data[0][0],phi.data[1][0]};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 0, spherical);
    }
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
        status = db_write(DB_CALIB_OMEGA_NORM, 0, &omegaR_norm);
    }

    matrix_free(errorV);
    matrix_free(Jacobian);
    matrix_free(phi);

    return status;
}
#endif // Alternative

ERROR_CODE cal_automatic_two_rotation_axes_calibrate(
    double omega1_from1[3],
    double omega2_from2[3],
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV1[3],
    double rotationV2[3])
{
    ERROR_CODE status = RET_OK; // Error code

    // Check arguments
    if (NULL == omega1_from1)       return RET_ARG_ERROR;
    if (NULL == omega2_from2)       return RET_ARG_ERROR;
    if (NULL == rotationV1)         return RET_ARG_ERROR;
    if (NULL == rotationV2)         return RET_ARG_ERROR;

    // Get the relative agular velocity and quaternion
    Quaternion q2_1 = arm_quaternion_between_two_get(q_sensor1, q_sensor2); // Quaternion to move from sensor 2 to 1
    Quaternion q1_2;                                                        // Quaternion to move from sensor 1 to 2
    double omegaR[3];                                                       // Relative angular velocity
    Quaternion_conjugate(&q2_1,&q1_2);                                      
    status = arm_relative_angular_vel_compute(q_sensor1, q_sensor2, omega1_from1, omega2_from2, omegaR);

    // Update observation vector
    if (RET_OK == status) {
        status = scal_observations_update();
    }
    
    // GAUSS-NEWTON
    int iterations = CALIB_TWO_ROT_AXES_MAX_ITERATIONS;                                         // Iterations of the algorithm
    int observations_num = db_field_buffer_current_size_get(DB_CALIB_TWO_AXES_OBSERVATIONS,0);  // Number of managed observations

    MATRIX errorV   = matrix_allocate(observations_num,1);  // Vector of residuals
    MATRIX Jacobian = matrix_allocate(observations_num,4);  // Jacobian matrix
    MATRIX phi      = matrix_allocate(4, 1);                // Parameters vector: theta1, rho1, theta2, rho2

    double error = 1e9;                                             // Last iteration error value
    double bestError = error;                                       // Best achieved error value
    double tempV1[] = {rotationV1[0],rotationV1[1],rotationV1[2]};  // Vector 1 to be tweaked by the algorithm
    double tempV2[] = {rotationV2[0],rotationV2[1],rotationV2[2]};  // Vector 2 to be tweaked by the algorithm
    int sph_alt1, sph_alt2;                                         // Spherical coordinate convention in use

    while (RET_OK == status && CALIB_TWO_ROT_AXES_MAX_ERROR < error && 0 < iterations) {
        iterations--;

        // Calculate error and Jacobian
        double squared_error = 0.0;                                     // Squared error value
        double dpart_th1[3], dpart_rh1[3], dpart_th2[3], dpart_rh2[3];  // Derivatives of each vector wrt each spherical coordinate
        double tempV2_from1[3];                                         // Second rotation vector from the first sensor

        for (int i = 0; RET_OK == status && i < observations_num; i++) {
            double     observation[7];
            status = db_field_buffer_from_tail_data_get(DB_CALIB_TWO_AXES_OBSERVATIONS,0, i, observation);
            double     wi[3] = {            // The ith observation of angular velocity
                observation[0],observation[1],observation[2]};
            Quaternion qi    = {            // The ith observation of quaternion
                .w=observation[3], .v={observation[4],observation[5],observation[6]}}; 
        
            // dbg_str("%s -> w%d: [%f, %f, %f]",__FUNCTION__,i, wi[0],wi[1],wi[2]);
            // dbg_str("%s -> q%d: [%f, %f, %f, %f]",__FUNCTION__,i, qi.w,qi.v[0],qi.v[1],qi.v[2]);
            // Compute rotation vector 2 from 1
            if (RET_OK == status) {
                Quaternion_rotate(&qi, tempV2, tempV2_from1);
            }
            // Compute phi values
            if (RET_OK == status) {
                vector3_to_spherical_coordinates_convert(tempV1,       &phi.data[0][0], &phi.data[1][0], &sph_alt1);
                vector3_to_spherical_coordinates_convert(tempV2_from1, &phi.data[2][0], &phi.data[3][0], &sph_alt2);
            }
            // Compute derivatives of each phi parameter
            if (RET_OK == status) {
                vector3_spherical_coordinates_derivatives_compute(phi.data[0][0], phi.data[1][0], sph_alt1, dpart_th1, dpart_rh1);
                vector3_spherical_coordinates_derivatives_compute(phi.data[2][0], phi.data[3][0], sph_alt2, dpart_th2, dpart_rh2);
            }
            // Compute error
            double rotationVn[3];   // Normal vector to both rotation vectors
            if (RET_OK == status) {
                status = vector3_cross(tempV1, tempV2_from1, rotationVn);
            }
            if (RET_OK == status) {
                status = vector3_dot(wi, rotationVn, &error);
            }
            if (RET_OK == status) {
                errorV.data[i][0] = error;
                squared_error += error*error;
            }

            // Build Jacobian matrix row calculating each partial of the error
            double aux_vector[3];   // Auxiliary 3D vector to be used by operations
            if (RET_OK == status) { // d(err)/d(theta1)
                status = vector3_cross(dpart_th1,tempV2_from1, aux_vector);
                if (RET_OK == status) {
                    status = vector3_dot(wi, aux_vector, &Jacobian.data[i][0]);
                }
            }
            if (RET_OK == status) { // d(err)/d(rho1)
                status = vector3_cross(dpart_rh1,tempV2_from1, aux_vector);
                if (RET_OK == status) {
                    status = vector3_dot(wi, aux_vector, &Jacobian.data[i][1]);
                }
            }
            if (RET_OK == status) { // d(err)/d(theta2)
                status = vector3_cross(tempV1,dpart_th2, aux_vector);
                if (RET_OK == status) {
                    status = vector3_dot(wi, aux_vector, &Jacobian.data[i][2]);
                }
            }
            if (RET_OK == status) { // d(err)/d(rho2)
                status = vector3_cross(tempV1,dpart_rh2, aux_vector);
                if (RET_OK == status) {
                    status = vector3_dot(wi, aux_vector, &Jacobian.data[i][3]);
                }
            }
        }

        // Compute total squared error value
        if (RET_OK == status) {
            error = squared_error/observations_num;
        }
        // Compute rotation vector 2 from 1
        if (RET_OK == status) {
            Quaternion_rotate(&q2_1, tempV2, tempV2_from1);
        }
        // Compute phi values
        if (RET_OK == status) {
            vector3_to_spherical_coordinates_convert(tempV1,       &phi.data[0][0], &phi.data[1][0], &sph_alt1);
            vector3_to_spherical_coordinates_convert(tempV2_from1, &phi.data[2][0], &phi.data[3][0], &sph_alt2);
        }
        // Use only the best set of rotation axes
        if (bestError > error) {
            // dbg_str("%s -> Updated best achieved rotation vectors error: %f -> %f",__FUNCTION__, bestError, error);
            if (RET_OK == status) {
                bestError = error;
            }
            if (RET_OK == status) {
                status = vector3_copy(tempV1,rotationV1);
            }
            if (RET_OK == status) {
                status = vector3_copy(tempV2,rotationV2);
            }
        }
        dbg_str("%s -> [it: %d] Current calib error: %f (best error: %f)",__FUNCTION__, 
            CALIB_TWO_ROT_AXES_MAX_ITERATIONS-iterations, error, bestError);
        // dbg_str("%s -> v1 <%f,%f,%f> v2 <%f,%f,%f> phi <%f,%f,%f,%f>",__FUNCTION__,
        //     tempV1[0],tempV1[1],tempV1[2],tempV2[0],tempV2[1],tempV2[2],
        //     phi.data[0][0],phi.data[1][0],phi.data[2][0],phi.data[3][0]);

        // Apply Gauss-Newton algorithm to update Phi values
        if (RET_OK == status && CALIB_TWO_ROT_AXES_MAX_ERROR < error) {
            MATRIX Jpinv            = matrix_allocate(4,observations_num);  // Pseudoinverse of the Jacobian
            MATRIX phi_correction   = matrix_allocate(4,1);                 // Parameters delta
            if (RET_OK == status) {
                status = matrix_pseudoinverse(Jacobian,&Jpinv);
            }
            if (RET_OK == status) {
                status = matrix_multiply(Jpinv, errorV, &phi_correction);
            }
            if (RET_OK == status) {
                status = matrix_substract(phi, phi_correction, &phi);
            }
            matrix_free(Jpinv);
            matrix_free(phi_correction); 
            // Set new vectors
            if (RET_OK == status) {
                vector3_from_spherical_coordinates_convert(phi.data[0][0], phi.data[1][0], sph_alt1, tempV1);
                vector3_from_spherical_coordinates_convert(phi.data[2][0], phi.data[3][0], sph_alt2, tempV2_from1);
                Quaternion_rotate(&q1_2, tempV2_from1, tempV2);
            }
        }
    }

    // Update database
    if (RET_OK == status) {
        double d_cost[]    = {Jacobian.data[0][0], Jacobian.data[0][1]};
        status = db_write(DB_CALIB_COST_DERIVATIVE, 0, d_cost);
    }
    if (RET_OK == status) {
        double d_cost[]    = {Jacobian.data[0][2], Jacobian.data[0][3]};
        status = db_write(DB_CALIB_COST_DERIVATIVE, 1, d_cost);
    }
    if (RET_OK == status) {
        double spherical[] = {phi.data[0][0],phi.data[1][0], sph_alt1};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 0, spherical);
    }
    if (RET_OK == status) {
        double spherical[] = {phi.data[2][0],phi.data[3][0], sph_alt2};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 1, spherical);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ERROR, 0, &bestError);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 0, rotationV1);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 1, rotationV2);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_OMEGA, 0, omegaR);
    }
    if (RET_OK == status) {
        double omegaR_norm;
        status = vector3_norm(omegaR, &omegaR_norm);
        if (RET_OK == status) {
            status = db_write(DB_CALIB_OMEGA_NORM, 0, &omegaR_norm);
        }
    }

    matrix_free(errorV);
    matrix_free(Jacobian);
    matrix_free(phi);

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


int observations_num = 0;
double observations_buffer[CALIB_TWO_ROT_AXES_WINDOW][7] = {{0.0}};

int scal_fitness_calculate(Chrom_Ptr chrom) {
    ERROR_CODE status = RET_OK;
    double theta1 =  chrom->gene[0];
    double rho1   =  chrom->gene[1];
    int sph_alt1  = (chrom->gene[2]>=0)?0:1;
    double theta2 =  chrom->gene[3];
    double rho2   =  chrom->gene[4];
    int sph_alt2  = (chrom->gene[5]>=0)?0:1;
    // Correct spherical choice genes
    chrom->gene[2] = (double)sph_alt1;
    chrom->gene[5] = (double)sph_alt2;
    // Get Vectors
    double v1[3], v2[3];
    vector3_from_spherical_coordinates_convert(theta1, rho1, sph_alt1, v1);
    vector3_from_spherical_coordinates_convert(theta2, rho2, sph_alt2, v2);

    double squared_error = 0.0;

    for (int i = 0; RET_OK == status && i < observations_num; i++) {
        // Retrieve ith observation 
        double     wi[3] = {            // The ith observation of angular velocity
            observations_buffer[i][0],observations_buffer[i][1],observations_buffer[i][2]};
        Quaternion qi    = {            // The ith observation of quaternion
            .w=observations_buffer[i][3], .v={observations_buffer[i][4],observations_buffer[i][5],observations_buffer[i][6]}}; 
        // Compute v2 from 1
        double v2_1[3];
        Quaternion_rotate(&qi, v2, v2_1);

        // Calculate error as fitness
        double error;
        double rotationVn[3];   // Normal vector to both rotation vectors

        status = vector3_cross(v1, v2_1, rotationVn);
        if (RET_OK == status) {
            status = vector3_normalize(rotationVn, rotationVn);
        }
        if (RET_OK == status) {
            status = vector3_dot(wi, rotationVn, &error);
        }
        if (RET_OK == status) {
            squared_error += error*error;
        }    
    }
    if (RET_OK == status) {
        chrom->fitness = squared_error;
    }
    else {
        chrom->fitness = 1e300;
    }
    
    return (RET_OK == status) ? OK : GA_ERROR;
}

static ERROR_CODE scal_observations_update() {
    ERROR_CODE status = RET_OK;
    double omegaR[3];

    int new_data_num = MIN(
        MIN(db_field_buffer_current_size_get(DB_IMU_GYROSCOPE,0),  db_field_buffer_current_size_get(DB_IMU_GYROSCOPE,1)),
        MIN(db_field_buffer_current_size_get(DB_IMU_QUATERNION,0), db_field_buffer_current_size_get(DB_IMU_QUATERNION,1))
    );
    for (int i = 0; RET_OK == status && i < new_data_num; i++) {
        double w1[3], w2[3];
        double q1[4], q2[4];
        double omega_norm;
        // Retrieve IMUs data
        db_field_buffer_from_tail_data_get(DB_IMU_GYROSCOPE, 0,i,w1);
        db_field_buffer_from_tail_data_get(DB_IMU_GYROSCOPE, 1,i,w2);
        db_field_buffer_from_tail_data_get(DB_IMU_QUATERNION,0,i,q1);
        db_field_buffer_from_tail_data_get(DB_IMU_QUATERNION,1,i,q2);
        // Compute angular velocity observation
        Quaternion q_1, q_2;
        quaternion_from_buffer_build(q1,&q_1);
        quaternion_from_buffer_build(q2,&q_2);
        status = arm_relative_angular_vel_compute(q_1, q_2, w1, w2, omegaR);
        if (RET_OK == status) {
            status = vector3_norm(omegaR,&omega_norm);
        }
        if (RET_OK == status /* && scal_min_velocity_norm < omega_norm */) {
            // Compute quaternion observation
            Quaternion q2_1 = arm_quaternion_between_two_get(q_2, q_1); // Quaternion to move from sensor 2 to 1
            // Write observations to database
            double observations[7] = {
                omegaR[0], omegaR[1] ,omegaR[2], q2_1.w, q2_1.v[0], q2_1.v[1], q2_1.v[2]
            };
            status = db_write(DB_CALIB_TWO_AXES_OBSERVATIONS,0,observations);
        }
    }
    if (RET_OK == status) {
        db_field_buffer_clear(DB_IMU_GYROSCOPE,0);
        db_field_buffer_clear(DB_IMU_GYROSCOPE,1);
        db_field_buffer_clear(DB_IMU_QUATERNION,0);
        db_field_buffer_clear(DB_IMU_QUATERNION,1);
    }
    return status;
}

ERROR_CODE scal_observations_get() {
    ERROR_CODE status = RET_OK;
    
    observations_num = db_field_buffer_current_size_get(DB_CALIB_TWO_AXES_OBSERVATIONS,0);
    for (int i = 0; RET_OK == status && i < observations_num; i++) {
        status = db_field_buffer_from_head_data_get(DB_CALIB_TWO_AXES_OBSERVATIONS,0,i,observations_buffer[i]);
    }

    return status;
}

GA_Info_Ptr ga_info = NULL;

ERROR_CODE cal_automatic_two_rotation_axes_ga_calibrate(
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV1[3],
    double rotationV2[3])
{
    ERROR_CODE status;
    char *config = "source/libGA100/GAconfig_full.conf";

    status = scal_observations_update();

    if (RET_OK == status) {
        status = scal_observations_get();
    }
    if (RET_OK == status) {
        if (NULL == ga_info) {
            /* Initialize the genetic algorithm */
            ga_info = GA_config(config, scal_fitness_calculate);
            remove(ga_info->rp_file);
        }
        /* Run the GA */
        GA_run(ga_info);
    }

    double theta1;
    double rho1;
    int sph_alt1;
    double theta2;
    double rho2;
    int sph_alt2;
    double bestError;
    if (RET_OK == status) {
        Chrom_Ptr chrom = ga_info->best;
        theta1 =  chrom->gene[0];
        rho1   =  chrom->gene[1];
        sph_alt1  = (chrom->gene[2]>=0)?0:1;
        theta2 =  chrom->gene[3];
        rho2   =  chrom->gene[4];
        sph_alt2  = (chrom->gene[5]>=0)?0:1;
        bestError = chrom->fitness;

        dbg_str("%s -> Best solution with error %f. %s in %d iterations out of %d",__FUNCTION__, 
            chrom->fitness, (ga_info->converged)?"Converged":"Did not converge", ga_info->iter, ga_info->max_iter);
        dbg_str("%s -> \t <%f,%f,%f,%f,%f,%f>",__FUNCTION__,chrom->gene[0],chrom->gene[1],chrom->gene[2],chrom->gene[3],chrom->gene[4],chrom->gene[5]);

        double tempV2_from1[3]; // Second rotation vector from the first sensor
        // Compute first rotation vector
        vector3_from_spherical_coordinates_convert(theta1, rho1, sph_alt1, rotationV1);
        // Compute second rotaton vector
        vector3_from_spherical_coordinates_convert(theta2, rho2, sph_alt2, tempV2_from1);
        Quaternion q2_1 = arm_quaternion_between_two_get(q_sensor1, q_sensor2); // Quaternion to move from sensor 2 to 1
        Quaternion q1_2;                                                        // Quaternion to move from sensor 1 to 2
        Quaternion_conjugate(&q2_1,&q1_2);                                      
        Quaternion_rotate(&q1_2,tempV2_from1, rotationV2);
    }

    // Update database
    if (RET_OK == status) {
        double spherical[] = {theta1,rho1,(double)sph_alt1};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 0, spherical);
    }
    if (RET_OK == status) {
        double spherical[] = {theta2,rho2,(double)sph_alt2};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 1, spherical);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ERROR, 0, &bestError);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 0, rotationV1);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 1, rotationV2);
    }
    return status;
}