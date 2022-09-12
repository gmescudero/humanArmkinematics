#include "calib.h"
#include "arm.h"
#include "vector3.h"
#include "constants.h"
#include "matrix.h"
#include <math.h>

/**
 * @brief Error value calculated from spherical coordinates of a single measure
 * 
 * @note Code generated by sympy
 * 
 * @param omega_x (input) X component of the relative angular velocity
 * @param omega_y (input) Y component of the relative angular velocity
 * @param omega_z (input) Z component of the relative angular velocity
 * @param theta_1 (input) Theta angle of the spherical representation of the first rotation vector
 * @param rho_1 (input) Rho angle of the spherical representation of the first rotation vector
 * @param theta_2 (input) Theta angle of the spherical representation of the second rotation vector
 * @param rho_2 (input) Rho angle of the spherical representation of the second rotation vector
 * @return double: Error value
 */
static double scal_error_sph_11(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2) {
   double error_sph_11_result;
   error_sph_11_result = (omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) - omega_y*(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) - omega_z*sin(theta_1)*sin(theta_2)*sin(rho_1 - rho_2))/sqrt(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2));
   return error_sph_11_result;
}
/**
 * @brief Error derivatives of each parameter for a single measure
 * 
 * @note Code generated by sympy
 * 
 * @param omega_x (input) X component of the relative angular velocity
 * @param omega_y (input) Y component of the relative angular velocity
 * @param omega_z (input) Z component of the relative angular velocity
 * @param theta_1 (input) Theta angle of the spherical representation of the first rotation vector
 * @param rho_1 (input) Rho angle of the spherical representation of the first rotation vector
 * @param theta_2 (input) Theta angle of the spherical representation of the second rotation vector
 * @param rho_2 (input) Rho angle of the spherical representation of the second rotation vector
 * @param out_xxxxxxxxxxxxxxxxxxx (output) Derivatives of the error according to each angle: de/dth1, de/drh1, de/dth2, de/drh2
 */
static void scal_error_derivatives_11(double omega_x, double omega_y, double omega_z, double rho_1, double rho_2, double theta_1, double theta_2, double *out_5514908373459938152) {
   out_5514908373459938152[0] = ((1.0/32.0)*(-omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*sin(theta_1)*sin(theta_2)*sin(rho_1 - rho_2))*(4*sin(2*theta_1) + 6*sin(2*theta_1 - 2*theta_2) + 6*sin(2*theta_1 + 2*theta_2) - 2*sin(-2*rho_1 + 2*rho_2 + 2*theta_1) - 2*sin(2*rho_1 - 2*rho_2 + 2*theta_1) + sin(-2*rho_1 + 2*rho_2 + 2*theta_1 + 2*theta_2) - 4*sin(-rho_1 + rho_2 + 2*theta_1 + 2*theta_2) - 4*sin(rho_1 - rho_2 - 2*theta_1 + 2*theta_2) + 4*sin(rho_1 - rho_2 + 2*theta_1 - 2*theta_2) - 4*sin(rho_1 - rho_2 + 2*theta_1 + 2*theta_2) - sin(2*rho_1 - 2*rho_2 - 2*theta_1 + 2*theta_2) + sin(2*rho_1 - 2*rho_2 + 2*theta_1 - 2*theta_2) + sin(2*rho_1 - 2*rho_2 + 2*theta_1 + 2*theta_2)) + (omega_x*(sin(rho_1)*cos(theta_1)*cos(theta_2) + sin(rho_2)*sin(theta_1)*sin(theta_2)) - omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_2) + cos(rho_1)*cos(theta_1)*cos(theta_2)) - omega_z*sin(theta_2)*sin(rho_1 - rho_2)*cos(theta_1))*(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2)))/pow(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2), 3.0/2.0);
   out_5514908373459938152[1] = ((sin(theta_1)*sin(theta_2)*cos(rho_1 - rho_2) + cos(theta_1)*cos(theta_2))*(-omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*sin(theta_1)*sin(theta_2)*sin(rho_1 - rho_2))*sin(theta_2)*sin(rho_1 - rho_2) + (omega_x*cos(rho_1)*cos(theta_2) + omega_y*sin(rho_1)*cos(theta_2) - omega_z*sin(theta_2)*cos(rho_1 - rho_2))*(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2)))*sin(theta_1)/pow(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2), 3.0/2.0);
   out_5514908373459938152[2] = ((-omega_x*(sin(rho_1)*sin(theta_1)*sin(theta_2) + sin(rho_2)*cos(theta_1)*cos(theta_2)) + omega_y*(sin(theta_1)*sin(theta_2)*cos(rho_1) + cos(rho_2)*cos(theta_1)*cos(theta_2)) - omega_z*sin(theta_1)*sin(rho_1 - rho_2)*cos(theta_2))*(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2)) - 1.0/32.0*(-omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*sin(theta_1)*sin(theta_2)*sin(rho_1 - rho_2))*(-4*sin(2*theta_2) + 6*sin(2*theta_1 - 2*theta_2) - 6*sin(2*theta_1 + 2*theta_2) + 2*sin(-2*rho_1 + 2*rho_2 + 2*theta_2) + 2*sin(2*rho_1 - 2*rho_2 + 2*theta_2) - sin(-2*rho_1 + 2*rho_2 + 2*theta_1 + 2*theta_2) + 4*sin(-rho_1 + rho_2 + 2*theta_1 + 2*theta_2) - 4*sin(rho_1 - rho_2 - 2*theta_1 + 2*theta_2) + 4*sin(rho_1 - rho_2 + 2*theta_1 - 2*theta_2) + 4*sin(rho_1 - rho_2 + 2*theta_1 + 2*theta_2) - sin(2*rho_1 - 2*rho_2 - 2*theta_1 + 2*theta_2) + sin(2*rho_1 - 2*rho_2 + 2*theta_1 - 2*theta_2) - sin(2*rho_1 - 2*rho_2 + 2*theta_1 + 2*theta_2)))/pow(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2), 3.0/2.0);
   out_5514908373459938152[3] = (-(sin(theta_1)*sin(theta_2)*cos(rho_1 - rho_2) + cos(theta_1)*cos(theta_2))*(-omega_x*(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1)) + omega_y*(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1)) + omega_z*sin(theta_1)*sin(theta_2)*sin(rho_1 - rho_2))*sin(theta_1)*sin(rho_1 - rho_2) + (-omega_x*cos(rho_2)*cos(theta_1) - omega_y*sin(rho_2)*cos(theta_1) + omega_z*sin(theta_1)*cos(rho_1 - rho_2))*(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2)))*sin(theta_2)/pow(pow(sin(rho_1)*sin(theta_1)*cos(theta_2) - sin(rho_2)*sin(theta_2)*cos(theta_1), 2) + pow(sin(theta_1)*cos(rho_1)*cos(theta_2) - sin(theta_2)*cos(rho_2)*cos(theta_1), 2) + pow(sin(theta_1), 2)*pow(sin(theta_2), 2)*pow(sin(rho_1 - rho_2), 2), 3.0/2.0);
}
/**
 * @brief Conversion from spherical coordinates to rotation vectors
 * 
 * @note Code generated by sympy
 * 
 * @param theta_1 (input) Theta angle of the spherical representation of the rotation vector
 * @param rho_1 (input) Rho angle of the spherical representation of the rotation vector
 * @param out_xxxxxxxxxxxxxxxxxxx (output) Rotation vector obtained
 */
static void scal_spherical_2_vec_1(double rho_1, double theta_1, double *out_8227029826163990492) {
   out_8227029826163990492[0] = sin(theta_1)*cos(rho_1);
   out_8227029826163990492[1] = sin(rho_1)*sin(theta_1);
   out_8227029826163990492[2] = cos(theta_1);
}
/**
 * @brief Conversion from cartesian coordinates vector to spherical coordinates
 * 
 * @note Code generated by sympy
 * 
 * @param x (input) X component of the cartesian vector
 * @param y (input) Y component of the cartesian vector
 * @param z (input) Z component of the cartesian vector
 * @param out_xxxxxxxxxxxxxxxxxxx (output) spherical coordinates: theta, rho
 */
void scal_spherical_1(double x, double y, double z, double *out_346291481195618932) {
   out_346291481195618932[0] = atan2(sqrt(pow(x, 2) + pow(y, 2)), z);
   out_346291481195618932[1] = atan2(y, x);
}


#define CAL_PARAMETERS_NUM (4)

typedef struct CAL_DATA_STRUCT {
    bool initialized;
    MATRIX phi;
    int sph_alt1;
    int sph_alt2;
    double error;

} CAL_DATA;

static CAL_DATA scal_data = {
    .initialized = false,
    .sph_alt1 = 0,
    .sph_alt2 = 0,
    .error = 1e300
};


/**
 * @brief Initialize two rotation axes calibration resources. It requires to set up buffers for database field 
 *  including gyroscope and quaternion measures as well as relative angular velocity.
 * 
 * @param imu_data_buff_size (input) Size of the imu measures buffer
 * @param obs_data_buff_size (input) Size of the observations buffer
 * @return ERROR_CODE 
 */
ERROR_CODE cal_two_rot_axes_calib_initialize(int imu_data_buff_size, int obs_data_buff_size) {
    ERROR_CODE status = RET_OK;

    // Initialize IMU measures buffer
    if (RET_OK == status) status = db_field_buffer_setup(DB_IMU_GYROSCOPE,0,imu_data_buff_size);
    if (RET_OK == status) status = db_field_buffer_setup(DB_IMU_GYROSCOPE,1,imu_data_buff_size);
    if (RET_OK == status) status = db_field_buffer_setup(DB_IMU_QUATERNION,0,imu_data_buff_size);
    if (RET_OK == status) status = db_field_buffer_setup(DB_IMU_QUATERNION,1,imu_data_buff_size);
    // Initialize observations buffer
    if (RET_OK == status) status = db_field_buffer_setup(DB_CALIB_OMEGA,0,obs_data_buff_size);
    // Set up parameters vector
    scal_data.phi = matrix_allocate(CAL_PARAMETERS_NUM,1);
    // Set initial error
    scal_data.error = 1e300;

    if (RET_OK == status) {
        log_str("Initialized calibration package");
        scal_data.initialized = true;
    }
    else {
        err_str("Failed to initialize calibration package");
    }

    return status;
}

/**
 * @brief Terminate the calibration package and destroy all used resources
 */
void cal_two_rot_axes_calib_terminate() {
    matrix_free(scal_data.phi);
    scal_data.initialized = false;
}

/**
 * @brief Update database observations.
 * 
 * @param omega1_from1 (input) Angular velocity of the first IMU sensor in IMU ref. frame
 * @param omega2_from2 (input) Angular velocity of the second IMU sensor in IMU ref. frame
 * @param q_sensor1 (input) Orientation quaternnion of the first IMU sensor
 * @param q_sensor2 (input) Orientation quaternnion of the second IMU sensor
 * @return ERROR_CODE 
 */
ERROR_CODE cal_two_rot_axes_calib_observations_update(double omega1_from1[3], double omega2_from2[3], Quaternion q_sensor1, Quaternion q_sensor2) {
    ERROR_CODE status = RET_OK;
    Quaternion qR;
    double omega2_from1[3];
    double omegaR[3];

    // Check arguments
    if (NULL == omega1_from1)       return RET_ARG_ERROR;
    if (NULL == omega2_from2)       return RET_ARG_ERROR;

    // Compute relative quaternion
    qR = arm_quaternion_between_two_get(q_sensor1,q_sensor2);
    // Compute relative angular velocity
    Quaternion_rotate(&qR, omega2_from2, omega2_from1);
    status = vector3_substract(omega2_from1, omega1_from1, omegaR);

    // Update database
    double omegaR_norm;
    if (RET_OK == status) status = vector3_norm(omegaR, &omegaR_norm);
    if (RET_OK == status) status = db_write(DB_CALIB_OMEGA,0,omegaR);
    if (RET_OK == status) status = db_write(DB_CALIB_OMEGA_NORM, 0, &omegaR_norm);
    
    return status;
}

/**
 * @brief Update database observations from database values.
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE cal_two_rot_axes_calib_observations_from_database_update() {
    ERROR_CODE status = RET_OK;
    double omega1_from1[3];
    double omega2_from2[3];
    double q_buff[4];
    Quaternion q_sensor1;
    Quaternion q_sensor2;

    int new_data_num = MIN(
        MIN(db_field_buffer_current_size_get(DB_IMU_GYROSCOPE,0),  db_field_buffer_current_size_get(DB_IMU_GYROSCOPE,1)),
        MIN(db_field_buffer_current_size_get(DB_IMU_QUATERNION,0), db_field_buffer_current_size_get(DB_IMU_QUATERNION,1))
    );
    
    for (int i = 0; RET_OK == status && i < new_data_num; i++) {
        // Retrieve IMUs data
        if (RET_OK == status) status = db_field_buffer_from_tail_data_get(DB_IMU_GYROSCOPE, 0,i,omega1_from1);
        if (RET_OK == status) status = db_field_buffer_from_tail_data_get(DB_IMU_GYROSCOPE, 1,i,omega2_from2);
        if (RET_OK == status) status = db_field_buffer_from_tail_data_get(DB_IMU_QUATERNION,0,i,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q_sensor1);
        if (RET_OK == status) status = db_field_buffer_from_tail_data_get(DB_IMU_QUATERNION,1,i,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q_sensor2);
        // Compute observations and log them into the database
        if (RET_OK == status) status = cal_two_rot_axes_calib_observations_update(omega1_from1, omega2_from2, q_sensor1, q_sensor2);
    }
    if (RET_OK == status) {
        // Clear buffers
        if (RET_OK == status) status = db_field_buffer_clear(DB_IMU_GYROSCOPE,0);
        if (RET_OK == status) status = db_field_buffer_clear(DB_IMU_GYROSCOPE,1);
        if (RET_OK == status) status = db_field_buffer_clear(DB_IMU_QUATERNION,0);
        if (RET_OK == status) status = db_field_buffer_clear(DB_IMU_QUATERNION,1);
    }
    return status;
}


/**
 * @brief Make a single iteration of the Gauss-Newton algorithm
 * 
 * @param parameter_vector (output) New achieved parameters vector
 * @param error (output) Mean error of the new solution
 * @return ERROR_CODE 
 */
ERROR_CODE cal_two_rot_axes_calib_gauss_newton_iteration(double parameter_vector[CAL_PARAMETERS_NUM], double *error) {
    ERROR_CODE status = RET_OK;

    int observations_num = db_field_buffer_current_size_get(DB_CALIB_OMEGA,0);  // Number of managed observations
    if (0 >= observations_num) {
        wrn_str("No available observations for two axes calibration");
        return RET_NO_EXEC;
    }

    MATRIX errorV   = matrix_allocate(observations_num,1);  // Vector of residuals
    MATRIX Jacobian = matrix_allocate(observations_num,CAL_PARAMETERS_NUM);  // Jacobian matrix
    double obs_err;
    
    // Compute residuals vector and Jacobian
    double omegaR[3];
    for (int i = 0; RET_OK == status && i < observations_num; i++) {
        // Retrieve omega R observation
        if (RET_OK == status) status = db_field_buffer_from_tail_data_get(DB_CALIB_OMEGA,0, i, omegaR);
        // Compute the error value
        if (RET_OK == status) obs_err = scal_error_sph_11(
            omegaR[0],omegaR[1],omegaR[2], 
            scal_data.phi.data[0][0], scal_data.phi.data[1][0], scal_data.phi.data[2][0], scal_data.phi.data[3][0]);
        // Build residuals vector
        if (RET_OK == status) errorV.data[i][0] = obs_err;
        // Compute error derivatives and build Jacobian matrix
        if (RET_OK == status) scal_error_derivatives_11(
            omegaR[0],omegaR[1],omegaR[2], 
            scal_data.phi.data[0][0], scal_data.phi.data[1][0], scal_data.phi.data[2][0], scal_data.phi.data[3][0],
            Jacobian.data[i]);
    }

    // Apply Gauss-Newton algorithm to update Phi values
    if (RET_OK == status) {
        MATRIX Jpinv            = matrix_allocate(CAL_PARAMETERS_NUM,observations_num);  // Pseudoinverse of the Jacobian
        MATRIX phi_correction   = matrix_allocate(CAL_PARAMETERS_NUM,1);                 // Parameters delta

        if (RET_OK == status) status = matrix_pseudoinverse(Jacobian,&Jpinv);
        if (RET_OK == status) status = matrix_multiply(Jpinv, errorV, &phi_correction);
        if (RET_OK == status) status = matrix_substract(scal_data.phi, phi_correction, &scal_data.phi);

        matrix_free(Jpinv);
        matrix_free(phi_correction); 
    }
    matrix_free(errorV);
    matrix_free(Jacobian);

    // Set output parameters vector
    if (RET_OK == status) {
        parameter_vector[0] = scal_data.phi.data[0][0];
        parameter_vector[1] = scal_data.phi.data[1][0];
        parameter_vector[2] = scal_data.phi.data[2][0];
        parameter_vector[3] = scal_data.phi.data[3][0];
    }

    // Compute total Root Mean Square value after correction
    double sqr_err = 0.0;
    for (int i = 0; RET_OK == status && i < observations_num; i++) {
        double omegaR[3];
        // Retrieve omega R observation
        if (RET_OK == status) status = db_field_buffer_from_tail_data_get(DB_CALIB_OMEGA,0, i, omegaR);
        // Compute the error value
        if (RET_OK == status) obs_err = scal_error_sph_11(
            omegaR[0],omegaR[1],omegaR[2], 
            scal_data.phi.data[0][0], scal_data.phi.data[1][0], scal_data.phi.data[2][0], scal_data.phi.data[3][0]);
        // Acumulate squared error values
        if (RET_OK == status) sqr_err += obs_err*obs_err;
    }
    *error = sqrt(sqr_err/observations_num);

    return status;
}

/**
 * @brief Compute two rotation axes automatic calibration from arbitrary motion
 * 
 * @param rotationV1 (output) First rotation vector 
 * @param rotationV2 (output) Second rotation vector 
 * @return ERROR_CODE 
 */
ERROR_CODE cal_two_rot_axes_calib_compute(double rotationV1[3], double rotationV2[3]) {
    ERROR_CODE status = RET_OK;

    double error = scal_data.error;                     // Error value
    double oldError = error*2;                          // Last iteration error value
    double spherical_coords[CAL_PARAMETERS_NUM];        // Spherical coords of both vectors

    int iterations = CALIB_TWO_ROT_AXES_MAX_ITERATIONS; // Iterations of the algorithm

    double sph[2];
    scal_spherical_1(rotationV1[0],rotationV1[1],rotationV1[2],sph);
    scal_data.phi.data[0][0] = sph[0];
    scal_data.phi.data[1][0] = sph[1];
    scal_data.sph_alt1 = 0;
    scal_spherical_1(rotationV2[0],rotationV2[1],rotationV2[2],sph);
    scal_data.phi.data[2][0] = sph[0];
    scal_data.phi.data[3][0] = sph[1];
    scal_data.sph_alt2 = 0;

    while (RET_OK == status && 0 < iterations--  && fabs(oldError-error) > CALIB_TWO_ROT_AXES_MAX_ERROR ) {
        // Previous iteration error value
        oldError = error;
        // Execute one iteration of the gauss newton algorithm
        status = cal_two_rot_axes_calib_gauss_newton_iteration(spherical_coords, &error);
        // Use only the best set of rotation axes
        if (RET_OK == status && oldError + EPSI > error) {
            scal_spherical_2_vec_1(spherical_coords[0], spherical_coords[1], rotationV1);
            scal_spherical_2_vec_1(spherical_coords[2], spherical_coords[3], rotationV2);
            scal_data.error = error;
        }
        dbg_str("%s -> [it: %d] Current calib error: %f (best error: %f)",__FUNCTION__, 
            CALIB_TWO_ROT_AXES_MAX_ITERATIONS-iterations, error, scal_data.error);
    }

    // Update database
    if (RET_OK == status) {
        double spherical[] = {spherical_coords[0],spherical_coords[1], scal_data.sph_alt1};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 0, spherical);
    }
    if (RET_OK == status) {
        double spherical[] = {spherical_coords[2],spherical_coords[3], scal_data.sph_alt2};
        status = db_write(DB_CALIB_SPHERICAL_COORDS, 1, spherical);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ERROR, 0, &scal_data.error);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 0, rotationV1);
    }
    if (RET_OK == status) {
        status = db_write(DB_CALIB_ROT_VECTOR, 1, rotationV2);
    }

    return status;
}