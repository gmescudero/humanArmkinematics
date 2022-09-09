#include "calib.h"
#include "arm.h"
#include "vector3.h"
#include "ga.h"


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
    static double oldBestError = 0.0;
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

        // Adjust mutation and crossover rate
        if (bestError > oldBestError) {
            ga_info->mu_rate = 1.0;
            ga_info->x_rate = 0.1;
        }
        else {
            ga_info->mu_rate = 0.1;
            ga_info->x_rate = 0.5;
        }
        oldBestError = bestError;
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