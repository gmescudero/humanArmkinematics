/**
 * @file database.h
 * @author German Moreno Escudero
 * @brief This module implements a dynamically allocated reentrant table-like database
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __database_h__
#define __database_h__

#include <stdbool.h>
#include "errors.h"

#define DB_FIELD_NAME_MAX_LENGTH  (128)
#define DB_FIELD_IDENTIFIER_INVALID (-1)

#define DB_INSTANCE_MAX_NUM (10)

#define DB_MULTIPLICITY_MAX_NUM (10)

#define DB_MAX_BUFFER_SIZE (1000)
#define DB_MAX_BUFFER_FIELDS (10)

typedef enum DB_FIELD_IDENTIFIER_ENUM {
    DB_IMU_NUMBER = 0,
    DB_IMU_TIMESTAMP,
    DB_IMU_ACCELEROMETER,
    DB_IMU_GYROSCOPE,
    DB_IMU_MAGNETOMETER,
    DB_IMU_QUATERNION,
    DB_IMU_LINEAR_ACCELERATION,
    DB_IMU_ANGULAR_VELOCITY,
    DB_IMU_NEW_DATA,
    DB_CALIB_ERROR,
    DB_CALIB_ROT_VECTOR,
    DB_CALIB_OMEGA,
    DB_CALIB_OMEGA_NORM,
    DB_CALIB_SPHERICAL_COORDS,
    DB_CALIB_COST_DERIVATIVE,
    DB_CALIB_TWO_AXES_OBSERVATIONS,
    DB_CALIB_ITERATIONS,
    DB_ARM_SHOULDER_POSITION,
    DB_ARM_SHOULDER_ORIENTATION,
    DB_ARM_ELBOW_POSITION,
    DB_ARM_ELBOW_ORIENTATION,
    DB_ARM_WRIST_POSITION,
    DB_ARM_WRIST_ORIENTATION,
    DB_ARM_ELBOW_ANGLES,
    DB_ARM_ELBOW_QUATERNION,
    DB_ARM_SHOULDER_ANGLES,
    DB_NUMBER_OF_ENTRIES
} DB_FIELD_IDENTIFIER;

typedef enum DB_FIELD_TYPE_ENUM {
    DB_INTEGER = 0,
    DB_REAL,
    DB_FIELD_TYPES_NUM
} DB_FIELD_TYPE;

typedef struct DB_BUFFER_STATS_STRUCT {
    double max[DB_MULTIPLICITY_MAX_NUM];
    double min[DB_MULTIPLICITY_MAX_NUM];
    double mean[DB_MULTIPLICITY_MAX_NUM];
    double var[DB_MULTIPLICITY_MAX_NUM];
} DB_BUFFER_STATS;

/**
 * @brief Inititalize the database with default values
 * 
 * @return ERROR_CODE: RET_OK on success, RET_ERROR otherwise
 */
ERROR_CODE db_initialize(void);
/**
 * @brief Clean and destroy database resources
 * 
 * @return ERROR_CODE: RET_OK on success, RET_ERROR otherwise
 */
ERROR_CODE db_terminate(void);

/**
 * @brief Get parameters of a given field of the database
 * 
 * @param field (input) Field identifier
 * @param multiplicity (output) Multiplicity of the given field
 * @param type (output) type of the given field
 * @param init (output) initialized status of the given field
 * @return ERROR_CODE 
 */
ERROR_CODE db_field_parameters_get(DB_FIELD_IDENTIFIER field, int *multiplicity, DB_FIELD_TYPE *type, int *init);

/**
 * @brief Write values to a database field
 * 
 * @param field (input) Field identifier
 * @param instance (input) Instance of the field
 * @param data (input) Data to write
 * @return ERROR_CODE 
 */
ERROR_CODE db_write(DB_FIELD_IDENTIFIER field, int instance, const void *data);
/**
 * @brief Write a single value of a database field
 * 
 * @param field (input) Field identifier
 * @param instance (input) Instance of the field
 * @param index (input) Data index
 * @param data (input) Data to write
 * @return ERROR_CODE 
 */
ERROR_CODE db_index_write(DB_FIELD_IDENTIFIER field, int instance, int index, const void *data);
/**
 * @brief Read values from the database
 * 
 * @param field (input) Field identifier
 * @param instance (input) Instance of the field
 * @param data (output) Data to retrieve
 * @return ERROR_CODE 
 */
ERROR_CODE db_read(DB_FIELD_IDENTIFIER field, int c, void *data);
/**
 * @brief Read a single value from a field of the database
 * 
 * @param field (input) Field identifier
 * @param instance (input) Instance of the field
 * @param index (input) Data index
 * @param data (output) Data to retrieve
 * @return ERROR_CODE 
 */
ERROR_CODE db_index_read(DB_FIELD_IDENTIFIER field, int instance, int index, void *data);

/**
 * @brief Add a database field to the CSV file
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @return ERROR_CODE 
 */
ERROR_CODE db_csv_field_add(DB_FIELD_IDENTIFIER field, int instance);
/**
 * @brief Reset csv logging
 * 
 */
void db_csv_reset();
/**
 * @brief Check if a specific field is being logged into the CSV file
 * 
 * @param field (input) Field identifier.
 * @param instance (input) Field instance.
 * @return true if the field is being logged to the CSV file.
 * @return false otherwise.
 */
bool db_csv_field_logging_check(DB_FIELD_IDENTIFIER field, int instance);
/**
 * @brief Dump configured data to the csv
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE db_csv_dump(void);
/**
 * @brief Print a database field throug console
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @return ERROR_CODE 
 */
ERROR_CODE db_field_print(DB_FIELD_IDENTIFIER field, int instance);

/**
 * @brief Set up a buffer for a specific field
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @param size (input) Size of the buffer
 * @return ERROR_CODE 
 */
ERROR_CODE db_field_buffer_setup(const DB_FIELD_IDENTIFIER field, int instance, int size);
/**
 * @brief Retrieve data from the tail of the buffer (the oldest data)
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @param offset (input) The offset from the tail of the buffer towards the newest data
 * @param data (output) The data to be retrieved
 * @return ERROR_CODE 
 */
ERROR_CODE db_field_buffer_from_tail_data_get(const DB_FIELD_IDENTIFIER field, int instance, int offset, double *data);
/**
 * @brief Retrieve data from the head of the buffer (the newest data)
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @param offset (input) The offset from the head of the buffer towards the oldest data
 * @param data (output) The data to be retrieved
 * @return ERROR_CODE 
 */
ERROR_CODE db_field_buffer_from_head_data_get(const DB_FIELD_IDENTIFIER field, int instance, int offset, double *data);
/**
 * @brief Empty the data in a field buffer
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @return ERROR_CODE 
 */
ERROR_CODE db_field_buffer_clear(const DB_FIELD_IDENTIFIER field, int instance);
/**
 * @brief Retrieve the current number of data in the buffer
 * 
 * @param field (input) Field identifier
 * @param instance (input) Field instance
 * @return int 
 */
int db_field_buffer_current_size_get(const DB_FIELD_IDENTIFIER field, int instance);
/**
 * @brief 
 * 
 * @param field 
 * @param instance 
 * @return DB_BUFFER_STATS 
 */
DB_BUFFER_STATS db_field_buffer_stats_compute(const DB_FIELD_IDENTIFIER field, int instance);


#endif /* __database_h__ */