
#ifndef __database_h__
#define __database_h__

#include "errors.h"
#include <semaphore.h>

#define DB_FIELD_NAME_MAX_LENGTH  (128)
#define DB_FIELD_IDENTIFIER_INVALID (-1)

typedef enum DB_FIELD_IDENTIFIER_ENUM {
    DB_IMU_TIMESTAMP = 0,
    DB_IMU_ACCELEROMETER,
    DB_IMU_GYROSCOPE,
    DB_IMU_MAGNETOMETER,
    DB_IMU_QUATERNION,
    DB_IMU_LINEAR_ACCELERATION,
    DB_CALIB_ERROR,
    DB_CALIB_ROT_VECTOR,
    DB_CALIB_OMEGA,
    DB_CALIB_OMEGA_NORM,
    DB_CALIB_SPHERICAL_COORDS,
    DB_CALIB_SPHERICAL_ALTERNATIVE,
    DB_CALIB_COST_DERIVATIVE,
    DB_NUMBER_OF_ENTRIES
} DB_FIELD_IDENTIFIER;

typedef enum DB_FIELD_TYPE_ENUM {
    DB_INTEGER = 0,
    DB_REAL,
    DB_FIELD_TYPES_NUM
} DB_FIELD_TYPE;

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
 * @param data (input) Data to write
 * @return ERROR_CODE 
 */
ERROR_CODE db_write(DB_FIELD_IDENTIFIER field, const void *data);
/**
 * @brief Write a single value of a database field
 * 
 * @param field (input) Field identifier
 * @param index (input) Data index
 * @param data (input) Data to write
 * @return ERROR_CODE 
 */
ERROR_CODE db_index_write(DB_FIELD_IDENTIFIER field, int index, const void *data);
/**
 * @brief Read values from the database
 * 
 * @param field (input) Field identifier
 * @param data (output) Data to retrieve
 * @return ERROR_CODE 
 */
ERROR_CODE db_read(DB_FIELD_IDENTIFIER field, void *data);
/**
 * @brief Read a single value from a field of the database
 * 
 * @param field (input) Field identifier
 * @param index (input) Data index
 * @param data (output) Data to retrieve
 * @return ERROR_CODE 
 */
ERROR_CODE db_index_read(DB_FIELD_IDENTIFIER field, int index, void *data);

/**
 * @brief Set up a csv with a set of database fields to monitor
 * 
 * @param fields (input) The array of monitored fields
 * @param fields_num (input) The total number of arrays to monitor
 * @return ERROR_CODE 
 */
ERROR_CODE db_csv_setup(DB_FIELD_IDENTIFIER fields[], int fields_num);
/**
 * @brief Dump configured data to the csv
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE db_csv_dump(void);

#endif /* __database_h__ */