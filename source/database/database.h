
#ifndef __database_h__
#define __database_h__

#include "errors.h"
#include <semaphore.h>

#define DB_FIELD_NAME_MAX_LENGTH  (128)

typedef enum DB_FIELD_IDENTIFIER_ENUM {
    DB_TIMESTAMP = 0,
    DB_CALIB_ERROR,
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
 * @brief Write values to a database field
 * 
 * @param field (input) Field identifier
 * @param data (input) Data to write
 * @return ERROR_CODE 
 */
ERROR_CODE db_write(DB_FIELD_IDENTIFIER field, const void *data);
/**
 * @brief Read values from the database
 * 
 * @param field (input) Field identifier
 * @param data (output) Data to retrieve
 * @return ERROR_CODE 
 */
ERROR_CODE db_read(DB_FIELD_IDENTIFIER field, void *data);


#endif /* __database_h__ */