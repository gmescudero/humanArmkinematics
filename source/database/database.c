
#include "database.h"
#include "general.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef struct DB_FIELD_STRUCT {
    DB_FIELD_IDENTIFIER identifier;
    char                name[DB_FIELD_NAME_MAX_LENGTH];
    DB_FIELD_TYPE       type;
    unsigned int        multiplicity;
    sem_t               mutex;
    void                *data_ptr;
    int                 initialized;
} DB_FIELD;

#define DB_FIELD_SIZE(type,mul) (mul* ((DB_INTEGER==type)?sizeof(int):sizeof(double)) )
#define DB_FIELD_INIT(id,n,t,mult) {.identifier=id,.name=n,.type=t,.multiplicity=mult,.mutex={{0}},.data_ptr=NULL,.initialized=0}
/*
    DB_FIELD_IDENTIFIER identifier;
    char                name[DB_FIELD_NAME_MAX_LENGTH];
    DB_FIELD_TYPE       type;
    unsigned int        multiplicity;
    sem_t               mutex;
    void                *data_ptr;
    */
static DB_FIELD database[DB_NUMBER_OF_ENTRIES] = {
/*   field_id      , name         , type      , multiplicity, mutex, data_ptr */
    DB_FIELD_INIT(DB_TIMESTAMP  ,"TIMESTAMP"  ,DB_REAL,1),
    DB_FIELD_INIT(DB_CALIB_ERROR,"CALIB_ERROR",DB_REAL,1),
/* 
    {.identifier=DB_TIMESTAMP  , .name="TIMESTAMP"  , .type=DB_REAL, .multiplicity=1, .mutex={{0}}, .data_ptr=NULL},
    {.identifier=DB_CALIB_ERROR, .name="CALIB_ERROR", .type=DB_REAL, .multiplicity=1, .mutex={{0}}, .data_ptr=NULL}, */
};

ERROR_CODE db_initialize(void) {
    ERROR_CODE status = RET_OK;
    size_t field_type_size[DB_FIELD_TYPES_NUM];

    field_type_size[DB_INTEGER] = sizeof(int);
    field_type_size[DB_REAL   ] = sizeof(double);

    log_str("Initialzing database fields");
    for (int field_id = 0; RET_OK == status && field_id < DB_NUMBER_OF_ENTRIES; field_id++) {
        dbg_str("Initializing field %s (field identifier: %d)",database[field_id].name, field_id);
        // Initialize database entry mutex
        if (0 != sem_init(&(database[field_id].mutex),0,1)) {
            status = RET_ERROR;
            err_str("Falied to create semaphore for database entry");
        };
        // Allocate memory for the database entry
        if (RET_OK == status) {
            database[field_id].data_ptr = malloc(
                (size_t)database[field_id].multiplicity * field_type_size[database[field_id].type]);
            if (NULL == database[field_id].data_ptr) {
                status = RET_ERROR;
                err_str("Falied to allocate memory for database entry");
            }
        }
        // Initialize value to 0
        if (RET_OK == status) {
            if (DB_INTEGER == database[field_id].type) {
                for (int i = 0; i < database[field_id].multiplicity; i++) {
                    ((int*) database[field_id].data_ptr)[i] = 0;
                }
            }
            if (DB_REAL == database[field_id].type) {
                for (int i = 0; i < database[field_id].multiplicity; i++) {
                    ((double*) database[field_id].data_ptr)[i] = 0.0;
                }
            }
        }
        // Mark the field as initialized
        if (RET_OK == status) {
            database[field_id].initialized = 1;
        }
    }
    return status;
}

ERROR_CODE db_terminate(void) {
    ERROR_CODE status = RET_OK;

    for (int field_id = 0; field_id < DB_NUMBER_OF_ENTRIES; field_id++) {
        // Terminate database entry mutex
        if (0 != sem_destroy(&(database[field_id].mutex))) {
            status += RET_ERROR;
            err_str("Falied to destroy semaphore for database entry");
        };
        // Free allocated memory
        free(database[field_id].data_ptr);
        // Mark the field as not initialized
        database[field_id].initialized = 0;
    }
    return (RET_OK == status) ? RET_OK: RET_ERROR;
}

ERROR_CODE db_write(DB_FIELD_IDENTIFIER field, const void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    memcpy(database[field].data_ptr, data, DB_FIELD_SIZE(database[field].type, database[field].multiplicity));
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_read(DB_FIELD_IDENTIFIER field, void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    memcpy(data, database[field].data_ptr, DB_FIELD_SIZE(database[field].type, database[field].multiplicity));
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}