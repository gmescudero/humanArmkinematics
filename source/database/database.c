
#include "database.h"
#include "general.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

typedef struct DB_FIELD_STRUCT {
    DB_FIELD_IDENTIFIER identifier;
    char                name[DB_FIELD_NAME_MAX_LENGTH];
    DB_FIELD_TYPE       type;
    unsigned int        multiplicity;
    sem_t               mutex;
    void                *data_ptr;
    int                 initialized;
} DB_FIELD;

#define DB_FIELD_INIT(id,n,t,mult) {.identifier=id,.name=n,.type=t,.multiplicity=mult,.mutex={{0}},.data_ptr=NULL,.initialized=0}


static size_t sdb_field_size_get(const DB_FIELD_IDENTIFIER field);

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
    DB_FIELD_INIT(DB_TIMESTAMP, "TIMESTAMP", DB_REAL,1),
    /* IMU data */
    DB_FIELD_INIT(DB_IMU_ACCELEROMETER,      "IMU_ACCELEROMETER",       DB_REAL,3),
    DB_FIELD_INIT(DB_IMU_GYROSCOPE,          "IMU_GYROSCOPE",           DB_REAL,3),
    DB_FIELD_INIT(DB_IMU_MAGNETOMETER,       "IMU_MAGNETOMETER",        DB_REAL,3),
    DB_FIELD_INIT(DB_IMU_QUATERNION,         "IMU_QUATERNION",          DB_REAL,4),
    DB_FIELD_INIT(DB_IMU_LINEAR_ACCELERATION,"IMU_LINEAR_ACCELERATION", DB_REAL,3),
    /**/
    DB_FIELD_INIT(DB_CALIB_ERROR,"CALIB_ERROR",DB_REAL,1),
/* 
    {.identifier=DB_TIMESTAMP  , .name="TIMESTAMP"  , .type=DB_REAL, .multiplicity=1, .mutex={{0}}, .data_ptr=NULL},
    {.identifier=DB_CALIB_ERROR, .name="CALIB_ERROR", .type=DB_REAL, .multiplicity=1, .mutex={{0}}, .data_ptr=NULL}, */
};

static DB_FIELD_IDENTIFIER csv_logging_fields[DB_NUMBER_OF_ENTRIES] = {DB_FIELD_IDENTIFIER_INVALID};

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
    memcpy(database[field].data_ptr, data, sdb_field_size_get(field));
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_index_write(DB_FIELD_IDENTIFIER field, int index, const void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    if (0 > index || database[field].multiplicity <= index) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    if (DB_INTEGER == database[field].type) {
        ((int*)(database[field].data_ptr))[index] = *((const int*)data);
    }
    else /* DB_REAL */ { 
        ((double*)(database[field].data_ptr))[index] = *((const double*)data);
    }
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
    memcpy(data, database[field].data_ptr, sdb_field_size_get(field));
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_index_read(DB_FIELD_IDENTIFIER field, int index, void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    if (0 > index || database[field].multiplicity <= index) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    if (DB_INTEGER == database[field].type) {
        *(int*)data = ((int*)database[field].data_ptr)[index];
    }
    else /* DB_REAL */ { 
        *(double*)data = ((double*)database[field].data_ptr)[index];
    }
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_csv_setup(DB_FIELD_IDENTIFIER fields[], int fields_num) {
    char headers[CSV_FILE_VALUES_NUMBER][CSV_HEADER_MAX_LENGTH] = {{'\0'}};
    int csv_index = 0;
    DB_FIELD_IDENTIFIER currentField;
    // Check arguments
    if (NULL == fields) return RET_ARG_ERROR;
    if (0 >= fields_num || DB_NUMBER_OF_ENTRIES < fields_num) return RET_ARG_ERROR;

    // Set headers
    dbg_str("Stablish database names as csv headers to track");
    for (int fields_ind = 0; fields_ind < fields_num && csv_index < CSV_FILE_VALUES_NUMBER; fields_ind++) {
        currentField = fields[fields_ind];
        dbg_str("\t -> %s",database[currentField].name);
        if (1 == database[currentField].multiplicity) {
            strcpy(headers[csv_index],database[currentField].name);
            csv_index++;
        }
        else {
            for (int ind = 0; ind < database[currentField].multiplicity && csv_index < CSV_FILE_VALUES_NUMBER; ind++) {
                sprintf(headers[csv_index],"%s_%d",database[currentField].name,ind);
                csv_index++;
            }
        }
    }

    if (CSV_FILE_VALUES_NUMBER <= csv_index) {
        wrn_str("Maximum csv length reached");
    }

    // Set csv headers
    dbg_str("Set the csv headers");
    csv_headers_set(headers, csv_index);
    // Set csv logging fields
    dbg_str("Store the tracking fields into local database data");
    memcpy(csv_logging_fields, fields, fields_num*sizeof(DB_FIELD_IDENTIFIER));

    return RET_OK;
}

ERROR_CODE db_csv_dump(void) {
    ERROR_CODE status = RET_OK;
    double csv_buff[CSV_FILE_VALUES_NUMBER] = {0.0};
    int csv_idx = 0;
    DB_FIELD_IDENTIFIER current_field;

    for (int fld_idx = 0; csv_logging_fields[fld_idx] != DB_FIELD_IDENTIFIER_INVALID 
        && fld_idx < DB_NUMBER_OF_ENTRIES && RET_OK == status; fld_idx++) 
    {
        current_field = csv_logging_fields[fld_idx];
        for (int i = 0; i<database[current_field].multiplicity && RET_OK == status; i++) {
            status = db_index_read(current_field, i, &(csv_buff[csv_idx++]));
        }
    }
    csv_log(csv_buff);
    return status;
}

static size_t sdb_field_size_get(const DB_FIELD_IDENTIFIER field) {
    size_t type_size;
    if (DB_INTEGER == database[field].type) {
        type_size = sizeof(int);
    }
    else /* DB_REAL */ { 
        type_size = sizeof(double);
    }
    return (type_size * (size_t)database[field].multiplicity);
}