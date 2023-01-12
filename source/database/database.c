
#include "database.h"
#include "general.h"
#include "imu.h"
#include <semaphore.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define DB_INSTANCE_MAX_NUM (10)

#define DB_MULTIPLICITY_MAX_NUM (10)

#define DB_MAX_BUFFER_SIZE (1000)
#define DB_MAX_BUFFER_FIELDS (10)

typedef struct DB_BUFFER_STRUCT {
    int size;                           // Total size of the buffer
    int current_size;                   // Number of data instances set in the buffer
    int first_index;                    // The first data index with the oldest data in the internal buffer
    int last_index;                     // The last data index with the newest data in the internal buffer
    double **buff;                      // Internal data buffer
} DB_BUFFER;

typedef struct DB_FIELD_STRUCT {
    DB_FIELD_IDENTIFIER identifier;                     // Field identifier
    char                name[DB_FIELD_NAME_MAX_LENGTH]; // Field Name
    DB_FIELD_TYPE       type;                           // Field Type (real, int, etc.)
    unsigned int        instances;                      // Number of instances
    unsigned int        multiplicity;                   // Multiplicity of the field
    sem_t               mutex;                          // Multi thread protection mutex
    void                *data_ptr[DB_INSTANCE_MAX_NUM]; // Field data
    int                 initialized;                    // Field initialized flag
    DB_BUFFER           *buffer[DB_INSTANCE_MAX_NUM];   // Buffer of data
} DB_FIELD;

typedef struct DB_CSV_LOG_STRUCT {
    bool first;                                                      // First csv file line
    int fields_num;                                                 // Total number of fields
    int csv_coulmns;                                                // Total number of data columns taking multiplicity into account
    DB_FIELD_IDENTIFIER fields[CSV_FILE_VALUES_NUMBER];             // Field identifiers
    char headers[CSV_FILE_VALUES_NUMBER][CSV_HEADER_MAX_LENGTH];    // Headers strigs
    int instances[CSV_FILE_VALUES_NUMBER];                          // Each field instance included
    int indexes[CSV_FILE_VALUES_NUMBER];                            // Field data index for each data column 
} DB_CSV_LOG_STRUCT;

#define DB_FIELD_INIT(_identifier, _name, _type, _instances, _multiplicity) { \
    .identifier=_identifier,\
    .name=_name,\
    .type=_type,\
    .instances=_instances,\
    .multiplicity=_multiplicity,\
    .mutex={{0}},\
    .data_ptr={NULL},\
    .initialized=0,\
    .buffer={NULL} }


static size_t sdb_field_size_get(const DB_FIELD_IDENTIFIER field);
static void sdb_field_buffer_data_add(const DB_FIELD_IDENTIFIER field, int instance, const double *data);
static void sdb_buffer_free(DB_BUFFER *field_buff);

/*
    DB_FIELD_IDENTIFIER identifier;
    char                name[DB_FIELD_NAME_MAX_LENGTH];
    DB_FIELD_TYPE       type;
    unsigned int        multiplicity;
    sem_t               mutex;
    void                *data_ptr;
    */
static DB_FIELD database[DB_NUMBER_OF_ENTRIES] = {
/*   field_id      , name         , type      , instances,  multiplicity, mutex, data_ptr */
    
    /* IMU data */
    DB_FIELD_INIT(DB_IMU_NUMBER,                  "IMU_NUMBER",                 DB_INTEGER, 1, 1),
    DB_FIELD_INIT(DB_IMU_TIMESTAMP,               "IMU_TIMESTAMP",              DB_REAL,IMU_MAX_NUMBER,1),
    DB_FIELD_INIT(DB_IMU_ACCELEROMETER,           "IMU_ACCELEROMETER",          DB_REAL,IMU_MAX_NUMBER,3),
    DB_FIELD_INIT(DB_IMU_GYROSCOPE,               "IMU_GYROSCOPE",              DB_REAL,IMU_MAX_NUMBER,3),
    DB_FIELD_INIT(DB_IMU_MAGNETOMETER,            "IMU_MAGNETOMETER",           DB_REAL,IMU_MAX_NUMBER,3),
    DB_FIELD_INIT(DB_IMU_QUATERNION,              "IMU_QUATERNION",             DB_REAL,IMU_MAX_NUMBER,4),
    DB_FIELD_INIT(DB_IMU_LINEAR_ACCELERATION,     "IMU_LINEAR_ACCELERATION",    DB_REAL,IMU_MAX_NUMBER,3),
    DB_FIELD_INIT(DB_IMU_ANGULAR_VELOCITY,        "IMU_ANGULAR_VELOCITY",       DB_REAL,IMU_MAX_NUMBER,3),
    DB_FIELD_INIT(DB_IMU_NEW_DATA,                "IMU_NEW_DATA",               DB_INTEGER,IMU_MAX_NUMBER,1),
    /* Online rotation axis calibration data */
    DB_FIELD_INIT(DB_CALIB_ERROR,                   "CALIB_ERROR",                 DB_REAL,1,1),
    DB_FIELD_INIT(DB_CALIB_ROT_VECTOR,              "CALIB_ROT_VECTOR",            DB_REAL,2,3),
    DB_FIELD_INIT(DB_CALIB_OMEGA,                   "CALIB_OMEGA",                 DB_REAL,1,3),
    DB_FIELD_INIT(DB_CALIB_OMEGA_NORM,              "CALIB_OMEGA_NORM",            DB_REAL,1,1),
    DB_FIELD_INIT(DB_CALIB_SPHERICAL_COORDS,        "CALIB_SPHERICAL_COORDS",      DB_REAL,2,3),
    DB_FIELD_INIT(DB_CALIB_COST_DERIVATIVE,         "CALIB_COST_DERIVATIVE",       DB_REAL,2,2),
    DB_FIELD_INIT(DB_CALIB_TWO_AXES_OBSERVATIONS,   "CALIB_TWO_AXES_OBSERVATIONS", DB_REAL,1,3),
    DB_FIELD_INIT(DB_CALIB_ITERATIONS,              "CALIB_ITERATIONS",            DB_INTEGER,1,1),
    /* Arm positions and orientations */
    DB_FIELD_INIT(DB_ARM_SHOULDER_POSITION,    "ARM_SHOULDER_POSITION",    DB_REAL,1,3),
    DB_FIELD_INIT(DB_ARM_SHOULDER_ORIENTATION, "ARM_SHOULDER_ORIENTATION", DB_REAL,1,4),
    DB_FIELD_INIT(DB_ARM_ELBOW_POSITION,       "ARM_ELBOW_POSITION",       DB_REAL,1,3),
    DB_FIELD_INIT(DB_ARM_ELBOW_ORIENTATION,    "ARM_ELBOW_ORIENTATION",    DB_REAL,1,4),
    DB_FIELD_INIT(DB_ARM_WRIST_POSITION,       "ARM_WRIST_POSITION",       DB_REAL,1,3),
    DB_FIELD_INIT(DB_ARM_WRIST_ORIENTATION,    "ARM_WRIST_ORIENTATION",    DB_REAL,1,4),
    DB_FIELD_INIT(DB_ARM_ELBOW_ANGLES,         "ARM_ELBOW_ANGLES",         DB_REAL,1,3),    
    DB_FIELD_INIT(DB_ARM_ELBOW_QUATERNION,     "ARM_ELBOW_QUATERNION",     DB_REAL,1,4),    
    DB_FIELD_INIT(DB_ARM_SHOULDER_ANGLES,      "ARM_SHOULDER_ANGLES",      DB_REAL,1,3),    

/* 
    {.identifier=DB_IMU_TIMESTAMP  , .name="TIMESTAMP"  , .type=DB_REAL, .multiplicity=1, .mutex={{0}}, .data_ptr=NULL},
    {.identifier=DB_CALIB_ERROR, .name="CALIB_ERROR", .type=DB_REAL, .multiplicity=1, .mutex={{0}}, .data_ptr=NULL}, */
};


static DB_CSV_LOG_STRUCT csv_logging_fields = {
    .first = true,
    .fields_num = 0,
    .csv_coulmns = 0,
    .fields = {DB_FIELD_IDENTIFIER_INVALID},
    .headers = {{'\0'}},
    .indexes = {0},
    .instances = {0}
};

static int field_buffers_num = 0;
static DB_BUFFER field_buffers[DB_MAX_BUFFER_FIELDS];

ERROR_CODE db_initialize(void) {
    ERROR_CODE status = RET_OK;

    log_str("Initialzing database fields");
    for (int field_id = 0; RET_OK == status && field_id < DB_NUMBER_OF_ENTRIES; field_id++) {
        dbg_str("\t -> Initializing field %s (field identifier: %d)",database[field_id].name, field_id);
        // Initialize database entry mutex
        if (0 != sem_init(&(database[field_id].mutex),0,1)) {
            status = RET_ERROR;
            err_str("Falied to create semaphore for database entry");
        };
        // Allocate memory for the database entry
        for (int inst = 0; RET_OK == status && inst < database[field_id].instances; inst++) {
            database[field_id].data_ptr[inst] = malloc( sdb_field_size_get(field_id) );
            
            if (NULL == database[field_id].data_ptr[inst]) {
                status = RET_ERROR;
                err_str("Falied to allocate memory for database entry");
            }
            else {
                // Initialize value to 0
                if (DB_INTEGER == database[field_id].type) {
                    for (int i = 0; i < database[field_id].multiplicity; i++) {
                        ((int*) database[field_id].data_ptr[inst])[i] = 0;
                    }
                }
                if (DB_REAL == database[field_id].type) {
                    for (int i = 0; i < database[field_id].multiplicity; i++) {
                        ((double*) database[field_id].data_ptr[inst])[i] = 0.0;
                    }
                }
            }
        }

        // Mark the field as initialized
        if (RET_OK == status) {
            database[field_id].initialized = 1;
        }
    }

    log_str("Initialize field buffers");
    for(int i = 0; RET_OK == status && i < DB_MAX_BUFFER_FIELDS; i++) {
        field_buffers[i].size = 0;
        field_buffers[i].current_size = 0;
        field_buffers[i].first_index = 0;
        field_buffers[i].last_index = 0;
        field_buffers[i].buff = NULL;
    }

    return status;
}

ERROR_CODE db_terminate(void) {
    ERROR_CODE status = RET_OK;

    log_str("Removing all database resources");

    // Reset csv logging
    db_csv_reset();

    // Reset data buffers
    for (int buff_id = 0; buff_id < field_buffers_num; buff_id++) {
        if (NULL == field_buffers[buff_id].buff) {
            sdb_buffer_free(&field_buffers[buff_id]);
        }
    }
    field_buffers_num = 0;

    // Reset database
    for (int field_id = 0; field_id < DB_NUMBER_OF_ENTRIES; field_id++) {
        if (1 == database[field_id].initialized) {
            for (int inst = 0; inst < database[field_id].instances; inst++) {
                // Reset buffer pointer
                database[field_id].buffer[inst] = NULL;
                // Free allocated memory
                free(database[field_id].data_ptr[inst]);
            }
            // Terminate database entry mutex
            if (0 != sem_destroy(&(database[field_id].mutex))) {
                status += RET_ERROR;
                err_str("Falied to destroy semaphore for database entry");
            }
            // Mark the field as not initialized
            database[field_id].initialized = 0;
        }
    }

    return (RET_OK == status) ? RET_OK: RET_ERROR;
}

ERROR_CODE db_field_parameters_get(DB_FIELD_IDENTIFIER field, int *multiplicity, DB_FIELD_TYPE *type, int *init) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == multiplicity && NULL == type && NULL == init) return RET_ARG_ERROR;

    if (NULL != multiplicity) {
        *multiplicity = database[field].multiplicity;
    }
    if (NULL != type) {
        *type = database[field].type;
    }
    if (NULL != init) {
        *init = database[field].initialized;
    }
    return RET_OK;
}

ERROR_CODE db_write(DB_FIELD_IDENTIFIER field, int instance, const void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    memcpy(database[field].data_ptr[instance], data, sdb_field_size_get(field));

    // Buffer data if set up
    if (DB_REAL == database[field].type) {
        sdb_field_buffer_data_add(field, instance, data);
    }

    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_index_write(DB_FIELD_IDENTIFIER field, int instance, int index, const void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    if (0 > index || database[field].multiplicity <= index) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    if (DB_INTEGER == database[field].type) {
        ((int*) database[field].data_ptr[instance])[index] =  *((const int*)data);
    }
    else /* DB_REAL */ { 
        ((double*) database[field].data_ptr[instance])[index] =  *((const double*)data);
    }
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    // Buffer data if set up
    if (DB_REAL == database[field].type) {
        sdb_field_buffer_data_add(field, instance, (double*) database[field].data_ptr[instance]);
    }
    
    return RET_OK;
}

ERROR_CODE db_read(DB_FIELD_IDENTIFIER field, int instance, void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    memcpy(data, database[field].data_ptr[instance], sdb_field_size_get(field));

    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_index_read(DB_FIELD_IDENTIFIER field, int instance, int index, void *data) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;
    if (0 > index || database[field].multiplicity <= index) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    // Check availability
    if (1 != database[field].initialized) return RET_ERROR;

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    // Copy data
    if (DB_INTEGER == database[field].type) {
        *(int*)data = ((int*) database[field].data_ptr[instance])[index];
    }
    else /* DB_REAL */ { 
        *(double*)data = ((double*) database[field].data_ptr[instance])[index];
    }

    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return RET_OK;
}

ERROR_CODE db_csv_field_add(DB_FIELD_IDENTIFIER field, int instance) {
    int csv_index = csv_logging_fields.csv_coulmns;
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    // Check availability
    if (CSV_FILE_VALUES_NUMBER <= csv_index) return RET_ERROR;

    for (int ind = 0; ind < database[field].multiplicity && csv_index < CSV_FILE_VALUES_NUMBER; ind++) {
        sprintf(csv_logging_fields.headers[csv_index],"%s(%d)_%d",database[field].name, instance, ind);
        csv_logging_fields.fields[csv_index]    = field;
        csv_logging_fields.instances[csv_index] = instance;
        csv_logging_fields.indexes[csv_index]   = ind;
        csv_index++;
    }

    // Set csv logging structure
    csv_logging_fields.first = true;
    csv_logging_fields.csv_coulmns = csv_index;
    csv_logging_fields.fields_num++;

    return RET_OK;
}

void db_csv_reset() {
    csv_logging_fields.first       = true;
    csv_logging_fields.csv_coulmns = 0;
    csv_logging_fields.fields_num  = 0;
    for (int ind = 0; ind < CSV_FILE_VALUES_NUMBER; ind++) {
        csv_logging_fields.fields[ind]    = DB_FIELD_IDENTIFIER_INVALID;
        csv_logging_fields.instances[ind] = 0;
        csv_logging_fields.indexes[ind]   = 0;
    }
}

bool db_csv_field_logging_check(DB_FIELD_IDENTIFIER field, int instance) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return false;
    if (0 > instance || database[field].instances <= instance) return false;

    // Go over all logging fields looking for the one requested
    for (int i = 0; i < csv_logging_fields.fields_num; i++) {
        if (csv_logging_fields.fields[i] == field && csv_logging_fields.instances[i] == instance)
            return true;
    }
    return false;
}

ERROR_CODE db_csv_dump(void) {
    ERROR_CODE status = RET_OK;
    double csv_buff[CSV_FILE_VALUES_NUMBER] = {0.0};
    DB_FIELD_IDENTIFIER current_field;
    int instance;
    int index;

    if (1 == csv_logging_fields.first) {
        log_str("Start CSV logging with %d data columns",csv_logging_fields.csv_coulmns);
        csv_headers_set(csv_logging_fields.headers, csv_logging_fields.csv_coulmns);
        csv_logging_fields.first = 0;
    }

    // dbg_str("%s -> Retrieve Database configured fields",__FUNCTION__);
    for (int ind = 0; ind < csv_logging_fields.csv_coulmns && RET_OK == status; ind++) {
        current_field   = csv_logging_fields.fields[ind];
        instance        = csv_logging_fields.instances[ind];
        index           = csv_logging_fields.indexes[ind];
        if (DB_REAL == database[current_field].type) {
            status = db_index_read(current_field, instance, index, &(csv_buff[ind]));
        }
        else { // DB_INTEGER
            int buff;
            status = db_index_read(current_field, instance, index, &buff);
            csv_buff[ind] = (double)buff;
        }
        // dbg_str("\t -> Retrieve data from field %s(%d)_%d -> %f",
            // database[current_field].name, instance, index, csv_buff[ind]);
    }

    // dbg_str("%s -> Write CSV data row",__FUNCTION__);
    csv_log(csv_buff);
    return status;
}

ERROR_CODE db_field_print(DB_FIELD_IDENTIFIER field, int instance) {
    ERROR_CODE status = RET_OK;
    char field_str[256];
    char partial_str[16];
    double values[DB_MULTIPLICITY_MAX_NUM];
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;

    status = db_read(field, instance, values);
    if (RET_OK == status) {
        sprintf(field_str, "field %s (%d):", database[field].name, instance);
    }
    for (int i = 0; RET_OK == status && i < database[field].multiplicity; i++) {
        sprintf(partial_str, "\t%f ",values[i]);
        strcat(field_str, partial_str);
    }

    log_str("%s",field_str);

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

ERROR_CODE db_field_buffer_setup(const DB_FIELD_IDENTIFIER field, int instance, int size){
    ERROR_CODE status = RET_OK;
    dbg_str("%s -> Setting up buffer of size %d for field %s_%d",__FUNCTION__, size,database[field].name, instance);
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (0 > size || DB_MAX_BUFFER_SIZE < size) return RET_ARG_ERROR;
    if (DB_REAL != database[field].type) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    // Check availability
    if (DB_MAX_BUFFER_FIELDS <= field_buffers_num) return RET_ERROR;

    if (0 == size) {
        log_str("Removing buffer for field %s_%d",database[field].name,instance);
        if (NULL != database[field].buffer[instance]) {
            sdb_buffer_free(database[field].buffer[instance]);
            database[field].buffer[instance] = NULL;
        }
        return RET_OK;
    }

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }
    int index = field_buffers_num;
    
    field_buffers[index].buff = (double **) malloc( sizeof(double)*size );
    if (NULL == field_buffers[index].buff) {
        err_str("Failed to allocate buffer for database field %s",database[field].name);
        status = RET_ERROR;
    }
    for (int i = 0; RET_OK == status && i < size; i++) {
        field_buffers[index].buff[i] = (double *) malloc( sdb_field_size_get(field) );
        if (NULL == field_buffers[index].buff[i]) {
            err_str("Failed to allocate buffer for database field %s",database[field].name);
            status = RET_ERROR;
        }
    }
    if (RET_OK == status) {
        field_buffers[index].size        = size;
        database[field].buffer[instance] = &field_buffers[index];
        field_buffers_num++;   
    }
    
    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return status;
}

/**
 * @brief Add new data to a buffer 
 * 
 * @param field (input) Field identifier
 * @param data (input) Data to add
 */
static void sdb_field_buffer_data_add(const DB_FIELD_IDENTIFIER field, int instance, const double *data) {
    // dbg_str("%s -> Adding data to buffer for field %s_%d",__FUNCTION__,database[field].name, instance);
    // Check arguments
    if (NULL == database[field].buffer[instance]) return;
    if (NULL == data) return;

    DB_BUFFER *buffer = (database[field].buffer[instance]);

    if (buffer->current_size < buffer->size) {
        // Buffer not full
        buffer->last_index = buffer->current_size;
        buffer->current_size++;
    }
    else {
        // Overwrite oldest data
        int old_last_ind = buffer->last_index;
        buffer->last_index  = (old_last_ind+1) % buffer->size;
        buffer->first_index = (old_last_ind+2) % buffer->size;
    }

    for (int i = 0; i < database[field].multiplicity; i++) {
        buffer->buff[buffer->last_index][i] = data[i];
    }
}

ERROR_CODE db_field_buffer_from_tail_data_get(const DB_FIELD_IDENTIFIER field, int instance, int offset, double *data) {
    ERROR_CODE status = RET_OK;
    // dbg_str("%s -> Retrieve data at %d positions from tail for field %s_%d",__FUNCTION__, offset, database[field].name, instance);
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    if (NULL == database[field].buffer[instance]) return RET_ARG_ERROR;
    if (0 > offset || database[field].buffer[instance]->current_size-1 < offset) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;

    DB_BUFFER *buffer = (database[field].buffer[instance]);

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }

    int buffer_data_index = buffer->first_index + offset;
    // Adjust offset to circular buffer
    if (buffer->size <= buffer_data_index) buffer_data_index -= buffer->size;

    for (int i = 0; i < database[field].multiplicity; i++) {
        data[i] = buffer->buff[buffer_data_index][i];
    }

    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return status;
}

ERROR_CODE db_field_buffer_from_head_data_get(const DB_FIELD_IDENTIFIER field, int instance, int offset, double *data) {
    ERROR_CODE status = RET_OK;
    // dbg_str("%s -> Retrieve data at %d positions from head for field %s_%d",__FUNCTION__, offset, database[field].name, instance);
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    if (NULL == database[field].buffer[instance]) return RET_ARG_ERROR;
    if (0 > offset || database[field].buffer[instance]->current_size-1 < offset) return RET_ARG_ERROR;
    if (NULL == data) return RET_ARG_ERROR;

    DB_BUFFER *buffer = (database[field].buffer[instance]);

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }

    int buffer_data_index = buffer->last_index - offset;
    // Adjust offset to circular buffer
    if (0 > buffer_data_index) buffer_data_index += buffer->size;

    for (int i = 0; i < database[field].multiplicity; i++) {
        data[i] = buffer->buff[buffer_data_index][i];
    }

    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return status;
}

ERROR_CODE db_field_buffer_clear(const DB_FIELD_IDENTIFIER field, int instance) {
    ERROR_CODE status = RET_OK;
    dbg_str("%s -> Clear buffer for field %s_%d",__FUNCTION__, database[field].name, instance);
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return RET_ARG_ERROR;
    if (0 > instance || database[field].instances <= instance) return RET_ARG_ERROR;
    if (NULL == database[field].buffer[instance]) return RET_ARG_ERROR;

    DB_BUFFER *buffer = (database[field].buffer[instance]);

    if (0 != sem_wait(&(database[field].mutex))) {
        err_str("Could not retrieve semaphore");
        return RET_ERROR;
    }

    buffer->current_size = 0;
    buffer->first_index  = 0;
    buffer->last_index   = 0;

    if (0 != sem_post(&(database[field].mutex))) {
        err_str("Could not release semaphore");
        return RET_ERROR;
    }

    return status;
}

int db_field_buffer_current_size_get(const DB_FIELD_IDENTIFIER field, int instance) {
    // Check arguments
    if (0 > field || DB_NUMBER_OF_ENTRIES <= field) return -1;
    if (0 > instance || database[field].instances <= instance) return -1;
    if (NULL == database[field].buffer[instance]) return -1;

    return (database[field].buffer[instance]->current_size);
}

/**
 * @brief Free a field buffer
 * 
 * @param field_buff (input) field buffer to free
 */
static void sdb_buffer_free(DB_BUFFER *field_buff) {
    for (int i = 0; i < field_buff->size; i++) {
        free(field_buff->buff[i]);
    }
    free(field_buff->buff);
    field_buff->size = 0;
    field_buff->current_size = 0;
    field_buff->first_index = 0;
    field_buff->last_index = 0;
}
