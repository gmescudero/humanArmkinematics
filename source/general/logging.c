
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include "general.h"

#define CSV_FILE_NAME "hak_csv"
#define CSV_FILE_EXTENSION "csv"
#define CSV_FILE_DIRECTORY "data"
#define CSV_FILE_NAME_LENGTH (128)

#define LOG_FILE_NAME "hak"
#define LOG_FILE_DIRECTORY "log"
#define LOG_FILE_EXTENSION "log"
#define LOG_FILE_NAME_LENGTH (128)

#define LOG_FILE_TRACE_ERROR   "[ERROR  ] "
#define LOG_FILE_TRACE_WARNING "[WARNING] "
#define LOG_FILE_TRACE_INFO    "[INFO   ] "
#define LOG_FILE_TRACE_DEBUG   "[DEBUG  ] "

static void slog_file_name_build(char *directory, char* name_prefix, char *extension, char *file_name);
static ERROR_CODE sfile_create(char *file_name);
static void sstr_console_print(const char *trace_type, const char *text, va_list va);
static void sstr_file_print(const char *file_name, const char *trace_type, const char *text, va_list va);
static void scsv_default_headers_set(void);


TRACE_LEVEL trace_level = WARNING;
TRACE_LEVEL trace_file_level = DEBUG;


char csv_file_name[CSV_FILE_NAME_LENGTH] = {'\0'};
char log_file_name[LOG_FILE_NAME_LENGTH] = {'\0'};

ERROR_CODE log_file_initalize(){
    ERROR_CODE status;

    slog_file_name_build(LOG_FILE_DIRECTORY,LOG_FILE_NAME,LOG_FILE_EXTENSION, log_file_name);
    status = sfile_create(log_file_name);
    if (RET_OK == status) {
        slog_file_name_build(CSV_FILE_DIRECTORY,CSV_FILE_NAME,CSV_FILE_EXTENSION, csv_file_name);
        status = sfile_create(csv_file_name);
        if (RET_OK == status) {
            scsv_default_headers_set();
        }
    }
    return status;
}

ERROR_CODE trace_level_set(TRACE_LEVEL lvl, TRACE_LEVEL file_lvl) {
    /* Check arguments */
    if (NONE > lvl      || NUMBER_OF_LEVELS <= lvl     ) return RET_ARG_ERROR;
    if (NONE > file_lvl || NUMBER_OF_LEVELS <= file_lvl) return RET_ARG_ERROR;

    trace_level = lvl;
    trace_file_level = file_lvl;
    return RET_OK;
}

void dbg_str(const char *text, ...){
    va_list args;
    if (DEBUG <= trace_level) {
        va_start(args, text);
        sstr_console_print(LOG_FILE_TRACE_DEBUG, text, args);
        va_end(args);
    }
    if (DEBUG <= trace_file_level) {
        va_start(args, text);
        sstr_file_print(log_file_name, LOG_FILE_TRACE_DEBUG, text, args);
        va_end(args);
    }
}

void log_str(const char *text, ...){
    va_list args;
    if (INFO <= trace_level) {
        va_start(args, text);
        sstr_console_print(LOG_FILE_TRACE_INFO, text, args);
        va_end(args);
    }
    if (INFO <= trace_file_level) {
        va_start(args, text);
        sstr_file_print(log_file_name, LOG_FILE_TRACE_INFO, text, args);
        va_end(args);
    }
}

void wrn_str(const char *text, ...){
    va_list args;
    if (WARNING <= trace_level) {
        va_start(args, text);
        sstr_console_print(LOG_FILE_TRACE_WARNING, text, args);
        va_end(args);
    }
    if (WARNING <= trace_file_level) {
        va_start(args, text);
        sstr_file_print(log_file_name, LOG_FILE_TRACE_WARNING, text, args);
        va_end(args);
    }
}

void err_str(const char *text, ...){
    va_list args;
    if (ERROR <= trace_level) {
        va_start(args, text);
        sstr_console_print(LOG_FILE_TRACE_ERROR, text, args);
        va_end(args);
    }
    if (ERROR <= trace_file_level) {
        va_start(args, text);
        sstr_file_print(log_file_name, LOG_FILE_TRACE_ERROR, text, args);
        va_end(args);
    }
}

/**
 * @brief Set the default headers for the CSV file
 */
static void scsv_default_headers_set(void) {
    FILE *fd = NULL;
    if ('\0' != csv_file_name[0]) {
        fd = fopen(csv_file_name, "w");
        if (NULL != fd) {
            for (int i = 0; i < CSV_FILE_VALUES_NUMBER; i++) {
                fprintf(fd,"data%d,",i);
                if (i<CSV_FILE_VALUES_NUMBER-1) fprintf(fd,",");
            }
            fprintf(fd,"\n");
            fclose(fd);
        }
    }
}

void csv_headers_set(const char *headers[CSV_FILE_VALUES_NUMBER]) {
    FILE *fd = NULL;
    if ('\0' != csv_file_name[0]) {
        fd = fopen(csv_file_name, "w");
        if (NULL != fd) {
            for (int i = 0; i < CSV_FILE_VALUES_NUMBER; i++) {
                if (0 < strlen(headers[i])){
                    fprintf(fd,"%s",headers[i]);
                }
                else {
                    fprintf(fd,"data%d",i);
                }
                if (i<CSV_FILE_VALUES_NUMBER-1) fprintf(fd,",");
            }
            fprintf(fd,"\n");
            fclose(fd);
        }
    }
}

void csv_log(const double data[CSV_FILE_VALUES_NUMBER]) {
    FILE *fd = NULL;
    if ('\0' != csv_file_name[0]) {
        fd = fopen(csv_file_name, "a");
        if (NULL != fd) {
            for (int i = 0; i < CSV_FILE_VALUES_NUMBER; i++) {
                fprintf(fd,"%f",data[i]);
                if (i<CSV_FILE_VALUES_NUMBER-1) fprintf(fd,",");
            }
            fprintf(fd,"\n");
            fclose(fd);
        }
    }
}

/**
 * @brief Print a trace through console
 * 
 * @param trace_type (input) Prefix given by the trace type
 * @param text (input) The text to be formated
 * @param va (input) The formating args
 */
static void sstr_console_print(const char *trace_type, const char *text, va_list va) {
    printf("%s",trace_type);
    vprintf(text, va);
    printf("\n");
}

/**
 * @brief Print a trace to file
 * 
 * @param trace_type (input) Prefix given by the trace type
 * @param text (input) The text to be formated
 * @param va (input) The formating args
 */
static void sstr_file_print(const char file_name[], const char *trace_type, const char *text, va_list va) {
    FILE *fd = NULL;
    if ('\0' != file_name[0]) {
        fd = fopen(file_name, "a");
        if (NULL != fd) {
            fprintf(fd,"%s",trace_type);
            vfprintf(fd, text, va);
            fprintf(fd,"\n");
            fclose(fd);
        }
    }
}

/**
 * @brief Build the log file name
 * 
 * @param directory (input) The directory in where to place the file name
 * @param name_prefix (input) Prefix name of the file
 * @param extension (input) File extension
 * @param file_name (output) The obtained file name
 */
static void slog_file_name_build(char *directory, char* name_prefix, char *extension, char *file_name) {
    time_t raw_time;
    struct tm * time_info;
    time ( &raw_time );
    time_info = localtime ( &raw_time );

    sprintf(file_name, "%s/%s_%d%02d%02d_%02d%02d%02d.%s",directory, name_prefix, time_info->tm_year + 1900,
        time_info->tm_mon + 1, time_info->tm_mday, time_info->tm_hour, time_info->tm_min,
        time_info->tm_sec, extension);
}

/**
 * @brief Create a file with a given name
 * 
 * @param file_name (input) File name
 * @return ERROR_CODE: RET_OK on success
 */
static ERROR_CODE sfile_create(char *file_name) {
    FILE *fd = NULL;
    fd = fopen(file_name,"w+");
    if (NULL == fd) {
        err_str("Failed to create file: %s",file_name);
        return RET_ERROR;
    }
    fclose(fd);
    return RET_OK;
}