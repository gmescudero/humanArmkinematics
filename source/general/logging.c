
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <stdbool.h>
#include "general.h"

#define LOG_FILE_NAME "hak"
#define LOG_FILE_DIRECTORY "log"
#define LOG_FILE_EXTENSION "log"
#define LOG_FILE_NAME_LENGTH (128)

#define LOG_FILE_TRACE_ERROR   "[ERROR  ] "
#define LOG_FILE_TRACE_WARNING "[WARNING] "
#define LOG_FILE_TRACE_INFO    "[INFO   ] "
#define LOG_FILE_TRACE_DEBUG   "[DEBUG  ] "

static void slog_file_name_build(char *file_name);
static void sstr_console_print(const char *trace_type, const char *text, va_list va);
static void sstr_file_print(const char *file_name, const char *trace_type, const char *text, va_list va);


TRACE_LEVEL trace_level = WARNING;
TRACE_LEVEL trace_file_level = DEBUG;

char log_file_name[LOG_FILE_NAME_LENGTH] = {'\0'};
bool log_file_initialized = false;

ERROR_CODE log_file_initalize(){
    FILE *fd = NULL;
    slog_file_name_build(log_file_name);
    fd = fopen(log_file_name,"w+");
    if (NULL == fd) {
        err_str("Failed to create log file");
        return RET_ERROR;
    }
    fclose(fd);
    log_str(log_file_name);
    return RET_OK;
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

    va_start(args, text);
    if (DEBUG <= trace_level) {
        sstr_console_print(LOG_FILE_TRACE_DEBUG, text,args);
    }
    if (DEBUG <= trace_file_level) {
        sstr_file_print(log_file_name, LOG_FILE_TRACE_DEBUG, text, args);
    }
    va_end(args);
}

void log_str(const char *text, ...){
    va_list args;

    va_start(args, text);
    if (INFO <= trace_level) {
        sstr_console_print(LOG_FILE_TRACE_INFO, text,args);
    }
    if (INFO <= trace_file_level) {
        sstr_file_print(log_file_name, LOG_FILE_TRACE_INFO, text, args);
    }
    va_end(args);
}

void wrn_str(const char *text, ...){
    va_list args;

    va_start(args, text);
    if (WARNING <= trace_level) {
        sstr_console_print(LOG_FILE_TRACE_WARNING, text,args);
    }
    if (WARNING <= trace_file_level) {
        sstr_file_print(log_file_name, LOG_FILE_TRACE_WARNING, text, args);
    }
    va_end(args);
}

void err_str(const char *text, ...){
    va_list args;

    va_start(args, text);
    if (ERROR <= trace_level) {
        sstr_console_print(LOG_FILE_TRACE_ERROR, text,args);
    }
    if (ERROR <= trace_file_level) {
        sstr_file_print(log_file_name, LOG_FILE_TRACE_ERROR, text, args);
    }
    va_end(args);
}

static void sstr_console_print(const char *trace_type, const char *text, va_list va) {
    printf("%s",trace_type);
    vprintf(text, va);
    printf("\n");
}

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

static void slog_file_name_build(char *file_name){
    time_t raw_time;
    struct tm * time_info;
    time ( &raw_time );
    time_info = localtime ( &raw_time );

    sprintf(file_name, "%s/%s_%d%02d%02d_%02d%02d%02d.%s",LOG_FILE_DIRECTORY, LOG_FILE_NAME, time_info->tm_year + 1900,
        time_info->tm_mon + 1, time_info->tm_mday, time_info->tm_hour, time_info->tm_min,
        time_info->tm_sec, LOG_FILE_EXTENSION);
}